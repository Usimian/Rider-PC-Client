"""PPO + CAPS spatial action-smoothness regularization (Mysore et al. 2021, arXiv 2012.06644).

Adds   loss += caps_lambda * mean_i || mu(s_i) - mu(s_i + N(0, sigma)) ||^2
to the PPO policy loss so NEARBY STATES MAP TO NEARBY ACTIONS. This directly kills the
threshold-switching bang-bang (action slamming -1<->+1 across a tiny state change) that our
reward-based rate penalty couldn't fix and that wrecks sim-to-real transfer. Temporal smoothness
is left to the env's reward rate penalty. Everything else is stock SB3 2.8 PPO.train().
"""
import numpy as np
import torch as th
from torch.nn import functional as F
from gymnasium import spaces
from stable_baselines3 import PPO
from stable_baselines3.common.utils import explained_variance


class CAPS_PPO(PPO):
    def __init__(self, *args, caps_lambda: float = 2.0, caps_sigma: float = 0.05, **kwargs):
        super().__init__(*args, **kwargs)
        self.caps_lambda = caps_lambda      # spatial-smoothness weight (ablate: too high -> can't balance)
        self.caps_sigma = caps_sigma        # state-perturbation std in (normalized) obs space
        # ANCHOR (fine-tuning, 2026-10-20): keep the policy's mean action on a fixed set of states close to a
        # reference policy's (loss += anchor_lambda * mean (mu(s) - mu_ref(s))^2). Set by train.py --anchor_lambda.
        self.anchor_lambda = 0.0
        self.anchor_obs = None              # (N, obs_dim) normalized observations
        self.anchor_mu = None               # (N, act_dim) the reference policy's mean actions on them
        self.anchor_batch = 1024
        # LEAD (2026-10-20): on recorded floor states, pull the mean action towards a target recorded slightly LATER
        # (loss += lead_lambda * mean (mu(s) - target)^2) -> the policy reacts earlier there. Set by train.py --lead_lambda.
        self.lead_lambda = 0.0
        self.lead_obs = None                # (N, obs_dim) normalized observations
        self.lead_target = None             # (N, act_dim) target actions
        self.lead_batch = 512

    def _excluded_save_params(self):
        return super()._excluded_save_params() + ["anchor_obs", "anchor_mu", "lead_obs", "lead_target"]  # data

    def train(self) -> None:
        self.policy.set_training_mode(True)
        self._update_learning_rate(self.policy.optimizer)
        clip_range = self.clip_range(self._current_progress_remaining)
        if self.clip_range_vf is not None:
            clip_range_vf = self.clip_range_vf(self._current_progress_remaining)

        entropy_losses = []
        pg_losses, value_losses = [], []
        clip_fractions = []
        caps_losses = []
        anchor_losses = []
        lead_losses = []
        continue_training = True

        for epoch in range(self.n_epochs):
            approx_kl_divs = []
            for rollout_data in self.rollout_buffer.get(self.batch_size):
                actions = rollout_data.actions
                if isinstance(self.action_space, spaces.Discrete):
                    actions = rollout_data.actions.long().flatten()

                values, log_prob, entropy = self.policy.evaluate_actions(rollout_data.observations, actions)
                values = values.flatten()
                advantages = rollout_data.advantages
                if self.normalize_advantage and len(advantages) > 1:
                    advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)

                ratio = th.exp(log_prob - rollout_data.old_log_prob)
                policy_loss_1 = advantages * ratio
                policy_loss_2 = advantages * th.clamp(ratio, 1 - clip_range, 1 + clip_range)
                policy_loss = -th.min(policy_loss_1, policy_loss_2).mean()

                pg_losses.append(policy_loss.item())
                clip_fraction = th.mean((th.abs(ratio - 1) > clip_range).float()).item()
                clip_fractions.append(clip_fraction)

                if self.clip_range_vf is None:
                    values_pred = values
                else:
                    values_pred = rollout_data.old_values + th.clamp(
                        values - rollout_data.old_values, -clip_range_vf, clip_range_vf)
                value_loss = F.mse_loss(rollout_data.returns, values_pred)
                value_losses.append(value_loss.item())

                if entropy is None:
                    entropy_loss = -th.mean(-log_prob)
                else:
                    entropy_loss = -th.mean(entropy)
                entropy_losses.append(entropy_loss.item())

                # --- CAPS spatial smoothness: similar states -> similar actions ---
                obs = rollout_data.observations
                mu = self.policy.get_distribution(obs).distribution.mean
                s_bar = obs + th.randn_like(obs) * self.caps_sigma
                mu_bar = self.policy.get_distribution(s_bar).distribution.mean
                caps_loss = ((mu - mu_bar) ** 2).sum(dim=1).mean()
                caps_losses.append(caps_loss.item())

                loss = (policy_loss + self.ent_coef * entropy_loss + self.vf_coef * value_loss
                        + self.caps_lambda * caps_loss)
                if self.anchor_lambda > 0.0:
                    idx = th.randint(0, self.anchor_obs.shape[0], (self.anchor_batch,))
                    mu_a = self.policy.get_distribution(self.anchor_obs[idx]).distribution.mean
                    anchor_loss = ((mu_a - self.anchor_mu[idx]) ** 2).sum(dim=1).mean()
                    anchor_losses.append(anchor_loss.item())
                    loss = loss + self.anchor_lambda * anchor_loss
                if self.lead_lambda > 0.0:
                    idx = th.randint(0, self.lead_obs.shape[0], (self.lead_batch,))
                    mu_l = self.policy.get_distribution(self.lead_obs[idx]).distribution.mean
                    lead_loss = ((mu_l - self.lead_target[idx]) ** 2).sum(dim=1).mean()
                    lead_losses.append(lead_loss.item())
                    loss = loss + self.lead_lambda * lead_loss

                with th.no_grad():
                    log_ratio = log_prob - rollout_data.old_log_prob
                    approx_kl_div = th.mean((th.exp(log_ratio) - 1) - log_ratio).cpu().numpy()
                    approx_kl_divs.append(approx_kl_div)
                if self.target_kl is not None and approx_kl_div > 1.5 * self.target_kl:
                    continue_training = False
                    break

                self.policy.optimizer.zero_grad()
                loss.backward()
                th.nn.utils.clip_grad_norm_(self.policy.parameters(), self.max_grad_norm)
                self.policy.optimizer.step()
            self._n_updates += 1
            if not continue_training:
                break

        explained_var = explained_variance(self.rollout_buffer.values.flatten(), self.rollout_buffer.returns.flatten())
        self.logger.record("train/entropy_loss", np.mean(entropy_losses))
        self.logger.record("train/policy_gradient_loss", np.mean(pg_losses))
        self.logger.record("train/value_loss", np.mean(value_losses))
        self.logger.record("train/caps_loss", np.mean(caps_losses))
        if anchor_losses:
            self.logger.record("train/anchor_loss", np.mean(anchor_losses))
        if lead_losses:
            self.logger.record("train/lead_loss", np.mean(lead_losses))
        self.logger.record("train/approx_kl", np.mean(approx_kl_divs))
        self.logger.record("train/clip_fraction", np.mean(clip_fractions))
        self.logger.record("train/loss", loss.item())
        self.logger.record("train/explained_variance", explained_var)
        if hasattr(self.policy, "log_std"):
            self.logger.record("train/std", th.exp(self.policy.log_std).mean().item())
        self.logger.record("train/n_updates", self._n_updates, exclude="tensorboard")
        self.logger.record("train/clip_range", clip_range)
