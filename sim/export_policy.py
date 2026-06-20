"""Export a trained SB3 PPO policy (+ VecNormalize obs stats) for ESP32 deployment.

Emits a .npz and a C header, and VALIDATES a pure-numpy reimplementation against
SB3's own output (the correctness gate before writing any firmware).

Deterministic policy:
    z = clip((obs - mean) / sqrt(var + eps), -clip, clip)      # VecNormalize
    a = clip( W3 @ tanh(W2 @ tanh(W1 @ z + b1) + b2) + b3, -1, 1 )

The ESP32 must, each control step, assemble `obs` = frame_stack copies of
[pitch, pitch_rate, x_err, x_vel, wheel_vel, INT_pitch, INT_x] (oldest..newest),
accumulating the two integrals (clamped) and the stack, then run the above.
"""
import sys
import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import VecNormalize, DummyVecEnv
from rider_env import RiderBalanceEnv

name = sys.argv[1] if len(sys.argv) > 1 else "ppo_v_dr_final"
FS = int(sys.argv[2]) if len(sys.argv) > 2 else 2
PFX = sys.argv[3] if len(sys.argv) > 3 else "POL"   # C macro/array prefix (POA for pos-aware -> no POL_ collision)
POS_AWARE = "--posaware" in sys.argv                  # validate in the position-aware env (policy SEES x_err/x_int)

model = PPO.load(name, device="cpu")
vn = VecNormalize.load(name + "_vecnorm.pkl",
                       DummyVecEnv([lambda: RiderBalanceEnv(frame_stack=FS, pure_balance=not POS_AWARE)]))

# --- extract weights: hidden Linears (policy_net) + output (action_net) ---
layers = []
for m in model.policy.mlp_extractor.policy_net:
    if isinstance(m, torch.nn.Linear):
        layers.append((m.weight.detach().numpy().astype(np.float64),
                       m.bias.detach().numpy().astype(np.float64)))
an = model.policy.action_net
layers.append((an.weight.detach().numpy().astype(np.float64),
               an.bias.detach().numpy().astype(np.float64)))

mean = vn.obs_rms.mean.astype(np.float64)
var = vn.obs_rms.var.astype(np.float64)
eps = float(vn.epsilon)
clip = float(vn.clip_obs)
obs_dim = mean.shape[0]


def npforward(raw_obs):
    z = np.clip((raw_obs - mean) / np.sqrt(var + eps), -clip, clip)
    x = z
    for i, (W, b) in enumerate(layers):
        x = W @ x + b
        if i < len(layers) - 1:
            x = np.tanh(x)
    return np.clip(x, -1.0, 1.0)


# --- validate against SB3 over real rollouts ---
env = RiderBalanceEnv(frame_stack=FS, add_noise=False, pure_balance=not POS_AWARE)
maxerr, nsteps = 0.0, 0
for s in range(20):
    obs, _ = env.reset(seed=s)
    for _ in range(40):
        a_sb3, _ = model.predict(vn.normalize_obs(obs), deterministic=True)
        a_np = npforward(obs.astype(np.float64))
        maxerr = max(maxerr, abs(float(a_sb3[0]) - float(a_np[0])))
        nsteps += 1
        obs, _, term, trunc, _ = env.step(a_sb3)
        if term or trunc:
            break

print(f"obs_dim={obs_dim} (frame_stack={FS} x 7)  hidden={[W.shape[0] for W,_ in layers[:-1]]}")
print(f"numpy-vs-SB3 max |action| error over {nsteps} steps: {maxerr:.2e}  "
      f"{'OK (portable)' if maxerr < 1e-5 else 'MISMATCH'}")

# --- save npz ---
out = {"mean": mean, "var": var, "eps": eps, "clip": clip, "frame_stack": FS}
for i, (W, b) in enumerate(layers):
    out[f"W{i}"] = W
    out[f"b{i}"] = b
np.savez(name + "_export.npz", **out)


# --- emit C header ---
def _fl(v):
    s = "%.8g" % float(v)                      # ensure a valid C float literal (e.g. "0" -> "0.0f")
    if not any(c in s for c in ".eEnf"):
        s += ".0"
    return s + "f"


def carr(a):
    return "{" + ",".join(_fl(v) for v in np.asarray(a).ravel()) + "}"


with open(name + "_policy.h", "w") as f:
    kind = "POSITION-AWARE (sees x_err/x_int)" if POS_AWARE else "deterministic"
    f.write(f"// auto-generated from {name} -- {kind} PPO policy for ESP32 ({PFX}_*)\n")
    f.write(f"// obs (len {obs_dim}) = {FS} frames of [pitch,prate,x_err,x_vel,wheel_vel,INT_pitch,INT_x]\n")
    f.write(f"#define {PFX}_OBS {obs_dim}\n#define {PFX}_FS {FS}\n")
    f.write(f"#define {PFX}_CLIP {clip:.1f}f\n#define {PFX}_EPS {eps:.8g}f\n")
    f.write(f"static const float {PFX}_MEAN[{obs_dim}]={carr(mean)};\n")
    f.write(f"static const float {PFX}_VAR[{obs_dim}]={carr(var)};\n")
    for i, (W, b) in enumerate(layers):
        r, c = W.shape
        f.write(f"static const float {PFX}_W{i}[{r}][{c}]={{")
        f.write(",".join(carr(W[k]) for k in range(r)))
        f.write(f"}};\nstatic const float {PFX}_B{i}[{r}]={carr(b)};\n")
print(f"wrote {name}_export.npz and {name}_policy.h ({sum(W.size+b.size for W,b in layers)} weights)")
