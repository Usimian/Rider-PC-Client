"""Export the trained diff-drive (joint balance+turn) PPO policy for ESP32 (Phase 3).

Same scheme as export_policy.py but for the 2-OUTPUT per-wheel policy on the 8-field obs.
Emits .npz + a C header and validates a pure-numpy reimplementation against SB3.

Deterministic policy:
    z = clip((obs - mean)/sqrt(var+eps), -clip, clip)
    a = clip( W3 @ tanh(W2 @ tanh(W1 @ z + b1) + b2) + b3, -1, 1 )   # a is 2-vector [aL, aR]

ESP32 obs each step = frame_stack copies of
[pitch, pitch_rate, yaw_rate, wheelL_vel, wheelR_vel, cmd_fwd, cmd_yaw, pitch_int] (oldest..newest).
The two outputs map straight to the left/right wheel velocity commands (NO u+tn summing).
"""
import sys
import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import VecNormalize, DummyVecEnv
from rider_env_3d import RiderDiffDriveEnv

name = sys.argv[1] if len(sys.argv) > 1 else "ppo_diffdrive_v2"
FS = int(sys.argv[2]) if len(sys.argv) > 2 else 2

model = PPO.load(name, device="cpu")
vn = VecNormalize.load(name + "_vecnorm.pkl", DummyVecEnv([lambda: RiderDiffDriveEnv(frame_stack=FS)]))

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
n_out = layers[-1][0].shape[0]


def npforward(raw_obs):
    z = np.clip((raw_obs - mean) / np.sqrt(var + eps), -clip, clip)
    x = z
    for i, (W, b) in enumerate(layers):
        x = W @ x + b
        if i < len(layers) - 1:
            x = np.tanh(x)
    return np.clip(x, -1.0, 1.0)


env = RiderDiffDriveEnv(frame_stack=FS, add_noise=False, domain_rand=False)
maxerr, nsteps = 0.0, 0
for s in range(20):
    obs, _ = env.reset(seed=s)
    for _ in range(60):
        a_sb3, _ = model.predict(vn.normalize_obs(obs), deterministic=True)
        a_np = npforward(obs.astype(np.float64))
        maxerr = max(maxerr, float(np.max(np.abs(np.asarray(a_sb3).flatten() - a_np))))
        nsteps += 1
        obs, _, term, trunc, _ = env.step(a_sb3)
        if term or trunc:
            break

print(f"obs_dim={obs_dim} (frame_stack={FS} x 8)  outputs={n_out}  hidden={[W.shape[0] for W,_ in layers[:-1]]}")
print(f"numpy-vs-SB3 max |action| error over {nsteps} steps: {maxerr:.2e}  "
      f"{'OK (portable)' if maxerr < 1e-5 else 'MISMATCH'}")

out = {"mean": mean, "var": var, "eps": eps, "clip": clip, "frame_stack": FS, "n_out": n_out}
for i, (W, b) in enumerate(layers):
    out[f"W{i}"] = W
    out[f"b{i}"] = b
np.savez(name + "_export.npz", **out)


def _fl(v):
    s = "%.8g" % float(v)
    if not any(c in s for c in ".eEnf"):
        s += ".0"
    return s + "f"


def carr(a):
    return "{" + ",".join(_fl(v) for v in np.asarray(a).ravel()) + "}"


with open(name + "_policy.h", "w") as f:
    f.write(f"// auto-generated from {name} -- deterministic diff-drive PPO policy for ESP32\n")
    f.write(f"// obs (len {obs_dim}) = {FS} frames of "
            f"[pitch,prate,yaw_rate,wL_vel,wR_vel,cmd_fwd,cmd_yaw,pitch_int]\n")
    f.write(f"// outputs ({n_out}) = [aL, aR] -> left/right wheel velocity cmd (no u+tn summing)\n")
    f.write(f"#define POLJ_OBS {obs_dim}\n#define POLJ_FS {FS}\n#define POLJ_OUT {n_out}\n")
    f.write(f"#define POLJ_CLIP {clip:.1f}f\n#define POLJ_EPS {eps:.8g}f\n")
    f.write(f"static const float POLJ_MEAN[{obs_dim}]={carr(mean)};\n")
    f.write(f"static const float POLJ_VAR[{obs_dim}]={carr(var)};\n")
    for i, (W, b) in enumerate(layers):
        r, c = W.shape
        f.write(f"static const float POLJ_W{i}[{r}][{c}]={{")
        f.write(",".join(carr(W[k]) for k in range(r)))
        f.write(f"}};\nstatic const float POLJ_B{i}[{r}]={carr(b)};\n")
print(f"wrote {name}_export.npz and {name}_policy.h ({sum(W.size+b.size for W,b in layers)} weights)")
