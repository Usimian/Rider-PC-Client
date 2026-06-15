#!/usr/bin/env python3
"""Offline symmetry / conditioning test for exported Rider PPO policies.

A balancer is physically mirror-symmetric: action(-state) should = -action(state).
A healthy policy should also sit near ZERO action at its operating point (the
VecNormalize mean, z=0) and not be jammed against the +/-1 action clip.

For each *_export.npz it reports, around the operating point:
  a_op      action at the training-mean state           (want ~0; +/-1 = saturated)
  sat%      fraction of the pitch-error sweep clipped    (want low)
  symErr    RMS odd-symmetry violation |a(+d)+a(-d)|     (want ~0)
  gNeg/gPos local gain on each side of the op point      (want equal magnitude)
  gRatio    max(|gNeg|,|gPos|)/min(...)                  (1 = symmetric; big = pathological)

Run:  /usr/bin/python3 sim/policy_symmetry.py
"""
import glob, os, numpy as np

def load(npz):
    d = np.load(npz)
    return (d['mean'], d['var'], float(d['eps']), float(d['clip']),
            [(d['W0'], d['b0']), (d['W1'], d['b1']), (d['W2'], d['b2'])])

def make_infer(mean, var, eps, clip, layers):
    def infer(o):
        x = np.clip((o - mean) / np.sqrt(var + eps), -clip, clip)
        for i, (W, b) in enumerate(layers):
            x = W @ x + b
            if i < len(layers) - 1:
                x = np.tanh(x)
        return float(np.clip(x, -1.0, 1.0)[0])
    return infer

def metrics(npz):
    mean, var, eps, clip, layers = load(npz)
    infer = make_infer(mean, var, eps, clip, layers)
    a_op = infer(mean)
    # pitch-error sweep about the operating point (obs idx 0 and 7 = pitch, both frames)
    pe = np.linspace(-0.3, 0.3, 241)
    a = []
    for v in pe:
        o = mean.copy(); o[0] += v; o[7] += v; a.append(infer(o))
    a = np.array(a)
    sat = np.mean(np.abs(a) >= 0.99)
    pos = pe > 0
    # odd-symmetry: a(+d) should equal -a(-d); compare mirrored halves
    half = len(pe) // 2
    a_neg = a[:half][::-1]      # a(-d) reversed -> aligns with a(+d)
    a_pos = a[half + 1:]
    n = min(len(a_neg), len(a_pos))
    sym_err = np.sqrt(np.mean((a_pos[:n] + a_neg[:n]) ** 2))
    # local gains at +/-0.05 rad
    def slope(center):
        o1 = mean.copy(); o1[0] += center + 0.02; o1[7] += center + 0.02
        o2 = mean.copy(); o2[0] += center - 0.02; o2[7] += center - 0.02
        return (infer(o1) - infer(o2)) / 0.04
    gNeg, gPos = slope(-0.05), slope(0.05)
    lo, hi = sorted([abs(gNeg), abs(gPos)])
    gRatio = (hi / lo) if lo > 1e-6 else float('inf')
    return dict(a_op=a_op, sat=sat, sym=sym_err, gNeg=gNeg, gPos=gPos, gRatio=gRatio)

def main():
    here = os.path.dirname(os.path.abspath(__file__))
    files = sorted(glob.glob(os.path.join(here, '*_export.npz')))
    print('%-26s %7s %6s %7s %8s %8s %8s' % ('policy', 'a_op', 'sat%', 'symErr', 'gNeg', 'gPos', 'gRatio'))
    print('-' * 80)
    for f in files:
        try:
            m = metrics(f)
        except Exception as e:
            print('%-26s  ERROR %s' % (os.path.basename(f), e)); continue
        flag = '  <-- saturated/asym' if (abs(m['a_op']) > 0.9 or m['gRatio'] > 3) else ''
        print('%-26s %+7.2f %6.0f %7.2f %+8.1f %+8.1f %8.1f%s'
              % (os.path.basename(f).replace('_export.npz', ''),
                 m['a_op'], 100 * m['sat'], m['sym'], m['gNeg'], m['gPos'], m['gRatio'], flag))

if __name__ == '__main__':
    main()
