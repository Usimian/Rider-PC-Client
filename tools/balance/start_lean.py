"""Each full-stick start and stop in the Pi's telemetry, its lean against the standing level (2026-11-03: the pitch
estimate corrected for the plan's acceleration, 'acccomp', A/B on the floor).

    python3 tools/balance/start_lean.py data/riderlog/<file>.txt [--split HH:MM:SS]

The input: telemetry lines from /home/pi/riderlog.txt (ssh rider "grep -a '<date>T<hour>.*rider/debug/telem' ...").
A start = the plan's speed (vdes, m/s) leaving +/-0.01 for past 0.05 and peaking at MIN_PEAK or more (slower ones are
the held home's gentle return, 0.15 m/s); its release = the plan's speed falling back under half its peak. Per
start, forward or backward, the pitch less its mean over the 0.5 s before, as a lean INTO the direction of travel
(+; th is the firmware's frame, a forward lean negative):
  t90        the time from the start to the wheels at 90 % of the plan's peak speed (wv; +-0.1 s at ~10 Hz)
  lean       the deepest lean into the travel in the first 1 s
  lie-back   the deepest lean against it 0.6-2 s on, while still driving (the estimate recovering from the start)
  brake      the deepest lean against it in the first 1 s after the release
  settle     the lean's range 1-3 s after the release
  rolls      how far the wheels go on in the direction of travel (wx) from the release to 0.8 s after
  raw-fused  the raw accelerometer pitch less the fused one, mean over the first 0.5 s (praw - th; the acceleration
             the accelerometer reads as tilt -- the fused pitch follows it at ~0.5 s unless corrected)
--split: the time the test switched (e.g. acccomp 1 -> 0): the starts before and after are summed up apart.
The telemetry is ~10 Hz: magnitudes over a fraction of a second, not fast detail.
"""
import argparse
import re

import numpy as np

KEYS = ("ms", "th", "praw", "vdes", "wx", "en", "wv")
SPEED_PER_MPS = 271.4      # telemetry wv units per m/s (main.cpp LQR_SPEED_PER_MPS)
MIN_PEAK = 0.25


def load(path):
    rows, stamps = [], []
    for line in open(path, errors="replace"):
        d = dict(re.findall(r"(\w+)=(-?[\d.]+)", line))
        if all(k in d for k in KEYS):
            rows.append([float(d[k]) for k in KEYS])
            stamps.append(line[11:19])
    return np.array(rows), stamps


def starts(a):
    ms, th, praw, vd, wx, en, wv = a.T
    out, i = [], 1
    while i < len(a):
        if abs(vd[i - 1]) <= 0.01 and abs(vd[i]) > 0.01 and en[i] == 1:
            j = i
            while j < len(a) and abs(vd[j]) <= 0.05 and ms[j] - ms[i] < 500:
                j += 1
            if j < len(a) and abs(vd[j]) > 0.05:
                k = j
                peak = abs(vd[j])
                while k < len(a) and en[k] == 1 and (abs(vd[k]) >= 0.5 * peak or ms[k] - ms[i] < 300):
                    peak = max(peak, abs(vd[k]))
                    k += 1
                if k < len(a) and en[k] == 1 and peak >= MIN_PEAK:
                    out.append((i, k, peak, float(np.sign(vd[j]))))
                i = k
        i += 1
    return out


def window(ms, t0, lo, hi):
    return (ms >= t0 + lo * 1000) & (ms < t0 + hi * 1000)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("file")
    ap.add_argument("--split", default=None, help="HH:MM:SS: the starts before and after are summed up apart")
    a_ = ap.parse_args()
    a, stamps = load(a_.file)
    ms, th, praw, vd, wx, en, wv = a.T
    res = []
    print("  start     m/s     t90 s   lean  lie-back  brake   settle range  rolls mm  raw-fused")
    for i, k, peak, sg in starts(a):
        t0, tr = ms[i], ms[k]
        pre = window(ms, t0, -0.5, 0.0)
        if pre.sum() < 3:
            continue
        lean = -sg * (th - th[pre].mean())                        # + = into the direction of travel
        fwd = lean[window(ms, t0, 0, 1)].max()
        lw = window(ms, t0, 0.6, min(2.0, (tr - t0) / 1000)) if tr - t0 > 700 else None
        lie = lean[lw].min() if lw is not None and lw.any() else np.nan   # a start just over 0.7 s: maybe no sample
        brake = lean[window(ms, tr, 0, 1)].min()
        st = lean[window(ms, tr, 1, 3)]
        rng = st.max() - st.min() if len(st) else np.nan
        w3 = window(ms, tr, 0.7, 0.9)
        rolls = sg * (wx[w3].mean() - wx[k]) * 1000 if w3.any() else np.nan
        rf = -sg * (praw - th)[window(ms, t0, 0, 0.5)].mean()
        reach = np.nonzero((ms >= t0) & (sg * wv / SPEED_PER_MPS >= 0.9 * peak))[0]
        t90 = (ms[reach[0]] - t0) / 1000 if len(reach) else np.nan
        res.append((stamps[i], sg * peak, t90, fwd, lie, brake, rng, rolls, rf))
        print(f"  {stamps[i]}  {sg * peak:+5.2f}   {t90:4.2f}   {fwd:+6.2f}    {lie:+6.2f}   {brake:+6.2f}   {rng:5.2f}       "
              f"{rolls:+6.0f}    {rf:+6.2f}")
    if a_.split and res:
        for name, sel in (("before " + a_.split, [r for r in res if r[0] < a_.split]),
                          ("after  " + a_.split, [r for r in res if r[0] >= a_.split])):
            if not sel:
                continue
            m = np.nanmean(np.array([r[1:] for r in sel]), axis=0)
            print(f"  {name}  ({len(sel)} starts) mean: t90 {m[1]:.2f} s  lean {m[2]:+.2f}  lie-back {m[3]:+.2f}  "
                  f"brake {m[4]:+.2f}  settle {m[5]:.2f}  rolls {m[6]:+.0f} mm  raw-fused {m[7]:+.2f}")


if __name__ == "__main__":
    main()
