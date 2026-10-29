#!/usr/bin/env python3
"""The balance loop's timing as the robot runs it (2026-10-29): where sim/lqr/fw_timing.py's numbers come from.

    python3 tools/balance/loop_timing.py data/captures/<balance_watch recording>.csv [more] [--telem riderlog.txt
            --since 2026-10-28T12:45]

From balance_watch.py recordings (a timestamp every cycle): per controller, the distribution of the loop period in
closed_loop.SUB_S (0.25 ms) bins -- the bins and their shares are what fw_timing.py draws each simulated cycle
from. Periods over MAX_MS are counted apart (rare stalls, not drawn).
--telem: a copy of the Pi's riderlog (rider/debug/telem lines): per controller, the medians of the per-window
worst cases the firmware reports -- wheel reads (rdms), compute + write (post), whole cycle (wkms), the policy
network (inf). Each is the worst of ~60 cycles, so a typical cycle is a little shorter (their spread is narrow).
"""
import argparse
import collections

import numpy as np

BIN_MS = 0.25
MAX_MS = 10.0


def periods(files):
    per = collections.defaultdict(list)
    for f in files:
        a = np.genfromtxt(f, delimiter=",", names=True, dtype=None, encoding=None)
        if a.dtype.names is None or not {"ctrl", "t_us", "window", "i", "cmd"} <= set(a.dtype.names):
            print(f"skipped (not a balance_watch recording): {f}")
            continue
        for w in np.unique(a["window"]):
            r = a[a["window"] == w]
            if len(r) < 300 or np.any(np.diff(r["i"]) != 1) or np.mean(r["cmd"] != 0) < 0.5:
                continue                                  # incomplete, or not balancing
            per[str(r["ctrl"][0])].append(np.diff(r["t_us"].astype(np.int64)) / 1000.0)
    return {c: np.concatenate(d) for c, d in per.items()}


def telem(path, since):
    rows = collections.defaultdict(list)
    keys = ("rdms", "post", "wkms", "inf")
    for ln in open(path, errors="replace"):
        if "rider/debug/telem" not in ln or " en=1 " not in ln or ln[:16] < since:
            continue
        d = dict(x.split("=", 1) for x in ln.split() if "=" in x)
        if all(k in d for k in keys) and "ctrl" in d:
            rows[d["ctrl"]].append([float(d[k]) for k in keys])
    return {c: (keys, np.array(r)) for c, r in rows.items()}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("files", nargs="+")
    ap.add_argument("--telem")
    ap.add_argument("--since", default="", help="telemetry from this time on, e.g. 2026-10-28T12:45 (the firmware "
                                                "the numbers describe: older lines ran other firmware)")
    a = ap.parse_args()
    for c, d in periods(a.files).items():
        stall = d[d > MAX_MS]
        d = d[d <= MAX_MS]
        b = np.round(d / BIN_MS) * BIN_MS
        vals, n = np.unique(b, return_counts=True)
        keep = n / len(b) >= 0.001
        print(f"{c}: {len(d)} cycles, mean {d.mean():.3f} ms ({1000 / d.mean():.0f} Hz), median {np.median(d):.2f}; "
              f"{len(stall)} stalls over {MAX_MS:.0f} ms ({', '.join(f'{s:.0f}' for s in stall)} ms)")
        print(f"   periods_ms = {tuple(float(v) for v in vals[keep])}")
        print(f"   shares     = {tuple(round(float(x), 4) for x in n[keep] / n[keep].sum())}")
    if a.telem:
        for c, (keys, r) in telem(a.telem, a.since).items():
            med = np.median(r, axis=0)
            lo, hi = np.percentile(r, 5, axis=0), np.percentile(r, 95, axis=0)
            print(f"{c}: {len(r)} telemetry windows, worst per window -- " + ", ".join(
                f"{k} {m:.2f} ms ({l:.2f}-{h:.2f})" for k, m, l, h in zip(keys, med, lo, hi)))


if __name__ == "__main__":
    main()
