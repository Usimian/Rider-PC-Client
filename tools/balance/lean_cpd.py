"""Leg counts per degree of body roll, measured with roll level holding set angles (2026-10-29).

    python3 tools/balance/lean_cpd.py [--deg 3] [--hold 6]
    python3 tools/balance/lean_cpd.py --deg 15 --hold 8     # past the legs' reach: the roll at full authority

The robot balancing in place on the floor (either controller), legs powered. Turns roll level on, holds the body at
0, +deg, -deg, 0 (levset; the accumulator moves the legs until the fused roll gets there), then turns it off (the legs
equalize). The roll split is the same count on both legs (legR + c, legL + c), so c = the change of (legR + legL) / 2;
counts per degree = the change of c over the change of roll, from the last half of each hold. Commands go through the
Pi (rider/control/line, JSON); readings from its riderlog (rider/debug/telem, ~10 Hz).
MEASURED 2026-10-29: +/-3 deg took +12 / -1 counts per leg from level (the servos' slack: no single feed-forward
count, so the turn lean is closed loop); +/-15 deg took +/-24, ~1.5 counts/deg both ways. At ride height the legs'
limits leave ~47 counts each way -> ~+/-30 deg at full reach (DERIVED, straight past 15): the roll fall cutoff
('fallroll') sits past that.
"""
import argparse
import json
import re
import subprocess
import time

import numpy as np


def send(line):
    msg = json.dumps({"line": line})
    subprocess.run(["ssh", "rider", f"mosquitto_pub -t rider/control/line -m '{msg}'"], check=True)


def telem(n):
    out = subprocess.run(["ssh", "rider", f"grep -a rider/debug/telem /home/pi/riderlog.txt | tail -{n}"],
                         capture_output=True, text=True, check=True).stdout
    rows = []
    for line in out.splitlines():
        d = dict(re.findall(r"(\w+)=(-?[\d.]+)", line))
        if all(k in d for k in ("roll", "legR", "legL", "en", "lev")):
            rows.append((float(d["roll"]), float(d["legR"]), float(d["legL"]), int(d["en"]), int(d["lev"])))
    return np.array(rows)


def hold(target, seconds):
    send(f"levset {target:g}")
    time.sleep(seconds)
    r = telem(int(seconds * 10 / 2))                     # the hold's last half
    if not (r[:, 3] == 1).all() or not (r[:, 4] == 1).all():
        raise SystemExit("not balancing with roll level on -- stopped")
    roll, c = r[:, 0].mean(), ((r[:, 1] + r[:, 2]) / 2).mean()
    print(f"  levset {target:+5.1f}: roll {roll:+6.2f} deg (sd {r[:, 0].std():.2f}), legs {r[-1, 1]:.0f}/{r[-1, 2]:.0f}, "
          f"c {c:7.1f}")
    return roll, c


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--deg", type=float, default=3.0)
    ap.add_argument("--hold", type=float, default=6.0)
    a = ap.parse_args()
    if not (telem(3)[:, 3] == 1).all():
        raise SystemExit("not balancing -- arm it first")
    send("level 1")
    try:
        pts = [hold(t, a.hold) for t in (0.0, a.deg, -a.deg, 0.0)]
    finally:
        send("levset 0")
        send("level 0")
    roll, c = np.array(pts).T
    k, b = np.polyfit(roll, c, 1)
    print(f"\ncounts per degree of roll: {abs(k):.2f}  (c = {k:+.2f} x roll {b:+.1f}; "
          f"+{a.deg:g} -> -{a.deg:g}: {abs((c[1] - c[2]) / (roll[1] - roll[2])):.2f})")
    print(f"0 at the start vs the end: roll {roll[0]:+.2f} / {roll[3]:+.2f}, c {c[0]:.1f} / {c[3]:.1f}")


if __name__ == "__main__":
    main()
