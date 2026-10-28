#!/usr/bin/env python3
"""Take an LQR2 floor recording apart term by term at its oscillation frequency (2026-10-28: LQR2 shook at
~14 Hz on the floor, pinned at +/-400, though the model called it safe).

    python3 tools/balance/lqr2_terms.py data/captures/2026-10-28_lqr2_watch.csv [windows, e.g. 5,6,7]

From the logged signals (balance_watch.py / drivecap: u, both wheels' speed registers, the raw gyro rate, the
fused pitch, every cycle) the LQR2 law is rebuilt as main.cpp computes it:
    u = -(L2X*x_axle + Xi*Ix + L2Vx*vS + L2Q*pitch + L2Dq*dqf) + fric*tanh(uLp/20)
    vS = 8 ms smoothing of (wheel_vr - r*rate*271), dqf = 0.6 dqf + 0.4 rate, pitch = theta - 3.55
(x_axle and Ix are rebuilt from the speed; at 14 Hz they are small). Printed per window: each term's amplitude
and phase at the oscillation's frequency against the logged command, the rebuilt sum against the logged
command (a check on the rebuild), and the encoder speed against r x the lean rate there (the hand test
gave -0.92 at slow rocking: is the correction still right at 14 Hz?).
"""
import sys

import os

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "capture"))
from balance_watch import sample_times

V_COUNTS = 327.6                     # speed register counts per m/s
SCALED = 4.5 * 60 * np.pi / 1024     # 0.828 scaled units per count
SPEED = V_COUNTS * SCALED            # 271 scaled units per m/s
R = 0.030
K = dict(x=-1844.0, xi=-30.0, vx=-6.89, q=46.0, dq=3.67)
VTAU, FRIC, FW, FTAU, SET = 0.008, 32.5, 20.0, 0.030, 3.55


def amp(x, t, f):
    """Complex amplitude of x at f Hz (single-bin DFT, mean removed)."""
    return 2 * np.sum((x - x.mean()) * np.exp(-2j * np.pi * f * t)) / len(t)


def main():
    a = np.genfromtxt(sys.argv[1], delimiter=",", names=True, dtype=None, encoding=None)
    only = {int(w) - 1 for w in sys.argv[2].split(",")} if len(sys.argv) > 2 else None
    for w in np.unique(a["window"]):
        r = a[a["window"] == w]
        if str(r["ctrl"][0]) != "LQR2" or np.mean(r["cmd"] != 0) < 0.5 or (only and w not in only):
            continue
        t = sample_times(r["i"], r["t_us"])
        dt = np.diff(t, prepend=t[0] - np.median(np.diff(t)))
        u = r["cmd"].astype(float)
        rate = r["rate_x10"] / 10.0                       # deg/s, firmware frame
        pitch = r["pitch_cdeg"] / 100.0 - SET
        vr = (r["v"] - r["vR"]) / 2.0 * SCALED              # wheel_vr: left + right (mirrored), scaled units
        vax = vr - R * np.radians(rate) * SPEED
        vS = np.zeros_like(vax); dqf = np.zeros_like(rate); ulp = np.zeros_like(u)
        vS[0], dqf[0] = vax[0], rate[0]
        for i in range(1, len(t)):
            av = np.exp(-dt[i] / VTAU)
            vS[i] = av * vS[i - 1] + (1 - av) * vax[i]
            dqf[i] = 0.6 * dqf[i - 1] + 0.4 * rate[i]
        x = np.cumsum(vr / SPEED * dt) - R * np.radians(pitch)
        x -= x.mean()
        terms = {"position": -K["x"] * x, "speed": -K["vx"] * vS, "pitch": -K["q"] * pitch, "rate": -K["dq"] * dqf}
        pre = sum(terms.values())
        for i in range(1, len(t)):
            af = np.exp(-dt[i] / FTAU)
            ulp[i] = af * ulp[i - 1] + (1 - af) * pre[i]
        terms["friction"] = FRIC * np.tanh(ulp / FW)
        rebuilt = np.clip(sum(terms.values()), -400, 400)
        fs = 1 / np.median(np.diff(t))
        X = np.abs(np.fft.rfft((u - u.mean()) * np.hanning(len(u)))); fr = np.fft.rfftfreq(len(u), 1 / fs)
        f = fr[fr > 5][np.argmax(X[fr > 5])]
        U = amp(u, t, f)
        print(f"window {w + 1}: oscillation {f:.1f} Hz, command amplitude {abs(U):.0f} units; "
              f"rebuilt vs logged command: corr {np.corrcoef(rebuilt, u)[0, 1]:+.3f}, rms diff {np.std(rebuilt - u):.0f}")
        for name, y in terms.items():
            Y = amp(y, t, f)
            print(f"   {name:9s} {abs(Y):6.0f} units  phase {np.degrees(np.angle(Y / U)):+5.0f} deg vs command")
        enc = amp(vr / SPEED, t, f)
        body = amp(R * np.radians(-rate), t, f)            # r x forward-lean rate
        print(f"   encoder speed vs r x lean rate at {f:.1f} Hz: {abs(enc / body):.2f} x, phase "
              f"{np.degrees(np.angle(enc / body)):+.0f} deg (axle still = 1.00 at 180 deg)")
        print(f"   left vs right wheel speed at {f:.1f} Hz: {abs(amp(r['v'], t, f) / amp(-r['vR'], t, f)):.2f} x, phase "
              f"{np.degrees(np.angle(amp(r['v'], t, f) / amp(-r['vR'], t, f))):+.0f} deg")


if __name__ == "__main__":
    main()
