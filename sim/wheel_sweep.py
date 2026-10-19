#!/usr/bin/env python3
"""Wheel frequency response from the firmware's frequency sweep ('chirp', 2026-10-19) -- floor vs sim.

The firmware adds a known log-swept sine r(t) to the forward wheel command while the robot balances. With r
known, the wheel's own response is   H(f) = S_ry / S_ru :  u = the total forward command (controller + r),
y = the forward wheel speed from the encoders. Closed-loop recordings without r (S_uy / S_uu) are biased by
the controller; this ratio is not, because r is independent of everything else in the loop.

Floor:  ./.venv/bin/python wheel_sweep.py floor.csv          (tools/capture/chirp_capture.py output)
Sim:    ./.venv/bin/python wheel_sweep.py --sim [--lat 0.004] [--tau 0.013] [--ctrl lqr|pol]
        runs the SAME sweep in drive_sim (robot balancing in place) and analyses it identically.
Both:   ./.venv/bin/python wheel_sweep.py floor.csv --sim ...   (side by side)
"""
import argparse
from dataclasses import replace
import numpy as np

FREQS = [1.0, 1.5, 2.0, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0]
AMP, F0, F1, SECS = 60.0, 1.0, 10.0, 20.0         # the firmware defaults (main.cpp gChirpA/F0/F1/Dur). 60, not 15-30:
                                                   # holding still, the wheel sits in its dead zone; a small sweep
                                                   # stays nonlinear (sim harness check: coherence ~0.5-0.8 and
                                                   # erratic lags at +/-30, ~0.98 and clean at +/-60)


def chirp(t, amp=AMP, f0=F0, f1=F1, secs=SECS, t0=0.0):
    """The firmware's sweep: log frequency sweep, 0.5 s fade in/out, zero outside [t0, t0+secs]."""
    tt = t - t0
    if tt <= 0 or tt >= secs:
        return 0.0
    k = np.log(f1 / f0) / secs
    ph = 2 * np.pi * f0 * (np.exp(k * tt) - 1) / k        # integral of f0*exp(k t)
    env = min(1.0, tt / 0.5, (secs - tt) / 0.5)
    return amp * env * np.sin(ph)


def frf(t, r, u, y, band=0.08):
    """H = S_ry / S_ru from the WHOLE record's spectra (a sweep visits each frequency once, so averaging short
    segments mostly averages in segments where the sweep is elsewhere), the cross-spectra summed over the bins
    within +/-band (fraction) of each frequency. Also the r->y coherence over the same bins."""
    dt = float(np.mean(np.diff(t)))
    w = np.hanning(len(r))
    R = np.fft.rfft((r - r.mean()) * w); U = np.fft.rfft((u - u.mean()) * w); Y = np.fft.rfft((y - y.mean()) * w)
    fr = np.fft.rfftfreq(len(r), dt)
    out = {}
    for f0 in FREQS:
        k = (fr >= f0 * (1 - band)) & (fr <= f0 * (1 + band))
        Sru, Sry = np.sum(np.conj(R[k]) * U[k]), np.sum(np.conj(R[k]) * Y[k])
        H = Sry / Sru
        coh = np.abs(Sry) ** 2 / (np.sum(np.abs(R[k]) ** 2) * np.sum(np.abs(Y[k]) ** 2) + 1e-12)
        lag = -np.degrees(np.angle(H))
        out[f0] = (abs(H), lag / 360.0 / f0 * 1000.0, coh)
    return out


def floor(path):
    a = np.genfromtxt(path, delimiter=",", names=True, dtype=None, encoding=None)
    t = (a["t_us"] - a["t_us"][0]) / 1e6
    r = a["inj"].astype(float)
    u = (a["cmdL"] - a["cmdR"]) / 2.0                     # forward command (L:+u / R:-u mirrored)
    wx = a["wx_01mm"] / 10.0                              # mm, encoder forward position
    y = np.gradient(wx, t)                                # mm/s
    return t, r, u, y


def sim(lat, tau, ctrl, gain):
    import drive_sim as ds
    from rider_params import DEFAULT
    p = replace(DEFAULT, latency_s=lat, actuator_tau_s=tau, vel_max_rad_s=DEFAULT.vel_max_rad_s * gain)
    t0 = 2.0
    kw = dict(controller="lqr") if ctrl == "lqr" else dict(drive="lead")
    infer = None if ctrl == "lqr" else ds.load_policy("ppo_ld8_s6")
    L, fell, dt = ds.simulate(infer, +1, 0.0, T=t0 + SECS + 0.5, params=p, inject=lambda t: chirp(t, t0=t0), **kw)
    if fell is not None:
        print(f"   (sim FELL at {fell:.1f} s)")
    wx = np.cumsum(L[:, 8]) * dt * 1000.0                 # the ENCODER position (mm): sum of its quantized steps
    m = (L[:, 0] >= t0) & (L[:, 0] <= t0 + SECS)
    t, r, u, wx = L[m, 0][::2], L[m, 9][::2], L[m, 3][::2] * ds.POL_FULL_CMD, wx[m][::2]   # every 2nd cycle, as fw
    return t, r, u, np.gradient(wx, t)                    # same speed estimate as the floor's


def show(name, res):
    print(f"{name}\n   " + "  ".join(f"{f:4.1f}Hz {g:4.2f}/{l:4.0f}ms c{c:.2f}" for f, (g, l, c) in res.items()))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("floor", nargs="?")
    ap.add_argument("--sim", action="store_true")
    ap.add_argument("--lat", type=float, default=None)
    ap.add_argument("--tau", type=float, default=None)
    ap.add_argument("--gain", type=float, default=1.0)
    ap.add_argument("--ctrl", default="lqr")
    a = ap.parse_args()
    print("cells: gain (mm/s of wheel speed per cmd) / lag (ms) / coherence r->y")
    if a.floor:
        show(f"FLOOR {a.floor}", frf(*floor(a.floor)))
    if a.sim:
        from rider_params import DEFAULT
        lat = DEFAULT.latency_s if a.lat is None else a.lat
        tau = DEFAULT.actuator_tau_s if a.tau is None else a.tau
        show(f"SIM {a.ctrl} lat {lat * 1000:.0f} ms tau {tau * 1000:.0f} ms gain x{a.gain}", frf(*sim(lat, tau, a.ctrl, a.gain)))


if __name__ == "__main__":
    main()
