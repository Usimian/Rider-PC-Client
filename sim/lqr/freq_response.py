"""The frequency sweep, robot against model (the Hover's sim/freq_response.py, for the Rider's chirp captures).

    python3 sim/lqr/freq_response.py                          # both standing LQR sweeps (10-19, 10-28)
    python3 sim/lqr/freq_response.py 2026-10-28_chirp_lqr.csv:2.82

The robot recording: tools/capture/chirp_capture.py (the firmware's 'chirp' added to the balance output,
before the stiction boost... no: after it, at the wheel command; main.cpp adds it to uout). The model
(closed_loop.py) runs the same sweep with the same balance point ('up', the IMU's reading with the centre
of mass over the axle: given per file after ':' or taken as the recording's mean pitch). For each 1/3-octave
band from the stretch where the sweep ran (cross and auto spectra summed over the band's bins):

  pitch/inj   the pitch (deg) per unit of sweep
  wheel/inj   encoder travel (mm) per unit of sweep
  reply/inj   the LQR's own output per unit of sweep (how hard it answers)
  coh         coherence, 0..1 (below ~0.5 the band is unreliable)

Gains are |H|, phases in degrees (negative = lags the sweep); ratio = robot / model.
"""
import os
import sys

import numpy as np

from closed_loop import Firmware, run

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "..", "..", "data", "captures")
DEFAULT = ["2026-10-19_chirp_lqr.csv", "2026-10-28_chirp_lqr.csv"]
SWEEP = (60.0, 1.0, 10.0, 20.0)
BANDS = [1.0 * 2 ** (k / 3) for k in range(0, 11)]   # 1 .. ~10 Hz


def load(path):
    d = np.genfromtxt(path, delimiter=",", names=True, dtype=None, encoding=None)
    return dict(t=(d["t_us"] - d["t_us"][0]) * 1e-6, inj=d["inj"].astype(float), u=(d["cmdL"] - d["cmdR"]) / 2.0,
                th=d["th_x100"] / 100.0, x=d["wx_01mm"] / 10.0)


def from_rows(rows):
    return dict(t=(rows[:, 0] - rows[0, 0]) * 1e-6, inj=rows[:, 1], u=rows[:, 2], th=rows[:, 5] / 100.0,
                x=rows[:, 4] / 10.0)


def bands(r):
    nz = np.nonzero(r["inj"])[0]
    sl = slice(nz[0], nz[-1])
    t = r["t"][sl]
    fs = 1.0 / np.median(np.diff(t))
    out = {}
    U = np.fft.rfft(r["inj"][sl] - r["inj"][sl].mean())
    fr = np.fft.rfftfreq(len(t), 1 / fs)
    for name, y in (("pitch", r["th"]), ("wheel", r["x"]), ("reply", r["u"] - r["inj"])):
        Y = np.fft.rfft(y[sl] - y[sl].mean())
        res = []
        for lo in BANDS[:-1]:
            hi = lo * 2 ** (1 / 3)
            m = (fr >= lo) & (fr < hi)
            suy, suu, syy = np.sum(np.conj(U[m]) * Y[m]), np.sum(np.abs(U[m]) ** 2), np.sum(np.abs(Y[m]) ** 2)
            res.append((np.sqrt(lo * hi), suy / suu, abs(suy) ** 2 / (suu * syy)))
        out[name] = res
    return out


def main():
    args = sys.argv[1:] or DEFAULT
    for a in args:
        name, _, up = a.partition(":")
        robot = load(os.path.join(DATA, name))
        up = float(up) if up else float(robot["th"].mean())
        rows, fell = run(fw=Firmware(up_deg=up), T=24.0, sweep=SWEEP, sweep_at=3.0)
        print(f"\n{name} (balance point {up:.2f} deg){'  -- MODEL FELL' if fell else ''}")
        rb, mb = bands(robot), bands(from_rows(rows))
        for q in ("pitch", "wheel", "reply"):
            print(f"  {q}/inj    band     robot             model             ratio  coh robot/model")
            for (f, hr, cr), (_, hm, cm) in zip(rb[q], mb[q]):
                print(f"            {f:4.1f} Hz  {abs(hr):7.4f} {np.degrees(np.angle(hr)):+5.0f} deg   "
                      f"{abs(hm):7.4f} {np.degrees(np.angle(hm)):+5.0f} deg   {abs(hr) / abs(hm):5.2f}  "
                      f"{cr:.2f}/{cm:.2f}")


if __name__ == "__main__":
    main()
