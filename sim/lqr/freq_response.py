"""The frequency sweep, robot against model (the Hover's sim/freq_response.py, for the Rider's chirp captures).

    python3 sim/lqr/freq_response.py                 # the four sweeps of 2026-10-29 (SWEEPS)
    python3 sim/lqr/freq_response.py stand_30hz drive  # those whose file name contains any of these

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

from closed_loop import flashed, run

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "..", "..", "data", "captures")
# The sweeps on the firmware the robot runs now (2026-10-29, the LQR, on the floor; tools/capture/chirp_capture.py):
# (file, (amplitude, f0, f1, seconds), the stick's speed m/s while it ran). The standing one first: its mean pitch
# is the balance point the model uses for all.
SWEEPS = [("2026-10-29_chirp_lqr_stand_10hz.csv", (60.0, 1.0, 10.0, 20.0), 0.0),
          ("2026-10-29_chirp_lqr_stand_30hz.csv", (30.0, 1.0, 30.0, 20.0), 0.0),
          ("2026-10-29_chirp_lqr_drive_fwd.csv", (60.0, 1.0, 10.0, 10.0), 0.30),
          ("2026-10-29_chirp_lqr_drive_rev.csv", (60.0, 1.0, 10.0, 10.0), -0.30)]
SWEEP_AT = 4.0                                # the model's sweep starts here (driving: steady by then)
def bands_for(f0, f1):
    """1/3-octave band edges from f0 to f1."""
    n = int(np.floor(3 * np.log2(f1 / f0)))
    return [f0 * 2 ** (k / 3) for k in range(0, n + 1)]


def load(path):
    d = np.genfromtxt(path, delimiter=",", names=True, dtype=None, encoding=None)
    return dict(t=(d["t_us"] - d["t_us"][0]) * 1e-6, inj=d["inj"].astype(float), u=(d["cmdL"] - d["cmdR"]) / 2.0,
                th=d["th_x100"] / 100.0, x=d["wx_01mm"] / 10.0)


def from_rows(rows):
    return dict(t=(rows[:, 0] - rows[0, 0]) * 1e-6, inj=rows[:, 1], u=rows[:, 2], th=rows[:, 5] / 100.0,
                x=rows[:, 4] / 10.0)


def bands(r, edges=None):
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
        for lo in (edges or bands_for(1.0, 10.0))[:-1]:
            hi = lo * 2 ** (1 / 3)
            m = (fr >= lo) & (fr < hi)
            suy, suu, syy = np.sum(np.conj(U[m]) * Y[m]), np.sum(np.abs(U[m]) ** 2), np.sum(np.abs(Y[m]) ** 2)
            res.append((np.sqrt(lo * hi), suy / suu, abs(suy) ** 2 / (suu * syy)))
        out[name] = res
    return out


def model_rows(sweep, v, up, p=None):
    """The flashed LQR on the model (p: model.Params / model_flex.FlexParams, the default model if None) at the
    robot's timing: standing, or driving at v m/s (the stick from 0.5 s), the sweep from SWEEP_AT on."""
    drive = (np.array([0.0, 0.5]), np.array([0.0, v])) if v else None
    return run(p, fw=flashed(up_deg=up), T=SWEEP_AT + sweep[3] + 1.0, sweep=sweep, sweep_at=SWEEP_AT, drive=drive)


def main():
    picked = [s for s in SWEEPS if not sys.argv[1:] or any(a in s[0] for a in sys.argv[1:])]
    up = float(load(os.path.join(DATA, SWEEPS[0][0]))["th"].mean())      # the balance point: the standing sweep's
    for name, sweep, v in picked:
        robot = load(os.path.join(DATA, name))
        edges = bands_for(sweep[1], sweep[2])
        rows, fell = model_rows(sweep, v, up)
        print(f"\n{name} (balance point {up:.2f} deg, sweep {sweep}, stick {v:+.2f} m/s)"
              f"{'  -- MODEL FELL' if fell else ''}")
        rb, mb = bands(robot, edges), bands(from_rows(rows), edges)
        for q in ("pitch", "wheel", "reply"):
            print(f"  {q}/inj    band     robot             model             ratio  coh robot/model")
            for (f, hr, cr), (_, hm, cm) in zip(rb[q], mb[q]):
                print(f"            {f:4.1f} Hz  {abs(hr):7.4f} {np.degrees(np.angle(hr)):+5.0f} deg   "
                      f"{abs(hm):7.4f} {np.degrees(np.angle(hm)):+5.0f} deg   {abs(hr) / abs(hm):5.2f}  "
                      f"{cr:.2f}/{cm:.2f}")


if __name__ == "__main__":
    main()
