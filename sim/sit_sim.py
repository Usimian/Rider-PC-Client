"""Sitting down (2026-11-14): from balancing upright, tip the body back and cut the wheels so it settles onto the
knee stub -- always the same resting pose, the one getup_sim.py / main.cpp's get-up starts from (Marc's idea).

Two ways to tip it back, then the wheels are cut (torque off: u 0, the motor's friction and damping stay) once the
firmware's own pitch reads CUT degrees back from upright:
  lean  -- the balance loop stays in charge: its upright reference (gLqrUp; closed_loop.FLASHED_UP_DEG, which the
           sim's firmware reads every cycle exactly as main.cpp reads gLqrUp) is ramped back at RATE deg/s; the LQR
           rolls the wheels forward under the body to tip it back;
  pulse -- open loop: the wheels driven forward at U until the cut.
Measured: the body's rate when the stub first touches (the impact), the largest force on the stub, where it ends
(back on the stub = sat; forward on the nose, or still up = not), how far it rolled. The plant is getup_sim's (the
stub and nose contacts, the measured motor), started upright; legs rigid (at the cut the real ones go limp and swing
to where they hang -- the stub's place in the sim is that hanging place, measured resting).

    ./.venv/bin/python sit_sim.py                 # methods x cut angles, nominal robot
    ./.venv/bin/python sit_sim.py --robust 16     # the same over randomized robots (getup_sim.RANGES)
"""
import argparse

import mujoco
import numpy as np

import getup_sim as G
from getup_sim import CL

T_STAND, T_AFTER = 1.0, 2.5


class SitPlant(G.GetupPlant):
    """getup_sim's plant started upright; records each physics step's body angle and rate and the force on the stub."""

    def __init__(self, var=None):
        super().__init__(None, var)
        self.stub = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "back_contact")
        self.nose = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "forward_contact")
        self.log = []                                                # (angle deg + fwd, rate deg/s, stub N, nose N)
        self._f = np.zeros(6)

    def _force(self, geom):
        n = 0.0
        for i in range(self.d.ncon):
            c = self.d.contact[i]
            if geom in (c.geom1, c.geom2):
                mujoco.mj_contactForce(self.model, self.d, i, self._f)
                n += abs(self._f[0])
        return n

    def step(self, u, dt):
        super().step(u, dt)
        self.log.append((np.degrees(self.d.qpos[self.qp]), np.degrees(self.d.qvel[self.dp]),
                         self._force(self.stub), self._force(self.nose)))


class SitFirmware:
    """The flashed firmware, balancing; after T_STAND it tips the body back (method) and cuts the wheels at `cut`."""

    def __init__(self, fw, method, amount, cut):
        self.fw, self.method, self.amount, self.cut = fw, method, amount, cut
        self.t, self.off, self.phase, self.t_cut = 0.0, 0.0, "stand", None

    def __getattr__(self, name):
        return getattr(self.fw, name)

    def control(self, dt, stick=0.0):
        self.t += dt
        q = self.fw.theta2 - CL.FLASHED_UP_DEG if self.fw.theta2 is not None else 0.0   # + = back, from upright
        if self.phase == "stand" and self.t >= T_STAND:
            self.phase = "tip"
        if self.phase == "tip" and q >= self.cut:
            self.phase, self.t_cut = "cut", self.t
        if self.phase == "tip" and self.method == "lean":
            self.off += self.amount * dt
        up = CL.FLASHED_UP_DEG
        CL.FLASHED_UP_DEG = up + self.off                            # main.cpp: gLqrUp, ramped
        try:
            u = self.fw.control(dt, stick)
        finally:
            CL.FLASHED_UP_DEG = up
        if self.phase == "cut":
            return 0.0
        if self.phase == "tip" and self.method == "pulse":
            return self.amount                                       # wheels forward: the body tips back
        return u


def trial(method, amount, cut, var=None, seed=0):
    plant = SitPlant(var)
    sf = SitFirmware(CL.flashed(rng=np.random.default_rng(seed)), method, amount, cut)
    L, fell = CL.run(fw=sf, plant=plant, T=T_STAND + 1.5 + T_AFTER, log_all=True, seed=seed)
    L = np.array(L)
    t, v = L[:, 0], L[:, 3]
    x = np.cumsum(v * np.gradient(t))
    P = np.array(plant.log)                                          # every physics step (closed_loop.SUB_S)
    ang, rate, f_stub, f_nose = P[:, 0], P[:, 1], P[:, 2], P[:, 3]
    r = dict(method=method, amount=amount, cut=cut, t_cut=sf.t_cut, var=var)
    if sf.t_cut is None:
        r["end"] = "never tipped to the cut"
        return r
    i0 = int(round(T_STAND / CL.SUB_S))
    touch = np.nonzero(f_stub[i0:] > 0.05)[0]
    nose = np.nonzero(f_nose[i0:] > 0.05)[0]
    end = ang[-int(0.3 / CL.SUB_S):].mean()
    if touch.size:
        k = i0 + touch[0]
        r["impact_dps"] = -rate[k]                                   # backward rotation, deg/s
        r["t_land"] = k * CL.SUB_S - sf.t_cut
        r["stub_peak_N"] = f_stub[k:].max()
    r["nose"] = bool(nose.size)
    r["end_deg"] = end
    r["end"] = ("sat" if end < -20 and abs(end - G.body_angle(G.POSES["back"][0]) * 180 / np.pi) < 3 and not nose.size
                else "on the nose" if end > 20 else "upright?" if abs(end) < 10 else f"at {end:+.0f}")
    r["roll_mm"] = 1000 * np.abs(x[t >= T_STAND] - x[t < T_STAND][-1]).max()
    r["fell_45"] = fell
    return r


def show(r):
    m = f"{r['method']:5s} {r['amount']:4.0f}{' deg/s' if r['method'] == 'lean' else ' cmd  '} cut {r['cut']:4.1f}: "
    if "impact_dps" not in r:
        return m + r["end"]
    return m + (f"{r['end']:10s} stub hit at {r['impact_dps']:4.0f} deg/s, {r['t_land']*1000:4.0f} ms after the cut, "
                f"peak {r['stub_peak_N']:5.1f} N; rolled {r['roll_mm']:3.0f} mm" + ("  (touched the nose)" if r["nose"] else ""))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robust", type=int, metavar="N")
    a = ap.parse_args()
    cases = [("lean", 20.0), ("lean", 40.0), ("pulse", 150.0), ("pulse", 250.0)]
    cuts = (4.0, 8.0, 12.0, 16.0)
    if not a.robust:
        for method, amount in cases:
            for cut in cuts:
                print(show(trial(method, amount, cut)), flush=True)
        return
    G.RANGES["com_dx"] = (-0.004, 0.004)                             # as the get-up's robust run (Marc's hand start)
    for method, amount in cases:
        for cut in cuts:
            rs = [trial(method, amount, cut, G.draw(np.random.default_rng(2000 + i)), seed=2000 + i) for i in range(a.robust)]
            sat = [r for r in rs if r["end"] == "sat"]
            hit = [r["impact_dps"] for r in rs if "impact_dps" in r]
            n_ = [r["stub_peak_N"] for r in rs if "stub_peak_N" in r]
            print(f"{method:5s} {amount:4.0f} cut {cut:4.1f}: sat {len(sat)}/{len(rs)}; impact deg/s median/max "
                  f"{np.median(hit):4.0f}/{np.max(hit):4.0f}; stub peak N median/max {np.median(n_):4.1f}/{np.max(n_):4.1f}; "
                  f"others: {sorted(set(r['end'] for r in rs if r['end'] != 'sat'))}", flush=True)


if __name__ == "__main__":
    main()
