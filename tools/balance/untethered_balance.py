#!/usr/bin/env python3
"""Untethered balance run, driven entirely from the Raspberry Pi over UART1.

No USB-C needed. Enables the balance loop on the ESP32, streams live telemetry,
and ALWAYS disables (torque -> 0) on exit: Ctrl-C, a tip past the fall limit, or
the time budget. The leg servos (12/22) are never touched — the firmware forces
them torque-off; this only commands the WHEELS via the balance loop.

  ~/xgovenv/bin/python untethered_balance.py [--cap] [--secs N]
    --cap     capture the current held tilt as the setpoint before enabling
    --secs N  auto-disable after N seconds (default 30)

Hard kill any time: Ctrl-C. Auto-kill if |tilt-set| exceeds the firmware fall
limit for ~0.3 s (belt-and-suspenders; the firmware also zeroes torque itself).
"""
import serial, time, sys, argparse

PORT, BAUD = "/dev/ttyAMA0", 115200
FALL_DEG = 22.0          # local safety; firmware also cuts at ~25
FALL_HOLD = 0.3          # seconds past the limit before we force-disable

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cap", action="store_true", help="capture current tilt as setpoint")
    ap.add_argument("--secs", type=float, default=30.0, help="auto-disable after N s")
    ap.add_argument("--kppos", type=float, default=None, help="cascade position P")
    ap.add_argument("--kpvel", type=float, default=None, help="cascade velocity P")
    ap.add_argument("--kivel", type=float, default=None, help="cascade velocity I")
    ap.add_argument("--kppit", type=float, default=None, help="cascade pitch P")
    ap.add_argument("--kdpit", type=float, default=None, help="pitch-rate damping")
    ap.add_argument("--izero", type=float, default=None, help="base lean angle (deg)")
    ap.add_argument("--kp", type=float, default=None, help="alias for --kppit")
    ap.add_argument("--kd", type=float, default=None, help="alias for --kdpit")
    ap.add_argument("--dither", type=float, default=None, help="stiction dither amplitude (0=off)")
    ap.add_argument("--ff", type=float, default=None, help="friction feed-forward magnitude")
    ap.add_argument("--ffband", type=float, default=None, help="friction FF deadband")
    ap.add_argument("--umax", type=float, default=None, help="set torque clamp before enabling")
    ap.add_argument("--capms", type=float, default=2500.0, help="capture-average window (ms)")
    ap.add_argument("--gcal", action="store_true", help="re-zero gyro bias at start (hold still)")
    ap.add_argument("--fall", type=float, default=None,
                    help="tilt-error cutoff (deg), firmware + local kill")
    ap.add_argument("--set", type=float, default=None, dest="setpt",
                    help="fixed tilt setpoint (deg); skips --cap")
    a = ap.parse_args()

    s = serial.Serial(PORT, BAUD, timeout=0.2)
    time.sleep(0.3)

    def send(c):
        s.reset_input_buffer(); s.write((c + "\n").encode()); s.flush()
        time.sleep(0.15)
        for _ in range(20):
            ln = s.readline().decode(errors="replace").strip()
            if ln.startswith("#"):
                return ln
        return None

    send("get")  # throwaway: absorbs any first-byte power-up transient
    print("state:", send("get"))

    if a.gcal:
        print(">>> HOLD STILL — re-zeroing gyro (~1.2s) <<<")
        s.write(b"gcal\n"); s.flush(); time.sleep(1.4)
        print("gyro:", send("get"))

    for name, val in (("kppos", a.kppos), ("kpvel", a.kpvel), ("kivel", a.kivel),
                      ("kppit", a.kppit if a.kppit is not None else a.kp),
                      ("kdpit", a.kdpit if a.kdpit is not None else a.kd),
                      ("izero", a.izero), ("dither", a.dither),
                      ("ff", a.ff), ("ffband", a.ffband),
                      ("fall", a.fall), ("umax", a.umax)):
        if val is not None:
            print(f"{name} {val}:", send(f"{name} {val}"))
    fall_deg = a.fall + 5 if a.fall is not None else FALL_DEG  # local kill just past firmware's

    if a.setpt is not None:
        print(f"set {a.setpt}:", send(f"set {a.setpt}"))
    elif a.cap:
        # Average tilt (and gyro rate) over a window while held steady at the
        # balance point, then set that mean as the setpoint. Reports spread so
        # we can tell the hold was steady and shows the gyro bias.
        win = a.capms / 1000.0
        print(f">>> HOLD AT BALANCE POINT — capturing level for {win:.1f}s <<<")
        ths, rates = [], []
        s.reset_input_buffer(); t0 = time.time()
        while time.time() - t0 < win:
            ln = s.readline().decode(errors="replace").strip()
            if not ln.startswith("th="):
                continue
            d = {k: v for k, v in (tok.split("=", 1) for tok in ln.split() if "=" in tok)}
            try:
                ths.append(float(d["th"])); rates.append(float(d.get("rate", 0)))
            except (KeyError, ValueError):
                continue
        if not ths:
            print("!!! no telemetry during capture — aborting"); s.close(); return
        mean = sum(ths) / len(ths)
        spread = max(ths) - min(ths)
        bias = sum(rates) / len(rates)
        print(f"  captured tilt = {mean:+.2f} deg  (spread {spread:.2f} over {len(ths)} samples)")
        print(f"  gyro bias at rest = {bias:+.2f} deg/s  (ideal ~0; uncalibrated offset)")
        if spread > 1.0:
            print("  ! hold wasn't steady (spread > 1 deg) — consider redoing capture")
        print(f"set {mean:.2f}:", send(f"set {mean:.2f}"))

    over_since = None
    try:
        print(">>> ENABLING balance (Ctrl-C to stop) <<<")
        print("en 1 :", send("en 1"))
        print("\n*** LET GO GENTLY NOW — keep a hand ready to catch ***\n")
        t0 = time.time()
        while time.time() - t0 < a.secs:
            ln = s.readline().decode(errors="replace").strip()
            if not ln.startswith("th="):
                continue
            d = {k: v for k, v in (tok.split("=", 1) for tok in ln.split() if "=" in tok)}
            th = float(d.get("th", 0)); st = float(d.get("set", 0)); u = d.get("u", "?")
            wx = d.get("wx", "?"); wv = d.get("wv", "?")
            print(f"  t={time.time()-t0:4.1f}  tilt={th:+6.2f}  err={th-st:+6.2f}  wx={wx:>7}  wv={wv:>6}  u={u:>5}")
            if abs(th - st) > fall_deg:
                over_since = over_since or time.time()
                if time.time() - over_since > FALL_HOLD:
                    print("!!! past fall limit — DISABLING"); break
            else:
                over_since = None
    except KeyboardInterrupt:
        print("\n^C")
    finally:
        print("disable:", send("d"))
        s.close()
        print("done — torque off, balance disabled.")

if __name__ == "__main__":
    main()
