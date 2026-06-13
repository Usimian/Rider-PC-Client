#!/usr/bin/env python3
"""LQR balance test over USB-C (/dev/ttyUSB0) for esp32_rider_fw (Milestone 5).

Sets the 4-state LQR gains, averages a level capture while held, enables, prints
a clear LET-GO cue, streams tilt + wheel position/velocity + torque, and ALWAYS
disables on Ctrl-C, a tilt/position runaway, or the time budget. Legs stay
torque-off (firmware-forced); this drives the WHEELS only.

  ~/.xgo-cal/bin/python lqr_balance.py [--k0 N --k1 N --k2 N --k3 N --umax N]
                                       [--capms MS --secs S]
"""
import serial, time, argparse

PORT, BAUD = "/dev/ttyUSB0", 115200
FALL_DEG = 22.0
FALL_POS = 0.40     # m of wheel runaway from home -> kill
FALL_HOLD = 0.3

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--k0", type=float, default=800.0)
    ap.add_argument("--k1", type=float, default=6.8)
    ap.add_argument("--k2", type=float, default=32.0)
    ap.add_argument("--k3", type=float, default=1.6)
    ap.add_argument("--umax", type=float, default=110.0)
    ap.add_argument("--capms", type=float, default=2000.0)
    ap.add_argument("--secs", type=float, default=20.0)
    a = ap.parse_args()

    s = serial.Serial(PORT, BAUD, timeout=0.2)
    time.sleep(0.3)

    def send(c):
        s.reset_input_buffer(); s.write((c + "\n").encode()); s.flush()
        time.sleep(0.15)
        for _ in range(30):
            ln = s.readline().decode(errors="replace").strip()
            if ln.startswith("#"):
                return ln
        return None

    send("get")
    print("k0 :", send(f"k0 {a.k0}"))
    print("k1 :", send(f"k1 {a.k1}"))
    print("k2 :", send(f"k2 {a.k2}"))
    print("k3 :", send(f"k3 {a.k3}"))
    print("umx:", send(f"umax {a.umax}"))

    # averaged level capture
    print(f">>> HOLD AT BALANCE POINT — capturing level for {a.capms/1000:.1f}s <<<")
    ths = []; s.reset_input_buffer(); t0 = time.time()
    while time.time() - t0 < a.capms / 1000.0:
        ln = s.readline().decode(errors="replace").strip()
        if ln.startswith("th="):
            d = {k: v for k, v in (t.split("=", 1) for t in ln.split() if "=" in t)}
            try: ths.append(float(d["th"]))
            except (KeyError, ValueError): pass
    if not ths:
        print("no telemetry — abort"); s.close(); return
    mean = sum(ths) / len(ths); spread = max(ths) - min(ths)
    print(f"  captured tilt = {mean:+.2f} deg (spread {spread:.2f}, {len(ths)} samples)")
    print("set:", send(f"set {mean:.2f}"))
    print("home:", send("home"))     # snapshot wheel home here

    over = None
    try:
        print(">>> ENABLING (Ctrl-C to stop) <<<")
        print("en :", send("en 1"))
        print("\n*** LET GO GENTLY NOW — hand ready to catch ***\n")
        t0 = time.time()
        while time.time() - t0 < a.secs:
            ln = s.readline().decode(errors="replace").strip()
            if not ln.startswith("th="):
                continue
            d = {k: v for k, v in (t.split("=", 1) for t in ln.split() if "=" in t)}
            th = float(d.get("th", 0)); st = float(d.get("set", 0))
            wx = float(d.get("wx", 0)); wv = float(d.get("wv", 0)); u = d.get("u", "?")
            print(f"  t={time.time()-t0:4.1f} tilt={th:+6.2f} err={th-st:+5.2f} wx={wx:+.3f} wv={wv:+6.1f} u={u:>5}")
            if abs(th - st) > FALL_DEG or abs(wx) > FALL_POS:
                over = over or time.time()
                if time.time() - over > FALL_HOLD:
                    print("!!! runaway (tilt/pos) — DISABLING"); break
            else:
                over = None
    except KeyboardInterrupt:
        print("\n^C")
    finally:
        print("disable:", send("d"))
        s.close()
        print("done — torque off, balance disabled.")

if __name__ == "__main__":
    main()
