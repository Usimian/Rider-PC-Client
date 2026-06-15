#!/usr/bin/env python3
"""Stand-only wheel characterization over USB-C (/dev/ttyUSB0).

Isolates the forward/backward drive asymmetry by driving the wheels OPEN-LOOP
(balance off) with a fixed torque step in each direction, on each wheel, and
measuring the resulting steady velocity. Uses the firmware's built-in 'stepcap'
(torque step + per-wheel velocity log) and 'lock' (select polled wheel).

  cmd>0  = forward drive (L:+cmd, R:-cmd, mirrored as in balance)
  cmd<0  = backward drive

Equal |cmd| both directions: if forward steady-speed < backward, forward has
more drag (or less torque). Compare wheel 11 vs 21 for a per-wheel imbalance.

REQUIRES: robot on a stand, WHEELS FREE OFF THE GROUND. Balance must be off.
Run:  /home/marc/.xgo-cal/bin/python tools/stand_wheel_test.py
"""
import serial, time, sys, statistics as st

PORT = "/dev/ttyUSB0"
WHEELS = [11, 21]                 # left, right servo IDs
LEVELS = [120, 200]              # torque magnitudes to test
ser = serial.Serial(PORT, 115200, timeout=0.2)


def send(line):
    ser.write((line + "\n").encode()); ser.flush()


def run_step(wheel, cmd, timeout=4.0):
    """Lock to `wheel`, fire a stepcap at `cmd`, collect the '# cap' block."""
    send("lock %d" % wheel)
    time.sleep(0.2)
    ser.reset_input_buffer()
    send("stepcap %d" % cmd)
    rows = []
    t0 = time.time()
    while time.time() - t0 < timeout:
        l = ser.readline().decode(errors="replace").strip()
        if l.startswith("# cap done"):
            break
        if l.startswith("# cap "):
            p = l.split()
            if len(p) == 6:
                try:
                    i, t_us, c, v = int(p[2]), int(p[3]), int(p[4]), int(p[5])
                    if v != -32768:
                        rows.append((i, t_us, c, v))
                except ValueError:
                    pass
    return rows


def summarize(rows):
    if not rows:
        return None
    base = [v for (i, t, c, v) in rows if i < 16]
    steady = [v for (i, t, c, v) in rows if i >= 100]
    b = st.mean(base) if base else 0.0
    s = st.mean(steady) if steady else 0.0
    return {"n": len(rows), "base": b, "steady": s, "abs": abs(s - b)}


def main():
    send("en 0"); time.sleep(0.3)
    print("=== stand wheel test  (wheels must be FREE) ===")
    print("%-6s %-5s %-4s %8s %8s %8s" % ("wheel", "dir", "cmd", "n", "steady", "|vel|"))
    results = {}
    for w in WHEELS:
        for mag in LEVELS:
            for cmd in (mag, -mag):
                r = run_step(w, cmd)
                s = summarize(r)
                tag = "fwd" if cmd > 0 else "rev"
                results[(w, mag, cmd)] = s
                if s:
                    print("%-6d %-5s %-4d %8d %8.1f %8.1f" % (w, tag, cmd, s["n"], s["steady"], s["abs"]))
                else:
                    print("%-6d %-5s %-4d   NO DATA" % (w, tag, cmd))
                time.sleep(0.6)   # let the wheel coast to rest between runs
    send("lock 0")               # restore alternating poll
    send("en 0")
    # asymmetry summary
    print("\n=== forward vs reverse |velocity| (same |torque|) ===")
    for w in WHEELS:
        for mag in LEVELS:
            f = results.get((w, mag, mag)); b = results.get((w, mag, -mag))
            if f and b and b["abs"] > 1e-3:
                ratio = f["abs"] / b["abs"]
                print("wheel %d @ %d:  fwd=%.1f  rev=%.1f  fwd/rev=%.2f%s"
                      % (w, mag, f["abs"], b["abs"], ratio,
                         "   <-- forward slower" if ratio < 0.9 else ""))


if __name__ == "__main__":
    main()
