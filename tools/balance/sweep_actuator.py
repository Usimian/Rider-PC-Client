#!/usr/bin/env python3
"""Sweep the wheel command ladder via the firmware 'stepcap' and SAVE the raw
rows -- the static command->speed map that the sim ActuatorModel must reproduce.

Unlike measure_actuator.py (one step, console only), this walks a fine ladder
THROUGH the stiction region (sub-breakaway, breakaway, linear) so we can validate
the stick-slip model: where the wheel breaks away and the min-creep jump.

Run on the workstation over USB-C (robot ON, balance off):
    FREE   (wheels off the ground, on the stand):
        /home/marc/.xgo-cal/bin/python tools/balance/sweep_actuator.py
    LOADED (robot on the floor, held upright, wheels bearing weight):
        /home/marc/.xgo-cal/bin/python tools/balance/sweep_actuator.py loaded

Saves: sim/actuator_bench_lqr[_loaded].csv  (cmd,i,t_us,vel)  -- raw, every sample.
The wheels spin briefly at each rung.
"""
import sys, time, serial

PORT = "/dev/ttyUSB0"
TAG = sys.argv[1] if len(sys.argv) > 1 else ""
OUT = "/home/marc/Rider-PC-Client/sim/actuator_bench_lqr%s.csv" % (("_" + TAG) if TAG else "")
# NB: never send cmd=0 -- the firmware's 'stepcap 0' does NOT reset the stored step,
# so it re-fires the previous (possibly full-speed) command. Always start nonzero & low.
if TAG == "loaded":
    # LOADED (robot on floor, held): breakaway only -> LOW commands so it can't run away.
    # The high-command linear region is already validated free-wheel; load shifts the
    # breakaway, not the slope. Fine steps through the expected loaded breakaway.
    LADDER = [15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 70, 80,
              -20, -35, -50, -70]
else:
    # FREE (wheels off the ground): full range incl. the linear region + negatives.
    LADDER = [15, 25, 35, 40, 43, 46, 50, 55, 65, 80, 100, 130, 170, 200, 233,
              -50, -120, -200]

s = serial.Serial(PORT, 115200, timeout=2.0)
time.sleep(0.4)
s.write(b"en 0\n"); time.sleep(0.3)        # balance MUST be off for stepcap
s.write(b"lock 11\n"); time.sleep(0.2)     # poll only the left wheel -> full-rate sampling
s.reset_input_buffer()


def capture(cmd):
    s.reset_input_buffer()
    s.write(("stepcap %d\n" % cmd).encode()); s.flush()
    rows, t0 = [], time.time()
    while time.time() - t0 < 6.0:
        line = s.readline().decode(errors="replace").strip()
        if line.startswith("# cap done"):
            break
        if line.startswith("# cap "):
            p = line.split()
            if len(p) == 6:
                try:
                    i, t_us, c, v = int(p[2]), int(p[3]), int(p[4]), int(p[5])
                    if v != -32768:
                        rows.append((i, t_us, c, v))
                except ValueError:
                    pass
    return rows


def main():
    allrows = []
    print("%5s %6s %8s %8s" % ("cmd", "n", "steady", "|steady|"))
    for cmd in LADDER:
        rows = capture(cmd)
        if not rows:
            print("%5d   NO DATA" % cmd)
            continue
        for (i, t_us, c, v) in rows:
            allrows.append((cmd, i, t_us, v))
        tail = [v for (i, t_us, c, v) in rows if i >= 80]
        steady = sum(tail) / len(tail) if tail else 0.0
        print("%5d %6d %8.1f %8.1f" % (cmd, len(rows), steady, abs(steady)))
        time.sleep(0.6)                    # let the wheel coast to rest between rungs
    s.write(b"lock 0\n"); s.write(b"en 0\n")
    s.close()
    with open(OUT, "w") as f:
        f.write("cmd,i,t_us,vel\n")
        for r in allrows:
            f.write("%d,%d,%d,%d\n" % r)
    print("\nsaved %d raw samples -> %s" % (len(allrows), OUT))


if __name__ == "__main__":
    main()
