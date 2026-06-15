#!/usr/bin/env python3
"""Read-only encoder monitor + CSV logger for the leg servos (IDs 12 & 22).

Continuously prints present position (reg 0x24) for the given legs and logs
every sample to a timestamped CSV. Does NOT touch torque — purely passive, so
you can move the legs by hand (torque off) or watch them hold. Ctrl-C to quit.

Requires passthrough firmware + Pi off (free bus).
Usage:  ~/.xgo-cal/bin/python leg_read.py [ID ...]   (default: 12 22)
CSV is written to leg_log_<YYYYmmdd_HHMMSS>.csv in the current directory.
"""
from scservo_sdk import PortHandler, PacketHandler
import sys, time, csv, datetime

IDS = [int(a) for a in sys.argv[1:]] or [12, 22]

port = PortHandler('/dev/ttyUSB0'); port.openPort(); port.setBaudRate(1000000)
pkt = PacketHandler(0)

def read(id):
    v, c, _ = pkt.read2ByteTxRx(port, id, 0x24)
    t, ct, _ = pkt.read1ByteTxRx(port, id, 0x18)
    return (v if c == 0 else None, t if ct == 0 else None)

fname = "leg_log_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S") + ".csv"
f = open(fname, "w", newline="")
w = csv.writer(f)
header = ["t_s"] + sum(([f"pos_{i}", f"tq_{i}"] for i in IDS), [])
w.writerow(header)

print(f"Logging to {fname}   (Ctrl-C to quit)")
print("  " + "   ".join(f"{'ID'+str(i):>12}" for i in IDS))
t0 = time.time()
try:
    while True:
        t = time.time() - t0
        row = [f"{t:.2f}"]; cells = []
        for i in IDS:
            p, tq = read(i)
            row += [p if p is not None else "", tq if tq is not None else ""]
            ps = f"{p}" if p is not None else "ERR"
            cells.append(f"{ps:>6}(tq{tq})")
        w.writerow(row); f.flush()
        print("  " + "   ".join(f"{c:>12}" for c in cells))
        time.sleep(0.1)
except KeyboardInterrupt:
    print("\nbye")
finally:
    f.close()
    port.closePort()
    print(f"saved {fname}")
