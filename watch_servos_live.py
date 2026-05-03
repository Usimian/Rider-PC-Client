#!/usr/bin/env python3
"""Disable torque on leg servos (12, 22) and print live encoder positions.
Requires passthrough firmware on the ESP32. Ctrl-C to exit (re-enables torque)."""
from scservo_sdk import PortHandler, PacketHandler
import time, sys

IDS = [12, 22]

port = PortHandler('/dev/ttyUSB0')
port.openPort()
port.setBaudRate(1000000)
pkt = PacketHandler(0)

for sid in IDS:
    pkt.write1ByteTxRx(port, sid, 0x18, 0)  # torque off (reg 0x18 on these servos)
time.sleep(0.5)
for sid in IDS:
    t, _, _ = pkt.read1ByteTxRx(port, sid, 0x18)
    print(f"  ID {sid}: torque = {t}")

print(f"Torque off on {IDS}. Move legs by hand. Ctrl-C to exit.\n")
print(f"{'t':>6}  " + "  ".join(f"ID{sid} pos".rjust(10) for sid in IDS))

t0 = time.time()
try:
    while True:
        cells = []
        for sid in IDS:
            pos, c, _ = pkt.read2ByteTxRx(port, sid, 0x24)
            if c == 0:
                pos_s = pos - 0x10000 if pos > 0x7FFF else pos
                cells.append(f"{pos_s:6d}")
            else:
                cells.append("   ---")
        print(f"{time.time()-t0:6.1f}    " + "      ".join(cells))
        sys.stdout.flush()
        time.sleep(0.1)
except KeyboardInterrupt:
    print("\nRe-enabling torque...")
finally:
    for sid in IDS:
        pkt.write1ByteTxRx(port, sid, 0x18, 1)
    port.closePort()
    print("Done.")
