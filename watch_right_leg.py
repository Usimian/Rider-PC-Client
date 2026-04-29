#!/usr/bin/env python3
"""Disable torque on right leg (ID 22) and print live encoder readings.
Move the leg slowly through its full mechanical range. Watch for 0<->4095 wrap.
Ctrl-C to exit."""
from scservo_sdk import PortHandler, PacketHandler
import time

port = PortHandler('/dev/ttyUSB0'); port.openPort(); port.setBaudRate(1000000)
pkt = PacketHandler(0)

print("Disabling torque on ID 22 (right leg) — you can now move it by hand.")
pkt.write1ByteTxRx(port, 22, 0x28, 0)
time.sleep(0.1)

print("Move the leg slowly through its FULL range. Ctrl-C to stop.")
print("Watch for the value crossing near 0 or 4095, especially jumping between them.\n")

try:
    last = None
    while True:
        pos, comm, _ = pkt.read2ByteTxRx(port, 22, 0x24)
        if comm == 0:
            if last is None or abs(pos - last) >= 3 or last in (0,4095) or pos in (0,4095):
                print(f"  encoder = {pos:4d}  (0x{pos:04X})")
                last = pos
        time.sleep(0.05)
except KeyboardInterrupt:
    print("\nDone.")
finally:
    port.closePort()
