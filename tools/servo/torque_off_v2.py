#!/usr/bin/env python3
"""Try torque-off via register 0x18 instead of 0x28."""
from scservo_sdk import PortHandler, PacketHandler
import time

port = PortHandler('/dev/ttyUSB0'); port.openPort(); port.setBaudRate(1000000)
pkt = PacketHandler(0)

for sid in [12, 22]:
    print(f"--- ID {sid} ---")
    v, _, _ = pkt.read1ByteTxRx(port, sid, 0x18)
    print(f"  before: 0x18={v}")
    c, e = pkt.write1ByteTxRx(port, sid, 0x18, 0)
    print(f"  WRITE 0 to 0x18: comm={c} err={e}")
    time.sleep(0.1)
    v, _, _ = pkt.read1ByteTxRx(port, sid, 0x18)
    print(f"  after:  0x18={v}")

print("\nTry to move the legs by hand now. Did torque release?")
port.closePort()
