#!/usr/bin/env python3
"""Diagnose torque-off attempts on legs 12 and 22."""
from scservo_sdk import PortHandler, PacketHandler
import time

port = PortHandler('/dev/ttyUSB0'); port.openPort(); port.setBaudRate(1000000)
pkt = PacketHandler(0)

for sid in [12, 22]:
    print(f"--- ID {sid} ---")
    for reg, label in [(0x28, 'torque@0x28'), (0x37, 'lock@0x37'), (0x30, 'lock@0x30'), (0x18, 'torque@0x18')]:
        v, c, e = pkt.read1ByteTxRx(port, sid, reg)
        print(f"  read {label}: val={v} comm={c} err={e}")

    print(f"  WRITE 0 to 0x37 (STS lock):")
    c, e = pkt.write1ByteTxRx(port, sid, 0x37, 0)
    print(f"    comm={c} err={e}")
    print(f"  WRITE 0 to 0x30 (SCS lock):")
    c, e = pkt.write1ByteTxRx(port, sid, 0x30, 0)
    print(f"    comm={c} err={e}")
    time.sleep(0.05)

    print(f"  WRITE 0 to 0x28 (torque off):")
    c, e = pkt.write1ByteTxRx(port, sid, 0x28, 0)
    print(f"    comm={c} err={e}")
    time.sleep(0.1)

    v, _, _ = pkt.read1ByteTxRx(port, sid, 0x28)
    print(f"  read back 0x28: {v}")

port.closePort()
