#!/usr/bin/env python3
"""Read-only status of leg servos — no writes. Tells us torque/lock/goal state."""
from scservo_sdk import PortHandler, PacketHandler

port = PortHandler("/dev/ttyUSB0")
port.openPort()
port.setBaudRate(1000000)
pkt = PacketHandler(0)

for sid in [12, 22]:
    pos, _, _    = pkt.read2ByteTxRx(port, sid, 0x24)
    goal, _, _   = pkt.read2ByteTxRx(port, sid, 0x2A)
    torque, _, _ = pkt.read1ByteTxRx(port, sid, 0x28)
    lock, _, _   = pkt.read1ByteTxRx(port, sid, 0x37)
    offset, _, _ = pkt.read2ByteTxRx(port, sid, 0x1F)
    pos_s = pos - 0x10000 if pos > 0x7FFF else pos
    print(f"ID {sid}:  pos@0x24={pos_s:6d}   goal=0x{goal:04X}   "
          f"torque={torque}  lock={lock}   offset_raw=0x{offset:04X}")

port.closePort()
