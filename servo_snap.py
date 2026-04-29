#!/usr/bin/env python3
"""On-demand servo snapshot. Press Enter to read; q+Enter to quit."""
from scservo_sdk import PortHandler, PacketHandler

port = PortHandler("/dev/ttyUSB0")
port.openPort()
port.setBaudRate(1000000)
pkt = PacketHandler(0)

i = 0
while True:
    cmd = input(f"\n[{i}] Position legs, then Enter (q=quit): ")
    if cmd.strip().lower() == "q":
        break
    for sid in [12, 22]:
        pos, _, _ = pkt.read2ByteTxRx(port, sid, 0x24)
        offs, _, _ = pkt.read2ByteTxRx(port, sid, 0x1F)
        offs_s = offs - 0x10000 if offs > 0x7FFF else offs
        pos_s = pos - 0x10000 if pos > 0x7FFF else pos
        print(f"  ID {sid}: pos={pos_s:6d} (raw {pos})   correction={offs_s:6d} (raw 0x{offs:04X})")
    i += 1

port.closePort()
