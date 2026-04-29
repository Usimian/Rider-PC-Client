#!/usr/bin/env python3
"""Continuously print every byte 0x00-0x7F of servos 12 and 22 so you can
watch what changes when you move the legs. Ctrl-C to stop.
"""
from scservo_sdk import PortHandler, PacketHandler
import time, sys

port = PortHandler("/dev/ttyUSB0")
port.openPort()
port.setBaudRate(1000000)
pkt = PacketHandler(0)

while True:
    sys.stdout.write("\033[2J\033[H")  # clear screen
    for sid in [12, 22]:
        print(f"=== ID {sid} ===")
        row = ""
        for addr in range(0x00, 0x80):
            v, c, e = pkt.read1ByteTxRx(port, sid, addr)
            if addr % 16 == 0:
                row += f"\n  0x{addr:02X}: "
            row += f"{v:3d} "
        print(row)
    sys.stdout.flush()
    time.sleep(0.3)
