#!/usr/bin/env python3
"""Try several candidate 'set zero' procedures on servo ID 22.

Each test:
 1. reads pos before
 2. performs candidate write(s)
 3. reads pos after
 4. reports if pos shifted toward 0

Hold the leg at the desired neutral position before running.
"""
from scservo_sdk import PortHandler, PacketHandler
import time

SID = 22
port = PortHandler('/dev/ttyUSB0')
port.openPort()
port.setBaudRate(1000000)
pkt = PacketHandler(0)

def get_pos():
    p, _, _ = pkt.read2ByteTxRx(port, SID, 0x24)
    return p - 0x10000 if p > 0x7FFF else p

def w1(addr, val):
    return pkt.write1ByteTxRx(port, SID, addr, val)

def w2(addr, val):
    v = val & 0xFFFF
    return pkt.write2ByteTxRx(port, SID, addr, v)

# Reset offset register to a known state
w2(0x1F, 0)
time.sleep(0.1)

input("Hold leg at desired neutral pose. Enter to start test:")
print(f"Initial pos: {get_pos()}\n")

tests = [
    ("Write +500 to 0x1F (offset reg, 2's complement)",
     lambda: w2(0x1F, 500)),
    ("Write +500 to 0x1F sign-magnitude (bit15=0)",
     lambda: w2(0x1F, 500)),
    ("Write -500 to 0x1F sign-magnitude (bit15=1, mag=500)",
     lambda: w2(0x1F, 0x8000 | 500)),
    ("Write 1 to 0x21 (XGO 'Reset zero' analog)",
     lambda: w1(0x21, 1)),
    ("Write 0x80 to 0x40 (some FT 'set zero' trigger)",
     lambda: w1(0x40, 0x80)),
    ("Toggle lock 0->1->0 after writing offset",
     lambda: (w2(0x1F, 0), w1(0x37, 1), w1(0x37, 0))),
    ("Write 1 to lock 0x37 (commit)",
     lambda: w1(0x37, 1)),
    ("Write 0xC8 magic to 0x37",
     lambda: w1(0x37, 0xC8)),
]

baseline = get_pos()
print(f"Baseline pos: {baseline}")
for name, action in tests:
    pre = get_pos()
    action()
    time.sleep(0.2)
    post = get_pos()
    delta = post - pre
    flag = "*** MAYBE ***" if abs(delta) > 50 else ""
    print(f"  {name}\n     pre={pre}  post={post}  delta={delta} {flag}")
    # Reset offset between tests
    w2(0x1F, 0)
    w1(0x37, 0)
    time.sleep(0.1)

port.closePort()
print("\nDone. If any test showed a large delta, that mechanism worked.")
