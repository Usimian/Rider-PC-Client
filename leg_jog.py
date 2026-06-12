#!/usr/bin/env python3
"""Interactive leg-servo jog tool (default right leg, ID 22).

Type a target encoder value; the leg gently ramps to it (reg 0x06) and holds
(torque on). Shows where it actually ended up so you can see sticking.
  - number  -> move to that value and hold
  - p       -> just print present position
  - q / Ctrl-C -> release torque and quit

Requires passthrough firmware + Pi off (free bus).
Usage:  ~/.xgo-cal/bin/python leg_jog.py [ID]   (ID default 22; use 12 for left)
"""
from scservo_sdk import PortHandler, PacketHandler
import sys, time

ID = int(sys.argv[1]) if len(sys.argv) > 1 else 22
RAMP_STEP = 8       # counts per tick (lower = gentler)
RAMP_DT   = 0.05    # seconds per tick

port = PortHandler('/dev/ttyUSB0'); port.openPort(); port.setBaudRate(1000000)
pkt = PacketHandler(0)

def pos():
    v, c, _ = pkt.read2ByteTxRx(port, ID, 0x24)
    return v if c == 0 else None

def set_torque(on):
    pkt.write1ByteTxRx(port, ID, 0x18, 1 if on else 0)

def goto(target):
    set_torque(True)
    cur = pos()
    if cur is None:
        print("  read failed"); return
    g = cur
    while g != target:
        g = min(target, g + RAMP_STEP) if target > g else max(target, g - RAMP_STEP)
        pkt.write2ByteTxRx(port, ID, 0x06, g)
        time.sleep(RAMP_DT)
    time.sleep(0.25)
    p = pos()
    lag = target - p if p is not None else 0
    note = "  (OK, holding)" if abs(lag) <= 5 else f"  *** didn't reach — stuck/limit (off by {lag:+d}) ***"
    print(f"  commanded {target}, now at {p}{note}")

try:
    p = pos()
    print(f"Leg ID {ID} at pos={p}.  Enter target value, 'p' to read, 'q' to quit.")
    while True:
        s = input("target> ").strip().lower()
        if s in ("q", "quit", "exit", ""):
            break
        if s == "p":
            print(f"  pos={pos()}"); continue
        try:
            t = int(s)
        except ValueError:
            print("  enter a number, 'p', or 'q'"); continue
        goto(t)
finally:
    set_torque(False)
    port.closePort()
    print("torque released. bye")
