#!/usr/bin/env python3
"""Read-only status of the leg servos (K-power RC08P, IDs 12 & 22) — no writes.

Reads only the hardware-verified registers (see docs/servo_registers.md):
  0x24 present position | 0x1E goal | 0x18 torque (1=on/0=limp) | 0x06/0x08 min/max angle limit
Requires passthrough firmware. Legs only — wheels (IDs 11/21) use a different raw
read protocol (see tools/wheel/wheel_encoder_monitor.py).
"""
from scservo_sdk import PortHandler, PacketHandler

port = PortHandler("/dev/ttyUSB0")
port.openPort()
port.setBaudRate(1000000)
pkt = PacketHandler(0)

def s16(v): return v - 0x10000 if v > 0x7FFF else v
def r2(sid, reg): return s16(pkt.read2ByteTxRx(port, sid, reg)[0])
def r1(sid, reg): return pkt.read1ByteTxRx(port, sid, reg)[0]

for sid in (12, 22):
    pos = r2(sid, 0x24)
    goal = r2(sid, 0x1E)
    tq = r1(sid, 0x18)
    mn = r2(sid, 0x06)
    mx = r2(sid, 0x08)
    print(f"ID {sid}:  pos@0x24={pos:5d}  goal@0x1E={goal:5d}  "
          f"torque@0x18={tq} ({'ON' if tq else 'limp'})  limits@0x06/0x08={mn}..{mx}")

port.closePort()
