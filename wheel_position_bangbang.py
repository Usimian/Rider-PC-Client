#!/usr/bin/env python3
"""Tier 2 (bang-bang): hold left wheel (ID 11) at start position by driving a
FIXED torque toward the setpoint until error is within deadband.

Overcomes the stiction steady-state error that proportional control leaves
(the wheel stalled ~62 ticks off target because Kp*err fell below friction).

  err = target - odom
  err >  deadband -> torque = +TORQUE   (positive torque raises position)
  err < -deadband -> torque = -TORQUE
  |err| <= deadband -> torque = 0

Requires passthrough firmware + Pi process killed. Robot on stand, wheels free.
Push the wheel by hand; it should drive back to the setpoint. Ctrl-C to stop.

Usage:
  ~/.xgo-cal/bin/python wheel_position_bangbang.py [TORQUE] [deadband]
  defaults: TORQUE=45 deadband=5
"""
import serial, time, sys

LEFT, RIGHT = 11, 21
WRAP, WRAP_THRESH = 1024, 800
TORQUE   = int(sys.argv[1]) if len(sys.argv) > 1 else 45
DEADBAND = int(sys.argv[2]) if len(sys.argv) > 2 else 5

def sync_torque(tl, tr):
    buf = [0xFF, 0xFF, 0xFE, 4 + 5*2, 0x83, 0x1E, 0x04]
    for sid, tor in [(LEFT, tl), (RIGHT, tr)]:
        t = int(tor) & 0xFFFF
        buf += [sid, 0x00, 0x00, t & 0xFF, (t >> 8) & 0xFF]
    buf.append((~sum(buf[2:])) & 0xFF)
    return bytes(buf)

def read_req(sid):
    ck = (~(sid + 0x04 + 0x02 + 0x24 + 0x06)) & 0xFF
    return bytes([0xFF, 0xFF, sid, 0x04, 0x02, 0x24, 0x06, ck])

def parse(resp, sid):
    i = resp.find(b'\xff\xff')
    if i < 0: return None
    f = resp[i:]
    if len(f) < 5 or f[2] != sid: return None
    LEN = f[3]
    if len(f) <= LEN: return None
    pos = f[LEN-3] | (f[LEN-2] << 8)
    vel = f[LEN-1] | (f[LEN] << 8)
    if vel >= 0x8000: vel -= 0x10000
    return pos, vel

def main():
    s = serial.Serial('/dev/ttyUSB0', 1000000, timeout=0.05)
    last_pos, odom = None, 0
    for _ in range(5):
        s.reset_input_buffer(); s.write(read_req(LEFT)); time.sleep(0.008)
        r = parse(s.read(32), LEFT)
        if r: last_pos = r[0]
        time.sleep(0.01)
    if last_pos is None:
        print("Could not read wheel. Abort."); s.close(); return

    print(f"Bang-bang hold, TORQUE=±{TORQUE}, deadband=±{DEADBAND}. Ctrl-C to stop.")
    print(f"{'odom':>7} {'err':>6} {'vel':>5} {'torque':>7}")
    try:
        while True:
            s.reset_input_buffer(); s.write(read_req(LEFT)); time.sleep(0.006)
            r = parse(s.read(32), LEFT)
            if r:
                pos, vel = r
                d = pos - last_pos
                if d < -WRAP_THRESH: d += WRAP
                elif d > WRAP_THRESH: d -= WRAP
                odom += d
                last_pos = pos

                err = 0 - odom
                if err > DEADBAND:    tor = +TORQUE
                elif err < -DEADBAND: tor = -TORQUE
                else:                 tor = 0
                s.write(sync_torque(tor, 0))
                print(f"{odom:7d} {err:6d} {vel:5d} {tor:7d}")
            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        for _ in range(5):
            s.write(sync_torque(0, 0)); time.sleep(0.01)
        s.close()
        print("Torque zeroed.")

if __name__ == "__main__":
    main()
