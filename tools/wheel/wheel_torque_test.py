#!/usr/bin/env python3
"""Tier 1: command a small torque to ONE wheel and watch the encoder respond.

Replicates RIG-Omni sendWheelTor(): SYNC_WRITE (0x83) to reg 0x1E, 4 bytes
per servo = [pos_lo, pos_hi, tor_lo, tor_hi], pos forced to 0.

Requires passthrough firmware + Pi powered down. Robot on its back, wheels
clear. Commands torque for a short burst then forces zero.

Usage:
  ~/.xgo-cal/bin/python wheel_torque_test.py [torque] [duration_s]
  torque default 40 (small), duration default 0.4
"""
import serial, time, sys

LEFT, RIGHT = 11, 21
TORQUE = int(sys.argv[1]) if len(sys.argv) > 1 else 40
DURATION = float(sys.argv[2]) if len(sys.argv) > 2 else 0.4

def sync_torque(tor_left, tor_right):
    ids = [(LEFT, tor_left), (RIGHT, tor_right)]
    buf = [0xFF, 0xFF, 0xFE, 4 + 5*len(ids), 0x83, 0x1E, 0x04]
    for sid, tor in ids:
        t = tor & 0xFFFF
        buf += [sid, 0x00, 0x00, t & 0xFF, (t >> 8) & 0xFF]
    chk = (~sum(buf[2:])) & 0xFF
    buf.append(chk)
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
    print(f"Commanding LEFT(ID{LEFT}) torque={TORQUE} for {DURATION}s, RIGHT=0")
    print(f"{'t':>6} {'L_pos':>6} {'L_vel':>6}")
    t0 = time.time()
    try:
        while time.time() - t0 < DURATION:
            s.write(sync_torque(TORQUE, 0))
            time.sleep(0.005)
            s.reset_input_buffer()
            s.write(read_req(LEFT))
            time.sleep(0.006)
            r = parse(s.read(32), LEFT)
            if r:
                print(f"{time.time()-t0:6.2f} {r[0]:6d} {r[1]:6d}")
            time.sleep(0.01)
    finally:
        # force zero torque, several times to be sure
        for _ in range(5):
            s.write(sync_torque(0, 0))
            time.sleep(0.01)
        s.close()
        print("Torque zeroed.")

if __name__ == "__main__":
    main()
