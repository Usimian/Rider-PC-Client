#!/usr/bin/env python3
"""Pure-turn wheel symmetry test.

Requires passthrough firmware on the ESP32. Commands left wheel (ID 11) at
+V and right wheel (ID 21) at -V for a configurable duration, while logging
both wheels' encoder positions and velocities at ~10 Hz. Reports per-wheel
mean velocity at the end so asymmetry is visible.

Protocol (per RIG-Omni xgo.cc):
  WRITE: register 0x35, 4 bytes = [pos_L, pos_H, vel_L, vel_H]
  READ : register 0x24, 6 bytes -> first 4 bytes are pos(L,H), vel(L,H)
  Position is 10-bit (0..1023), wraps. Velocity scale (approx rad/s):
        v_rads = 4.5 * raw * (60 * pi / 1024)

Usage:
  ~/.xgo-cal/bin/python wheel_turn_test.py [V] [duration_sec]
  V default = 100  (try small first!)
  duration default = 3.0
"""
from scservo_sdk import PortHandler, PacketHandler
import math, sys, time

V = int(sys.argv[1]) if len(sys.argv) > 1 else 100
DURATION = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0
LEFT, RIGHT = 11, 21
REG_GOAL = 0x35
REG_FB = 0x24
K_V = 60.0 * math.pi / 1024.0
SCALE = 4.5 * K_V

port = PortHandler('/dev/ttyUSB0')
port.openPort(); port.setBaudRate(1000000)
pkt = PacketHandler(0)

def cmd(sid, pos, vel):
    data = [pos & 0xFF, (pos >> 8) & 0xFF, vel & 0xFF, (vel >> 8) & 0xFF]
    return pkt.writeTxRx(port, sid, REG_GOAL, 4, data)

def read_fb(sid):
    data, comm, err = pkt.readTxRx(port, sid, REG_FB, 6)
    if comm != 0 or err != 0 or len(data) < 4:
        return None
    pos = data[0] | (data[1] << 8)
    vel_raw = data[2] | (data[3] << 8)
    # signed 16-bit interpretation for velocity
    if vel_raw >= 0x8000: vel_raw -= 0x10000
    return pos, vel_raw

def stop_all():
    for sid in (LEFT, RIGHT):
        cmd(sid, 0, 0)

try:
    print(f"Commanding LEFT(ID{LEFT})=+{V}  RIGHT(ID{RIGHT})=-{V}  for {DURATION}s")
    print(f"{'t':>6} {'L_pos':>7} {'L_vraw':>7} {'L_v_rads':>9} {'R_pos':>7} {'R_vraw':>7} {'R_v_rads':>9}")
    cmd(LEFT, 0, +V)
    cmd(RIGHT, 0, -V)

    t0 = time.time()
    samples = []
    while time.time() - t0 < DURATION:
        l = read_fb(LEFT); r = read_fb(RIGHT)
        if l and r:
            lp, lv = l; rp, rv = r
            samples.append((lv, rv))
            print(f"{time.time()-t0:6.2f} {lp:7d} {lv:7d} {SCALE*lv:9.3f} {rp:7d} {rv:7d} {SCALE*rv:9.3f}")
        time.sleep(0.1)

    print("\nStopping...")
    stop_all()

    if samples:
        lvs = [s[0] for s in samples]
        rvs = [s[1] for s in samples]
        l_mean = sum(lvs) / len(lvs)
        r_mean = sum(rvs) / len(rvs)
        print(f"\nMean L raw vel: {l_mean:+.1f}   ({SCALE*l_mean:+.3f} rad/s)")
        print(f"Mean R raw vel: {r_mean:+.1f}   ({SCALE*r_mean:+.3f} rad/s)")
        print(f"|L| - |R| raw : {abs(l_mean) - abs(r_mean):+.1f}  (asymmetry; ideal 0)")
        print(f"L + R raw     : {l_mean + r_mean:+.1f}  (forward bias; ideal 0)")
finally:
    stop_all()
    port.closePort()
