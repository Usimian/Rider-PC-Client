#!/usr/bin/env python3
"""Tier 2: host-side closed-loop wheel POSITION HOLD on the left wheel (ID 11).

Reads multi-turn odometry, runs a PD controller that outputs torque to hold
the wheel at a target (default: wherever it is at startup). Push the wheel by
hand and it should drive back to the setpoint.

PID law:  torque = Kp*err + Ki*∫err - Kd*vel,  err = target - odom  (clamped)
  Positive torque increases position (confirmed: torque 40 -> pos up).
  The Ki term eliminates the stiction steady-state error that pure PD left
  (wheel stalled ~62 ticks off target). Integral has anti-windup clamping.

Requires passthrough firmware + Pi process killed. Robot on stand, wheels free.
Ctrl-C to exit (force-zeros torque).

Usage:
  ~/.xgo-cal/bin/python wheel_position_hold.py [Kp] [Ki] [Kd] [clamp] [deadband]
  defaults: Kp=0.4 Ki=0.6 Kd=1.5 clamp=80 deadband=2
"""
import serial, time, sys

LEFT, RIGHT = 11, 21
WRAP, WRAP_THRESH = 1024, 800

Kp       = float(sys.argv[1]) if len(sys.argv) > 1 else 0.4
Ki       = float(sys.argv[2]) if len(sys.argv) > 2 else 0.6
Kd       = float(sys.argv[3]) if len(sys.argv) > 3 else 1.5
CLAMP    = int(sys.argv[4])   if len(sys.argv) > 4 else 80
DEADBAND = int(sys.argv[5])   if len(sys.argv) > 5 else 2

def sync_torque(tor_left, tor_right):
    ids = [(LEFT, tor_left), (RIGHT, tor_right)]
    buf = [0xFF, 0xFF, 0xFE, 4 + 5*len(ids), 0x83, 0x1E, 0x04]
    for sid, tor in ids:
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

def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v

def main():
    s = serial.Serial('/dev/ttyUSB0', 1000000, timeout=0.05)

    # establish starting position
    last_pos, odom = None, 0
    for _ in range(5):
        s.reset_input_buffer(); s.write(read_req(LEFT)); time.sleep(0.008)
        r = parse(s.read(32), LEFT)
        if r: last_pos = r[0]
        time.sleep(0.01)
    if last_pos is None:
        print("Could not read wheel. Abort."); s.close(); return
    target = 0  # odom starts at 0 -> hold the start position

    # anti-windup: cap the integral's torque contribution to the clamp
    integral = 0.0
    i_limit = CLAMP / Ki if Ki > 0 else 0.0

    print(f"Holding LEFT wheel at start. Kp={Kp} Ki={Ki} Kd={Kd} clamp=±{CLAMP} deadband=±{DEADBAND}")
    print("Push the wheel by hand; it should drive back. Ctrl-C to stop.")
    print(f"{'odom':>7} {'err':>6} {'vel':>5} {'P':>6} {'I':>6} {'D':>6} {'torque':>7}")
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

                err = target - odom
                if abs(err) <= DEADBAND:
                    integral = 0.0
                    tor = 0.0
                    P = I = D = 0.0
                else:
                    integral += err
                    if i_limit:
                        integral = clamp(integral, -i_limit, i_limit)
                    P = Kp * err
                    I = Ki * integral
                    D = -Kd * vel
                    tor = clamp(P + I + D, -CLAMP, CLAMP)
                s.write(sync_torque(tor, 0))
                print(f"{odom:7d} {err:6d} {vel:5d} {P:6.1f} {I:6.1f} {D:6.1f} {tor:7.1f}")
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
