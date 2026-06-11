#!/usr/bin/env python3
"""Live wheel-encoder monitor for XGO Rider (raw FeeTech-FOC protocol).

Requires passthrough firmware on the ESP32 and the Pi powered DOWN (so it
doesn't drive the bus). Read-only: commands no motion.

Protocol (reverse-engineered + confirmed against RIG-Omni xgo.cc):
  Request : FF FF ID 04 02 24 06 CK   (read 6 bytes from reg 0x24)
  Response: FF FF ID LEN ERR ...payload... CK   (LEN=0x0B observed)
  Within the response, pos/vel sit at the tail:
    pos_low  = resp[LEN-3]   pos_high = resp[LEN-2]
    vel_low  = resp[LEN-1]   vel_high = resp[LEN]
  pos is 10-bit (0..1023) and wraps; odometry accumulates with a
  +/-800 wrap threshold (firmware convention).

Turn a wheel by hand and watch pos change and odom accumulate across wraps.
Ctrl-C to exit.
"""
import serial, time, sys

LEFT, RIGHT = 11, 21
WRAP = 1024          # 10-bit wheel encoder span
WRAP_THRESH = 800

def read_req(sid):
    ck = (~(sid + 0x04 + 0x02 + 0x24 + 0x06)) & 0xFF
    return bytes([0xFF, 0xFF, sid, 0x04, 0x02, 0x24, 0x06, ck])

def parse(resp, sid):
    # locate header
    i = resp.find(b'\xff\xff')
    if i < 0 or len(resp) < i + 5:
        return None
    f = resp[i:]
    if f[2] != sid:
        return None
    LEN = f[3]
    if len(f) <= LEN:
        return None
    pos = f[LEN-3] | (f[LEN-2] << 8)
    vel = f[LEN-1] | (f[LEN] << 8)
    if vel >= 0x8000:
        vel -= 0x10000
    return pos, vel

def main():
    s = serial.Serial('/dev/ttyUSB0', 1000000, timeout=0.05)
    odom = {LEFT: 0, RIGHT: 0}
    last = {LEFT: None, RIGHT: None}

    print(f"{'L_pos':>6} {'L_vel':>6} {'L_odom':>8}   {'R_pos':>6} {'R_vel':>6} {'R_odom':>8}")
    try:
        while True:
            row = {}
            for sid in (LEFT, RIGHT):
                s.reset_input_buffer()
                s.write(read_req(sid))
                time.sleep(0.008)
                r = parse(s.read(32), sid)
                row[sid] = r
                if r:
                    pos, vel = r
                    if last[sid] is not None:
                        d = pos - last[sid]
                        if d < -WRAP_THRESH:   d += WRAP
                        elif d > WRAP_THRESH:  d -= WRAP
                        odom[sid] += d
                    last[sid] = pos
            lp = row[LEFT]; rp = row[RIGHT]
            ls = f"{lp[0]:6d} {lp[1]:6d} {odom[LEFT]:8d}" if lp else "   ---    ---      ---"
            rs = f"{rp[0]:6d} {rp[1]:6d} {odom[RIGHT]:8d}" if rp else "   ---    ---      ---"
            print(f"{ls}   {rs}")
            time.sleep(0.04)
    except KeyboardInterrupt:
        print("\nDone.")
    finally:
        s.close()

if __name__ == "__main__":
    main()
