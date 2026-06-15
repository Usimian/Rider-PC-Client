#!/usr/bin/env python3
"""PID step-response harness for tuning the left-wheel position loop.

Holds at 0 briefly, commands a step to +STEP ticks, logs the response, then
auto-zeros torque. Prints metrics (overshoot, settling time, steady-state
error) so gains can be tuned objectively and repeatably.

Usage:
  ~/.xgo-cal/bin/python wheel_step_tune.py Kp Ki Kd [STEP] [DURATION] [--quiet]
"""
import serial, time, sys

LEFT, RIGHT = 11, 21
WRAP, WRAP_THRESH = 1024, 800

args = [a for a in sys.argv[1:] if not a.startswith('--')]
QUIET = '--quiet' in sys.argv
CLAMP = next((int(a.split('=')[1]) for a in sys.argv if a.startswith('--clamp=')), 80)
Kp = float(args[0]); Ki = float(args[1]); Kd = float(args[2])
STEP     = int(args[3])   if len(args) > 3 else 150
DURATION = float(args[4]) if len(args) > 4 else 3.0

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

def clamp(v, lo, hi): return lo if v < lo else hi if v > hi else v

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

    integral = 0.0
    i_limit = CLAMP / Ki if Ki > 0 else 0.0
    t0 = time.time()
    samples = []  # (t, odom, err)
    try:
        while time.time() - t0 < DURATION:
            t = time.time() - t0
            target = 0 if t < 0.5 else STEP
            s.reset_input_buffer(); s.write(read_req(LEFT)); time.sleep(0.006)
            r = parse(s.read(32), LEFT)
            if r:
                pos, vel = r
                d = pos - last_pos
                if d < -WRAP_THRESH: d += WRAP
                elif d > WRAP_THRESH: d -= WRAP
                odom += d; last_pos = pos
                err = target - odom
                integral += err
                if i_limit: integral = clamp(integral, -i_limit, i_limit)
                tor = clamp(Kp*err + Ki*integral - Kd*vel, -CLAMP, CLAMP)
                s.write(sync_torque(tor, 0))
                samples.append((t, odom, err))
                if not QUIET:
                    print(f"{t:5.2f} odom={odom:6d} err={err:6d} vel={vel:4d} tor={tor:7.1f}")
            time.sleep(0.02)
    finally:
        for _ in range(5):
            s.write(sync_torque(0, 0)); time.sleep(0.01)
        s.close()

    # metrics over the post-step window
    post = [(t, odom, err) for (t, odom, err) in samples if t >= 0.5]
    if post and STEP != 0:
        peak = max(odom for _, odom, _ in post)
        overshoot = max(0.0, (peak - STEP) / STEP * 100.0)
        final = post[-1][1]
        sse = STEP - final
        # settling: last time |err| first stays within 5% band
        band = max(3, abs(STEP) * 0.05)
        settle_t = None
        for k in range(len(post)):
            if all(abs(STEP - o) <= band for _, o, _ in post[k:]):
                settle_t = post[k][0] - 0.5
                break
        print(f"\n[Kp={Kp} Ki={Ki} Kd={Kd} step={STEP}] "
              f"overshoot={overshoot:.0f}%  settle={settle_t if settle_t is not None else 'none':}  "
              f"final_odom={final}  sse={sse}")

if __name__ == "__main__":
    main()
