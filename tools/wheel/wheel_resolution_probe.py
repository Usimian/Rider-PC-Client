#!/usr/bin/env python3
"""Wheel-encoder RESOLUTION PROBE — is a 14-bit measurement reachable over the bus?

WHY THIS EXISTS
  The Rider wheels are K-power 4012-family FOC BLDC motors with a 14-bit internal
  magnetic encoder, but the 0x24 position read wraps at 1024 (10-bit). The balancer's
  velocity term is quantization-noisy *because* of that 10-bit ceiling — which caps the
  `lqrvx` damping gain and feeds the policy hold-shimmy (hence the g-h velocity observer
  in esp32_rider_fw). IF a 14-bit position, or a clean 14-bit-derived velocity, is exposed
  anywhere on the bus, using it would raise the damping ceiling and cut the shimmy far more
  than gain-tuning. Earlier work only ever read 0x24 (6 bytes) — this probe hunts wider.
  (Open question logged 2026-06-24; see docs/servo_registers.md wheel section.)

REQUIREMENTS  (all important)
  - PASSTHROUGH firmware on the ESP32 — NOT the balance fw (that owns the bus and this
    won't talk to it). Flash: cd firmware/esp32_passthrough && ~/.xgo-cal/bin/pio run -t upload
  - Pi powered down / not driving the bus (single bus owner).
  - A wheel FREE TO SPIN by hand for Phase 2.
  - READ-ONLY: this never sync-writes / never drives the wheels. Safe on the stand.
  Run:  /home/marc/.xgo-cal/bin/python tools/wheel/wheel_resolution_probe.py [WHEEL_ID]
        (WHEEL_ID default 21 = right; use 11 for left)

WHAT IT DOES
  Phase 1 (wheel STILL, automated):
    - Dumps the vendor's 0x48 'ReadMotorState' response — the 2nd read RIG-Omni uses and
      that we NEVER tried on the wheels. May carry finer position / current / cleaner vel.
    - Dumps the known-good 0x24 response for reference.
    - Sweeps read lengths (more bytes may expose more fields) and flags any 16-bit field
      whose value is >1023 (a 10-bit field can't exceed 1023; a 14-bit one ranges 0..16383).
  Phase 2 (spin the wheel by hand, steadily, ~12 s):
    - Logs the servo's REPORTED `vel` and the 10-bit position simultaneously to a CSV, and
      compares sample-to-sample JITTER of reported-vel vs velocity *differenced* from the
      10-bit position. Cleaner reported vel => FOC controller is giving better-than-10-bit
      velocity => the RL policy (which currently differences position) should consume it.

HOW TO READ THE RESULT
  - Phase 1 flags a field that counts PAST 1023 as you rotate (confirm by re-running after
    moving the wheel) -> that's your 14-bit position. Wire it into the firmware's wheel read.
  - Phase 2 reported-vel jitter clearly (>~2x) lower than differenced-pos jitter at steady
    speed -> reported vel is the cleaner signal (likely 14-bit-internal); feed it to the policy.
  - Everything wraps at 1024 AND vel is no cleaner -> 14-bit is genuinely not exposed; the
    g-h observer is already the best available. Close the question, stop looking.
  NOTE: a hand-spin isn't perfectly constant, so the jitter metric is rough — look for a
  CLEAR difference, not a subtle one. Raw CSV is saved for proper offline analysis.
"""
import serial, time, sys, statistics, datetime

WHEEL = int(sys.argv[1]) if len(sys.argv) > 1 else 21   # 21=right (usually free), 11=left

def req(sid, reg, n):
    ck = (~(sid + 0x04 + 0x02 + reg + n)) & 0xFF
    return bytes([0xFF, 0xFF, sid, 0x04, 0x02, reg, n, ck])

def read(s, sid, reg, n):
    """Send a read request, return the response bytes [FFFF..checksum] or None."""
    s.reset_input_buffer(); s.write(req(sid, reg, n)); time.sleep(0.008)
    resp = s.read(48)
    i = resp.find(b"\xff\xff")
    if i < 0: return None
    f = resp[i:]
    if len(f) < 5 or f[2] != sid: return None
    LEN = f[3]
    if len(f) <= LEN: return None
    return list(f[:LEN + 2])           # header .. checksum

def main():
    s = serial.Serial("/dev/ttyUSB0", 1000000, timeout=0.05)
    print(f"=== Phase 1: read dumps (wheel ID{WHEEL}, keep it STILL) ===")
    for reg in (0x24, 0x48):
        r = read(s, WHEEL, reg, 6)
        print(f"  reg 0x{reg:02X} len6: {[hex(b) for b in r] if r else 'no/invalid response'}")
    print("  -- longer reads (more bytes may expose more fields) --")
    for reg in (0x24, 0x48):
        for n in (8, 10, 12):
            r = read(s, WHEEL, reg, n)
            print(f"    reg 0x{reg:02X} len{n}: {[hex(b) for b in r] if r else 'none'}")
    print("  -- 16-bit LE fields > 1023 (CANDIDATE 14-bit; confirm by re-running after rotating) --")
    found = False
    for reg in (0x24, 0x48):
        r = read(s, WHEEL, reg, 12)
        if not r: continue
        for k in range(4, len(r) - 2):          # skip FFFF/ID/LEN/ERR header
            w = r[k] | (r[k + 1] << 8)
            if 1023 < w < 16384:
                print(f"    reg 0x{reg:02X} bytes[{k},{k+1}] = {w}  <-- exceeds 10-bit!")
                found = True
    if not found:
        print("    (none found > 1023 — but re-run while rotating; a 14-bit field changes)")

    print(f"\n=== Phase 2: vel quality — spin wheel ID{WHEEL} STEADILY by hand ~12 s ===")
    try:
        input("    Press ENTER, then start spinning at a steady rate... ")
    except EOFError:
        print("    (no TTY — skipping Phase 2)"); s.close(); return
    fname = "wheel_velprobe_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S") + ".csv"
    reps, diffs = [], []
    last = None; t0 = time.time()
    with open(fname, "w") as f:
        f.write("t_s,pos,reported_vel,diffed_step\n")
        while time.time() - t0 < 12:
            r = read(s, WHEEL, 0x24, 6)
            if r and len(r) >= 12:
                LEN = r[3]
                pos = r[LEN - 3] | (r[LEN - 2] << 8)
                vel = r[LEN - 1] | (r[LEN] << 8)
                if vel >= 0x8000: vel -= 0x10000
                step = ""
                if last is not None:
                    d = pos - last
                    if d < -800: d += 1024
                    elif d > 800: d -= 1024
                    diffs.append(d); step = d
                reps.append(vel); last = pos
                f.write(f"{time.time()-t0:.3f},{pos},{vel},{step}\n")
            time.sleep(0.02)
    s.close()
    print(f"  raw samples saved to {fname}")
    def jitter(x):   # sample-to-sample stdev (isolates high-freq quantization noise from slow speed drift)
        if len(x) < 3: return None
        return statistics.pstdev([x[i+1] - x[i] for i in range(len(x)-1)])
    if len(reps) > 5 and len(diffs) > 5:
        jr, jd = jitter(reps), jitter(diffs)
        print(f"  reported vel : mean={statistics.mean(reps):7.2f}  jitter={jr:.3f}")
        print(f"  diffed pos   : mean={statistics.mean(diffs):7.2f}  jitter={jd:.3f}")
        print("  -> normalize each jitter by |mean| and compare; clearly-lower reported-vel")
        print("     jitter => reported vel is the cleaner velocity source for the policy.")
    else:
        print("  not enough samples — spin faster/longer and rerun.")

if __name__ == "__main__":
    main()
