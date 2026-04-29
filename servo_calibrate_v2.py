#!/usr/bin/env python3
"""Calibration v2 — using LeRobot's verified approach for STS/SMS-family servos.

Key fixes vs previous attempts:
  - Homing_Offset (register 31, 0x1F) uses sign-magnitude with sign bit at BIT 11
    (NOT bit 15). Magnitude range: 0..2047.
  - Lock=1 sequence (don't re-unlock after committing).
  - Goal_Position is set to actual position before lock=1 so torque-engage
    doesn't drive the servo anywhere.

Procedure:
  1. Read present position P from each leg servo (try BOTH 0x24 and 0x38, report).
  2. Set Goal_Position = current_actual to prevent driving when torque enables.
  3. Write Homing_Offset = P (encoded sign-mag bit 11) to register 31.
  4. Write Lock=1 to commit to EEPROM (this also enables torque).
  5. STAY locked. User power-cycles.
"""
from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS
import time

PORT = "/dev/ttyUSB0"
BAUD = 1000000
LEG_SERVOS = [12, 22]

# Register addresses (LeRobot STS/SMS table)
REG_HOMING_OFFSET = 0x1F  # 31
REG_TORQUE_ENABLE = 0x28  # 40
REG_GOAL_POSITION = 0x2A  # 42
REG_LOCK          = 0x37  # 55
REG_PRESENT_POS   = 0x38  # 56  (LeRobot says this is position)
REG_USER_POS      = 0x24  # 36  (User said this is position on these servos)

def from_2c(raw, bits=16):
    return raw - (1 << bits) if raw & (1 << (bits - 1)) else raw

def encode_signmag_bit11(value):
    """Sign-magnitude with sign bit at bit 11. Magnitude 0..2047."""
    mag = abs(value)
    if mag > 2047:
        raise ValueError(f"Magnitude {mag} > 2047 (max for bit-11 sign-mag)")
    return (0x800 | mag) if value < 0 else mag

def main():
    port = PortHandler(PORT)
    if not port.openPort():
        print("Port open failed"); return
    port.setBaudRate(BAUD)
    pkt = PacketHandler(0)

    # --- Read state, both candidate position registers ---
    print("=== Current state ===")
    plan = []
    for sid in LEG_SERVOS:
        pos_24, _, _ = pkt.read2ByteTxRx(port, sid, REG_USER_POS)
        pos_38, _, _ = pkt.read2ByteTxRx(port, sid, REG_PRESENT_POS)
        torque, _, _ = pkt.read1ByteTxRx(port, sid, REG_TORQUE_ENABLE)
        lock, _, _   = pkt.read1ByteTxRx(port, sid, REG_LOCK)
        offset_raw, _, _ = pkt.read2ByteTxRx(port, sid, REG_HOMING_OFFSET)
        print(f"  ID {sid}: pos@0x24={from_2c(pos_24):6d}   pos@0x38={from_2c(pos_38):6d}   "
              f"torque={torque}  lock={lock}  offset_raw=0x{offset_raw:04X}")
        # Use 0x24 as position source (user-verified)
        plan.append((sid, from_2c(pos_24)))

    # --- Show plan ---
    print("\n=== Plan ===")
    for sid, p in plan:
        try:
            enc = encode_signmag_bit11(p)
            print(f"  ID {sid}: write Homing_Offset = {p}  (encoded bit-11 sign-mag = 0x{enc:04X})")
        except ValueError as e:
            print(f"  ID {sid}: ERROR — {e}")
            print("  Position out of ±2047 range — cannot encode. Aborting.")
            port.closePort(); return

    print("\n" + "!"*60)
    print("WARNING: Step 3 writes Lock=1 which ALSO ENABLES TORQUE.")
    print("Before that we set Goal_Position = current_actual to prevent")
    print("the servo from driving anywhere. But hold the legs anyway,")
    print("ready to kill power if something twitches hard.")
    print("!"*60)
    if input("\nProceed? (type 'yes'): ").strip().lower() != "yes":
        print("Aborted, no writes.")
        port.closePort(); return

    # --- Write sequence ---
    print("\nStep 1: ensure lock=0, torque=0")
    for sid, _ in plan:
        pkt.write1ByteTxRx(port, sid, REG_LOCK, 0)
        pkt.write1ByteTxRx(port, sid, REG_TORQUE_ENABLE, 0)
        time.sleep(0.05)

    print("Step 2: pre-set Goal_Position = current actual (so torque-on won't drive)")
    for sid, p in plan:
        # Goal position is bit-15 sign-mag per LeRobot. For positive p, just write p.
        # For negative, encode bit-15 sign-mag.
        goal_enc = (0x8000 | abs(p)) if p < 0 else p
        c, e = pkt.write2ByteTxRx(port, sid, REG_GOAL_POSITION, goal_enc)
        print(f"  ID {sid}: goal=0x{goal_enc:04X}  comm={c} err={e}")
        time.sleep(0.05)

    print("Step 3: write Homing_Offset (bit-11 sign-mag)")
    for sid, p in plan:
        enc = encode_signmag_bit11(p)
        c, e = pkt.write2ByteTxRx(port, sid, REG_HOMING_OFFSET, enc)
        print(f"  ID {sid}: offset=0x{enc:04X}  comm={c} err={e}")
        time.sleep(0.05)

    print("Step 4: Lock=1 (commits to EEPROM, also enables torque)")
    for sid, _ in plan:
        c, e = pkt.write1ByteTxRx(port, sid, REG_LOCK, 1)
        print(f"  ID {sid}: lock=1 comm={c} err={e}")
        time.sleep(0.05)

    # --- Verify (read back) ---
    print("\n=== Verification (still in current power session) ===")
    for sid, _ in plan:
        ofs, _, _ = pkt.read2ByteTxRx(port, sid, REG_HOMING_OFFSET)
        lock, _, _ = pkt.read1ByteTxRx(port, sid, REG_LOCK)
        pos_24, _, _ = pkt.read2ByteTxRx(port, sid, REG_USER_POS)
        print(f"  ID {sid}: offset_raw=0x{ofs:04X}  lock={lock}  pos@0x24={from_2c(pos_24)}")

    port.closePort()
    print("\n" + "="*60)
    print("Now POWER-CYCLE THE ROBOT.")
    print("After it boots back up, hold legs at the same calibration pose")
    print("and run servo_snap.py — present position should read ~0.")
    print("="*60)

if __name__ == "__main__":
    main()
