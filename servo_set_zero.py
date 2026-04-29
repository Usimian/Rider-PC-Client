#!/usr/bin/env python3
"""Set current physical position as the new zero for servos 12 and 22.

Procedure:
  1. Read current present position P for each servo
  2. Write P (sign-magnitude format) to OFFSET register 0x1F
  3. Lock EEPROM (write 1 to 0x37) to commit
  4. Tell user to power-cycle so servos pick up new offset

WARNING: writing the offset register may cause the servo to twitch.
Hold the legs firmly at desired neutral pose before pressing Enter.
"""
from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS
import time

PORT = "/dev/ttyUSB0"
BAUD = 1000000

def to_signmag(value: int) -> int:
    """Convert signed int to FeeTech 16-bit sign-magnitude format.
    bit15 = sign (1 = negative), bits 0..14 = magnitude.
    """
    if value < 0:
        return 0x8000 | (-value & 0x7FFF)
    return value & 0x7FFF

def from_signmag(raw: int) -> int:
    """FeeTech 16-bit sign-magnitude -> signed int."""
    if raw & 0x8000:
        return -(raw & 0x7FFF)
    return raw & 0x7FFF

def from_2c(raw: int) -> int:
    return raw - 0x10000 if raw > 0x7FFF else raw

def main():
    port = PortHandler(PORT)
    if not port.openPort():
        print("Failed to open port"); return
    port.setBaudRate(BAUD)
    pkt = PacketHandler(0)

    # Step 1: dry-read to show what we're about to do
    plan = []
    print("Reading current state of leg servos...\n")
    for sid in [12, 22]:
        pos_raw, _, _ = pkt.read2ByteTxRx(port, sid, 0x24)
        pos_signed = from_2c(pos_raw)
        offset_to_write = to_signmag(pos_signed)
        plan.append((sid, pos_signed, offset_to_write))
        print(f"  ID {sid}: present_pos = {pos_signed}")
        print(f"           will write offset = {pos_signed} "
              f"(sign-mag raw = 0x{offset_to_write:04X})")

    print("\n" + "!" * 60)
    print("WARNING: writing offsets MAY CAUSE THE SERVOS TO TWITCH.")
    print("Hold both legs FIRMLY at the desired neutral pose right now.")
    print("Be ready to KILL POWER if anything sounds wrong.")
    print("!" * 60)
    ans = input("\nProceed? (type 'yes' to write offsets): ").strip()
    if ans.lower() != "yes":
        print("Aborted, no changes made.")
        port.closePort()
        return

    # Step 2: write offsets
    print("\nWriting offsets...")
    for sid, _, offset in plan:
        c, e = pkt.write2ByteTxRx(port, sid, 0x1F, offset)
        ok = "✓" if c == COMM_SUCCESS else "✗"
        print(f"  {ok} ID {sid}: write offset 0x{offset:04X}  comm={c} err={e}")
        time.sleep(0.05)

    # Step 3: commit to EEPROM (write 1 to lock register 0x37)
    print("\nCommitting to EEPROM (lock=1)...")
    for sid, _, _ in plan:
        c, e = pkt.write1ByteTxRx(port, sid, 0x37, 1)
        ok = "✓" if c == COMM_SUCCESS else "✗"
        print(f"  {ok} ID {sid}: lock=1  comm={c} err={e}")
        time.sleep(0.05)

    # Optional: re-unlock so future tests are easier
    for sid, _, _ in plan:
        pkt.write1ByteTxRx(port, sid, 0x37, 0)
        time.sleep(0.05)

    # Step 4: read back to confirm offset stored
    print("\nVerifying stored offsets:")
    for sid, _, _ in plan:
        v, _, _ = pkt.read2ByteTxRx(port, sid, 0x1F)
        print(f"  ID {sid}: 0x1F = 0x{v:04X} (sign-mag = {from_signmag(v)})")

    port.closePort()

    print("\n" + "=" * 60)
    print("Now POWER-CYCLE the robot.")
    print("After it boots back up, run servo_snap.py and check position.")
    print("If pos reads ~0 at this physical pose, calibration WORKED.")
    print("If pos reads ~2x original value, sign was wrong — we'll flip it.")
    print("=" * 60)

if __name__ == "__main__":
    main()
