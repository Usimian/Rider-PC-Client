#!/usr/bin/env python3
"""Diagnostic test for EEPROM-commit mechanism on these specific servos.

Operates ONLY on ID 22, with a SMALL test offset value (50) so any
unintended motion is bounded to a small angular change.

Tests two hypotheses in sequence:
  A) REG_WRITE (deferred) + ACTION may be required for EEPROM commit
     (some FeeTech servos use this pattern instead of plain WRITE).
  B) FeeTech RESET instruction (0x06) makes the servo reload from EEPROM,
     so reading the register AFTER reset tells us what's actually in EEPROM.

Expected outcomes:
  - If after RESET the offset is 50 -> we found the commit mechanism.
  - If after RESET the offset reverts to the original value -> EEPROM was
    never written, regardless of what we tried.

NO Lock=1, NO Torque enable. Servo should stay limp throughout.
"""
from scservo_sdk import (
    PortHandler, PacketHandler, COMM_SUCCESS,
    PKT_HEADER0, PKT_HEADER1, PKT_ID, PKT_LENGTH, PKT_INSTRUCTION,
)
INST_RESET = 6  # not exported by scservo_sdk; raw FeeTech instruction byte
import time

PORT = "/dev/ttyUSB0"
BAUD = 1000000
SID  = 22
TEST_OFFSET = 50            # small positive offset, magnitude bits only
REG_HOMING_OFFSET = 0x1F

def from_2c(raw, bits=16):
    return raw - (1 << bits) if raw & (1 << (bits - 1)) else raw

def signed_short(v):
    """Display sign-mag bit 11 decoded value for clarity."""
    if v & 0x800:
        return -(v & 0x7FF)
    return v & 0x7FF

def reset_inst(pkt: PacketHandler, port: PortHandler, sid: int):
    """Send the FeeTech RESET instruction (0x06) — make servo reload EEPROM->RAM."""
    txpacket = [0] * 6
    txpacket[PKT_HEADER0] = 0xFF
    txpacket[PKT_HEADER1] = 0xFF
    txpacket[PKT_ID] = sid
    txpacket[PKT_LENGTH] = 2
    txpacket[PKT_INSTRUCTION] = INST_RESET
    return pkt.txRxPacket(port, txpacket)

def read_offset(pkt, port):
    raw, _, _ = pkt.read2ByteTxRx(port, SID, REG_HOMING_OFFSET)
    return raw

def main():
    port = PortHandler(PORT)
    if not port.openPort(): print("port open failed"); return
    port.setBaudRate(BAUD)
    pkt = PacketHandler(0)

    print(f"=== Initial state (ID {SID}) ===")
    base = read_offset(pkt, port)
    print(f"  Homing_Offset (0x1F) raw = 0x{base:04X}  signed-bit11 = {signed_short(base)}")

    # === Test A: REG_WRITE + ACTION ===
    print(f"\n=== Test A: REG_WRITE 0x{TEST_OFFSET:04X} to 0x1F, then ACTION ===")
    data_bytes = [TEST_OFFSET & 0xFF, (TEST_OFFSET >> 8) & 0xFF]  # low byte first
    c, e = pkt.regWriteTxRx(port, SID, REG_HOMING_OFFSET, 2, data_bytes)
    print(f"  REG_WRITE: comm={c} err={e}")
    time.sleep(0.1)
    c, e = pkt.action(port, SID)
    print(f"  ACTION:    comm={c} err={e}")
    time.sleep(0.1)
    after_action = read_offset(pkt, port)
    print(f"  Read after action: raw = 0x{after_action:04X}")
    if after_action == TEST_OFFSET:
        print("  ✓ Value visible in register after REG_WRITE+ACTION")
    else:
        print(f"  ✗ Value not what we wrote — got {after_action} expected {TEST_OFFSET}")

    # === Test B: RESET — reload EEPROM into RAM ===
    print("\n=== Test B: send RESET (instruction 0x06) ===")
    print("  RESET makes the servo reload register state from EEPROM.")
    c, e = reset_inst(pkt, port, SID)
    print(f"  RESET: comm={c} err={e}")
    time.sleep(0.5)  # give it time to reload

    after_reset = read_offset(pkt, port)
    print(f"  Read after reset: raw = 0x{after_reset:04X}")
    if after_reset == TEST_OFFSET:
        print(f"  *** EEPROM committed *** — REG_WRITE+ACTION wrote to EEPROM")
    elif after_reset == base:
        print(f"  *** EEPROM unchanged *** — REG_WRITE+ACTION only touched RAM (reverted to {base})")
    else:
        print(f"  Value is {after_reset} — neither original ({base}) nor test ({TEST_OFFSET})")

    port.closePort()

if __name__ == "__main__":
    main()
