#!/usr/bin/env python3
"""Cleaner EEPROM diagnostic. ID 22 only. Writes value 100 to register 0x1F
and tries multiple commit mechanisms, verifying via RESET (0x06) which makes
the servo reload its registers from EEPROM.

Steps:
  1. Read baseline (current persisted EEPROM value)
  2. Explicitly disable torque + unlock
  3. Plain WRITE 100 to 0x1F
  4. Read back -> RAM state
  5. Send RESET -> reload from EEPROM
  6. Read back -> if 100 = EEPROM was committed by plain WRITE
                   if baseline = EEPROM unchanged
"""
from scservo_sdk import (
    PortHandler, PacketHandler, COMM_SUCCESS,
    PKT_HEADER0, PKT_HEADER1, PKT_ID, PKT_LENGTH, PKT_INSTRUCTION,
)
import time

PORT = "/dev/ttyUSB0"; BAUD = 1000000; SID = 22
TEST_VALUE = 100   # picked so it's clearly different from 255 (current EEPROM)

INST_RESET = 6

def reset_inst(pkt, port, sid):
    txpacket = [0xFF, 0xFF, sid, 2, INST_RESET]
    return pkt.txRxPacket(port, txpacket)

def read_offset(pkt, port):
    raw, _, _ = pkt.read2ByteTxRx(port, SID, 0x1F)
    return raw

def main():
    port = PortHandler(PORT); port.openPort(); port.setBaudRate(BAUD)
    pkt = PacketHandler(0)

    base = read_offset(pkt, port)
    print(f"Baseline EEPROM offset for ID {SID}: 0x{base:04X} ({base})")

    print("\nExplicit torque off + lock off (safety)")
    pkt.write1ByteTxRx(port, SID, 0x28, 0)  # torque
    pkt.write1ByteTxRx(port, SID, 0x37, 0)  # lock
    time.sleep(0.05)

    print(f"\nPlain WRITE {TEST_VALUE} to 0x1F")
    c, e = pkt.write2ByteTxRx(port, SID, 0x1F, TEST_VALUE)
    print(f"  comm={c}  err={e}")
    time.sleep(0.1)
    in_ram = read_offset(pkt, port)
    print(f"  read back (RAM): 0x{in_ram:04X} ({in_ram})")

    print("\nSend RESET (0x06) -> reloads from EEPROM")
    try:
        result = reset_inst(pkt, port, SID)
        print(f"  result: {result}")
    except Exception as ex:
        print(f"  txRxPacket exception: {ex}")
    time.sleep(0.5)

    after = read_offset(pkt, port)
    print(f"\nAfter RESET: 0x{after:04X} ({after})")
    if after == TEST_VALUE:
        print("=> EEPROM committed by plain WRITE (no lock toggle needed)")
    elif after == base:
        print("=> EEPROM unchanged. Plain WRITE only touched RAM.")
    else:
        print(f"=> Unexpected — neither baseline ({base}) nor test ({TEST_VALUE})")

    port.closePort()

if __name__ == "__main__":
    main()
