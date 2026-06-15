#!/usr/bin/env python3
"""Test if ANY EEPROM register persists after power cycle.

Writes a distinctive value to P_Coefficient (reg 0x15, safe tuning param).
Power-cycle and re-run to see if it persisted.

Usage:
  ~/.xgo-cal/bin/python persist_eeprom.py 22       # write test value
  (power cycle robot)
  ~/.xgo-cal/bin/python persist_eeprom.py 22 --check  # just read
"""
from scservo_sdk import PortHandler, PacketHandler
import time, sys

PORT = "/dev/ttyUSB0"; BAUD = 1000000
SID = 22
TEST_VALUE = 77   # distinctive
REG_P = 0x15

check_only = "--check" in sys.argv

def main():
    port = PortHandler(PORT); port.openPort(); port.setBaudRate(BAUD)
    pkt = PacketHandler(0)

    p_now, _, _ = pkt.read1ByteTxRx(port, SID, REG_P)
    print(f"P_Coefficient (0x15) = {p_now}")
    if check_only:
        if p_now == TEST_VALUE:
            print(f"*** PERSISTED *** — EEPROM commits work. Offset register must be specially blocked.")
        else:
            print(f"*** NOT PERSISTED *** — even safe EEPROM writes don't persist. Lock is somewhere we haven't found.")
        port.closePort(); return

    print(f"\nWriting {TEST_VALUE} to P_Coefficient...")
    c, e = pkt.write1ByteTxRx(port, SID, REG_P, TEST_VALUE)
    print(f"  WRITE: comm={c} err={e}")
    time.sleep(0.2)
    p_after, _, _ = pkt.read1ByteTxRx(port, SID, REG_P)
    print(f"  After write (RAM): {p_after}")
    print(f"\nPower-cycle, then run:\n  ~/.xgo-cal/bin/python {sys.argv[0]} 22 --check")
    port.closePort()

if __name__ == "__main__":
    main()
