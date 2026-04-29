#!/usr/bin/env python3
"""Read key registers from each servo to confirm what we're talking to."""
from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS

PORT = "/dev/ttyUSB0"
BAUD = 1000000

# Common FeeTech SMS/STS register addresses
REGS = {
    0x03: ("ID", 1),
    0x04: ("Baud", 1),
    0x05: ("Return Delay", 1),
    0x06: ("Response Status Level", 1),
    0x09: ("Min Angle Limit (LO)", 2),
    0x0B: ("Max Angle Limit (LO)", 2),
    0x14: ("Min Voltage", 1),
    0x15: ("Max Voltage", 1),
    0x16: ("Max Torque", 2),
    0x1F: ("Position Correction (offset)", 2),
    0x37: ("Lock", 1),
    0x38: ("Present Position", 2),
    0x3A: ("Present Speed", 2),
    0x3F: ("Present Voltage", 1),
}

def main():
    port = PortHandler(PORT)
    port.openPort()
    port.setBaudRate(BAUD)
    pkt = PacketHandler(0)

    for sid in [11, 12]:
        print(f"\n=== Servo ID {sid} ===")
        for addr, (name, size) in REGS.items():
            if size == 1:
                val, comm, err = pkt.read1ByteTxRx(port, sid, addr)
            else:
                val, comm, err = pkt.read2ByteTxRx(port, sid, addr)
            ok = "✓" if comm == COMM_SUCCESS else "✗"
            print(f"  {ok} 0x{addr:02X} {name:35s} = {val} (0x{val:04X})")

    port.closePort()

if __name__ == "__main__":
    main()
