#!/usr/bin/env python3
"""Ping all servo IDs via the ESP32 passthrough firmware.

Sends a FeeTech-style ping to each ID 1..253 and reports which ones respond.
Validates that GPIO 16/17 is the correct servo bus for our passthrough firmware.
"""
import sys
from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS

PORT = "/dev/ttyUSB0"
BAUD = 1000000

def main():
    port = PortHandler(PORT)
    if not port.openPort():
        print(f"FAILED to open {PORT}")
        sys.exit(1)
    if not port.setBaudRate(BAUD):
        print(f"FAILED to set baud {BAUD}")
        sys.exit(1)

    pkt = PacketHandler(0)  # protocol 0 = SCS/STS standard

    print(f"Pinging IDs 1..253 at {BAUD} baud on {PORT} ...")
    found = []
    for sid in range(1, 254):
        model, comm, err = pkt.ping(port, sid)
        if comm == COMM_SUCCESS:
            print(f"  ID {sid:3d}: model {model:#06x}, err={err}")
            found.append(sid)

    port.closePort()
    print(f"\n{len(found)} servo(s) responded: {found}")
    if not found:
        print("\nNo response on this UART/baud. Likely wrong GPIO pins for the servo bus.")

if __name__ == "__main__":
    main()
