"""Time the MPC drive planner's QP on the ESP32 (2026-11-03): send firmware 'mpcbench' over USB-C, print the reply.

    /home/marc/.xgo-cal/bin/python tools/diag/mpcbench.py [reps ...] [--port /dev/ttyUSB0]

The robot on the stand, balancing off, USB-C to this machine. The port is opened with DTR / RTS held low, so the
ESP32 does not reset. Each reps value is one run (main.cpp mpcBenchTask: the QP of firmware/esp32_rider_fw/src/
mpc_bench_data.h, from sim/lqr/mpc_export.py, solved reps times on core 0 below the IMU loop). Reply fields: us = the
microseconds a solve, diff_ref = its answer against Python's after the same iterations (precision), diff_conv =
against the converged one, jerk = the plan's first jerk (m/s^3). The planner's step at the ESP32 size is ~41 ms.
MEASURED 2026-11-03 (on the stand): 14.2-14.3 ms a solve (20 / 100 / 200 reps; 25 jerks, 75 limit rows, 50
iterations from cold), the answer within 6e-6 of Python's -- ~31 M multiply-adds a second; ~35 % of core 0 at the
planner's 41 ms step. The matrices are read from flash and the firmware is built for size (-Os): room to gain.
"""
import argparse
import time

import serial


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("reps", type=int, nargs="*", default=[20, 100])
    ap.add_argument("--port", default="/dev/ttyUSB0")
    a = ap.parse_args()
    ser = serial.Serial()
    ser.port, ser.baudrate, ser.timeout = a.port, 115200, 0.2
    ser.dtr = ser.rts = False                          # no auto-reset
    ser.open()
    ser.write(b"\n")                                  # the first line after opening the port is lost: a blank one
    time.sleep(0.3)
    try:
        for reps in a.reps:
            ser.reset_input_buffer()
            ser.write(f"mpcbench {reps}\n".encode())
            t_end = time.time() + 5.0 + reps * 0.05
            while time.time() < t_end:
                line = ser.readline().decode(errors="replace").strip()
                if line.startswith("# mpcbench"):
                    print(line)
                    break
            else:
                print(f"no reply to 'mpcbench {reps}'")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
