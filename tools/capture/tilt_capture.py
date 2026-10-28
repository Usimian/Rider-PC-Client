#!/usr/bin/env python3
"""Back-to-back high-rate captures while the robot is rocked BY HAND, balance OFF (2026-10-28).

For the IMU position fit and the encoder-against-the-body check (tools/balance/imu_position_fit.py): the
firmware's drive capture ('drivecap', 600 samples at loop rate, ~2.9 s) also records with balance off. This
arms it, waits for it to fill, fetches it with 'logdump', and repeats, so a continuous rocking session comes
back as a series of ~2.9 s windows (a short gap during each dump). Sends only 'drivecap' / 'logdump'; refuses
to start while balancing (wheels must be limp: nothing here drives them).

Run (workstation):  /usr/bin/python3 tools/capture/tilt_capture.py out.csv [windows]   (default 10)
The broker comes from gui/rider_config.ini, as for tools/diag/mqtt_monitor.py.
"""
import sys, os, time, json, configparser
import paho.mqtt.client as mqtt

_ini = configparser.ConfigParser()
_ini.read(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "gui", "rider_config.ini"))
BROKER = _ini.get("mqtt", "broker_host", fallback="localhost")
OUT = sys.argv[1] if len(sys.argv) > 1 else "tilt.csv"
WINDOWS = int(sys.argv[2]) if len(sys.argv) > 2 else 10
# the firmware's '# dcap' fields, in order (main.cpp logdump), as in drivecap_capture.py
COLS = ["i", "t_us", "cmd", "wid", "v", "pitch_cdeg", "rate_x10", "cmdL", "cmdR", "vR",
        "pol_pitch_x100", "pol_rate_x10", "pol_xvel_mm", "pol_int_m", "pol_bias_x100", "pacc_x100", "yaw_x10",
        "yrc_x1000"]
st = {"en": None, "rows": [], "done": False}


def on_connect(c, u, fl, rc, p=None):
    c.subscribe("rider/debug/telem"); c.subscribe("rider/debug/dcap")


def on_message(c, u, m):
    pl = m.payload.decode(errors="replace").strip()
    if m.topic == "rider/debug/telem" and pl.startswith("th="):
        d = dict(x.split("=", 1) for x in pl.split() if "=" in x)
        st["en"] = d.get("en")
    elif pl.startswith("# dcap done"):
        st["done"] = True
    elif pl.startswith("# dcap ") and not pl.startswith("# dcap n="):
        p = pl.split()
        if len(p) == 2 + len(COLS):
            st["rows"].append(",".join(p[2:]))


def send(c, s):
    c.publish("rider/control/line", json.dumps({"line": s}), qos=1)


def main():
    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="tilt_capture")
    c.on_connect = on_connect; c.on_message = on_message
    c.connect(BROKER, 1883, 30); c.loop_start()
    t0 = time.time()
    while st["en"] is None and time.time() - t0 < 10:
        time.sleep(0.1)
    if st["en"] is None:
        sys.exit("no telemetry from the robot")
    if st["en"] != "0":
        sys.exit("balance is ON -- this test needs balance off (en 0)")
    with open(OUT, "w") as f:
        f.write("window," + ",".join(COLS) + "\n")
        for w in range(WINDOWS):
            st["rows"] = []; st["done"] = False
            send(c, "drivecap")
            print(f"window {w + 1}/{WINDOWS}: recording", flush=True)
            time.sleep(3.3)
            send(c, "logdump")
            t1 = time.time()
            while not st["done"] and time.time() - t1 < 15:
                time.sleep(0.05)
            for r in st["rows"]:
                f.write(f"{w},{r}\n")
            print(f"window {w + 1}/{WINDOWS}: {len(st['rows'])} samples", flush=True)
    c.loop_stop()


if __name__ == "__main__":
    main()
