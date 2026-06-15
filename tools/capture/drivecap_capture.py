#!/usr/bin/env python3
"""Auto-capture the firmware's high-rate drive buffer (commanded vs actual wheel
velocity + pitch) over the Pi MQTT broker -- works while the robot drives
UNTETHERED (USB-C unplugged). No timing coordination needed from the driver.

Subscribes to the Pi broker: rider/debug/telem (to detect a sustained drive) and
rider/debug/dcap (the dump). On a sustained drive it publishes `drivecap` to
rider/control/line, collects the `# dcap` block, and appends it to CSV tagged
fwd/rev. Just drive forward a few times, then backward.

Run (workstation):  /usr/bin/python3 tools/drivecap_capture.py [out.csv]
"""
import sys, time, json
import paho.mqtt.client as mqtt
from collections import deque

BROKER = "10.0.0.95"
OUT = sys.argv[1] if len(sys.argv) > 1 else "drivecap.csv"
THR = 0.12
f = open(OUT, "w", buffering=1)
f.write("episode,dir,i,t_us,cmd,wid,v,pitch_cdeg\n")
st = {"ep": 0, "last": 0.0, "cap": False, "dir": "", "rows": [], "seen": False}
hist = deque(maxlen=6)


def on_connect(c, u, fl, rc, p=None):
    c.subscribe("rider/debug/telem"); c.subscribe("rider/debug/dcap")
    print("monitoring Pi broker for driving (Ctrl-C to stop) ...", flush=True)


def on_message(c, u, m):
    pl = m.payload.decode(errors="replace").strip()
    if m.topic == "rider/debug/dcap":
        if pl.startswith("# dcap done"):
            if st["cap"]:
                for r in st["rows"]:
                    f.write("%d,%s,%s\n" % (st["ep"], st["dir"], r))
                print("   episode %d (%s): %d samples saved" % (st["ep"], st["dir"], len(st["rows"])), flush=True)
                st["ep"] += 1; st["cap"] = False; st["rows"] = []; st["last"] = time.time(); hist.clear()
        elif pl.startswith("# dcap ") and st["cap"]:
            p = pl.split()
            if len(p) == 8:
                st["rows"].append(",".join(p[2:]))
        return
    if not pl.startswith("th="):
        return
    d = dict(x.split("=") for x in pl.split() if "=" in x)
    try:
        vdes = float(d.get("vdes", 0)); en = int(float(d.get("en", 0)))
    except ValueError:
        return
    if not st["seen"]:
        st["seen"] = True
        print("  telemetry flowing (en=%d vdes=%.2f) -- ready to drive" % (en, vdes), flush=True)
    hist.append(vdes)
    if (en and not st["cap"] and len(hist) == hist.maxlen
            and all(abs(x) > THR for x in hist)
            and all((x > 0) == (hist[0] > 0) for x in hist)
            and time.time() - st["last"] > 2.5):
        st["dir"] = "fwd" if hist[0] > 0 else "rev"; st["cap"] = True; st["rows"] = []
        print("  %s drive detected -> drivecap" % st["dir"], flush=True)
        c.publish("rider/control/line", json.dumps({"line": "drivecap"}), qos=1)


cl = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="drivecap")
cl.on_connect = on_connect
cl.on_message = on_message
cl.connect(BROKER, 1883, 30)
cl.loop_forever()
