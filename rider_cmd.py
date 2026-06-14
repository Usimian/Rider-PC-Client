#!/usr/bin/env python3
"""Send a raw ESP32 line command to the Rider through the Pi bridge (MQTT relay).

The bridge (rider_status_screen.py on the Pi) owns /dev/ttyAMA0 and forwards
anything published to rider/control/line straight to the ESP32 as a line command.

Usage:
  python3 rider_cmd.py polrun 1      # arm the neural balance policy (no motion yet)
  python3 rider_cmd.py en 1          # enable control -> robot balances
  python3 rider_cmd.py ptgt 0.30     # drive to position target 0.30 m
  python3 rider_cmd.py en 0          # stop / disable

Convenience:
  python3 rider_cmd.py start         # = polrun 1 then en 1  (run the policy)
  python3 rider_cmd.py stop          # = en 0
"""
import sys, json, time
import paho.mqtt.client as mqtt

BROKER = "10.0.0.95"
TOPIC = "rider/control/line"


def send(client, line):
    client.publish(TOPIC, json.dumps({"line": line}), qos=1)
    print("sent:", line)
    time.sleep(0.25)


def main():
    args = sys.argv[1:]
    if not args:
        print(__doc__); return
    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="rider_cmd")
    c.connect(BROKER, 1883); c.loop_start()
    if args[0] == "start":
        send(c, "polrun 1"); send(c, "en 1")
    elif args[0] == "stop":
        send(c, "en 0")
    else:
        send(c, " ".join(args))
    time.sleep(0.2); c.disconnect()


if __name__ == "__main__":
    main()
