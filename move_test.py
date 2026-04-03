#!/usr/bin/env python3
# coding=utf-8
"""
Robot movement test tool.

Usage:
  python move_test.py move 200          # forward 200mm
  python move_test.py move -300         # backward 300mm
  python move_test.py turn 45           # left 45°
  python move_test.py turn -90          # right 90°
  python move_test.py move 200 --raw    # skip scaling, send value directly

Scale factors are read from rider_config.ini [calibration] section.
Use --raw to send the value as-is (useful for finding the right scale factor).
"""

import argparse
import configparser
import json
import os
import time
import sys
import paho.mqtt.client as mqtt

TOPIC = 'rider/control/movement'

# Defaults if rider_config.ini is missing or incomplete
DEFAULT_HOST = '10.0.0.95'
DEFAULT_PORT = 1883
DEFAULT_SCALES = {
    'move_forward_scale':  3.0,
    'move_backward_scale': 3.0,
    'turn_left_scale':     1.8,
    'turn_right_scale':    1.8,
}


def load_config():
    cfg = configparser.ConfigParser()
    ini = os.path.join(os.path.dirname(__file__), 'rider_config.ini')
    cfg.read(ini)

    host = cfg.get('mqtt', 'broker_host', fallback=DEFAULT_HOST)
    port = cfg.getint('mqtt', 'broker_port', fallback=DEFAULT_PORT)

    scales = {}
    for key, default in DEFAULT_SCALES.items():
        scales[key] = cfg.getfloat('calibration', key, fallback=default)

    return host, port, scales


def send_command(host: str, port: int, command: dict):
    connected = False
    client = mqtt.Client(
        client_id=f"move_test_{int(time.time())}",
        callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
        protocol=mqtt.MQTTv5
    )

    def on_connect(c, userdata, flags, reason_code, props):
        nonlocal connected
        connected = True

    client.on_connect = on_connect
    client.connect(host, port, 60)
    client.loop_start()

    deadline = time.time() + 5.0
    while not connected and time.time() < deadline:
        time.sleep(0.05)

    if not connected:
        print(f"ERROR: could not connect to {host}:{port}")
        client.loop_stop()
        sys.exit(1)

    client.publish(TOPIC, json.dumps(command))
    time.sleep(0.2)
    client.loop_stop()
    client.disconnect()


def main():
    cfg_host, cfg_port, scales = load_config()

    parser = argparse.ArgumentParser(description='Robot movement test tool')
    parser.add_argument('--host', default=cfg_host, help='MQTT broker host')
    parser.add_argument('--port', type=int, default=cfg_port, help='MQTT broker port')
    parser.add_argument('--raw', action='store_true',
                        help='Send value as-is without scaling')

    sub = parser.add_subparsers(dest='action', required=True)

    move_p = sub.add_parser('move', help='Move forward/backward')
    move_p.add_argument('distance', type=int,
                        help='Distance in mm (positive=forward, negative=backward)')

    turn_p = sub.add_parser('turn', help='Turn left/right')
    turn_p.add_argument('angle', type=int,
                        help='Angle in degrees (positive=left, negative=right)')

    args = parser.parse_args()

    if args.action == 'move':
        value = args.distance
        if args.raw:
            scaled = value
            print(f"Move {'forward' if value >= 0 else 'backward'}: sending {scaled}mm (raw, no scaling)")
        else:
            scale = scales['move_forward_scale'] if value >= 0 else scales['move_backward_scale']
            scaled = int(value / scale)
            direction = 'forward' if value >= 0 else 'backward'
            print(f"Move {direction}: {abs(value)}mm ÷ {scale} = {scaled}mm sent to robot")
        command = {'action': 'move', 'distance': scaled, 'timestamp': time.time()}

    elif args.action == 'turn':
        value = args.angle
        if args.raw:
            scaled = value
            print(f"Turn {'left' if value >= 0 else 'right'}: sending {scaled}° (raw, no scaling)")
        else:
            scale = scales['turn_left_scale'] if value >= 0 else scales['turn_right_scale']
            scaled = int(value / scale)
            direction = 'left' if value >= 0 else 'right'
            print(f"Turn {direction}: {abs(value)}° ÷ {scale} = {scaled}° sent to robot")
        command = {'action': 'turn', 'angle': scaled, 'timestamp': time.time()}

    send_command(args.host, args.port, command)
    print("Sent.")


if __name__ == '__main__':
    main()
