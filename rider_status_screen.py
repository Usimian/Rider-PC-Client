#!/usr/bin/env python3
"""Rider Pi bridge — LCD status screen + MQTT republish + command relay.

Single owner of /dev/ttyAMA0 (the ESP32 balance-fw line protocol: th/roll/wx/...).
Three jobs in one process (must be one — only one thing can own the serial port):

  1. LCD  : render live balance telemetry on the XGO 2-inch display.
  2. PUB  : republish telemetry over MQTT for the workstation GUI
            (rider/status + rider/status/imu), broker = local mosquitto.
  3. RELAY: subscribe to rider/control/* and forward to the ESP32 as line
            commands (rider/control/line {"line":"en 1"} -> 'en 1\\n';
            rider/control/system emergency_stop -> 'en 0').

Run:  /home/pi/xgovenv/bin/python rider_status_screen.py
"""
import time
import re
import json
import queue
import serial
import psutil
import lgpio
import paho.mqtt.client as mqtt
import xgoscreen.LCD_2inch as LCD_2inch
from PIL import Image, ImageDraw, ImageFont

PORT = "/dev/ttyAMA0"
FONT = "/home/pi/model/msyh.ttc"
BG = (15, 21, 46)
BROKER = "localhost"
BROKER_PORT = 1883

f_s = ImageFont.truetype(FONT, 15)
f_m = ImageFont.truetype(FONT, 20)
f_l = ImageFont.truetype(FONT, 30)

lcd = LCD_2inch.LCD_2inch()
lcd.Init()
lcd.clear()
ser = serial.Serial(PORT, 115200, timeout=0.1)

# physical buttons on the XGO 2-inch board (active-low, pull-up). C = upper-left,
# next to the RIDER text -> balance start/stop toggle.
BTN_BALANCE = 17                # GPIO17 = C button (upper-left)
try:
    _chip = lgpio.gpiochip_open(0)
    lgpio.gpio_claim_input(_chip, BTN_BALANCE, lgpio.SET_PULL_UP)
    btn_ok = True
except Exception as e:
    print("button GPIO unavailable: %s" % e, flush=True)
    btn_ok = False
btn_prev = 1                    # 1 = released (pulled up)
btn_last = 0.0                  # last-press time (debounce)

tel = {}
cmd_q = queue.Queue()           # inbound MQTT -> ESP32 line commands


def parse(line):
    for k, v in re.findall(r"(\w+)=(-?[\d.]+)", line):
        tel[k] = float(v)


def cpu_temp_c():
    try:
        return float(open("/sys/class/thermal/thermal_zone0/temp").read()) / 1000.0
    except Exception:
        return float("nan")


# ---------------- MQTT (republish + relay) ----------------
def on_connect(client, userdata, flags, reason_code, properties=None):
    for t in ("rider/control/line", "rider/control/system",
              "rider/control/movement", "rider/control/settings"):
        client.subscribe(t)


def on_message(client, userdata, msg):
    try:
        p = json.loads(msg.payload.decode())
    except Exception:
        p = {}
    t = msg.topic
    if t == "rider/control/line":
        line = p.get("line") or p.get("cmd")
        if line:
            cmd_q.put(str(line))
    elif t == "rider/control/system":
        c = (p.get("command") or p.get("action") or "").lower()
        if c in ("emergency_stop", "stop", "disable"):
            cmd_q.put("en 0")
        elif c in ("enable", "balance_on"):
            cmd_q.put("en 1")
    # movement/settings: logged only for now (full mapping is the GUI-reconcile step)
    else:
        print("relay: unmapped %s %s" % (t, p), flush=True)


mqc = None
try:
    mqc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="rider_bridge")
    mqc.on_connect = on_connect
    mqc.on_message = on_message
    mqc.will_set("rider/client/disconnect",
                 json.dumps({"source": "rider_bridge"}), retain=False)
    mqc.connect(BROKER, BROKER_PORT, keepalive=30)
    mqc.loop_start()
except Exception as e:
    print("MQTT unavailable (%s) -- screen-only" % e, flush=True)
    mqc = None


def publish():
    if mqc is None:
        return
    en = int(tel.get("en", 0)); pol = int(tel.get("polrun", 0))
    th = tel.get("th", 0.0); roll = tel.get("roll", 0.0); yaw = tel.get("yaw", 0.0)
    batt = int(tel.get("batt", 0)); vbat = round(tel.get("vbat", 0.0), 2)
    status = {
        "controller_connected": True,
        "connection_status": "connected",
        "roll_balance_enabled": bool(en),
        "battery_level": batt,
        "battery_voltage": vbat,
        "cpu_percent": psutil.cpu_percent(),
        "cpu_temp": round(cpu_temp_c(), 1),
        "loop_hz": int(tel.get("lhz", 0)),
        "fault": int(tel.get("fault", 0)),
        "rfail": tel.get("rfail", 0.0),
        "mode": "policy" if pol else "pid",
        "position": tel.get("wx", 0.0),
        "target": tel.get("ptgt", 0.0),
        # NOTE: roll/pitch/yaw are published ONLY on rider/status/imu, not here.
        # Duplicating them into rider/status makes the GUI's update_status() write
        # them first, which starves update_imu()'s change-gate -> IMU panel never fires.
    }
    try:
        mqc.publish("rider/status", json.dumps(status))
        mqc.publish("rider/status/imu",
                    json.dumps({"roll": roll, "pitch": th, "yaw": yaw}))
        mqc.publish("rider/status/battery",
                    json.dumps({"level": batt, "voltage": vbat}))
    except Exception:
        pass


# ---------------- LCD render ----------------
def render():
    img = Image.new("RGB", (320, 240), BG)
    d = ImageDraw.Draw(img)
    en = int(tel.get("en", 0))
    pol = int(tel.get("polrun", 0))
    mq_ok = mqc is not None and mqc.is_connected()

    d.text((10, 6), "RIDER", fill=(255, 255, 255), font=f_l)
    # MQTT link dot
    d.ellipse((92, 18, 104, 30), fill=(0, 200, 120) if mq_ok else (110, 110, 110))
    # battery (header, between title and badge)
    batt = int(tel.get("batt", 0)); vbat = tel.get("vbat", 0.0)
    bcol = (0, 255, 140) if batt > 40 else (255, 200, 70) if batt > 15 else (255, 90, 90)
    d.text((112, 8), "%d%%" % batt, fill=bcol, font=f_m)
    d.text((112, 28), "%.1fV" % vbat, fill=(150, 165, 205), font=f_s)
    state = "BALANCING" if en else "IDLE"
    col = (0, 255, 140) if en else (150, 150, 150)
    d.rectangle((176, 10, 312, 44), outline=col, width=2)
    d.text((188, 14), state, fill=col, font=f_m)

    th = tel.get("th", 0.0); roll = tel.get("roll", 0.0); yaw = tel.get("yaw", 0.0)
    for x, lbl, val in ((10, "tilt", th), (112, "roll", roll), (214, "yaw", yaw)):
        d.text((x, 52), lbl, fill=(150, 165, 205), font=f_s)
        d.text((x, 70), "%+.1f°" % val, fill=(255, 255, 255), font=f_m)

    wx = tel.get("wx", 0.0); tg = tel.get("ptgt", 0.0); err = (wx - tg) * 1000.0
    d.text((10, 116), "position  /  target  (m)", fill=(150, 165, 205), font=f_s)
    d.text((10, 134), "%+.3f  →  %+.3f" % (wx, tg), fill=(255, 255, 255), font=f_m)
    ecol = (255, 200, 70) if abs(err) > 30 else (0, 255, 140)
    d.text((10, 160), "err %+.0f mm" % err, fill=ecol, font=f_m)

    rf = int(tel.get("rfail", 0)); flt = int(tel.get("fault", 0))
    hz = int(tel.get("lhz", 0)); mode = "POLICY" if pol else "PID"
    hcol = (255, 90, 90) if (rf > 5 or flt) else (150, 165, 205)
    d.text((10, 192), "rfail %d%%   fault %d   %dHz   %s" % (rf, flt, hz, mode), fill=hcol, font=f_s)

    cpu = psutil.cpu_percent(); temp = cpu_temp_c()
    tcol = (255, 90, 90) if temp >= 70 else (170, 195, 235)
    d.text((10, 214), "CPU %d%%" % cpu, fill=(170, 195, 235), font=f_m)
    d.text((170, 214), "%.0f°C" % temp, fill=tcol, font=f_m)
    lcd.ShowImage(img)


last_render = 0.0
last_pub = 0.0
psutil.cpu_percent()  # prime
while True:
    line = ser.readline().decode(errors="replace").strip()
    if line.startswith("th="):
        parse(line)
        # high-rate raw-telemetry republish for untethered diagnostics (~ESP32 rate)
        if mqc is not None:
            try:
                mqc.publish("rider/debug/telem", line)
            except Exception:
                pass
    # relay any queued commands to the ESP32
    while not cmd_q.empty():
        try:
            ser.write((cmd_q.get_nowait() + "\n").encode())
        except Exception:
            break
    # C button (upper-left, GPIO17): toggle the balance policy on each press
    if btn_ok:
        lvl = lgpio.gpio_read(_chip, BTN_BALANCE)
        tnow = time.time()
        if btn_prev == 1 and lvl == 0 and tnow - btn_last > 0.4:   # falling edge + debounce
            btn_last = tnow
            if int(tel.get("en", 0)) == 1:
                ser.write(b"en 0\n")                                # balancing -> stop
            else:
                ser.write(b"polrun 1\nen 1\n")                      # idle -> arm policy + enable
        btn_prev = lvl
    now = time.time()
    if now - last_render >= 0.2:        # LCD ~5 Hz
        last_render = now
        render()
    if now - last_pub >= 0.3:           # MQTT ~3 Hz
        last_pub = now
        publish()
