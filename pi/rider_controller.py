#!/usr/bin/env python3
"""Rider DS4 controller -> MQTT command publisher (runs on the Pi).

Reads a paired PS4 DualShock 4 via pygame and publishes ESP32 line commands to
rider/control/line, which the bridge (rider_status_screen.py) relays to the ESP32.
Kept separate from the bridge so a controller hiccup can't disturb the balance
serial/telemetry -- the bridge stays the single serial owner.

Mapping:
  Left stick up/down  -> drive speed (push = move, release = stop and hold there)
  Right stick left/right -> turn rate
  Right stick up/down -> body HEIGHT (legs extend/retract at a rate set by the stick; firmware
                         clamps to the servo limits). Acts only when the stick is mostly vertical,
                         so a turn doesn't also change height.
  R3 (right stick click) -> height back to the normal ride height (firmware 'leghome', slow)
  Triangle            -> balance controller = RL POLICY  (firmware 'ctrl 0')  } only while balance
  Cross (X)           -> balance controller = LQR        (firmware 'ctrl 1')  } is OFF; LCD shows it
  Square (#)          -> toggle roll-leveling (IMU-roll differential legs; level 1/0)
  Circle (O)          -> reset position (poszero) without dropping balance
  Share               -> BALANCE / STOP: idle -> en 1; balancing -> en 0 + leglimp (legs limp too),
                         the same stop as the LCD upper-left button and the web page's E-STOP
  Options             -> toggle joystick <-> computer drive

Run normally:  /home/pi/xgovenv/bin/python rider_controller.py
Verify mapping: /home/pi/xgovenv/bin/python rider_controller.py --test
"""
import os, sys, time, json, signal, subprocess, wave
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
# pygame/SDL can't open this Pi's ALSA "default" device (error 524); we beep via aplay to the wm8960
# by name instead, and disable pygame audio so it neither errors nor grabs a device.
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")
os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
import pygame
import paho.mqtt.client as mqtt

_running = True


def _shutdown(*_a):
    global _running
    _running = False

BROKER, PORT = "localhost", 1883
TOPIC_CMD, TOPIC_STATUS = "rider/control/line", "rider/status"
TOPIC_FWDLIMIT = "rider/safety/fwd_limit"  # ToF safety governor -> forward-speed factor (rider_tof_safety.py)
TOPIC_MAXSPEED = "rider/control/maxspeed"  # RETAINED {"mps": v}: full-stick speed, set by the web page's slider.
                                           # Scales the whole stick range (stays proportional); capped at MAX_SPEED
                                           # (= the firmware's gPosVmax) -- going faster needs both raised.
MIN_SPEED = 0.05

# --- mapping (verify indices with --test, then adjust) ---
AXIS_DRIVE = 1       # left stick Y (stick up = negative)
AXIS_TURN = 3        # right stick X (verify with --test)
AXIS_HEIGHT = 4      # right stick Y (stick up = negative). 6-axis DS4 under hid_playstation: 0 LX, 1 LY,
                     # 2 L2, 3 RX, 4 RY, 5 R2 (3 = RX verified; 4 = RY verified with --test 2026-10-18)
BTN_BALANCE     = 8  # Share        -- toggle balance (en 1/en 0); off = stop. (moved to Share 2026-06-24; VERIFY index w/ --test)
BTN_CTRL_POLICY = 2  # Triangle    -- balance controller = RL policy ('ctrl 0'); only while balance is off
BTN_CTRL_LQR    = 0  # Cross (X)   -- balance controller = LQR ('ctrl 1');       only while balance is off
BTN_DISTZERO    = 1  # Circle (O)  -- poszero / reset position, balance stays on (Cross=0,Circle=1,Triangle=2,Square=3 verified --test 2026-06-16)
BTN_LEVEL       = 3  # Square       -- toggle roll-leveling (IMU-roll differential legs: level 1/0)
BTN_MODE        = 9  # Start/Options -- toggles joystick<->computer drive
BTN_HEIGHT_HOME = 12 # R3 (right stick click) -- legs back to the normal ride height ('leghome', slow).
                     # 13-button DS4 under hid_playstation: ... 10 PS, 11 L3, 12 R3 (VERIFY with --test)
HEIGHT_RATE     = 50.0  # leg encoder counts/s at full right-stick-vertical (leg travel ~95 counts -> ~2 s
                        # end to end). Sent as 'legrate'; firmware integrates it, clamps to the servo limits.
HEIGHT_HZ       = 10    # 'legrate' heartbeat while the stick is held (firmware times out after 300 ms)
HEIGHT_DEADZONE = 0.40  # same as the turn deadzone: the right stick is worn (rest offset ~0.32 on X)
DEADZONE = 0.20       # left stick (drive). 0.12 -> 0.20 (2026-10-19): the worn left stick now rests ~0.15 off centre,
                      # past 0.12 -> a steady 0.01 m/s command crept the robot forward ~0.5 m a minute.
TURN_DEADZONE = 0.40  # right stick (turn) resting offset has DRIFTED to ~0.32 (worn stick), past the old 0.30
                      # deadzone -> it kept commanding turnrate 0.03 and, with no heartbeat to refresh it, that
                      # value LATCHED in the firmware (robot turned on a centered stick). Widened to 0.40 to
                      # reject the worn-stick rest; the turn heartbeat below now clears/refreshes like drive.
MAX_SPEED = 0.30     # m/s at full stick (and the web slider's ceiling). ->0.30 (2026-10-19, Marc: "a little faster";
                     # was 0.25). Must match the firmware's gPosVmax/gPosVmaxBack ('posvmax'/'posvmaxb'): set live to
                     # 0.30 for the trial -- their compiled default is still 0.25, so after an ESP32 reboot the top
                     # of the stick clips until the firmware default is raised too.
MAX_YAW_RATE = 1.0   # rad/s commanded at full right-stick (firmware closes the loop on the gyro)
TURN_SIGN = 1        # flip if right-stick-right turns the wrong way
SEND_HZ = 20
LOOP_HZ = 50

BEEP_WAV = "/home/pi/beep.wav"
# Address the wm8960 by NAME, not card number -- ALSA indices reshuffle across reboots/kernel updates.
BEEP_DEV = "plughw:CARD=wm8960soundcard,DEV=0"
BEEP_GAP_S = 0.12   # silence between beeps. One aplay per beep put each aplay start-up in the gap (slow, uneven);
                    # n beeps are now ONE wav (built once per n, cached in /tmp) played by ONE aplay.
def beep(n):
    """n short beeps, non-blocking: 1 = joystick mode, 2 = computer mode, 3 = boot ready."""
    try:
        path = "/tmp/rider_beep_%d.wav" % n
        if not os.path.exists(path):
            with wave.open(BEEP_WAV) as w:
                params, tone = w.getparams(), w.readframes(w.getnframes())
            gap = b"\0" * (int(BEEP_GAP_S * params.framerate) * params.sampwidth * params.nchannels)
            with wave.open(path, "wb") as o:
                o.setparams(params)
                o.writeframes(gap.join([tone] * n))
        subprocess.Popen(["aplay", "-q", "-D", BEEP_DEV, path],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception:
        pass

state = {"en": 0, "pos": 0.0, "fwd_factor": 1.0,   # fwd_factor: ToF safety cap (1=clear..0=stop)
         "max_speed": MAX_SPEED}                     # full-stick speed (m/s), from TOPIC_MAXSPEED


def on_message(client, userdata, msg):
    try:
        p = json.loads(msg.payload.decode())
    except Exception:
        return
    if msg.topic == TOPIC_FWDLIMIT:                 # ToF governor: forward-speed factor
        try:
            state["fwd_factor"] = max(0.0, min(1.0, float(p["factor"])))
        except Exception:
            pass
        return
    if msg.topic == TOPIC_MAXSPEED:                 # web page's top-speed slider (retained)
        try:
            state["max_speed"] = max(MIN_SPEED, min(MAX_SPEED, float(p["mps"])))
            print("top speed -> %.2f m/s" % state["max_speed"], flush=True)
        except Exception:
            pass
        return
    if "roll_balance_enabled" in p:
        state["en"] = 1 if p["roll_balance_enabled"] else 0
    if "position" in p:
        try:
            state["pos"] = float(p["position"])
        except Exception:
            pass


def main():
    test = "--test" in sys.argv
    signal.signal(signal.SIGTERM, _shutdown)
    signal.signal(signal.SIGINT, _shutdown)

    mqc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="rider_controller")
    mqc.on_message = on_message
    # subscribe in on_connect so the balance-state sync (-> drive gate) survives the
    # initial CONNACK race AND any reconnect. A subscribe before loop_start/CONNACK is
    # dropped by the broker -> state["en"] stuck 0 -> drive/turn silently suppressed.
    def _on_connect(c, u, flags, rc, properties=None):
        c.subscribe(TOPIC_STATUS)
        c.subscribe(TOPIC_FWDLIMIT)
        c.subscribe(TOPIC_MAXSPEED)
    mqc.on_connect = _on_connect
    mqc.reconnect_delay_set(min_delay=1, max_delay=10)
    mqc.connect(BROKER, PORT, keepalive=30)
    mqc.loop_start()

    def send(line):
        mqc.publish(TOPIC_CMD, json.dumps({"line": line}), qos=0)

    def pub_mode(m):
        mqc.publish("rider/control/drivemode", json.dumps({"mode": m}), qos=0, retain=True)

    pygame.init()
    pygame.joystick.init()
    beep(3)          # boot-ready signal: 3 beeps once the controller is up
    js = None

    def ensure_js():
        nonlocal js
        if pygame.joystick.get_count() > 0:
            js = pygame.joystick.Joystick(0); js.init()
            print("joystick:", js.get_name(), "axes", js.get_numaxes(), "buttons", js.get_numbuttons(), flush=True)
            return True
        js = None
        return False

    last_vel = None
    prev_btn = {}
    last_send = 0.0
    last_turn = None
    last_turn_send = 0.0
    drive_mode = "computer"   # BOOT in computer mode (safe default). 'joystick' = stick drives; press Start to opt in.
    lev_on = False            # local roll-leveling toggle (firmware has no level-state feedback; track it here)
    last_height_rate = 0.0    # last height rate sent (counts/s)
    last_height_send = 0.0
    period = 1.0 / LOOP_HZ
    send_period = 1.0 / SEND_HZ

    def dz(v, deadzone=DEADZONE):    # deadzone + rescale to full range
        if abs(v) < deadzone:
            return 0.0
        return (v - (deadzone if v > 0 else -deadzone)) / (1 - deadzone)

    time.sleep(0.5); pub_mode(drive_mode)   # boot in computer mode -> tell the LCD

    while _running:
        # process the event queue: keeps axis/button state fresh AND handles hotplug --
        # when the controller sleeps/wakes/recharges, REMOVED/ADDED fire and we rebind to
        # the live device instead of clinging to a dead js0 handle.
        for ev in pygame.event.get():
            if ev.type in (pygame.JOYDEVICEADDED, pygame.JOYDEVICEREMOVED):
                js = None
        if js is None:
            if not ensure_js():
                if drive_mode == "joystick":          # joystick gone -> fall back to computer
                    drive_mode = "computer"; pub_mode(drive_mode); beep(2)
                    send("dv 0"); send("turnrate 0")  # and STOP: the firmware holds the last drive command
                                                      # (2026-10-20: a controller that died mid-drive left dv 0.30)
                    print("joystick disconnected -> computer mode", flush=True)
                time.sleep(1.0); continue
        try:
            naxes, nbtn = js.get_numaxes(), js.get_numbuttons()
            axes = [js.get_axis(i) for i in range(naxes)]
            buttons = [js.get_button(i) for i in range(nbtn)]
        except pygame.error:
            print("joystick lost; waiting for reconnect", flush=True)
            js = None; continue

        if test:
            pressed = [i for i, b in enumerate(buttons) if b]
            print("axes:", [round(a, 2) for a in axes], " buttons pressed:", pressed, flush=True)
            time.sleep(0.15); continue

        now = time.time()

        def edge(idx):
            cur = buttons[idx] if idx < len(buttons) else 0
            return cur and not prev_btn.get(idx, 0)

        if edge(BTN_BALANCE):                  # Share: BALANCE / STOP
            if state["en"]:
                send("en 0"); send("leglimp")  # STOP = balance off + legs limp -- same as the LCD button + web E-STOP
            else:
                send("en 1")                       # firmware homes the target on enable
        for btn, c, name in ((BTN_CTRL_POLICY, 0, "POLICY"), (BTN_CTRL_LQR, 1, "LQR")):
            if edge(btn):                      # Triangle = policy, Cross = LQR
                if state["en"]:                # the firmware refuses a swap mid-balance; don't even ask
                    print("ctrl %s ignored: balance is on (Share to stop first)" % name, flush=True)
                else:
                    send("ctrl %d" % c)        # LCD shows the active controller from telemetry
                    print("ctrl -> %s" % name, flush=True)
        if edge(BTN_HEIGHT_HOME):              # R3: back to the normal ride height, slowly (balancing or not)
            send("leghome")
        if edge(BTN_DISTZERO):                 # Circle: re-zero distance / reset position, balance stays on
            send("poszero")
        if edge(BTN_LEVEL):                    # Square: toggle IMU-roll leg leveling
            lev_on = not lev_on
            send("level %d" % (1 if lev_on else 0))
        if edge(BTN_MODE):                     # Start/Options toggles drive source
            drive_mode = "computer" if drive_mode == "joystick" else "joystick"
            send("dv 0"); send("turnrate 0")   # clear any stick command on the switch
            beep(2 if drive_mode == "computer" else 1)   # 1 beep=joystick, 2=computer
            pub_mode(drive_mode)               # -> LCD shows the active drive source
            print("drive_mode -> %s" % drive_mode, flush=True)
        prev_btn = {i: buttons[i] for i in range(len(buttons))}

        # body height (right stick Y) = a RATE the firmware integrates every control cycle into 1-count
        # leg moves (smooth). Any mode/balance state, like the old leg buttons. Only when the stick is
        # mostly VERTICAL, so a sideways turn push doesn't also move the legs. Sent on change (incl. the
        # return to 0) plus a heartbeat while moving; the firmware stops the legs if the heartbeat stops.
        hy = -axes[AXIS_HEIGHT] if AXIS_HEIGHT < len(axes) else 0.0      # stick up = positive = body up
        hx = axes[AXIS_TURN] if AXIS_TURN < len(axes) else 0.0
        h = dz(hy, HEIGHT_DEADZONE) if abs(hy) > 1.5 * abs(hx) else 0.0
        rate = round(h * HEIGHT_RATE, 1)
        if ((rate != last_height_rate) or (rate != 0.0 and now - last_height_send >= 1.0 / HEIGHT_HZ)) \
                and now - last_height_send >= send_period:
            last_height_send = now
            last_height_rate = rate
            send("legrate %.1f" % rate)                                 # counts/s: + up / - down

        # drive (left stick Y) + turn (right stick X): only while balancing AND in joystick mode.
        # In computer mode the controller stays silent so MQTT/autonomy drive commands aren't stomped.
        if state["en"] and drive_mode == "joystick":
            # VELOCITY drive: the stick commands a SPEED (m/s), not an accumulating position.
            # Firmware drives at this speed while non-zero and latches the current position as
            # the hold-home the instant it returns to 0 (-> drives smooth, stops where you let
            # go, no windup/overshoot). 'ptgt' position moves are still honored separately.
            d = dz(-axes[AXIS_DRIVE]) if AXIS_DRIVE < len(axes) else 0.0   # stick up = forward
            vel = d * state["max_speed"]                # full stick = the web page's top speed
            if vel > 0.0:                       # ToF safety governor: cap FORWARD only (reverse = escape)
                vel *= state["fwd_factor"]
            # send on meaningful change (incl. return to 0) OR as a periodic heartbeat
            if ((last_vel is None or abs(vel - last_vel) > 0.01) or
                    (now - last_send >= 0.5)) and now - last_send >= send_period:
                last_send = now
                last_vel = vel
                send("dv %.3f" % vel)
            # turn: right stick X -> commanded yaw rate (rad/s); send on change incl. return to 0
            tn = dz(axes[AXIS_TURN], TURN_DEADZONE) if AXIS_TURN < len(axes) else 0.0
            yaw_cmd = TURN_SIGN * tn * MAX_YAW_RATE
            # send on meaningful change (incl. all the way to 0) OR a 0.5s heartbeat -- the SAME logic as
            # drive above. The old change>0.05-only rule with no heartbeat let a small turn value LATCH in
            # the firmware and never clear -> the robot turned on its own with the stick centered.
            if ((last_turn is None or abs(yaw_cmd - last_turn) > 0.01) or
                    (now - last_turn_send >= 0.5)) and now - last_turn_send >= send_period:
                last_turn_send = now
                last_turn = yaw_cmd
                send("turnrate %.2f" % yaw_cmd)
        else:
            last_vel = None
            last_turn = None

        time.sleep(period)

    # clean exit (leave the robot's balance state alone -- the bridge keeps it running)
    try:
        mqc.loop_stop(); mqc.disconnect()
    except Exception:
        pass


if __name__ == "__main__":
    main()
