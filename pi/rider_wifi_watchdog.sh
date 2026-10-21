#!/usr/bin/env bash
# rider-wifi-watchdog: notice when the Pi's WiFi stops working, bring it back, and SAY SO.
#
# Why (2026-10-21): mid-run, the WiFi went dead silently -- no driver error, no NetworkManager
# disconnect -- while Bluetooth, the services and the controller kept working. Nothing noticed, so
# nothing fixed it; only a power-cycle did, and that threw away the live state of the dead link.
#
# What it does, every CHECK_S seconds: ping the router over wlan0.
#   - DOWN_AFTER misses in a row          -> DOWN event (logged with the time of the first miss)
#   - down for RESTART_AFTER_S             -> snapshot the WiFi's live state to a file, restart the
#                                             WiFi driver, then tell NetworkManager to connect;
#                                             again every RETRY_S while still down
#   - the router answers again             -> UP event with the outage length and what fixed it
#   - the Pi restarted while down           -> UNRESOLVED event on the next start
# Every event is one line in EVENTS (persistent, never trimmed), in the journal, and in the retained
# MQTT topic rider/wifi (the web page's fault item shows today's dropouts until midnight). Nothing is
# fixed quietly.
# Bash, not Python: every step is a system command (ping, iw, nmcli, modprobe, mosquitto_pub).
set -u

EVENTS=/home/pi/wifi_events.log
SNAPDIR=/home/pi/wifi_snapshots
CHECK_S=5
DOWN_AFTER=3            # misses in a row (~15 s) before an outage is declared
RESTART_AFTER_S=30      # down this long -> snapshot + driver restart
RETRY_S=120             # still down -> restart again this often
BOOT_GRACE_S=60         # at start, WiFi gets this long to come up before it counts as an outage

now()  { date +%s; }
hms()  { date -d "@$1" +%H:%M:%S; }

event() {               # one line per event: file + journal (stdout), then refresh the MQTT summary
  local line; line="$(date '+%F %T') $*"
  echo "$line" >> "$EVENTS"
  echo "$line"
  publish
}

publish() {             # retained summary for the web page; the broker is local, so this works while WiFi is down
  local today n last
  today=$(date +%F)
  n=$(grep -c "^$today .* DOWN " "$EVENTS" 2>/dev/null); n=${n:-0}
  last=$(grep "^$today " "$EVENTS" 2>/dev/null | grep -E ' (DOWN|RESTART|UP|UNRESOLVED) ' | tail -n 1 | sed 's/"/'"'"'/g')
  mosquitto_pub -q 1 -r -t rider/wifi \
    -m "{\"state\":\"$state\",\"dropouts_today\":$n,\"last_event\":\"$last\"}" >/dev/null 2>&1
}

gateway() { ip route show default dev wlan0 2>/dev/null | awk '{print $3; exit}'; }

snapshot() {            # the live state we lost on 2026-10-21 -- captured BEFORE touching anything
  mkdir -p "$SNAPDIR"
  local f; f="$SNAPDIR/$(date +%F_%H%M%S).txt"
  {
    echo "=== $(date '+%F %T')  down since $(hms "$down_since")  router $gw"
    echo "--- vcgencmd get_throttled (0x50005 = under-voltage now + since boot)"; vcgencmd get_throttled
    echo "--- ip addr wlan0";              ip addr show wlan0
    echo "--- ip route";                   ip route
    echo "--- iw link";                    iw dev wlan0 link
    echo "--- iw station dump";            iw dev wlan0 station dump
    echo "--- nmcli device";               nmcli -f DEVICE,TYPE,STATE,CONNECTION device
    echo "--- nmcli wlan0";                nmcli -f GENERAL,WIFI-PROPERTIES,IP4 device show wlan0
    echo "--- ping router x3";             ping -c 3 -W 1 -I wlan0 "$gw"
    echo "--- arp";                        ip neigh show dev wlan0
    echo "--- kernel log, last 10 min";    journalctl -k --since -10min --no-pager
    echo "--- NetworkManager + wpa_supplicant, last 10 min"
    journalctl -u NetworkManager -u wpa_supplicant --since -10min --no-pager
  } > "$f" 2>&1
  echo "$f"
}

restart_driver() {      # Bluetooth is a separate driver (hci_uart) and is not touched
  modprobe -r brcmfmac_cyw brcmfmac
  sleep 2
  modprobe brcmfmac
  sleep 5               # let the new wlan0 + its supplicant come up
}

reconnect() {           # NetworkManager may hold the connection off after the outage (it did after a
                        # 'disconnect' on 2026-10-21: new device, autoconnect never fired) -- ask it outright.
                        # But first give its own autoconnect 15 s: it picks by priority (home before the phone
                        # hotspot), while 'device connect' takes the last-used network (it moved home -> phone).
  local i c
  for i in $(seq 15); do
    c=$(nmcli -t -f DEVICE,STATE,CONNECTION device | awk -F: '$1=="wlan0" && $2=="connected" {print $3}')
    [ -n "$c" ] && { echo "NetworkManager reconnected on its own ($c)"; return; }
    sleep 1
  done
  nmcli --wait 30 device connect wlan0 2>&1 | sed -E 's/\x1b\[[0-9;]*[A-Za-z]//g' | tr '\n' ' '   # drop nmcli's progress-line escapes
}

# ---- start: was the last boot left with the WiFi down? ----
touch "$EVENTS"
lastkind=$(grep -E ' (DOWN|UP|UNRESOLVED) ' "$EVENTS" | tail -n 1 | awk '{print $3}')
state=starting
if [ "$lastkind" = "DOWN" ]; then
  event "UNRESOLVED the previous outage never recovered -- the Pi was restarted while the WiFi was down"
fi

start=$(now); gw=$(gateway); misses=0; down_since=0; first_miss=0; last_restart=0; restarts=0
day=$(date +%F)
while true; do
  [ "$(date +%F)" != "$day" ] && { day=$(date +%F); publish; }   # new day -> the page's dropout count resets
  g=$(gateway); [ -n "$g" ] && gw=$g
  if [ -n "$gw" ] && ping -c 1 -W 2 -I wlan0 "$gw" >/dev/null 2>&1; then
    if [ "$state" = starting ]; then
      state=up; event "START WiFi up $(( $(now) - start )) s after the watchdog started (router $gw)"
    elif [ "$state" = down ]; then
      how="came back on its own"; [ "$restarts" -gt 0 ] && how="back after $restarts WiFi driver restart(s)"
      state=up; event "UP after $(( $(now) - down_since )) s down -- $how"
    fi
    misses=0; restarts=0
  else
    misses=$((misses + 1)); [ "$misses" -eq 1 ] && first_miss=$(now)
    t=$(now)
    if [ "$state" = up ] && [ "$misses" -ge "$DOWN_AFTER" ]; then
      state=down; down_since=$first_miss
      event "DOWN no reply from the router (${gw:-no route}) since $(hms "$down_since")"
    elif [ "$state" = starting ] && [ $((t - start)) -ge "$BOOT_GRACE_S" ]; then
      state=down; down_since=$start
      event "DOWN no WiFi $BOOT_GRACE_S s after the watchdog started (boot)"
    fi
    if [ "$state" = down ] && [ $((t - down_since)) -ge "$RESTART_AFTER_S" ] && [ $((t - last_restart)) -ge "$RETRY_S" ]; then
      f=$(snapshot)
      event "RESTART WiFi driver after $((t - down_since)) s down (snapshot $f)"
      restart_driver
      event "RECONNECT asked NetworkManager to connect wlan0: $(reconnect)"
      last_restart=$(now); restarts=$((restarts + 1))
    fi
  fi
  sleep "$CHECK_S"
done
