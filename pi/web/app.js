// Rider control page -- step 2: status bar, pose, footer stats, E-STOP (docs/WEB_UI_DESIGN.md).
// Rules: the page holds no robot state (everything shown comes from the robot's messages) and
// sends nothing on load or close; the only command here is E-STOP.
"use strict";
const $ = (id) => document.getElementById(id);
const set = (id, text, cls) => { const e = $(id); e.textContent = text; if (cls !== undefined) e.className = cls; };
const deg = (v) => (v === undefined || v === null) ? "–" : `${Number(v).toFixed(1)}°`;
const m = (v) => (v === undefined || v === null) ? "–" : `${Number(v).toFixed(3)} m`;

let lastStatus = 0;

const client = new MqttLite("ws://" + location.host + "/", {
  onConnect: () => set("linktext", "connected"),
  onClose:   () => set("linktext", "reconnecting…"),
  onMessage: (topic, payload) => {
    let p; try { p = JSON.parse(payload); } catch { return; }
    if (topic === "rider/status") status(p);
    else if (topic === "rider/status/imu") { set("pitch", deg(p.pitch)); set("roll", deg(p.roll)); }
    else if (topic === "rider/status/odom") set("heading", deg(p.heading));
    else if (topic === "rider/control/drivemode") {
      const j = p.mode === "joystick";
      set("drive", j ? "JOYSTICK" : "COMPUTER", j ? "on" : "warn");
    }
  },
});
for (const t of ["rider/status", "rider/status/imu", "rider/status/odom", "rider/control/drivemode"]) client.subscribe(t);

function status(s) {
  lastStatus = Date.now();
  $("raw").textContent = JSON.stringify(s, null, 2);
  const lowBatt = s.battery_level < 15;
  set("batt", `${s.battery_level} %  ${Number(s.battery_voltage).toFixed(2)} V`, lowBatt ? "num bad" : "num");
  set("ds4", s.controller_connected ? "connected" : "not connected", s.controller_connected ? "on" : "off");
  set("ctrl", s.mode === "POL" ? "POLICY" : s.mode, s.mode === "POL" ? "pol" : "lqr");
  set("bal", s.roll_balance_enabled ? "ON" : "off", s.roll_balance_enabled ? "on" : "off");
  set("lev", s.roll_leveling_enabled ? "ON" : "off", s.roll_leveling_enabled ? "on" : "off");
  set("pos", m(s.position)); set("tgt", m(s.target));
  set("xy", `${Number(s.pose_x).toFixed(2)}, ${Number(s.pose_y).toFixed(2)} m`);
  set("cpu", `${Math.round(s.cpu_percent)} %`);
  set("temp", `${Number(s.cpu_temp).toFixed(0)} °C`, s.cpu_temp >= 80 ? "num bad" : s.cpu_temp >= 70 ? "num warn" : "num");
  set("pw", `${Number(s.power_w).toFixed(1)} W`);
  set("hz", `${s.loop_hz} Hz`);
  set("rfail", `${Number(s.rfail).toFixed(1)} %`, s.rfail > 0 ? "num warn" : "num");
  set("fault", s.fault ? `wheel ${s.fault} off bus` : "none", s.fault ? "bad" : "off");
}

// Link = live status within the last 2 s (design rule), not merely an open socket. E-STOP is
// usable whenever the socket is up -- even with stale status, a stop must still be sendable.
setInterval(() => {
  $("link").classList.toggle("ok", Date.now() - lastStatus < 2000);
  $("estop").disabled = !client.connected;
}, 250);

// E-STOP = the LCD stop button: balance off, then legs limp. Two lines on the same topic keep
// their order through the bridge's queue. No confirm: it's an emergency control.
$("estop").addEventListener("click", () => {
  const a = client.publish("rider/control/line", JSON.stringify({ line: "en 0" }));
  const b = client.publish("rider/control/line", JSON.stringify({ line: "leglimp" }));
  const btn = $("estop");
  btn.textContent = (a && b) ? "STOP SENT" : "NOT SENT";
  btn.classList.add("sent");
  setTimeout(() => { btn.textContent = "E-STOP"; btn.classList.remove("sent"); }, 1500);
});
