// Rider control page -- steps 2-4: status bar, pose, stats, E-STOP, balance card, legs (docs/WEB_UI_DESIGN.md).
// Rules: the page holds no robot state -- every control shows and decides from the robot's own
// messages (the top speed is a RETAINED broker message, so every open page sees the same value) --
// and it sends nothing on load or close.
"use strict";
const $ = (id) => document.getElementById(id);
const set = (id, text, cls) => { const e = $(id); e.textContent = text; if (cls !== undefined) e.className = cls; };
const deg = (v) => (v === undefined || v === null) ? "–" : `${Number(v).toFixed(1)}°`;
const m = (v) => (v === undefined || v === null) ? "–" : `${Number(v).toFixed(3)} m`;

let lastStatus = 0;
let S = null;                       // latest rider/status (robot truth); controls decide from it
let speedRetained = null;           // latest retained rider/control/maxspeed (m/s), null = never set
const DEFAULT_SPEED = 0.30;         // the joystick service's own default (MAX_SPEED) when nothing is retained
const line = (l) => client.publish("rider/control/line", JSON.stringify({ line: l }));

const client = new MqttLite("ws://" + location.host + "/", {
  onConnect: () => set("linktext", "connected"),
  onClose:   () => set("linktext", "reconnecting…"),
  onMessage: (topic, payload) => {
    if (topic === "rider/debug/telem") { height(payload); return; }   // raw "k=v k=v" line, not JSON
    let p; try { p = JSON.parse(payload); } catch { return; }
    if (topic === "rider/status") status(p);
    else if (topic === "rider/status/imu") { set("pitch", deg(p.pitch)); set("roll", deg(p.roll)); }
    else if (topic === "rider/status/odom") set("heading", deg(p.heading));
    else if (topic === "rider/control/drivemode") {
      const j = p.mode === "joystick";
      set("drive", j ? "JOYSTICK" : "COMPUTER", j ? "on" : "warn");
    }
    else if (topic === "rider/control/maxspeed") {
      speedRetained = Number(p.mps);
      if (!speedDragging) showSpeed(speedRetained);
    }
  },
});
for (const t of ["rider/status", "rider/status/imu", "rider/status/odom", "rider/control/drivemode",
                 "rider/control/maxspeed", "rider/debug/telem"]) client.subscribe(t);

function status(s) {
  lastStatus = Date.now();
  S = s;
  controls();
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
// Every other control needs a LIVE link (it decides from the robot's current state).
const live = () => client.connected && S && Date.now() - lastStatus < 2000;
setInterval(() => {
  $("link").classList.toggle("ok", Date.now() - lastStatus < 2000);
  $("estop").disabled = !client.connected;
  controls();
}, 250);

// ---- balance card ----
function controls() {
  const ok = live();
  const bal = ok && S.roll_balance_enabled, lev = ok && S.roll_leveling_enabled;
  const bb = $("balbtn");
  bb.disabled = !ok;
  bb.textContent = !ok ? "–" : bal ? "Stop balancing" : "Start balancing";
  bb.className = "btn" + (bal ? " stop" : "");
  const lb = $("levbtn");
  lb.disabled = !ok;
  lb.textContent = "Roll Balance: " + (!ok ? "–" : lev ? "ON" : "off");
  lb.className = "btn" + (lev ? " active" : "");
  // controller: switchable only while NOT balancing (the firmware refuses it anyway)
  const pol = ok && S.mode === "POL", lqr = ok && S.mode === "LQR";
  $("ctrlpol").className = "btn pol" + (pol ? " active" : "");
  $("ctrllqr").className = "btn lqr" + (lqr ? " active" : "");
  $("ctrlpol").disabled = $("ctrllqr").disabled = !ok || bal;
  set("ctrlhint", ok && bal ? "stop balancing to switch" : "");
  $("speed").disabled = !ok;
  $("zero").disabled = !ok;
  $("height").disabled = $("leghome").disabled = !ok;
}

// Balance: idle -> 'en 1'; balancing -> STOP, same as the LCD button and E-STOP (balance off + legs limp)
$("balbtn").addEventListener("click", () => {
  if (!live()) return;
  if (S.roll_balance_enabled) { line("en 0"); line("leglimp"); } else line("en 1");
});
$("levbtn").addEventListener("click", () => {
  if (!live()) return;
  line(S.roll_leveling_enabled ? "level 0" : "level 1");
});
$("ctrlpol").addEventListener("click", () => { if (live() && !S.roll_balance_enabled) line("ctrl 0"); });
$("ctrllqr").addEventListener("click", () => { if (live() && !S.roll_balance_enabled) line("ctrl 1"); });
$("zero").addEventListener("click", () => { if (live()) line("poszero"); });

// Top speed: a RETAINED message the joystick service reads (full stick = this speed). Shown from the
// retained value; while dragging, the local position wins until release, then it's published.
let speedDragging = false;
function showSpeed(v) {
  const val = (v === null || isNaN(v)) ? DEFAULT_SPEED : v;
  $("speed").value = val;
  set("speedval", `${val.toFixed(2)} m/s` + (v === null ? " (default)" : ""));
}
showSpeed(null);
$("speed").addEventListener("input", () => {
  speedDragging = true;
  set("speedval", `${Number($("speed").value).toFixed(2)} m/s`);
});
$("speed").addEventListener("change", () => {
  speedDragging = false;
  const v = Number($("speed").value);
  if (live()) client.publish("rider/control/maxspeed", JSON.stringify({ mps: v }), true);   // retained
  else showSpeed(speedRetained);
});

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

// ---- legs (step 4): ONE slider that is both the height control and its indicator ----
// Not moving: the slider sits at the legs' actual height (telemetry). Dragging: the legs FOLLOW the
// slider continuously -- every 100 ms the page sends 'legrate' (the DS4 right stick's smooth rate
// command, integrated by the firmware into 1-count steps every cycle) proportional to the gap
// between the slider and the legs, capped, until they arrive; then 'legrate 0'. Heartbeat-safe: if
// the page stops sending, the firmware stops the legs within 300 ms. legrate goes through the same
// mirrored path as the leg buttons, so roll leveling (when on) carries on around the new height.
// Height is the mirrored part of the two leg goals, h = (legR + (LEG_SUM - legL)) / 2 in right-leg
// counts, so a roll-leveling tilt (equal and opposite) doesn't move it.
const LEG = { rMin: 440, rMax: 535, rMid: 480, sum: 1400 };   // firmware LEG_R_MIN/MAX/MID, LEG_EQUAL_SUM
const toPct = (h) => Math.max(0, Math.min(100, 100 * (h - LEG.rMin) / (LEG.rMax - LEG.rMin)));
const toH = (pct) => LEG.rMin + pct / 100 * (LEG.rMax - LEG.rMin);
let legH = null;                     // current height from telemetry (right-leg counts)
let heightTarget = null;             // where the slider wants the legs (right-leg counts); null = idle
const H_GAIN = 3.0, H_MAX = 60, H_MIN = 8;   // counts/s per count of gap; cap (~DS4 full stick 50); floor

function height(t) {
  const r = / legR=(\d+)/.exec(t), l = / legL=(\d+)/.exec(t);
  if (!r || !l) return;
  legH = (Number(r[1]) + (LEG.sum - Number(l[1]))) / 2;
  if (heightTarget === null) showHeight(legH);
}
function showHeight(h) {
  $("height").value = toPct(h).toFixed(0);
  set("hval", Math.abs(h - LEG.rMid) < 1 ? "ride height" : `${Math.round(toPct(h))} %`);
}
$("height").addEventListener("input", () => {                 // fires continuously while dragging
  if (!live()) return;
  heightTarget = toH(Number($("height").value));
  set("hval", Math.abs(heightTarget - LEG.rMid) < 1 ? "ride height" : `${$("height").value} %`);
});
// follow loop: steer the legs toward the target, stop when there, when the link is gone, or when the
// legs make no progress for 2 s (e.g. limp after a STOP: the firmware ignores height until balance-on)
let lastProgressH = null, lastProgressT = 0;
setInterval(() => {
  if (heightTarget === null) { lastProgressH = null; return; }
  if (!live() || legH === null) { line("legrate 0"); heightTarget = null; return; }
  const gap = heightTarget - legH;
  if (Math.abs(gap) < 1) { line("legrate 0"); heightTarget = null; showHeight(legH); return; }
  if (lastProgressH === null || Math.abs(legH - lastProgressH) >= 1) { lastProgressH = legH; lastProgressT = Date.now(); }
  else if (Date.now() - lastProgressT > 2000) { line("legrate 0"); heightTarget = null; showHeight(legH); return; }
  const rate = Math.sign(gap) * Math.min(H_MAX, Math.max(H_MIN, H_GAIN * Math.abs(gap)));
  line(`legrate ${rate.toFixed(1)}`);
}, 100);
$("leghome").addEventListener("click", () => { if (live()) line("leghome"); });
