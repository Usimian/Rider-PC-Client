// Rider control page -- steps 2-4: status bar, pose, stats, E-STOP, balance card, legs (docs/WEB_UI_DESIGN.md);
// + the ToF card (8x8 view, obstacle/edge stop switch).
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
    else if (topic === "rider/response/image_capture") camFrame(p);
    else if (topic === "rider/tof") tofFrame(p);
    else if (topic === "rider/safety/fwd_limit") tofSafety(p);
    else if (topic === "rider/wifi") wifi(p);
  },
});
for (const t of ["rider/status", "rider/status/imu", "rider/status/odom", "rider/control/drivemode",
                 "rider/control/maxspeed", "rider/debug/telem", "rider/tof", "rider/safety/fwd_limit",
                 "rider/response/image_capture", "rider/wifi"]) client.subscribe(t);

// WiFi watchdog (retained rider/wifi): today's dropouts and the latest one go in the fault item, spelled out
let wifiFault = "";
function wifi(w) {
  const n = Number(w.dropouts_today) || 0;
  wifiFault = n === 0 ? "" : `WiFi: ${n} dropout${n > 1 ? "s" : ""} today · ${w.last_event.slice(11)}`;
  fault();
}
// Pi under-voltage alarms count since the Pi booted (a loose 5 V lead connector shows up here first)
function fault() {
  const wheel = S && S.fault ? `wheel ${S.fault} off bus` : "";
  const n = S ? Number(S.undervolt_alarms) || 0 : 0;
  const volt = n ? `Pi 5 V low: ${n} alarm${n > 1 ? "s" : ""} since boot` : "";
  const text = [wheel, volt, wifiFault].filter(Boolean).join(" · ");
  set("fault", text || "none", wheel ? "bad" : (volt || wifiFault) ? "warn" : "off");
}

function status(s) {
  lastStatus = Date.now();
  S = s;
  controls();
  $("raw").textContent = JSON.stringify(s, null, 2);
  // battery icon: the fill is the charge, coloured green (full) -> yellow (half) -> red (empty)
  const lvl = Math.max(0, Math.min(100, Number(s.battery_level)));
  const fill = $("battfill");
  fill.setAttribute("width", (23 * lvl / 100).toFixed(1));
  fill.setAttribute("fill", `hsl(${(1.2 * lvl).toFixed(0)} 80% 50%)`);       // 120 = green, 60 = yellow, 0 = red
  set("batt", `${s.battery_level} %`, lvl < 15 ? "num bad" : "num");
  $("battitem").title = `battery ${s.battery_level} %, ${Number(s.battery_voltage).toFixed(2)} V`;
  // DS4 icon: grey when not connected; connected, its body fills left to right with the CONTROLLER's charge,
  // coloured like the robot battery (green -> yellow -> red); the unfilled part stays dim
  const on = !!s.controller_connected, cb = s.controller_battery;
  const clvl = on ? (cb == null ? 100 : Math.max(0, Math.min(100, Number(cb)))) : 0;
  $("ds4").setAttribute("class", "icon " + (on ? "on" : "off"));
  $("ds4fill").setAttribute("width", (32 * clvl / 100).toFixed(1));
  $("ds4fill").setAttribute("fill", `hsl(${(1.2 * clvl).toFixed(0)} 80% 50%)`);
  $("ds4chg").style.display = on && s.controller_charging ? "" : "none";     // lightning bolt while charging
  $("ds4").parentElement.title = !on ? "DS4 controller not connected" :
    "DS4 controller connected" + (cb == null ? "" : `, battery ${cb} %` + (s.controller_charging ? " (charging)" : ""));
  set("ctrl", s.mode === "POL" ? "POLICY" : s.mode, s.mode === "POL" ? "pol" : "lqr");
  set("bal", s.roll_balance_enabled ? "ON" : "off", s.roll_balance_enabled ? "on" : "off");
  set("lev", s.roll_leveling_enabled ? "ON" : "off", s.roll_leveling_enabled ? "on" : "off");
  set("pos", m(s.position)); set("tgt", m(s.target));
  set("xy", `${Number(s.pose_x).toFixed(2)}, ${Number(s.pose_y).toFixed(2)} m`);
  set("cpu", `${Math.round(s.cpu_percent)} %`);
  set("temp", `${Number(s.cpu_temp).toFixed(0)} °C`, s.cpu_temp >= 80 ? "num bad" : s.cpu_temp >= 70 ? "num warn" : "num");
  set("pw", `${Number(s.power_w).toFixed(1)} W`);
  set("hz", `${s.loop_hz} Hz`);
  set("rfail", `${s.read_fails}`, s.read_fails > 0 ? "num warn" : "num");     // failed wheel reads since ESP32 boot
  fault();
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

// ---- camera (design v1): request a frame, show it, request the next ----
// rider-camera answers rider/control/image_capture with one 640x480 JPEG (base64) on
// rider/response/image_capture, and powers the camera down a few seconds after the last request -- so
// the feed runs only while some page has it switched on (off by default; nothing is sent on load).
// Each page tags its requests and paces itself on its OWN replies; other pages' frames are ignored.
const CAM_MIN_MS = 100, CAM_TIMEOUT_MS = 2000;    // at most ~10 frames/s; re-ask if a reply is lost
let camOn = false, camReq = null, camSentT = 0, camFrames = [], camTimer = null;
const camId = Math.random().toString(16).slice(2, 8);
let camSeq = 0;
function camAsk() {
  camTimer = null;
  if (!camOn || !client.connected) return;
  camReq = `${camId}-${++camSeq}`;
  camSentT = Date.now();
  client.publish("rider/control/image_capture", JSON.stringify({ request_id: camReq, resolution: "high" }));
}
function camFrame(p) {
  if (!camOn || p.request_id !== camReq) return;
  camReq = null;
  if (p.success) {
    $("cam").src = "data:image/jpeg;base64," + p.image_data;
    const now = Date.now();
    camFrames.push(now); camFrames = camFrames.filter((t) => now - t < 2000);
    set("caminfo", `${(camFrames.length / 2).toFixed(1)} frames/s`);
  } else set("caminfo", "camera error: " + (p.error || "?"));
  camTimer = setTimeout(camAsk, Math.max(0, CAM_MIN_MS - (Date.now() - camSentT)));
}
setInterval(() => {                                // lost reply / reconnect: ask again
  $("cambtn").disabled = !client.connected;
  if (camOn && camReq !== null && Date.now() - camSentT > CAM_TIMEOUT_MS) { camReq = null; camAsk(); }
  if (camOn && camReq === null && camTimer === null) camAsk();
}, 500);
$("cambtn").addEventListener("click", () => {
  camOn = !camOn;
  $("cambtn").textContent = "Camera: " + (camOn ? "on" : "off");
  $("cambtn").className = "btn" + (camOn ? " active" : "");
  if (camOn) { set("caminfo", "starting…"); camAsk(); }
  else { camReq = null; if (camTimer) { clearTimeout(camTimer); camTimer = null; } $("cam").removeAttribute("src"); set("caminfo", ""); }
});

// ---- obstacles (ToF): the 8x8 view + the obstacle/edge stop switch ----
// Distances come from the sensor (rider/tof, ~10 Hz); the zone classes, what is ahead and whether the
// stop is ON come from the governor (rider/safety/fwd_limit, rider_tof_safety.py) -- robot truth, as
// for every other control. The switch is the RETAINED rider/safety/enable {"on"}: the governor limits
// forward drive only while it is on; off, it still reports what it sees ('would' = the limit it would set).
const tofCells = [];
for (let k = 0; k < 64; k++) { const c = document.createElement("div"); $("tofgrid").appendChild(c); tofCells.push(c); }
let tofT = 0, safeT = 0, SAFE = null, tofCls = ".".repeat(64);
const TOF_VALID = new Set([5, 6]);                  // VL53L5CX target_status: 5 valid, 6 50 % valid
function tofFrame(f) {
  tofT = Date.now();
  for (let k = 0; k < 64; k++) {
    const c = tofCells[k], mm = f.d[k], ok = TOF_VALID.has(f.s[k]) && mm > 0;
    const cl = tofCls[k];
    c.className = !ok ? "none" : (cl === "o" || cl === "f" || cl === "d") ? cl : "";
    if (!ok) { c.textContent = "–"; c.style.background = ""; continue; }
    const hue = 220 * Math.min(1, mm / 2000);     // 0 mm red .. 2 m+ blue
    c.style.background = `hsl(${hue.toFixed(0)} 70% 38%)`;
    c.textContent = mm >= 1000 ? (mm / 1000).toFixed(1) + "m" : Math.round(mm / 10);
  }
}
function tofSafety(p) {
  safeT = Date.now();
  SAFE = p;
  if (typeof p.cls === "string" && p.cls.length === 64) tofCls = p.cls;
  tofShow();
}
const safeLive = () => client.connected && SAFE && Date.now() - safeT < 2000;
function tofShow() {
  $("tofgrid").classList.toggle("stale", Date.now() - tofT > 2000);
  const b = $("tofbtn"), ok = safeLive();
  b.disabled = !ok;
  b.textContent = "Obstacle & edge stop: " + (!ok ? "–" : SAFE.on ? "ON" : "off");
  b.className = "btn" + (ok && SAFE.on ? " active" : "");
  if (!ok) { set("tofstate", "ToF governor not running", "off"); return; }
  const what = SAFE.reason === "obstacle" ? "obstacle" : SAFE.reason === "cliff" ? "table edge" : null;
  const dist = !what ? "clear" : SAFE.min_mm >= 0 ? `${what} ${Math.round(SAFE.min_mm / 10)} cm`
             : `${what} — no floor ahead`;
  const lim = SAFE.on ? Number(SAFE.factor) : Number(SAFE.would);
  const pct = `${Math.round(100 * lim)} %`;
  const txt = dist + (lim < 0.999 ? (SAFE.on ? ` — forward speed capped at ${pct}` : ` — would cap forward at ${pct}`) : "");
  set("tofstate", txt, !SAFE.on ? "off" : lim <= 0.001 ? "bad" : lim < 0.999 ? "warn" : "on");
}
setInterval(tofShow, 250);
$("tofbtn").addEventListener("click", () => {
  if (safeLive()) client.publish("rider/safety/enable", JSON.stringify({ on: !SAFE.on }), true);   // retained
});
