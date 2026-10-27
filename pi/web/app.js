// Rider control page -- steps 2-4: status bar, pose, stats, E-STOP, balance card, legs (docs/WEB_UI_DESIGN.md);
// + the ToF card (8x8 view, obstacle stop switch).
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

// ---- obstacles (ToF): the 8x8 view + the obstacle stop switch ----
// Distances come from the sensor (rider/tof, ~10 Hz); the zone classes, what is ahead and whether the
// stop is ON come from the governor (rider/safety/fwd_limit, rider_tof_safety.py) -- robot truth, as
// for every other control. The switch is the RETAINED rider/safety/enable {"on"}: the governor limits
// forward drive only while it is on; off, it still reports what it sees ('would' = the limit it would set).
const tofCells = [];
for (let k = 0; k < 64; k++) { const c = document.createElement("div"); $("tofgrid").appendChild(c); tofCells.push(c); }
let tofT = 0, safeT = 0, SAFE = null, tofCls = ".".repeat(64);
const TOF_VALID = new Set([5, 6, 9]);               // VL53L5CX target_status: 5 valid, 6 50 % valid, 9 valid (large
                                                    // pulse) -- the same set as rider_tof_safety.py's VALID
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
  depth3d();
}
const safeLive = () => client.connected && SAFE && Date.now() - safeT < 2000;
function tofShow() {
  $("tofgrid").classList.toggle("stale", Date.now() - tofT > 2000);
  $("depth3d").classList.toggle("stale", !safeLive());
  const b = $("tofbtn"), ok = safeLive();
  b.disabled = !ok;
  b.textContent = "Obstacle: " + (!ok ? "–" : SAFE.on ? "ON" : "off");
  b.className = "btn" + (ok && SAFE.on ? " active" : "");
  if (!ok) { set("tofstate", "ToF governor not running", "off"); return; }
  const dist = SAFE.reason === "obstacle" && SAFE.min_mm >= 0 ? `obstacle ${Math.round(SAFE.min_mm / 10)} cm` : "clear";
  const lim = SAFE.on ? Number(SAFE.factor) : Number(SAFE.would);
  const pct = `${Math.round(100 * lim)} %`;
  const txt = dist + (lim < 0.999 ? (SAFE.on ? ` — FWD speed capped at ${pct}` : ` — would cap FWD at ${pct}`) : "");
  set("tofstate", txt, !SAFE.on ? "off" : lim <= 0.001 ? "bad" : lim < 0.999 ? "warn" : "on");
}
setInterval(tofShow, 250);
$("tofbtn").addEventListener("click", () => {
  if (safeLive()) client.publish("rider/safety/enable", JSON.stringify({ on: !SAFE.on }), true);   // retained
});

// ---- obstacle card view switch: the 8x8 grid or the 3D view, in the same space. Which one is a per-browser
// display preference (not robot state), remembered in localStorage when the browser allows it.
let view3d = false;
try { view3d = localStorage.getItem("tofview") === "3d"; } catch { /* storage blocked: start on the grid */ }
function showTofView() {
  $("tofviewgrid").hidden = view3d;
  $("tofview3d").hidden = !view3d;
  $("tofviewbtn").textContent = view3d ? "Grid view" : "3D view";
  $("tofresetbtn").hidden = !view3d;
  if (view3d) depth3d();
}
$("tofviewbtn").addEventListener("click", () => {
  view3d = !view3d;
  try { localStorage.setItem("tofview", view3d ? "3d" : "grid"); } catch { /* not remembered */ }
  showTofView();
});

// ---- 3D view: the governor's floor-referenced zone points (rider/safety/fwd_limit "pts", mm: forward, left,
// up; origin on the floor under the sensor), drawn as an orthographic three-quarter view. Drag sideways to turn,
// up/down to tilt, middle-button drag to move, scroll wheel or two-finger pinch to zoom, Reset button (or a
// mouse double-click) to reset.
const D3_HOME = { az: 35, el: 28, zoom: 1, panX: 0, panY: 0 };
const D3 = { ...D3_HOME, drag: null };   // azimuth (deg, + = behind-right), elevation (deg), zoom, pan (px) after the fit
const D3_COL = { o: "#ff5a5a", f: "#3ecf72", d: "#ffc53d", b: "#c9a24a", h: "#7d86a8" };
const D3_LANE = Math.tan(11.25 * Math.PI / 180);     // drive corridor = the middle 4 of 8 columns of a 45 deg view
const D3_XMAX = 1.2, D3_YMAX = 0.5;                   // floor drawn: 0..1.2 m ahead, +-0.5 m to the sides
function depth3d() {
  const cv = $("depth3d");
  if (!view3d) return;
  const dpr = window.devicePixelRatio || 1, W = cv.clientWidth, H = cv.clientHeight;
  if (!W || !H) return;
  if (cv.width !== Math.round(W * dpr) || cv.height !== Math.round(H * dpr)) {
    cv.width = Math.round(W * dpr); cv.height = Math.round(H * dpr);
  }
  const g = cv.getContext("2d");
  g.setTransform(dpr, 0, 0, dpr, 0, 0);
  g.clearRect(0, 0, W, H);
  const a = D3.az * Math.PI / 180, e = D3.el * Math.PI / 180;
  const r0 = Math.sin(a), r1 = -Math.cos(a), f0 = Math.cos(a), f1 = Math.sin(a), ce = Math.cos(e), se = Math.sin(e);
  // fit the drawn floor (plus 0.4 m of height) to the canvas at this angle: scale + offset from its projected box
  const u0 = (x, y) => x * r0 + y * r1, v0 = (x, y, z) => -(z * ce + (x * f0 + y * f1) * se);
  let uMin = Infinity, uMax = -Infinity, vMin = Infinity, vMax = -Infinity;
  for (const x of [-0.1, D3_XMAX]) for (const y of [-D3_YMAX, D3_YMAX]) for (const z of [0, 0.4]) {
    const u = u0(x, y), v = v0(x, y, z);
    uMin = Math.min(uMin, u); uMax = Math.max(uMax, u); vMin = Math.min(vMin, v); vMax = Math.max(vMax, v);
  }
  const pad = 14, sc = Math.min((W - 2 * pad) / (uMax - uMin), (H - 2 * pad) / (vMax - vMin));   // px per metre
  const ox = (W - sc * (uMax - uMin)) / 2 - sc * uMin, oy = (H - sc * (vMax - vMin)) / 2 - sc * vMin;
  // then the user's zoom about the canvas centre, and pan (the wheel pans so the point under the pointer stays put)
  const cx = W / 2, cy = H / 2, zm = D3.zoom;
  const P = (x, y, z) => [cx + zm * (ox + sc * u0(x, y) - cx) + D3.panX, cy + zm * (oy + sc * v0(x, y, z) - cy) + D3.panY];
  const path = (pts, close) => {
    g.beginPath(); pts.forEach(([x, y, z], i) => { const [u, v] = P(x, y, z); i ? g.lineTo(u, v) : g.moveTo(u, v); });
    if (close) g.closePath();
  };

  // floor grid, 10 cm; every 50 cm brighter and labelled
  for (let i = 0; i <= Math.round(D3_XMAX * 10); i++) {
    const x = i / 10, major = i % 5 === 0;
    g.strokeStyle = major ? "#3a4370" : "#232a48"; g.lineWidth = 1;
    path([[x, D3_YMAX, 0], [x, -D3_YMAX, 0]]); g.stroke();
    if (major && i) { const [u, v] = P(x, D3_YMAX, 0); g.fillStyle = "#8b93b5"; g.font = "11px system-ui";
                      g.fillText(`${x} m`, u - 30, v); }
  }
  for (let i = -Math.round(D3_YMAX * 10); i <= Math.round(D3_YMAX * 10); i++) {
    g.strokeStyle = i === 0 ? "#3a4370" : "#232a48";
    path([[0, i / 10, 0], [D3_XMAX, i / 10, 0]]); g.stroke();
  }

  // drive lane, shaded by what the stop does there (distances from the governor)
  const live = SAFE && Array.isArray(SAFE.pts);
  const stop = live ? SAFE.stop_mm / 1000 : 0.10, slow = live ? SAFE.slow_mm / 1000 : 0.40;
  const band = (x0, x1, col) => {
    path([[x0, x0 * D3_LANE, 0], [x1, x1 * D3_LANE, 0], [x1, -x1 * D3_LANE, 0], [x0, -x0 * D3_LANE, 0]], true);
    g.fillStyle = col; g.fill();
  };
  band(0, stop, "rgba(255,90,90,.28)");
  band(stop, slow, "rgba(255,197,61,.20)");
  band(slow, D3_XMAX, "rgba(102,194,255,.08)");

  // the robot: a wire box behind the sensor, the sensor a dot at its measured height
  const h = live ? SAFE.h_mm / 1000 : 0.062;
  const bx = [-0.10, 0], by = [-0.09, 0.09], bz = [0, h + 0.07];
  g.strokeStyle = "#8b93b5"; g.lineWidth = 1;
  for (const z of bz) { path([[bx[0], by[0], z], [bx[1], by[0], z], [bx[1], by[1], z], [bx[0], by[1], z]], true); g.stroke(); }
  for (const x of bx) for (const y of by) { path([[x, y, bz[0]], [x, y, bz[1]]]); g.stroke(); }
  const [su, sv] = P(0, 0, h);
  g.fillStyle = "#66c2ff"; g.beginPath(); g.arc(su, sv, 4, 0, 2 * Math.PI); g.fill();

  if (!live) {
    g.fillStyle = "#8b93b5"; g.font = "13px system-ui"; g.fillText("no ToF governor data", 12, 20);
    return;
  }
  // zone points, far to near so near ones paint on top; obstacles get a stalk to the floor
  const pts = [];
  SAFE.pts.forEach((p, k) => {
    if (!p) return;
    const x = p[0] / 1000, y = p[1] / 1000, z = p[2] / 1000;
    if (x < -0.1 || x > D3_XMAX + 0.4 || Math.abs(y) > D3_YMAX + 0.3) return;    // off the drawn floor
    pts.push({ x, y, z, c: tofCls[k], depth: x * f0 + y * f1 });
  });
  pts.sort((p, q) => q.depth - p.depth);
  for (const p of pts) {
    const col = D3_COL[p.c] || "#e8ecf8";
    if (p.c === "o") { g.strokeStyle = col; g.globalAlpha = .45; path([[p.x, p.y, 0], [p.x, p.y, p.z]]); g.stroke(); g.globalAlpha = 1; }
    const [u, v] = P(p.x, p.y, p.z);
    g.fillStyle = col; g.beginPath(); g.arc(u, v, 2.8, 0, 2 * Math.PI); g.fill();
  }
}
const cv3 = $("depth3d");
const fromCentre = (x, y) => { const r = cv3.getBoundingClientRect(); return [x - r.left - r.width / 2, y - r.top - r.height / 2]; };
// zoom to z1 keeping the canvas point m (px from the centre) on screen where the zoom-z0 view had point m0
const zoomAt = (z0, z1, m0, m, pan0) => {
  D3.zoom = z1;
  D3.panX = m[0] - (z1 / z0) * (m0[0] - pan0[0]);
  D3.panY = m[1] - (z1 / z0) * (m0[1] - pan0[1]);
};
const clampZoom = (z) => Math.max(0.5, Math.min(8, z));
// one drag (mouse, or one finger): turn/tilt, or move with the middle button
const startDrag = (x, y, pan) => { D3.drag = { x, y, az: D3.az, el: D3.el, panX: D3.panX, panY: D3.panY, pan }; };
// phone: touches by pointerId; two fingers = pinch to zoom + move, anchored at the fingers' midpoint
const touches = new Map();
let pinch = null;
const resetView = () => { Object.assign(D3, D3_HOME); depth3d(); };
const twoFingers = () => {
  const [a, b] = [...touches.values()];
  return { mid: fromCentre((a.x + b.x) / 2, (a.y + b.y) / 2), dist: Math.hypot(a.x - b.x, a.y - b.y) || 1 };
};
cv3.addEventListener("pointerdown", (ev) => {
  if (ev.button === 1) ev.preventDefault();              // middle button: no browser autoscroll, it pans the view
  try { cv3.setPointerCapture(ev.pointerId); } catch { /* pointer already gone */ }
  if (ev.pointerType === "touch") {
    touches.set(ev.pointerId, { x: ev.clientX, y: ev.clientY });
    if (touches.size >= 2) {                             // second finger: the turn stops, the pinch starts
      D3.drag = null;
      const f = twoFingers();
      pinch = { mid: f.mid, dist: f.dist, zoom: D3.zoom, pan: [D3.panX, D3.panY] };
      return;
    }
  }
  startDrag(ev.clientX, ev.clientY, ev.button === 1);
});
cv3.addEventListener("pointermove", (ev) => {
  if (ev.pointerType === "touch" && touches.has(ev.pointerId)) {
    touches.set(ev.pointerId, { x: ev.clientX, y: ev.clientY });
    if (pinch && touches.size >= 2) {
      const f = twoFingers();
      zoomAt(pinch.zoom, clampZoom(pinch.zoom * f.dist / pinch.dist), pinch.mid, f.mid, pinch.pan);
      depth3d();
      return;
    }
  }
  if (!D3.drag) return;
  const dx = ev.clientX - D3.drag.x, dy = ev.clientY - D3.drag.y;
  if (D3.drag.pan) { D3.panX = D3.drag.panX + dx; D3.panY = D3.drag.panY + dy; }          // middle drag: move
  else {
    D3.az = Math.max(-170, Math.min(170, D3.drag.az - dx * 0.4));                         // turn: scene follows the drag
    D3.el = Math.max(2, Math.min(88, D3.drag.el + dy * 0.3));                             // tilt: flat .. top-down
  }
  depth3d();
});
cv3.addEventListener("mousedown", (ev) => { if (ev.button === 1) ev.preventDefault(); });   // autoscroll starts here
for (const t of ["pointerup", "pointercancel"]) cv3.addEventListener(t, (ev) => {
  touches.delete(ev.pointerId);
  if (pinch && touches.size < 2) {                       // back to one finger: carry on turning from where it is
    pinch = null;
    const [rest] = [...touches.values()];
    if (rest) { startDrag(rest.x, rest.y, false); return; }
  }
  D3.drag = null;
});
cv3.addEventListener("wheel", (ev) => {
  ev.preventDefault();                                   // zoom the view, not the page
  const m = fromCentre(ev.clientX, ev.clientY);          // keep the point under the pointer where it is
  zoomAt(D3.zoom, clampZoom(D3.zoom * Math.exp(-ev.deltaY * 0.0015)), m, m, [D3.panX, D3.panY]);
  depth3d();
}, { passive: false });
cv3.addEventListener("dblclick", resetView);   // mouse only: phones send no dblclick here (the view takes all touch)
$("tofresetbtn").addEventListener("click", resetView);
new ResizeObserver(() => depth3d()).observe(cv3);
showTofView();
