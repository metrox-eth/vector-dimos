/* MOSS cockpit page.
 *
 * Talks only to the server that served it (relative paths, no other host: the
 * field has no internet). Contract: tools/moss_cockpit.py.
 *
 *   GET  /state.json  1 Hz        GET /scan.json  3 Hz      GET /path.json 1 Hz
 *   GET  /map_meta    1 Hz, then /map.png only when map_meta.stamp changed
 *   GET  /log.json    0.5 Hz      GET /preflight.json on load and after a check
 *   GET  /launch.json while a stack command runs
 *   POST /cmd/<explore|goal|stop|episode|stack>
 *
 * Each poller waits for its answer before scheduling the next one, so a slow
 * Jetson never gets a queue of requests. Pollers sleep while the tab is hidden.
 */
"use strict";

var POLL = { state: 1000, meta: 1000, scan: 333, path: 1000, log: 2000, launch: 1000 };
var FETCH_TIMEOUT_MS = 2500;
var TAP_SLOP_PX = 6;          // a pointer that moved less than this is a tap, not a drag
var STALE_S = 3;              // a topic older than this is shown as stale
var LIVE_S = 2;               // telemetry, scan and path older than this are not drawn or read as live
var EP_FULL_S = 300;          // the bottom progress line is full after 5 min of episode
var RUN_FULL_S = 1800;        // ... or after 30 min of run when no episode records
var CAM_RETRY_MS = 3000;      // reconnect the MJPEG stream this long after it ended or failed, forever
var CAM_GRACE_MS = 6000;      // a stream younger than this is not judged dead (first frame wait + a state poll)

var S = {
  online: false, state: null,
  meta: null, mapImg: null, mapStamp: null, scan: null, path: null,
  preflight: null, launchBusy: false, launchAction: null,
  view: { scale: 40, x: -3, y: 3 }, fitted: false,
  pendingGoal: null, camPaused: false, camOk: false, camAt: 0, camLost: null, pfAll: false,
  stateAt: 0
};

function $(id) { return document.getElementById(id); }

// ---------------------------------------------------------------- utilities

function fetchJSON(url, opts) {
  var ctl = ("AbortController" in window) ? new AbortController() : null;
  var t = ctl ? setTimeout(function () { ctl.abort(); }, FETCH_TIMEOUT_MS) : null;
  var o = opts || {};
  if (ctl) o.signal = ctl.signal;
  o.cache = "no-store";
  return fetch(url, o).then(function (r) {
    if (t) clearTimeout(t);
    if (r.status === 404) return { __missing: true };
    return r.json().then(function (d) {
      if (!r.ok && d && typeof d === "object" && !("ok" in d)) d.ok = false;
      return d;
    });
  }, function (e) { if (t) clearTimeout(t); throw e; });
}

function post(name, body) {
  return fetchJSON("/cmd/" + name, {
    method: "POST", headers: { "Content-Type": "application/json", "X-Requested-With": "moss-cockpit" },
    body: JSON.stringify(body || {})
  });
}

function poller(fn, period) {
  function tick() {
    if (document.hidden) { setTimeout(tick, period); return; }
    var p;
    try { p = fn(); } catch (e) { p = null; }
    Promise.resolve(p).catch(function () {}).then(function () { setTimeout(tick, period); });
  }
  tick();
}

function num(v, d) { return (typeof v === "number" && isFinite(v)) ? v.toFixed(d) : "--"; }
function signed(v, d) { return (typeof v === "number" && isFinite(v)) ? ((v >= 0 ? "+" : "") + v.toFixed(d)) : "--"; }
function pad2(n) { return (n < 10 ? "0" : "") + n; }
function clock(s) {
  if (typeof s !== "number" || !isFinite(s) || s < 0) return "--:--";
  s = Math.floor(s);
  var h = Math.floor(s / 3600), m = Math.floor((s % 3600) / 60), sec = s % 60;
  return (h ? h + ":" + pad2(m) : m) + ":" + pad2(sec);
}
function span(s) {               // long durations (machine uptime): days and hours read better than 254:08:52
  if (typeof s !== "number" || !isFinite(s) || s < 0) return "--";
  if (s < 86400) return clock(s);
  return Math.floor(s / 86400) + " d " + Math.floor((s % 86400) / 3600) + " h";
}
function dur(s) {
  if (typeof s !== "number" || !isFinite(s)) return "--";
  if (s < 90) return Math.round(s) + " s";
  if (s < 5400) return Math.round(s / 60) + " min";
  return Math.round(s / 3600) + " h";
}
function ago(s) {
  if (typeof s !== "number" || !isFinite(s)) return "--";
  return s < 1 ? "now" : dur(s) + " ago";
}
function setText(id, t) { var el = $(id); if (el && el.textContent !== t) el.textContent = t; }
function meter(id, frac, attn) {
  var el = $(id); if (!el) return;
  var f = (typeof frac === "number" && isFinite(frac)) ? Math.max(0, Math.min(1, frac)) : 0;
  el.firstElementChild.style.width = (f * 100).toFixed(1) + "%";
  el.classList.toggle("attn", !!attn);
}

var sayTimer = null;
function say(text, bad) {
  var el = $("say");
  el.textContent = text;
  el.className = bad ? "err" : "ok";
  clearTimeout(sayTimer);
  // a success line fades after a while; an error stays until the next action
  if (!bad) sayTimer = setTimeout(function () { if (el.textContent === text) el.textContent = ""; }, 8000);
}

function answer(d, okText) {
  if (d && d.ok) { say(okText || ("Sent on " + (d.sent || "the bus") + "."), false); return true; }
  say((d && d.error) ? d.error : "The cockpit refused the command.", true);
  return false;
}

// ---------------------------------------------------------------- state

function stack() { return (S.state && S.state.stack) || { running: false }; }
function blueprint() { var b = stack().blueprint || ""; return String(b); }
function isExploreStack() { return stack().running && /explore/.test(blueprint()); }
function isDemo() { return !!(S.state && S.state.mode === "demo"); }
function canGoal() { return S.online && (isExploreStack() || isDemo()); }
function exploring() { return !!(S.state && S.state.nav && S.state.nav.exploring); }

// seconds since the last state the cockpit answered: the server's ages froze then
function offlineFor() { return S.online || !S.stateAt ? 0 : Math.max(0, (Date.now() - S.stateAt) / 1000); }

// a bus topic's age as the server measured it now, or null when it is silent or unknown
function topicAge(name) {
  var t = S.state && S.state.topics && S.state.topics[name];
  return t && typeof t.age_s === "number" ? t.age_s + offlineFor() : null;
}

function pollState() {
  return fetchJSON("/state.json").then(function (d) {
    if (!d || d.__missing) throw new Error("no state");
    var wasOffline = !S.online && S.stateAt > 0;
    S.state = d; S.online = true; S.stateAt = Date.now();
    if (wasOffline) camRestart("Cockpit back. Reconnecting the stream.");   // its stream died with it
    render();
  }, function () {
    S.online = false;
    render();
  });
}

function render() {
  document.body.classList.toggle("offline", !S.online);
  renderHeader();
  renderBase();
  renderSystem();
  renderControls();
  renderCaption();
  renderCamera();
  renderPreflightWhen();
  requestDraw();
}

function renderHeader() {
  var st = stack(), chip = $("chip"), cls, label;
  if (!S.online) { cls = "off"; label = "OFFLINE"; }
  else if (st.running && exploring()) { cls = "on"; label = "EXPLORING"; }
  else if (st.running) { cls = "on"; label = "MAPPING"; }
  else { cls = "idle"; label = "IDLE"; }
  chip.className = "chip " + cls;
  setText("chip", label);
  var name = st.running ? (st.run_id || st.blueprint || "stack") : "no stack";
  if (isDemo()) name += "  ·  demo data";
  setText("run-name", name);
}

function modeName(b) {
  if (b.mode_name) return b.mode_name;
  return ({ 0: "off", 1: "ready", 2: "drive", 3: "test" })[b.mode] || "--";
}

function setTrack(barId, valId, v) {
  var bar = $(barId);
  var p = (typeof v === "number" && isFinite(v)) ? Math.max(-100, Math.min(100, v)) : 0;
  var h = Math.abs(p) / 2;                      // 100 % fills half the bar
  bar.style.height = h + "%";
  bar.style.bottom = p >= 0 ? "50%" : (50 - h) + "%";
  bar.classList.toggle("rev", p < 0);
  setText(valId, (typeof v === "number") ? String(Math.round(v)) : "--");
}

function renderBase() {
  var b = S.state && S.state.base;
  var sec = $("mod-base");
  if (!b) {
    setText("base-title", S.online ? "No telemetry." : "Cockpit offline.");
    setText("b-volt", "--"); setText("b-pct", "--"); meter("b-meter", 0);
    setTrack("t-l", "t-lv", null); setTrack("t-r", "t-rv", null);
    ["b-mode", "b-ticks", "b-fw", "b-age"].forEach(function (id) { setText(id, "--"); });
    setText("t-target", "target -- / --");
    return;
  }
  var low = typeof b.battery_v === "number" && b.battery_v < 10.8;
  // age of the data = time since the cockpit got the message + its age when it was sent
  var age = (typeof b.age_s === "number" ? b.age_s : 0) + (typeof b.firmware_age_s === "number" ? b.firmware_age_s : 0)
            + offlineFor();
  var stale = age > LIVE_S;
  var title;
  if (stale) title = "No telemetry for " + dur(age) + ".";
  else if (b.ina_ok === false) title = "Battery sensor not seen.";
  else if (low) title = "Battery low. Land it.";
  else if (b.mode === 2) title = "Driving.";
  else if (b.mode === 1) title = "Armed, ready.";
  else title = "Disarmed.";
  if (b.mock && !stale) title = "Fake firmware. " + title;
  setText("base-title", title);
  $("base-title").classList.toggle("attn", stale || low || b.ina_ok === false);
  setText("b-volt", num(b.battery_v, 2));
  setText("b-pct", typeof b.battery_pct === "number" ? String(Math.round(b.battery_pct)) : "--");
  $("b-volt").parentNode.classList.toggle("attn", low);
  meter("b-meter", (b.battery_pct || 0) / 100, low);
  var o = b.output_pct || [], t = b.target_pct || [];
  setTrack("t-l", "t-lv", o[0]); setTrack("t-r", "t-rv", o[1]);
  setText("t-target", "target " + (t.length ? Math.round(t[0]) + " / " + Math.round(t[1]) : "-- / --"));
  setText("b-mode", modeName(b) + (b.reason && b.reason !== modeName(b) ? "  ·  " + b.reason : ""));
  setText("b-ticks", b.ticks ? b.ticks[0] + " / " + b.ticks[1] : "--");
  setText("b-fw", (b.fw || "--") + (b.driver ? "  ·  " + b.driver : ""));
  setText("b-age", stale ? "stale  ·  " + ago(age) : ago(age));
  sec.classList.toggle("stale", stale);
}

var TOPIC_ORDER = ["odom", "lidar", "global_costmap", "path", "cmd_vel", "base_telemetry", "color_image"];

function renderSystem() {
  var s = S.state && S.state.system;
  if (s) {
    setText("s-cpu", num(s.cpu_percent, 0) + " %");
    setText("s-ram", num(s.ram_used_gb, 1) + " / " + num(s.ram_total_gb, 1) + " GB");
    var tp = s.temps || {}, parts = [];
    if (typeof tp.cpu === "number") parts.push("CPU " + tp.cpu.toFixed(0) + "°");
    if (typeof tp.gpu === "number") parts.push("GPU " + tp.gpu.toFixed(0) + "°");
    setText("s-temp", parts.length ? parts.join("  ·  ") : "--");
    setText("s-load", num(s.load_1m, 2));
    setText("s-up", span(s.uptime_s));
    meter("s-cpu-m", (s.cpu_percent || 0) / 100, s.cpu_percent > 90);
    meter("s-ram-m", (s.ram_percent || 0) / 100, s.ram_percent > 90);
    var dl = $("s-disks"), html = "";
    (s.disks || []).forEach(function (d) {
      html += "<dt>Disk " + esc(d.mountpoint) + "</dt><dd>" + num(d.free_gb, 0) + " GB free</dd>";
    });
    if (dl.innerHTML !== html) dl.innerHTML = html;
    setText("sys-title", s.cpu_percent > 90 || s.ram_percent > 90 ? "Jetson under load." : "Jetson.");
  }
  var topics = (S.state && S.state.topics) || {}, rows = "";
  var names = TOPIC_ORDER.filter(function (n) { return n in topics; });
  Object.keys(topics).forEach(function (n) { if (names.indexOf(n) < 0) names.push(n); });
  names.forEach(function (n) {
    var t = topics[n] || {}, fresh = typeof t.age_s === "number" && t.age_s <= STALE_S;
    rows += '<span class="n">' + esc(n) + '</span><span class="r ' + (fresh ? "" : "dim") + '">'
         + (typeof t.hz === "number" ? t.hz.toFixed(1) + " Hz" : "--") + '</span><span class="r '
         + (fresh ? "" : "attn") + '">' + (typeof t.age_s === "number" ? ago(t.age_s) : "silent") + "</span>";
  });
  if (!names.length) rows = '<span class="n">no topic yet</span><span></span><span></span>';
  if ($("bus").innerHTML !== rows) $("bus").innerHTML = rows;
}

function esc(s) {
  return String(s == null ? "" : s).replace(/[&<>"]/g, function (c) {
    return { "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c];
  });
}

function renderControls() {
  var st = stack(), on = S.online, busy = S.launchBusy;
  $("b-map").disabled = !on || st.running || busy;
  $("b-explore").disabled = !on || st.running || busy;
  $("b-stack-stop").disabled = !on || (!st.running && !busy);
  var exOk = on && (isExploreStack() || isDemo());
  $("b-ex-start").disabled = !exOk || exploring();
  $("b-ex-stop").disabled = !exOk || !exploring();
  if (!exOk) $("ex-confirm").hidden = true;
  setText("ex-note", exOk ? (exploring() ? "Exploring. A pad stick or Stop exploration takes over."
                                         : "Armed only by this button or EXPLORE=1 moss-fly.sh explore.")
                          : "Needs the explore stack (Launch explore).");
  var rec = (S.state && S.state.recording) || {}, ep = rec.episode;
  var canRec = on && (st.running || isDemo());
  $("b-ep").disabled = !canRec;
  $("b-ep").textContent = ep ? "Stop" : "Start";
  $("b-ep").classList.toggle("primary", !ep && canRec);
  $("ep-name").disabled = !!ep;
  if (ep && document.activeElement !== $("ep-name")) $("ep-name").value = ep.name || "";
  var note;
  if (ep) note = "Recording " + ep.name + "  ·  " + clock(epElapsed(ep));
  else if (rec.exists) note = "Run database " + num(rec.size_mb, 1) + " MB. An episode adds markers to it.";
  else note = "The run records to its database while the stack is up.";
  setText("rec-note", note);
  var title;
  if (!on) title = "Cockpit offline.";
  else if (st.running && exploring()) title = "Exploring on its own.";
  else if (st.running) title = "Stack up. Nothing moves by itself.";
  else title = "Nothing moves by itself.";
  setText("ctl-title", title);
}

function epElapsed(ep) {
  if (!ep || ep.started_at == null) return null;
  var t0 = typeof ep.started_at === "number" ? ep.started_at : Date.parse(ep.started_at) / 1000;
  var now = (S.state && S.state.ts) || Date.now() / 1000;
  return isFinite(t0) ? Math.max(0, now - t0) : null;
}

function renderCaption() {
  var st = stack(), nav = (S.state && S.state.nav) || {};
  var bp = (st.blueprint || "none").replace(/^vector-dimos\./, "");
  setText("cap-kicker", "RUN / " + bp.toUpperCase());
  var mins = st.running && typeof st.uptime_s === "number" ? Math.floor(st.uptime_s / 60) : 0;
  setText("cap-num", pad2(mins));
  var title, sub = "";
  if (!S.online) title = "Cockpit offline. Last values shown.";
  else if (!st.running) title = isDemo() ? "Demo data. Nothing is connected." : "No stack running.";
  else if (exploring()) title = "Exploring on its own.";
  else if (nav.goal && nav.goal_reached !== true) title = "Heading to a goal.";
  else if (/explore/.test(bp)) title = "Mapping. Exploration not armed.";
  else title = "Mapping. Drive with the pad.";
  if (nav.explore_done) sub = "Exploration finished: no frontier left.";
  else if (nav.goal) sub = "Goal " + signed(nav.goal.x, 2) + ", " + signed(nav.goal.y, 2) + " m"
                        + (nav.goal_reached === true ? "  ·  reached" : nav.goal_reached === false ? "  ·  not reached" : "");
  var c = nav.last_cmd_vel;
  if (c && typeof c.age_s === "number" && c.age_s < 2) sub += (sub ? "  ·  " : "") + "cmd " + num(c.vx, 2) + " m/s  " + num(c.wz, 2) + " rad/s";
  setText("cap-title", title);
  setText("cap-sub", sub);
  setText("map-title", S.meta ? (st.running ? "Live costmap." : "Last costmap.") : "Waiting for a map.");
  // the costmap writes nothing until its mission gate opens: explore_cmd, or 5 cm from the boot pose
  setText("map-empty-why", !S.online ? "No map yet. The cockpit is offline."
    : !st.running ? "No map yet. Launch a stack to build one."
    : /explore/.test(bp) ? "No map yet. It opens when exploration starts or the rover moves 5 cm."
    : "No map yet. It opens once the rover has moved 5 cm.");

  var rec = (S.state && S.state.recording) || {}, ep = rec.episode, frac, left, right;
  if (ep) {
    var e = epElapsed(ep) || 0;
    frac = (e % EP_FULL_S) / EP_FULL_S;
    left = "EPISODE / " + String(ep.name || "").toUpperCase();
    right = clock(e);
  } else if (st.running) {
    frac = Math.min(1, (st.uptime_s || 0) / RUN_FULL_S);
    left = "no episode recording";
    right = "run " + clock(st.uptime_s);
  } else { frac = 0; left = "no episode"; right = ""; }
  $("cap-progress").style.width = (frac * 100).toFixed(2) + "%";
  setText("cap-left", left);
  setText("cap-right", right);
}

// ---------------------------------------------------------------- camera

function camSilent() {
  var t = S.state && S.state.topics && S.state.topics.color_image;
  return !(t && typeof t.age_s === "number" && t.age_s <= STALE_S);
}

var camTimer = null;
function camConnect() {
  var img = $("cam-img");
  if (S.camPaused || document.hidden) { img.removeAttribute("src"); S.camOk = false; return; }
  if (!img.getAttribute("src")) { S.camAt = Date.now(); img.src = "/camera.mjpg?t=" + Date.now(); }
}

// Drop the stream and ask again, with a fresh URL, CAM_RETRY_MS later. An MJPEG
// stream the server closes cleanly fires no error (the browser holds the last
// frame as a finished image), so this also runs when the state says it is gone.
function camRestart(why) {
  $("cam-img").removeAttribute("src");
  S.camOk = false; S.camLost = why;
  if (!camTimer) camTimer = setTimeout(function () { camTimer = null; camConnect(); renderCamera(); }, CAM_RETRY_MS);
}

function camWatch() {
  if (S.camPaused || document.hidden || !S.online || camTimer) return;
  if (!$("cam-img").getAttribute("src")) { camConnect(); return; }
  if (Date.now() - S.camAt < CAM_GRACE_MS) return;
  var cam = S.state && S.state.camera;
  if (cam && cam.clients === 0) camRestart("The stream ended. Reconnecting.");          // the server holds no stream: ours is gone
  else if (S.camOk && camSilent() && !isDemo()) camRestart("The camera topic went silent. Reconnecting.");
}

function renderCamera() {
  camWatch();
  var ph = $("cam-ph"), why;
  if (S.camPaused) why = "Video paused in this browser.";
  else if (!S.online) why = "Cockpit offline.";
  else if (!S.camOk && camTimer) why = S.camLost || "Reconnecting.";
  else if (!S.camOk && camSilent() && !isDemo() && Date.now() - S.camAt >= CAM_GRACE_MS) why = "The camera topic is silent. Waiting for frames.";
  else if (!S.camOk) why = "Connecting to the stream.";
  else if (camSilent() && !isDemo()) why = "The camera topic is silent.";
  ph.hidden = !why;
  if (why) setText("cam-why", why);
  setText("b-cam", S.camPaused ? "Resume video" : "Pause video");
}

(function () {
  var img = $("cam-img");
  img.addEventListener("load", function () { S.camOk = true; S.camLost = null; renderCamera(); });
  img.addEventListener("error", function () {
    if (!img.getAttribute("src")) return;              // our own removeAttribute, not a failure
    camRestart("No stream from the cockpit. Retrying every " + CAM_RETRY_MS / 1000 + " s."); renderCamera();
  });
  try { S.camPaused = localStorage.getItem("moss.cockpit.camPaused") === "1"; } catch (e) {}
  $("b-cam").addEventListener("click", function () {
    S.camPaused = !S.camPaused;
    try { localStorage.setItem("moss.cockpit.camPaused", S.camPaused ? "1" : "0"); } catch (e) {}
    camConnect(); renderCamera();
  });
  // MJPEG is the heaviest thing the Jetson serves: drop it while nobody looks.
  document.addEventListener("visibilitychange", camConnect);
})();

// ---------------------------------------------------------------- map

var cv = $("map"), ctx = cv.getContext("2d"), wrap = $("map-wrap");
var drawQueued = false;

function sx(x) { return (x - S.view.x) * S.view.scale; }
function sy(y) { return (S.view.y - y) * S.view.scale; }
function wx(px) { return S.view.x + px / S.view.scale; }
function wy(py) { return S.view.y - py / S.view.scale; }
function W() { return cv.clientWidth; }
function H() { return cv.clientHeight; }

function resize() {
  var r = window.devicePixelRatio || 1;
  cv.width = Math.max(1, Math.round(W() * r));
  cv.height = Math.max(1, Math.round(H() * r));
  ctx.setTransform(r, 0, 0, r, 0, 0);
  requestDraw();
}

function fitView() {
  var m = S.meta; if (!m) return;
  var w = m.width * m.res, h = m.height * m.res;
  // leave the lower third to the caption: fit into the top 74 % of the pane
  var avH = H() * 0.74, s = Math.min(W() / w, avH / h) * 0.96;
  if (!isFinite(s) || s <= 0) return;
  S.view.scale = s;
  S.view.x = m.ox - (W() / s - w) / 2;
  S.view.y = m.oy + h + (avH / s - h) / 2;
  S.fitted = true;
  requestDraw();
}

function requestDraw() {
  if (drawQueued) return;
  drawQueued = true;
  requestAnimationFrame(function () { drawQueued = false; draw(); });
}

function drawGrid() {
  // 1 m grid, only over the map itself: outside it the pane stays bare studio
  var m = S.meta;
  if (S.view.scale < 14 || !m) return;
  var x0 = Math.ceil(Math.max(wx(0), m.ox)), x1 = Math.floor(Math.min(wx(W()), m.ox + m.width * m.res));
  var y0 = Math.ceil(Math.max(wy(H()), m.oy)), y1 = Math.floor(Math.min(wy(0), m.oy + m.height * m.res));
  var gx0 = sx(m.ox), gx1 = sx(m.ox + m.width * m.res), gy0 = sy(m.oy + m.height * m.res), gy1 = sy(m.oy);
  ctx.strokeStyle = "rgba(234,240,239,.07)";
  ctx.lineWidth = 1;
  ctx.beginPath();
  for (var x = x0; x <= x1; x++) { var X = Math.round(sx(x)) + .5; ctx.moveTo(X, gy0); ctx.lineTo(X, gy1); }
  for (var y = y0; y <= y1; y++) { var Y = Math.round(sy(y)) + .5; ctx.moveTo(gx0, Y); ctx.lineTo(gx1, Y); }
  ctx.stroke();
}

function pose() { return S.state && S.state.pose; }

function scanWorld() {
  var sc = S.scan; if (!sc || !sc.points) return [];
  if (sc.frame === "world") return sc.points;
  var p = pose();                    // lidar frame: place it at the robot
  var px = p ? p.x : 0, py = p ? p.y : 0, c = Math.cos(p ? p.yaw : 0), s = Math.sin(p ? p.yaw : 0);
  return sc.points.map(function (q) { return [px + c * q[0] - s * q[1], py + s * q[0] + c * q[1]]; });
}

// The server keeps the last scan and path however old they are: only fresh ones
// are drawn as live. The scan's own age_s is cached with it, so the bus age wins.
function scanLive() {
  var sc = S.scan; if (!sc) return false;
  var topics = (S.state && S.state.topics) || {};
  var a = sc.source in topics ? topicAge(sc.source)
        : (typeof sc.age_s === "number" ? sc.age_s + offlineFor() : null);
  return a !== null && a < LIVE_S;
}
// One /path message per plan, not a stream: a plan stays valid while the stack runs.
function pathLive() {
  var pa = S.path, a = pa && typeof pa.age_s === "number" ? pa.age_s + offlineFor() : null;
  if (a !== null && a < LIVE_S) return true;
  var odom = topicAge("odom");
  return stack().running && odom !== null && odom < LIVE_S;
}

function draw() {
  var w = W(), h = H();
  ctx.clearRect(0, 0, w, h);
  var m = S.meta;
  if (S.mapImg && m) {
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(S.mapImg, sx(m.ox), sy(m.oy + m.height * m.res), m.width * m.res * S.view.scale, m.height * m.res * S.view.scale);
  }
  drawGrid();

  // path: mint line, dimmed once the stack that planned it is gone
  var pts = S.path && S.path.points;
  if (pts && pts.length > 1) {
    ctx.strokeStyle = pathLive() ? "#6CF8C7" : "rgba(108,248,199,.3)";
    ctx.lineWidth = 2.25; ctx.lineJoin = "round"; ctx.lineCap = "round";
    ctx.beginPath(); ctx.moveTo(sx(pts[0][0]), sy(pts[0][1]));
    for (var i = 1; i < pts.length; i++) ctx.lineTo(sx(pts[i][0]), sy(pts[i][1]));
    ctx.stroke();
  }

  // scan: mint dots, only while the lidar speaks (an old scan is not where the robot is)
  var sc = scanLive() ? scanWorld() : [], r = Math.max(1.1, Math.min(2.4, S.view.scale / 40));
  ctx.fillStyle = "rgba(108,248,199,.9)";
  for (var k = 0; k < sc.length; k++) {
    var X = sx(sc[k][0]), Y = sy(sc[k][1]);
    if (X < -4 || Y < -4 || X > w + 4 || Y > h + 4) continue;
    ctx.fillRect(X - r / 2, Y - r / 2, r, r);
  }

  // goal: salmon ring (dashed while it waits for confirmation)
  var nav = S.state && S.state.nav;
  if (nav && nav.goal) ring(nav.goal.x, nav.goal.y, false);
  if (S.pendingGoal) ring(S.pendingGoal.x, S.pendingGoal.y, true);

  // pose: white triangle pointing along yaw
  var p = pose();
  if (p && typeof p.x === "number") {
    var cx = sx(p.x), cy = sy(p.y), a = -(p.yaw || 0), L = 15, B = 8;
    ctx.save(); ctx.translate(cx, cy); ctx.rotate(a);
    ctx.beginPath(); ctx.moveTo(L * .62, 0); ctx.lineTo(-L * .38, B); ctx.lineTo(-L * .2, 0); ctx.lineTo(-L * .38, -B); ctx.closePath();
    ctx.fillStyle = (typeof p.age_s === "number" && p.age_s > STALE_S) ? "rgba(234,240,239,.5)" : "#EAF0EF";
    ctx.strokeStyle = "rgba(8,15,26,.65)"; ctx.lineWidth = 1.2;
    ctx.fill(); ctx.stroke(); ctx.restore();
  }
  positionChip();
}

function ring(x, y, dashed) {
  ctx.save();
  ctx.strokeStyle = "#FF947E"; ctx.lineWidth = 2.25;
  if (dashed) ctx.setLineDash([4, 4]);
  ctx.beginPath(); ctx.arc(sx(x), sy(y), 9, 0, 2 * Math.PI); ctx.stroke();
  ctx.setLineDash([]);
  ctx.beginPath(); ctx.arc(sx(x), sy(y), 2, 0, 2 * Math.PI); ctx.fillStyle = "#FF947E"; ctx.fill();
  ctx.restore();
}

function pollMeta() {
  return fetchJSON("/map_meta").then(function (m) {
    if (!m || m.__missing || m.error || !(m.width > 0)) {
      $("map-empty").hidden = !!S.mapImg; return;
    }
    var stamp = String(m.stamp != null ? m.stamp : (m.map_mtime != null ? m.map_mtime : ""));
    if (stamp === S.mapStamp && S.mapImg) return;
    return new Promise(function (ok) {
      var img = new Image();
      img.onload = function () {
        // /map.png is a second request: the server may have decoded a newer, differently
        // cropped costmap in between. The crop only grows, so a size mismatch catches it;
        // keep the old pair and leave the stamp, the next poll fetches both again.
        if (img.naturalWidth !== m.width || img.naturalHeight !== m.height) { ok(); return; }
        // swap image and metadata together, so the transform always matches the picture
        S.mapImg = img; S.meta = m; S.mapStamp = stamp;
        $("map-empty").hidden = true;
        if (!S.fitted) fitView();
        renderCaption(); requestDraw(); ok();
      };
      img.onerror = function () { ok(); };
      img.src = "/map.png?stamp=" + encodeURIComponent(stamp);
    });
  }, function () {});
}

function pollScan() {
  return fetchJSON("/scan.json").then(function (d) {
    if (d && !d.__missing && d.points) { S.scan = d; requestDraw(); }
  }, function () {});
}

function pollPath() {
  return fetchJSON("/path.json").then(function (d) {
    S.path = (d && !d.__missing && d.points) ? d : null; requestDraw();
  }, function () {});
}

// pointer events: drag = pan, wheel / pinch = zoom around the cursor, tap = goal
var ptrs = {}, gesture = null;

function local(e) { var b = cv.getBoundingClientRect(); return { x: e.clientX - b.left, y: e.clientY - b.top }; }
function ptrList() { return Object.keys(ptrs).map(function (k) { return ptrs[k]; }); }

cv.addEventListener("pointerdown", function (e) {
  if (e.button !== undefined && e.button > 0) return;
  cv.setPointerCapture(e.pointerId);
  ptrs[e.pointerId] = local(e);
  var l = ptrList();
  if (l.length === 1) {
    gesture = { kind: "pan", start: l[0], vx: S.view.x, vy: S.view.y, moved: false };
  } else if (l.length === 2) {
    var mx = (l[0].x + l[1].x) / 2, my = (l[0].y + l[1].y) / 2;
    gesture = { kind: "pinch", d0: Math.hypot(l[0].x - l[1].x, l[0].y - l[1].y) || 1, s0: S.view.scale,
                wx: wx(mx), wy: wy(my), moved: true };
  }
});

cv.addEventListener("pointermove", function (e) {
  var p = local(e);
  setText("hud-xy", "x " + signed(wx(p.x), 2) + " m   y " + signed(wy(p.y), 2) + " m");
  if (!(e.pointerId in ptrs)) return;
  ptrs[e.pointerId] = p;
  if (!gesture) return;
  var l = ptrList();
  if (gesture.kind === "pan" && l.length === 1) {
    var dx = p.x - gesture.start.x, dy = p.y - gesture.start.y;
    if (!gesture.moved && Math.abs(dx) + Math.abs(dy) > TAP_SLOP_PX) { gesture.moved = true; cv.classList.add("grabbing"); }
    if (gesture.moved) { S.view.x = gesture.vx - dx / S.view.scale; S.view.y = gesture.vy + dy / S.view.scale; requestDraw(); }
  } else if (gesture.kind === "pinch" && l.length >= 2) {
    var d = Math.hypot(l[0].x - l[1].x, l[0].y - l[1].y) || 1;
    var mx = (l[0].x + l[1].x) / 2, my = (l[0].y + l[1].y) / 2;
    zoomTo(gesture.s0 * d / gesture.d0, gesture.wx, gesture.wy, mx, my);
  }
});

function endPointer(e) {
  var was = gesture, p = ptrs[e.pointerId] || local(e);
  delete ptrs[e.pointerId];
  cv.classList.remove("grabbing");
  var left = ptrList().length;
  if (left === 0) {
    gesture = null;
    if (e.type === "pointerup" && was && was.kind === "pan" && !was.moved) tap(p);
  } else if (left === 1) {
    var q = ptrList()[0];             // pinch -> pan with the remaining finger, never a tap
    gesture = { kind: "pan", start: q, vx: S.view.x, vy: S.view.y, moved: true };
  }
}
cv.addEventListener("pointerup", endPointer);
cv.addEventListener("pointercancel", endPointer);
cv.addEventListener("pointerleave", function () { setText("hud-xy", "x -- m   y -- m"); });

function zoomTo(scale, wxp, wyp, px, py) {
  S.view.scale = Math.max(4, Math.min(600, scale));
  S.view.x = wxp - px / S.view.scale;
  S.view.y = wyp + py / S.view.scale;
  requestDraw();
}

cv.addEventListener("wheel", function (e) {
  e.preventDefault();
  var p = local(e);
  zoomTo(S.view.scale * Math.pow(1.0015, -e.deltaY), wx(p.x), wy(p.y), p.x, p.y);
}, { passive: false });

$("b-fit").addEventListener("click", fitView);

// tap on the map -> inline confirm chip, never a modal
function tap(p) {
  var x = wx(p.x), y = wy(p.y);
  if (!canGoal()) {
    S.pendingGoal = null;
    showChip(p, "Goals need the explore stack running.", false);
    return;
  }
  S.pendingGoal = { x: x, y: y, px: p.x, py: p.y };
  showChip(p, "Send goal (" + signed(x, 2) + ", " + signed(y, 2) + ")? Stops exploration first.", true);
  requestDraw();
}

var chipTimer = null;
function showChip(p, text, ask) {
  var chip = $("goal-chip");
  setText("goal-text", text);
  $("b-goal-yes").hidden = !ask;
  $("b-goal-no").textContent = ask ? "Cancel" : "OK";
  chip.hidden = false;
  chip.dataset.px = p.x; chip.dataset.py = p.y;
  positionChip();
  clearTimeout(chipTimer);
  if (ask) $("b-goal-yes").focus({ preventScroll: true });
  else chipTimer = setTimeout(hideChip, 4000);
}

function positionChip() {
  var chip = $("goal-chip");
  if (chip.hidden) return;
  var px = S.pendingGoal ? sx(S.pendingGoal.x) : +chip.dataset.px;
  var py = S.pendingGoal ? sy(S.pendingGoal.y) : +chip.dataset.py;
  var cw = chip.offsetWidth, ch = chip.offsetHeight;
  var left = Math.max(0, Math.min(W() - cw, px - cw / 2));
  var top = py - ch - 18;
  if (top < 44) top = py + 18;
  chip.style.left = left + "px";
  chip.style.top = Math.max(0, Math.min(H() - ch, top)) + "px";
}

function hideChip() { $("goal-chip").hidden = true; S.pendingGoal = null; requestDraw(); }

$("b-goal-no").addEventListener("click", hideChip);
$("b-goal-yes").addEventListener("click", function () {
  var g = S.pendingGoal; if (!g) return;
  hideChip();
  post("goal", { x: +g.x.toFixed(3), y: +g.y.toFixed(3) }).then(function (d) {
    answer(d, "Goal sent: " + signed(g.x, 2) + ", " + signed(g.y, 2) + " m.");
    pollState();
  }, function () { say("Goal not sent: the cockpit did not answer.", true); });
});

window.addEventListener("keydown", function (e) {
  if (e.key === "Escape") { hideChip(); $("ex-confirm").hidden = true; }
});

// ---------------------------------------------------------------- commands

function stackCmd(action, okText) {
  S.launchBusy = true; S.launchAction = action; renderControls();
  post("stack", { action: action }).then(function (d) {
    if (!answer(d, okText)) { S.launchBusy = false; renderControls(); return; }
    watchLaunch();
  }, function () { S.launchBusy = false; renderControls(); say("The cockpit did not answer.", true); });
}

var launchWatching = false;
function watchLaunch() {
  if (launchWatching) return;
  launchWatching = true;
  (function step() {
    fetchJSON("/launch.json").then(function (d) {
      var lines = (d && (d.lines || d.tail)) || [];
      var ul = $("launch-tail"), html = "";
      lines.slice(-5).forEach(function (l) { html += "<li>" + esc(l) + "</li>"; });
      ul.innerHTML = html;
      if (d && d.running) { setTimeout(step, POLL.launch); return; }
      launchWatching = false;
      var was = S.launchAction;
      S.launchBusy = false; S.launchAction = null;
      renderControls();
      if (was) loadPreflight();          // check, map and explore all rewrite preflight.json
      pollState();
    }, function () { setTimeout(step, POLL.launch * 2); });
  })();
}

$("b-map").addEventListener("click", function () { stackCmd("map", "Launching the mapping stack."); });
$("b-explore").addEventListener("click", function () { stackCmd("explore", "Launching the explore stack. Exploration stays off."); });
$("b-stack-stop").addEventListener("click", function () { stackCmd("stop", "Stopping the stack."); });
$("b-check").addEventListener("click", function () { stackCmd("check", "Flight check running."); });
$("b-check2").addEventListener("click", function () { stackCmd("check", "Flight check running."); });

$("b-ex-start").addEventListener("click", function () { $("ex-confirm").hidden = false; $("b-ex-yes").focus(); });
$("b-ex-no").addEventListener("click", function () { $("ex-confirm").hidden = true; });
$("b-ex-yes").addEventListener("click", function () {
  $("ex-confirm").hidden = true;
  post("explore", { action: "start" }).then(function (d) { answer(d, "Exploration started."); pollState(); },
                                            function () { say("The cockpit did not answer.", true); });
});
$("b-ex-stop").addEventListener("click", function () {
  post("explore", { action: "stop" }).then(function (d) { answer(d, "Exploration stopped."); pollState(); },
                                           function () { say("The cockpit did not answer.", true); });
});

$("b-ep").addEventListener("click", function () {
  var rec = (S.state && S.state.recording) || {}, ep = rec.episode;
  var name = ep ? ep.name : ($("ep-name").value.trim() || "ep1");
  post("episode", { action: ep ? "stop" : "start", name: name }).then(function (d) {
    if (answer(d, ep ? "Episode " + name + " closed." : "Episode " + name + " started.") && ep) {
      var m = /^(.*?)(\d+)$/.exec(name);
      $("ep-name").value = m ? m[1] + (parseInt(m[2], 10) + 1) : name + "-2";
    }
    pollState();
  }, function () { say("The cockpit did not answer.", true); });
});

var stopTimer = null;
function stopShow(label, bad) {         // #say may be scrolled away: the button that was pressed tells too
  ["b-stop", "stop-fab"].forEach(function (id) { var b = $(id); b.textContent = label; b.classList.toggle("fail", !!bad); });
  clearTimeout(stopTimer);
  stopTimer = setTimeout(function () { stopShow("STOP", false); }, bad ? 8000 : 2500);
}
function softStop() {
  hideChip();
  post("stop", {}).then(function (d) {
    stopShow(answer(d, "Soft stop sent: exploration, navigation, zero twist.") ? "STOP SENT" : "STOP NOT SENT", !(d && d.ok));
    pollState();
  }, function () {
    stopShow("STOP NOT SENT", true);
    say("STOP NOT SENT: the cockpit did not answer. Use the pad or moss-fly.sh stop.", true);
  });
}
$("b-stop").addEventListener("click", softStop);
$("stop-fab").addEventListener("click", softStop);

// ---------------------------------------------------------------- flight check

function loadPreflight() {
  return fetchJSON("/preflight.json").then(function (d) {
    if (!d || d.__missing || d.available === false || !d.checks) { S.preflight = null; }
    else S.preflight = d;
    renderPreflight();
  }, function () {});
}

function renderPreflight() {
  var d = S.preflight, list = $("pf-list");
  $("pf-empty").hidden = !!d;
  list.hidden = !d;
  $("pf-again").hidden = !d;
  if (!d) { setText("pf-title", "No flight check yet."); setText("pf-when", ""); return; }
  var ko = typeof d.ko_count === "number" ? d.ko_count : d.checks.filter(function (c) { return !c.ok; }).length;
  setText("pf-title", ko ? ko + (ko > 1 ? " checks" : " check") + " to fix." : "Ready to fly.");
  $("pf-title").classList.toggle("attn", ko > 0);
  renderPreflightWhen();
  // Brief by default: what needs fixing stays in view, the passing rows fold
  // away so the base and system modules are not pushed off the screen.
  var html = "", sec = null, pass = 0, skippedN = 0;
  d.checks.forEach(function (c) {
    if (c.section && c.section !== sec) {
      sec = c.section;
      html += '<li class="sec"><span class="kicker">' + esc(String(sec).toUpperCase()) + "</span></li>";
    }
    var skipped = c.ok && /skipp|held by the stack/i.test(c.detail || "");
    var mark = skipped ? '<span class="v na">--</span>' : c.ok ? '<span class="v ok">OK</span>' : '<span class="v ko">KO</span>';
    if (c.ok && !skipped) pass++;
    if (skipped) skippedN++;
    html += '<li class="' + (c.ok ? "pass" : "fail") + '">' + mark + '<span class="lb">' + esc(c.label) + "</span>"
         + (c.detail ? '<span class="dt">' + esc(c.detail) + "</span>" : "") + "</li>";
  });
  var skipTxt = skippedN ? ", " + skippedN + " skipped" : "";
  html += '<li class="passline">' + (pass + skippedN === d.checks.length ? pass + " checks pass" + skipTxt + "."
                                     : pass + " passing " + (pass === 1 ? "check" : "checks") + " folded" + skipTxt + ".") + "</li>";
  list.innerHTML = html;
  list.classList.toggle("brief", !S.pfAll);
  setText("b-pf-all", S.pfAll ? "Fold passing checks" : "Show passing checks");
  $("b-pf-all").setAttribute("aria-expanded", S.pfAll ? "true" : "false");
}

// the check's age on the Jetson's clock (no NTP in the field), refreshed by every state poll
function renderPreflightWhen() {
  var d = S.preflight;
  if (!d) return;
  var now = (S.state && typeof S.state.ts === "number") ? S.state.ts + offlineFor() : Date.now() / 1000;
  var when = typeof d.ts === "number" ? ago(now - d.ts) : "";
  setText("pf-when", (d.summary || "") + (when ? "  ·  " + when : ""));
}

$("b-pf-all").addEventListener("click", function () { S.pfAll = !S.pfAll; renderPreflight(); });

// ---------------------------------------------------------------- log

function fmtTs(ts) {
  if (typeof ts === "number") { var d = new Date(ts * 1000); return pad2(d.getHours()) + ":" + pad2(d.getMinutes()) + ":" + pad2(d.getSeconds()); }
  // dimOS stamps main.jsonl in UTC ISO with a "Z": parse it, show local time
  var t = typeof ts === "string" && /^\d{4}-\d{2}-\d{2}T/.test(ts) ? new Date(ts) : null;
  if (t && !isNaN(t.getTime())) return pad2(t.getHours()) + ":" + pad2(t.getMinutes()) + ":" + pad2(t.getSeconds());
  var m = /(\d{2}:\d{2}:\d{2})/.exec(String(ts || ""));
  return m ? m[1] : "";
}

function pollLog() {
  return fetchJSON("/log.json?n=20").then(function (d) {
    var lines = Array.isArray(d) ? d : (d && (d.lines || d.log)) || [];
    var el = $("log"), html = "";
    lines.slice(-20).forEach(function (l) {
      var lv = String(l.level || "").toLowerCase(), warn = /warn|error|crit/.test(lv);
      html += '<li class="' + (warn ? "warn" : "") + '"><span class="t">' + esc(fmtTs(l.ts)) + '</span><span class="e">'
           + esc(l.event) + (l.logger ? ' <span class="lg">' + esc(String(l.logger).split(".").pop()) + "</span>" : "")
           + "</span></li>";
    });
    if (!html) html = '<li><span class="t"></span><span class="e dim">' + (stack().running ? "Log is empty." : "No stack, no log.") + "</span></li>";
    if (el.innerHTML !== html) el.innerHTML = html;
  }, function () {});
}

// ---------------------------------------------------------------- go

if ("ResizeObserver" in window) new ResizeObserver(function () { resize(); if (!S.fitted) fitView(); }).observe(wrap);
window.addEventListener("resize", resize);
resize();
camConnect();
renderCamera();
loadPreflight();
fetchJSON("/launch.json").then(function (d) { if (d && d.running) { S.launchBusy = true; watchLaunch(); } }, function () {});
poller(pollState, POLL.state);
poller(pollMeta, POLL.meta);
poller(pollScan, POLL.scan);
poller(pollPath, POLL.path);
poller(pollLog, POLL.log);
