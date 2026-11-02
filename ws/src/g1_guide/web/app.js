// G1 展馆导览机器人 · 网页展示控制台。
// 数据（导航、人群、视觉、语音）走 rosbridge JSON；相机画面是 demo_web 的 MJPEG（/stream/<名>）。
"use strict";

const $ = (id) => document.getElementById(id);

const NAV_STATE = {
  unknown: "未知", requested: "收到目标", moving: "移动中", aligning: "调整朝向",
  arrived: "已到达", blocked: "路线受阻", backing_up: "后退避让", cancelled: "已停止",
  invalid_goal: "目标无效", sensor_stale: "传感器中断", invalid_command: "指令异常",
  facing: "转向观众", faced: "已面向观众", follow_starting: "开始跟随", following: "跟随中",
  follow_waiting: "跟随·等待", follow_searching: "跟随·寻找", follow_blocked: "跟随·受阻",
  follow_lost: "跟丢了", no_target: "没找到要跟的人",
};
const NAV_CLASS = { arrived: "ok", faced: "ok", following: "ok", follow_waiting: "ok",
  blocked: "warn", backing_up: "warn", follow_searching: "warn", follow_blocked: "warn",
  invalid_goal: "bad", sensor_stale: "bad", invalid_command: "bad", follow_lost: "bad", no_target: "bad" };
const VOICE_STATE = {
  listening: ["倾听中", "#2ea043"], hearing: ["听到声音", "#3fa7ff"], thinking: ["思考中", "#d29922"],
  speaking: ["说话中", "#a371f7"], muted: ["麦克风关闭", "#6e7681"], idle: ["等待输入", "#6e7681"],
  sleeping: ["待唤醒：说“你好小吉”", "#57606a"],
};
const GESTURES = [["wave", "挥手"], ["point", "指向展品"], ["present", "摊手介绍"], ["explain_loop", "讲解循环"],
  ["emphasize", "强调"], ["invite", "双手邀请"], ["home", "放下手臂"]];
const HAND = { palm: "手掌（停）", wave: "挥手", thumbs_up: "竖拇指", one: "比 1", two: "比 2", three: "比 3", four: "比 4" };
const CHIPS = ["你好小吉", "有什么展品？", "带我去看青花瓷", "介绍一下这幅画", "讲简单点", "幽默一点",
  "跟我来", "别跟了", "下一站", "停下"];
const FRONT_W = 640, FRONT_H = 480;   // 前置相机分辨率（/guide/vision 的像素坐标）

// ---------------------------------------------------------------- rosbridge
class Rosbridge {
  constructor(url, onState) {
    this.url = url; this.onState = onState; this.subs = []; this.adv = new Set();
    this.ws = null; this.connected = false; this.callId = 0; this.calls = {};
    this.connect();
  }
  connect() {
    const ws = new WebSocket(this.url);
    this.ws = ws;
    ws.onopen = () => {
      this.connected = true; this.adv.clear(); this.onState(true);
      for (const s of this.subs) this.send({ op: "subscribe", topic: s.topic, type: s.type, ...s.opts });
    };
    ws.onclose = () => {
      if (this.connected) this.onState(false);
      this.connected = false;
      setTimeout(() => this.connect(), 2000);
    };
    ws.onerror = () => ws.close();
    ws.onmessage = (ev) => this.onMessage(ev.data);
  }
  onMessage(text) {
    let msg;
    try { msg = JSON.parse(text); }
    catch { try { msg = JSON.parse(text.replace(/\b(-?Infinity|NaN)\b/g, "null")); } catch { return; } }
    if (msg.op === "publish") {
      for (const s of this.subs) if (s.topic === msg.topic) {
        try { s.cb(msg.msg); } catch (e) { console.error(s.topic, e); }
      }
    } else if (msg.op === "service_response" && this.calls[msg.id]) {
      this.calls[msg.id](msg.result ? msg.values : null); delete this.calls[msg.id];
    }
  }
  send(obj) { if (this.connected) this.ws.send(JSON.stringify(obj)); }
  subscribe(topic, type, cb, opts = {}) {
    const s = { topic, type, cb, opts: { queue_length: 1, ...opts } };
    this.subs.push(s);
    this.send({ op: "subscribe", topic, type, ...s.opts });
  }
  publish(topic, type, msg) {
    if (!this.connected) return false;
    if (!this.adv.has(topic)) { this.send({ op: "advertise", topic, type }); this.adv.add(topic); }
    this.send({ op: "publish", topic, msg });
    return true;
  }
  publishString(topic, data) { return this.publish(topic, "std_msgs/String", { data }); }
  callService(service, args = {}) {
    return new Promise((resolve) => {
      const id = "call_" + (++this.callId);
      this.calls[id] = resolve;
      this.send({ op: "call_service", id, service, args });
      setTimeout(() => { if (this.calls[id]) { delete this.calls[id]; resolve(null); } }, 4000);
    });
  }
}

// ---------------------------------------------------------------- state
const S = {
  scene: null, nav: { state: "unknown" }, navAt: 0, view: null, viewAt: 0, shownPose: null,
  people: null, peopleAt: 0, vision: null, visionAt: 0, visionTimes: [],
  voice: null, voiceAt: 0, map: null, mapImage: null,
};
let ros;

function pill(id, text, cls) { const el = $(id); el.textContent = text; el.className = "pill " + (cls || ""); }
function parseData(msg) { try { return JSON.parse(msg.data); } catch { return null; } }
function now() { return performance.now() / 1000; }
function esc(text) { const d = document.createElement("div"); d.textContent = text; return d.innerHTML; }

function addEvent(text) {
  const list = $("eventList");
  const row = document.createElement("div");
  row.className = "row";
  row.textContent = new Date().toLocaleTimeString() + "  " + text;
  list.prepend(row);
  while (list.children.length > 60) list.lastChild.remove();
}

// ---------------------------------------------------------------- page tabs & streams
function setupTabs() {
  for (const b of $("pageTabs").querySelectorAll("button")) {
    b.onclick = () => {
      for (const x of $("pageTabs").querySelectorAll("button")) x.classList.toggle("active", x === b);
      for (const p of document.querySelectorAll(".page")) p.classList.toggle("active", p.id === "page-" + b.dataset.page);
      // 离开导览页时断开它的相机流，Gazebo 就不再渲染（demo_web 3 秒后取消订阅）。
      const tour = b.dataset.page === "tour";
      for (const seg of document.querySelectorAll("#page-tour .seg")) {
        const img = $(seg.dataset.target);
        if (tour) img.src = streamUrl(seg.querySelector("button.active").dataset.stream);
        else img.removeAttribute("src");
      }
      if (b.dataset.page === "system") refreshSystem();
      if (b.dataset.page === "gait") loadGait();
    };
  }
  for (const seg of document.querySelectorAll(".seg")) {
    const img = $(seg.dataset.target);
    for (const b of seg.querySelectorAll("button")) {
      b.onclick = () => {
        for (const x of seg.querySelectorAll("button")) x.classList.toggle("active", x === b);
        img.src = streamUrl(b.dataset.stream);
      };
    }
    img.src = streamUrl(seg.querySelector("button.active").dataset.stream);
  }
}
function streamUrl(name) { return "/stream/" + name + "?t=" + Date.now(); }

// ---------------------------------------------------------------- controls
function goal(key) {
  if (!ros.publishString("/nav/goal_key", key)) addEvent("rosbridge 未连接，指令没发出去");
  else addEvent("导航指令：" + key);
}
function gesture(name) {
  ros.publishString("/guide/gesture", JSON.stringify({ name }));
  addEvent("手势：" + name);
}
function voiceCmd(cmd) { return ros.publishString("/guide/voice_cmd", JSON.stringify(cmd)); }

function setupControls() {
  const box = $("stationBtns");
  for (const e of S.scene.exhibits) {
    const b = document.createElement("button");
    b.className = "go";
    b.textContent = `${e.station.key}  ${e.title}`;
    b.title = e.exhibit_id;
    b.onclick = () => goal(e.station.key);
    box.appendChild(b);
  }
  for (const b of document.querySelectorAll("[data-goal]")) b.onclick = () => goal(b.dataset.goal);
  const gbox = $("gestureBtns");
  for (const [name, label] of GESTURES) {
    const b = document.createElement("button");
    b.textContent = label; b.title = name; b.onclick = () => gesture(name);
    gbox.appendChild(b);
  }
  for (const text of CHIPS) {
    const b = document.createElement("button");
    b.textContent = text; b.onclick = () => sendText(text);
    $("chips").appendChild(b);
  }
  $("sayForm").onsubmit = (ev) => {
    ev.preventDefault();
    const text = $("sayInput").value.trim();
    if (text) { sendText(text); $("sayInput").value = ""; }
  };
  $("btnInterrupt").onclick = () => voiceCmd({ cmd: "interrupt" });
  $("optAudience").onchange = (ev) => voiceCmd({ cmd: "style", audience: ev.target.value });
  $("optHumor").oninput = (ev) => { $("humorVal").textContent = ev.target.value; };
  $("optHumor").onchange = (ev) => voiceCmd({ cmd: "style", humor: +ev.target.value });
  $("optPresent").onchange = (ev) => {
    voiceCmd({ cmd: "present_mode", mode: ev.target.value });
    ros.publishString("/guide/present_mode", ev.target.value);   // 没开语音端时也生效
  };
  $("optWake").onchange = (ev) => voiceCmd({ cmd: "wake_word", on: ev.target.checked });
  $("optWakeMode").onchange = (ev) => voiceCmd({ cmd: "wake_mode", mode: ev.target.value });
  $("optCrowd").onchange = (ev) => voiceCmd({ cmd: "crowd", on: ev.target.checked });
  $("optMic").onchange = (ev) => voiceCmd({ cmd: "mic", on: ev.target.checked });
  $("optBarge").onchange = (ev) => voiceCmd({ cmd: "barge_in", on: ev.target.checked });
  $("optLog").onchange = (ev) => $("chat").classList.toggle("hide-log", !ev.target.checked);
  $("chat").classList.add("hide-log");
}
function sendText(text) {
  if (!voiceLive()) { addEvent("语音端没在运行（Windows：voice\\run.cmd 或 demo.cmd）"); return; }
  voiceCmd({ cmd: "text", text });
}

// ---------------------------------------------------------------- voice
let robotBubble = null;
function chatAdd(cls, text) {
  const chat = $("chat");
  const stick = chat.scrollTop + chat.clientHeight >= chat.scrollHeight - 30;
  const row = document.createElement("div");
  row.className = "msg " + cls;
  const bubble = document.createElement("div");
  bubble.className = "bubble";
  bubble.textContent = text;
  row.appendChild(bubble);
  chat.appendChild(row);
  while (chat.children.length > 400) chat.firstChild.remove();
  if (stick) chat.scrollTop = chat.scrollHeight;
  return bubble;
}
function onVoiceItem(item) {
  switch (item.kind) {
    case "user": robotBubble = null; chatAdd("user", item.text); break;
    case "event": robotBubble = null; chatAdd("event", item.text); break;
    case "reply_start": robotBubble = chatAdd("robot", ""); break;
    case "reply_chunk":
      if (!robotBubble) robotBubble = chatAdd("robot", "");
      robotBubble.textContent += item.text;
      { const chat = $("chat"); chat.scrollTop = chat.scrollHeight; }
      break;
    case "reply_end": robotBubble = null; break;
    case "log": chatAdd("log", item.text); break;
  }
}
let historyLoaded = false;
function onVoice(msg) {
  const data = parseData(msg);
  if (!data) return;
  if (data.kind === "state") {
    const first = !voiceLive();
    S.voice = data; S.voiceAt = now();
    if (first || !historyLoaded) { historyLoaded = true; voiceCmd({ cmd: "history" }); }
    syncVoiceControls(data, first);
    return;
  }
  if (data.kind === "history") {
    $("chat").innerHTML = ""; robotBubble = null;
    for (const item of data.items) onVoiceItem(item);
    return;
  }
  onVoiceItem(data);
}
function voiceLive() { return S.voice && now() - S.voiceAt < 2.0; }
function syncVoiceControls(v, force) {
  const set = (id, prop, value) => { const el = $(id); if (force || document.activeElement !== el) el[prop] = value; };
  set("optAudience", "value", v.audience);
  set("optHumor", "value", v.humor); $("humorVal").textContent = $("optHumor").value;
  set("optPresent", "value", v.present_mode);
  set("optWake", "checked", v.wake_word);
  set("optWakeMode", "value", v.wake_mode);
  set("optCrowd", "checked", v.crowd);
  set("optMic", "checked", v.mic);
  set("optBarge", "checked", v.barge_in);
  $("optMic").disabled = $("optBarge").disabled = !v.listen;
}
function renderVoice() {
  if (!voiceLive()) {
    $("voiceState").textContent = "语音端未运行"; $("voiceLamp").style.background = "#444";
    $("micLevel").style.width = "0"; $("llmLabel").textContent = "";
    pill("pillVoice", "语音 未运行", "bad");
    return;
  }
  const v = S.voice;
  const [text, color] = VOICE_STATE[v.state] || [v.state, "#6e7681"];
  $("voiceState").textContent = text;
  $("voiceLamp").style.background = color;
  $("micLevel").style.width = Math.min(100, v.level * 400) + "%";
  $("llmLabel").textContent = "大脑：" + v.llm;
  pill("pillVoice", "语音 " + text, v.state === "sleeping" ? "warn" : "ok");
}

// ---------------------------------------------------------------- nav / people / vision
function onNavStatus(msg) {
  const st = parseData(msg);
  if (!st) return;
  const prev = S.nav.state;
  S.nav = st; S.navAt = now();
  if (st.state !== prev) addEvent("导航：" + (NAV_STATE[st.state] || st.state) + (st.goal_key ? `（目标 ${st.goal_key}）` : ""));
  renderNav();
}
function renderNav() {
  const st = S.nav;
  const ex = S.scene && S.scene.exhibits.find((e) => e.station.key === String(st.goal_key));
  const target = ex ? ` → ${ex.station.key} ${ex.title}` : (st.goal_key ? ` → ${st.goal_key}` : "");
  pill("pillNav", "导航 " + (NAV_STATE[st.state] || st.state) + target, NAV_CLASS[st.state] || "");
  const skip = new Set(["state", "goal_key"]);
  const lines = [`状态：${NAV_STATE[st.state] || st.state}（${st.state}）${target}`];
  for (const [k, v] of Object.entries(st)) if (!skip.has(k)) lines.push(`${k}: ${typeof v === "object" ? JSON.stringify(v) : v}`);
  $("navDetail").textContent = lines.join("\n");
}
function onPeople(msg) {
  const p = parseData(msg);
  if (!p) return;
  S.people = p; S.peopleAt = now();
}
function renderPeople() {
  if (!S.people || now() - S.peopleAt > 2.0) {
    pill("pillPeople", "雷达人群 无数据", "");
    $("peopleList").textContent = "无数据（需要 lidar:=mid360）";
    return;
  }
  const people = S.people.people || [];
  const near = people.filter((p) => p.distance <= 3.0);
  const count = near.reduce((n, p) => n + (p.group || 1), 0);
  const child = near.length === 1 && near[0].child;
  pill("pillPeople", `3 m 内 ${count} 人${child ? "（儿童）" : ""}`, count ? "ok" : "");
  $("peopleList").innerHTML = people.length ? people.slice(0, 8).map((p) =>
    `<div class="row">#${p.id}　${p.distance.toFixed(1)} m　${p.bearing_deg > 0 ? "左" : "右"} ${Math.abs(p.bearing_deg).toFixed(0)}°` +
    `　${p.height_clipped ? "≥" : ""}${p.height.toFixed(2)} m` +
    (p.child ? '<span class="tag child">儿童</span>' : "") + (p.group > 1 ? `<span class="tag">${p.group} 人一团</span>` : "") +
    "</div>").join("") : "周围没有人";
}
function onVision(msg) {
  const v = parseData(msg);
  if (!v) return;
  S.vision = v; S.visionAt = now();
  S.visionTimes.push(S.visionAt);
  while (S.visionTimes.length && S.visionAt - S.visionTimes[0] > 3) S.visionTimes.shift();
  if (v.gesture) addEvent("视觉手势事件：" + (HAND[v.gesture] || v.gesture));
}
function visionLive() { return S.vision && now() - S.visionAt < 2.0; }
function renderVision() {
  if (!visionLive()) {
    pill("pillVision", "视觉 未运行", "");
    $("visionList").textContent = "无数据（需要 vision\\watch.py）";
    $("visionText").textContent = "";
    return;
  }
  const v = S.vision;
  const fps = S.visionTimes.length > 1 ? (S.visionTimes.length - 1) / (S.visionTimes.at(-1) - S.visionTimes[0]) : 0;
  pill("pillVision", `视觉 ${fps.toFixed(1)} 帧/秒`, "ok");
  const hand = v.hand ? (HAND[v.hand] || v.hand) : "无";
  $("visionList").innerHTML =
    `<div class="row">人 ${v.persons.length}，人脸 ${v.faces.length}，正对机器人 ${v.facing}</div>` +
    `<div class="row">当前手势：${esc(hand)}</div>` +
    v.persons.slice(0, 5).map((p) => `<div class="row">#${p.track_id ?? "?"}　方位 ${p.bearing_deg.toFixed(0)}°　置信度 ${p.conf.toFixed(2)}</div>`).join("");
  $("visionText").textContent = `YOLO：${v.persons.length} 人　人脸：${v.faces.length}（正对 ${v.facing}）　手势：${hand}`;
}

// 前置相机上叠加 YOLO 人框、人脸框（红 = 正对机器人）。
function drawVisionOverlay() {
  const canvas = $("visionOverlay"), img = $("robotImg");
  const box = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  if (canvas.width !== Math.round(box.width * dpr) || canvas.height !== Math.round(box.height * dpr)) {
    canvas.width = Math.round(box.width * dpr); canvas.height = Math.round(box.height * dpr);
  }
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, box.width, box.height);
  const front = document.querySelector('[data-target="robotImg"] button.active').dataset.stream === "front";
  if (!front || !visionLive()) return;
  const iw = img.naturalWidth || FRONT_W, ih = img.naturalHeight || FRONT_H;
  const scale = Math.min(box.width / iw, box.height / ih);
  const ox = (box.width - iw * scale) / 2, oy = (box.height - ih * scale) / 2;
  const sx = scale * iw / FRONT_W, sy = scale * ih / FRONT_H;
  const rect = (b, color, label) => {
    ctx.strokeStyle = color; ctx.lineWidth = 2;
    ctx.strokeRect(ox + b[0] * sx, oy + b[1] * sy, (b[2] - b[0]) * sx, (b[3] - b[1]) * sy);
    if (label) {
      ctx.font = "bold 14px sans-serif"; ctx.fillStyle = color;
      ctx.fillText(label, ox + b[0] * sx + 2, oy + b[1] * sy - 4);
    }
  };
  for (const p of S.vision.persons) rect(p.box, "#3fb950", (p.track_id != null ? "#" + p.track_id : "人") + " " + p.conf.toFixed(2));
  for (const f of S.vision.faces) rect(f.box, f.facing ? "#ff4040" : "#ffa940",
    (f.facing ? "正对 " : "") + (f.distance != null ? f.distance.toFixed(1) + " m" : ""));
}

// ---------------------------------------------------------------- 2D map
const map = { bounds: null, scale: 1, ox: 0, oy: 0, w: 0, h: 0, stations: [] };
function sceneBounds(scene) {
  let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity;
  const add = (x, y) => { x0 = Math.min(x0, x); y0 = Math.min(y0, y); x1 = Math.max(x1, x); y1 = Math.max(y1, y); };
  for (const r of scene.nav_obstacles || []) {
    add(r.position[0] - r.size[0] / 2, r.position[1] - r.size[1] / 2);
    add(r.position[0] + r.size[0] / 2, r.position[1] + r.size[1] / 2);
  }
  for (const e of scene.exhibits) { add(e.position[0], e.position[1]); add(e.station.x, e.station.y); }
  if (!isFinite(x0)) { const w = scene.room.width / 2, d = scene.room.depth / 2; return [-w, -d, w, d]; }
  return [x0 - 0.5, y0 - 0.5, x1 + 0.5, y1 + 0.5];
}
const W = (x) => map.ox + (x - map.bounds[0]) * map.scale;
const H = (y) => map.oy + (map.bounds[3] - y) * map.scale;

function drawRect(ctx, cx, cy, sx, sy, yaw) {
  const c = Math.cos(yaw), s = Math.sin(yaw);
  ctx.beginPath();
  for (const [dx, dy] of [[-1, -1], [1, -1], [1, 1], [-1, 1]]) {
    const x = cx + (dx * sx / 2) * c - (dy * sy / 2) * s, y = cy + (dx * sx / 2) * s + (dy * sy / 2) * c;
    ctx.lineTo(W(x), H(y));
  }
  ctx.closePath();
}
function drawMap() {
  const canvas = $("mapCanvas");
  const box = canvas.getBoundingClientRect();
  if (!S.scene || box.width < 10) return;
  const dpr = window.devicePixelRatio || 1;
  if (canvas.width !== Math.round(box.width * dpr) || canvas.height !== Math.round(box.height * dpr)) {
    canvas.width = Math.round(box.width * dpr); canvas.height = Math.round(box.height * dpr);
  }
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  const b = map.bounds = map.bounds || sceneBounds(S.scene);
  map.scale = Math.min(box.width / (b[2] - b[0]), box.height / (b[3] - b[1]));
  map.ox = (box.width - (b[2] - b[0]) * map.scale) / 2;
  map.oy = (box.height - (b[3] - b[1]) * map.scale) / 2;
  ctx.fillStyle = "#0d1217"; ctx.fillRect(0, 0, box.width, box.height);
  ctx.fillStyle = "#151d25"; ctx.fillRect(W(b[0]), H(b[3]), (b[2] - b[0]) * map.scale, (b[3] - b[1]) * map.scale);

  // SLAM 地图（slam_nav 模式下有 /map 时）
  if (S.mapImage) {
    const m = S.map.info, o = m.origin.position;
    ctx.globalAlpha = 0.8;
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(S.mapImage, W(o.x), H(o.y + m.height * m.resolution), m.width * m.resolution * map.scale, m.height * m.resolution * map.scale);
    ctx.globalAlpha = 1;
  }
  // 墙体（烘焙的矩形）和额外障碍
  ctx.fillStyle = "#5b6b7a";
  for (const r of [...(S.scene.nav_obstacles || []), ...(S.scene.obstacles || [])]) {
    drawRect(ctx, r.position[0], r.position[1], r.size[0], r.size[1], r.yaw || 0); ctx.fill();
  }
  // 静态参观者、走动的人的路线
  for (const p of S.scene.people || []) {
    if (p.kind === "static") {
      ctx.fillStyle = "#4a5866"; drawRect(ctx, p.position[0], p.position[1], p.size[0], p.size[1], p.yaw || 0); ctx.fill();
    } else if (p.waypoints) {
      ctx.strokeStyle = "rgba(139,154,168,0.35)"; ctx.setLineDash([4, 4]); ctx.lineWidth = 1;
      ctx.beginPath(); for (const [x, y] of [...p.waypoints, p.waypoints[0]]) ctx.lineTo(W(x), H(y)); ctx.stroke();
      ctx.setLineDash([]);
    }
  }
  // 展品和站位
  map.stations = [];
  ctx.font = "bold 12px sans-serif"; ctx.textAlign = "center"; ctx.textBaseline = "middle";
  for (const e of S.scene.exhibits) {
    ctx.fillStyle = e.kind === "poster" ? "#c9a227" : "#b8733a";
    drawRect(ctx, e.position[0], e.position[1], Math.max(e.size[0], 0.2), Math.max(e.size[1], 0.2), e.yaw || 0); ctx.fill();
    const sx = W(e.station.x), sy = H(e.station.y), r = Math.max(9, 0.35 * map.scale);
    const active = String(S.nav.goal_key) === e.station.key;
    ctx.fillStyle = active ? "#3fa7ff" : "rgba(63,167,255,0.25)";
    ctx.strokeStyle = "#3fa7ff"; ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.arc(sx, sy, r, 0, 2 * Math.PI); ctx.fill(); ctx.stroke();
    ctx.fillStyle = "#fff"; ctx.fillText(e.station.key, sx, sy);
    ctx.fillStyle = "#c9d1d9"; ctx.font = "12px sans-serif";
    const ty = e.position[1] > e.station.y ? H(e.position[1]) - 12 : H(e.position[1]) + 14;
    ctx.fillText(e.title, W(e.position[0]), Math.min(Math.max(ty, 10), box.height - 8));
    ctx.font = "bold 12px sans-serif";
    map.stations.push({ key: e.station.key, x: sx, y: sy, r: r + 4 });
  }
  const view = S.view && now() - S.viewAt < 2.0 ? S.view : null;
  if (view) {
    ctx.fillStyle = "#d29922";
    for (const [x, y] of view.obstacles) ctx.fillRect(W(x) - 2.5, H(y) - 2.5, 5, 5);
    ctx.fillStyle = "#f85149";
    for (const [x, y] of view.scan) ctx.fillRect(W(x) - 1.5, H(y) - 1.5, 3, 3);
    if (view.path.length > 1) {
      ctx.strokeStyle = "#3fb950"; ctx.lineWidth = 3; ctx.lineJoin = "round";
      ctx.beginPath(); for (const [x, y] of view.path) ctx.lineTo(W(x), H(y)); ctx.stroke();
      const [gx, gy] = view.path.at(-1);
      ctx.fillStyle = "#3fb950"; ctx.beginPath(); ctx.arc(W(gx), H(gy), 5, 0, 2 * Math.PI); ctx.fill();
    }
  }
  // 雷达人群
  if (S.people && now() - S.peopleAt < 2.0) {
    const target = S.nav.target_id ?? S.nav.follow_id;
    for (const p of S.people.people || []) {
      const x = W(p.x), y = H(p.y), r = Math.max(6, 0.25 * map.scale) * Math.sqrt(p.group || 1);
      ctx.fillStyle = p.child ? "#ff6ec7" : "#58a6ff";
      ctx.beginPath(); ctx.arc(x, y, r, 0, 2 * Math.PI); ctx.fill();
      if (target != null && String(target) === String(p.id)) {
        ctx.strokeStyle = "#fff"; ctx.lineWidth = 2; ctx.beginPath(); ctx.arc(x, y, r + 4, 0, 2 * Math.PI); ctx.stroke();
      }
      ctx.fillStyle = "#fff"; ctx.font = "11px sans-serif"; ctx.fillText(String(p.id), x, y - r - 7);
    }
    // 3 m 讲解范围
    if (view && view.pose) {
      ctx.strokeStyle = "rgba(88,166,255,0.35)"; ctx.setLineDash([5, 5]); ctx.lineWidth = 1;
      ctx.beginPath(); ctx.arc(W(S.shownPose[0]), H(S.shownPose[1]), 3 * map.scale, 0, 2 * Math.PI); ctx.stroke();
      ctx.setLineDash([]);
    }
  }
  // 机器人（显示位姿向最新位姿平滑过渡）
  if (view && view.pose) {
    const p = view.pose;
    if (!S.shownPose) S.shownPose = [...p];
    const k = 0.25, sp = S.shownPose;
    sp[0] += (p[0] - sp[0]) * k; sp[1] += (p[1] - sp[1]) * k;
    let dy = p[2] - sp[2]; dy = Math.atan2(Math.sin(dy), Math.cos(dy)); sp[2] += dy * k;
    const x = W(sp[0]), y = H(sp[1]), r = Math.max(8, 0.35 * map.scale), a = -sp[2];
    ctx.fillStyle = "#ff7b72"; ctx.strokeStyle = "#fff"; ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(x + Math.cos(a) * r * 1.4, y + Math.sin(a) * r * 1.4);
    ctx.lineTo(x + Math.cos(a + 2.4) * r, y + Math.sin(a + 2.4) * r);
    ctx.lineTo(x + Math.cos(a - 2.4) * r, y + Math.sin(a - 2.4) * r);
    ctx.closePath(); ctx.fill(); ctx.stroke();
  } else {
    ctx.fillStyle = "#8b9aa8"; ctx.textAlign = "left"; ctx.font = "13px sans-serif";
    ctx.fillText("等待 /nav/view（exhibition_nav 未运行？）", 10, 16);
  }
}
function onMapClick(ev) {
  const box = $("mapCanvas").getBoundingClientRect();
  const x = ev.clientX - box.left, y = ev.clientY - box.top;
  for (const s of map.stations) if (Math.hypot(s.x - x, s.y - y) <= s.r) { goal(s.key); return; }
}
function onOccupancy(msg) {
  const info = msg.info, data = msg.data;
  if (!info || !data || !data.length) return;
  const c = document.createElement("canvas");
  c.width = info.width; c.height = info.height;
  const ctx = c.getContext("2d"), img = ctx.createImageData(info.width, info.height);
  for (let row = 0; row < info.height; row++) {
    for (let col = 0; col < info.width; col++) {
      const v = data[row * info.width + col], i = ((info.height - 1 - row) * info.width + col) * 4;
      const g = v < 0 ? 40 : v > 50 ? 200 : 25;
      img.data[i] = img.data[i + 1] = img.data[i + 2] = g; img.data[i + 3] = v < 0 ? 0 : 255;
    }
  }
  ctx.putImageData(img, 0, 0);
  S.map = msg; S.mapImage = c;
}

// ---------------------------------------------------------------- gait page
let gaitLoaded = false;
async function loadGait() {
  if (gaitLoaded) return;
  gaitLoaded = true;
  const media = await (await fetch("/api/media")).json();
  const groups = {};
  for (const m of media) (groups[m.group] ||= []).push(m);
  const info = {
    g1_rough_n16384_mb8: ["第一版：基线", "骨盆 0.49 m（半蹲）；台阶、坡基本 ≥ 98%"],
    g1_rough_height072_n16384_mb8: ["第二版：+ 机身高度奖励", "骨盆 0.68 m；20° 坡降到 78%"],
    g1_rough_rand070_n16384_mb8: ["第三版：高度 0.70 + 域随机化", "骨盆 0.64 m；13 项全部 ≥ 98.4%，零摔倒（最好）"],
  };
  const order = Object.keys(groups).sort((a, b) => Object.keys(info).indexOf(b) - Object.keys(info).indexOf(a));
  const list = $("gaitList");
  list.innerHTML = order.length ? "" : "没有找到 mujoco/policies/*/demo/*.mp4";
  for (const g of order) {
    const [title, note] = info[g] || [g, ""];
    const card = document.createElement("div");
    card.className = "gait-card";
    card.innerHTML = `<h3>${esc(title)}${g.includes("rand070") ? '<span class="best">当前最好</span>' : ""}</h3>` +
      `<div class="note">${esc(note)}<br><code>${esc(g)}</code></div>`;
    for (const m of groups[g].filter((m) => m.kind === "mp4")) {
      const v = document.createElement("video");
      v.src = "/media/" + m.path; v.controls = true; v.loop = true; v.muted = true; v.preload = "metadata";
      card.appendChild(v);
    }
    const evals = groups[g].filter((m) => m.kind === "jsonl").sort((a, b) => b.size - a.size);
    if (evals.length) card.appendChild(await evalTable(evals[0]));
    list.appendChild(card);
  }
}
async function evalTable(item) {
  const terrain = { stairs_up: "上台阶", stairs_down: "下台阶", slope_up: "上坡", slope_down: "下坡",
    flat: "平地", rough: "起伏", boxes: "随机方块" };
  const rows = (await (await fetch("/media/" + item.path)).text()).trim().split("\n")
    .map((l) => { try { return JSON.parse(l); } catch { return null; } }).filter(Boolean);
  const table = document.createElement("table");
  table.innerHTML = "<tr><th>Isaac 测试地形</th><th>参数</th><th>速度</th><th>通过率</th><th>摔倒率</th><th>骨盆高度</th></tr>" +
    rows.map((r) => {
      const unit = String(r.terrain).startsWith("stairs") ? ` ${(r.value * 100).toFixed(0)} cm` :
        String(r.terrain).startsWith("slope") ? ` ${r.value}°` : (r.value ?? "");
      return `<tr><td>${esc(terrain[r.terrain] || r.terrain)}</td><td>${esc(String(unit))}</td><td>${r.vx ?? ""} m/s</td>` +
        `<td>${(r.pass_rate * 100).toFixed(1)}%</td><td>${(r.fall_rate * 100).toFixed(1)}%</td><td>${r.base_height != null ? r.base_height + " m" : "—"}</td></tr>`;
    }).join("");
  const wrap = document.createElement("div");
  wrap.innerHTML = `<div class="note" style="margin:6px 0 0">评估：${esc(item.name)}</div>`;
  wrap.appendChild(table);
  return wrap;
}

// ---------------------------------------------------------------- system page
async function refreshSystem() {
  try {
    const streams = await (await fetch("/api/streams")).json();
    $("streamList").innerHTML = streams.map((s) =>
      `<div class="row"><b>${esc(s.name)}</b>　<code>${esc(s.topic)}</code>　观看 ${s.clients}　` +
      (s.live ? `<span class="tag ok">有画面</span>` : `<span class="tag">无画面</span>`) + "</div>").join("");
  } catch { $("streamList").textContent = "demo_web 未响应"; }
  const topics = await ros.callService("/rosapi/topics");
  $("topicList").innerHTML = topics ? topics.topics.slice().sort().map((t) => `<div>${esc(t)}</div>`).join("")
    : "rosapi 不可用（rosbridge 未连接）";
}

// ---------------------------------------------------------------- main
async function main() {
  const config = await (await fetch("/api/config")).json();
  S.scene = await (await fetch("/api/scene")).json();
  ros = new Rosbridge(`ws://${location.hostname}:${config.rosbridge_port}`, (ok) => {
    pill("pillRos", ok ? "rosbridge 已连接" : "rosbridge 断开，重连中…", ok ? "ok" : "bad");
    if (ok) { historyLoaded = false; addEvent("rosbridge 已连接"); }
  });
  ros.subscribe("/nav/status", "std_msgs/String", onNavStatus, { queue_length: 5 });
  ros.subscribe("/nav/view", "std_msgs/String", (m) => { const v = parseData(m); if (v) { S.view = v; S.viewAt = now(); } }, { throttle_rate: 150 });
  ros.subscribe("/guide/people", "std_msgs/String", onPeople, { throttle_rate: 250 });
  ros.subscribe("/guide/vision", "std_msgs/String", onVision, { queue_length: 5 });
  ros.subscribe("/guide/gesture_status", "std_msgs/String", (m) => {
    const g = parseData(m); if (g) addEvent("手势状态：" + JSON.stringify(g));
  }, { queue_length: 5 });
  ros.subscribe("/guide/voice", "std_msgs/String", onVoice, { queue_length: 100 });
  ros.subscribe("/map", "nav_msgs/OccupancyGrid", onOccupancy, { throttle_rate: 5000 });

  setupTabs();
  setupControls();
  $("mapCanvas").addEventListener("click", onMapClick);
  $("btnTopics").onclick = refreshSystem;
  $("btnLowlevel").onclick = () => {
    const box = $("lowlevelBox"); box.hidden = !box.hidden;
    if (box.hidden) $("lowlevelImg").removeAttribute("src"); else $("lowlevelImg").src = streamUrl("lowlevel");
  };
  const frame = () => { drawMap(); drawVisionOverlay(); requestAnimationFrame(frame); };
  requestAnimationFrame(frame);
  setInterval(() => { renderVoice(); renderPeople(); renderVision();
    if (S.navAt && now() - S.navAt > 5 && S.nav.state !== "unknown") renderNav(); }, 250);
}
main().catch((e) => { document.body.insertAdjacentHTML("afterbegin", `<pre style="color:#f85149">${esc(String(e))}</pre>`); });
