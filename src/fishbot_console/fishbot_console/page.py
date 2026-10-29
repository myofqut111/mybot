#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""单页前端：一张 HTML 里塞完 CSS 和 JS（机器人没网，不引 CDN）。

页面分六块：状态条 / 地图 / 摄像头 / 巡检与建图按钮 / 日志 / 看板 + AI。
地图点击 = 发导航目标，这是这个页面最有用的一个交互。
"""

HTML = r"""<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>fishbot 控制台</title>
<style>
:root{--bg:#0f1420;--panel:#171e2c;--line:#26304a;--fg:#e6ebf5;--dim:#8b97ad;
      --ok:#37c98a;--warn:#f0b445;--err:#ef5f5f;--accent:#4c8dff;}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--fg);
     font:14px/1.5 -apple-system,"Segoe UI","Microsoft YaHei",sans-serif}
header{display:flex;flex-wrap:wrap;gap:14px;align-items:center;
       padding:10px 16px;background:var(--panel);border-bottom:1px solid var(--line)}
header h1{font-size:15px;margin:0 8px 0 0;font-weight:600;letter-spacing:.5px}
.badge{padding:2px 9px;border-radius:999px;background:#20293c;color:var(--dim);
       font-size:12px;border:1px solid var(--line)}
.badge.ok{color:var(--ok);border-color:#1e5c42}
.badge.warn{color:var(--warn);border-color:#5c4a1e}
.badge.err{color:var(--err);border-color:#5c2020}
#sysRow{display:flex;flex-wrap:wrap;gap:8px;padding:6px 16px 0;background:var(--panel)}
#sysRow:empty{display:none}
#sysHint{padding:4px 16px 8px;background:var(--panel);color:var(--warn);
         font-size:12px;line-height:1.6;border-bottom:1px solid var(--line)}
#sysHint:empty{display:none}
main{display:grid;gap:12px;padding:12px;
     grid-template-columns:minmax(320px,1.15fr) minmax(300px,1fr)}
@media(max-width:900px){main{grid-template-columns:1fr}}
section{background:var(--panel);border:1px solid var(--line);border-radius:10px;
        padding:10px}
section h2{font-size:13px;margin:0 0 8px;color:var(--dim);font-weight:600;
           letter-spacing:.6px;text-transform:uppercase}
#mapSection{grid-column:1/-1}
#mapWrap{position:relative;height:58vh;overflow:hidden;background:#111826;
                    border-radius:8px;
#map{position:absolute;inset:0;background:#111826;border-radius:8px}
.leaflet-container{background:#111826;font:inherit}
.leaflet-control-scale-line{background:rgba(17,24,38,.72);border-color:#3a4664;
                            color:#c9d3e6;font-size:11px}
#teleopPad{display:grid;grid-template-columns:repeat(3,54px);
           grid-template-rows:repeat(3,46px);gap:6px;justify-content:start}
#teleopPad button{font-size:17px}
#teleopPad .mid{opacity:.55}
#mapWrapOld{position:relative;background:#111826;border-radius:8px;overflow:auto;
         min-height:260px;height:60vh;display:block;text-align:center}
#mapCanvas{max-width:none;cursor:crosshair;display:inline-block;
           image-rendering:pixelated}
#mapWrap.mode-pose #mapCanvas{cursor:crosshair}
.chk{display:flex;align-items:center;gap:5px;font-size:12px;color:var(--dim)}
.chk input{width:auto}
#mapHint{color:var(--dim);font-size:12px;padding:20px;text-align:center}
.row{display:flex;gap:8px;flex-wrap:wrap;align-items:center;margin-top:8px}
button{background:#22304a;color:var(--fg);border:1px solid var(--line);
       border-radius:7px;padding:7px 12px;cursor:pointer;font-size:13px}
button:hover{background:#2a3a58}
button.primary{background:var(--accent);border-color:var(--accent);color:#fff}
button.danger{background:#5c2020;border-color:#7a2a2a}
input,select{background:#111826;color:var(--fg);border:1px solid var(--line);
             border-radius:7px;padding:6px 9px;font-size:13px}
input[type=number]{width:78px}
#cam{width:100%;border-radius:8px;background:#0b101a;display:block}
#logList{max-height:230px;overflow:auto;font:12px/1.55 ui-monospace,Consolas,monospace}
#logList div{padding:2px 4px;border-bottom:1px solid #1d2434;white-space:pre-wrap;
             word-break:break-all}
.lv-DEBUG{color:#5f6b80}.lv-INFO{color:#c6cfdd}.lv-WARN{color:var(--warn)}
.lv-ERROR,.lv-FATAL{color:var(--err)}
table{width:100%;border-collapse:collapse;font-size:12px}
th,td{text-align:left;padding:4px 6px;border-bottom:1px solid #1d2434}
th{color:var(--dim);font-weight:500}
#chatLog{max-height:200px;overflow:auto;display:flex;flex-direction:column;gap:6px}
.bubble{padding:7px 10px;border-radius:9px;max-width:92%;font-size:13px;
        white-space:pre-wrap}
.bubble.me{align-self:flex-end;background:#243b63}
.bubble.ai{align-self:flex-start;background:#1c2536;border:1px solid var(--line)}
.bubble.act{border-left:3px solid var(--accent)}
#toast{position:fixed;left:50%;bottom:24px;transform:translateX(-50%);
       background:#22304a;border:1px solid var(--line);padding:9px 16px;
       border-radius:9px;opacity:0;transition:opacity .25s;pointer-events:none}
</style>
<link rel="stylesheet" href="/static/leaflet.css">
<script src="/static/leaflet.js"></script>
</head>
<body>
<header>
  <h1>fishbot 控制台</h1>
  <span class="badge" id="bState">—</span>
  <span class="badge" id="bBatt">电量 —</span>
  <span class="badge" id="bPose">位姿 —</span>
  <span class="badge" id="bCam">相机 —</span>
  <span class="badge" id="bErr">错误 0</span>
</header>
<div id="sysRow"></div>
<div id="sysHint"></div>

<main>
  <section id="mapSection">
    <h2>地图</h2>
    <div class="row" style="margin-top:0">
      <select id="mapMode" title="左键模式">
        <option value="pan">左键拖拽 = 平移地图（滚轮/双指缩放）</option>
        <option value="nav">左键拖拽 = 发导航目标（带朝向）</option>
        <option value="pose">左键拖拽 = 设定初始位姿</option>
      </select>
      <button id="btnZoomOut" title="缩小">−</button>
      <span class="badge" id="zoomLabel">100%</span>
      <button id="btnZoomIn" title="放大">+</button>
      <button id="btnZoomFit">适应窗口</button>
      <label class="chk"><input type="checkbox" id="chkCostmap" checked> 代价地图</label>
      <label class="chk"><input type="checkbox" id="chkScan" checked> 雷达点云</label>
      <label class="chk"><input type="checkbox" id="chkPlan" checked> 全局路径</label>
      <span id="mapCoord" class="chk">—</span>
      <span id="mapModeHint" style="color:var(--dim);font-size:12px"></span>
    </div>
    <div id="mapWrap"><div id="map"></div>
      <div id="mapHint" style="position:absolute;left:12px;top:12px;z-index:500;
        background:rgba(17,24,38,.8);padding:4px 8px;border-radius:6px">
        还没有收到 /map</div></div>
    <div class="row">
      <button id="btnCancelNav">取消导航</button>
      <input type="number" id="navX" placeholder="x" step="0.1">
      <input type="number" id="navY" placeholder="y" step="0.1">
      <button id="btnSendNav">发送</button>
    </div>
  </section>

  <section>
    <h2>手动控制</h2>
    <div class="row" style="margin-top:0">
      <span class="chk">按住方向键行走，松手立即停（0.6 s 无刷新服务端也会停机）</span>
      <input id="teleSpeed" type="range" min="0.05" max="0.6" step="0.05" value="0.2"
             style="width:120px">
      <span class="badge" id="teleLabel">0.20 m/s</span>
    </div>
    <div id="teleopPad">
      <span></span><button data-v="1" data-w="0" title="前进（W）">▲</button><span></span>
      <button data-v="0" data-w="1" title="左转（A）">↺</button>
      <button data-v="0" data-w="0" class="mid" title="停（空格）">■</button>
      <button data-v="0" data-w="-1" title="右转（D）">↻</button>
      <span></span><button data-v="-1" data-w="0" title="后退（S）">▼</button><span></span>
    </div>
  </section>

  <section>
    <h2>摄像头</h2>
    <img id="cam" alt="等待画面">
    <div class="row">
      <select id="camKind">
        <option value="anno">检测标注图</option>
        <option value="raw">原始图</option>
      </select>
      <button id="btnCapture">抓拍存盘</button>
    </div>
  </section>

  <section>
    <h2>巡检</h2>
    <div class="row">
      <input id="patrolRoute" placeholder="路线名（空=默认）" size="12">
      <select id="patrolMode">
        <option value="0">单次</option><option value="1">重复</option>
        <option value="2">随机</option>
      </select>
      <input type="number" id="patrolRepeat" value="1" min="1" max="50">
      <button class="primary" id="btnPatrol">开始巡检</button>
      <button class="danger" id="btnPatrolCancel">停止</button>
    </div>
  </section>

  <section>
    <h2>建图</h2>
    <div class="row">
      <input id="mapName" placeholder="地图名" value="map" size="10">
      <button id="btnMapStart">开始建图</button>
      <button id="btnMapStop">停止建图</button>
      <button id="btnMapSave">保存地图</button>
    </div>
  </section>

  <section style="grid-column:1/-1">
    <h2>日志</h2>
    <div class="row" style="margin:0 0 8px">
      <select id="logLevel">
        <option value="">全部</option><option value="WARN">仅 WARN 以上</option>
        <option value="ERROR">仅 ERROR 以上</option>
      </select>
      <span class="badge" id="logCount">0</span>
    </div>
    <div id="logList"></div>
  </section>

  <section>
    <h2>历史巡检</h2>
    <table><thead><tr><th>时间</th><th>路线</th><th>点</th><th>成功</th>
      <th>耗时</th><th>召回</th></tr></thead><tbody id="runsBody"></tbody></table>
    <div id="events" style="margin-top:8px;color:var(--dim);font-size:12px"></div>
  </section>

  <section>
    <h2>AI 助手</h2>
    <div id="chatLog"><div class="bubble ai">问我：现在什么状态？还有多少电？
      去 x=1.2 y=-0.5</div></div>
    <div class="row">
      <input id="askBox" placeholder="说点什么…" style="flex:1">
      <button class="primary" id="btnAsk">发送</button>
    </div>
  </section>
</main>
<div id="toast"></div>

<script>
const $ = id => document.getElementById(id);
let MAP = {width:0,height:0,resolution:0.05,origin_x:0,origin_y:0,
           min_x:0,max_x:0,min_y:0,max_y:0};
let MAPIMG = null;
let LAYERSIMG = null;        // 代价地图叠加层（RGBA，尺寸同底图）
let SCAN = [];               // 雷达点（map 坐标系）
let PLAN = [];               // nav2 全局路径（map 坐标系）
let PAN = null;              // 中键/右键/双指拖动平移的状态
let TOUCH = null;            // 双指缩放的中心与距离
let GOAL = null;             // 最近一次发的导航目标，画个绿箭头
let NAVDRAG = null;          // 发目标时的拖拽状态（位置+朝向）
let SUPPRESS_CLICK = false;  // 拖拽结束后浏览器还会补一个 click，要吞掉

function toast(text){
  const el = $('toast'); el.textContent = text; el.style.opacity = 1;
  clearTimeout(el._t); el._t = setTimeout(()=>{el.style.opacity = 0}, 2200);
}
async function api(path, body){
  const opt = body === undefined ? {} : {
    method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify(body)};
  const res = await fetch(path, opt);
  if(!res.ok) throw new Error(res.status + ' ' + res.statusText);
  return await res.json();
}
function setBadge(el, text, cls){ el.textContent = text; el.className = 'badge ' + (cls||''); }

/* ---------------- 地图（Leaflet，手感对齐 RViz） ---------------- */
const MAPM = {ready:false, key:'', w:0, h:0, res:0.05, ox:0, oy:0, yaw:0};
const CANVAS_R = L.canvas({padding:0.4});
const LL = {};
let LF = null, DRAG = null;

function ensureMap(){
  if(LF) return LF;
  LF = L.map('map', {crs:L.CRS.Simple, zoomControl:false,
                     attributionControl:false, doubleClickZoom:false,
                     zoomSnap:0.25, zoomDelta:0.5, minZoom:-4, maxZoom:8,
                     renderer:CANVAS_R});
  LL.plan = L.layerGroup().addTo(LF);
  LL.scan = L.layerGroup().addTo(LF);
  LL.goal = L.layerGroup().addTo(LF);
  LL.robot = L.layerGroup().addTo(LF);
  LF.on('mousemove', ev => {
    if(DRAG) return;
    $('mapCoord').textContent = ev.latlng.lng.toFixed(2) + ', ' +
                                ev.latlng.lat.toFixed(2) + ' m';
  });
  LF.on('zoomend', () => {
    $('zoomLabel').textContent = Math.round(Math.pow(2, LF.getZoom()) * 100) + '%';
  });
  LF.on('mousedown', onDown);
  LF.on('mousemove', onMove);
  LF.on('mouseup', onUp);
  return LF;
}

function mapBounds(){
  return [[MAPM.oy, MAPM.ox],
          [MAPM.oy + MAPM.h * MAPM.res, MAPM.ox + MAPM.w * MAPM.res]];
}
function robotIcon(yaw){
  const deg = -yaw * 180 / Math.PI;
  return L.divIcon({className:'', iconSize:[26,26], iconAnchor:[13,13],
    html:'<svg width="26" height="26" viewBox="0 0 26 26" ' +
         'style="transform:rotate(' + deg.toFixed(1) + 'deg)">' +
         '<polygon points="13,1 23,24 13,19 3,24" fill="#4c8dff" ' +
         'stroke="#eaf1ff" stroke-width="1.2"/></svg>'});
}
function goalIcon(yaw){
  const deg = -yaw * 180 / Math.PI;
  return L.divIcon({className:'', iconSize:[30,30], iconAnchor:[15,15],
    html:'<svg width="30" height="30" viewBox="0 0 30 30" ' +
         'style="transform:rotate(' + deg.toFixed(1) + 'deg)">' +
         '<polygon points="15,2 21,26 15,21 9,26" fill="#37c98a" ' +
         'fill-opacity=".85" stroke="#d8ffe9" stroke-width="1"/></svg>'});
}

async function loadMap(){
  ensureMap();
  let st;
  try{ st = await api('/api/state'); }catch(e){ return; }
  const m = (st && st.map) || {};
  if(!m.ready){ $('mapHint').style.display = 'block'; return; }
  $('mapHint').style.display = 'none';
  const res = +(m.resolution || m.res || 0);
  const w = m.width | 0, h = m.height | 0;
  if(!(res > 0) || !w || !h) return;
  const ox = (m.origin_x != null ? +m.origin_x : 0);
  const oy = (m.origin_y != null ? +m.origin_y : 0);
  const key = [w, h, res, ox, oy].join(',');
  if(key !== MAPM.key){
    MAPM.key = key;
    Object.assign(MAP, {ready:true, w:w, h:h, res:res, ox:ox, oy:oy});
    if(LL.img){ LF.removeLayer(LL.img); }
    LL.img = L.imageOverlay('/api/map.png?t=' + Date.now(), mapBounds(),
                            {opacity:1}).addTo(LF);
    LL.img.bringToBack();
    if(LL.cost){ LF.removeLayer(LL.cost); LL.cost = null; }
    LF.setMaxBounds(mapBounds());
    LF.fitBounds(mapBounds());
  }
  drawRobot(st);
  MAPM.yaw = (st.pose && st.pose.yaw) || 0;
}

function drawRobot(st){
  if(!MAPM.ready) return;
  const pose = (st && st.pose) || null;
  LL.robot.clearLayers();
  if(!pose || pose.x == null) return;
  const yaw = pose.yaw || 0;
  L.marker([pose.y, pose.x], {icon:robotIcon(yaw), interactive:false,
                              keyboard:false}).addTo(LL.robot);
}

function paint(){ loadLayers(); pollScan(); pollPlan(); }
function drawMap(){ loadMap(); }
function drawMapZoom(){ /* Leaflet 自己处理缩放 */ }
function fitZoom(){ if(LF && MAPM.ready) LF.fitBounds(mapBounds()); }
function zoomBy(factor){ if(LF) LF.setZoom(LF.getZoom() + Math.log2(factor)); }

async function loadLayers(){
  if(!MAPM.ready) return;
  if(!$('chkCostmap').checked){
    if(LL.cost){ LF.removeLayer(LL.cost); LL.cost = null; }
    return;
  }
  if(!LL.cost){
    LL.cost = L.imageOverlay(
      'data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7',
      mapBounds(), {opacity:0.85, interactive:false}).addTo(LF);
  }
  const element = LL.cost.getElement();
  if(element) element.src = '/api/layers.png?t=' + Date.now();
}

async function pollScan(){
  if(!MAPM.ready) return;
  if(!$('chkScan').checked){ LL.scan.clearLayers(); return; }
  let r;
  try{ r = await api('/api/scan'); }catch(e){ return; }
  const points = (r && r.points) || [];
  LL.scan.clearLayers();
  for(let i = 0; i < points.length; i++){
    const p = points[i];
    L.circleMarker([p[1], p[0]], {radius:1.6, stroke:false,
      fillColor:'#ff5b5b', fillOpacity:0.92, renderer:CANVAS_R,
      interactive:false}).addTo(LL.scan);
  }
}

async function pollPlan(){
  if(!MAPM.ready) return;
  if(!$('chkPlan').checked){ LL.plan.clearLayers(); return; }
  let r;
  try{ r = await api('/api/plan.json'); }catch(e){ return; }
  const points = (r && r.points) || [];
  LL.plan.clearLayers();
  if(points.length < 2) return;
  L.polyline(points.map(p => [p[1], p[0]]),
             {color:'#37c98a', weight:2, opacity:0.9,
              interactive:false}).addTo(LL.plan);
}

function mapMode(){ return $('mapMode').value; }
function updateMapMode(){
  ensureMap();
  const mode = mapMode();
  if(mode === 'pan'){
    LF.dragging.enable();
    $('mapWrap').style.cursor = 'grab';
    $('mapModeHint').textContent = '拖动平移，滚轮/双指缩放，双击放大';
  }else{
    LF.dragging.disable();
    $('mapWrap').style.cursor = 'crosshair';
    $('mapModeHint').textContent = mode === 'nav'
      ? '按下定位置、拖出朝向、松手直接发目标'
      : '按下定位姿、拖出朝向、松手设为初始位姿';
  }
}
$('mapMode').addEventListener('change', updateMapMode);
$('btnZoomIn').addEventListener('click', () => zoomBy(1.4));
$('btnZoomOut').addEventListener('click', () => zoomBy(1 / 1.4));
$('btnZoomFit').addEventListener('click', () => fitZoom());
$('chkCostmap').addEventListener('change', () => paint());
$('chkScan').addEventListener('change', () => paint());
$('chkPlan').addEventListener('change', () => paint());

function onDown(ev){
  if(mapMode() === 'pan') return;
  LF.dragging.disable();
  DRAG = {x:ev.latlng.lng, y:ev.latlng.lat, yaw:null};
  updateGhost();
}
function onMove(ev){
  if(!DRAG) return;
  const dx = ev.latlng.lng - DRAG.x, dy = ev.latlng.lat - DRAG.y;
  const len = Math.sqrt(dx * dx + dy * dy);
  if(len > MAPM.res * 8) DRAG.yaw = Math.atan2(dy, dx);
  $('mapCoord').textContent = '→ ' + DRAG.x.toFixed(2) + ', ' +
      DRAG.y.toFixed(2) + (DRAG.yaw == null ? '' :
      '  ' + Math.round(DRAG.yaw * 180 / Math.PI) + '°');
  updateGhost();
}
function updateGhost(){
  LL.goal.clearLayers();
  if(!DRAG) return;
  const yaw = (DRAG.yaw == null ? MAPM.yaw : DRAG.yaw);
  L.marker([DRAG.y, DRAG.x], {icon:goalIcon(yaw), interactive:false,
                              keyboard:false}).addTo(LL.goal);
}
async function onUp(){
  if(!DRAG) return;
  const done = DRAG;
  DRAG = null;
  const yaw = (done.yaw == null ? MAPM.yaw : done.yaw);
  if(mapMode() === 'pose'){
    LL.goal.clearLayers();
    try{ await api('/api/initialpose', {x:done.x, y:done.y, yaw:yaw}); }catch(e){}
    return;
  }
  setGoal(done.x, done.y, yaw);
  sendNav(done.x, done.y, yaw);
}
function setGoal(x, y, yaw){
  $('navX').value = x.toFixed(2);
  $('navY').value = y.toFixed(2);
  LL.goal.clearLayers();
  L.marker([y, x], {icon:goalIcon(yaw), interactive:false,
                    keyboard:false}).addTo(LL.goal);
}
async function sendNav(x, y, yaw){
  try{
    const r = await api('/api/nav', {x:x, y:y, yaw:yaw});
    if(!r.ok) bubble('导航失败：' + (r.message || ''), 'ai');
  }catch(e){ bubble('导航失败：' + e.message, 'ai'); }
}

/* ---------------- 手动控制 ---------------- */
let TELE = {timer:null, vx:0, wz:0};
function teleSpeed(){ return parseFloat($('teleSpeed').value) || 0.2; }
function teleStart(vx, wz){
  teleStop();
  const speed = teleSpeed();
  TELE.vx = vx * speed; TELE.wz = wz * speed * 5;
  sendTele();
  TELE.timer = setInterval(sendTele, 100);
}
function teleStop(){
  if(TELE.timer){ clearInterval(TELE.timer); TELE.timer = null; }
  TELE.vx = 0; TELE.wz = 0;
  sendTele();
}
function sendTele(){
  api('/api/teleop', {vx:TELE.vx, wz:TELE.wz}).catch(() => {});
}
document.querySelectorAll('#teleopPad button').forEach(btn => {
  const v = parseFloat(btn.dataset.v), w = parseFloat(btn.dataset.w);
  btn.addEventListener('pointerdown', ev => {
    ev.preventDefault(); teleStart(v, w);
  });
  btn.addEventListener('pointerup', ev => { ev.preventDefault(); teleStop(); });
  btn.addEventListener('pointerleave', () => teleStop());
  btn.addEventListener('pointercancel', () => teleStop());
});
$('teleSpeed').addEventListener('input', () => {
  $('teleLabel').textContent = teleSpeed().toFixed(2) + ' m/s';
  if(TELE.timer){ teleStop(); }
});
window.addEventListener('blur', teleStop);

/* ---------------- 状态轮询 ---------------- */
let STATE = null;
async function pollState(){
  try{
    STATE = await api('/api/state');
    const st = STATE.status || {};
    setBadge($('bState'), st.state || '空闲', st.running ? 'ok' : '');
    if(st.battery_v != null){
      const cls = st.battery_band === 'crit' ? 'err'
                : st.battery_band === 'warn' ? 'warn' : 'ok';
      setBadge($('bBatt'), '电量 ' + st.battery_v.toFixed(2) + 'V', cls);
    }else{ setBadge($('bBatt'), '电量 —', ''); }
    if(st.pose){
      setBadge($('bPose'), '位姿 ' + st.pose.x.toFixed(2) + ',' + st.pose.y.toFixed(2),
               st.pose.age_s > 3 ? 'warn' : 'ok');
    }else{ setBadge($('bPose'), '位姿 —', ''); }
    const age = STATE.camera ? STATE.camera.anno_age_s : null;
    if(age == null){ setBadge($('bCam'), '相机 无', 'err'); }
    else if(age > 3){ setBadge($('bCam'), '相机 ' + age.toFixed(1) + 's', 'warn'); }
    else{ setBadge($('bCam'), '相机 ' + age.toFixed(1) + 's', 'ok'); }
    const counts = STATE.log_counts || {};
    const errs = (counts.ERROR||0) + (counts.FATAL||0);
    setBadge($('bErr'), '错误 ' + errs, errs ? 'err' : '');
    if(STATE.map && STATE.map.width && !MAPIMG) { MAP = STATE.map; loadMap(); }
    else if(STATE.map && STATE.map.width) MAP = STATE.map;
    renderSubsystems(STATE.subsystems || []);
    drawMap();
  }catch(e){}
}
function renderSubsystems(items){
  const row = $('sysRow'), hint = $('sysHint');
  if(!row || !items.length){ if(row) row.textContent = ''; if(hint) hint.textContent = ''; return; }
  row.innerHTML = items.map(function(it){
    return '<span class="badge ' + (it.ok ? 'ok' : 'err') + '" title="'
         + (it.detail || '') + '">' + (it.ok ? '✓ ' : '✗ ') + it.label + '</span>';
  }).join('');
  const missing = items.filter(function(it){ return !it.ok; });
  hint.textContent = missing.length
    ? '缺少：' + missing.map(function(it){ return it.label + '（' + it.hint + '）'; }).join('；')
    : '';
}
async function pollFrame(){
  const kind = $('camKind').value;
  const img = $('cam');
  const next = new Image();
  next.onload = ()=>{ img.src = next.src; };
  next.onerror = ()=>{};
  next.src = '/api/frame.jpg?kind=' + kind + '&t=' + Date.now();
}
async function pollLogs(){
  try{
    const level = $('logLevel').value;
    const r = await api('/api/logs?limit=150' + (level ? ('&level=' + level) : ''));
    const box = $('logList');
    const atBottom = box.scrollTop + box.clientHeight >= box.scrollHeight - 40;
    box.innerHTML = (r.items||[]).map(i =>
      '<div class="lv-' + i.level + '">' + i.time + ' ' + i.level + ' ' +
      i.node + ' ' + escapeHtml(i.text) + '</div>').join('');
    if(atBottom) box.scrollTop = box.scrollHeight;
    $('logCount').textContent = (r.items||[]).length + ' 条';
  }catch(e){}
}
async function pollRuns(){
  try{
    const r = await api('/api/runs');
    $('runsBody').innerHTML = (r.items||[]).slice(0,8).map(x => {
      const t = x.started_at ? new Date(x.started_at*1000).toLocaleTimeString() : '—';
      return '<tr><td>' + t + '</td><td>' + (x.route_name||'—') + '</td><td>' +
        x.total + '</td><td>' + x.ok + '</td><td>' + fmtDur(x.duration_sec) +
        '</td><td>' + fmtPct(x.recall) + '</td></tr>';
    }).join('') || '<tr><td colspan="6" style="color:var(--dim)">还没有完成的巡检</td></tr>';
  }catch(e){}
}
function fmtDur(s){ if(s==null) return '—'; s = Math.round(s);
  return s<60 ? s+'s' : (s<3600 ? Math.floor(s/60)+'m' : Math.floor(s/3600)+'h'); }
function fmtPct(v){ return v==null ? '—' : (v*100).toFixed(1)+'%'; }
function escapeHtml(s){ return String(s).replace(/[&<>]/g, c =>
  ({'&':'&amp;','<':'&lt;','>':'&gt;'}[c])); }

/* ---------------- 按钮 ---------------- */
async function act(path, body, okText){
  try{
    const r = await api(path, body);
    toast(r.ok ? (okText || r.message) : ('失败：' + (r.message || '')));
    if(!r.ok) return;
  }catch(e){ toast('失败：' + e.message); }
}
$('btnSendNav').onclick = ()=>{
  const x = parseFloat($('navX').value), y = parseFloat($('navY').value);
  if(isNaN(x) || isNaN(y)) return toast('先填 x/y');
  sendNav(x, y);
};
$('btnCancelNav').onclick = ()=> act('/api/cancel_nav', {});
$('btnCapture').onclick = ()=> act('/api/capture', {}, '已抓拍');
$('btnPatrol').onclick = ()=> act('/api/patrol', {
  route: $('patrolRoute').value, mode: parseInt($('patrolMode').value),
  repeat: parseInt($('patrolRepeat').value)});
$('btnPatrolCancel').onclick = ()=> act('/api/patrol/cancel', {});
$('btnMapStart').onclick = ()=> act('/api/mapping', {action:'start'}, '建图已启动');
$('btnMapStop').onclick = ()=> act('/api/mapping', {action:'stop'}, '建图已停止');
$('btnMapSave').onclick = ()=> act('/api/mapping',
  {action:'save', name: $('mapName').value}, '地图已保存');
$('camKind').onchange = pollFrame;
$('logLevel').onchange = pollLogs;

/* ---------------- AI ---------------- */
function bubble(text, cls){
  const div = document.createElement('div');
  div.className = 'bubble ' + cls; div.textContent = text;
  $('chatLog').appendChild(div);
  $('chatLog').scrollTop = $('chatLog').scrollHeight;
  return div;
}
async function ask(){
  const q = $('askBox').value.trim();
  if(!q) return;
  $('askBox').value = '';
  bubble(q, 'me');
  try{
    const r = await api('/api/ask', {q:q});
    bubble(r.reply || '(空)', 'ai' + (r.action ? ' act' : ''));
    if(r.action){
      const ok = await runAction(r.action);
      bubble((ok ? '✔ ' : '✘ ') + describe(r.action), 'ai');
    }
  }catch(e){ bubble('出错了：' + e.message, 'ai'); }
}
function describe(a){
  if(a.type === 'nav') return '导航到 (' + a.x.toFixed(2) + ', ' + a.y.toFixed(2) + ')';
  if(a.type === 'patrol') return '开始巡检 ' + (a.route || '');
  if(a.type === 'pause') return '暂停';
  if(a.type === 'resume') return '恢复';
  if(a.type === 'cancel_nav') return '取消导航';
  return a.type;
}
async function runAction(a){
  try{
    if(a.type === 'nav'){ const r = await api('/api/nav', {x:a.x, y:a.y}); return r.ok; }
    if(a.type === 'patrol'){ const r = await api('/api/patrol',
      {route:a.route, mode:a.mode||0, repeat:a.repeat||1}); return r.ok; }
    if(a.type === 'cancel_nav'){ const r = await api('/api/cancel_nav', {}); return r.ok; }
    if(a.type === 'pause'){ const r = await api('/api/patrol/cancel', {}); return r.ok; }
    if(a.type === 'resume'){ return true; }
    return false;
  }catch(e){ return false; }
}
$('btnAsk').onclick = ask;
$('askBox').addEventListener('keydown', e => { if(e.key === 'Enter') ask(); });

/* ---------------- 循环 ---------------- */
updateMapMode();
pollState(); loadMap(); pollFrame(); pollLogs(); pollRuns();
setInterval(pollState, 1000);
setInterval(pollFrame, 700);
setInterval(pollLogs, 2000);
setInterval(pollRuns, 10000);
setInterval(loadMap, 6000);
setInterval(loadLayers, 1000);      // 代价地图每秒重取一次（服务端按版本缓存）
setInterval(pollScan, 250);         // 点云要跟得上移动，4 Hz
setInterval(pollPlan, 700);         // 全局路径
</script>
</body>
</html>
"""
