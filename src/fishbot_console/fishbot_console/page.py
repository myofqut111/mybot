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
#mapWrap{position:relative;background:#111826;border-radius:8px;overflow:hidden;
         min-height:260px;display:flex;align-items:center;justify-content:center}
#mapCanvas{max-width:100%;cursor:crosshair;display:block}
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
  <section>
    <h2>地图（点一下 = 发导航目标）</h2>
    <div id="mapWrap"><div id="mapHint">还没有收到 /map</div>
      <canvas id="mapCanvas" hidden></canvas></div>
    <div class="row">
      <button id="btnCancelNav">取消导航</button>
      <input type="number" id="navX" placeholder="x" step="0.1">
      <input type="number" id="navY" placeholder="y" step="0.1">
      <button id="btnSendNav">发送</button>
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

/* ---------------- 地图 ---------------- */
function drawMap(){
  const canvas = $('mapCanvas');
  if(!MAPIMG || !MAP.width) return;
  canvas.hidden = false; $('mapHint').hidden = true;
  const avail = $('mapWrap').clientWidth || 480;
  const scale = Math.min(1, avail / MAP.width);
  canvas.width = MAP.width; canvas.height = MAP.height;
  canvas.style.width = (MAP.width*scale) + 'px';
  canvas.style.height = (MAP.height*scale) + 'px';
  const ctx = canvas.getContext('2d');
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(MAPIMG, 0, 0);
  const pose = STATE && STATE.status && STATE.status.pose;
  if(pose){
    const px = (pose.x - MAP.origin_x)/MAP.resolution;
    const py = (MAP.height-1) - (pose.y - MAP.origin_y)/MAP.resolution;
    ctx.fillStyle = '#4c8dff'; ctx.strokeStyle = '#fff'; ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.arc(px, py, 5, 0, Math.PI*2); ctx.fill(); ctx.stroke();
  }
}
async function loadMap(){
  try{
    const res = await fetch('/api/map.png?t=' + Date.now());
    if(!res.ok) return;
    const blob = await res.blob();
    const img = new Image();
    img.onload = ()=>{ MAPIMG = img; drawMap(); };
    img.src = URL.createObjectURL(blob);
  }catch(e){}
}
$('mapCanvas').addEventListener('click', ev=>{
  if(!MAPIMG) return;
  const canvas = $('mapCanvas');
  const rect = canvas.getBoundingClientRect();
  const px = (ev.clientX - rect.left) * (canvas.width / rect.width);
  const py = (ev.clientY - rect.top) * (canvas.height / rect.height);
  const wx = MAP.origin_x + px * MAP.resolution;
  const wy = MAP.origin_y + ((MAP.height-1) - py) * MAP.resolution;
  $('navX').value = wx.toFixed(2); $('navY').value = wy.toFixed(2);
  sendNav(wx, wy);
});
async function sendNav(x, y){
  try{
    const r = await api('/api/nav', {x:x, y:y});
    toast(r.ok ? r.message : ('失败：' + r.message));
  }catch(e){ toast('发送失败：' + e.message); }
}

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
pollState(); loadMap(); pollFrame(); pollLogs(); pollRuns();
setInterval(pollState, 1000);
setInterval(pollFrame, 700);
setInterval(pollLogs, 2000);
setInterval(pollRuns, 10000);
setInterval(loadMap, 6000);
</script>
</body>
</html>
"""
