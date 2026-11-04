# -*- coding: utf-8 -*-
"""控制台页面（0.5.0）：只做业务，不再承担地图/导航/建图。

为什么把地图整块删掉：2026-10 之前这个页面用 Leaflet 复刻 RViz 的地图交互，
连修了四轮（拖拽橡皮筋回弹、两层代价图叠在一起滑走、maxBounds 被强改导致
地图空白、pad() 不存在），每次都是"看着能用、一上手就别扭"。而 RViz 那边
这些早就有了，还带 2D Pose Estimate 和 map_saver。

所以分工改成：**控制台管"点了会发生什么、出了事怎么看出来"，RViz 管专业操作**。
这一版页面只剩四件事：启停业务栈、看状态、看报告、下发巡检。

页面不引用任何外部资源：没有 CDN、没有 Leaflet，一个 urllib 就能拿全。
"""

HTML = r'''<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>fishbot 控制台</title>
<style>
:root{
  --bg:#0f1419; --panel:#161d26; --panel2:#1e2734; --line:#2a3543;
  --fg:#d8e1ea; --dim:#8695a6; --ok:#43b581; --warn:#e0a33e; --bad:#e05c5c;
  --accent:#3d8bfd;
}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--fg);
  font:14px/1.5 -apple-system,"Segoe UI","Microsoft YaHei",sans-serif}
h1{font-size:16px;margin:0 12px 0 0;font-weight:600}
header{display:flex;flex-wrap:wrap;align-items:center;gap:8px;
  padding:10px 14px;background:var(--panel);border-bottom:1px solid var(--line);
  position:sticky;top:0;z-index:9}
.badge{padding:3px 9px;border-radius:11px;background:var(--panel2);
  border:1px solid var(--line);font-size:12px;white-space:nowrap}
.badge.ok{color:var(--ok);border-color:#245c44}
.badge.warn{color:var(--warn);border-color:#5c4a24}
.badge.bad{color:var(--bad);border-color:#5c2b2b}
#btnEstop{margin-left:auto;padding:9px 18px;border-radius:8px;cursor:pointer;
  background:#b3312f;color:#fff;border:1px solid #d45452;font-weight:600;
  font-size:14px;letter-spacing:1px}
#btnEstop.latched{background:#8c1b1a;animation:pulse 1s infinite alternate}
@keyframes pulse{to{background:#d8322f}}
#estopBar{display:none;padding:8px 14px;background:#5c1a19;color:#ffd9d7;
  border-bottom:1px solid #8c2b2a;font-size:13px}
#estopBar.show{display:block}
main{display:grid;grid-template-columns:1fr 1fr;gap:12px;padding:12px;
  max-width:1400px;margin:0 auto}
section{background:var(--panel);border:1px solid var(--line);border-radius:10px;
  padding:12px 14px}
section.wide{grid-column:1/-1}
h2{font-size:13px;margin:0 0 10px;color:var(--dim);font-weight:600;
  letter-spacing:.5px;text-transform:uppercase}
.row{display:flex;gap:8px;align-items:center;flex-wrap:wrap;margin:6px 0}
label{color:var(--dim);font-size:12px}
input,select,button{font:inherit;font-size:13px}
input,select{background:var(--panel2);color:var(--fg);border:1px solid var(--line);
  border-radius:6px;padding:5px 8px;min-width:70px}
button{background:var(--panel2);color:var(--fg);border:1px solid var(--line);
  border-radius:6px;padding:5px 12px;cursor:pointer}
button:hover{border-color:var(--accent)}
button.primary{background:#1c3f6e;border-color:#2f6ab5}
button.danger{background:#5c2422;border-color:#8c3b38}
button:disabled{opacity:.45;cursor:not-allowed}
table{width:100%;border-collapse:collapse;font-size:12.5px}
th,td{text-align:left;padding:5px 6px;border-bottom:1px solid var(--line)}
th{color:var(--dim);font-weight:600;font-size:11.5px;text-transform:uppercase}
tr:last-child td{border-bottom:none}
.dot{display:inline-block;width:8px;height:8px;border-radius:50%;
  margin-right:6px;background:#5a6675;vertical-align:middle}
.dot.running{background:var(--ok)}
.dot.starting{background:var(--warn)}
.dot.degraded{background:var(--warn)}
.dot.exited,.dot.failed{background:var(--bad)}
.mono{font-family:ui-monospace,Consolas,monospace;font-size:12px}
.dim{color:var(--dim)}
#camWrap{background:#000;border-radius:8px;overflow:hidden;min-height:180px;
  display:flex;align-items:center;justify-content:center}
#cam{max-width:100%;max-height:340px;display:block}
#logList{height:210px;overflow:auto;font-family:ui-monospace,Consolas,monospace;
  font-size:11.5px;background:#0b0f14;border-radius:8px;padding:8px;
  border:1px solid var(--line)}
#logList div{padding:1px 0;white-space:pre-wrap;word-break:break-all}
.lv40{color:#ff9b9b}.lv30{color:#ffcf7a}.lv20{color:#9fd4a8}.lv10{color:#7d8b9c}
#chatLog{height:150px;overflow:auto;background:#0b0f14;border-radius:8px;
  padding:8px;border:1px solid var(--line);font-size:12.5px}
#chatLog p{margin:2px 0}
#chatLog .me{color:#8fc7ff}
#toast{position:fixed;right:14px;bottom:14px;background:var(--panel2);
  border:1px solid var(--line);border-radius:8px;padding:9px 13px;
  font-size:13px;opacity:0;transition:opacity .25s;pointer-events:none;z-index:99}
#toast.show{opacity:1}
@media(max-width:820px){main{grid-template-columns:1fr}}
</style>
</head>
<body>
<header>
  <h1>fishbot 控制台</h1>
  <span class="badge" id="bState">--</span>
  <span class="badge" id="bBatt">--</span>
  <span class="badge" id="bPose">--</span>
  <span class="badge" id="bCam">--</span>
  <span class="badge" id="bErr">--</span>
  <span class="badge" id="bConn">连接中</span>
  <button id="btnEstop" title="按一下立即停（Esc 解除）">停 急 停</button>
</header>
<div id="estopBar">急停已按下：正在以 20 Hz 持续发零速，导航与巡检已取消。按 <b>Esc</b> 或再点一次按钮解除。</div>

<main>
  <section class="wide">
    <h2>一键启停</h2>
    <table>
      <thead><tr><th>业务</th><th>状态</th><th>节点</th><th>说明</th><th></th></tr></thead>
      <tbody id="stackRows"><tr><td colspan="5" class="dim">读取中…</td></tr></tbody>
    </table>
    <div class="row" style="margin-top:8px">
      <span class="dim" style="font-size:12px">导航与建图互斥；点「导航」会自动拉起 RViz，地图交互请在 RViz 里做。</span>
    </div>
  </section>

  <section>
    <h2>摄像头</h2>
    <div id="camWrap"><img id="cam" alt="等待画面"></div>
    <div class="row">
      <select id="camKind">
        <option value="anno">检测标注图</option>
        <option value="raw">原始图</option>
      </select>
      <button id="btnCapture">抓拍存盘</button>
      <span class="dim" id="camHint">没有图像：视觉节点没起？</span>
    </div>
  </section>

  <section>
    <h2>巡检</h2>
    <div class="row">
      <label>路线</label><input id="patrolRoute" list="routeList" placeholder="路线名（空=默认）">
      <datalist id="routeList"></datalist>
      <label>模式</label>
      <select id="patrolMode">
        <option value="0">单次</option><option value="1">重复</option><option value="2">随机</option>
      </select>
      <label>轮数</label><input id="patrolRepeat" type="number" value="1" min="1" style="width:64px">
    </div>
    <div class="row">
      <button id="btnPatrol" class="primary">开始巡检</button>
      <button id="btnPatrolCancel" class="danger">停止</button>
      <span class="dim" id="patrolHint"></span>
    </div>
    <h2 style="margin-top:14px">录点</h2>
    <div class="row">
      <label>路线</label><input id="recRoute" list="routeList" placeholder="路线名（空=默认）">
      <label>点 ID</label><input id="recName" placeholder="留空自动编号" style="width:110px">
    </div>
    <div class="row">
      <label>期望</label>
      <select id="recExpect">
        <option value="">不标</option>
        <option value="present">应该有灭火器</option>
        <option value="absent">应该没有</option>
      </select>
      <label>等待</label><input id="recWait" type="number" value="0" step="0.5" style="width:64px">秒
      <button id="btnRecord">记下当前位姿</button>
    </div>
    <div class="row"><span class="dim" style="font-size:12px" id="recHint">用当前位置记一个点；点位在 RViz 里显示，删点也在 RViz 做。</span></div>
  </section>

  <section class="wide">
    <h2>报告</h2>
    <div class="row">
      <label>运行</label><select id="runSel" style="min-width:320px"></select>
      <button id="btnReload">刷新</button>
      <span class="dim" id="runHint"></span>
    </div>
    <table>
      <thead><tr><th>时间</th><th>点</th><th>结果</th><th>任务</th><th>轮/序号</th><th>备注</th></tr></thead>
      <tbody id="eventBody"><tr><td colspan="6" class="dim">选一次运行看逐点事件</td></tr></tbody>
    </table>
  </section>

  <section>
    <h2>日志</h2>
    <div class="row">
      <select id="logLevel">
        <option value="">全部</option><option value="30">仅 WARN 以上</option><option value="40">仅 ERROR</option>
      </select>
      <span class="dim" id="logCounts"></span>
    </div>
    <div id="logList"></div>
  </section>

  <section>
    <h2>AI 助手</h2>
    <div id="chatLog"><p class="dim">可以问「现在什么状态」「去 (3,2)」「开始巡检 main」。</p></div>
    <div class="row">
      <input id="askBox" placeholder="输入指令…" style="flex:1;min-width:120px">
      <button id="btnAsk" class="primary">发送</button>
    </div>
    <div class="row"><span class="dim" style="font-size:12px">AI 的动作要过白名单，不能直接改速度、不能跑任意命令。</span></div>
  </section>
</main>
<div id="toast"></div>

<script>
/* ---------- 小工具 ---------- */
const $ = id => document.getElementById(id);
let toastTimer = null;
function toast(text){
  const box = $('toast');
  box.textContent = text;
  box.classList.add('show');
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => box.classList.remove('show'), 2600);
}
function reportError(text){
  /* 浏览器里的异常打回服务端日志：页面报错在页面上是看不见的 */
  try{
    fetch('/api/client_log', {
      method:'POST', headers:{'Content-Type':'application/json'},
      body: JSON.stringify({text: String(text).slice(0, 480), level:'ERROR'})
    });
  }catch(e){/* 再失败也不能自己抛 */}
}
window.addEventListener('error', ev => reportError(
  (ev.message || 'error') + ' @ ' + (ev.filename || '?') + ':' + (ev.lineno || 0)));
window.addEventListener('unhandledrejection', ev => reportError(
  'unhandled rejection: ' + (ev.reason && ev.reason.message || ev.reason)));

async function api(path, opts){
  const res = await fetch(path, opts);
  if (!res.ok) throw new Error(path + ' -> HTTP ' + res.status);
  return res.json();
}
async function post(path, body){
  return api(path, {method:'POST', headers:{'Content-Type':'application/json'},
                    body: JSON.stringify(body || {})});
}
function badge(el, text, kind){
  el.textContent = text;
  el.className = 'badge' + (kind ? ' ' + kind : '');
}
function fmt(v, digits){
  if (v === null || v === undefined || v === '' || isNaN(Number(v))) return '--';
  return Number(v).toFixed(digits === undefined ? 2 : digits);
}

/* ---------- 状态 ---------- */
let lastState = null;
async function pollState(){
  let data;
  try{
    data = await api('/api/state');
    badge($('bConn'), '实时 已连', 'ok');
  }catch(err){
    badge($('bConn'), '连接断开', 'bad');
    return;
  }
  lastState = data;
  const st = data.status || {};
  badge($('bState'), st.state || 'IDLE',
        st.state === 'IDLE' ? '' : 'ok');
  const band = st.battery_band;
  const bv = st.battery_v;
  badge($('bBatt'), '电量 ' + (bv === null || bv === undefined ? '--' : fmt(bv, 2) + 'V'),
        band === 'crit' ? 'bad' : (band === 'warn' ? 'warn' : (band === 'ok' ? 'ok' : '')));
  const pose = st.pose || data.pose || {};
  if (pose.age_s !== undefined && pose.age_s !== null){
    badge($('bPose'), '位姿 ' + fmt(pose.x) + ',' + fmt(pose.y), pose.age_s > 2 ? 'warn' : 'ok');
  }else{
    badge($('bPose'), '位姿 无', 'warn');
  }

  /* 子系统：相机、错误数 */
  const subs = data.subsystems || [];
  const camSub = subs.find(s => s.key === 'camera');
  badge($('bCam'), '相机' + (camSub && camSub.ok ? ' 有' : ' 无'),
        camSub && camSub.ok ? 'ok' : 'warn');
  const counts = data.log_counts || {};
  const errs = counts['40'] || counts['ERROR'] || 0;
  badge($('bErr'), '错误 ' + errs, errs > 0 ? 'bad' : '');

  /* 急停 */
  const estop = st.estop || {};
  $('btnEstop').classList.toggle('latched', !!estop.active);
  $('estopBar').classList.toggle('show', !!estop.active);

  if (st.running){
    $('patrolHint').textContent = '运行中：' + (st.route_name || '') +
      ' 第 ' + (st.current_loop || 0) + ' 轮，已完成 ' + (st.completed_points || 0) + ' 点';
  }
}

/* ---------- 一键启停 ---------- */
const STATE_TEXT = {running:'运行中', starting:'启动中', degraded:'半启动',
                    exited:'已退出', stopped:'未启动', failed:'失败'};
async function pollStacks(){
  let data;
  try{ data = await api('/api/stacks'); }catch(err){ return; }
  const items = data.items || {};
  const names = Object.keys(items);
  const body = $('stackRows');
  if (!names.length){
    body.innerHTML = '<tr><td colspan="5" class="dim">后端没返回任何业务栈</td></tr>';
    return;
  }
  body.innerHTML = '';
  names.forEach(name => {
    const it = items[name];
    const state = it.state || 'stopped';
    const probes = it.probes || {};
    const expect = (probes.expect || []).length;
    const found = (probes.found || []).length;
    const tr = document.createElement('tr');
    tr.innerHTML =
      '<td>' + (it.label || name) + '</td>' +
      '<td><span class="dot ' + state + '"></span>' + (STATE_TEXT[state] || state) +
        (it.managed ? '' : (state === 'running' ? ' <span class="dim">(外部)</span>' : '')) + '</td>' +
      '<td class="mono dim">' + found + '/' + expect + '</td>' +
      '<td class="dim" style="max-width:420px">' + (it.description || '') +
        (it.last_error ? '<br><span style="color:#e05c5c">' + it.last_error + '</span>' : '') + '</td>' +
      '<td style="white-space:nowrap"></td>';
    const cell = tr.lastElementChild;
    const startBtn = document.createElement('button');
    startBtn.textContent = '开始';
    startBtn.className = 'primary';
    startBtn.disabled = (state === 'running' || state === 'starting');
    startBtn.onclick = () => stackAction('start', name, startBtn);
    const stopBtn = document.createElement('button');
    stopBtn.textContent = '停止';
    stopBtn.className = 'danger';
    stopBtn.style.marginLeft = '6px';
    stopBtn.disabled = (state === 'stopped' && !it.managed);
    stopBtn.onclick = () => stackAction('stop', name, stopBtn);
    cell.appendChild(startBtn);
    cell.appendChild(stopBtn);
    body.appendChild(tr);
  });
}
async function stackAction(action, name, btn){
  btn.disabled = true;
  try{
    const out = await post('/api/stacks/' + action, {name: name});
    toast(out.message || (out.ok ? '已提交' : '失败'));
  }catch(err){
    toast('请求失败：' + err.message);
  }finally{
    btn.disabled = false;
    pollStacks();
  }
}

/* ---------- 急停 ---------- */
async function estop(on){
  try{
    const out = await post('/api/estop', {active: on});
    toast(out.message || (on ? '已急停' : '已解除'));
  }catch(err){ toast('急停请求失败：' + err.message); }
  pollState();
}
$('btnEstop').onclick = () => {
  const active = lastState && lastState.status && lastState.status.estop
                 && lastState.status.estop.active;
  estop(!active);
};
document.addEventListener('keydown', ev => {
  if (ev.key === 'Escape'){
    const active = lastState && lastState.status && lastState.status.estop
                   && lastState.status.estop.active;
    if (active) estop(false);
  }
});

/* ---------- 摄像头 ---------- */
let camOn = true;
function tickCam(){
  if (!camOn) return;
  const kind = $('camKind').value;
  const img = $('cam');
  img.src = '/api/frame.jpg?kind=' + kind + '&t=' + Date.now();
}
$('cam').onerror = () => {
  $('camHint').textContent = '没有图像：视觉节点没起，或两台机器不在同一个 DOMAIN_ID';
  $('cam').style.visibility = 'hidden';
};
$('cam').onload = () => {
  $('camHint').textContent = '';
  $('cam').style.visibility = 'visible';
};
$('btnCapture').onclick = async () => {
  try{
    const out = await post('/api/capture', {});
    toast(out.message || (out.ok ? '已存盘' : '抓拍失败'));
  }catch(err){ toast('抓拍失败：' + err.message); }
};

/* ---------- 巡检 ---------- */
$('btnPatrol').onclick = async () => {
  const body = {route: $('patrolRoute').value.trim(),
                mode: parseInt($('patrolMode').value, 10) || 0,
                repeat: parseInt($('patrolRepeat').value, 10) || 1};
  try{
    const out = await post('/api/patrol', body);
    toast(out.message || (out.ok ? '已下发' : '下发失败'));
    if (!out.ok) $('patrolHint').textContent = out.message || '';
  }catch(err){ toast('下发失败：' + err.message); }
};
$('btnPatrolCancel').onclick = async () => {
  try{
    const out = await post('/api/patrol/cancel', {});
    toast(out.message || '已取消');
  }catch(err){ toast('取消失败：' + err.message); }
};

/* ---------- 录点 ---------- */
$('btnRecord').onclick = async () => {
  const body = {route: $('recRoute').value.trim() || $('patrolRoute').value.trim(),
                point_id: $('recName').value.trim(),
                expect: $('recExpect').value,
                wait: parseFloat($('recWait').value) || 0,
                task_id: 'detect'};
  try{
    const out = await post('/api/record_point', body);
    toast(out.message || (out.ok ? '已记录' : '记录失败'));
    $('recHint').textContent = out.ok
      ? ('已记下 ' + ((out.point && out.point.point_id) || '') +
         '，去 RViz 里能看到这个点')
      : (out.message || '记录失败');
    if (out.ok) $('recName').value = '';
  }catch(err){ toast('记录失败：' + err.message); }
};

/* ---------- 报告 ---------- */
let runsCache = [];
function renderRuns(runs){
  const sel = $('runSel');
  const keep = sel.value;
  sel.innerHTML = '';
  if (!runs.length){
    sel.innerHTML = '<option value="">（还没有历史运行）</option>';
    $('eventBody').innerHTML =
      '<tr><td colspan="6" class="dim">选一次运行看逐点事件</td></tr>';
    return;
  }
  runs.forEach(r => {
    const opt = document.createElement('option');
    opt.value = r.name;
    const when = (r.started_at || '').replace('T', ' ').slice(0, 19);
    opt.textContent = when + ' · ' + (r.route_name || '(未记录路线)') +
      ' · ' + (r.ok || 0) + '/' + (r.total || 0) +
      (r.duration_sec ? ' · ' + fmt(r.duration_sec, 0) + 's' : '');
    sel.appendChild(opt);
  });
  if (keep && runs.some(r => r.name === keep)) sel.value = keep;
  const cur = runs.find(r => r.name === sel.value) || runs[0];
  renderRunSummary(cur);
  loadEvents(cur.name);
}
function renderRunSummary(run){
  if (!run) return;
  const parts = [];
  if (run.duration_sec) parts.push('耗时 ' + fmt(run.duration_sec, 0) + 's');
  parts.push('成功 ' + (run.ok || 0) + '/' + (run.total || 0));
  if (run.skipped) parts.push('跳过 ' + run.skipped);
  if (run.failed) parts.push('失败 ' + run.failed);
  if (run.graded_points) parts.push('召回 ' + fmt(run.recall, 2) +
    ' / 精确 ' + fmt(run.precision, 2));
  $('runHint').textContent = parts.join(' · ');
}
async function loadEvents(name){
  if (!name) return;
  let data;
  try{ data = await api('/api/events?run=' + encodeURIComponent(name)); }
  catch(err){ return; }
  const items = data.items || [];
  const body = $('eventBody');
  if (!items.length){
    body.innerHTML = '<tr><td colspan="6" class="dim">这次运行没有事件记录</td></tr>';
    return;
  }
  body.innerHTML = '';
  items.slice(-120).reverse().forEach(ev => {
    const tr = document.createElement('tr');
    const t = (ev.time || '').replace('T', ' ').slice(11, 19);
    const status = ev.status || ev.type || '';
    const cls = /ok|succe/i.test(status) ? 'ok'
              : (/fail|error/i.test(status) ? 'bad'
              : (/skip/i.test(status) ? 'warn' : ''));
    tr.innerHTML =
      '<td class="mono dim">' + t + '</td>' +
      '<td class="mono">' + (ev.point_id || '') + '</td>' +
      '<td><span class="badge ' + cls + '" style="border:none;padding:0">' + status + '</span></td>' +
      '<td class="mono dim">' + (ev.task_id || '') + '</td>' +
      '<td class="mono dim">L' + (ev.loop_index === undefined ? '' : ev.loop_index) +
        ' N' + (ev.route_index === undefined ? '' : ev.route_index) + '</td>' +
      '<td class="dim" style="max-width:460px">' +
        (ev.message || ev.report || ev.summary || '') + '</td>';
    body.appendChild(tr);
  });
}
async function pollRuns(){
  let data;
  try{ data = await api('/api/runs'); }catch(err){ return; }
  runsCache = data.items || [];
  renderRuns(runsCache);
  const names = new Set(runsCache.map(r => r.route_name).filter(Boolean));
  const list = $('routeList');
  const have = Array.from(list.options).map(o => o.value);
  names.forEach(n => {
    if (have.indexOf(n) < 0){
      const opt = document.createElement('option');
      opt.value = n;
      list.appendChild(opt);
    }
  });
}
$('runSel').onchange = () => {
  const run = runsCache.find(r => r.name === $('runSel').value);
  renderRunSummary(run);
  loadEvents($('runSel').value);
};
$('btnReload').onclick = () => { pollRuns(); pollLogs(); };

/* ---------- 日志 ---------- */
let seenLog = new Set();
async function pollLogs(){
  const level = $('logLevel').value;
  let data;
  try{ data = await api('/api/logs?limit=120' + (level ? '&level=' + level : '')); }
  catch(err){ return; }
  const items = data.items || [];
  const counts = data.counts || {};
  const warn = counts['30'] || counts['WARN'] || 0;
  const err = counts['40'] || counts['ERROR'] || 0;
  $('logCounts').textContent = 'WARN ' + warn + ' / ERROR ' + err;
  const box = $('logList');
  const atBottom = box.scrollTop + box.clientHeight >= box.scrollHeight - 24;
  box.innerHTML = '';
  items.forEach(item => {
    const t = (item.ts || item.time || '').toString();
    const lv = Number(item.level || 20);
    const div = document.createElement('div');
    div.className = 'lv' + lv;
    const stamp = t.length > 11 ? t.slice(11, 19) : t;
    div.textContent = stamp + ' ' + (item.source || '') + ' ' +
      (item.text || item.message || '');
    box.appendChild(div);
  });
  if (atBottom) box.scrollTop = box.scrollHeight;
}
$('logLevel').onchange = pollLogs;

/* ---------- AI ---------- */
function chat(text, mine){
  const p = document.createElement('p');
  if (mine) p.className = 'me';
  p.textContent = text;
  $('chatLog').appendChild(p);
  $('chatLog').scrollTop = $('chatLog').scrollHeight;
}
async function ask(){
  const q = $('askBox').value.trim();
  if (!q) return;
  chat('我：' + q, true);
  $('askBox').value = '';
  try{
    const out = await post('/api/ask', {q: q});
    const reply = out.reply || out.answer || out.message ||
      (out.ok === false ? '（没听懂）' : '好的');
    chat('助手：' + reply, false);
    if (out.actions) chat('已执行：' + JSON.stringify(out.actions), false);
  }catch(err){ chat('助手：请求失败 ' + err.message, false); }
}
$('btnAsk').onclick = ask;
$('askBox').addEventListener('keydown', ev => { if (ev.key === 'Enter') ask(); });

/* ---------- 启动 ---------- */
pollState(); pollStacks(); pollLogs(); pollRuns();
setInterval(pollState, 1000);
setInterval(pollStacks, 1500);
setInterval(pollLogs, 2000);
setInterval(pollRuns, 6000);
setInterval(tickCam, 900);
tickCam();
</script>
</body>
</html>
'''
