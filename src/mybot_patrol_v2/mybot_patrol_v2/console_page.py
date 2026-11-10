#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""网页控制台的页面（单个字符串，不用模板引擎、不引 CDN）。

为什么要内嵌：ament_python 装静态资源要走 data_files / package_data，
路径在各种 install 布局下容易出岔子（symlink-install / --merge-install）。
一个常量字符串没有这问题，代价是改样式得改 Python 文件。
"""

PAGE_HTML = r"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>消防巡检控制台</title>
<style>
  :root{
    --bg:#0f1420; --panel:#18202f; --line:#26324a; --text:#e6ecf5;
    --dim:#8fa0bb; --ok:#3ecf8e; --warn:#e8b83a; --crit:#ef5350;
    --accent:#4c8dff;
  }
  *{box-sizing:border-box}
  body{margin:0;background:var(--bg);color:var(--text);
    font:14px/1.5 -apple-system,"Segoe UI","Noto Sans CJK SC",sans-serif}
  header{display:flex;align-items:center;gap:14px;padding:12px 18px;
    background:var(--panel);border-bottom:1px solid var(--line);flex-wrap:wrap}
  header h1{font-size:16px;margin:0;font-weight:600}
  .pill{display:inline-flex;align-items:center;gap:6px;padding:3px 10px;
    border-radius:999px;background:#111826;border:1px solid var(--line);
    font-size:12px;color:var(--dim)}
  .dot{width:8px;height:8px;border-radius:50%;background:#555}
  .dot.on{background:var(--ok);box-shadow:0 0 8px var(--ok)}
  .dot.off{background:var(--crit)}
  main{display:grid;grid-template-columns:340px 1fr;gap:14px;padding:14px 18px 40px}
  @media(max-width:900px){main{grid-template-columns:1fr}}
  .card{background:var(--panel);border:1px solid var(--line);border-radius:10px;
    padding:14px}
  .card h2{margin:0 0 10px;font-size:13px;color:var(--dim);font-weight:600;
    letter-spacing:.06em;text-transform:uppercase}
  label{display:block;font-size:12px;color:var(--dim);margin:8px 0 4px}
  input,select{width:100%;padding:7px 9px;background:#111826;color:var(--text);
    border:1px solid var(--line);border-radius:7px;font:inherit}
  .row{display:flex;gap:8px}
  .row>*{flex:1}
  button{padding:9px 12px;border-radius:8px;border:1px solid var(--line);
    background:#1d2738;color:var(--text);font:inherit;cursor:pointer}
  button:hover{border-color:var(--accent)}
  button.primary{background:var(--accent);border-color:var(--accent);color:#fff;
    font-weight:600}
  button.danger{background:#3a1f24;border-color:#5c2a31;color:#ffb4b4}
  button:disabled{opacity:.45;cursor:not-allowed}
  .btn-row{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-top:10px}
  .btn-row .wide{grid-column:1 / -1}
  .state{font-size:22px;font-weight:700;letter-spacing:.02em}
  .kv{display:flex;justify-content:space-between;gap:10px;padding:5px 0;
    border-bottom:1px dashed #202b40;font-size:13px}
  .kv span:first-child{color:var(--dim)}
  .kv:last-child{border-bottom:none}
  .bar{height:7px;background:#111826;border-radius:4px;overflow:hidden;margin:8px 0}
  .bar>i{display:block;height:100%;background:var(--accent);width:0;transition:width .3s}
  .big-num{font-variant-numeric:tabular-nums}
  #events{font-family:ui-monospace,Consolas,monospace;font-size:12px;
    max-height:230px;overflow:auto;background:#0c1220;border-radius:8px;padding:8px}
  #events div{padding:2px 0;border-bottom:1px solid #141d2c;color:#b9c8dd;
    white-space:pre-wrap;word-break:break-all}
  table{width:100%;border-collapse:collapse;font-size:13px}
  th,td{padding:7px 8px;text-align:left;border-bottom:1px solid #1f2a3d;
    white-space:nowrap}
  th{color:var(--dim);font-weight:600;font-size:12px}
  tbody tr:hover{background:#1b2536;cursor:pointer}
  .b-ok{color:var(--ok)} .b-warn{color:var(--warn)} .b-crit{color:var(--crit)}
  .muted{color:var(--dim)}
  .toast{position:fixed;right:18px;bottom:18px;background:#1d2738;border:1px solid var(--line);
    border-left:3px solid var(--accent);padding:10px 14px;border-radius:8px;
    max-width:420px;opacity:0;transform:translateY(8px);transition:.2s;pointer-events:none}
  .toast.show{opacity:1;transform:none}
  .toast.err{border-left-color:var(--crit)}
</style>
</head>
<body>
<header>
  <h1>消防巡检控制台</h1>
  <span class="pill"><i id="linkDot" class="dot"></i><span id="linkText">连接中…</span></span>
  <span class="pill">状态 <b id="hdrState">—</b></span>
  <span class="pill">电量 <b id="hdrBatt">—</b></span>
  <span class="pill">路线 <b id="hdrRoute">—</b></span>
</header>

<main>
  <section>
    <div class="card">
      <h2>执行控制</h2>
      <label>路线</label>
      <input id="route" value="main" placeholder="main">
      <div class="row">
        <div>
          <label>模式</label>
          <select id="mode">
            <option value="0">once（一次）</option>
            <option value="1">repeat（重复）</option>
            <option value="2">random（随机）</option>
          </select>
        </div>
        <div>
          <label>轮次</label>
          <input id="repeat" type="number" min="1" value="1">
        </div>
      </div>
      <div class="btn-row">
        <button class="primary wide" id="btnStart">开始巡检</button>
        <button id="btnPause">暂停</button>
        <button id="btnResume">恢复</button>
        <button class="danger wide" id="btnClear">清空当前点位（抢占）</button>
      </div>
    </div>

    <div class="card" style="margin-top:14px">
      <h2>存点位</h2>
      <label>point_id（留空自动编号）</label>
      <input id="pid" placeholder="p1">
      <div class="row">
        <div>
          <label>task_id</label>
          <select id="tid">
            <option>inspect</option><option>detect</option>
            <option>vision</option><option>yolo</option>
          </select>
        </div>
        <div>
          <label>expect</label>
          <select id="expect">
            <option value="">不判定</option>
            <option value="present">present</option>
            <option value="absent">absent</option>
          </select>
        </div>
      </div>
      <div class="btn-row">
        <button class="wide" id="btnRecord">记录当前位置</button>
      </div>
      <p class="muted" style="font-size:12px;margin:8px 0 0">
        需要 amcl 已收敛；记录的是车<b>此刻</b>的位姿。</p>
    </div>
  </section>

  <section>
    <div class="card">
      <h2>实时状态</h2>
      <div style="display:flex;justify-content:space-between;align-items:flex-end;gap:12px">
        <div class="state" id="stState">—</div>
        <div class="muted" id="stMsg" style="text-align:right;font-size:12px;flex:1"></div>
      </div>
      <div class="bar"><i id="stBar"></i></div>
      <div class="kv"><span>当前点位</span><span id="stPoint">—</span></div>
      <div class="kv"><span>进度</span><span id="stProgress" class="big-num">—</span></div>
      <div class="kv"><span>轮次</span><span id="stLoop" class="big-num">—</span></div>
      <div class="kv"><span>最近一次检测</span><span id="stDetect">—</span></div>
      <div class="kv"><span>状态更新</span><span id="stAge">—</span></div>
    </div>

    <div class="card" style="margin-top:14px">
      <h2>事件流（最近 80 条）</h2>
      <div id="events"><div class="muted">暂无事件</div></div>
    </div>

    <div class="card" style="margin-top:14px">
      <h2>历史运行（点一行展开事件）</h2>
      <div style="overflow:auto">
        <table>
          <thead><tr>
            <th>时间</th><th>路线</th><th>点</th><th>成功</th><th>跳过</th>
            <th>失败</th><th>检出</th><th>召回</th><th>准确</th><th>耗时</th>
          </tr></thead>
          <tbody id="runs"><tr><td colspan="10" class="muted">加载中…</td></tr></tbody>
        </table>
      </div>
      <div id="detail" style="margin-top:10px"></div>
    </div>
  </section>
</main>

<div class="toast" id="toast"></div>

<script>
const $ = (id) => document.getElementById(id);
let lastState = null;

function toast(text, isErr){
  const el = $('toast');
  el.textContent = text;
  el.className = 'toast show' + (isErr ? ' err' : '');
  clearTimeout(el._t);
  el._t = setTimeout(() => { el.className = 'toast'; }, 2600);
}

async function api(path, body){
  const opt = body ? {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify(body),
  } : {};
  const res = await fetch(path, opt);
  const data = await res.json().catch(() => ({}));
  if (!res.ok || data.ok === false){
    throw new Error(data.error || ('HTTP ' + res.status));
  }
  return data;
}

function bandClass(band){
  return band === 'crit' ? 'b-crit' : (band === 'warn' ? 'b-warn' :
    (band === 'ok' ? 'b-ok' : 'muted'));
}

function fmtDur(s){
  if (s === null || s === undefined) return '—';
  s = Math.max(0, Math.round(s));
  if (s < 60) return s + 's';
  if (s < 3600) return Math.floor(s/60) + 'm' + String(s%60).padStart(2,'0') + 's';
  return Math.floor(s/3600) + 'h' + String(Math.floor((s%3600)/60)).padStart(2,'0') + 'm';
}

function fmtRate(v){
  return (v === null || v === undefined) ? '—' : (100*v).toFixed(1) + '%';
}

function renderState(s){
  const link = $('linkDot');
  if (!s || !s.connected){
    link.className = 'dot off';
    $('linkText').textContent = '未连接执行器';
    $('stState').textContent = '—';
    return;
  }
  const age = s.age_s;
  const stale = age !== null && age > 10;
  link.className = 'dot ' + (stale ? 'off' : 'on');
  $('linkText').textContent = stale ? ('状态已 ' + fmtDur(age) + ' 前') : '已连接';
  $('hdrState').textContent = s.state || '—';
  $('hdrRoute').textContent = s.route_name || '—';
  $('stState').textContent = s.state || '—';
  $('stState').style.color = s.state === 'FAILED' ? 'var(--crit)'
      : (s.state === 'PAUSED' ? 'var(--warn)' : 'var(--text)');
  $('stMsg').textContent = s.message || '';
  $('stPoint').textContent = s.point_id || '—';
  $('stProgress').textContent = s.completed_points + ' 点'
      + (s.current_index ? ' / 第 ' + s.current_index + ' 点' : '');
  $('stLoop').textContent = s.current_loop ? ('第 ' + s.current_loop + ' 轮') : '—';
  $('stAge').textContent = age === null ? '—' : (fmtDur(age) + ' 前');
  const batt = s.battery_v;
  const cell = $('hdrBatt');
  cell.textContent = (batt === null || batt === undefined) ? '—'
      : (batt.toFixed(2) + ' V');
  cell.className = bandClass(s.battery_band);
  const total = Math.max(1, s.completed_points + 1);
  $('stBar').style.width = Math.min(100, 100 * s.completed_points / total) + '%';
}

async function pollState(){
  try {
    const s = await api('/api/state');
    lastState = s;
    renderState(s);
    const btn = $('btnStart');
    btn.disabled = !!(s && s.running);
    btn.textContent = (s && s.running) ? '巡检进行中…' : '开始巡检';
    if (Array.isArray(s.events)) renderEvents(s.events);
    if (s.detection) $('stDetect').textContent = s.detection;
  } catch (err){
    renderState(null);
  }
}

function renderEvents(events){
  const box = $('events');
  if (!events.length){ box.innerHTML = '<div class="muted">暂无事件</div>'; return; }
  box.innerHTML = events.slice().reverse().map(e =>
    '<div>' + (e.time ? '<span class="muted">' + e.time + '</span> ' : '') + escapeHtml(e.text) + '</div>'
  ).join('');
}

function escapeHtml(s){
  return String(s).replace(/[&<>"]/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]));
}

async function loadRuns(){
  try {
    const data = await api('/api/runs');
    const rows = data.runs || [];
    if (!rows.length){
      $('runs').innerHTML = '<tr><td colspan="10" class="muted">还没有运行记录</td></tr>';
      return;
    }
    $('runs').innerHTML = rows.map((r, i) => {
      const when = r.mtime ? new Date(r.mtime*1000).toLocaleString('zh-CN', {hour12:false}) : '—';
      const cls = r.failed > 0 ? 'b-warn' : '';
      return '<tr data-i="' + i + '">'
        + '<td>' + when + '</td>'
        + '<td>' + escapeHtml(r.route_name || '—') + '</td>'
        + '<td>' + r.total + '</td>'
        + '<td class="b-ok">' + r.ok + '</td>'
        + '<td class="muted">' + r.skipped + '</td>'
        + '<td class="' + cls + '">' + r.failed + '</td>'
        + '<td>' + r.detections_total + '</td>'
        + '<td>' + fmtRate(r.recall) + '</td>'
        + '<td>' + fmtRate(r.precision) + '</td>'
        + '<td>' + fmtDur(r.duration_sec) + '</td>'
        + '</tr>';
    }).join('');
    window._runs = rows;
    [...$('runs').querySelectorAll('tr')].forEach(tr => {
      tr.onclick = () => showDetail(rows[Number(tr.dataset.i)]);
    });
  } catch (err){
    $('runs').innerHTML = '<tr><td colspan="10" class="muted">读取失败：'
      + escapeHtml(err.message) + '</td></tr>';
  }
}

async function showDetail(run){
  try {
    const data = await api('/api/events?path=' + encodeURIComponent(run.path));
    $('detail').innerHTML = '<h2 style="font-size:12px;color:var(--dim);margin:0 0 6px">'
      + escapeHtml(run.name) + ' 的事件</h2><div id="events" style="max-height:200px">'
      + (data.events || []).map(e => '<div>' + escapeHtml(e.text) + '</div>').join('')
      + '</div>';
  } catch (err){
    toast('读取事件失败：' + err.message, true);
  }
}

$('btnStart').onclick = async () => {
  try {
    $('btnStart').disabled = true;
    await api('/api/run', {
      route: $('route').value.trim() || 'main',
      mode: Number($('mode').value),
      repeat: Number($('repeat').value) || 1,
    });
    toast('已下发巡检目标');
  } catch (err){
    $('btnStart').disabled = false;
    toast('下发失败：' + err.message, true);
  }
};
$('btnPause').onclick = async () => {
  try { await api('/api/pause', {}); toast('已暂停'); }
  catch (err){ toast('暂停失败：' + err.message, true); }
};
$('btnResume').onclick = async () => {
  try { await api('/api/resume', {}); toast('已恢复'); }
  catch (err){ toast('恢复失败：' + err.message, true); }
};
$('btnClear').onclick = async () => {
  try { await api('/api/preempt', {point_id: ''}); toast('已发送抢占（只记录不跳转）'); }
  catch (err){ toast('抢占失败：' + err.message, true); }
};
$('btnRecord').onclick = async () => {
  try {
    const data = await api('/api/record_point', {
      route: $('route').value.trim() || 'main',
      point_id: $('pid').value.trim(),
      task_id: $('tid').value,
      expect: $('expect').value,
    });
    toast(data.message || '已记录');
  } catch (err){ toast('记录失败：' + err.message, true); }
};

pollState();
loadRuns();
setInterval(pollState, 1000);
setInterval(loadRuns, 15000);
</script>
</body>
</html>
"""
