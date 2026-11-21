// Smart-Community infra workbench — entry module.
//
//   * Tab switching (views emit 'view' so modules start/stop polling).
//   * State WebSocket → dashboard + 'state' event for other modules.
//   * Launch list with an args dialog (required args, choices, last values).
//   * Live log WebSocket and the emergency stop button.

import { api, connectWithRetry, emit, escapeHtml, fmtDuration, on, toast } from './common.js';
import './nav.js?v=teleop-keyboard-1';
import './camera.js?v=plate-building-fire-1';
import './mission.js?v=inspection-auto-branch-1';

const $ = (id) => document.getElementById(id);
const views = Array.from(document.querySelectorAll('.view'));
const navButtons = Array.from(document.querySelectorAll('.nav-item'));
const LOG_MAX_LINES = 300;

// --- Tabs -------------------------------------------------------------------

function viewFromHash() {
  const name = window.location.hash.slice(1);
  if (name === 'dashboard') return 'diagnostics';
  return views.some(view => view.dataset.view === name) ? name : 'teleop';
}
function activateView(name) {
  const tab = name === 'mission-records' ? 'mission' : name;
  for (const btn of navButtons) {
    const selected = btn.dataset.view === tab;
    btn.classList.toggle('is-active', selected);
    if (selected) btn.setAttribute('aria-current', name === tab ? 'page' : 'location');
    else btn.removeAttribute('aria-current');
  }
  for (const v of views) v.classList.toggle('is-active', v.dataset.view === name);
  const diagnostics = $('diagnosticsLink');
  diagnostics.classList.toggle('is-on', name === 'diagnostics');
  if (name === 'diagnostics') diagnostics.setAttribute('aria-current', 'page');
  else diagnostics.removeAttribute('aria-current');
  document.querySelector('.main').scrollTop = 0;
  emit('view', name);
}
window.addEventListener('hashchange', () => activateView(viewFromHash()));
for (const btn of navButtons) btn.addEventListener('click', () => {
  window.location.hash = btn.dataset.view;
});

// --- Dashboard --------------------------------------------------------------

function setConn(ok, text) {
  $('connDot').classList.toggle('ok', !!ok);
  $('connDot').classList.toggle('err', !ok);
  $('connText').textContent = text;
}
function fmtPct(v) { return (v === null || v === undefined) ? '—' : Number(v).toFixed(0); }
function setBar(id, pct) { $(id).style.width = `${Math.max(0, Math.min(100, Number(pct) || 0))}%`; }

// Topics expected on the real robot; others seen by the bridge are appended.
const KEY_TOPICS = ['/odom_raw', '/odom', '/wit/imu', '/scan', '/amcl_pose', '/usb_cam/image_raw',
  '/map', '/navigation_map', '/move_base/GlobalPlanner/plan', '/move_base/local_plan', '/mission/state'];

function renderTopics(topics) {
  const names = KEY_TOPICS.concat(Object.keys(topics).filter((t) => !KEY_TOPICS.includes(t)));
  $('topicGrid').innerHTML = names.map((name) => {
    const t = topics[name];
    // Latched/slow topics (maps, plans, mission state) are fine once received.
    const slowOk = /map|plan|mission/.test(name);
    let cls = 'err'; let text = '无数据';
    if (t) {
      const fresh = t.age_s < 2 || slowOk;
      cls = fresh ? 'ok' : 'warn';
      text = t.age_s < 2 ? `${t.hz.toFixed(1)} Hz` : `${t.age_s.toFixed(0)} s 前`;
    }
    return `<div class="topic"><span class="dot ${cls}"></span><code>${escapeHtml(name)}</code><span class="muted">${text}</span></div>`;
  }).join('');
}

function renderDashboard(snap) {
  $('lastUpdate').textContent = snap.ts ? new Date(snap.ts * 1000).toLocaleTimeString() : '—';
  const ros = snap.ros || {};
  $('rosMaster').textContent = ros.master ? '在线' : '离线';
  $('rosNodes').textContent = `节点: ${(ros.nodes || []).length}`;
  const host = snap.host || {};
  $('hostCpu').textContent = fmtPct(host.cpu);
  $('hostMem').textContent = fmtPct(host.mem);
  setBar('hostCpuBar', host.cpu); setBar('hostMemBar', host.mem);
  const p = snap.pose;
  $('poseText').textContent = p ? `${p.x.toFixed(2)}, ${p.y.toFixed(2)}, ${(p.yaw * 180 / Math.PI).toFixed(0)}°` : '—';
  $('poseSource').textContent = `来源: ${p ? (p.source === 'tf' ? 'TF map→base' : '/amcl_pose') : '—'}`;
  lastRosSnapshot = snap;
  if (viewFromHash() === 'diagnostics') renderDiagnostics();
  renderLaunches(snap.launches || []);
  emit('state', snap);
}

let lastRosSnapshot = null;
function renderDiagnostics() {
  if (!lastRosSnapshot) return;
  const names = (lastRosSnapshot.ros?.nodes || []).map(node => {
    const item = document.createElement('code');
    item.textContent = node;
    return item;
  });
  $('diagnosticNodeCount').textContent = names.length + ' 个';
  if (names.length) $('diagnosticNodes').replaceChildren(...names);
  else $('diagnosticNodes').textContent = '没有可用节点';
  renderTopics(lastRosSnapshot.topics || {});
}
on('view', name => { if (name === 'diagnostics') renderDiagnostics(); });

// --- Launches ---------------------------------------------------------------

let launchRegistry = [];
let lastRecords = [];

async function fetchLaunchRegistry() {
  try { launchRegistry = (await api('/api/launches')).launches || []; }
  catch (err) { console.warn('failed to load launch registry', err); }
  renderLaunches(lastRecords);
}

function renderStatusBadge(status) {
  const map = { running: ['ok', '运行中'], exited: ['', '已退出'], failed: ['err', '异常'], idle: ['', '未启动'] };
  const [cls, text] = map[status] || ['', status];
  return `<span class="badge ${cls}">${text}</span>`;
}

let launchSignature = '';
let raceStarting = false;
let writable = false;

function updateRaceButton() {
  const running = lastRecords.some((r) => r.name === 'finals' && r.status === 'running');
  $('raceStart').disabled = !writable || raceStarting || running;
  $('raceStart').textContent = raceStarting ? '比赛启动中…' : running ? '比赛运行中' : '比赛一键启动';
}

$('raceStart').addEventListener('click', async () => {
  raceStarting = true;
  updateRaceButton();
  try {
    await api('/api/launch/finals/start', { method: 'POST', body: { base_protocol: 'aa_bb' } });
    toast('已启动决赛流程', 'ok');
  } catch (err) {
    toast(`比赛启动失败：${err.message}`, 'err');
  } finally {
    raceStarting = false;
    updateRaceButton();
  }
});

function renderLaunches(records) {
  lastRecords = records;
  updateRaceButton();
  const byName = new Map(records.map((r) => [r.name, r]));
  $('launchHint').textContent = `${launchRegistry.length} 项`;
  const tbody = $('launchTbody');
  // Rebuild rows only when something other than the timers changed, so a
  // click is never lost to a re-render between mousedown and mouseup.
  const signature = JSON.stringify([launchRegistry.map((e) => e.name),
    records.map((r) => [r.name, r.status, r.pid, r.exit_code, r.args])]);
  if (signature === launchSignature) {
    for (const tr of tbody.children) {
      const r = byName.get(tr.dataset.name);
      tr.querySelector('.since').textContent = fmtDuration(r && r.since_s != null ? r.since_s : null);
    }
    return;
  }
  launchSignature = signature;
  tbody.innerHTML = '';
  for (const entry of launchRegistry) {
    const r = byName.get(entry.name);
    const status = r && r.status ? r.status : 'idle';
    const running = status === 'running';
    const args = r && r.args ? Object.entries(r.args).filter(([, v]) => v !== '').map(([k, v]) => `${k}=${v}`).join(' ') : '';
    const tr = document.createElement('tr');
    tr.dataset.name = entry.name;
    tr.innerHTML = `
      <td><b>${escapeHtml(entry.name)}</b></td>
      <td class="muted">${escapeHtml(entry.description || '')}${args ? `<div class="args-line">${escapeHtml(args)}</div>` : ''}</td>
      <td>${renderStatusBadge(status)}${r && !running && r.exit_code != null ? ` <span class="muted">code ${r.exit_code}</span>` : ''}</td>
      <td class="muted">${r && r.pid ? r.pid : '—'}</td>
      <td class="muted since">${fmtDuration(r && r.since_s != null ? r.since_s : null)}</td>
      <td class="row-actions">
        <button class="btn btn-primary" data-act="start" ${running ? 'disabled' : ''}>启动</button>
        <button class="btn" data-act="stop" ${running ? '' : 'disabled'}>停止</button>
        <button class="btn btn-ghost" data-act="log" ${r ? '' : 'disabled'}>日志</button>
      </td>`;
    tr.querySelector('[data-act=start]').addEventListener('click', () => openArgsDialog(entry));
    tr.querySelector('[data-act=stop]').addEventListener('click', () => stopLaunch(entry.name));
    tr.querySelector('[data-act=log]').addEventListener('click', () => window.open(`/api/logs/${entry.name}/download`, '_blank'));
    tbody.appendChild(tr);
  }
}

function savedArgs(name) {
  try { return JSON.parse(localStorage.getItem(`launchArgs:${name}`) || '{}'); } catch { return {}; }
}

function openArgsDialog(entry) {
  const saved = savedArgs(entry.name);
  $('argsTitle').textContent = `启动 ${entry.name}`;
  $('argsDesc').textContent = `${entry.description}（${entry.package}/${entry.launch_file}）`;
  $('argsError').textContent = '';
  const fields = $('argsFields');
  fields.innerHTML = '';
  for (const [key, def] of Object.entries(entry.args)) {
    const value = saved[key] !== undefined ? saved[key] : def;
    const required = entry.required.includes(key);
    const choices = entry.choices[key];
    const label = document.createElement('label');
    label.className = 'field';
    label.innerHTML = `<span>${escapeHtml(key)}${required ? ' <b class="req">*必填</b>' : ''}</span>`;
    let input;
    if (choices) {
      input = document.createElement('select');
      const opts = (required ? [''] : []).concat(choices);
      input.innerHTML = opts.map((c) => `<option value="${escapeHtml(c)}">${c === '' ? '— 请选择 —' : escapeHtml(c)}</option>`).join('');
    } else {
      input = document.createElement('input');
      input.placeholder = def === '' ? '留空使用 launch 默认值' : '';
    }
    input.className = 'input';
    input.name = key;
    input.value = value;
    label.appendChild(input);
    fields.appendChild(label);
  }
  $('argsModal').hidden = false;
  $('argsForm').onsubmit = async (ev) => {
    ev.preventDefault();
    const body = {};
    for (const el of fields.querySelectorAll('[name]')) body[el.name] = el.value.trim();
    const missing = entry.required.filter((k) => !body[k]);
    if (missing.length) { $('argsError').textContent = `请填写：${missing.join(', ')}`; return; }
    try {
      await api(`/api/launch/${entry.name}/start`, { method: 'POST', body });
      localStorage.setItem(`launchArgs:${entry.name}`, JSON.stringify(body));
      $('argsModal').hidden = true;
      toast(`已启动 ${entry.name}`, 'ok');
    } catch (err) {
      $('argsError').textContent = err.message;
    }
  };
}
$('argsCancel').addEventListener('click', () => { $('argsModal').hidden = true; });

async function stopLaunch(name) {
  toast(`正在停止 ${name}…`);
  try { await api(`/api/launch/${name}/stop`, { method: 'POST' }); toast(`已停止 ${name}`, 'ok'); }
  catch (err) { toast(`停止 ${name} 失败：${err.message}`, 'err'); }
}

// --- Emergency stop (no confirmation: it must act immediately) --------------

$('estopBtn').addEventListener('click', async () => {
  emit('estop');
  try {
    const res = await api('/api/estop', { method: 'POST' });
    toast(`急停：${res.steps.join(' → ')}`, 'err');
  } catch (err) {
    toast(`急停失败：${err.message}`, 'err');
  }
});

// --- Logs -------------------------------------------------------------------

function appendLog(msg) {
  const logbox = $('logbox');
  if (logbox.dataset.started !== '1') { logbox.textContent = ''; logbox.dataset.started = '1'; }
  const stick = logbox.scrollTop + logbox.clientHeight >= logbox.scrollHeight - 20;
  const line = document.createElement('div');
  line.innerHTML = `<span class="log-name">[${escapeHtml(msg.name)}]</span>${escapeHtml(msg.line || '')}`;
  logbox.appendChild(line);
  while (logbox.childNodes.length > LOG_MAX_LINES) logbox.removeChild(logbox.firstChild);
  if (stick) logbox.scrollTop = logbox.scrollHeight;
}

// --- Bootstrap --------------------------------------------------------------

async function fetchHealth() {
  try {
    const data = await api('/api/health');
    $('robotIp').textContent = data.robot_ip || '—';
    writable = !data.readonly;
    updateRaceButton();
  } catch { return; }
}

on('launch-changed', fetchLaunchRegistry);

async function main() {
  await Promise.all([fetchHealth(), fetchLaunchRegistry()]);
  connectWithRetry('/api/state', renderDashboard, {
    onOpen: () => setConn(true, '实时连接'),
    onClose: (ev) => setConn(false, ev.code === 4401 ? 'Token 无效' : '断线重连…'),
  });
  connectWithRetry('/api/logs', appendLog);
  setInterval(fetchLaunchRegistry, 30000);
  activateView(viewFromHash());
}

main().catch((err) => console.error(err));
