// Smart-Community infra workbench — batch-1 frontend bootstrap.
//
// Responsibilities:
//   * Tab switching between the four sidebar views.
//   * Single long-lived WebSocket to /api/state → dashboard rendering.
//   * Secondary WebSocket to /api/logs → scrolling log pane.
//   * Launch list rendering + start/stop buttons (fetch POST).
//   * Emergency-stop confirmation modal.
//
// Deliberately framework-free; everything lives in one ES module so it
// can run from `python -m http.server` during dev.

const views = Array.from(document.querySelectorAll('.view'));
const navButtons = Array.from(document.querySelectorAll('.nav-item'));
const viewTitleEl = document.getElementById('viewTitle');
const connDot = document.getElementById('connDot');
const connText = document.getElementById('connText');
const lastUpdateEl = document.getElementById('lastUpdate');
const roModeEl = document.getElementById('roMode');

const launchTbody = document.getElementById('launchTbody');
const launchHint = document.getElementById('launchHint');
const logbox = document.getElementById('logbox');

const estopBtn = document.getElementById('estopBtn');
const confirmModal = document.getElementById('confirmModal');
const confirmTitle = document.getElementById('confirmTitle');
const confirmBody = document.getElementById('confirmBody');
const confirmOk = document.getElementById('confirmOk');
const confirmCancel = document.getElementById('confirmCancel');

const LOG_MAX_LINES = 200;

// --- Tab switching ---------------------------------------------------------

function activateView(name) {
  for (const btn of navButtons) btn.classList.toggle('is-active', btn.dataset.view === name);
  for (const v of views) v.classList.toggle('is-active', v.dataset.view === name);
  const active = navButtons.find((b) => b.dataset.view === name);
  if (active) viewTitleEl.textContent = active.textContent.trim();
}

for (const btn of navButtons) {
  btn.addEventListener('click', () => activateView(btn.dataset.view));
}

// --- Confirmation modal ----------------------------------------------------

function confirmAction({ title, body }) {
  return new Promise((resolve) => {
    confirmTitle.textContent = title;
    confirmBody.textContent = body;
    confirmModal.hidden = false;
    const cleanup = () => {
      confirmModal.hidden = true;
      confirmOk.removeEventListener('click', onOk);
      confirmCancel.removeEventListener('click', onCancel);
    };
    const onOk = () => { cleanup(); resolve(true); };
    const onCancel = () => { cleanup(); resolve(false); };
    confirmOk.addEventListener('click', onOk);
    confirmCancel.addEventListener('click', onCancel);
  });
}

// --- WebSocket helpers -----------------------------------------------------

function wsUrl(path) {
  const proto = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
  return `${proto}//${window.location.host}${path}`;
}

function setConn(ok, text) {
  connDot.classList.toggle('ok', !!ok);
  connDot.classList.toggle('err', !ok);
  connText.textContent = text;
}

function connectWithRetry(path, onMessage, label) {
  let backoff = 500;
  const open = () => {
    const ws = new WebSocket(wsUrl(path));
    ws.addEventListener('open', () => {
      backoff = 500;
      if (label === 'state') setConn(true, '实时连接');
    });
    ws.addEventListener('message', (ev) => {
      try { onMessage(JSON.parse(ev.data)); }
      catch (err) { console.warn('bad ws payload', err); }
    });
    ws.addEventListener('close', () => {
      if (label === 'state') setConn(false, '断线重连…');
      setTimeout(open, backoff);
      backoff = Math.min(backoff * 2, 5000);
    });
    ws.addEventListener('error', () => ws.close());
  };
  open();
}

// --- State rendering -------------------------------------------------------

function fmtPct(v) { return (v === null || v === undefined) ? '—' : Number(v).toFixed(0); }
function fmtDuration(s) {
  if (s === null || s === undefined) return '—';
  s = Math.max(0, Math.floor(s));
  const h = Math.floor(s / 3600); s %= 3600;
  const m = Math.floor(s / 60);   s %= 60;
  if (h) return `${h}h${m}m`;
  if (m) return `${m}m${s}s`;
  return `${s}s`;
}
function setBar(id, pct) {
  const bar = document.getElementById(id);
  if (!bar) return;
  const clamped = Math.max(0, Math.min(100, Number(pct) || 0));
  bar.style.width = `${clamped}%`;
}

function renderDashboard(snap) {
  lastUpdateEl.textContent = snap.ts
    ? new Date(snap.ts * 1000).toLocaleTimeString()
    : '—';

  const ros = snap.ros || {};
  document.getElementById('rosMaster').textContent = ros.master ? '在线' : '离线';
  document.getElementById('rosNodes').textContent  = `节点: ${(ros.nodes || []).length}`;

  const host = snap.host || {};
  document.getElementById('hostCpu').textContent  = fmtPct(host.cpu);
  document.getElementById('hostMem').textContent  = fmtPct(host.mem);
  document.getElementById('hostDisk').textContent = fmtPct(host.disk);
  setBar('hostCpuBar',  host.cpu);
  setBar('hostMemBar',  host.mem);
  setBar('hostDiskBar', host.disk);

  renderLaunches(snap.launches || []);
}

function renderLaunches(records) {
  // Merge snapshot records (keyed by name) with registry entries loaded
  // from /api/launches so we always show every known launch, even those
  // that have never been started.
  const byName = new Map(records.map((r) => [r.name, r]));
  const rows = (launchRegistry || []).map((entry) => {
    const r = byName.get(entry.name);
    const status = r && r.status ? r.status : 'idle';
    const since = r && r.since_s != null ? r.since_s : null;
    return { entry, r, status, since };
  });

  launchHint.textContent = `${rows.length} 项`;
  launchTbody.innerHTML = '';
  for (const row of rows) {
    const tr = document.createElement('tr');
    tr.innerHTML = `
      <td><b>${row.entry.name}</b></td>
      <td class="muted">${row.entry.description || ''}</td>
      <td>${renderStatusBadge(row.status)}</td>
      <td class="muted">${row.r && row.r.pid ? row.r.pid : '—'}</td>
      <td class="muted">${fmtDuration(row.since)}</td>
      <td class="row-actions">
        <button class="btn btn-primary" data-act="start">启动</button>
        <button class="btn" data-act="stop">停止</button>
        <button class="btn btn-ghost" data-act="log">日志</button>
      </td>`;
    const [startBtn, stopBtn, logBtn] = tr.querySelectorAll('button');
    const running = row.status === 'running';
    startBtn.disabled = running;
    stopBtn.disabled = !running;
    startBtn.addEventListener('click', () => startLaunch(row.entry));
    stopBtn.addEventListener('click',  () => stopLaunch(row.entry));
    logBtn.addEventListener('click',   () => window.open(`/api/logs/${row.entry.name}`, '_blank'));
    launchTbody.appendChild(tr);
  }
}

function renderStatusBadge(status) {
  const map = {
    running: ['ok', '运行中'],
    exited:  ['', '已退出'],
    failed:  ['err', '异常'],
    idle:    ['', '未启动'],
  };
  const [cls, text] = map[status] || ['', status];
  return `<span class="badge ${cls}">${text}</span>`;
}

// --- Launch registry -------------------------------------------------------

let launchRegistry = [];

async function fetchLaunchRegistry() {
  try {
    const r = await fetch('/api/launches');
    const data = await r.json();
    launchRegistry = data.launches || [];
  } catch (err) {
    console.warn('failed to load launch registry', err);
  }
}

async function startLaunch(entry) {
  const body = {};
  // For batch 1 we start with registry defaults; a proper args editor
  // modal lands with the Mission tab (batch 4).
  const r = await fetch(`/api/launch/${entry.name}/start`, {
    method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body),
  });
  if (!r.ok) {
    const detail = await r.json().catch(() => ({}));
    alert(`启动 ${entry.name} 失败：${JSON.stringify(detail.detail || detail)}`);
  }
}

async function stopLaunch(entry) {
  const r = await fetch(`/api/launch/${entry.name}/stop`, { method: 'POST' });
  if (!r.ok) {
    const detail = await r.json().catch(() => ({}));
    alert(`停止 ${entry.name} 失败：${JSON.stringify(detail.detail || detail)}`);
  }
}

// --- Emergency stop --------------------------------------------------------

estopBtn.addEventListener('click', async () => {
  const ok = await confirmAction({
    title: '紧急停止',
    body: '将停止所有正在运行的 launch（hardware / navigation / services / mission 等）。确定继续？',
  });
  if (!ok) return;
  const r = await fetch('/api/estop', { method: 'POST' });
  if (!r.ok) alert('紧急停止失败');
});

// --- Log stream ------------------------------------------------------------

function appendLog(msg) {
  const line = document.createElement('div');
  line.innerHTML = `<span class="log-name">[${escapeHtml(msg.name)}]</span>${escapeHtml(msg.line || '')}`;
  logbox.appendChild(line);
  while (logbox.childNodes.length > LOG_MAX_LINES) logbox.removeChild(logbox.firstChild);
  logbox.scrollTop = logbox.scrollHeight;
}
function escapeHtml(s) {
  return String(s).replace(/[&<>"']/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
}

// --- Bootstrap -------------------------------------------------------------

async function fetchHealth() {
  try {
    const r = await fetch('/api/health');
    const data = await r.json();
    roModeEl.textContent = data.readonly ? '只读' : '读写';
  } catch { roModeEl.textContent = '未知'; }
}

async function main() {
  await Promise.all([fetchHealth(), fetchLaunchRegistry()]);
  renderLaunches([]);  // seed from registry even before first WS tick
  connectWithRetry('/api/state', renderDashboard, 'state');
  connectWithRetry('/api/logs', appendLog, 'logs');
  // Refresh registry occasionally in case launches are added at runtime.
  setInterval(fetchLaunchRegistry, 30000);
}

main().catch((err) => console.error(err));
