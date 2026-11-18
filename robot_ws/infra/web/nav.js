// 遥控 · 地图 view: map/costmap layers, scan, plans, robot pose,
// 2D Pose Estimate / 2D Nav Goal by drag, teleop over WebSocket.

import { api, connectWithRetry, kvRows, on, toast } from './common.js';

const $ = (id) => document.getElementById(id);
const canvas = $('mapCanvas');
const ctx = canvas.getContext('2d');

let active = false;
let mode = 'view';
let overlay = null;
let overlayTimer = null;
const grids = {};        // layer → { bitmap, origin:[x,y,yaw], res, w, h, version, fetchedAt }
const loading = {};
const failedAt = {};      // layer → time of last failed load (retry after 2 s)
let view = null;         // { s, tx, ty }: screen = (tx + x*s, ty - y*s)
let drag = null;         // pose/goal arrow or pan state

// --- Grid layers -------------------------------------------------------------

async function loadGrid(layer, version) {
  if (loading[layer]) return;
  loading[layer] = true;
  try {
    const r = await fetch(`/api/nav/grid.png?layer=${layer}`, { cache: 'no-store' });
    if (!r.ok) {
      failedAt[layer] = performance.now();
      const d = await r.json().catch(() => ({}));
      if (layer === baseLayer()) $('mapStatus').textContent = d.detail || `地图加载失败 (${r.status})`;
      return;
    }
    const [ox, oy, oyaw] = r.headers.get('X-Map-Origin').split(',').map(Number);
    const [w, h] = r.headers.get('X-Map-Size').split(',').map(Number);
    const bitmap = await createImageBitmap(await r.blob());
    grids[layer] = { bitmap, origin: [ox, oy, oyaw], res: Number(r.headers.get('X-Map-Resolution')), w, h,
      version, fetchedAt: performance.now() };
    if (layer === baseLayer() && !view) fitView();
  } catch (err) {
    console.warn('grid load failed', layer, err);
  } finally {
    loading[layer] = false;
  }
}

function baseLayer() { return $('mapLayer').value; }

function refreshGrids() {
  const versions = (overlay && overlay.grid_versions) || {};
  const wanted = [[baseLayer(), 0]];
  if ($('showGlobalCost').checked) wanted.push(['global_costmap', 2000]);
  // Local costmap is a rolling window: its origin moves with the robot.
  if ($('showLocalCost').checked) wanted.push(['local_costmap', 500]);
  for (const [layer, minMs] of wanted) {
    const g = grids[layer];
    const version = versions[layer];
    if (version === undefined) continue;
    if (failedAt[layer] && performance.now() - failedAt[layer] < 2000) continue;
    const stale = !g || g.version !== version;
    if (stale && (!g || performance.now() - g.fetchedAt >= minMs)) loadGrid(layer, version);
  }
}

// --- View transform ---------------------------------------------------------

function resizeCanvas() {
  const rect = canvas.parentElement.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  canvas.width = Math.max(1, Math.round(rect.width * dpr));
  canvas.height = Math.max(1, Math.round(rect.height * dpr));
  canvas.style.width = `${rect.width}px`;
  canvas.style.height = `${rect.height}px`;
}

function fitView() {
  const g = grids[baseLayer()];
  if (!g) return;
  const wm = g.w * g.res; const hm = g.h * g.res;
  const s = 0.95 * Math.min(canvas.width / wm, canvas.height / hm);
  const cx = g.origin[0] + wm / 2; const cy = g.origin[1] + hm / 2;
  view = { s, tx: canvas.width / 2 - cx * s, ty: canvas.height / 2 + cy * s };
}

function toWorld(ev) {
  const rect = canvas.getBoundingClientRect();
  const px = (ev.clientX - rect.left) * (canvas.width / rect.width);
  const py = (ev.clientY - rect.top) * (canvas.height / rect.height);
  return { x: (px - view.tx) / view.s, y: (view.ty - py) / view.s, px, py };
}

// --- Drawing ----------------------------------------------------------------

function worldTransform() { ctx.setTransform(view.s, 0, 0, -view.s, view.tx, view.ty); }

function drawGrid(g, alpha = 1) {
  worldTransform();
  ctx.globalAlpha = alpha;
  ctx.translate(g.origin[0], g.origin[1]);
  ctx.rotate(g.origin[2]);
  ctx.scale(g.res, g.res);
  // Row 0 of the image is the grid origin row; with the y-up transform it
  // lands at the origin and grows towards +y as in ROS.
  ctx.drawImage(g.bitmap, 0, 0);
  ctx.globalAlpha = 1;
}

function drawPolyline(points, color, width) {
  if (!points || points.length < 2) return;
  worldTransform();
  ctx.beginPath();
  ctx.moveTo(points[0][0], points[0][1]);
  for (const [x, y] of points.slice(1)) ctx.lineTo(x, y);
  ctx.setTransform(1, 0, 0, 1, 0, 0);
  ctx.strokeStyle = color; ctx.lineWidth = width; ctx.stroke();
}

function drawArrow(x, y, yaw, color, len = 0.25) {
  const sx = view.tx + x * view.s; const sy = view.ty - y * view.s;
  const L = Math.max(14, len * view.s);
  ctx.setTransform(1, 0, 0, 1, 0, 0);
  ctx.save();
  ctx.translate(sx, sy);
  ctx.rotate(-yaw);
  ctx.fillStyle = color; ctx.strokeStyle = '#fff'; ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.moveTo(L, 0); ctx.lineTo(-L * 0.45, L * 0.45); ctx.lineTo(-L * 0.2, 0); ctx.lineTo(-L * 0.45, -L * 0.45);
  ctx.closePath(); ctx.fill(); ctx.stroke();
  ctx.restore();
}

function draw() {
  ctx.setTransform(1, 0, 0, 1, 0, 0);
  ctx.fillStyle = '#0c1521';
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  if (!view) return;
  ctx.imageSmoothingEnabled = false;
  const base = grids[baseLayer()];
  if (base) drawGrid(base);
  if ($('showGlobalCost').checked && grids.global_costmap) drawGrid(grids.global_costmap, 0.8);
  if ($('showLocalCost').checked && grids.local_costmap) drawGrid(grids.local_costmap, 0.9);
  if (overlay) {
    if ($('showPlans').checked) {
      drawPolyline(overlay.global_plan, '#16a34a', 2.5);
      drawPolyline(overlay.local_plan, '#2563eb', 3);
    }
    if ($('showScan').checked && overlay.scan.length) {
      ctx.setTransform(1, 0, 0, 1, 0, 0);
      ctx.fillStyle = '#ef4444';
      for (const [x, y] of overlay.scan) ctx.fillRect(view.tx + x * view.s - 1.5, view.ty - y * view.s - 1.5, 3, 3);
    }
    if (overlay.pose) drawArrow(overlay.pose.x, overlay.pose.y, overlay.pose.yaw, '#00a383');
  }
  if (drag && drag.kind !== 'pan' && drag.end) {
    const yaw = Math.atan2(drag.end.y - drag.start.y, drag.end.x - drag.start.x);
    drawArrow(drag.start.x, drag.start.y, yaw, drag.kind === 'pose' ? '#f59e0b' : '#7c3aed');
  }
}

function renderInfo() {
  const p = overlay && overlay.pose;
  const g = grids[baseLayer()];
  $('navInfo').innerHTML = kvRows([
    ['ROS', overlay && overlay.ready ? '已连接' : '未连接'],
    ['位姿', p ? `${p.x.toFixed(2)}, ${p.y.toFixed(2)}, ${(p.yaw * 180 / Math.PI).toFixed(0)}°` : '—'],
    ['全局路径', overlay && overlay.global_plan.length ? `${overlay.global_plan.length} 点` : '无'],
    ['激光点', overlay ? overlay.scan.length : 0],
  ]);
  if (g) $('mapStatus').textContent = `${baseLayer() === 'navigation' ? '/navigation_map' : '/map'} ${g.w}×${g.h} @ ${g.res} m · 滚轮缩放，浏览模式下拖动平移`;
}

async function pollOverlay() {
  try {
    overlay = await api('/api/nav/overlay');
    refreshGrids();
    renderInfo();
  } catch (err) {
    $('mapStatus').textContent = `获取导航数据失败：${err.message}`;
  }
  draw();
}

// --- Mouse: pan / zoom / pose / goal ----------------------------------------

canvas.addEventListener('wheel', (ev) => {
  if (!view) return;
  ev.preventDefault();
  const w = toWorld(ev);
  const k = ev.deltaY < 0 ? 1.15 : 1 / 1.15;
  view.s *= k;
  view.tx = w.px - w.x * view.s;
  view.ty = w.py + w.y * view.s;
  draw();
}, { passive: false });

canvas.addEventListener('pointerdown', (ev) => {
  if (!view) return;
  canvas.setPointerCapture(ev.pointerId);
  const w = toWorld(ev);
  drag = mode === 'view' ? { kind: 'pan', px: w.px, py: w.py } : { kind: mode, start: w, end: null };
});
canvas.addEventListener('pointermove', (ev) => {
  if (!drag) return;
  const w = toWorld(ev);
  if (drag.kind === 'pan') {
    view.tx += w.px - drag.px; view.ty += w.py - drag.py;
    drag.px = w.px; drag.py = w.py;
  } else {
    drag.end = w;
  }
  draw();
});
canvas.addEventListener('pointerup', async (ev) => {
  if (!drag) return;
  const d = drag; drag = null;
  if (d.kind === 'pan') return;
  const end = toWorld(ev);
  const dx = end.x - d.start.x; const dy = end.y - d.start.y;
  // Like RViz: the drag direction sets the heading (no drag → facing +x).
  const yaw = Math.hypot(dx, dy) * view.s > 8 ? Math.atan2(dy, dx) : 0;
  const body = { x: Number(d.start.x.toFixed(3)), y: Number(d.start.y.toFixed(3)), yaw: Number(yaw.toFixed(4)) };
  const path = d.kind === 'pose' ? '/api/nav/pose_estimate' : '/api/nav/goal';
  try {
    await api(path, { method: 'POST', body });
    toast(`${d.kind === 'pose' ? '已发布初始位姿' : '已发布导航目标'} (${body.x}, ${body.y}, ${(yaw * 180 / Math.PI).toFixed(0)}°)`, 'ok');
  } catch (err) {
    toast(`发布失败：${err.message}`, 'err');
  }
  setMode('view');
  draw();
});

function setMode(m) {
  mode = m;
  for (const b of $('navModes').querySelectorAll('button')) b.classList.toggle('is-on', b.dataset.mode === m);
  canvas.style.cursor = m === 'view' ? 'grab' : 'crosshair';
}
for (const b of $('navModes').querySelectorAll('button')) b.addEventListener('click', () => setMode(b.dataset.mode));

$('mapLayer').addEventListener('change', () => {
  view = null;
  if (grids[baseLayer()]) fitView();
  refreshGrids();
  draw();
});
for (const id of ['showGlobalCost', 'showLocalCost', 'showScan', 'showPlans']) $(id).addEventListener('change', () => { refreshGrids(); draw(); });
$('fitMap').addEventListener('click', () => { fitView(); draw(); });
$('cancelNav').addEventListener('click', async () => {
  try { await api('/api/nav/cancel', { method: 'POST' }); toast('已取消导航目标', 'ok'); }
  catch (err) { toast(`取消失败：${err.message}`, 'err'); }
});

// --- Teleop -----------------------------------------------------------------

let teleopWs = null;
let teleopTimer = null;
const held = new Set();
const KEY_DIRS = { w: 'fwd', arrowup: 'fwd', s: 'back', arrowdown: 'back', a: 'left', arrowleft: 'left',
  d: 'right', arrowright: 'right', q: 'fwdleft', e: 'fwdright' };

function teleopCommand() {
  const lin = Number($('linSpeed').value); const ang = Number($('angSpeed').value);
  let linear = 0; let angular = 0;
  for (const dir of held) {
    if (dir === 'fwd' || dir.startsWith('fwd')) linear += lin;
    if (dir === 'back') linear -= lin;
    if (dir === 'left' || dir === 'fwdleft') angular += ang;
    if (dir === 'right' || dir === 'fwdright') angular -= ang;
  }
  return { linear: Math.max(-lin, Math.min(lin, linear)), angular: Math.max(-ang, Math.min(ang, angular)) };
}

function sendTeleop(cmd) {
  if (teleopWs) teleopWs.send(JSON.stringify(cmd));
}

function updateTeleop() {
  if (held.size && !teleopTimer) {
    sendTeleop(teleopCommand());
    teleopTimer = setInterval(() => sendTeleop(teleopCommand()), 100);
  } else if (!held.size && teleopTimer) {
    clearInterval(teleopTimer); teleopTimer = null;
    sendTeleop({ linear: 0, angular: 0 });
  }
  for (const b of document.querySelectorAll('.pad-btn')) b.classList.toggle('is-on', held.has(b.dataset.dir));
}

function releaseAll() { held.clear(); updateTeleop(); }

function openTeleop() {
  if (teleopWs) return;
  teleopWs = connectWithRetry('/api/nav/teleop', (msg) => { if (msg.ok === false) $('teleopState').textContent = msg.error; }, {
    onOpen: () => { $('teleopState').textContent = '已连接'; },
    onClose: (ev) => {
      $('teleopState').textContent = ev.code === 4409 ? '只读模式，禁止遥控' : ev.code === 4401 ? 'Token 无效' : '未连接';
    },
  });
}

function closeTeleop() {
  releaseAll();
  if (teleopWs) { teleopWs.close(); teleopWs = null; }
  $('teleopState').textContent = '未连接';
}

for (const b of document.querySelectorAll('.pad-btn')) {
  const dir = b.dataset.dir;
  if (dir === 'stop') { b.addEventListener('click', releaseAll); continue; }
  b.addEventListener('pointerdown', (ev) => { ev.preventDefault(); b.setPointerCapture(ev.pointerId); held.add(dir); updateTeleop(); });
  for (const evName of ['pointerup', 'pointercancel']) b.addEventListener(evName, () => { held.delete(dir); updateTeleop(); });
}

window.addEventListener('keydown', (ev) => {
  const el = document.activeElement;
  const typing = el && (el.tagName === 'SELECT' || el.tagName === 'TEXTAREA'
    || (el.tagName === 'INPUT' && !['range', 'checkbox'].includes(el.type)));
  if (!active || ev.repeat || typing) return;
  const dir = KEY_DIRS[ev.key.toLowerCase()];
  if (dir) { ev.preventDefault(); held.add(dir); updateTeleop(); }
  if (ev.key === ' ') { ev.preventDefault(); releaseAll(); }
});
window.addEventListener('keyup', (ev) => {
  const dir = KEY_DIRS[ev.key.toLowerCase()];
  if (dir) { held.delete(dir); updateTeleop(); }
});
window.addEventListener('blur', releaseAll);
document.addEventListener('visibilitychange', () => { if (document.hidden) releaseAll(); });
$('linSpeed').addEventListener('input', () => { $('linVal').textContent = Number($('linSpeed').value).toFixed(2); });
$('angSpeed').addEventListener('input', () => { $('angVal').textContent = Number($('angSpeed').value).toFixed(2); });

// --- Lifecycle --------------------------------------------------------------

on('estop', releaseAll);
on('view', (name) => {
  const now = name === 'teleop';
  if (now === active) return;
  active = now;
  if (active) {
    resizeCanvas();
    if (view) fitView();
    openTeleop();
    pollOverlay();
    overlayTimer = setInterval(pollOverlay, 300);
  } else {
    clearInterval(overlayTimer); overlayTimer = null;
    closeTeleop();
  }
});
window.addEventListener('resize', () => { if (active) { resizeCanvas(); fitView(); draw(); } });
setMode('view');
$('navInfo').innerHTML = kvRows([['ROS', '—']]);
