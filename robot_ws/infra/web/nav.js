// 遥控、建图与取点： map/costmap layers, scan, plans, robot pose,
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
const waypoints = [];
let waypointLayer = null;
let nextWaypointTask = { task: 'navigation' };
let drag = null;         // pose/goal arrow or pan state

// --- Grid layers -------------------------------------------------------------

async function loadGrid(layer, version, url = "/api/nav/grid.png?layer=" + layer) {
  if (loading[layer]) return;
  loading[layer] = true;
  try {
    const r = await fetch(url, { cache: 'no-store' });
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
    return true;
  } catch (err) {
    console.warn('grid load failed', layer, err);
  } finally {
    loading[layer] = false;
  }
}

function baseLayer() { return $('mapLayer').value; }

function refreshGrids() {
  if (baseLayer() === 'saved') return;
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
  if (baseLayer() !== 'saved' && $('showGlobalCost').checked && grids.global_costmap) drawGrid(grids.global_costmap, 0.8);
  if (baseLayer() !== 'saved' && $('showLocalCost').checked && grids.local_costmap) drawGrid(grids.local_costmap, 0.9);
  if (baseLayer() !== 'saved' && overlay) {
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
  if (base && baseLayer() === waypointLayer) {
    drawPolyline(waypoints.map(point => [point.x, point.y]), '#a98aff', 2);
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.font = '12px sans-serif';
    for (const point of waypoints) {
      const x = view.tx + point.x * view.s;
      const y = view.ty - point.y * view.s;
      ctx.fillStyle = '#8f5cff';
      ctx.beginPath();
      ctx.arc(x, y, 5, 0, Math.PI * 2);
      ctx.fill();
      ctx.fillStyle = '#e5edf7';
      ctx.fillText(point.id, x + 8, y - 6);
      drawArrow(point.x, point.y, point.yaw, '#8f5cff', 0.15);
    }
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
  if (g) $('mapStatus').textContent = `${baseLayer() === 'saved' ? '已加载地图' : baseLayer() === 'navigation' ? '/navigation_map' : '/map'} ${g.w}×${g.h} @ ${g.res} m · 滚轮缩放，浏览模式下拖动平移`;
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
  if (d.kind === 'waypoint') {
    const grid = grids[baseLayer()];
    const dx = body.x - (grid?.origin[0] ?? 0);
    const dy = body.y - (grid?.origin[1] ?? 0);
    const angle = grid?.origin[2] ?? 0;
    const gx = (dx * Math.cos(angle) + dy * Math.sin(angle)) / (grid?.res ?? 1);
    const gy = (-dx * Math.sin(angle) + dy * Math.cos(angle)) / (grid?.res ?? 1);
    if (grid && ['localization', 'saved'].includes(baseLayer()) && gx >= 0 && gy >= 0 && gx < grid.w && gy < grid.h) {
      addWaypoint(body);
    } else {
      toast('请先加载当前地图，在地图范围内取点', 'err');
    }
    setMode('view');
    draw();
    return;
  }
  if (baseLayer() === 'saved') {
    toast('请先启动定位，或切回实时地图', 'err');
    setMode('view');
    draw();
    return;
  }
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
let teleopConnected = false;
let keyboardEnabled = false;
const held = new Set();
const KEY_DIRS = { w: 'fwd', arrowup: 'fwd', s: 'back', arrowdown: 'back', a: 'left', arrowleft: 'left',
  d: 'right', arrowright: 'right', q: 'strafeleft', e: 'straferight' };

function teleopCommand() {
  const lin = Number($('linSpeed').value); const ang = Number($('angSpeed').value);
  const axis = (positive, negative) => Number(held.has(positive)) - Number(held.has(negative));
  const x = axis('fwd', 'back');
  const y = axis('strafeleft', 'straferight');
  const scale = lin / Math.max(1, Math.hypot(x, y));
  return { linear: x * scale, lateral: y * scale, angular: axis('left', 'right') * ang };
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
    sendTeleop({ linear: 0, lateral: 0, angular: 0 });
  }
  for (const b of document.querySelectorAll('.pad-btn')) b.classList.toggle('is-on', held.has(b.dataset.dir));
}

function releaseAll() { held.clear(); updateTeleop(); }

function teleopReady() {
  return active && teleopConnected && !fieldBusy && (fieldState?.ros?.nodes || []).includes('/community_serial');
}

function renderTeleopControls() {
  const ready = teleopReady();
  if (!ready) {
    keyboardEnabled = false;
    if (held.size) releaseAll();
  }
  const button = $('keyboardToggle');
  button.disabled = !ready;
  button.textContent = keyboardEnabled ? '关闭键盘控制' : '启用键盘控制';
  button.classList.toggle('is-on', keyboardEnabled);
  button.setAttribute('aria-pressed', String(keyboardEnabled));
  for (const pad of document.querySelectorAll('.pad-btn')) pad.disabled = !ready;
}

$('keyboardToggle').addEventListener('click', () => {
  if (!teleopReady()) return;
  keyboardEnabled = !keyboardEnabled;
  if (!keyboardEnabled) releaseAll();
  renderTeleopControls();
});

function openTeleop() {
  if (teleopWs) return;
  teleopWs = connectWithRetry('/api/nav/teleop', (msg) => { if (msg.ok === false) $('teleopState').textContent = msg.error; }, {
    onOpen: () => {
      teleopConnected = true;
      $('teleopState').textContent = '已连接';
      renderTeleopControls();
    },
    onClose: (ev) => {
      teleopConnected = false;
      renderTeleopControls();
      $('teleopState').textContent = ev.code === 4409 ? '只读模式，禁止遥控' : ev.code === 4401 ? 'Token 无效' : '未连接';
    },
  });
}

function closeTeleop() {
  releaseAll();
  if (teleopWs) { teleopWs.close(); teleopWs = null; }
  teleopConnected = false;
  renderTeleopControls();
  $('teleopState').textContent = '未连接';
}

for (const b of document.querySelectorAll('.pad-btn')) {
  const dir = b.dataset.dir;
  if (dir === 'stop') { b.addEventListener('click', releaseAll); continue; }
  b.addEventListener('pointerdown', ev => {
    if (!teleopReady()) return;
    ev.preventDefault();
    b.setPointerCapture(ev.pointerId);
    held.add(dir);
    updateTeleop();
  });
  for (const evName of ['pointerup', 'pointercancel', 'lostpointercapture']) b.addEventListener(evName, () => { held.delete(dir); updateTeleop(); });
}

window.addEventListener('keydown', (ev) => {
  const el = document.activeElement;
  const typing = el && (el.tagName === 'SELECT' || el.tagName === 'TEXTAREA'
    || (el.tagName === 'INPUT' && !['range', 'checkbox'].includes(el.type)));
  if (!keyboardEnabled || !teleopReady() || ev.repeat || typing || ev.ctrlKey || ev.altKey || ev.metaKey) return;
  if (ev.key === ' ' && el?.id === 'keyboardToggle') return;
  const dir = KEY_DIRS[ev.key.toLowerCase()];
  if (dir) { ev.preventDefault(); held.add(dir); updateTeleop(); }
  if (ev.key === ' ') { ev.preventDefault(); releaseAll(); }
});
window.addEventListener('keyup', (ev) => {
  if (!keyboardEnabled) return;
  const dir = KEY_DIRS[ev.key.toLowerCase()];
  if (dir) { held.delete(dir); updateTeleop(); }
});
window.addEventListener('blur', releaseAll);
document.addEventListener('visibilitychange', () => { if (document.hidden) releaseAll(); });
$('linSpeed').addEventListener('input', () => { $('linVal').textContent = Number($('linSpeed').value).toFixed(2); });
$('angSpeed').addEventListener('input', () => { $('angVal').textContent = Number($('angSpeed').value).toFixed(2); });

// --- Mapping, localization, and waypoint drafts -----------------------------

function setFieldMode(fieldMode) {
  const takingPoints = fieldMode === 'waypoints';
  $('fieldMappingPane').hidden = takingPoints;
  $('fieldWaypointPane').hidden = !takingPoints;
  document.querySelector('#navModes [data-mode="waypoint"]').hidden = !takingPoints;
  if (!takingPoints) {
    if (drag?.kind === 'waypoint') drag = null;
    if (mode === 'waypoint') setMode('view');
  }
  for (const button of document.querySelectorAll('[data-field-mode]')) {
    const selected = button.dataset.fieldMode === fieldMode;
    button.classList.toggle('is-on', selected);
    button.setAttribute('aria-pressed', String(selected));
  }
}

for (const button of document.querySelectorAll('[data-field-mode]')) {
  button.addEventListener('click', () => setFieldMode(button.dataset.fieldMode));
}

let fieldState = null;
let fieldBusy = false;

function renderFieldState() {
  const nodes = fieldState?.ros?.nodes || [];
  const records = fieldState?.launches || [];
  const running = name => records.some(row => row.name === name && row.status === 'running');
  const hardware = nodes.includes('/community_serial') || running('hardware');
  const mapping = running('mapping');
  const navigation = running('navigation');
  const localizing = navigation && records.some(row => row.name === 'navigation' && row.status === 'running' && row.args?.localization_only === 'true');
  const localizationInUse = nodes.includes('/ekf_localization') || mapping || navigation;
  const hardwareReady = nodes.includes('/community_serial');
  $('fieldHardware').disabled = fieldBusy || !fieldState?.ros?.master || hardware;
  $('teleopHardwareStatus').textContent = hardwareReady ? '底盘与传感器已启动'
    : hardware ? '等待底盘节点就绪' : '底盘与传感器未启动';
  renderTeleopControls();
  $('slamStart').disabled = fieldBusy || !hardware || localizationInUse;
  $('slamSave').disabled = fieldBusy || !mapping;
  $('slamStop').disabled = fieldBusy || !mapping;
  $('fieldPreview').disabled = fieldBusy;
  $('fieldLocalize').disabled = fieldBusy || !hardware || localizationInUse;
  $('fieldLocalizeStop').disabled = fieldBusy || !localizing;
  $('fieldStatus').textContent = fieldBusy ? '正在处理…'
    : mapping ? '建图中' : localizing ? '定位中，可用 2D Pose Estimate 设置初始位姿'
    : localizationInUse ? '已有建图或导航节点在运行'
    : hardware ? '可开始建图或加载地图' : '请先在遥控区启动底盘与传感器';
}

on('state', state => {
  fieldState = state;
  if (active) renderFieldState();
});

async function refreshSavedMaps(selected = $('fieldMap').value) {
  const { maps } = await api('/api/nav/maps');
  $('fieldMap').replaceChildren(new Option('当前配置地图', ''), ...maps.map(map => new Option(map.name, map.path)));
  $('fieldMap').value = selected;
}

function focusCurrentMap() {
  $('mapLayer').value = 'localization';
  $('mapLayer').dispatchEvent(new Event('change'));
}

async function fieldAction(action) {
  if (fieldBusy) return;
  releaseAll();
  fieldBusy = true;
  renderFieldState();
  try {
    await action();
    fieldState = await api('/api/state');
  } catch (err) {
    toast(err.message, 'err');
  } finally {
    fieldBusy = false;
    renderFieldState();
  }
}

const startLaunch = (name, body) => api('/api/launch/' + name + '/start', { method: 'POST', body });
const stopLaunch = name => api('/api/launch/' + name + '/stop', { method: 'POST' });
$('fieldHardware').addEventListener('click', () => fieldAction(async () => {
  await startLaunch('hardware', { base_protocol: 'aa_bb' });
  emit('launch-changed');
}));
function confirmMapChange() {
  return !waypoints.length || window.confirm('切换地图后将清空本页航点，请先导出要保留的航点。是否继续？');
}
function startWithMap(name, args) {
  if (!confirmMapChange()) return;
  fieldAction(async () => {
    await startLaunch(name, args);
    waypoints.length = 0;
    renderWaypoints();
    focusCurrentMap();
    emit('launch-changed');
  });
}
$('slamStart').addEventListener('click', () => startWithMap('mapping', { algorithm: $('slamAlgorithm').value }));
$('slamSave').addEventListener('click', () => fieldAction(async () => {
  const saved = await api('/api/nav/maps/save', { method: 'POST' });
  await refreshSavedMaps(saved.path);
  toast('已保存地图：' + saved.name, 'ok');
}));
$('slamStop').addEventListener('click', () => fieldAction(() => stopLaunch('mapping')));
$('fieldPreview').addEventListener('click', () => {
  if (!confirmMapChange()) return;
  fieldAction(async () => {
    const url = '/api/nav/saved_grid.png?map_file=' + encodeURIComponent($('fieldMap').value);
    if (!await loadGrid('saved', 0, url)) throw new Error('加载地图失败');
    waypoints.length = 0;
    $('savedLayer').disabled = false;
    $('mapLayer').value = 'saved';
    $('mapLayer').dispatchEvent(new Event('change'));
    renderInfo();
    renderWaypoints();
  });
});
$('fieldLocalize').addEventListener('click', () => startWithMap('navigation', { map: $('fieldMap').value, localization_only: 'true' }));
$('fieldLocalizeStop').addEventListener('click', () => fieldAction(() => stopLaunch('navigation')));

function selectedWaypoint() {
  const id = $('waypointTarget').value;
  return waypoints.find(point => point.id === id);
}

function syncWaypointTaskForm() {
  const { task, area, parking_lot } = selectedWaypoint() || nextWaypointTask;
  $('waypointTask').value = task;
  $('waypointArea').value = area || 'A';
  $('waypointLot').value = parking_lot || 1;
  $('waypointAreaField').hidden = task !== 'person';
  $('waypointLotField').hidden = task !== 'plate';
}

function applyWaypointTask() {
  const task = $('waypointTask').value;
  $('waypointAreaField').hidden = task !== 'person';
  $('waypointLotField').hidden = task !== 'plate';
  const fields = { task };
  if (task === 'person') fields.area = $('waypointArea').value;
  if (task === 'plate') {
    const lot = Number($('waypointLot').value);
    if (!Number.isSafeInteger(lot) || lot < 1) {
      toast('车位编号须为正整数', 'err');
      return false;
    }
    fields.parking_lot = lot;
  }
  const point = selectedWaypoint();
  if (point) {
    delete point.area;
    delete point.parking_lot;
    Object.assign(point, fields);
    renderWaypoints();
  } else {
    nextWaypointTask = fields;
  }
  return true;
}
$('waypointTarget').addEventListener('change', syncWaypointTaskForm);
for (const id of ['waypointTask', 'waypointArea', 'waypointLot']) {
  $(id).addEventListener('change', applyWaypointTask);
}

function renderWaypoints() {
  const select = $('waypointTarget');
  const selectedId = select.value;
  select.replaceChildren(new Option('下一航点', ''), ...waypoints.map(point => new Option(point.id, point.id)));
  select.value = waypoints.some(point => point.id === selectedId) ? selectedId : '';
  syncWaypointTaskForm();
  $('waypointCount').textContent = waypoints.length + ' 个';
  $('undoWaypoint').disabled = $('exportWaypoints').disabled = !waypoints.length;
  const taskLabels = new Map(Array.from($('waypointTask').options, option => [option.value, option.text]));
  $('waypointList').replaceChildren(...waypoints.map(point => {
    const row = document.createElement('li');
    row.textContent = point.id + ' · ' + point.x.toFixed(2) + ', ' + point.y.toFixed(2)
      + ' m · ' + (point.yaw * 180 / Math.PI).toFixed(0) + '° · ' + taskLabels.get(point.task)
      + (point.area ? ' ' + point.area + ' 区' : '') + (point.parking_lot ? ' ' + point.parking_lot + ' 号' : '');
    return row;
  }));
  draw();
}

function addWaypoint(pose) {
  if (waypoints.length && waypointLayer !== baseLayer()) {
    toast('请先导出并撤销已有航点，避免混用地图', 'err');
    return;
  }
  if (!$('waypointTarget').value && !applyWaypointTask()) return;
  waypointLayer = baseLayer();
  const id = 'wp_' + String(waypoints.length + 1).padStart(2, '0');
  waypoints.push({ id, ...pose, ...nextWaypointTask });
  renderWaypoints();
}

$('recordWaypoint').addEventListener('click', async () => {
  const button = $('recordWaypoint');
  button.disabled = true;
  try {
    const pose = await api('/api/nav/waypoint_pose');
    focusCurrentMap();
    addWaypoint(pose);
  } catch (err) {
    toast(err.message, 'err');
  } finally {
    button.disabled = false;
  }
});
$('undoWaypoint').addEventListener('click', () => { waypoints.pop(); renderWaypoints(); });
function serializeWaypoints() {
  const lines = ['points:'];
  let recording = false;
  for (const { id, x, y, yaw, task, area, parking_lot } of waypoints) {
    if (task === 'record_start') {
      if (recording) throw new Error(id + '：上一次录像尚未结束');
      recording = true;
    } else if (task === 'record_stop') {
      if (!recording) throw new Error(id + '：请先设置开始录像航点');
      recording = false;
    }
    lines.push('  - id: ' + id, '    x: ' + x, '    y: ' + y, '    yaw: ' + yaw, '    task: ' + task);
    if (task === 'person') lines.push('    area: ' + area);
    if (task === 'plate') lines.push('    parking_lot: ' + parking_lot);
  }
  if (recording) throw new Error('请补上结束录像航点');
  return lines.join('\n') + '\n';
}

$('exportWaypoints').addEventListener('click', async () => {
  try {
    if (!applyWaypointTask()) return;
    const route = serializeWaypoints();
    if (typeof window.showSaveFilePicker === 'function') {
      const file = await window.showSaveFilePicker({
        suggestedName: 'waypoints.yaml',
        types: [{ description: '航点路线 YAML', accept: { 'application/yaml': ['.yaml', '.yml'] } }],
      });
      const writer = await file.createWritable();
      await writer.write(route);
      await writer.close();
      toast('已保存 ' + file.name, 'ok');
    } else {
      const url = URL.createObjectURL(new Blob([route], { type: 'application/yaml' }));
      const link = document.createElement('a');
      link.href = url;
      link.download = 'waypoints.yaml';
      link.click();
      setTimeout(() => URL.revokeObjectURL(url), 1000);
    }
  } catch (err) {
    if (err.name !== 'AbortError') toast(err.message, 'err');
  }
});

// --- Lifecycle --------------------------------------------------------------

on('estop', () => {
  keyboardEnabled = false;
  releaseAll();
  renderTeleopControls();
});
on('view', (name) => {
  const now = name === 'teleop';
  if (now === active) return;
  active = now;
  if (active) {
    resizeCanvas();
    if (view) fitView();
    openTeleop();
    renderFieldState();
    refreshSavedMaps().catch(err => toast(err.message, 'err'));
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
