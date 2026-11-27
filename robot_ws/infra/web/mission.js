// 巡检任务 view: start/stop mission.launch, live /mission/state, run browser.

import { api, escapeHtml, fmtDuration, kvRows, on, toast } from './common.js';

const $ = (id) => document.getElementById(id);
let active = false;
let waypointsLoaded = false;
let lastSnapshot = null;
const preparing = new Set();
let missionStarting = false;
let liveRunId = '';
let livePending = false;
let liveNextAt = 0;
let liveDone = false;
let liveSignature = '';

const STATE_BADGE = {
  INIT: ['', '初始化'], NAVIGATE: ['ok', '导航中'], TASK: ['ok', '执行任务'], NEXT_POINT: ['ok', '切换航点'],
  FINISH: ['ok', '已完成'], ERROR: ['err', '失败'],
};

async function loadWaypoints() {
  try {
    const { points } = await api('/api/mission/waypoints');
    $('misWaypoint').innerHTML = points.map((p, i) =>
      `<option value="${escapeHtml(p.id)}">${i + 1}. ${escapeHtml(p.id)}（${escapeHtml(p.task)}）</option>`).join('');
    waypointsLoaded = true;
  } catch (err) {
    toast(`读取航点失败：${err.message}`, 'err');
  }
}

function renderReadiness() {
  const nodes = lastSnapshot?.ros?.nodes || [];
  const online = Boolean(lastSnapshot?.ros?.master);
  const launches = (lastSnapshot?.launches || []).filter(record => record.status === 'running');
  const composite = record => ['community', 'field', 'finals'].includes(record.name);
  const hardwareLaunch = launches.some(record => record.name === 'hardware' || composite(record));
  const navigationLaunch = launches.find(record => (record.name === 'navigation' || composite(record))
    && record.args?.localization_only !== 'true');
  const servicesLaunch = launches.some(record => record.name === 'services' || composite(record));
  const debugPerception = launches.some(record => record.name === 'debug_perception');
  const hardware = nodes.includes('/community_serial');
  const navigation = nodes.includes('/move_base');
  const services = nodes.includes('/speech_server');
  const noServices = $('misNavOnly').checked;
  const taskOnly = $('misTaskOnly').checked;
  const camera = nodes.includes('/usb_cam') || hardwareLaunch || debugPerception;
  const localizing = !navigation && !navigationLaunch && (nodes.includes('/ekf_localization')
    || launches.some(record => record.name === 'mapping' || record.args?.localization_only === 'true'));
  const planner = navigationLaunch ? (navigationLaunch.args?.local_planner || 'dwa').toUpperCase() : '';
  const mission = nodes.includes('/community_mission') || Boolean(lastSnapshot?.mission?.node_alive)
    || launches.some(record => ['mission', 'field', 'finals'].includes(record.name)
      || (record.name === 'community' && record.args?.run_mission === 'true'));
  const starting = (name, running, ready) => preparing.has(name) || running && !ready;

  const startingHardware = starting('hardware', hardwareLaunch, hardware);
  const startingNavigation = starting('navigation', Boolean(navigationLaunch), navigation);
  const startingServices = preparing.has('debug_perception')
    || starting('services', servicesLaunch || debugPerception, services);
  $('misHardwareStatus').textContent = taskOnly ? '仅任务无需启动底盘' : hardware ? '已就绪' : startingHardware ? '启动中…' : '未启动';
  $('misPlannerStatus').textContent = taskOnly ? '仅任务无需启动导航' : navigation ? '已就绪' + (planner ? ' · ' + planner : '')
    : startingNavigation ? '启动中…' : localizing ? '请先停止建图或取点定位'
    : hardware ? '未启动' : '等待底盘就绪';
  $('misServicesStatus').textContent = noServices ? '仅导航无需启动' : services ? '已就绪' : startingServices ? '启动中…' : '未启动';
  $('misHardwareStart').hidden = taskOnly || hardware || startingHardware;
  $('misHardwareStart').disabled = !online || mission || startingHardware;
  $('misNavStart').hidden = taskOnly || navigation || startingNavigation;
  $('misNavStart').disabled = !online || !hardware || localizing || mission || startingNavigation;
  $('misPlanner').disabled = taskOnly || navigation || startingNavigation;
  $('misServicesStart').hidden = noServices || services || startingServices;
  $('misServicesStart').disabled = !online || mission || startingServices;
  $('misStart').disabled = !online || (taskOnly ? !camera : !hardware || !navigation)
    || (!noServices && !services) || mission || missionStarting;
  $('misServicesStart').textContent = taskOnly && !camera ? '启用相机与识别服务' : '启用识别服务';
  $('misStop').disabled = !online || !mission;
}

async function prepareLaunch(name, body, message) {
  if (preparing.has(name)) return;
  preparing.add(name);
  renderReadiness();
  try {
    const { record } = await api('/api/launch/' + name + '/start', { method: 'POST', body });
    if (record?.status === 'running' && lastSnapshot) {
      const launches = (lastSnapshot.launches || []).filter(item => item.name !== name);
      lastSnapshot = { ...lastSnapshot, launches: [...launches, record] };
    }
    toast(message, 'ok');
  } catch (err) {
    toast('启动失败：' + err.message, 'err');
  } finally {
    preparing.delete(name);
    renderReadiness();
  }
}

$('misHardwareStart').addEventListener('click', () => prepareLaunch('hardware', { base_protocol: 'aa_bb' }, '已请求启动底盘与传感器'));
$('misNavStart').addEventListener('click', () => {
  const planner = $('misPlanner').value;
  prepareLaunch('navigation', { local_planner: planner }, '已请求启动导航 · ' + planner.toUpperCase());
});
$('misServicesStart').addEventListener('click', () => {
  const taskOnly = $('misTaskOnly').checked;
  const camera = (lastSnapshot?.ros?.nodes || []).includes('/usb_cam')
    || (lastSnapshot?.launches || []).some(record => record.status === 'running'
      && ['hardware', 'community', 'field', 'finals', 'debug_perception'].includes(record.name));
  prepareLaunch(taskOnly && !camera ? 'debug_perception' : 'services', {}, '已请求启用相机与识别服务');
});
function syncNavigationMode() {
  const navigationOnly = $('misNavOnly').checked;
  const taskOnly = $('misTaskOnly').checked;
  $('misNavOnly').disabled = taskOnly;
  $('misTaskOnly').disabled = navigationOnly;
  $('misObstacleField').hidden = !navigationOnly;
  $('misObstacle').disabled = !navigationOnly;
  $('misAutoRouteHint').hidden = navigationOnly || taskOnly;
  $('misNavigationHint').hidden = !navigationOnly;
  $('misTaskHint').hidden = !taskOnly;
  renderReadiness();
}
$('misNavOnly').addEventListener('change', syncNavigationMode);
$('misTaskOnly').addEventListener('change', syncNavigationMode);
syncNavigationMode();

$('misScope').addEventListener('change', () => { $('misWaypoint').disabled = $('misScope').value !== 'single'; });

$('misStart').addEventListener('click', async () => {
  const single = $('misScope').value === 'single';
  if (single && !$('misWaypoint').value) { toast('请先选择航点（航点列表未加载）', 'err'); return; }
  const body = {
    waypoint: single ? $('misWaypoint').value : '',
    navigation_only: $('misNavOnly').checked,
    task_only: $('misTaskOnly').checked,
  };
  if (body.navigation_only) body.obstacle_mode = $('misObstacle').value;
  missionStarting = true;
  renderReadiness();
  try {
    const { record } = await api('/api/mission/start', { method: 'POST', body });
    if (record?.status === 'running' && lastSnapshot) {
      const launches = (lastSnapshot.launches || []).filter(item => item.name !== 'mission');
      lastSnapshot = { ...lastSnapshot, launches: [...launches, record] };
    }
    toast(`已启动${single ? `单航点 ${body.waypoint}` : '完整巡检'}${body.task_only ? '（仅任务）' : body.navigation_only ? '（仅导航）' : ''}`, 'ok');
  } catch (err) {
    toast(`启动失败：${err.message}`, 'err');
  } finally {
    missionStarting = false;
    renderReadiness();
  }
});

$('misStop').addEventListener('click', async () => {
  try {
    const res = await api('/api/mission/stop', { method: 'POST' });
    toast(res.message, 'ok');
  } catch (err) {
    toast(`停止失败：${err.message}`, 'err');
  }
});

function renderMission(state) {
  if (!state) {
    $('misBadge').className = 'badge';
    $('misBadge').textContent = '—';
    $('misInfo').innerHTML = '<div class="muted">尚未收到 /mission/state（巡检节点未运行过）</div>';
    return;
  }
  const [cls, text] = STATE_BADGE[state.state] || ['', state.state];
  const stale = !state.node_alive && !['FINISH', 'ERROR'].includes(state.state);
  $('misBadge').className = `badge ${stale ? 'warn' : cls}`;
  $('misBadge').textContent = stale ? `${text}（节点已退出）` : text;
  const op = state.last_operation;
  $('misInfo').innerHTML = kvRows([
    ['节点', state.node_alive ? '运行中' : '未运行'],
    ['航点', `${escapeHtml(state.current_waypoint)}（${state.index + 1}/${state.total}）`],
    ['任务类型', escapeHtml(state.task)],
    ['说明', escapeHtml(state.reason)],
    ['模式', `${state.single_waypoint ? `单航点 ${escapeHtml(state.single_waypoint)}` : '完整巡检'}${state.task_only ? ' · 仅任务 · 不导航' : state.navigation_only ? ' · 仅导航 · ' + (state.obstacle_mode === 'clear' ? '预设直行' : '预设绕行') : ' · 识别后自动选路'}`],
    ['最近操作', op ? `${escapeHtml(op.waypoint_id)} ${escapeHtml(op.task_type)} <span class="badge ${op.status === 'FAILED' ? 'err' : op.status === 'SUCCESS' ? 'ok' : ''}">${escapeHtml(op.status)}</span> ${escapeHtml(op.reason || '')}` : '—'],
    ['run_id', `<code>${escapeHtml(state.run_id)}</code>`],
    ['更新', `${state.received_age_s}s 前`],
  ]);
}

// --- Live results ------------------------------------------------------------

const TASK_LABELS = {
  navigation: '导航', person: '人员识别', plate: '车牌识别', traffic_light: '红绿灯',
  fire: '火焰识别', trash: '垃圾分类', obstacle: '障碍物识别', vehicle_photo: '车辆拍照',
  announcement: '播报', record_start: '开始录像', record_stop: '结束录像',
};
const RESULT_BADGES = {
  SUCCESS: ['ok', '成功'], FAILED: ['err', '失败'],
  RUNNING: ['warn', '进行中'], EMPTY_VALID: ['ok', '未检出目标'],
  SKIPPED: ['warn', '已跳过'],
};

function liveMedia(runId, op, files) {
  const paths = Array.isArray(op.paths) ? op.paths : [];
  const evidence = files.filter(file => paths.some(path => typeof path === 'string'
    && (path === file || path.endsWith('/' + file))));
  const photos = evidence.filter(file => /\.(jpe?g|png)$/i.test(file));
  const videos = op.ended_at ? evidence.filter(file => /\.(mp4|avi|mkv)$/i.test(file)) : [];
  const pictures = photos.length ? '<div class="thumbs">' + photos.map(file => {
    const url = fileUrl(runId, file);
    return '<a href="' + url + '" target="_blank" rel="noopener"><img src="' + url
      + '" loading="lazy" alt="' + escapeHtml(file.split('/').pop()) + '"></a>';
  }).join('') + '</div>' : '';
  return pictures + videos.map(file => '<video controls preload="metadata" src="' + fileUrl(runId, file) + '"></video>').join('');
}

function renderLiveResults(runId, operations, files, navigationOnly) {
  const results = operations.filter(op => navigationOnly || op.task_type !== 'navigation' || op.status === 'FAILED').reverse();
  const signature = JSON.stringify([runId, results, files]);
  if (signature === liveSignature) return results.length;
  liveSignature = signature;
  $('misLiveResults').innerHTML = results.map(op => {
    const [badge, label] = RESULT_BADGES[op.status] || ['', op.status || '—'];
    const result = op.result == null ? '' : typeof op.result === 'object'
      ? JSON.stringify(op.result, null, 2) : String(op.result);
    return '<section class="mission-live-result"><header><h3>' + escapeHtml(op.waypoint_id) + ' · '
      + escapeHtml(TASK_LABELS[op.task_type] || op.task_type)
      + '</h3><span class="badge ' + badge + '">' + escapeHtml(label) + '</span></header>'
      + (op.reason ? '<p class="muted">' + escapeHtml(op.reason) + '</p>' : '')
      + (result ? '<pre>' + escapeHtml(result) + '</pre>' : '')
      + liveMedia(runId, op, files) + '</section>';
  }).join('');
  return results.length;
}

async function refreshLive(state) {
  const runId = state?.run_id || '';
  if (runId !== liveRunId) {
    liveRunId = runId;
    liveDone = false;
    liveNextAt = 0;
    liveSignature = '';
    $('misLiveResults').replaceChildren();
    $('misLiveStatus').textContent = runId ? '等待首个任务结果' : '等待巡检任务开始';
  }
  if (!runId || liveDone || livePending || Date.now() < liveNextAt) return;
  livePending = true;
  liveNextAt = Date.now() + 1000;
  try {
    const { result, files } = await api('/api/mission/runs/' + encodeURIComponent(runId));
    if (!active || liveRunId !== runId) return;
    const count = renderLiveResults(runId, result.operations || [], files || [], state.navigation_only);
    liveDone = ['FINISH', 'ERROR'].includes(result.status);
    $('misLiveStatus').textContent = count ? '本次巡检 · ' + count + ' 条结果'
      : liveDone ? '本次巡检没有产生任务结果' : '等待首个任务结果';
  } catch (err) {
    if (active && liveRunId === runId) $('misLiveStatus').textContent = '等待车端写入结果…';
  } finally {
    livePending = false;
  }
}

// --- Runs --------------------------------------------------------------------

function fmtTime(iso) { return iso ? new Date(iso).toLocaleString() : '—'; }

async function loadRuns() {
  try {
    const { runs } = await api('/api/mission/runs');
    $('runsTbody').innerHTML = runs.map((r) => {
      const s = r.summary || {};
      const cls = r.status === 'FINISH' ? 'ok' : r.status === 'ERROR' ? 'err' : 'warn';
      return `<tr>
        <td><code>${escapeHtml(r.run_id)}</code></td>
        <td><span class="badge ${cls}">${escapeHtml(r.status)}</span>${r.error ? `<div class="muted clip" title="${escapeHtml(r.error)}">${escapeHtml(r.error)}</div>` : ''}</td>
        <td class="muted">${fmtTime(r.started_at)}</td>
        <td class="muted">${fmtDuration(r.duration_sec)}</td>
        <td>${s.completed_waypoints ?? '—'}</td>
        <td>${s.successful_tasks ?? '—'} / ${s.failed_tasks ?? '—'}</td>
        <td>${r.photo_count} / ${r.video_count}</td>
        <td class="row-actions">
          <button class="btn btn-ghost" data-run="${escapeHtml(r.run_id)}">查看</button>
          <a class="btn btn-ghost" href="/api/mission/runs/${encodeURIComponent(r.run_id)}/download">下载</a>
        </td></tr>`;
    }).join('') || '<tr><td colspan="8" class="muted">暂无巡检记录</td></tr>';
    for (const b of $('runsTbody').querySelectorAll('[data-run]')) b.addEventListener('click', () => showRun(b.dataset.run));
  } catch (err) {
    toast(`读取巡检记录失败：${err.message}`, 'err');
  }
}

function fileUrl(runId, path) {
  return `/api/mission/runs/${encodeURIComponent(runId)}/file?path=${encodeURIComponent(path)}`;
}

function briefResult(op) {
  if (op.reason) return escapeHtml(op.reason);
  if (op.result === null || op.result === undefined) return '';
  const text = typeof op.result === 'object' ? JSON.stringify(op.result) : String(op.result);
  return `<span class="clip" title="${escapeHtml(text)}">${escapeHtml(text)}</span>`;
}

async function showRun(runId) {
  try {
    const { result: d, files } = await api(`/api/mission/runs/${encodeURIComponent(runId)}`);
    $('runTitle').textContent = `记录 ${runId}`;
    const wps = (d.waypoints || []).map((w) => `<tr><td>${escapeHtml(w.id)}</td>
      <td><span class="badge ${w.status === 'SUCCESS' ? 'ok' : w.status === 'FAILED' ? 'err' : ''}">${escapeHtml(w.status)}</span></td>
      <td class="muted">${w.duration_sec != null ? `${w.duration_sec.toFixed(1)}s` : '—'}</td></tr>`).join('');
    const ops = (d.operations || []).filter((o) => o.task_type !== 'navigation' || o.status !== 'SUCCESS').map((o) => `<tr>
      <td>${escapeHtml(o.waypoint_id)}</td><td>${escapeHtml(o.task_type)}</td>
      <td><span class="badge ${o.status === 'FAILED' ? 'err' : o.status === 'SUCCESS' ? 'ok' : ''}">${escapeHtml(o.status)}</span></td>
      <td>${briefResult(o)}</td></tr>`).join('');
    const photos = files.filter((f) => /\.(jpe?g|png)$/i.test(f));
    const videos = files.filter((f) => /\.(mp4|avi|mkv)$/i.test(f));
    $('runBody').innerHTML = `
      <div class="kv run-kv">${kvRows([
        ['状态', escapeHtml(d.status)], ['开始', fmtTime(d.started_at)], ['结束', fmtTime(d.ended_at)],
        ['执行模式', d.task_only ? '仅任务（不导航）' : d.navigation_only ? '仅导航' : '导航与任务'],
        ['当前/最后航点', escapeHtml(d.current_waypoint || '—')], ['错误', escapeHtml(d.error || '无')],
      ])}</div>
      <div class="run-cols">
        <div><h3>航点</h3><table class="table compact"><tr><th>航点</th><th>状态</th><th>耗时</th></tr>${wps}</table></div>
        <div><h3>任务结果</h3><table class="table compact"><tr><th>航点</th><th>任务</th><th>状态</th><th>结果 / 原因</th></tr>${ops || '<tr><td colspan="4" class="muted">无</td></tr>'}</table></div>
      </div>
      <h3>照片（${photos.length}）</h3>
      <div class="thumbs big-thumbs">${photos.map((f) => `<a href="${fileUrl(runId, f)}" target="_blank"><img src="${fileUrl(runId, f)}" loading="lazy" alt="${escapeHtml(f)}"><span>${escapeHtml(f.split('/').pop())}</span></a>`).join('') || '<span class="muted">无</span>'}</div>
      <h3>视频（${videos.length}）</h3>
      ${videos.map((f) => `<div class="video-item"><video controls preload="metadata" src="${fileUrl(runId, f)}"></video><a href="${fileUrl(runId, f)}" download>${escapeHtml(f)}</a></div>`).join('') || '<span class="muted">无</span>'}
      <p><a class="btn btn-ghost" href="${fileUrl(runId, 'mission_results.json')}" target="_blank">mission_results.json</a>
         ${files.includes('mission.log') ? `<a class="btn btn-ghost" href="${fileUrl(runId, 'mission.log')}" target="_blank">mission.log</a>` : ''}</p>`;
    $('runDetail').hidden = false;
    $('runDetail').scrollIntoView({ behavior: 'smooth' });
  } catch (err) {
    toast(`读取记录失败：${err.message}`, 'err');
  }
}

$('runsRefresh').addEventListener('click', loadRuns);
$('runClose').addEventListener('click', () => { $('runDetail').hidden = true; });

on('state', snap => {
  lastSnapshot = snap;
  if (active) {
    renderMission(snap.mission);
    renderReadiness();
    refreshLive(snap.mission);
  }
});

on('view', name => {
  active = name === 'mission';
  if (name === 'mission-records') loadRuns();
  if (!active) return;
  renderMission(lastSnapshot?.mission);
  renderReadiness();
  refreshLive(lastSnapshot?.mission);
  if (!waypointsLoaded) loadWaypoints();
});
