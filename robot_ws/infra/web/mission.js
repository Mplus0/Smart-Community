// 巡检任务 view: start/stop mission.launch, live /mission/state, run browser.

import { api, escapeHtml, fmtDuration, kvRows, on, toast } from './common.js';

const $ = (id) => document.getElementById(id);
let active = false;
let waypointsLoaded = false;
let lastMissionState = null;

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

$('misScope').addEventListener('change', () => { $('misWaypoint').disabled = $('misScope').value !== 'single'; });

$('misStart').addEventListener('click', async () => {
  const single = $('misScope').value === 'single';
  if (single && !$('misWaypoint').value) { toast('请先选择航点（航点列表未加载）', 'err'); return; }
  const body = {
    waypoint: single ? $('misWaypoint').value : '',
    navigation_only: $('misNavOnly').checked,
    obstacle_mode: $('misObstacle').value,
  };
  try {
    await api('/api/mission/start', { method: 'POST', body });
    toast(`已启动${single ? `单航点 ${body.waypoint}` : '完整巡检'}${body.navigation_only ? '（仅导航）' : ''}`, 'ok');
  } catch (err) {
    toast(`启动失败：${err.message}`, 'err');
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
    ['模式', `${state.single_waypoint ? `单航点 ${escapeHtml(state.single_waypoint)}` : '完整巡检'}${state.navigation_only ? ' · 仅导航' : ''} · ${escapeHtml(state.obstacle_mode)}`],
    ['最近操作', op ? `${escapeHtml(op.waypoint_id)} ${escapeHtml(op.task_type)} <span class="badge ${op.status === 'FAILED' ? 'err' : op.status === 'SUCCESS' ? 'ok' : ''}">${escapeHtml(op.status)}</span> ${escapeHtml(op.reason || '')}` : '—'],
    ['run_id', `<code>${escapeHtml(state.run_id)}</code>`],
    ['更新', `${state.received_age_s}s 前`],
  ]);
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

on('state', (snap) => {
  const state = snap.mission;
  if (active) renderMission(state);
  // Refresh the run list when a mission ends.
  const prev = lastMissionState;
  lastMissionState = state ? `${state.run_id}:${state.state}` : null;
  if (active && state && prev !== lastMissionState && ['FINISH', 'ERROR'].includes(state.state)) loadRuns();
});

on('view', async (name) => {
  active = name === 'mission';
  if (!active) return;
  if (!waypointsLoaded) await loadWaypoints();
  loadRuns();
});
