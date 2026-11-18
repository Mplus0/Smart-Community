// 相机 · 检测 view: one backend pipeline (source → detector → overlay → MJPEG).
// The stream <img> is only attached while the tab is open so the backend
// pipeline (and ONNX inference) stops when nobody watches.

import { api, escapeHtml, kvRows, on, toast } from './common.js';

const $ = (id) => document.getElementById(id);
let active = false;
let statusTimer = null;
let sourceTimer = null;

function showSources(sources, selected) {
  const select = $('camSource');
  const options = sources.map((x) => `<option value="${escapeHtml(x.id)}">${escapeHtml(x.label)}</option>`).join('')
    || '<option value="">暂无可用视频流</option>';
  if (select.dataset.options !== options) {
    select.innerHTML = options;
    select.dataset.options = options;
  }
  if (sources.length && !sources.some((x) => x.id === selected)) {
    if (!select.querySelector('option[value=""]')) {
      select.insertAdjacentHTML('afterbegin', '<option value="" disabled>请选择视频流</option>');
    }
    select.value = '';
  } else {
    select.value = selected;
  }
  select.disabled = !sources.length;
}

async function refreshSources() {
  try {
    const { sources } = await api('/api/camera/sources');
    if (active) showSources(sources, $('camSource').value);
  } catch { /* retry on next refresh */ }
}

async function loadOptions() {
  try {
    const [{ sources }, { detectors }, cfg] = await Promise.all([
      api('/api/camera/sources'), api('/api/camera/detectors'), api('/api/camera/config')]);
    const s = cfg.settings;
    showSources(sources, s.source);
    $('camDetector').innerHTML = detectors.map((d) => `<option value="${escapeHtml(d)}">${d === '__none__' ? '不检测（原始画面）' : d === '__trash_pipeline__' ? '垃圾分类完整流程' : escapeHtml(d)}</option>`).join('');
    $('camDetector').value = s.detector;
    $('camConf').value = s.conf ?? '';
    $('camFps').value = s.fps;
  } catch (err) {
    toast(`加载相机选项失败：${err.message}`, 'err');
  }
}

let pendingChange = Promise.resolve();

function applyChange(control, key, value) {
  if (!control.reportValidity()) return;
  pendingChange = pendingChange.then(async () => {
    try {
      await api('/api/camera/config', { method: 'POST', body: { [key]: value } });
    } catch (err) {
      toast(`切换失败：${err.message}`, 'err');
      await loadOptions();
    }
  });
}

let recording = false;
let captureBusy = false;
let lastPhoto = '';
for (const id of ['captureDir', 'captureName']) {
  $(id).value = localStorage.getItem(id) || '';
  $(id).addEventListener('change', () => localStorage.setItem(id, $(id).value));
}

async function collect(video) {
  if (captureBusy) return;
  captureBusy = true;
  $('captureVideo').disabled = true;
  try {
    await pendingChange;
    const stop = video && recording;
    const url = stop ? '/api/camera/record/stop' : '/api/camera/collect/' + (video ? 'video' : 'photo');
    const body = { directory: $('captureDir').value.trim(), name: $('captureName').value.trim() };
    const result = await api(url, { method: 'POST', body });
    if (!video) lastPhoto = result.path;
    toast((video && !stop ? '录像中：' : '已保存：') + result.path, 'ok');
  } catch (err) {
    toast(err.message, 'err');
  } finally {
    captureBusy = false;
    await pollStatus();
  }
}
$('capturePhoto').addEventListener('click', () => collect(false));
$('captureVideo').addEventListener('click', () => collect(true));

function renderStatus({ settings, status }) {
  recording = !!status.recording;
  $('captureVideo').disabled = captureBusy;
  $('captureVideo').textContent = recording ? '停止并保存录像' : '开始录像 AVI';
  $('captureStatus').textContent = [
    status.record_error && ('录像中断：' + status.record_error),
    status.video_path && ((recording ? '录像中：' : '最近录像：') + status.video_path),
    lastPhoto && ('最近图片：' + lastPhoto),
  ].filter(Boolean).join('  ·  ');
  $('camState').textContent = status.running ? '运行中' : '未运行';
  if (status.error) {
    $('camResult').innerHTML = `<div class="form-error">${escapeHtml(status.error)}</div>`;
    return;
  }
  if (settings.detector === '__none__') {
    $('camResult').textContent = '未选择检测器';
  } else if (status.report) {
    const r = status.report;
    $('camResult').innerHTML = kvRows([
      ['放置结果', escapeHtml(r.placement_status)],
      ['垃圾桶 / 垃圾', `${r.bin_count} / ${r.waste_count}`],
      ['匹配', escapeHtml((r.matches || []).map((m) => `${m.bin_label}:${m.status}`).join('，') || '无')],
    ]);
  } else {
    const dets = status.detections || [];
    $('camResult').innerHTML = dets.length
      ? `<table class="table compact"><tr><th>类别</th><th>置信度</th><th>框</th></tr>${dets.map((d) =>
        `<tr><td>${escapeHtml(d.label)}</td><td>${d.confidence.toFixed(2)}</td><td class="muted">${d.box.join(',')}</td></tr>`).join('')}</table>`
      : '当前画面无目标';
  }
}

async function pollStatus() {
  try { renderStatus(await api('/api/camera/config')); } catch { /* transient */ }
}

async function loadSnaps() {
  try {
    const { snaps } = await api('/api/camera/snaps?limit=8');
    $('camSnaps').innerHTML = snaps.map((s) =>
      `<a href="${s.url}" target="_blank" title="${escapeHtml(s.name)}"><img src="${s.url}" alt="${escapeHtml(s.name)}" loading="lazy"></a>`).join('')
      || '<span class="muted">暂无</span>';
  } catch { /* ignore */ }
}

for (const [id, key] of [['camSource', 'source'], ['camDetector', 'detector'], ['camConf', 'conf'], ['camFps', 'fps']]) {
  $(id).addEventListener('change', ({ target }) => {
    const value = key === 'conf' ? (target.value === '' ? 'auto' : Number(target.value))
      : key === 'fps' ? Number(target.value) : target.value;
    applyChange(target, key, value);
  });
}
$('camSnap').addEventListener('click', async () => {
  try {
    const res = await api('/api/camera/snapshot', { method: 'POST' });
    toast(`已保存 ${res.name}`, 'ok');
    loadSnaps();
  } catch (err) {
    toast(`快照失败：${err.message}`, 'err');
  }
});

on('view', async (name) => {
  const now = name === 'camera';
  if (now === active) return;
  active = now;
  if (active) {
    loadOptions();
    sourceTimer = setInterval(refreshSources, 5000);
    $('camStream').src = `/api/camera/stream.mjpeg?t=${Date.now()}`;
    pollStatus(); loadSnaps();
    statusTimer = setInterval(pollStatus, 1000);
  } else {
    // Dropping the src closes the HTTP stream; the pipeline stops with no client.
    $('camStream').src = 'data:image/gif;base64,R0lGODlhAQABAAAAACw=';
    clearInterval(sourceTimer);
    clearInterval(statusTimer); statusTimer = null;
  }
});
