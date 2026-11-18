// 相机 · 检测 view: one backend pipeline (source → detector → overlay → MJPEG).
// The stream <img> is only attached while the tab is open so the backend
// pipeline (and ONNX inference) stops when nobody watches.

import { api, escapeHtml, kvRows, on, toast } from './common.js';

const $ = (id) => document.getElementById(id);
let active = false;
let statusTimer = null;
let optionsLoaded = false;

async function loadOptions() {
  try {
    const [{ sources }, { detectors }, cfg] = await Promise.all([
      api('/api/camera/sources'), api('/api/camera/detectors'), api('/api/camera/config')]);
    const s = cfg.settings;
    $('camSource').innerHTML = sources.map((x) => `<option value="${escapeHtml(x.id)}">${escapeHtml(x.label)}</option>`).join('');
    if (!sources.some((x) => x.id === s.source)) {
      $('camSource').insertAdjacentHTML('afterbegin', `<option value="${escapeHtml(s.source)}">${escapeHtml(s.source)}</option>`);
    }
    $('camSource').value = s.source;
    $('camDetector').innerHTML = detectors.map((d) => `<option value="${escapeHtml(d)}">${d === '__none__' ? '不检测（原始画面）' : d === '__trash_pipeline__' ? '垃圾分类完整流程' : escapeHtml(d)}</option>`).join('');
    $('camDetector').value = s.detector;
    $('camConf').value = s.conf ?? '';
    $('camFps').value = s.fps;
    optionsLoaded = true;
  } catch (err) {
    toast(`加载相机选项失败：${err.message}`, 'err');
  }
}

async function apply() {
  const conf = $('camConf').value.trim();
  try {
    await api('/api/camera/config', {
      method: 'POST',
      body: { source: $('camSource').value, detector: $('camDetector').value,
        conf: conf === '' ? 'auto' : Number(conf), fps: Number($('camFps').value) || 10 },
    });
    toast('已切换，模型首次加载需几秒', 'ok');
  } catch (err) {
    toast(`切换失败：${err.message}`, 'err');
  }
}

function renderStatus({ settings, status }) {
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

$('camApply').addEventListener('click', apply);
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
    if (!optionsLoaded) await loadOptions();
    $('camStream').src = `/api/camera/stream.mjpeg?t=${Date.now()}`;
    pollStatus(); loadSnaps();
    statusTimer = setInterval(pollStatus, 1000);
  } else {
    // Dropping the src closes the HTTP stream; the pipeline stops with no client.
    $('camStream').src = 'data:image/gif;base64,R0lGODlhAQABAAAAACw=';
    clearInterval(statusTimer); statusTimer = null;
  }
});
