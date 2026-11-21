// 相机 · 检测 view: one backend pipeline (source → detector → overlay → MJPEG).
// The stream <img> is only attached while the tab is open so the backend
// pipeline (and ONNX inference) stops when nobody watches.

import { api, escapeHtml, kvRows, on, toast } from './common.js';

const $ = (id) => document.getElementById(id);
let active = false;
let statusTimer = null;
let sourceTimer = null;
let availableSources = [];
let selectedSource = '';

const detectorLabels = {
  __none__: '不检测（原始画面）',
  __trash_pipeline__: '垃圾分类完整流程',
  __thermal__: '红外测温（楼宇 D）',
  __meter__: '仪表盘识别（UGREEN 4K）',
  __meter_debug__: '仪表盘调试（实时过程图）',
  __plate__: '车牌识别',
  __building_fire__: '楼宇火灾识别',
};

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
    if (active) {
      availableSources = sources;
      showSources(availableSources, selectedSource);
    }
  } catch { /* retry on next refresh */ }
}

function syncSelection(settings) {
  selectedSource = settings.source;
  showSources(availableSources, selectedSource);
  $('camDetector').value = settings.detector;
  $('camConf').disabled = ['__thermal__', '__meter__', '__meter_debug__', '__plate__'].includes(settings.detector);
}

async function loadOptions() {
  try {
    const [{ sources }, { detectors }, cfg] = await Promise.all([
      api('/api/camera/sources'), api('/api/camera/detectors'), api('/api/camera/config')]);
    const s = cfg.settings;
    availableSources = sources;
    $('camDetector').innerHTML = detectors.map((d) =>
      `<option value="${escapeHtml(d)}">${escapeHtml(detectorLabels[d] ?? d)}</option>`).join('');
    syncSelection(s);
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
      const { settings } = await api('/api/camera/config', { method: 'POST', body: { [key]: value } });
      syncSelection(settings);
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
  $('camState').textContent = status.error ? '相机未就绪'
    : status.running ? '相机预览中' : settings.source ? '等待相机画面' : '请选择相机源';
  if (status.error) {
    $('camResult').innerHTML = `<div class="form-error">${escapeHtml(status.error)}</div>`;
    return;
  }
  if (settings.detector === '__none__') {
    $('camResult').textContent = '未选择检测器';
  } else if (settings.detector === '__thermal__') {
    const message = status.report?.kind === 'thermal' ? status.report.message : '';
    if (message) {
      $('camResult').innerHTML = message.split('\n').map((line) => `<div>${escapeHtml(line)}</div>`).join('');
    } else {
      $('camResult').textContent = '等待楼宇 D 测温结果';
    }
  } else if (['__meter__', '__meter_debug__'].includes(settings.detector)) {
    const r = status.report;
    if (r?.kind !== 'meter') {
      $('camResult').textContent = '等待读表结果';
      return;
    }
    let alignment = r.alignment === 'tracking' ? '跟踪中' : '已定位';
    if (r.alignment === 'alignment_failed') alignment = '未可靠定位表盘';
    else if (r.temperature_c == null || r.humidity_percent == null) alignment += '，部分指针未读出';
    $('camResult').innerHTML = kvRows([
      ['表盘状态', alignment],
      ...(r.inlier_count == null ? [] : [['匹配内点', escapeHtml(r.inlier_count)]]),
      ['温度', r.temperature_c == null ? '—' : `${escapeHtml(r.temperature_c)} ℃`],
      ['湿度', r.humidity_percent == null ? '—' : `${escapeHtml(r.humidity_percent)} %RH`],
    ]);
  } else if (settings.detector === '__plate__') {
    const r = status.report;
    if (r?.kind !== 'plate' || r.pending) {
      $('camResult').textContent = '正在识别车牌';
      return;
    }
    if (r.ocr_error) {
      $('camResult').innerHTML = `<div class="form-error">车牌 OCR 不可用：${escapeHtml(r.ocr_error)}</div>`;
      return;
    }
    $('camResult').innerHTML = r.plate ? kvRows([
      ['车牌号码', escapeHtml(r.plate)],
      ['OCR 置信度', Number.isFinite(r.confidence) ? `${(100 * r.confidence).toFixed(1)}%` : '—'],
    ]) : '未识别到车牌';
  } else if (settings.detector === '__building_fire__') {
    const r = status.report;
    if (r?.kind !== 'building_fire') {
      $('camResult').textContent = '等待火焰检测结果';
      return;
    }
    if (!r.fire_count) {
      $('camResult').textContent = '当前画面未检测到火焰';
      return;
    }
    const building = r.building || (r.ocr_error ? '楼名 OCR 不可用' : r.pending ? '正在识别楼名' : '未识别到楼名');
    $('camResult').innerHTML = kvRows([
      ['火情', '发现火焰'],
      ['火焰数量', escapeHtml(r.fire_count)],
      ['楼名', escapeHtml(building)],
      ...(r.ocr_error ? [['OCR 状态', escapeHtml(r.ocr_error)]] : []),
    ]);
  } else if (settings.detector === '__trash_pipeline__' && status.report) {
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
