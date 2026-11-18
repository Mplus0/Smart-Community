// Shared helpers: token-aware fetch / WebSocket, toasts, small DOM utils.

// Token: open the page once with ?token=XXX (stored in localStorage), or
// enter it when a request is rejected with 401.
const params = new URLSearchParams(window.location.search);
if (params.get('token') !== null) {
  localStorage.setItem('infraToken', params.get('token'));
  params.delete('token');
  const qs = params.toString();
  history.replaceState(null, '', window.location.pathname + (qs ? `?${qs}` : ''));
}
export function getToken() { return localStorage.getItem('infraToken') || ''; }

export async function api(path, { method = 'GET', body } = {}, retried = false) {
  const headers = {};
  const token = getToken();
  if (token) headers['X-Infra-Token'] = token;
  if (body !== undefined) headers['Content-Type'] = 'application/json';
  const r = await fetch(path, { method, headers, body: body === undefined ? undefined : JSON.stringify(body) });
  const data = await r.json().catch(() => ({}));
  if (!r.ok) {
    if (r.status === 401 && !retried) {
      const t = window.prompt('工作台启用了 Token，请输入：', token);
      if (t !== null) {
        localStorage.setItem('infraToken', t.trim());
        return api(path, { method, body }, true);
      }
    }
    const detail = data.detail !== undefined ? data.detail : r.statusText;
    throw new Error(typeof detail === 'string' ? detail : JSON.stringify(detail));
  }
  return data;
}

export function wsUrl(path) {
  const proto = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
  const token = getToken();
  const sep = path.includes('?') ? '&' : '?';
  return `${proto}//${window.location.host}${path}${token ? `${sep}token=${encodeURIComponent(token)}` : ''}`;
}

// Reconnecting JSON WebSocket. Returns { close(), send(text) }.
export function connectWithRetry(path, onMessage, { onOpen, onClose } = {}) {
  let backoff = 500;
  let ws = null;
  let closed = false;
  const open = () => {
    ws = new WebSocket(wsUrl(path));
    ws.addEventListener('open', () => { backoff = 500; if (onOpen) onOpen(); });
    ws.addEventListener('message', (ev) => {
      try { onMessage(JSON.parse(ev.data)); } catch (err) { console.warn('bad ws payload', err); }
    });
    ws.addEventListener('close', (ev) => {
      if (onClose) onClose(ev);
      if (closed) return;
      setTimeout(open, backoff);
      backoff = Math.min(backoff * 2, 5000);
    });
    ws.addEventListener('error', () => ws.close());
  };
  open();
  return {
    close() { closed = true; if (ws) ws.close(); },
    send(data) { if (ws && ws.readyState === WebSocket.OPEN) ws.send(data); },
  };
}

export function toast(text, kind = '') {
  const box = document.getElementById('toasts');
  const el = document.createElement('div');
  el.className = `toast ${kind}`;
  el.textContent = text;
  box.appendChild(el);
  setTimeout(() => el.remove(), kind === 'err' ? 7000 : 3500);
}

export function escapeHtml(s) {
  return String(s ?? '').replace(/[&<>"']/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
}

export function fmtDuration(s) {
  if (s === null || s === undefined) return '—';
  s = Math.max(0, Math.floor(s));
  const h = Math.floor(s / 3600); s %= 3600;
  const m = Math.floor(s / 60); s %= 60;
  if (h) return `${h}h${m}m`;
  if (m) return `${m}m${s}s`;
  return `${s}s`;
}

// Minimal event hub between modules (state snapshots, view switches, e-stop).
const listeners = {};
export function on(name, fn) { (listeners[name] = listeners[name] || []).push(fn); }
export function emit(name, value) { for (const fn of listeners[name] || []) fn(value); }

export function kvRows(rows) {
  return rows.map(([k, v]) => `<div class="kv-k">${escapeHtml(k)}</div><div class="kv-v">${v}</div>`).join('');
}
