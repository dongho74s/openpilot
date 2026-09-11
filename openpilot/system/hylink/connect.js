'use strict';
const key = document.getElementById('key');
const copy = document.getElementById('copy');
const status = document.getElementById('status');
let pending = null;
let checking = false;

function resizeKey() {
  key.style.height = 'auto';
  key.style.height = key.scrollHeight + 'px';
}
function clearKey(message = '') {
  key.value = '';
  copy.disabled = true;
  copy.textContent = '키 복사';
  status.textContent = message;
}
async function connect() {
  if (pending) return;
  const controller = new AbortController();
  pending = controller;
  clearKey('키를 불러오는 중…');
  const timeout = setTimeout(() => controller.abort(), 25000);
  try {
    // Same-origin POST activates defaults; GET and browser prefetch never do.
    const response = await fetch('/api/connect', {
      method: 'POST', headers: {'Content-Type': 'application/json', 'X-Hylink-Request': '1'},
      cache: 'no-store', body: '{}', signal: controller.signal,
    });
    const data = await response.json();
    if (!response.ok) throw new Error(data.error || '연결하지 못했어요. 새로고침해 주세요.');
    if (!data.ready || !data.enabled || !/^wayon_[A-Za-z0-9_-]{32,128}$/.test(data.key)) {
      throw new Error('키를 준비하지 못했어요. 새로고침해 주세요.');
    }
    if (controller.signal.aborted) return;
    key.value = data.key;
    copy.disabled = false;
    status.textContent = '';
    resizeKey();
  } catch (error) {
    if (pending === controller) clearKey(error.name === 'AbortError' || error instanceof TypeError
      ? '연결을 확인하고 새로고침해 주세요.' : error.message);
  } finally {
    clearTimeout(timeout);
    if (pending === controller) pending = null;
  }
}
copy.onclick = async () => {
  if (!key.value || copy.disabled) return;
  try {
    if (navigator.clipboard && window.isSecureContext) await navigator.clipboard.writeText(key.value);
    else { key.select(); if (!document.execCommand('copy')) throw new Error('copy'); }
    copy.textContent = '복사됨';
    status.textContent = '키를 복사했어요.';
  } catch {
    key.select();
    status.textContent = '키를 길게 눌러 복사해 주세요.';
  }
};
window.addEventListener('resize', resizeKey);
window.addEventListener('pagehide', () => {
  const request = pending;
  pending = null;
  request?.abort();
  clearKey();
});
window.addEventListener('pageshow', event => { if (event.persisted) connect(); });
connect();
// Read only: a stopped connection stays stopped until the page is reopened.
setInterval(async () => {
  if (document.hidden || !key.value || pending || checking) return;
  checking = true;
  try {
    const response = await fetch('/api/key', {
      headers: {'X-Hylink-Request': '1'}, cache: 'no-store', signal: AbortSignal.timeout(2500),
    });
    const data = await response.json();
    if (!response.ok || !data.ready || !data.enabled || data.key !== key.value) throw new Error('closed');
  } catch {
    clearKey('연결이 종료됐어요. 시동을 끄고 같은 Wi-Fi에서 새로고침해 주세요.');
  } finally { checking = false; }
}, 5000);
