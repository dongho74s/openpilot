/* App updates are independent of the vehicle key and never invoke vehicle APIs. */
(() => {
  'use strict';
  const $ = id => document.getElementById(id);
  let state = {state: 'idle', currentVersion: '', message: '앱을 열면 새 버전을 자동으로 확인해요.'};
  function presentation(value) {
    const phase = value.state;
    const busy = ['checking', 'downloading', 'verifying', 'installing'].includes(phase);
    const action = phase === 'available' ? 'download' : ['ready', 'permission'].includes(phase) ? 'install' : 'check';
    const label = {checking: '확인 중…', downloading: '다운로드 중…', verifying: '검증 중…', installing: '설치 확인 중…',
      available: '업데이트 다운로드', ready: '업데이트 설치', permission: '설치 허용 후 계속', error: '다시 확인'}[phase] || '업데이트 확인';
    const progress = Number.isInteger(value.progress) ? Math.max(0, Math.min(100, value.progress)) : null;
    return {busy, action, label, progress, cancel: ['checking', 'downloading', 'verifying'].includes(phase)};
  }
  function render() {
    const view = presentation(state);
    const displayVersion = version => (version || '').replace(/-.*$/, '');
    const current = displayVersion(state.currentVersion);
    $('app-update-version').textContent = current ? '현재 ' + current : 'Hylink Dev';
    $('app-update-latest').textContent = state.hasUpdate && state.latestVersion ? '새 버전 ' + displayVersion(state.latestVersion) : '';
    $('app-update-status').textContent = state.message || '업데이트를 확인해 주세요.';
    $('app-update-button').textContent = view.label;
    $('app-update-button').disabled = view.busy;
    $('app-update-button').dataset.action = view.action;
    $('app-update-cancel').hidden = !view.cancel;
    $('app-update-progress-wrap').hidden = state.state !== 'downloading';
    if (view.progress === null) $('app-update-progress').removeAttribute('value');
    else $('app-update-progress').value = view.progress;
    $('app-update-percent').textContent = view.progress === null ? '준비 중' : view.progress + '%';
    $('app-update-notes').textContent = typeof state.notes === 'string' ? state.notes : '';
    $('app-update-details').hidden = !state.hasUpdate || !state.notes;
    $('app-update-card').dataset.state = state.state;
    $('app-update-badge').hidden = !state.hasUpdate;
    document.querySelector('.tab-bar [data-navigate="vehicle"]').setAttribute('aria-label', state.hasUpdate ? '차량 · 앱 업데이트 있음' : '차량');
  }
  window.onHylinkUpdateState = value => {
    if (!value || typeof value !== 'object') return;
    state = {...value};
    render();
  };
  $('app-update-button').addEventListener('click', () => {
    const method = {check: 'checkAppUpdate', download: 'downloadAppUpdate', install: 'installAppUpdate'}[presentation(state).action];
    if (typeof window.Android?.[method] === 'function') window.Android[method]();
    else $('app-update-status').textContent = '업데이트는 Android Hylink 앱에서 사용할 수 있어요.';
  });
  $('app-update-cancel').addEventListener('click', () => window.Android?.cancelAppUpdate?.());
  // Pure mapping also used by offline regression tests; no installer test hook.
  window.HylinkUpdatePresentation = presentation;
  render();
})();
