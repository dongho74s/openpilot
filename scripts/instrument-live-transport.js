// Temporary real-socket event observer; no URLs, protocols, payloads or credentials.
// Reloading/restarting the WebView removes it. Does not synthesize network events.
(() => {
  if (window.__hylinkTransportDiagnostic) return;
  const Original = window.WebSocket;
  const events = [];
  let sequence = 0;
  const record = (event, fields = {}) => {
    const item = {at: Date.now(), event, ...fields};
    events.push(item);
    if (events.length > 64) events.shift();
    console.info('HYLINK_TRANSPORT ' + JSON.stringify(item));
  };
  window.__hylinkTransportDiagnostic = {events};
  window.WebSocket = class extends Original {
    constructor(...args) {
      super(...args);
      if (new URL(args[0]).pathname !== '/api/live/stream') return;
      const id = ++sequence;
      record('create', {id});
      this.addEventListener('open', () => record('open', {id}));
      this.addEventListener('error', () => record('error', {id}));
      this.addEventListener('close', e => record('close', {id, code: e.code, clean: e.wasClean}));
    }
  };
  window.addEventListener('online', () => record('online'));
  window.addEventListener('offline', () => record('offline'));
  record('observer_ready', {online: navigator.onLine});
})();
