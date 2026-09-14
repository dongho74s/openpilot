// Read-only actual WebView observation. Open live in the app before running.
// Never log URLs, headers, protocols, frame payloads, keys or coordinates.
const duration = Math.min(360, Math.max(10, Number(process.argv[2]) || 120));
const targets = await (await fetch('http://127.0.0.1:9339/json/list')).json();
const target = targets.find(t => t.url === 'file:///android_asset/main.html');
if (!target) throw Error('Hylink WebView not found');
const ws = new WebSocket(target.webSocketDebuggerUrl), pending = new Map();
let seq = 0;
const started = Date.now();
const age = () => Math.round((Date.now() - started) / 10) / 100;
await new Promise((resolve, reject) => { ws.onopen = resolve; ws.onerror = reject; });
ws.onclose = () => {
  for (const p of pending.values()) p.reject(Error('Diagnostic USB/CDP connection closed'));
  pending.clear();
};
ws.onmessage = e => {
  const v = JSON.parse(e.data);
  if (v.id) { const p = pending.get(v.id); pending.delete(v.id); v.error ? p.reject(v.error) : p.resolve(v.result); return; }
  if (v.method === 'Network.webSocketClosed') console.log(JSON.stringify({elapsed_s: age(), event: 'websocket_closed'}));
  if (v.method === 'Network.webSocketFrameError') console.log(JSON.stringify({elapsed_s: age(), event: 'websocket_frame_error'}));
  if (['Network.webSocketFrameReceived', 'Network.webSocketFrameSent'].includes(v.method) && v.params.response.opcode === 8) {
    console.log(JSON.stringify({elapsed_s: age(), event: v.method.endsWith('Sent') ? 'close_sent' : 'close_received'}));
  }
};
const send = (method, params = {}) => new Promise((resolve, reject) => {
  const id = ++seq; pending.set(id, {resolve, reject}); ws.send(JSON.stringify({id, method, params}));
});
try {
  await send('Network.enable');
  while (age() < duration) {
    const state = await send('Runtime.evaluate', {returnByValue: true, expression: `({status:document.getElementById('wayon-live-status-text').textContent,active:document.getElementById('wayon-live-overlay').classList.contains('active'),visible:document.visibilityState})`});
    console.log(JSON.stringify({elapsed_s: age(), ...state.result.value}));
    await new Promise(resolve => setTimeout(resolve, Math.min(10000, Math.max(1, (duration - age()) * 1000))));
  }
} finally { if (ws.readyState === WebSocket.OPEN) await send('Network.disable'); ws.close(); }
