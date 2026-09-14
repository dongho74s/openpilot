import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';

const source = fs.readFileSync(new URL('../app/src/main/assets/wayon_live.js', import.meta.url), 'utf8');
const start = source.indexOf('    function startHeartbeat(currentSocket) {');
const end = source.indexOf('    function suspendNativeRefresh', start);
assert.ok(start >= 0 && end > start);
const sent = [];
const currentSocket = {readyState: 1, send: value => sent.push(value)};
let heartbeat;
const context = vm.createContext({
  socket: currentSocket, currentSocket, WebSocket: {OPEN: 1}, Uint8Array,
  stopHeartbeat() {},
  setInterval(fn, delay) {assert.equal(delay, 3000); heartbeat = fn; return 1;},
  heartbeatTimer: 0, console,
});
vm.runInContext(source.slice(start, end) + '\nstartHeartbeat(currentSocket);', context);
heartbeat();
assert.equal(sent.length, 2);
for (const frame of sent) {
  // WayonDeviceRelay intentionally drops all client text frames.
  assert.ok(frame instanceof Uint8Array);
  assert.equal(Buffer.from(frame).toString('ascii'), 'WLP1');
}
currentSocket.readyState = 3;
heartbeat();
assert.equal(sent.length, 2);
currentSocket.readyState = 1;
context.socket = {readyState: 1};
heartbeat();
assert.equal(sent.length, 2);
console.log('Live heartbeat: binary framing, cadence and stale-socket guards passed');
