// Traverse Live connection, ported from the phone app (~/Documents/traverse-viz-app/web/src/net.js, 2026-11-05).
// In the carrot web the page is served by the comma itself, so the socket goes to the same origin:
//   /ws/viz                      services at the app's cadence + CAN batches ("vizCan") on one socket (default)
//   /ws/raw_multiplex + /ws/viz_can   older comma builds without /ws/viz
// Payloads are decoded with the carrot web's own raw capnp decoder (globalThis.CarrotRawCapnp).
export const RAW_SERVICES = ['carState', 'selfdriveState', 'carControl', 'modelV2', 'radarState'];
const HUD = new Set(['carState', 'selfdriveState']);
const td = new TextDecoder();

export function sameOriginSocketBase(location = globalThis.location) {
  const secure = String(location?.protocol || '') === 'https:';
  return `${secure ? 'wss' : 'ws'}://${location?.host || ''}`;
}

// 'VC' v2 batch: u8 version, u16 count, then fixed 14-byte records (u8 bus, u16 addr, u16 age_ms, u8 len, data[8]).
export function decodeVizCan(u8) {
  if (!u8 || u8.length < 5 || u8[0] !== 0x56 || u8[1] !== 0x43) return null;
  const dv = new DataView(u8.buffer, u8.byteOffset, u8.byteLength);
  const ver = u8[2], count = dv.getUint16(3, true); let o = 5; const frames = [];
  for (let i = 0; i < count && o + 6 <= u8.length; i++) {
    const bus = u8[o], addr = dv.getUint16(o + 1, true), age = dv.getUint16(o + 3, true) / 1000, len = Math.min(u8[o + 5], ver >= 2 ? 8 : 64);
    frames.push({ bus, addr, age, d: u8.slice(o + 6, o + 6 + len) });
    o += ver >= 2 ? 14 : 6 + len;
  }
  return frames;
}

export class TraverseLiveLink {
  constructor(handlers, options = {}) {
    this.h = handlers; this.base = options.base || sameOriginSocketBase(); this.sockets = []; this.running = false;
    this.stat = { viz: 'idle' }; this.bytes = 0; this.legacy = false;
    this.decoder = options.decoder || (() => globalThis.CarrotRawCapnp);
    this.WebSocketCtor = options.WebSocket || globalThis.WebSocket;
    this.timers = new Set();
  }
  start() {
    this.stop(); this.running = true; this.legacy = false; this.stat = { viz: 'idle' }; this.open('viz');
  }
  stop() {
    this.running = false;
    for (const t of this.timers) clearTimeout(t);
    this.timers.clear();
    for (const s of this.sockets) { s.onclose = null; s.onmessage = null; try { s.close(); } catch (_) { /* ignore */ } }
    this.sockets = [];
  }
  open(kind) {
    if (!this.running) return;
    const path = kind === 'viz' ? '/ws/viz' : kind === 'raw' ? `/ws/raw_multiplex?services=${RAW_SERVICES.join(',')}` : '/ws/viz_can';
    let opened = false;
    const ws = new this.WebSocketCtor(`${this.base}${path}`);
    ws.binaryType = 'arraybuffer';
    this.stat[kind] = 'connecting'; this.h.status?.(this.stat);
    ws.onopen = () => { opened = true; this.stat[kind] = 'open'; this.h.status?.(this.stat); };
    ws.onmessage = (ev) => {
      if (typeof ev.data === 'string') return;            // hello
      const t = performance.now() / 1000; const u8 = new Uint8Array(ev.data); this.bytes += u8.length;
      if (kind === 'can') this.onCan(u8, t); else this.onRaw(u8, t);
    };
    ws.onclose = () => {
      this.stat[kind] = 'closed'; this.h.status?.(this.stat);
      this.sockets = this.sockets.filter((s) => s !== ws);
      if (!this.running) return;
      if (kind === 'viz' && !opened && !this.legacy) {        // endpoint missing on this comma build
        this.legacy = true; delete this.stat.viz; this.open('raw'); this.open('can'); return;
      }
      const timer = setTimeout(() => { this.timers.delete(timer); this.open(kind); }, 1000);
      this.timers.add(timer);
    };
    ws.onerror = () => { try { ws.close(); } catch (_) { /* ignore */ } };
    this.sockets.push(ws);
  }
  isOpen() {
    return this.legacy ? (this.stat.raw === 'open' && this.stat.can === 'open') : this.stat.viz === 'open';
  }
  onRaw(u8, t) {
    const n = u8[0]; const svc = td.decode(u8.subarray(1, 1 + n)); const payload = u8.subarray(1 + n);
    if (svc === 'vizCan') { this.onCan(payload, t); return; }
    const C = this.decoder();
    if (!C) return;
    let d = null;
    try { d = HUD.has(svc) ? C.decodeHudEvent(svc, payload) : C.decodeOverlayEvent(svc, payload); } catch (e) { return; }
    this.h.raw?.(svc, d, t);
  }
  onCan(u8, t) {
    const frames = decodeVizCan(u8);
    if (frames) this.h.can?.(frames, t);
  }
}
