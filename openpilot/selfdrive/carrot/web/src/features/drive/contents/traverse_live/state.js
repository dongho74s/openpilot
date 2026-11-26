// Ported from the Traverse Live phone app (~/Documents/traverse-viz-app/web/src/state.js, 2026-11-05); keep in sync.
// Live vehicle state fed by the comma streams, and the GM camera/radar CAN decoding (ported from drive_viz render.py / render2.py).
// World frame everywhere: x forward (m, from the comma camera), y left, z up.

const GEAR = { park: 'P', drive: 'D', reverse: 'R', neutral: 'N', sport: 'D', low: 'D', manumatic: 'D', eco: 'D', brake: 'D' };

// The comma forwards CAN in 100 ms batches: a frame can be up to one batch older when it is drawn than on the car.
const CAN_BATCH_SLACK = 0.12;

export function signed(v, bits) { return (v & (1 << (bits - 1))) ? v - (1 << bits) : v; }

export class LiveState {
  constructor() {
    this.cs = {}; this.sd = {}; this.cc = {}; this.md = null; this.rs = {};
    this.can = new Map();                 // (bus << 16 | addr) -> { t, d }
    this.turnSeen = [0, -99];             // [code, t] last 0x427 turn code >= 2
    this.dist = 0; this.lastT = null;
    this.rx = { raw: -99, can: -99, model: -99 };
  }
  onCarState(c, t) {
    if (!c) return;
    this.cs = {
      v: c.vEgo ?? 0, a: c.aEgo ?? 0, lb: !!c.leftBlinker, rb: !!c.rightBlinker,
      lbs: !!c.leftBlindspot, rbs: !!c.rightBlindspot, brake: !!c.brakePressed,
      gear: GEAR[String(c.gearShifter || '').toLowerCase()] || '-',
      cen: !!c.cruiseState?.enabled, cspdKph: Number(c.vCruiseCluster) || 0,
      // Carrot web 3D view extras
      sa: Number.isFinite(Number(c.steeringAngleDeg)) ? Number(c.steeringAngleDeg) : 0,
      gas: !!c.gasPressed, steerPressed: !!c.steeringPressed, standstill: !!c.standstill,
      // stock camera own-lane markings as card publishes them for Navdy: [left, right], -1 unknown / 0 dashed / 1 solid
      slm: c.stockLaneMarkings?.supported ? [c.stockLaneMarkings.left, c.stockLaneMarkings.right] : null,
    };
    this.rx.raw = t;
  }
  onSelfdrive(s) {
    if (!s) return;
    const personality = Number.isFinite(Number(s.personality)) ? Number(s.personality) : null;
    this.sd = { a1: s.alertText1 || '', a2: s.alertText2 || '', en: !!s.enabled, state: String(s.state || ''),
      exp: !!s.experimentalMode, personality, alertStatus: Number(s.alertStatus) || 0 };
  }
  onCarControl(c) { if (c) this.cc = { lat: !!c.latActive, lon: !!c.longActive }; }
  onRadar(r) {
    const l = r?.leadOne;
    this.rs = l ? { st: !!l.status, d: l.dRel ?? 0, vr: l.vRel ?? 0, vl: Number.isFinite(l.vLead) ? l.vLead : null, radar: !!l.radar,
      y: Number.isFinite(l.yRel) ? l.yRel : 0 } : {};
  }
  onModel(m, t) {
    if (!m || !Array.isArray(m.laneLines) || m.laneLines.length !== 4) return;
    const xy = (l) => [Array.from(l?.x || []), Array.from(l?.y || [])];
    this.md = {
      ll: m.laneLines.map(xy), lp: Array.from(m.laneLineProbs || [0, 0, 0, 0]),
      re: (m.roadEdges || []).map(xy), path: xy(m.position),
    };
    this.rx.model = t;
  }
  onCanFrames(frames, t) {
    for (const f of frames) {
      const ft = t - f.age;
      this.can.set((f.bus << 16) | f.addr, { t: ft, d: f.d });
      if (f.bus === 1 && f.addr === 0x427 && f.d.length === 8) {
        const code = ((f.d[3] & 3) << 1) | (f.d[4] >> 7);
        if (code >= 2) this.turnSeen = [code, ft];
      }
    }
    this.rx.can = t;
  }
  advance(t) {
    if (this.lastT !== null) this.dist += (this.cs.v || 0) * Math.max(0, Math.min(0.2, t - this.lastT));
    this.lastT = t;
  }
  frame(bus, addr, t, maxAge = 0.3) {
    const x = this.can.get((bus << 16) | addr);
    return x && t - x.t <= maxAge + CAN_BATCH_SLACK ? x.d : null;
  }
}

// ---------- GM front camera objects (F_Vision_Obj_Track, bus 1) ----------
export function visionObjects(st, t) {
  const objs = []; let cipv = 0;
  const h = st.frame(1, 0x420, t);
  if (h) cipv = (h[3] >> 1) & 0x3F;
  const addrs = [0x421, 0x422, 0x423, 0x424, 0x425, 0x426, 0x441, 0x442, 0x443, 0x444, 0x445, 0x446];
  for (const a of addrs) {
    const d = st.frame(1, a, t, 0.15);
    if (!d || d.length !== 8) continue;
    const oid = d[0] >> 2;
    if (oid === 0) continue;
    const typ = (d[1] >> 3) & 0xF;
    const az = signed(((d[1] & 7) << 7) | (d[2] >> 1), 10) * 0.1 * Math.PI / 180;
    const rng = (((d[2] & 1) << 11) | (d[3] << 3) | (d[4] >> 5)) * 0.1;
    if (rng < 1) continue;
    objs.push({ id: oid, type: [1, 2, 3, 4].includes(typ) ? typ : 1, x: rng * Math.cos(az), y: rng * Math.sin(az),
      w: (d[7] & 0x3F) * 0.25, cipv: oid === cipv && a === 0x421, oncoming: (d[1] >> 7) === 0 });
  }
  return objs;
}

// camera objects + moving radar targets that the camera does not already cover
export function detections(st, t) {
  const vis = visionObjects(st, t);
  const dets = vis.map((o) => ({ key: 'v' + o.id, src: 'v', type: o.type, x: o.x, y: o.y, w: o.w, cipv: o.cipv, oncoming: o.oncoming }));
  for (let a = 0x461; a < 0x475; a++) {
    const d = st.frame(1, a, t, 0.25);
    if (!d || d.length !== 8) continue;
    const meas = d[0] >> 6, oid = d[7] & 0x3F;
    if (meas === 0 || oid === 0) continue;
    const rng = (((d[0] & 0x3F) << 5) | (d[1] >> 3)) * 0.125;
    const dyn = (d[4] >> 4) & 7, conf = d[6] & 3;
    const az = signed(((d[4] & 0xF) << 8) | d[5], 12) * 0.125 * Math.PI / 180;
    if (rng > 140 || conf < 2 || ![2, 3, 4].includes(dyn)) continue;
    const x = rng * Math.cos(az) + 1.6, y = rng * Math.sin(az);
    if (vis.some((o) => Math.abs(o.x - x) < 5 && Math.abs(o.y - y) < 1.8)) continue;
    dets.push({ key: 'r' + oid, src: 'r', type: 0, x, y, w: 1.8, cipv: false, oncoming: dyn === 4 });
  }
  return dets;
}

// stock camera lane marking types for [left-left, left, right, right-right] (0x352/0x350/0x354/0x355)
export function laneTypes(st, t) {
  const d352 = st.frame(1, 0x352, t), d350 = st.frame(1, 0x350, t);
  let lt = d352 ? (d352[6] >> 3) & 7 : 0, rt = d352 ? d352[6] & 7 : 0;
  if (d350) { if ((d350[0] >> 2) & 1) lt = 0; if (d350[0] & 1) rt = 0; }
  // own lane: same source as Navdy (carState.stockLaneMarkings) when it knows; CAN code otherwise (also keeps other codes)
  const slm = st.cs.slm, fromCs = (v, raw) => (v === 1 ? 1 : v === 0 ? 2 : raw);
  if (slm) { lt = fromCs(slm[0], lt); rt = fromCs(slm[1], rt); }
  const d354 = st.frame(1, 0x354, t), d355 = st.frame(1, 0x355, t);
  return [d354 ? (d354[0] >> 2) & 7 : 0, lt, rt, d355 ? (d355[0] >> 2) & 7 : 0];
}

export function leadLights(st, t) {
  const d427 = st.frame(1, 0x427, t);
  const brake = d427 ? ((d427[4] >> 5) & 3) === 2 : false;
  const [code, last] = st.turnSeen;
  return { brake, turn: t - last < 1.2 ? code : 0 };
}
