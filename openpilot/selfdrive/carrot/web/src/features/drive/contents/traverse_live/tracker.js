// Ported from the Traverse Live phone app (~/Documents/traverse-viz-app/web/src/tracker.js, 2026-11-05); keep in sync.
// Object tracks with in-place fades (port of drive_viz drive3d.Tracker2, v7 behaviour).
// - eased fade in (0.5 s) / out (0.8 s)
// - while unseen: only longitudinal coasting at the relative speed fitted to this sensor ID's raw detections; never sideways
// - a nearby fading track is inherited when the sensor hands the same object a new ID (or radar <-> camera)
// - if the sensor re-uses an ID for a far-away object, the old track fades where it is and a new track starts

const fam = (t) => ([0, 1, 2].includes(t) ? 'veh' : 'vru');

function coastV(h) {
  if (h.length < 4 || h[h.length - 1][0] - h[0][0] < 0.25) return 0;
  const t0 = h[0][0]; let st = 0, sx = 0, stt = 0, stx = 0; const n = h.length;
  for (const [t, x] of h) { const u = t - t0; st += u; sx += x; stt += u * u; stx += u * x; }
  const den = n * stt - st * st;
  if (Math.abs(den) < 1e-9) return 0;
  return Math.max(-35, Math.min(35, (n * stx - st * sx) / den));
}

export class Tracker {
  constructor() { this.tr = new Map(); this.alias = new Map(); this.t = null; this.n = 0; }
  update(t, dets) {
    const dt = this.t === null ? 0.05 : Math.max(1e-3, t - this.t); this.t = t;
    const ax = 1 - Math.exp(-dt / 0.16), ay = 1 - Math.exp(-dt / 0.28);
    const FADE_IN = 0.5, FADE_OUT = 0.8;
    const seen = new Set();
    for (const d of dets) {
      let k = this.alias.get(d.key) ?? d.key;
      let tr = this.tr.get(k);
      if (tr && (Math.abs(tr.x - d.x) > Math.max(4, 0.15 * Math.abs(d.x)) || Math.abs(tr.y - d.y) > 1.5 + 0.02 * Math.abs(d.x))) {
        this.tr.delete(k); this.tr.set('old' + tr.uid, tr);
        for (const [a, b] of this.alias) if (b === k) this.alias.delete(a);
        k = d.key; tr = undefined;
      }
      if (!tr) {
        let best = null;
        for (const [kk, o] of this.tr) {
          if (seen.has(kk) || o.last >= t || fam(o.type) !== fam(d.type)) continue;
          const dx = Math.abs(o.x - d.x), dy = Math.abs(o.y - d.y);
          if (dx < Math.max(3, 0.12 * Math.abs(d.x)) && dy < 1.3 && (!best || dx + dy < best[0])) best = [dx + dy, kk];
        }
        if (best) { k = best[1]; this.alias.set(d.key, k); tr = this.tr.get(k); tr.h = []; }
      }
      if (!tr) {
        this.n += 1;
        tr = { x: d.x, y: d.y, w: d.w ?? 1.8, type: d.type, cand: d.type, since: t, last: t, vis: 0, uid: this.n, h: [] };
      } else {
        tr.x += ax * (d.x - tr.x); tr.y += ay * (d.y - tr.y);
        tr.w += ay * ((d.w || tr.w) - tr.w);
        if (d.type !== tr.cand) { tr.cand = d.type; tr.since = t; }
        if (tr.cand !== tr.type && t - tr.since >= 0.6) tr.type = tr.cand;
      }
      tr.last = t; tr.cipv = d.cipv; tr.oncoming = d.oncoming; tr.src = d.src; tr.coast = null;
      tr.h = tr.h.filter((p) => t - p[0] <= 0.6); tr.h.push([t, d.x, d.y]);
      this.tr.set(k, tr); seen.add(k);
    }
    const out = [];
    for (const [k, tr] of this.tr) {
      if (seen.has(k)) tr.vis = Math.min(1, tr.vis + dt / FADE_IN);
      else {
        if (tr.coast === null || tr.coast === undefined) tr.coast = coastV(tr.h);
        tr.vis -= dt / FADE_OUT; tr.x += tr.coast * dt; tr.cipv = false;
        if (tr.vis <= 0) { this.tr.delete(k); continue; }
      }
      const v = tr.vis;
      out.push({ ...tr, key: 't' + tr.uid, alpha: v * v * (3 - 2 * v) });
    }
    for (const [a, b] of this.alias) if (!this.tr.has(b)) this.alias.delete(a);
    return out;
  }
}
