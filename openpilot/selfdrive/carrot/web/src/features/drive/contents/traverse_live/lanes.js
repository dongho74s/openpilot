// Ported from the Traverse Live phone app (~/Documents/traverse-viz-app/web/src/lanes.js, 2026-11-05); keep in sync.
// Lane geometry helpers and the temporal lane filter (ported from drive_viz render.py / render2.py / drive5.py).

export function interp(x, xp, fp) {
  const n = xp.length;
  if (x <= xp[0]) return fp[0];
  if (x >= xp[n - 1]) return fp[n - 1];
  let lo = 0, hi = n - 1;
  while (hi - lo > 1) { const m = (lo + hi) >> 1; if (xp[m] <= x) lo = m; else hi = m; }
  const f = (x - xp[lo]) / (xp[hi] - xp[lo] || 1e-9);
  return fp[lo] + f * (fp[hi] - fp[lo]);
}

// model line (x forward, y right-positive) -> polyline [[x, yLeft], ...] extended 14 m behind the car
export function lineXY(xs, ys, xmax = 90) {
  const X = [], Y = [];
  for (let i = 0; i < xs.length; i++) if (xs[i] > -2 && xs[i] < xmax) { X.push(xs[i]); Y.push(ys[i]); }
  if (X.length < 2) return null;
  const x0 = Math.max(Math.min(...X), 0.5), x1 = Math.min(Math.max(...X), xmax);
  const pts = [], xi = [], yi = [];
  for (let k = 0; k < 120; k++) { const x = x0 + (x1 - x0) * k / 119; xi.push(x); yi.push(-interp(x, X, Y)); }
  const slope = (yi[3] - yi[0]) / (xi[3] - xi[0] + 1e-6);
  for (let k = 0; k < 12; k++) { const x = -14 + (xi[0] + 14) * k / 12; pts.push([x, yi[0] + slope * (x - xi[0])]); }
  for (let k = 0; k < 120; k++) pts.push([xi[k], yi[k]]);
  return pts;
}

export const XG = Array.from({ length: 105 }, (_, i) => -14 + i);   // -14 .. 90 m

export class LaneFilter {
  constructor() { this.y = {}; this.p = {}; this.t = null; this.typ = {}; }
  update(t, lines) {
    const dt = this.t === null ? 0.05 : Math.max(1e-3, t - this.t); this.t = t;
    const ap = 1 - Math.exp(-dt / 0.4);
    const out = {};
    for (const [k, [xy, prob]] of Object.entries(lines)) {
      if (!xy) {
        this.p[k] = (this.p[k] || 0) * (1 - ap);
        out[k] = [this.y[k] ? XG.map((x, i) => [x, this.y[k][i]]) : null, this.p[k]];
        continue;
      }
      const xs = xy.map((p) => p[0]), ys = xy.map((p) => p[1]);
      const ynew = XG.map((x) => interp(x, xs, ys));
      const cur = this.y[k];
      if (!cur || Math.abs(interp(5, XG, ynew) - interp(5, XG, cur)) > 1.2) this.y[k] = ynew;
      else for (let i = 0; i < XG.length; i++) {
        const tau = 0.20 + 0.55 * Math.min(1, Math.max(0, XG[i] / 80));
        cur[i] += (1 - Math.exp(-dt / tau)) * (ynew[i] - cur[i]);
      }
      const p0 = this.p[k] ?? prob;
      this.p[k] = p0 + ap * (prob - p0);
      out[k] = [XG.map((x, i) => [x, this.y[k][i]]), this.p[k]];
    }
    return out;
  }
  stableType(k, raw, t, hold = 0.35) {
    let [cur, cand, since] = this.typ[k] || [raw, raw, t];
    if (raw !== cand) { cand = raw; since = t; }
    if (cand !== cur && t - since >= hold) cur = cand;
    this.typ[k] = [cur, cand, since];
    return cur;
  }
}

// World-anchored 3 m dash / 6 m gap pattern cut at exact arc-length positions so dashes glide smoothly.
export function dashes(pts, dist, dash = 3.0, period = 9.0) {
  const s = [0];
  for (let i = 1; i < pts.length; i++) s.push(s[i - 1] + Math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]));
  const off = pts[0][0] + dist, out = [], total = s[s.length - 1];
  const px = pts.map((p) => p[0]), py = pts.map((p) => p[1]);
  for (let k = Math.floor(off / period) - 1; ; k++) {
    const a = k * period - off, b = a + dash;
    if (a > total) break;
    const a_ = Math.max(a, 0), b_ = Math.min(b, total);
    if (b_ - a_ <= 0.05) continue;
    const seg = [[interp(a_, s, px), interp(a_, s, py)]];
    for (let i = 0; i < s.length; i++) if (s[i] > a_ && s[i] < b_) seg.push(pts[i]);
    seg.push([interp(b_, s, px), interp(b_, s, py)]);
    out.push(seg);
  }
  return out;
}

// left/right offset polylines of a 2D centre line (normals from central differences)
export function ribbon(pts, width) {
  const n = pts.length, A = [], B = [];
  if (n < 2) return null;
  for (let i = 0; i < n; i++) {
    const p = pts[Math.max(0, i - 1)], q = pts[Math.min(n - 1, i + 1)];
    let dx = q[0] - p[0], dy = q[1] - p[1]; const L = Math.hypot(dx, dy) || 1; dx /= L; dy /= L;
    const nx = -dy * width / 2, ny = dx * width / 2;
    A.push([pts[i][0] + nx, pts[i][1] + ny]); B.push([pts[i][0] - nx, pts[i][1] - ny]);
  }
  return [A, B];
}
