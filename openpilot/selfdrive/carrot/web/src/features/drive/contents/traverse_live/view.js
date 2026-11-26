// Traverse Live view for the carrot web drive tab: the phone app's live scene (road, lanes, nearby cars, brake lights,
// turn signals) drawn by the viewing device from the comma's /ws/viz stream. Ported from
// ~/Documents/traverse-viz-app/web/src/main.js (2026-11-05): same frame build, same scene, same HUD.
// Loaded lazily (js/generated/traverse-live.js) so three.js is only downloaded when someone starts it.
import { LiveState, detections, laneTypes, leadLights } from './state.js';
import { LaneFilter, lineXY } from './lanes.js';
import { Tracker } from './tracker.js';
import { DriveScene } from './scene.js';
import { Hud } from './hud.js';
import { TraverseLiveLink } from './link.js';

export const STYLE_ID = 'carrot-traverse-live-style';

// Phone app look (web/index.html), scoped to the view and sized by --ctl-u (720 units for the shorter side).
export const CSS = `
.ctl-view{position:absolute;inset:0;overflow:hidden;color:#eef1f4;background:linear-gradient(#06080a,#16191c);
  --ctl-u:1px;font-family:"Wanted Sans Variable","Pretendard","Noto Sans KR","Apple SD Gothic Neo",system-ui,sans-serif;
  -webkit-user-select:none;user-select:none;color-scheme:dark}
.ctl-canvas{position:absolute;inset:0;width:100%;height:100%;display:block}
.ctl-hud{position:absolute;inset:0;pointer-events:none}
.ctl-hud [hidden],.ctl-view [hidden]{display:none!important}
.ctl-tl{position:absolute;left:calc(40 * var(--ctl-u));top:calc(18 * var(--ctl-u))}
.ctl-speed-row{display:flex;align-items:flex-start;gap:calc(18 * var(--ctl-u))}
.ctl-speed{font-size:calc(74 * var(--ctl-u));font-weight:200;line-height:1;letter-spacing:-.02em;font-variant-numeric:tabular-nums}
.ctl-unit{font-size:calc(16 * var(--ctl-u));color:#848c94;margin:calc(4 * var(--ctl-u)) 0 0 calc(4 * var(--ctl-u))}
.ctl-gears{margin-top:calc(16 * var(--ctl-u));font-size:calc(17 * var(--ctl-u));font-weight:500;letter-spacing:calc(9 * var(--ctl-u))}
.ctl-gear{color:#464c52}.ctl-gear.is-current{color:#eef1f4}
.ctl-row{display:flex;gap:calc(8 * var(--ctl-u));margin-top:calc(12 * var(--ctl-u));align-items:center}
.ctl-pill{font-size:calc(17 * var(--ctl-u));font-weight:500;padding:calc(6 * var(--ctl-u)) calc(14 * var(--ctl-u));border-radius:999px;white-space:pre}
.ctl-pill--on{background:rgba(48,136,255,.92);color:#fff}.ctl-pill--wait{background:rgba(54,60,66,.9)}
.ctl-pill--off{background:rgba(34,38,42,.86);color:#848c94}.ctl-pill--set{background:rgba(34,38,42,.86)}
.ctl-lead{font-size:calc(15 * var(--ctl-u));color:#848c94;margin-top:calc(14 * var(--ctl-u));white-space:pre}
.ctl-badges{margin-top:calc(8 * var(--ctl-u));display:flex;gap:calc(8 * var(--ctl-u))}
.ctl-badge{font-size:calc(15 * var(--ctl-u));padding:calc(4 * var(--ctl-u)) calc(11 * var(--ctl-u));border-radius:999px}
.ctl-badge--brake{background:rgba(210,36,36,.92);color:#fff}
.ctl-badge--turn{background:rgba(255,172,0,.94);color:#181000}.ctl-badge--turn.is-dim{background:rgba(130,96,20,.78)}
.ctl-tr{position:absolute;right:calc(40 * var(--ctl-u) + var(--ctl-tr-gap, 0px));top:calc(30 * var(--ctl-u));font-size:calc(15 * var(--ctl-u));color:#848c94;white-space:pre}
.ctl-alert{position:absolute;left:50%;transform:translateX(-50%);top:calc(26 * var(--ctl-u));font-size:calc(17 * var(--ctl-u));
  background:rgba(36,40,44,.92);padding:calc(7 * var(--ctl-u)) calc(16 * var(--ctl-u));border-radius:999px;white-space:nowrap}
.ctl-caption{position:absolute;left:calc(40 * var(--ctl-u));bottom:calc(16 * var(--ctl-u));font-size:calc(13 * var(--ctl-u));color:#464c52}
.ctl-conn{position:absolute;right:calc(14 * var(--ctl-u));bottom:calc(12 * var(--ctl-u));font-size:calc(12 * var(--ctl-u));color:#5a6168;
  display:flex;align-items:center;gap:6px;pointer-events:none;white-space:nowrap}
.ctl-conn::before{content:"";width:7px;height:7px;border-radius:50%;background:#a05a1e}
.ctl-conn.is-live::before{background:#2fae5a}.ctl-conn.is-idle::before{background:#3088ff}
.ctl-idle{position:absolute;left:50%;top:24%;transform:translate(-50%,-50%);text-align:center;color:#8a929a;
  font-size:calc(17 * var(--ctl-u));line-height:1.6;pointer-events:none}
.ctl-idle b{color:#eef1f4;font-weight:600}
`;

const TEXT = {
  ko: { connecting: '연결 중…', receiving: '수신 중', open: '연결됨 · 시동을 켜면 표시', noCan: ' (CAN 없음)',
    idleTitle: '콤마에 연결됨', idleBody: '시동을 켜면 주행 화면이 나옵니다' },
  en: { connecting: 'Connecting…', receiving: 'Receiving', open: 'Connected · shows when the car is on', noCan: ' (no CAN)',
    idleTitle: 'Connected to the comma', idleBody: 'The drive view appears when the car is on' },
  zh: { connecting: '正在连接…', receiving: '接收中', open: '已连接 · 车辆启动后显示', noCan: '（无 CAN）',
    idleTitle: '已连接 comma', idleBody: '车辆启动后显示行驶画面' },
};

function viewText(doc) {
  const code = String(doc.documentElement?.lang || 'ko').toLowerCase().split(/[-_]/)[0];
  return TEXT[code] || TEXT.en;
}

export function injectTraverseLiveStyle(doc) {
  if (!doc || doc.getElementById(STYLE_ID)) return;
  const style = doc.createElement('style');
  style.id = STYLE_ID;
  style.textContent = CSS;
  (doc.head || doc.documentElement).appendChild(style);
}

// One scene frame from the live state: drive_viz drive5.frame (unchanged from the app).
export function buildFrame(st, LF, TRK, t) {
  const cs = st.cs, md = st.md;
  const fr = { lines: [], edges: [], objs: [], dist: st.dist, bsm: [!!cs.lbs, !!cs.rbs] };
  const raw = laneTypes(st, t);
  const types = raw.map((x, i) => LF.stableType(i, x, t));
  if (md) {
    const lines = {};
    for (let i = 0; i < 4; i++) lines['l' + i] = [lineXY(md.ll[i][0], md.ll[i][1]), md.lp[i] ?? 0];
    md.re.forEach((e, j) => { lines['e' + j] = [lineXY(e[0], e[1]), 1.0]; });
    lines.path = [lineXY(md.path[0], md.path[1], 70), 1.0];
    const fl = LF.update(t, lines);
    const ll = [0, 1, 2, 3].map((i) => fl['l' + i][0]), lp = [0, 1, 2, 3].map((i) => fl['l' + i][1]);
    const re = ['e0', 'e1'].filter((k) => k in fl).map((k) => fl[k][0]);
    const L = re[0] || ll[0], R = re[1] || ll[3];
    if (L && R) fr.road = { l: L, r: R };
    for (let i = 0; i < 4; i++) {
      let typ = types[i];
      if ((i === 0 || i === 3) && typ === 0) typ = lp[i] > 0.5 ? -1 : 0;
      let prob = typ !== 0 ? lp[i] : lp[i] * 0.4;
      if ((i === 1 || i === 2) && (typ === 1 || typ === 2)) prob = Math.max(prob, 0.6);
      fr.lines.push({ pts: ll[i], typ: typ >= 0 ? typ : 9, prob });
    }
    for (const e of re) if (e) fr.edges.push(e);
    if (fl.path[0]) fr.path = { pts: fl.path[0], on: !!st.cc.lat };
  }
  const lights = leadLights(st, t);
  const blink = (t * 1.5) % 1.0 < 0.55;
  for (const o of TRK.update(t, detections(st, t))) {
    if (!(o.x > -6 && o.x < 150)) continue;
    const typ = o.type, lead = !!o.cipv;
    const model = { 2: 'truck', 3: 'moto', 4: 'ped' }[typ] || ((o.w || 1.8) > 1.86 ? 'suv' : 'sedan');
    fr.objs.push({ k: o.key, m: model, x: o.x, y: o.y, f: o.oncoming && [0, 1, 2].includes(typ) ? 1 : 0, a: o.alpha, lead, r: typ === 0,
      br: lead && lights.brake, tl: lead && blink && [2, 4].includes(lights.turn), tr: lead && blink && [3, 4].includes(lights.turn),
      ph: t * 6.5 + (o.uid % 7) });
  }
  const eb = !!(cs.brake || (st.cc.lon && (cs.a || 0) < -0.8));
  fr.ego = { br: eb, tl: blink && !!cs.lb, tr: blink && !!cs.rb };
  return { fr, lights, blink };
}

export function createTraverseLiveView(options = {}) {
  const root = options.root;
  if (!root) throw new TypeError('Traverse Live needs a root element');
  const doc = root.ownerDocument;
  const target = options.target || doc.defaultView || globalThis;
  injectTraverseLiveStyle(doc);
  // Data arrives at 10-20 Hz; 60 fps is smooth and halves the load of 120 Hz screens (same cap as the app).
  const maxFps = Number(options.maxFps) > 0 ? Number(options.maxFps) : 60;

  const view = doc.createElement('div'); view.className = 'ctl-view';
  const canvas = doc.createElement('canvas'); canvas.className = 'ctl-canvas';
  const idle = doc.createElement('div'); idle.className = 'ctl-idle'; idle.hidden = true;
  const idleTitle = doc.createElement('b'); const idleBody = doc.createElement('span');
  idle.append(idleTitle, doc.createElement('br'), idleBody);
  const conn = doc.createElement('div'); conn.className = 'ctl-conn';
  view.append(canvas);
  root.append(view);
  const hud = new Hud(view);
  view.append(idle, conn);

  const st = new LiveState(), LF = new LaneFilter(), TRK = new Tracker();
  const scene = new DriveScene(canvas, options.models || {});
  const link = new TraverseLiveLink({
    raw(svc, d, t) {
      if (svc === 'carState') st.onCarState(d, t);
      else if (svc === 'selfdriveState') st.onSelfdrive(d);
      else if (svc === 'carControl') st.onCarControl(d);
      else if (svc === 'modelV2') st.onModel(d, t);
      else if (svc === 'radarState') st.onRadar(d);
    },
    can(frames, t) { st.onCanFrames(frames, t); },
    status() { renderStatus(); },
  }, { base: options.socketBase, decoder: options.decoder });

  let rafId = 0, running = false, destroyed = false;
  let lastHud = 0, frames = 0, fpsT = 0, fps = 0, lastDraw = 0;
  const perf = { build: 0, draw: 0, n: 0, text: '' };
  let size = { w: 0, h: 0 };

  function resize() {
    const rect = view.getBoundingClientRect();
    const w = Math.max(1, Math.round(rect.width)), h = Math.max(1, Math.round(rect.height));
    if (w === size.w && h === size.h) return false;
    size = { w, h };
    scene.resize(w, h, Math.min(target.devicePixelRatio || 1, 2));
    view.style.setProperty('--ctl-u', `${(Math.min(h, w * 9 / 16) / 720).toFixed(4)}px`);
    return true;
  }
  const resizeObserver = typeof target.ResizeObserver === 'function' ? new target.ResizeObserver(() => resize()) : null;
  resizeObserver?.observe(view);

  function renderStatus() {
    const T = viewText(doc);
    const t = performance.now() / 1000;
    const live = t - st.rx.model < 1.5;
    const canLive = t - st.rx.can < 1.5;
    const open = link.isOpen();
    conn.textContent = `${live ? T.receiving : (open ? T.open : T.connecting)}${live && !canLive ? T.noCan : ''}`
      + `${running ? `  ·  ${fps.toFixed(0)} fps${perf.text}` : ''}`;
    conn.className = live ? 'ctl-conn is-live' : (open ? 'ctl-conn is-idle' : 'ctl-conn');
    idleTitle.textContent = T.idleTitle; idleBody.textContent = T.idleBody;
    idle.hidden = !(open && !live);
  }

  function loop(nowMs) {
    rafId = 0;
    if (!running || destroyed) return;
    rafId = target.requestAnimationFrame(loop);
    if (nowMs - lastDraw < 1000 / maxFps - 2) return;
    lastDraw = nowMs;
    resize();
    const t = performance.now() / 1000;
    st.advance(t);
    const c0 = performance.now();
    const { fr, lights, blink } = buildFrame(st, LF, TRK, t);
    const c1 = performance.now();
    scene.update(fr);
    const c2 = performance.now();
    perf.build += c1 - c0; perf.draw += c2 - c1; perf.n++;
    if (t - lastHud > 0.1) { hud.update(st, t, lights, blink); lastHud = t; }
    frames++;
    const now = performance.now();
    if (!fpsT) fpsT = now;
    if (now - fpsT > 1000) {
      fps = frames * 1000 / (now - fpsT); frames = 0; fpsT = now;
      perf.text = perf.n ? `  ·  js ${(perf.build / perf.n).toFixed(1)}+${(perf.draw / perf.n).toFixed(1)} ms` : '';
      perf.build = perf.draw = perf.n = 0;
      renderStatus();
    }
  }

  function start() {
    if (destroyed || running) return false;
    running = true;
    link.start();
    resize();
    renderStatus();
    rafId = target.requestAnimationFrame(loop);
    return true;
  }

  function stop() {
    if (!running) return false;
    running = false;
    link.stop();
    if (rafId) target.cancelAnimationFrame(rafId);
    rafId = 0;
    renderStatus();
    return true;
  }

  function destroy() {
    if (destroyed) return false;
    stop();
    destroyed = true;
    resizeObserver?.disconnect();
    hud.destroy();
    scene.dispose();
    view.remove();
    return true;
  }

  function status() {
    const t = performance.now() / 1000;
    return Object.freeze({
      running,
      destroyed,
      socket: { ...link.stat },
      legacy: link.legacy,
      bytes: link.bytes,
      receiving: t - st.rx.model < 1.5,
      canReceiving: t - st.rx.can < 1.5,
      fps: Math.round(fps),
      size: { ...size },
    });
  }

  return Object.freeze({ root: view, start, stop, destroy, resize, status, state: st });
}
