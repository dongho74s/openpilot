// DOM HUD of the drive tab's 3D view. Started from the Traverse Live phone app HUD
// (~/Documents/traverse-viz-app/web/src/hud.js, 2026-11-05); the carrot web version lives inside the drive slot,
// translates its labels and shows a few more values that already arrive on /ws/viz (lead speed, steering angle,
// following gap, turn arrows, acceleration) plus the speed limit / next turn / signal when the carrot web is
// already receiving carrotMan for another view (it never opens a stream for them).

const TEXT = {
  ko: {
    unit: 'km/h', op: '오픈파일럿', both: '조향 · 가감속', lat: '조향', lon: '가감속', wait: '크루즈 대기', manual: '수동',
    override: '운전자 조작', set: 'SET', lead: '앞차', sec: '초', brake: '브레이크', turnL: '◀ 깜빡이', turnR: '깜빡이 ▶',
    hazard: '비상등', street: '가로등', high: '상향등 권장', caption: '3D 뷰 · 순정 카메라·레이더 CAN', steer: '핸들',
    gap: '차간', gapUnit: '단계', cam: '단속', red: '빨간불', green: '초록불',
  },
  en: {
    unit: 'km/h', op: 'openpilot', both: 'steer · speed', lat: 'steer', lon: 'speed', wait: 'Cruise ready', manual: 'Manual',
    override: 'Driver input', set: 'SET', lead: 'Lead', sec: 's', brake: 'Brake', turnL: '◀ Signal', turnR: 'Signal ▶',
    hazard: 'Hazards', street: 'Street lights', high: 'High beam OK', caption: '3D View · stock camera · radar CAN', steer: 'Wheel',
    gap: 'Gap', gapUnit: '', cam: 'Camera', red: 'Red', green: 'Green',
  },
  zh: {
    unit: 'km/h', op: 'openpilot', both: '转向 · 加减速', lat: '转向', lon: '加减速', wait: '巡航待机', manual: '手动',
    override: '驾驶员操作', set: 'SET', lead: '前车', sec: '秒', brake: '刹车', turnL: '◀ 转向灯', turnR: '转向灯 ▶',
    hazard: '双闪', street: '路灯', high: '建议远光', caption: '3D 视图 · 原厂摄像头·雷达 CAN', steer: '方向盘',
    gap: '跟车', gapUnit: '档', cam: '测速', red: '红灯', green: '绿灯',
  },
};

export function hudText(lang) {
  const code = String(lang || 'ko').toLowerCase().split(/[-_]/)[0];
  return TEXT[code] || TEXT.en;
}

const EXTERNAL_MAX_AGE_MS = 3000;

/* carrotMan from the carrot web's own compact stream, only if it is already flowing (Carrot Vision in another area,
 * the mini HUD, ...) and fresh. Returns null otherwise: the 3D view does not subscribe to it. */
export function externalCarrotMan(target, nowMs = Date.now()) {
  const carrotMan = target?.CarrotHudState?.carrotMan;
  const at = Number(target?.CarrotStateUpdatedAt?.carrotMan);
  if (!carrotMan || !Number.isFinite(at) || nowMs - at > EXTERNAL_MAX_AGE_MS) return null;
  const num = (v) => (Number.isFinite(Number(v)) ? Number(v) : 0);
  return {
    roadLimit: num(carrotMan.nRoadLimitSpeed) > 0 && num(carrotMan.nRoadLimitSpeed) < 200 ? Math.round(num(carrotMan.nRoadLimitSpeed)) : null,
    camLimit: num(carrotMan.xSpdLimit) > 0 && num(carrotMan.xSpdDist) > 0 ? Math.round(num(carrotMan.xSpdLimit)) : null,
    camDist: num(carrotMan.xSpdDist) > 0 ? Math.round(num(carrotMan.xSpdDist)) : null,
    turnDist: num(carrotMan.xDistToTurn) > 0 ? Math.round(num(carrotMan.xDistToTurn)) : null,
    turnText: String(carrotMan.szTBTMainText || '').trim(),
    traffic: num(carrotMan.trafficState) === 1 ? 'red' : num(carrotMan.trafficState) === 2 ? 'green' : null,
  };
}

export function formatDistance(m) {
  if (!(m > 0)) return '';
  return m >= 1000 ? `${(m / 1000).toFixed(m >= 10000 ? 0 : 1)} km` : `${Math.round(m / 10) * 10} m`;
}

function el(doc, tag, className, text) {
  const node = doc.createElement(tag);
  if (className) node.className = className;
  if (text != null) node.textContent = text;
  return node;
}

export class Hud {
  constructor(root, options = {}) {
    const doc = root.ownerDocument;
    this.doc = doc;
    this.target = options.target || doc.defaultView || globalThis;
    this.lang = options.lang || (() => doc.documentElement?.lang || 'ko');
    this.last = '';
    this.root = el(doc, 'div', 'ctl-hud');
    const tl = el(doc, 'div', 'ctl-tl');
    const speedRow = el(doc, 'div', 'ctl-speed-row');
    this.arrowL = el(doc, 'span', 'ctl-arrow', '◀');
    this.arrowR = el(doc, 'span', 'ctl-arrow', '▶');
    const speedBox = el(doc, 'div', 'ctl-speed-box');
    this.speed = el(doc, 'div', 'ctl-speed', '0');
    this.unit = el(doc, 'div', 'ctl-unit', 'km/h');
    this.accel = el(doc, 'div', 'ctl-accel');
    this.accelFill = el(doc, 'div', 'ctl-accel-fill');
    this.accel.append(el(doc, 'span', 'ctl-accel-mid'), this.accelFill);
    speedBox.append(this.speed, this.unit, this.accel);
    const gears = el(doc, 'div', 'ctl-gears');
    this.gears = {};
    for (const g of 'PRND') { this.gears[g] = el(doc, 'span', 'ctl-gear', g); gears.append(this.gears[g]); }
    speedRow.append(this.arrowL, speedBox, gears, this.arrowR);
    const row = el(doc, 'div', 'ctl-row');
    this.mode = el(doc, 'span', 'ctl-pill ctl-pill--off', '');
    this.setSpeed = el(doc, 'span', 'ctl-pill ctl-pill--set', '');
    this.limit = el(doc, 'span', 'ctl-limit', '');
    this.setSpeed.hidden = true;
    this.limit.hidden = true;
    row.append(this.mode, this.setSpeed, this.limit);
    this.lead = el(doc, 'div', 'ctl-lead', '');
    this.badges = el(doc, 'div', 'ctl-badges');
    tl.append(speedRow, row, this.lead, this.badges);
    const tr = el(doc, 'div', 'ctl-tr');
    this.topRight = el(doc, 'div', 'ctl-tr-line', '');
    this.topRight2 = el(doc, 'div', 'ctl-tr-line ctl-tr-line--sub', '');
    tr.append(this.topRight, this.topRight2);
    this.alert = el(doc, 'div', 'ctl-alert', '');
    this.alert.hidden = true;
    this.nav = el(doc, 'div', 'ctl-nav', '');
    this.nav.hidden = true;
    this.caption = el(doc, 'div', 'ctl-caption', '');
    this.root.append(tl, tr, this.alert, this.nav, this.caption);
    root.append(this.root);
  }

  update(st, t, lights, blink) {
    const T = hudText(this.lang());
    const cs = st.cs, lat = !!st.cc.lat, lon = !!st.cc.lon;
    const speed = Math.round(Math.abs((cs.v || 0) * 3.6));
    let mode, modeCls;
    if ((lat || lon) && st.sd.state === 'overriding') { mode = T.override; modeCls = 'ctl-pill ctl-pill--override'; }
    else if (lat || lon) { mode = `${T.op}  ${lat && lon ? T.both : (lat ? T.lat : T.lon)}`; modeCls = 'ctl-pill ctl-pill--on'; }
    else if (cs.cen) { mode = T.wait; modeCls = 'ctl-pill ctl-pill--wait'; }
    else { mode = T.manual; modeCls = 'ctl-pill ctl-pill--off'; }
    const setSpd = cs.cen && cs.cspdKph > 1 && cs.cspdKph < 250 ? `${T.set} ${Math.round(cs.cspdKph)}` : '';
    const ext = externalCarrotMan(this.target);
    const limit = ext?.roadLimit ?? null;
    let lead = '';
    const badges = [];
    if (st.rs.st) {
      const d510 = st.frame(2, 0x510, t, 0.5);
      const gap = d510 && (d510[2] & 0x1F) ? (d510[2] & 0x1F) * 0.1 : null;
      const vr = Math.round(st.rs.vr * 3.6);
      const vl = st.rs.vl != null ? Math.max(0, Math.round(st.rs.vl * 3.6)) : null;
      lead = `${T.lead}  ${Math.round(st.rs.d)} m${vl != null ? `  ·  ${vl} km/h` : ''}   (${vr >= 0 ? '+' : ''}${vr})${gap ? `   ${gap.toFixed(1)}${T.sec}` : ''}`;
      if (lights.brake) badges.push(['ctl-badge ctl-badge--brake', T.brake]);
      if ([2, 3, 4].includes(lights.turn)) {
        const lab = { 2: T.turnL, 3: T.turnR, 4: T.hazard }[lights.turn];
        badges.push([`ctl-badge ctl-badge--turn${blink ? '' : ' is-dim'}`, lab]);
      }
    }
    if (ext?.camLimit) badges.push(['ctl-badge ctl-badge--cam', `${T.cam} ${ext.camLimit}  ${formatDistance(ext.camDist)}`]);
    const tr = [];
    const now = new Date(); tr.push(`${String(now.getHours()).padStart(2, '0')}:${String(now.getMinutes()).padStart(2, '0')}`);
    const d3fc = st.frame(0, 0x3FC, t, 2.0);
    if (d3fc) tr.push(`${Math.round(d3fc[4] * 0.5 - 40)}°C`);
    const l135 = st.frame(1, 0x135, t, 0.6);
    if (l135 && (l135[0] & 3)) tr.push({ 1: T.street, 2: T.high }[l135[0] & 3] || '');
    const tr2 = [`${T.steer} ${Math.round(cs.sa || 0)}°`];
    if (st.sd.personality != null) tr2.push(`${T.gap} ${'▮'.repeat(st.sd.personality + 1)}${'▯'.repeat(3 - st.sd.personality)}`);
    const alert = st.sd.a1 || '';
    const navParts = [];
    if (ext?.traffic) navParts.push(ext.traffic === 'red' ? `● ${T.red}` : `● ${T.green}`);
    if (ext?.turnDist) navParts.push(`${formatDistance(ext.turnDist)}${ext.turnText ? `  ${ext.turnText}` : ''}`);
    const nav = navParts.join('   ');
    const accel = Math.max(-3, Math.min(2, cs.a || 0));
    const accelKey = Math.round(accel * 10);
    const key = [speed, cs.gear, mode, setSpd, limit, lead, badges.map((b) => b.join(':')).join(','), tr.join(' '), tr2.join(' '),
      alert, nav, T.caption, cs.lb && blink, cs.rb && blink, cs.lb, cs.rb, accelKey, ext?.traffic].join('|');
    if (key === this.last) return;
    this.last = key;
    this.speed.textContent = String(speed);
    this.unit.textContent = T.unit;
    for (const g of 'PRND') this.gears[g].className = g === cs.gear ? 'ctl-gear is-current' : 'ctl-gear';
    this.arrowL.className = `ctl-arrow${cs.lb ? ' is-on' : ''}${cs.lb && !blink ? ' is-dim' : ''}`;
    this.arrowR.className = `ctl-arrow${cs.rb ? ' is-on' : ''}${cs.rb && !blink ? ' is-dim' : ''}`;
    // Acceleration: green to the right of the centre tick, red to the left (braking), full scale +2 / -3 m/s².
    const frac = accel >= 0 ? accel / 2 : accel / 3;
    this.accelFill.style.left = frac >= 0 ? '50%' : `${50 + frac * 50}%`;
    this.accelFill.style.width = `${Math.abs(frac) * 50}%`;
    this.accelFill.className = `ctl-accel-fill${accel < 0 ? ' is-brake' : ''}`;
    this.mode.textContent = mode; this.mode.className = modeCls;
    this.setSpeed.textContent = setSpd; this.setSpeed.hidden = !setSpd;
    this.limit.textContent = limit ? String(limit) : ''; this.limit.hidden = !limit;
    this.lead.textContent = lead;
    this.badges.replaceChildren(...badges.map(([cls, text]) => el(this.doc, 'span', cls, text)));
    this.topRight.textContent = tr.join('   ');
    this.topRight2.textContent = tr2.join('   ');
    this.alert.textContent = alert; this.alert.hidden = !alert;
    this.nav.textContent = nav; this.nav.hidden = !nav || Boolean(alert);
    this.nav.className = `ctl-nav${ext?.traffic ? ` is-${ext.traffic}` : ''}`;
    this.caption.textContent = T.caption;
  }

  destroy() {
    this.root.remove();
  }
}
