// DOM HUD of Traverse Live (same content as the phone app / drive_viz v7 video HUD),
// ported from ~/Documents/traverse-viz-app/web/src/hud.js (2026-11-05). In the carrot web it lives inside the
// drive slot instead of the whole page, so it builds its own nodes and only labels are translated.

const TEXT = {
  ko: {
    unit: 'km/h', op: '오픈파일럿', both: '조향 · 가감속', lat: '조향', lon: '가감속', wait: '크루즈 대기', manual: '수동',
    lead: '앞차', sec: '초', brake: '브레이크', turnL: '◀ 깜빡이', turnR: '깜빡이 ▶', hazard: '비상등',
    street: '가로등', high: '상향등 권장', caption: 'TRAVERSE · 순정 카메라·레이더 CAN',
  },
  en: {
    unit: 'km/h', op: 'openpilot', both: 'steer · speed', lat: 'steer', lon: 'speed', wait: 'Cruise ready', manual: 'Manual',
    lead: 'Lead', sec: 's', brake: 'Brake', turnL: '◀ Signal', turnR: 'Signal ▶', hazard: 'Hazards',
    street: 'Street lights', high: 'High beam OK', caption: 'TRAVERSE · stock camera · radar CAN',
  },
  zh: {
    unit: 'km/h', op: 'openpilot', both: '转向 · 加减速', lat: '转向', lon: '加减速', wait: '巡航待机', manual: '手动',
    lead: '前车', sec: '秒', brake: '刹车', turnL: '◀ 转向灯', turnR: '转向灯 ▶', hazard: '双闪',
    street: '路灯', high: '建议远光', caption: 'TRAVERSE · 原厂摄像头·雷达 CAN',
  },
};

export function hudText(lang) {
  const code = String(lang || 'ko').toLowerCase().split(/[-_]/)[0];
  return TEXT[code] || TEXT.en;
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
    this.lang = options.lang || (() => doc.documentElement?.lang || 'ko');
    this.last = '';
    this.root = el(doc, 'div', 'ctl-hud');
    const tl = el(doc, 'div', 'ctl-tl');
    const speedRow = el(doc, 'div', 'ctl-speed-row');
    const speedBox = el(doc, 'div');
    this.speed = el(doc, 'div', 'ctl-speed', '0');
    this.unit = el(doc, 'div', 'ctl-unit', 'km/h');
    speedBox.append(this.speed, this.unit);
    const gears = el(doc, 'div', 'ctl-gears');
    this.gears = {};
    for (const g of 'PRND') { this.gears[g] = el(doc, 'span', 'ctl-gear', g); gears.append(this.gears[g]); }
    speedRow.append(speedBox, gears);
    const row = el(doc, 'div', 'ctl-row');
    this.mode = el(doc, 'span', 'ctl-pill ctl-pill--off', '');
    this.setSpeed = el(doc, 'span', 'ctl-pill ctl-pill--set', '');
    this.setSpeed.hidden = true;
    row.append(this.mode, this.setSpeed);
    this.lead = el(doc, 'div', 'ctl-lead', '');
    this.badges = el(doc, 'div', 'ctl-badges');
    tl.append(speedRow, row, this.lead, this.badges);
    this.topRight = el(doc, 'div', 'ctl-tr', '');
    this.alert = el(doc, 'div', 'ctl-alert', '');
    this.alert.hidden = true;
    this.caption = el(doc, 'div', 'ctl-caption', '');
    this.root.append(tl, this.topRight, this.alert, this.caption);
    root.append(this.root);
  }

  update(st, t, lights, blink) {
    const T = hudText(this.lang());
    const cs = st.cs, lat = !!st.cc.lat, lon = !!st.cc.lon;
    const speed = Math.round(Math.abs((cs.v || 0) * 3.6));
    let mode, modeCls;
    if (lat || lon) { mode = `${T.op}  ${lat && lon ? T.both : (lat ? T.lat : T.lon)}`; modeCls = 'ctl-pill ctl-pill--on'; }
    else if (cs.cen) { mode = T.wait; modeCls = 'ctl-pill ctl-pill--wait'; }
    else { mode = T.manual; modeCls = 'ctl-pill ctl-pill--off'; }
    const setSpd = cs.cen && cs.cspdKph > 1 && cs.cspdKph < 250 ? `${Math.round(cs.cspdKph)}` : '';
    let lead = '';
    const badges = [];
    if (st.rs.st) {
      const d510 = st.frame(2, 0x510, t, 0.5);
      const gap = d510 && (d510[2] & 0x1F) ? (d510[2] & 0x1F) * 0.1 : null;
      const vr = Math.round(st.rs.vr * 3.6);
      lead = `${T.lead}  ${Math.round(st.rs.d)} m   ${vr >= 0 ? '+' : ''}${vr} km/h${gap ? `   ${gap.toFixed(1)}${T.sec}` : ''}`;
      if (lights.brake) badges.push(['ctl-badge ctl-badge--brake', T.brake]);
      if ([2, 3, 4].includes(lights.turn)) {
        const lab = { 2: T.turnL, 3: T.turnR, 4: T.hazard }[lights.turn];
        badges.push([`ctl-badge ctl-badge--turn${blink ? '' : ' is-dim'}`, lab]);
      }
    }
    const tr = [];
    const now = new Date(); tr.push(`${String(now.getHours()).padStart(2, '0')}:${String(now.getMinutes()).padStart(2, '0')}`);
    const d3fc = st.frame(0, 0x3FC, t, 2.0);
    if (d3fc) tr.push(`${Math.round(d3fc[4] * 0.5 - 40)}°C`);
    const l135 = st.frame(1, 0x135, t, 0.6);
    if (l135 && (l135[0] & 3)) tr.push({ 1: T.street, 2: T.high }[l135[0] & 3] || '');
    const alert = st.sd.a1 || '';
    const key = [speed, cs.gear, mode, setSpd, lead, badges.map((b) => b.join(':')).join(','), tr.join(' '), alert, T.caption].join('|');
    if (key === this.last) return;
    this.last = key;
    this.speed.textContent = String(speed);
    this.unit.textContent = T.unit;
    for (const g of 'PRND') this.gears[g].className = g === cs.gear ? 'ctl-gear is-current' : 'ctl-gear';
    this.mode.textContent = mode; this.mode.className = modeCls;
    this.setSpeed.textContent = setSpd; this.setSpeed.hidden = !setSpd;
    this.lead.textContent = lead;
    this.badges.replaceChildren(...badges.map(([cls, text]) => el(this.root.ownerDocument, 'span', cls, text)));
    this.topRight.textContent = tr.join('   ');
    this.alert.textContent = alert; this.alert.hidden = !alert;
    this.caption.textContent = T.caption;
  }

  destroy() {
    this.root.remove();
  }
}
