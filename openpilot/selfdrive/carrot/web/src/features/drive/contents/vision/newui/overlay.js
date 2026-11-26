"use strict";

/* Carrot Vision "new ui 1": the C3/C3X (tici) NewUi=1 driving HUD ("v10-nui").
 *
 * C3 reference: selfdrive/ui/onroad/v10_nui/hud.py (bottom HUD: current speed,
 * SET, gap, LFA, APN badge, turn signals, round steering-wheel button),
 * view.py (gradients, blind-spot icons, screen corners) and alerts.py. Positions
 * and sizes are the C3's, in a 1080-unit-tall SVG whose width follows the view,
 * so left items keep their distance from the left edge, right items from the
 * right edge and the speed stays centred.
 *
 * It reads the compact realtime state the rest of the drive view uses
 * (window.CarrotHudState / window.CarrotOverlayState) on every animation frame
 * while visible, like the device redraws every frame. */

import {
  ALERT_COLLISION,
  ALERT_COLORS,
  ALERT_LANE_DEPARTURE,
  ALERT_RESUME_REQUIRED,
  DESIGN_HEIGHT,
  SIZE_FULL,
  SIZE_MID,
  SIZE_SMALL,
  alertBox,
  alertCoversHud,
  alertIcon,
  alertKind,
  blindspotBlinkVisible,
  cameraFilter,
  currentSpeedColors,
  deriveNewUiInputs,
  fontSize,
  formatMinutesSeconds,
  gapTint,
  lfaStyle,
  lineHeight,
  setSpeedColors,
  textBaseline,
  turnSignalOn,
  wheelStyle,
} from "./state.js";
import { injectNewUiStyle, NEWUI_ASSET_ROOT } from "./style.js";

const SVG_NS = "http://www.w3.org/2000/svg";
const FRAME_MIN_MS = 30;
const DEFAULT_WIDTH = 2160;
// The narrowest C3 onroad view (2160 minus the 300 px sidebar). A narrower web
// view scales the whole HUD down to this width instead of letting it overlap.
export const MIN_DESIGN_WIDTH = 1860;
const ICONS = "icons/v10_nui";

// hud.py
const STEERING_ICON_SIZE = 156;
const STEERING_ICON_WARNING_SIZE = 188;
const LFA_ICON_SIZE = 104;
const GAP_ICON_WIDTH = 124;
const GAP_ICON_HEIGHT = 84;
const CARROT_BADGE_WIDTH = 112;
const CARROT_BADGE_HEIGHT = 52;
const CARROT_BADGE_GAP = 18;
const TURN_SIGNAL_ICON_SIZE = 72;
const TURN_SIGNAL_GAP = 28;
const WARNING_ICON_SIZE = 76;
const HUD_SIDE_MARGIN = 104;
const HUD_BOTTOM_MARGIN = 96;
const SET_SPEED_GAP = 28;
const SET_SPEED_WIDTH = 236;
const SET_SPEED_HEIGHT = 118;
const SPEED_GROUP_FROM_BOTTOM = 246;
const TURN_SIGNAL_GREEN = [0x1E, 0xFF, 0x5C, 255];
// view.py: blind-spot icons 196x224 keeping the 392x447 aspect -> 196x223.
const BLINDSPOT_W = 196;
const BLINDSPOT_H = 223;
const BLINDSPOT_GLOW = [0xFF, 0x67, 0x4A, 255];
// alerts.py: autohold.png 368x252 fitted into 184 -> 184x126.
const AUTOHOLD_ICON = 184;
const AUTOHOLD_ICON_HEIGHT = 126;

let idCounter = 0;

function svg(doc, name, attrs = {}, children = []) {
  const node = doc.createElementNS(SVG_NS, name);
  for (const [key, value] of Object.entries(attrs)) {
    if (value != null) node.setAttribute(key, String(value));
  }
  for (const child of children) if (child) node.appendChild(child);
  return node;
}

function html(doc, name, className, children = []) {
  const node = doc.createElement(name);
  if (className) node.className = className;
  for (const child of children) if (child) node.appendChild(child);
  return node;
}

function asset(path) {
  return `${NEWUI_ASSET_ROOT}/${path}`;
}

function rgba([r, g, b, a = 255], opacity = 1) {
  const alpha = Math.max(0, Math.min(255, a * opacity)) / 255;
  return `rgba(${Math.round(r)},${Math.round(g)},${Math.round(b)},${alpha.toFixed(3)})`;
}

// hud.py set_speed_rect / speed_rect / gap_rect / wheel_center, w = view width, h = 1080.
export function hudLayout(w, h = DESIGN_HEIGHT) {
  const setSpeed = [w - HUD_SIDE_MARGIN - STEERING_ICON_SIZE - SET_SPEED_GAP - SET_SPEED_WIDTH,
    h - HUD_BOTTOM_MARGIN - SET_SPEED_HEIGHT, SET_SPEED_WIDTH, SET_SPEED_HEIGHT];
  const groupTop = h - SPEED_GROUP_FROM_BOTTOM;
  const speed = [w / 2 - 230, groupTop - 18, 460, 162];
  const gap = [setSpeed[0] - GAP_ICON_WIDTH - 34, setSpeed[1] + Math.floor((SET_SPEED_HEIGHT - GAP_ICON_HEIGHT) / 2) + 2,
    GAP_ICON_WIDTH, GAP_ICON_HEIGHT];
  const wheel = (size) => [w - HUD_SIDE_MARGIN - size / 2, h - HUD_BOTTOM_MARGIN - size / 2];
  return { setSpeed, speed, gap, wheel };
}

// One C3 unit in CSS px and the view in C3 units: 1080 tall, at least 1860 wide.
export function newUiDesignSize(cssWidth, cssHeight) {
  const unitPx = Math.min(cssHeight / DESIGN_HEIGHT, cssWidth / MIN_DESIGN_WIDTH);
  return { unitPx, width: cssWidth / unitPx, height: cssHeight / unitPx };
}

export function createNewUiHudOverlay(doc, options = {}) {
  const target = options.target || doc?.defaultView || globalThis;
  injectNewUiStyle(doc);
  const uid = `cnui${++idCounter}`;
  const now = () => (typeof target.performance?.now === "function" ? target.performance.now() : Date.now());

  // ---- DOM ------------------------------------------------------------------
  const root = html(doc, "div", "cnui");
  root.setAttribute("data-carrot-hud", "overlay");
  root.setAttribute("data-carrot-newui", "c3");
  root.setAttribute("aria-hidden", "true");
  const under = html(doc, "div", "cnui-under", [
    html(doc, "div", "cnui-under-chrome"),
    html(doc, "div", "cnui-under-footer"),
    html(doc, "div", "cnui-under-hud"),
  ]);
  under.setAttribute("aria-hidden", "true");

  const defs = svg(doc, "defs");
  // A white icon tinted like raylib draw_texture(tint): the icon's alpha masks a
  // rectangle filled with the tint.
  function tintedIcon(name, width, height) {
    const id = `${uid}-${name}-${width}x${height}`;
    defs.appendChild(svg(doc, "mask", { id, maskUnits: "userSpaceOnUse", x: 0, y: 0, width, height, style: "mask-type:alpha" }, [
      svg(doc, "image", { href: asset(`${ICONS}/${name}.png`), x: 0, y: 0, width, height, preserveAspectRatio: "none" }),
    ]));
    const rect = svg(doc, "rect", { x: 0, y: 0, width, height, mask: `url(#${id})` });
    const group = svg(doc, "g", {}, [rect]);
    return { group, rect };
  }
  function radialGlow(name) {
    const inner = svg(doc, "stop", { offset: "0" });
    const middle = svg(doc, "stop", { offset: "0.45" });
    const outer = svg(doc, "stop", { offset: "1" });
    const gradient = svg(doc, "radialGradient", { id: `${uid}-${name}`, gradientUnits: "userSpaceOnUse" }, [inner, middle, outer]);
    defs.appendChild(gradient);
    const ellipse = svg(doc, "ellipse", { fill: `url(#${uid}-${name})` });
    return { gradient, inner, middle, outer, ellipse };
  }

  const setLabel = svg(doc, "text", { "font-size": fontSize(28).toFixed(2) });
  setLabel.textContent = "SET";
  const setValue = svg(doc, "text", { "font-size": fontSize(88).toFixed(2) });
  const setGroup = svg(doc, "g", {}, [setLabel, setValue]);

  const speedValue = svg(doc, "text", { "font-size": fontSize(148).toFixed(2), "text-anchor": "middle" });
  const speedUnit = svg(doc, "text", { "font-size": fontSize(34).toFixed(2), "text-anchor": "middle" });

  const turnLeft = tintedIcon("turn_signal_left", TURN_SIGNAL_ICON_SIZE, TURN_SIGNAL_ICON_SIZE);
  const turnRight = tintedIcon("turn_signal_right", TURN_SIGNAL_ICON_SIZE, TURN_SIGNAL_ICON_SIZE);

  const badgeShadow = svg(doc, "rect", { width: CARROT_BADGE_WIDTH, height: CARROT_BADGE_HEIGHT, rx: 18, fill: "rgba(0,0,0,0.337)" });
  const badgeBox = svg(doc, "rect", { width: CARROT_BADGE_WIDTH, height: CARROT_BADGE_HEIGHT, rx: 18 });
  const badgeText = svg(doc, "text", { "font-size": fontSize(28).toFixed(2), "text-anchor": "middle", fill: "#fff" });
  badgeText.textContent = "APN";
  const badgeGroup = svg(doc, "g", {}, [badgeShadow, badgeBox, badgeText]);

  const gapIcons = [1, 2, 3, 4].map((level) => ({
    shadow: tintedIcon(`gap${level}`, GAP_ICON_WIDTH, GAP_ICON_HEIGHT),
    main: tintedIcon(`gap${level}`, GAP_ICON_WIDTH, GAP_ICON_HEIGHT),
  }));
  const gapGroup = svg(doc, "g", {}, gapIcons.flatMap(({ shadow, main }) => [shadow.group, main.group]));

  const lfaGlow = radialGlow("lfa-glow");
  const lfaIcon = tintedIcon("lfa", LFA_ICON_SIZE, LFA_ICON_SIZE);
  const lfaGroup = svg(doc, "g", {}, [lfaGlow.ellipse, lfaIcon.group]);

  const warningGlow = radialGlow("warning-glow");
  const warningIcon = svg(doc, "image", { href: asset(`${ICONS}/warning.png`), width: WARNING_ICON_SIZE, height: WARNING_ICON_SIZE, preserveAspectRatio: "none" });
  const warningGroup = svg(doc, "g", {}, [warningGlow.ellipse, warningIcon]);

  const wheelButton = svg(doc, "circle");
  const wheelIcons = {
    150: tintedIcon("steeringwheel", 150, 150),
    178: tintedIcon("steeringwheel", 178, 178),
  };
  const wheelRotate = svg(doc, "g", {}, [wheelIcons[150].group, wheelIcons[178].group]);
  const wheelGroup = svg(doc, "g", {}, [wheelButton, wheelRotate]);

  const blindspots = ["left", "right"].map((side) => {
    const glow = radialGlow(`bsm-${side}`);
    const image = svg(doc, "image", {
      href: asset(`${ICONS}/blindspot_${side}.png`), width: BLINDSPOT_W, height: BLINDSPOT_H, preserveAspectRatio: "none",
    });
    return { side, glow, image, group: svg(doc, "g", {}, [glow.ellipse, image]) };
  });

  // Alerts
  const alertFullBg = svg(doc, "rect", { x: 0, y: 0 });
  const alertFullGradient = svg(doc, "linearGradient", { id: `${uid}-alert-bg`, x1: 0, y1: 0, x2: 0, y2: 1 });
  defs.appendChild(alertFullGradient);
  alertFullBg.setAttribute("fill", `url(#${uid}-alert-bg)`);
  const alertShadow = svg(doc, "rect", { fill: "rgba(0,0,0,0.2)" });
  const alertBoxRect = svg(doc, "rect");
  const alertIconGlow = radialGlow("alert-icon-glow");
  const alertIconImage = svg(doc, "image", { preserveAspectRatio: "none" });
  const alertIconGroup = svg(doc, "g", {}, [alertIconGlow.ellipse, alertIconImage]);
  const alertTitle = svg(doc, "text", { "text-anchor": "middle", fill: "#fff" });
  const alertSubtitle = svg(doc, "text", { "text-anchor": "middle", fill: "#fff" });
  const autoholdIcon = svg(doc, "image", {
    href: asset(`${ICONS}/autohold.png`), width: AUTOHOLD_ICON, height: AUTOHOLD_ICON_HEIGHT, preserveAspectRatio: "none",
  });
  const autoholdTimer = svg(doc, "text", { "text-anchor": "middle", fill: "#fff", "font-size": fontSize(112).toFixed(2) });
  const alertGroup = svg(doc, "g", {}, [alertFullBg, alertShadow, alertBoxRect, alertIconGroup, autoholdIcon, autoholdTimer,
    alertTitle, alertSubtitle]);

  const hudGroup = svg(doc, "g", {}, [setGroup, speedValue, speedUnit, turnLeft.group, turnRight.group, badgeGroup, gapGroup,
    lfaGroup, warningGroup, wheelGroup]);
  const svgRoot = svg(doc, "svg", {
    class: "cnui-svg",
    viewBox: `0 0 ${DEFAULT_WIDTH} ${DESIGN_HEIGHT}`,
    preserveAspectRatio: "xMidYMid meet",
    "aria-hidden": "true",
    focusable: "false",
  }, [defs, hudGroup, ...blindspots.map((b) => b.group), alertGroup]);

  const fullTitle = html(doc, "div", "cnui-full");
  const fullSubtitle = html(doc, "div", "cnui-full");
  const corners = html(doc, "div", "cnui-corners");
  root.append(svgRoot, fullTitle, fullSubtitle, corners);

  // ---- state ----------------------------------------------------------------
  let width = DEFAULT_WIDTH;
  let height = DESIGN_HEIGHT;
  let unitPx = 1;
  let active = false;
  let destroyed = false;
  let rafId = 0;
  let lastTick = 0;
  let lastPayload = null;
  let lastInputs = null;
  let vEgoClusterSeen = false;
  let resumeSince = null;
  let resizeObserver = null;
  const suppressions = new Set();
  const written = new Map();

  function write(node, name, value) {
    let map = written.get(node);
    if (!map) {
      map = new Map();
      written.set(node, map);
    }
    const next = value == null ? null : String(value);
    if (map.get(name) === next) return;
    map.set(name, next);
    if (next == null) node.removeAttribute(name);
    else node.setAttribute(name, next);
  }

  function show(node, visible) {
    write(node, "visibility", visible ? null : "hidden");
  }

  function setText(node, text) {
    const value = text == null ? "" : String(text);
    if (node.textContent !== value) node.textContent = value;
  }

  // v10_nui/draw.draw_text for one line in [x, y, w, h].
  function placeText(node, text, size, [x, y, w, h], color, hAlign = "left", vAlign = "center") {
    setText(node, text);
    write(node, "font-size", fontSize(size).toFixed(2));
    write(node, "text-anchor", hAlign === "center" ? "middle" : hAlign === "right" ? "end" : "start");
    write(node, "x", (hAlign === "center" ? x + w / 2 : hAlign === "right" ? x + w : x).toFixed(2));
    write(node, "y", textBaseline(size, y, h, vAlign).toFixed(2));
    write(node, "fill", rgba(color));
  }

  function placeIcon(icon, x, y, color, opacity = 1) {
    write(icon.group, "transform", `translate(${x.toFixed(2)} ${y.toFixed(2)})`);
    write(icon.rect, "fill", rgba(color, opacity));
  }

  // QRadialGradient(centre of rect, radius) inside the ellipse of rect.
  function placeGlow(glow, [x, y, w, h], radius, color, inner, middle) {
    const cx = x + w / 2;
    const cy = y + h / 2;
    write(glow.ellipse, "cx", cx.toFixed(2));
    write(glow.ellipse, "cy", cy.toFixed(2));
    write(glow.ellipse, "rx", (w / 2).toFixed(2));
    write(glow.ellipse, "ry", (h / 2).toFixed(2));
    write(glow.gradient, "cx", cx.toFixed(2));
    write(glow.gradient, "cy", cy.toFixed(2));
    write(glow.gradient, "r", radius.toFixed(2));
    const [r, g, b] = color;
    write(glow.inner, "stop-color", rgba([r, g, b, inner]));
    write(glow.middle, "stop-color", rgba([r, g, b, middle]));
    write(glow.outer, "stop-color", rgba([r, g, b, 0]));
  }

  function stage() {
    return root.parentNode && root.parentNode.nodeType === 1 ? root.parentNode : null;
  }

  function attachUnderlay() {
    const stageNode = stage();
    if (!stageNode || under.parentNode === stageNode) return;
    // Under the lanes and path: right after the video layers, before the overlay canvases.
    const before = stageNode.querySelector?.("#carrotPerformanceCanvas, #carrotOverlayCanvas") || null;
    stageNode.insertBefore(under, before && before.parentNode === stageNode ? before : null);
  }

  function visible() {
    return active && !destroyed && suppressions.size === 0;
  }

  // ---- layout ---------------------------------------------------------------
  function relayout() {
    if (destroyed) return false;
    const rect = root.getBoundingClientRect?.();
    const w = Number(rect?.width);
    const h = Number(rect?.height);
    if (!(w > 0 && h > 0)) return false;
    const design = newUiDesignSize(w, h);
    unitPx = design.unitPx;
    width = design.width;
    height = design.height;
    write(svgRoot, "viewBox", `0 0 ${width.toFixed(3)} ${height.toFixed(3)}`);
    root.style.setProperty("--cnui-u", `${unitPx.toFixed(5)}px`);
    return true;
  }

  // ---- HUD (hud.py NuiHud.draw) ----------------------------------------------
  function drawHud(inputs, nowMs) {
    show(hudGroup, true);
    const layout = hudLayout(width, height);
    const status = inputs.status;

    // Set speed
    show(setGroup, inputs.cruiseAvailable);
    if (inputs.cruiseAvailable) {
      const [valueColor, labelColor] = setSpeedColors(inputs.cruiseSet, status);
      const [sx, sy, sw, sh] = layout.setSpeed;
      placeText(setLabel, "SET", 28, [sx, sy + 44, 72, 34], labelColor);
      placeText(setValue, inputs.cruiseSet ? String(Math.round(inputs.setSpeed)) : "–", 88, [sx + 72, sy - 6, sw - 72, sh], valueColor);
    }

    // Current speed (the 60 s standstill timer needs carState.standstill, not sent to the web)
    const [speedColor, unitColor] = currentSpeedColors(status);
    const [rx, ry, rw, rh] = layout.speed;
    placeText(speedValue, String(Math.round(inputs.speed)), 148, [rx, ry, rw, rh], speedColor, "center", "bottom");
    placeText(speedUnit, inputs.isMetric ? "KM/H" : "MPH", 34, [width / 2 - 116, ry + rh + 1, 232, 40], unitColor, "center", "top");

    // Turn signals beside the speed
    const iconY = ry + Math.floor((rh - 1) / 2) - TURN_SIGNAL_ICON_SIZE / 2 + 2;
    const signalOpacity = turnSignalOn(nowMs) ? 0.96 : 0.14;
    show(turnLeft.group, inputs.leftSignal);
    show(turnRight.group, inputs.rightSignal);
    if (inputs.leftSignal) placeIcon(turnLeft, rx - TURN_SIGNAL_ICON_SIZE - TURN_SIGNAL_GAP, iconY, TURN_SIGNAL_GREEN, signalOpacity);
    if (inputs.rightSignal) placeIcon(turnRight, rx + rw - 1 + TURN_SIGNAL_GAP, iconY, TURN_SIGNAL_GREEN, signalOpacity);

    // APN badge over the gap icon
    const [gx, gy, gw] = layout.gap;
    show(badgeGroup, inputs.carrotLevel >= 1);
    if (inputs.carrotLevel >= 1) {
      const bx = gx + Math.floor((gw - CARROT_BADGE_WIDTH) / 2);
      const by = gy - CARROT_BADGE_HEIGHT - CARROT_BADGE_GAP;
      write(badgeShadow, "x", bx);
      write(badgeShadow, "y", by + 4);
      write(badgeBox, "x", bx);
      write(badgeBox, "y", by);
      write(badgeBox, "fill", rgba(inputs.carrotLevel >= 2 ? [0x2F, 0xC8, 0x67, 0xF2] : [0x4F, 0x8D, 0xFF, 0xEC]));
      placeText(badgeText, "APN", 28, [bx, by, CARROT_BADGE_WIDTH, CARROT_BADGE_HEIGHT], [255, 255, 255, 255], "center", "center");
    }

    // Gap icon (a dark copy 4 units lower, then the tinted icon)
    gapIcons.forEach(({ shadow, main }, index) => {
      const visibleLevel = inputs.gapLevel === index + 1;
      show(shadow.group, visibleLevel);
      show(main.group, visibleLevel);
      if (!visibleLevel) return;
      placeIcon(shadow, gx, gy + 4, [0, 0, 0, 0x74]);
      placeIcon(main, gx, gy, gapTint(status));
    });

    // LFA icon at the right end of the speed group
    const ix = rx + rw - 1 - 84;
    const iy = ry + 2;
    const [lfaTint, lfaGlowColor] = lfaStyle(inputs.longOverride, inputs.lateralOnly, inputs.enabled, inputs.engageable);
    show(lfaGlow.ellipse, Boolean(lfaGlowColor));
    if (lfaGlowColor) {
      const gw2 = LFA_ICON_SIZE + 52;
      placeGlow(lfaGlow, [ix - 26, iy - 22, gw2, LFA_ICON_SIZE + 46], gw2 * 0.55, lfaGlowColor, 76, 34);
    }
    placeIcon(lfaIcon, ix, iy, lfaTint);

    // Steer-limit warning and the steering-wheel button
    const warning = inputs.steerLimitWarning;
    const button = warning ? STEERING_ICON_WARNING_SIZE : STEERING_ICON_SIZE;
    const iconSize = warning ? 178 : 150;
    const [cx, cy] = layout.wheel(button);
    show(warningGroup, warning);
    if (warning) {
      const wx = cx - STEERING_ICON_WARNING_SIZE / 2 - 84;
      const wy = cy - STEERING_ICON_WARNING_SIZE / 2 - 20;
      const gw3 = WARNING_ICON_SIZE + 52;
      placeGlow(warningGlow, [wx - 26, wy - 22, gw3, WARNING_ICON_SIZE + 44], gw3 * 0.55, [0xFF, 0x68, 0x5B], 64, 26);
      write(warningIcon, "x", wx.toFixed(2));
      write(warningIcon, "y", wy.toFixed(2));
    }
    const steeringActive = inputs.enabled || inputs.longOverride || inputs.latOverride || inputs.lateralOnly;
    const [buttonColor, wheelTint] = wheelStyle(inputs.torque, steeringActive, inputs.latOverride);
    write(wheelButton, "cx", cx.toFixed(2));
    write(wheelButton, "cy", cy.toFixed(2));
    write(wheelButton, "r", (button / 2).toFixed(2));
    write(wheelButton, "fill", rgba(buttonColor));
    write(wheelRotate, "transform", `translate(${cx.toFixed(2)} ${cy.toFixed(2)}) rotate(${inputs.steeringAngle.toFixed(1)})`);
    for (const size of [150, 178]) {
      const icon = wheelIcons[size];
      show(icon.group, size === iconSize);
      if (size === iconSize) placeIcon(icon, -size / 2, -size / 2, wheelTint);
    }
  }

  // ---- blind-spot icons (view.py _draw_blindspots) ----------------------------
  function drawBlindspots(inputs, nowMs) {
    const blinkOn = blindspotBlinkVisible(nowMs);
    const y = Math.floor((height - 1) / 2) - 180;
    for (const spot of blindspots) {
      const left = spot.side === "left";
      const on = left ? inputs.leftBlindspot : inputs.rightBlindspot;
      const blinking = left ? inputs.leftSignal : inputs.rightSignal;
      const shown = on && (!blinking || blinkOn);
      show(spot.group, shown);
      if (!shown) continue;
      const x = left ? 44 : width - 40 - BLINDSPOT_W;
      const gw = BLINDSPOT_W + 68;
      placeGlow(spot.glow, [x - 34, y - 28, gw, BLINDSPOT_H + 56], gw * 0.56, BLINDSPOT_GLOW, 82, 34);
      write(spot.image, "x", x);
      write(spot.image, "y", y);
    }
  }

  // ---- alerts (alerts.py NuiAlerts.render) -------------------------------------
  function setFullGradient(stops) {
    while (alertFullGradient.firstChild) alertFullGradient.removeChild(alertFullGradient.firstChild);
    for (const [offset, color] of stops) {
      alertFullGradient.appendChild(svg(doc, "stop", { offset, "stop-color": rgba(color) }));
    }
  }

  function placeFullText(node, text, size, top, height) {
    if (node.textContent !== text) node.textContent = text;
    node.hidden = !text;
    node.style.top = `${(top * unitPx).toFixed(2)}px`;
    node.style.maxHeight = `${(height * unitPx).toFixed(2)}px`;
    node.style.fontSize = `${(fontSize(size) * unitPx).toFixed(2)}px`;
    node.style.lineHeight = `${(lineHeight(size) * unitPx).toFixed(2)}px`;
  }

  let alertSignature = "";
  function drawAlert(inputs, nowMs) {
    const alert = inputs.alert;
    const kind = alert ? alertKind(alert.type) : "";
    if (kind === ALERT_RESUME_REQUIRED) {
      if (resumeSince == null) resumeSince = nowMs;
    } else {
      resumeSince = null;
    }
    const covers = alertCoversHud(alert);
    show(alertGroup, covers);
    if (!covers) {
      fullTitle.hidden = true;
      fullSubtitle.hidden = true;
      alertSignature = "";
      return false;
    }
    const { size, radius, rect } = alertBox(alert, width, height);
    const [bx, by, bw, bh] = rect;
    const resume = kind === ALERT_RESUME_REQUIRED;
    const iconAlert = kind === ALERT_COLLISION || kind === ALERT_LANE_DEPARTURE;
    const status = resume ? 0 : alert.status;
    const color = ALERT_COLORS[status] || ALERT_COLORS[0];

    const signature = `${alert.type}|${size}|${status}|${width.toFixed(1)}`;
    if (signature !== alertSignature) {
      alertSignature = signature;
      if (resume) {
        const [r, g, b] = color;
        setFullGradient([[0, [r, g, b, 178]], [0.45, [r, g, b, 194]], [1, [Math.max(r - 18, 0), Math.max(g - 18, 0), Math.max(b - 18, 0), 212]]]);
      } else if (iconAlert) {
        setFullGradient([[0, [0x2C, 0x33, 0x3A, 0x74]], [0.25, [0x25, 0x2B, 0x32, 0x98]], [0.55, [0x1E, 0x24, 0x2A, 0xBF]], [1, [0x16, 0x1A, 0x1F, 0xD8]]]);
      }
    }
    show(alertFullBg, resume || iconAlert);
    write(alertFullBg, "width", width.toFixed(2));
    write(alertFullBg, "height", height.toFixed(2));

    const boxShown = !resume;
    show(alertShadow, boxShown);
    show(alertBoxRect, boxShown);
    if (boxShown) {
      for (const node of [alertShadow, alertBoxRect]) {
        write(node, "x", bx.toFixed(2));
        write(node, "y", by.toFixed(2));
        write(node, "width", bw.toFixed(2));
        write(node, "height", bh.toFixed(2));
        write(node, "rx", radius);
      }
      write(alertBoxRect, "fill", rgba(color));
    }

    // FCW/AEB and LDW icon, blinking, with its glow
    let iconShown = false;
    if (iconAlert) {
      const icon = alertIcon(kind === ALERT_LANE_DEPARTURE);
      iconShown = Math.floor(nowMs / icon.intervalMs) % 2 === 0;
      if (iconShown) {
        const ix = (width - icon.size) / 2;
        const iy = icon.top;
        const gw = icon.size + 116;
        placeGlow(alertIconGlow, [ix - 58, iy - 44, gw, icon.size + 96], gw * 0.56, icon.glow, 82, 34);
        write(alertIconImage, "href", asset(`${ICONS}/${icon.name}.png`));
        write(alertIconImage, "x", ix.toFixed(2));
        write(alertIconImage, "y", iy);
        write(alertIconImage, "width", icon.size);
        write(alertIconImage, "height", icon.size);
      }
    }
    show(alertIconGroup, iconShown);

    const white = [255, 255, 255, 255];
    const cy = by + bh / 2;
    let title = false;
    let subtitle = false;
    let autohold = false;
    let full = false;
    if (size === SIZE_SMALL) {
      placeText(alertTitle, alert.text1, 74, rect, white, "center", "center");
      title = true;
    } else if (size === SIZE_MID && resume) {
      autohold = true;
      const gap = 28;
      const timerWidth = 260;
      const groupLeft = (width - (AUTOHOLD_ICON + gap + timerWidth)) / 2;
      const groupTop = Math.floor((height - 1) / 2) - 150;
      write(autoholdIcon, "x", groupLeft.toFixed(2));
      write(autoholdIcon, "y", (groupTop + (AUTOHOLD_ICON - AUTOHOLD_ICON_HEIGHT) / 2).toFixed(2));
      const seconds = resumeSince == null ? 0 : (nowMs - resumeSince) / 1000;
      placeText(autoholdTimer, formatMinutesSeconds(seconds), 112,
        [groupLeft + AUTOHOLD_ICON - 1 + gap, groupTop + 10, timerWidth, AUTOHOLD_ICON - 20], white, "center", "center");
      placeText(alertSubtitle, alert.text2 || alert.text1, 66, [0, groupTop + AUTOHOLD_ICON + 26, width, 90], white, "center", "top");
      subtitle = true;
    } else if (size === SIZE_MID) {
      placeText(alertTitle, alert.text1, 88, [0, cy - 125, width, 150], white, "center", "top");
      placeText(alertSubtitle, alert.text2, 66, [0, cy + 21, width, 90], white, "center", "top");
      title = true;
      subtitle = true;
    } else if (size === SIZE_FULL) {
      full = true;
      const isLong = Array.from(alert.text1).length > 15;
      placeFullText(fullTitle, alert.text1, isLong ? 132 : 177, by + (isLong ? 240 : 270), 600);
      placeFullText(fullSubtitle, alert.text2, 88, height - (isLong ? 361 : 420), 300);
    }
    show(alertTitle, title);
    show(alertSubtitle, subtitle);
    show(autoholdIcon, autohold);
    show(autoholdTimer, autohold);
    if (!full) {
      fullTitle.hidden = true;
      fullSubtitle.hidden = true;
    }
    return true;
  }

  // ---- per-frame update ------------------------------------------------------
  function tick(nowMs) {
    lastTick = nowMs;
    // The same merged snapshot the road overlay draws from (home_drive renderActiveFrame).
    const snapshot = target.CarrotDriveLiveStateProvider?.snapshot?.() || null;
    const hud = snapshot?.hudState || target.CarrotHudState || {};
    const overlay = snapshot?.overlayState || target.CarrotOverlayState || {};
    const inputs = deriveNewUiInputs(hud, overlay, { isMetric: lastPayload?.isMetric !== false, vEgoClusterSeen });
    vEgoClusterSeen = inputs.vEgoClusterSeen;
    lastInputs = inputs;
    const wallMs = Date.now();
    drawHud(inputs, wallMs);
    drawBlindspots(inputs, wallMs);
    drawAlert(inputs, nowMs);

    const stageNode = stage();
    if (stageNode?.dataset) {
      const [gray] = cameraFilter(inputs.fresh, inputs.status, inputs.lateralOnly);
      const tone = gray > 0 ? "dim" : "";
      if ((stageNode.dataset.cnuiTone || "") !== tone) {
        if (tone) stageNode.dataset.cnuiTone = tone;
        else delete stageNode.dataset.cnuiTone;
      }
    }
  }

  function frame(timestamp) {
    rafId = 0;
    if (!visible()) return;
    if (target.document?.hidden) {
      lastTick = 0;
      schedule();
      return;
    }
    const nowMs = Number.isFinite(timestamp) ? timestamp : now();
    if (!lastTick || nowMs - lastTick >= FRAME_MIN_MS) tick(nowMs);
    schedule();
  }

  function schedule() {
    if (rafId || !visible()) return;
    if (typeof target.requestAnimationFrame === "function") rafId = target.requestAnimationFrame(frame);
  }

  function stopLoop() {
    if (rafId && typeof target.cancelAnimationFrame === "function") target.cancelAnimationFrame(rafId);
    rafId = 0;
    lastTick = 0;
  }

  function syncVisibility() {
    const shown = visible();
    root.hidden = !shown;
    under.hidden = !shown;
    const stageNode = stage();
    stageNode?.classList?.toggle?.("has-cnui", shown);
    if (shown) {
      attachUnderlay();
      relayout();
      schedule();
    } else {
      stopLoop();
      if (stageNode?.dataset) delete stageNode.dataset.cnuiTone;
    }
    return shown;
  }

  // ---- public interface (same as the cluster overlay) -------------------------
  function update(payload) {
    if (destroyed || !payload) return false;
    lastPayload = payload;
    schedule();
    return true;
  }

  function activate() {
    if (destroyed) return false;
    const changed = !active;
    active = true;
    syncVisibility();
    return changed;
  }

  function deactivate() {
    if (destroyed) return false;
    const changed = active;
    active = false;
    syncVisibility();
    return changed;
  }

  function setSuppressed(reason, value) {
    if (destroyed) return false;
    const key = String(reason || "external");
    if (value) suppressions.add(key);
    else suppressions.delete(key);
    return syncVisibility();
  }

  function startLayout(layoutTarget = target) {
    if (destroyed) return false;
    const ResizeObserverCtor = layoutTarget?.ResizeObserver;
    if (!resizeObserver && typeof ResizeObserverCtor === "function") {
      resizeObserver = new ResizeObserverCtor(() => relayout());
      resizeObserver.observe(root);
    }
    attachUnderlay();
    relayout();
    return true;
  }

  function status() {
    return Object.freeze({
      active,
      destroyed,
      visible: !root.hidden,
      suppressions: Object.freeze(Array.from(suppressions)),
      design: Object.freeze({ width, height, unitPx }),
      data: lastInputs,
    });
  }

  function destroy() {
    if (destroyed) return false;
    active = false;
    destroyed = true;
    suppressions.clear();
    stopLoop();
    resizeObserver?.disconnect?.();
    resizeObserver = null;
    const stageNode = stage();
    stageNode?.classList?.remove?.("has-cnui");
    if (stageNode?.dataset) delete stageNode.dataset.cnuiTone;
    root.hidden = true;
    under.remove?.();
    root.remove?.();
    return true;
  }

  root.hidden = true;
  under.hidden = true;
  return {
    root,
    underlay: under,
    update,
    relayout,
    activate,
    deactivate,
    setSuppressed,
    status,
    destroy,
    startClock() {},
    stopClock() {},
    startLayout,
    tick,
  };
}
