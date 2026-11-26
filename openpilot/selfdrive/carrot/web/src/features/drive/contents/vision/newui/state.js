"use strict";

/* Carrot Vision "new ui 1": the C3/C3X (tici) NewUi=1 driving screen ("v10-nui"),
 * rebuilt for the web from the same rules. Everything here is pure: it turns the
 * compact realtime state the web already receives into what the C3 would show.
 *
 * C3 reference: openpilot/selfdrive/ui/onroad/v10_nui/ (state.py, hud.py,
 * model.py, alerts.py, view.py). Coordinates are the C3 screen: 1080 units tall,
 * width follows the view (2160 on the device). */

export const DESIGN_HEIGHT = 1080;
export const SET_SPEED_NA = 255;
export const KM_TO_MILE = 0.621371;
export const MS_TO_KPH = 3.6;
export const MS_TO_MPH = 2.236936;
export const TURN_SIGNAL_CYCLE_MS = 800;
export const TURN_SIGNAL_ON_MS = 420;
export const BLINDSPOT_BLINK_MS = 120;
export const CAMERA_DIM_BRIGHTNESS = 0.62;

export const STATUS_DISENGAGED = "disengaged";
export const STATUS_ENGAGED = "engaged";
export const STATUS_OVERRIDE = "override";

export const LANE_CHANGE_STARTING = 2;
export const LANE_CHANGE_FINISHING = 3;
export const DIRECTION_LEFT = 1;
export const DIRECTION_RIGHT = 2;

export const LATERAL_ONLY_MINT = Object.freeze([0x67, 0xF5, 0xD1, 255]);
export const LANE_CHANGE_MINT = Object.freeze([0x53, 0xEE, 0xB6, 255]);
export const OVERRIDE_BLUE = Object.freeze([0x4F, 0x8D, 0xFF, 255]);
export const ENGAGED_GREEN = Object.freeze([0x49, 0xD2, 0x83, 255]);

/* Text: the device draws every size x1.16 with Pretendard SemiBold (all weights),
 * and raylib sizes a font by its full line (ascent - descent = 2444 of 2048
 * units), not its em. So a C3 size N is a CSS font-size of N*1.16*2048/2444, its
 * line is N*1.16 tall and the baseline sits 1950/2444 of the line below its top. */
export const TEXT_SCALE = 1.16;
const EM_PER_LINE = 2048 / 2444;
const BASELINE_PER_LINE = 1950 / 2444;

export function fontSize(size) {
  return size * TEXT_SCALE * EM_PER_LINE;
}

export function lineHeight(size) {
  return size * TEXT_SCALE;
}

// v10_nui/draw.draw_text vertical alignment, as an SVG baseline y.
export function textBaseline(size, y, h, vAlign = "center") {
  const line = lineHeight(size);
  const top = vAlign === "top" ? y : vAlign === "bottom" ? y + h - line : y + (h - line) / 2;
  return top + line * BASELINE_PER_LINE;
}

export function finite(value) {
  if (value === null || value === undefined || value === "") return null;
  const number = Number(value);
  return Number.isFinite(number) ? number : null;
}

export function clamp(value, lo, hi) {
  return value < lo ? lo : value > hi ? hi : value;
}

export function withAlpha(color, alpha) {
  return [color[0], color[1], color[2], alpha];
}

// hud.cc blendColor: channel-wise mix with std::lround.
export function blend(a, b, t) {
  const k = clamp(t, 0, 1);
  return a.map((x, i) => Math.floor(x + (b[i] - x) * k + 0.5));
}

// model.cc grayToneColor: luminance * 0.82, alpha kept.
export function grayTone([r, g, b, a]) {
  const toned = clamp((0.299 * r + 0.587 * g + 0.114 * b) / 255 * 0.82, 0, 1);
  const v = Math.round(toned * 255);
  return [v, v, v, a];
}

export function hsla(hDeg, s, l, a) {
  const h = (((hDeg / 360) % 1) + 1) % 1;
  if (s === 0) {
    const v = Math.round(l * 255);
    return [v, v, v, Math.round(a * 255)];
  }
  const q = l < 0.5 ? l * (1 + s) : l + s - l * s;
  const p = 2 * l - q;
  const channel = (t) => {
    let x = t;
    if (x < 0) x += 1;
    if (x > 1) x -= 1;
    if (x < 1 / 6) return p + (q - p) * 6 * x;
    if (x < 1 / 2) return q;
    if (x < 2 / 3) return p + (q - p) * (2 / 3 - x) * 6;
    return p;
  };
  return [Math.round(channel(h + 1 / 3) * 255), Math.round(channel(h) * 255), Math.round(channel(h - 1 / 3) * 255), Math.round(a * 255)];
}

// model.cc updatePathGradient
export const THROTTLE_COLORS = Object.freeze([hsla(148, 0.94, 0.51, 0.4), hsla(112, 1.0, 0.68, 0.35), hsla(112, 1.0, 0.68, 0.0)]);
export const NO_THROTTLE_COLORS = Object.freeze([hsla(148, 0.62, 0.70, 0.4), hsla(112, 0.72, 0.72, 0.35), hsla(112, 0.72, 0.72, 0.0)]);

// Steering assist without openpilot engaged (always-on lateral): mint instead of grey.
export function lateralOnlyActive(enabled, longActive, latActive, latEnabled = false) {
  return !enabled && !longActive && (latActive || latEnabled);
}

// (grayscale mix, dim amount) for the road camera: disengaged and not steering is grey and 62 % bright.
export function cameraFilter(started, status, lateralOnly) {
  if (started && status === STATUS_DISENGAGED && !lateralOnly) return [1, 1 - CAMERA_DIM_BRIGHTNESS];
  return [0, 0];
}

// hud.cc drawSteeringWheelIcon: [button background, wheel tint].
export function wheelStyle(torque, steeringActive, lateralOverride) {
  if (lateralOverride) return [[0x0A, 0x12, 0x1F, 0xB8], OVERRIDE_BLUE];
  if (steeringActive) {
    const white = [0xF4, 0xF7, 0xFB, 0xF4];
    const warm = [0xFF, 0xC7, 0x58, 0xF6];
    const hot = [0xFF, 0x5D, 0x57, 0xF8];
    const t = clamp(torque, 0, 1);
    const tint = t < 0.65 ? blend(white, warm, t / 0.65 * 0.35) : blend(warm, hot, (t - 0.65) / 0.35);
    return [[0x0D, 0x16, 0x12, 0xB6], tint];
  }
  return [[0x0A, 0x10, 0x16, 0xA8], [0x9F, 0xA8, 0xB2, 0xE6]];
}

// hud.cc drawLfaIcon: [icon tint, glow colour or null].
export function lfaStyle(longOverride, lateralOnly, enabled, engageable) {
  if (longOverride) return [OVERRIDE_BLUE, OVERRIDE_BLUE];
  if (lateralOnly) return [LATERAL_ONLY_MINT, LATERAL_ONLY_MINT];
  if (enabled) return [ENGAGED_GREEN, ENGAGED_GREEN];
  if (engageable) return [[0xF4, 0xF7, 0xFB, 0xF4], null];
  return [[0x92, 0x9D, 0xA8, 0xE6], null];
}

// hud.cc drawSetSpeed: [value colour, "SET" colour].
export function setSpeedColors(cruiseSet, status) {
  if (!cruiseSet || status === STATUS_DISENGAGED) return [[0xD2, 0xD9, 0xE1, 0xCA], [0xA7, 0xB0, 0xBA, 0xB8]];
  if (status === STATUS_OVERRIDE) return [[0xF3, 0xE6, 0xCB, 0xE2], [0xD6, 0xC3, 0x98, 0xBA]];
  return [[0xF2, 0xF5, 0xF9, 0xF2], [0xD1, 0xD8, 0xDF, 0xD0]];
}

// hud.cc drawCurrentSpeed: [speed colour, unit colour].
export function currentSpeedColors(status) {
  if (status === STATUS_DISENGAGED) return [[0xE2, 0xE7, 0xEC, 0xE6], [0xC5, 0xCF, 0xD8, 0xC8]];
  if (status === STATUS_OVERRIDE) return [[0xF6, 0xF8, 0xFB, 0xF4], [0xE8, 0xDC, 0xC2, 0xC4]];
  return [[0xF6, 0xF8, 0xFB, 0xF4], [0xDF, 0xE5, 0xEA, 0xD4]];
}

export function gapTint(status) {
  if (status === STATUS_DISENGAGED) return [0xD4, 0xDD, 0xE6, 0xDA];
  if (status === STATUS_OVERRIDE) return [0xF2, 0xE6, 0xC9, 0xEA];
  return [0xF6, 0xF8, 0xFB, 0xEE];
}

// 1..4 bars from the longitudinal personality (aggressive=0 .. moreRelaxed=3).
export function gapLevel(personality, fallbackPersonality = 1) {
  const value = personality == null ? fallbackPersonality : personality;
  return clamp(Math.trunc(value) + 1, 1, 4);
}

// [displayed set speed, cruise set, cruise available]; km/h in, km/h or mph out.
export function resolveSetSpeed(vCruiseCluster, vCruiseFallback, isMetric) {
  let speed = vCruiseCluster === 0 ? vCruiseFallback : vCruiseCluster;
  const cruiseSet = speed > 0 && speed < SET_SPEED_NA;
  const available = speed !== -1;
  if (cruiseSet && !isMetric) speed *= KM_TO_MILE;
  return [speed, cruiseSet, available];
}

export function formatMinutesSeconds(seconds) {
  const total = Math.max(0, Math.trunc(seconds));
  return `${Math.trunc(total / 60)}:${String(total % 60).padStart(2, "0")}`;
}

export const ALERT_NORMAL = "normal";
export const ALERT_STEER_SATURATED = "steer_saturated";
export const ALERT_RESUME_REQUIRED = "resume_required";
export const ALERT_COLLISION = "collision";
export const ALERT_LANE_DEPARTURE = "lane_departure";
export const ALERT_LEAD_DEPARTING = "lead_departing";

// alerts.cc type matching (case-insensitive substrings of selfdriveState.alertType).
export function alertKind(alertType) {
  const t = String(alertType || "").toLowerCase();
  if (t.includes("leaddeparting")) return ALERT_LEAD_DEPARTING;
  if (t.includes("steersaturated")) return ALERT_STEER_SATURATED;
  if (t.includes("resumerequired")) return ALERT_RESUME_REQUIRED;
  if (t.includes("fcw") || t.includes("aeb")) return ALERT_COLLISION;
  if (t.includes("ldw")) return ALERT_LANE_DEPARTURE;
  return ALERT_NORMAL;
}

export function turnSignalOn(nowMs) {
  return ((nowMs % TURN_SIGNAL_CYCLE_MS) + TURN_SIGNAL_CYCLE_MS) % TURN_SIGNAL_CYCLE_MS < TURN_SIGNAL_ON_MS;
}

export function blindspotBlinkVisible(nowMs) {
  return Math.floor(nowMs / BLINDSPOT_BLINK_MS) % 2 === 0;
}

export const LANE_PLAIN = "plain";
export const LANE_SKIP = "skip";
export const LANE_CHANGE = "change";
export const LANE_MODE = "mode";

// model.cc drawLaneLines: hide the line being crossed, highlight the current and target lane edges.
export function laneLineStyle(index, laneChangeState, direction, laneModeActive) {
  const changing = (laneChangeState === LANE_CHANGE_STARTING || laneChangeState === LANE_CHANGE_FINISHING)
    && (direction === DIRECTION_LEFT || direction === DIRECTION_RIGHT);
  if (changing) {
    const crossing = direction === DIRECTION_LEFT ? 1 : 2;
    const highlight = direction === DIRECTION_LEFT ? [2, 0] : [1, 3];
    if (index === crossing) return LANE_SKIP;
    if (highlight.includes(index)) return LANE_CHANGE;
  }
  if (laneModeActive && (index === 1 || index === 2)) return LANE_MODE;
  return LANE_PLAIN;
}

// model.cc update_model: thicker than stock; the ego edges thicker still in lane mode.
export function laneLineHalfWidth(prob, index, laneModeActive) {
  const width = 0.025 * prob;
  if (laneModeActive && (index === 1 || index === 2)) return Math.max(width * 1.7, 0.034);
  return Math.max(width * 1.25, 0.028);
}

// [fill, outline or null, outline width in screen px] for one lane line.
export function laneLineColors(style, prob) {
  if (style === LANE_CHANGE) {
    const alpha = clamp(0.22 + prob * 0.62, 0.22, 0.88);
    return [withAlpha(LANE_CHANGE_MINT, Math.round(alpha * 255)), withAlpha(LANE_CHANGE_MINT, 128), 9];
  }
  if (style === LANE_MODE) {
    const alpha = clamp(0.15 + prob * 0.65, 0.15, 0.8);
    const glowAlpha = clamp(alpha * 0.5, 0.16, 0.34);
    const fill = [Math.round(0.14 * 255), Math.round(0.90 * 255), Math.round(0.38 * 255), Math.round(alpha * 255)];
    const glow = [Math.round(0.24 * 255), Math.round(0.95 * 255), Math.round(0.46 * 255), Math.round(glowAlpha * 255)];
    return [fill, glow, 10];
  }
  return [[255, 255, 255, Math.round(clamp(prob, 0, 0.7) * 255)], null, 0];
}

/* model.cc drawPath: [[position, rgba]...] with 0 = bottom of the view, 1 = top.
 * The web never receives longitudinalPlan.allowThrottle or experimentalMode, so
 * the normal path always uses the throttle colours. */
export function pathStops(lateralOnly, laneModeActive, hidden) {
  let stops;
  if (lateralOnly) {
    stops = [[0, withAlpha(LATERAL_ONLY_MINT, 110)], [0.55, withAlpha(LATERAL_ONLY_MINT, 84)], [1, withAlpha(LATERAL_ONLY_MINT, 8)]];
  } else if (laneModeActive) {
    const white = [0xF6, 0xF8, 0xFB, 0];
    stops = [[0, withAlpha(white, 102)], [0.55, withAlpha(white, 58)], [1, white]];
  } else {
    stops = [[0, THROTTLE_COLORS[0]], [0.5, THROTTLE_COLORS[1]], [1, THROTTLE_COLORS[2]]];
  }
  return hidden ? stops.map(([at, color]) => [at, grayTone(color)]) : stops;
}

// ---- alerts (alerts.cc) -------------------------------------------------------
export const SIZE_NONE = 0;
export const SIZE_SMALL = 1;
export const SIZE_MID = 2;
export const SIZE_FULL = 3;
export const ALERT_HEIGHTS = Object.freeze({ [SIZE_SMALL]: 271, [SIZE_MID]: 420 });
export const ALERT_MARGIN = 40;
export const ALERT_RADIUS = 30;
export const ALERT_COLORS = Object.freeze({
  0: Object.freeze([0x15, 0x15, 0x15, 0xF1]),
  1: Object.freeze([0xDA, 0x6F, 0x25, 0xF1]),
  2: Object.freeze([0xC9, 0x22, 0x31, 0xF1]),
});

export function deriveAlert(selfdriveState) {
  if (!selfdriveState) return null;
  const size = finite(selfdriveState.alertSize) ?? SIZE_NONE;
  if (size === SIZE_NONE) return null;
  return Object.freeze({
    text1: String(selfdriveState.alertText1 || ""),
    text2: String(selfdriveState.alertText2 || ""),
    type: String(selfdriveState.alertType || ""),
    size,
    status: finite(selfdriveState.alertStatus) ?? 0,
  });
}

/* alerts.layout_size: FCW/AEB and LDW draw as mid alerts with an icon, auto-hold
 * as its own mid layout, and a mid box with one line uses the small box. */
export function alertLayoutSize(alert) {
  const kind = alertKind(alert.type);
  if (kind === ALERT_LEAD_DEPARTING) return SIZE_FULL;
  if (kind === ALERT_RESUME_REQUIRED && alert.size !== SIZE_NONE) return SIZE_MID;
  const size = (kind === ALERT_COLLISION || kind === ALERT_LANE_DEPARTURE) && alert.size !== SIZE_NONE ? SIZE_MID : alert.size;
  if (size === SIZE_MID && !alert.text2.trim()) return SIZE_SMALL;
  return size;
}

// A steer-limit alert only grows the wheel; every other alert covers the HUD.
export function alertCoversHud(alert) {
  return Boolean(alert) && alertKind(alert.type) !== ALERT_STEER_SATURATED && alertLayoutSize(alert) !== SIZE_NONE;
}

// The alert box in design units: [x, y, w, h] and its corner radius.
export function alertBox(alert, width, height = DESIGN_HEIGHT) {
  const size = alertLayoutSize(alert);
  const boxHeight = ALERT_HEIGHTS[size] ?? height;
  const margin = size === SIZE_FULL ? 0 : ALERT_MARGIN;
  return {
    size,
    radius: size === SIZE_FULL ? 0 : ALERT_RADIUS,
    rect: [margin, height - boxHeight + margin, width - margin * 2, boxHeight - margin * 2],
  };
}

// alerts.alert_icon: [icon, size, blink interval ms, icon top] for FCW/AEB or LDW.
export function alertIcon(laneDeparture) {
  return laneDeparture
    ? { name: "lanecrossing", size: 292, intervalMs: 320, top: 188, glow: [0xFF, 0xC3, 0x53, 255] }
    : { name: "collision", size: 348, intervalMs: 120, top: 164, glow: [0xFF, 0x5D, 0x57, 255] };
}

/* One snapshot of everything the HUD draws, from the web's compact state.
 * hud: window.CarrotHudState, overlay: window.CarrotOverlayState. */
export function deriveNewUiInputs(hud = {}, overlay = {}, options = {}) {
  const carState = hud.carState || null;
  const selfdriveState = hud.selfdriveState || null;
  const controlsState = hud.controlsState || {};
  const carControl = overlay.carControl || hud.carControl || {};
  const isMetric = options.isMetric !== false;
  const fresh = Boolean(carState);
  const enabled = Boolean(selfdriveState?.enabled);
  // selfdriveState.state (override) and engageable are not in the compact state.
  const status = enabled ? STATUS_ENGAGED : STATUS_DISENGAGED;
  const longActive = Boolean(carControl.longActive);
  const latActive = Boolean(carControl.latActive);
  const lateralOnly = fresh && lateralOnlyActive(enabled, longActive, latActive);
  const alert = deriveAlert(selfdriveState);
  const [setSpeed, cruiseSet, cruiseAvailable] = fresh
    ? resolveSetSpeed(finite(carState.vCruiseCluster) ?? 0, finite(controlsState.vCruiseCluster) ?? 0, isMetric)
    : [SET_SPEED_NA, false, true];
  const vEgoCluster = finite(carState?.vEgoCluster) ?? 0;
  const vEgo = options.vEgoClusterSeen || vEgoCluster !== 0 ? vEgoCluster : (finite(carState?.vEgo) ?? 0);
  const personality = finite(selfdriveState?.personality);
  const lateralOutput = finite(controlsState.lateralOutput);
  const angle = finite(carState?.steeringAngleDeg);
  return Object.freeze({
    fresh,
    enabled,
    engageable: enabled,
    status,
    longActive,
    latActive,
    lateralOnly,
    longOverride: false,
    latOverride: false,
    isMetric,
    setSpeed,
    cruiseSet,
    cruiseAvailable,
    vEgoClusterSeen: Boolean(options.vEgoClusterSeen || vEgoCluster !== 0),
    speed: fresh ? Math.max(0, vEgo * (isMetric ? MS_TO_KPH : MS_TO_MPH)) : 0,
    // hud.cc: |output| of the active lateral controller (saturation is not sent).
    torque: lateralOutput == null ? 0 : clamp(Math.abs(lateralOutput), 0, 1),
    steeringAngle: fresh && angle != null ? -angle : 0,
    gapLevel: gapLevel(personality),
    carrotLevel: finite(hud.carrotMan?.activeCarrot) ?? 0,
    leftSignal: fresh && Boolean(carState.leftBlinker),
    rightSignal: fresh && Boolean(carState.rightBlinker),
    leftBlindspot: fresh && Boolean(carState.leftBlindspot),
    rightBlindspot: fresh && Boolean(carState.rightBlindspot),
    steerLimitWarning: alertKind(alert?.type) === ALERT_STEER_SATURATED,
    laneMode: Boolean(controlsState.activeLaneLine),
    alert,
  });
}
