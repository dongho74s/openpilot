/* Road overlay for Carrot Vision "new ui 1": the C3/C3X (tici) NewUi=1 look.
 *
 * C3 reference: selfdrive/ui/onroad/v10_nui/model.py and state.py.
 *  - Lane lines: thin white ribbons (alpha from confidence, max 0.7); in lane
 *    mode the ego edges turn green with a glow; during a lane change the line
 *    being crossed disappears and the current/target edges turn mint.
 *  - Road edges: red, alpha 1 - std.
 *  - Lanes and edges are hidden while disengaged (unless steering on its own).
 *  - Path: throttle gradient (mint when steering alone, white in lane mode),
 *    grey while disengaged, cut short in front of a lead.
 *  - Lead boxes only when ShowPathEnd is on: drawn by the classic lead renderer.
 *
 * It draws through the same batched geometry helpers as the classic renderer
 * (see road_overlay_geometry_renderer.js for the invariants it keeps). */

import {
  DESIGN_HEIGHT,
  LANE_SKIP,
  laneLineColors,
  laneLineHalfWidth,
  laneLineStyle,
  lateralOnlyActive,
  pathStops,
} from "./newui/state.js";

const MIN_DRAW_DISTANCE = 10;
const MAX_DRAW_DISTANCE = 100;
const PATH_HALF_WIDTH = 0.9;
const PATH_SMOOTH_TAU_MS = 110;
const LANE_SMOOTH_TAU_MS = 140;
const GEOMETRY_QUALITY_LANE = "lane";
const GEOMETRY_QUALITY_ROAD_EDGE = "road-edge";

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

function finiteNumber(value, fallback = 0) {
  const number = Number(value);
  return Number.isFinite(number) ? number : fallback;
}

function rgbaString([r, g, b, a]) {
  return `rgba(${r},${g},${b},${(a / 255).toFixed(3)})`;
}

// clip(path_x[-1], MIN_DRAW_DISTANCE, MAX_DRAW_DISTANCE)
export function newUiSceneMaxDistance(model) {
  const xs = Array.isArray(model?.position?.x) ? model.position.x : [];
  return clamp(finiteNumber(xs[xs.length - 1], MIN_DRAW_DISTANCE), MIN_DRAW_DISTANCE, MAX_DRAW_DISTANCE);
}

// The path stops short of a lead: lead_d = 2·dRel, max = lead_d − min(0.35·lead_d, 10).
export function newUiPathMaxDistance(model, leadOne) {
  let maxDistance = newUiSceneMaxDistance(model);
  const dRel = finiteNumber(leadOne?.dRel, NaN);
  if (leadOne?.status && Number.isFinite(dRel)) {
    const leadD = dRel * 2;
    maxDistance = clamp(leadD - Math.min(leadD * 0.35, 10), 0, maxDistance);
  }
  return maxDistance;
}

// What the C3 overlay shows for this sample (shared with the lead-box gate in home_drive).
export function newUiOverlayMode(overlayState, hudState) {
  const enabled = Boolean(hudState?.selfdriveState?.enabled);
  const control = overlayState?.carControl || {};
  const lateralOnly = lateralOnlyActive(enabled, Boolean(control.longActive), Boolean(control.latActive));
  return {
    enabled,
    lateralOnly,
    hidden: !enabled && !lateralOnly,
    laneMode: Boolean(hudState?.controlsState?.activeLaneLine),
  };
}

export function createRoadOverlayNewUiRenderer(options = {}) {
  const context = options.context || null;
  const geometry = options.geometry || {};
  const isPerformanceActive = typeof options.isPerformanceActive === "function"
    ? options.isPerformanceActive
    : () => false;
  const getCachedGradient = typeof options.getCachedGradient === "function"
    ? options.getCachedGradient
    : (_key, factory) => factory();
  const pathZOffset = finiteNumber(options.pathZOffset, 1.22);
  if (
    typeof geometry.drawPolygon !== "function"
    || typeof geometry.buildRibbon !== "function"
    || typeof geometry.smoothRibbon !== "function"
  ) {
    return null;
  }

  function drawLaneLines(model, overlayState, calibTransform, mode, unit) {
    const laneLines = Array.isArray(model?.laneLines) ? model.laneLines : [];
    const probs = Array.isArray(model?.laneLineProbs) ? model.laneLineProbs : [];
    const lateralPlan = overlayState?.lateralPlan || {};
    const laneChangeState = finiteNumber(lateralPlan.laneChangeState, 0);
    const direction = finiteNumber(lateralPlan.laneChangeDirection, 0);
    const maxDistance = newUiSceneMaxDistance(model);
    for (let i = 0; i < laneLines.length; i += 1) {
      const style = laneLineStyle(i, laneChangeState, direction, mode.laneMode);
      if (style === LANE_SKIP) continue;
      const prob = finiteNumber(probs[i], 0);
      const [fill, outline, outlineWidth] = laneLineColors(style, prob);
      if (fill[3] <= 0 && !outline) continue;
      const raw = geometry.buildRibbon(calibTransform, laneLines[i], laneLineHalfWidth(prob, i, mode.laneMode), 0,
        maxDistance, true, 0, GEOMETRY_QUALITY_LANE);
      const ribbon = geometry.smoothRibbon(`newui:lane:${i}`, raw, LANE_SMOOTH_TAU_MS);
      if (!(ribbon?.polygon?.length >= 3)) continue;
      geometry.drawPolygon(ribbon.polygon, fill[3] > 0 ? rgbaString(fill) : "", outline ? rgbaString(outline) : "",
        outline ? outlineWidth * unit : 0);
    }
  }

  function drawRoadEdges(model, calibTransform) {
    const roadEdges = Array.isArray(model?.roadEdges) ? model.roadEdges.slice(0, 2) : [];
    const stds = Array.isArray(model?.roadEdgeStds) ? model.roadEdgeStds : [];
    const maxDistance = newUiSceneMaxDistance(model);
    for (let i = 0; i < roadEdges.length; i += 1) {
      const alpha = clamp(1 - finiteNumber(stds[i], 1), 0, 1);
      if (alpha <= 0.004) continue;
      const ribbon = geometry.buildRibbon(calibTransform, roadEdges[i], 0.025, 0, maxDistance, true, 0, GEOMETRY_QUALITY_ROAD_EDGE);
      if (ribbon?.polygon?.length >= 3) geometry.drawPolygon(ribbon.polygon, `rgba(255,0,0,${alpha.toFixed(3)})`);
    }
  }

  function pathFill(stops, view) {
    const gpuStops = stops.map(([at, [r, g, b, a]]) => ({ at, r, g, b, a: a / 255 }));
    if (isPerformanceActive()) {
      return { type: "vertical-stops", top: view.top, bottom: view.bottom, stops: gpuStops };
    }
    if (!context?.createLinearGradient) return rgbaString(stops[0][1]);
    const key = `newui-path:${Math.round(view.top)}:${Math.round(view.bottom)}:${stops.map(([at, c]) => `${at}/${c.join(",")}`).join(";")}`;
    return getCachedGradient(key, () => {
      const gradient = context.createLinearGradient(0, view.bottom, 0, view.top);
      for (const stop of gpuStops) gradient.addColorStop(stop.at, `rgba(${stop.r},${stop.g},${stop.b},${stop.a.toFixed(3)})`);
      return gradient;
    });
  }

  function drawPath(model, overlayState, calibTransform, view, mode) {
    const path = model?.position;
    if (!path || !Array.isArray(path.x) || path.x.length < 2) return;
    const maxDistance = newUiPathMaxDistance(model, overlayState?.radarState?.leadOne);
    if (!(maxDistance > 0)) return;
    const raw = geometry.buildRibbon(calibTransform, path, PATH_HALF_WIDTH, pathZOffset, maxDistance, false);
    const ribbon = geometry.smoothRibbon("newui:path", raw, PATH_SMOOTH_TAU_MS);
    if (!ribbon?.polygon || ribbon.polygon.length < 3) return;
    geometry.drawPolygon(ribbon.polygon, pathFill(pathStops(mode.lateralOnly, mode.laneMode, mode.hidden), view));
  }

  /* view: the visible part of the video in video pixels { left, top, right,
   * bottom }; one C3 screen pixel is view height / 1080. */
  function draw(model, overlayState, hudState, calibTransform, view) {
    if (!model) return null;
    const mode = newUiOverlayMode(overlayState, hudState);
    const unit = Math.max(1e-3, (view.bottom - view.top) / DESIGN_HEIGHT);
    if (!mode.hidden) {
      drawLaneLines(model, overlayState, calibTransform, mode, unit);
      drawRoadEdges(model, calibTransform);
    }
    drawPath(model, overlayState, calibTransform, view, mode);
    return mode;
  }

  return Object.freeze({ draw, drawLaneLines, drawRoadEdges, drawPath });
}

export const DriveVisionRoadOverlayNewUiRenderer = Object.freeze({ create: createRoadOverlayNewUiRenderer });

export function installDriveVisionRoadOverlayNewUiRendererFacade(target = globalThis) {
  target.DriveVisionRoadOverlayNewUiRenderer = DriveVisionRoadOverlayNewUiRenderer;
  return DriveVisionRoadOverlayNewUiRenderer;
}
