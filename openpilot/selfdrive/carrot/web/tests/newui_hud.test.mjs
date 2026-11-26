import assert from "node:assert/strict";
import test from "node:test";

import {
  THROTTLE_COLORS,
  alertBox,
  alertCoversHud,
  alertLayoutSize,
  cameraFilter,
  deriveNewUiInputs,
  fontSize,
  gapLevel,
  laneLineColors,
  laneLineHalfWidth,
  laneLineStyle,
  lfaStyle,
  pathStops,
  resolveSetSpeed,
  textBaseline,
  turnSignalOn,
  wheelStyle,
} from "../src/features/drive/contents/vision/newui/state.js";
import { hudLayout, newUiDesignSize } from "../src/features/drive/contents/vision/newui/overlay.js";
import {
  createRoadOverlayNewUiRenderer,
  newUiPathMaxDistance,
} from "../src/features/drive/contents/vision/road_overlay_newui_renderer.js";

test("C3 bottom HUD sits where v10-nui puts it on a 2160x1080 screen", () => {
  const layout = hudLayout(2160);
  assert.deepEqual(layout.setSpeed, [2160 - 104 - 156 - 28 - 236, 1080 - 96 - 118, 236, 118]);
  assert.deepEqual(layout.speed, [850, 1080 - 246 - 18, 460, 162]);
  assert.deepEqual(layout.gap, [1636 - 124 - 34, 866 + 17 + 2, 124, 84]);
  assert.deepEqual(layout.wheel(156), [2160 - 104 - 78, 1080 - 96 - 78]);
  // A wider web view keeps the right group on the right edge and the speed centred.
  assert.equal(hudLayout(2400).setSpeed[0] - hudLayout(2160).setSpeed[0], 240);
  assert.equal(hudLayout(2400).speed[0] - hudLayout(2160).speed[0], 120);
});

test("a web view narrower than the C3 shrinks the HUD instead of overlapping it", () => {
  // Phone landscape: height decides, the view is wider than a C3 screen.
  const wide = newUiDesignSize(712, 338);
  assert.equal(wide.height, 1080);
  assert.ok(wide.width > 2160);
  // An unfolded Fold or portrait (camera aspect ~1.6): width decides, 1860 units wide.
  const narrow = newUiDesignSize(390, 244);
  assert.ok(Math.abs(narrow.width - 1860) < 1e-9);
  assert.ok(narrow.height > 1080);
  const layout = hudLayout(narrow.width, narrow.height);
  const lfaRight = layout.speed[0] + layout.speed[2] - 1 - 84 + 104;
  assert.ok(lfaRight <= layout.gap[0] + 1, "LFA icon and gap icon must not overlap");
});

test("C3 text maths: raylib sizes the full line, not the em", () => {
  // Pretendard: em 2048 of a 2444-unit line, baseline 1950 below its top, every size x1.16.
  assert.ok(Math.abs(fontSize(148) - 148 * 1.16 * 2048 / 2444) < 1e-9);
  assert.ok(Math.abs(textBaseline(148, 100, 162, "bottom") - (100 + 162 - 148 * 1.16 + 148 * 1.16 * 1950 / 2444)) < 1e-9);
  assert.ok(Math.abs(textBaseline(28, 0, 34, "top") - 28 * 1.16 * 1950 / 2444) < 1e-9);
});

test("set speed, gap bars and colours follow hud.cc", () => {
  assert.deepEqual(resolveSetSpeed(0, 88, true), [88, true, true]);
  assert.deepEqual(resolveSetSpeed(100, 88, false).slice(1), [true, true]);
  assert.ok(Math.abs(resolveSetSpeed(100, 88, false)[0] - 62.1371) < 1e-6);
  assert.deepEqual(resolveSetSpeed(255, 0, true), [255, false, true]);
  assert.equal(gapLevel(0), 1);
  assert.equal(gapLevel(3), 4);
  assert.equal(gapLevel(null), 2);
  assert.deepEqual(lfaStyle(false, false, true, true)[0], [0x49, 0xD2, 0x83, 255]);
  assert.deepEqual(lfaStyle(false, true, false, false)[1], [0x67, 0xF5, 0xD1, 255]);
  assert.equal(lfaStyle(false, false, false, false)[1], null);
  // The wheel warms with torque while steering.
  assert.deepEqual(wheelStyle(0, true, false)[1], [0xF4, 0xF7, 0xFB, 0xF4]);
  assert.deepEqual(wheelStyle(1, true, false)[1], [0xFF, 0x5D, 0x57, 0xF8]);
  assert.deepEqual(wheelStyle(0.5, false, false)[1], [0x9F, 0xA8, 0xB2, 0xE6]);
  assert.equal(turnSignalOn(100), true);
  assert.equal(turnSignalOn(500), false);
});

test("camera is grey and dim only while disengaged and not steering", () => {
  assert.deepEqual(cameraFilter(true, "disengaged", false).map((v) => +v.toFixed(2)), [1, 0.38]);
  assert.deepEqual(cameraFilter(true, "disengaged", true), [0, 0]);
  assert.deepEqual(cameraFilter(true, "engaged", false), [0, 0]);
});

test("lane lines: crossing line hidden, edges mint during a lane change, green in lane mode", () => {
  // Left lane change: line 1 is crossed, lines 2 and 0 highlight.
  assert.equal(laneLineStyle(1, 2, 1, false), "skip");
  assert.equal(laneLineStyle(2, 2, 1, false), "change");
  assert.equal(laneLineStyle(0, 3, 1, false), "change");
  assert.equal(laneLineStyle(3, 2, 1, false), "plain");
  assert.equal(laneLineStyle(1, 0, 0, true), "mode");
  assert.equal(laneLineStyle(0, 0, 0, true), "plain");
  assert.deepEqual(laneLineColors("plain", 0.9)[0], [255, 255, 255, 179]);
  assert.equal(laneLineColors("change", 0.5)[2], 9);
  assert.equal(laneLineColors("mode", 0.5)[2], 10);
  assert.equal(laneLineHalfWidth(0.1, 0, false), 0.028);
  assert.equal(laneLineHalfWidth(1, 1, true), 0.025 * 1.7);
});

test("path colours: throttle green, mint alone, white in lane mode, grey while disengaged", () => {
  assert.deepEqual(pathStops(false, false, false)[0], [0, THROTTLE_COLORS[0]]);
  assert.deepEqual(THROTTLE_COLORS[0], [13, 248, 122, 102]);
  assert.deepEqual(pathStops(true, false, false)[0], [0, [0x67, 0xF5, 0xD1, 110]]);
  assert.deepEqual(pathStops(false, true, false)[1], [0.55, [0xF6, 0xF8, 0xFB, 58]]);
  const grey = pathStops(false, false, true)[0][1];
  assert.equal(grey[0], grey[1]);
  assert.equal(grey[1], grey[2]);
  assert.equal(grey[3], 102);
});

test("alerts: one-line mid alerts use the small box, FCW and LDW become mid, steer limit stays on the wheel", () => {
  const mid = { text1: "조향 해제", text2: "", type: "x/warning", size: 2, status: 1 };
  assert.equal(alertLayoutSize(mid), 1);
  assert.deepEqual(alertBox(mid, 2160).rect, [40, 1080 - 271 + 40, 2080, 191]);
  assert.equal(alertLayoutSize({ ...mid, type: "fcw/permanent", size: 3, text2: "추돌 위험" }), 2);
  assert.equal(alertLayoutSize({ ...mid, type: "resumeRequired/warning", size: 1 }), 2);
  const full = { text1: "TAKE CONTROL", text2: "", type: "x/immediateDisable", size: 3, status: 2 };
  assert.deepEqual(alertBox(full, 2160), { size: 3, radius: 0, rect: [0, 0, 2160, 1080] });
  assert.equal(alertCoversHud({ ...mid, type: "steerSaturated/warning" }), false);
  assert.equal(alertCoversHud(null), false);
});

test("HUD inputs come from the compact state the web already receives", () => {
  const inputs = deriveNewUiInputs({
    carState: { vCruiseCluster: 0, vEgoCluster: 10, steeringAngleDeg: 12.5, leftBlinker: true, rightBlindspot: true },
    controlsState: { vCruiseCluster: 88, lateralOutput: -0.4, activeLaneLine: true },
    selfdriveState: { enabled: true, personality: 2, alertSize: 0 },
    carrotMan: { activeCarrot: 2 },
  }, {
    carControl: { latActive: true, longActive: true },
  });
  assert.equal(inputs.status, "engaged");
  assert.equal(inputs.setSpeed, 88);
  assert.equal(inputs.speed, 36);
  assert.equal(inputs.torque, 0.4);
  assert.equal(inputs.steeringAngle, -12.5);
  assert.equal(inputs.gapLevel, 3);
  assert.equal(inputs.carrotLevel, 2);
  assert.equal(inputs.leftSignal, true);
  assert.equal(inputs.rightBlindspot, true);
  assert.equal(inputs.laneMode, true);
  assert.equal(inputs.alert, null);

  const lateral = deriveNewUiInputs({ carState: {}, selfdriveState: { enabled: false } }, { carControl: { latActive: true } });
  assert.equal(lateral.lateralOnly, true);
  assert.equal(lateral.status, "disengaged");

  const idle = deriveNewUiInputs({}, {});
  assert.equal(idle.fresh, false);
  assert.equal(idle.cruiseSet, false);
  assert.equal(idle.speed, 0);
});

test("the path stops short of a lead like the C3", () => {
  const model = { position: { x: [0, 20, 60, 90], y: [0, 0, 0, 0], z: [0, 0, 0, 0] } };
  assert.equal(newUiPathMaxDistance(model, null), 90);
  assert.equal(newUiPathMaxDistance(model, { status: true, dRel: 20 }), 30);
});

test("new ui road overlay: lanes and red edges only while engaged, grey path otherwise", () => {
  const drawn = [];
  const renderer = createRoadOverlayNewUiRenderer({
    isPerformanceActive: () => true,
    geometry: {
      drawPolygon: (points, fill, stroke, width) => drawn.push({ fill, stroke, width }),
      buildRibbon: () => ({ polygon: [{ x: 0, y: 0 }, { x: 1, y: 0 }, { x: 1, y: 1 }] }),
      smoothRibbon: (_key, ribbon) => ribbon,
    },
  });
  const model = {
    position: { x: [0, 50], y: [0, 0], z: [0, 0] },
    laneLines: [{}, {}, {}, {}],
    laneLineProbs: [0.0, 0.9, 0.5, 0.0],
    roadEdges: [{}, {}],
    roadEdgeStds: [0.2, 1.0],
  };
  const view = { left: 0, top: 0, right: 200, bottom: 108 };

  let mode = renderer.draw(model, {}, { selfdriveState: { enabled: false } }, null, view);
  assert.equal(mode.hidden, true);
  assert.equal(drawn.length, 1);
  assert.equal(drawn[0].fill.type, "vertical-stops");
  assert.equal(drawn[0].fill.stops[0].r, drawn[0].fill.stops[0].g);

  drawn.length = 0;
  mode = renderer.draw(model, { carControl: { latActive: true, longActive: true } },
    { selfdriveState: { enabled: true }, controlsState: { activeLaneLine: true } }, null, view);
  assert.equal(mode.laneMode, true);
  // 4 lanes (0 and 3 at the 0.028 m minimum, alpha 0 -> skipped), 1 red edge, the path.
  const fills = drawn.map((entry) => entry.fill);
  assert.ok(fills.includes("rgba(255,0,0,0.800)"));
  const laneModeLine = drawn.find((entry) => entry.stroke);
  assert.equal(laneModeLine.width, 10 * 0.1);
  assert.equal(drawn.at(-1).fill.stops[1].r, 0xF6);
});
