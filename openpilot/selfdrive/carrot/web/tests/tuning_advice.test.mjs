import assert from "node:assert/strict";
import test from "node:test";

import { applyState, describe, formatChange, formatValue, metricLines, recommendationText, statusNotes } from "../src/features/tools/tuning_advice.js";

const change = { status: "change", param: "LateralAccelLimit", from: 35, to: 40, delta: { better: 4, windows: 5, err_pct: -9.5, outward_cm: -3.2 } };
const base = { ok: true, onroad: false, running: false, can_run: true, status: { state: "done" }, current: { LateralAccelLimit: 35 } };

test("values show in the units of the Carrot web setting", () => {
  assert.equal(formatValue("LateralAccelLimit", 35), "3.5 m/s²");
  assert.equal(formatValue("LateralTorqueKiV", 70), "0.70");
  assert.equal(formatValue("LatTorqueLimitPFilter", 0, (k) => ({ off: "끔" })[k]), "끔");
  assert.equal(formatValue("LatTorqueLimitPFilter", 30, (k, v) => `${v.v}초`), "0.30초");
  assert.equal(formatValue("LatTorqueLimitPFilter", 30), "0.30 s");
  const ko = (k) => ({ name_LateralTorqueKpV: "조향 P" })[k] ?? k;
  assert.equal(formatChange("LateralTorqueKpV", 160, 170, ko), "조향 P 1.60 → 1.70");
});

test("the heading follows the job and the recommendation", () => {
  assert.deepEqual(describe({ ...base, running: true, status: { progress: 42 } }), { state: "running", pct: 42 });
  assert.deepEqual(describe({ ...base, result: null }), { state: "wait" });
  assert.deepEqual(describe({ ...base, status: { state: "error" }, result: null }), { state: "error" });
  assert.deepEqual(describe({ ...base, status: { state: "error" }, result: { recommendation: change } }), { state: "error" });
  assert.deepEqual(describe({ ...base, result: { recommendation: change } }), { state: "change" });
  assert.deepEqual(describe({ ...base, result: { recommendation: { status: "keep" } } }), { state: "keep" });
  assert.deepEqual(describe({ ...base, result: { recommendation: { status: "not_enough_data" } } }), { state: "none" });
});

test("recommendation text names the change and why nothing else", () => {
  const text = recommendationText(change);
  assert.match(text.headline, /Lateral accel limit 3\.5 m\/s² → 4\.0 m\/s²/);
  assert.match(text.detail, /4 of 5/);
  assert.match(text.detail, /-9\.5%/);
  assert.match(recommendationText({ status: "not_enough_data", reason: "curves" }).detail, /Fewer than 3 curves/);
  assert.match(recommendationText({ status: "no_simulation", reason: "plant" }).detail, /Too little steady steering/);
  assert.match(recommendationText({ status: "no_simulation", reason: "exception:KeyError" }).detail, /exception:KeyError/);
  assert.match(recommendationText({ status: "keep", candidates: [{ delta: { windows: 4 } }] }).detail, /4 curves/);
});

test("apply only for a suggested change, parked, and not already applied", () => {
  const p = { ...base, result: { recommendation: change } };
  assert.deepEqual(applyState(p), { visible: true, enabled: true, applied: false, changed: false, now: 35, stale: false });
  assert.equal(applyState({ ...p, onroad: true }).enabled, false);
  assert.equal(applyState({ ...p, running: true }).enabled, false);
  assert.deepEqual(applyState({ ...p, current: { LateralAccelLimit: 40 } }), { visible: true, enabled: false, applied: true, changed: false, now: 40, stale: false });
  // a newer drive not analysed yet, or the latest analysis failed: the older advice is not applied
  assert.equal(applyState({ ...p, needs_run: true }).enabled, false);
  assert.equal(applyState({ ...p, status: { state: "error" } }).enabled, false);
  assert.equal(applyState({ ...p, current: { LateralAccelLimit: 45 } }).changed, true);
  assert.equal(applyState({ ...base, result: { recommendation: { status: "keep" } } }).visible, false);
});

test("drive figures list only what happened", () => {
  const lines = metricLines({ takeovers_per_min: 2, takeovers: { against: 6, with: 11 }, release_per_min: 0.57, overshoot_per_min: 0,
    ratio_low: 0.996, ratio_high: null, sat_pct: 1.61, clip_seconds: 0, eps_refusals: 0, takeovers_against_model_over_road: 0 });
  assert.equal(lines.length, 5);
  assert.match(lines[0], /6 against openpilot, 11 wanting more/);
  assert.match(lines[3], /low speed 1\.00, high speed -/);
});

test("footnotes follow the job: P at a standstill, driving, put off, waiting", () => {
  assert.deepEqual(statusNotes({ ...base, onroad: true, running: true, status: { step: "reading", drive_state: "parked" } }), ["parked_note"]);
  assert.deepEqual(statusNotes({ ...base, onroad: true, running: true, status: { step: "waiting", drive_state: "driving" } }), ["gentle_note"]);
  assert.deepEqual(statusNotes({ ...base, running: true, status: { step: "reading", drive_state: "offroad" } }), []);
  assert.deepEqual(statusNotes({ ...base, status: { state: "interrupted", step: "busy" }, result: null }), ["busy_note"]);
  assert.deepEqual(statusNotes({ ...base, status: { state: "interrupted", step: "stopped" }, result: null }), ["interrupted_note"]);
  assert.deepEqual(statusNotes({ ...base, result: null }), ["waiting_note"]);
  assert.deepEqual(statusNotes({ ...base, onroad: true, result: { recommendation: change } }), ["onroad_note"]);
  assert.deepEqual(statusNotes({ ...base, result: { recommendation: change } }), []);
  assert.deepEqual(statusNotes({ ...base, needs_run: true, result: { route: "00000252--x", recommendation: change } }), ["stale_note"]);
  assert.deepEqual(statusNotes({ ...base, status: { state: "error" }, result: { route: "00000252--x", recommendation: change } }), ["error_note"]);
});

test("custom torque switch and mismatch reason read naturally", () => {
  const ko = (k) => ({ name_LateralTorqueCustom: "사용자 토크 튜닝", off: "끔", on: "켬" })[k] ?? k;
  assert.equal(formatChange("LateralTorqueCustom", 0, 1, ko), "사용자 토크 튜닝 끔 → 켬");
  assert.match(recommendationText({ status: "no_simulation", reason: "mismatch" }).detail, /lane mode/);
});
