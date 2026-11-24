import assert from "node:assert/strict";
import test from "node:test";

import { applyState, changedNote, describe, learnedRows, valueText, formatChange, formatValue, metricLines, rawChange, recommendationText, settingRows, statusNotes } from "../src/features/tools/tuning_advice.js";

const change = { status: "change", param: "LateralAccelLimit", from: 35, to: 40, delta: { better: 4, windows: 5, err_pct: -9.5, outward_cm: -3.2 } };
const base = { ok: true, onroad: false, running: false, can_run: true, status: { state: "done" }, current: { LateralAccelLimit: 35 } };

test("values show in the units of the Carrot web setting", () => {
  assert.equal(formatValue("LateralAccelLimit", 35), "3.5 m/s²");
  assert.equal(formatValue("LateralTorqueKiV", 70), "0.70");
  assert.equal(formatValue("LatTorqueLimitPFilter", 0, (k) => ({ off: "끔" })[k]), "끔");
  assert.equal(formatValue("LatTorqueLimitPFilter", 30, (k, v) => `${v.v}초`), "0.30초");
  assert.equal(formatValue("LatTorqueLimitPFilter", 30), "0.30 s");
  const ko = (k) => ({ name_LateralTorqueKpV: "조향 P" })[k] ?? k;
  assert.equal(formatChange("LateralTorqueKpV", 160, 170, ko), "조향 P 160 → 170 (1.60 → 1.70)");
  assert.equal(rawChange("LateralAccelLimit", 35, 40), "35 → 40 (3.5 → 4.0 m/s²)");
  assert.equal(rawChange("LatTorqueLimitPFilter", 0, 30, (k, v) => (k === "off" ? "끔" : `${v.v}초`)), "0 → 30 (끔 → 0.30초)");
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
  assert.equal(text.headline, "Change one value: Lateral accel limit 35 → 40 (3.5 → 4.0 m/s²)");
  assert.match(text.detail, /same 5 curves, 4 got better and none got worse/);
  assert.match(recommendationText({ status: "not_enough_data", reason: "curves" }).detail, /Fewer than 3 curves/);
  assert.match(recommendationText({ status: "no_simulation", reason: "plant" }).detail, /Too little steady steering/);
  assert.match(recommendationText({ status: "no_simulation", reason: "exception:KeyError" }).detail, /exception:KeyError/);
  const keep = recommendationText({ status: "keep", candidates: [{ delta: { windows: 4 } }] });
  assert.equal(keep.headline, "Keep your current settings");
  assert.match(keep.detail, /same 4 curves/);
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
  assert.equal(formatChange("LateralTorqueCustom", 0, 1, ko), "사용자 토크 튜닝 0 → 1 (끔 → 켬)");
  assert.match(recommendationText({ status: "no_simulation", reason: "mismatch" }).detail, /lane mode/);
});

test("one row per setting: keep or change, and why", () => {
  const cand = (param, to, rejected, extra = {}) => ({ param, from: { LateralAccelLimit: 35, LateralTorqueKiV: 70, LatTorqueLimitPFilter: 0, LateralTorqueKpV: 160 }[param],
    to, rejected, reference_only: param === "LateralTorqueKpV", delta: { windows: 5, better: 0, worse: 0 }, ...extra });
  const result = {
    settings: { LateralAccelLimit: 35, LateralTorqueKiV: 70, LatTorqueLimitPFilter: 0, LateralTorqueKpV: 160 },
    recommendation: { status: "keep", candidates: [
      cand("LateralTorqueKiV", 90, ["no_effect", "small"]), cand("LateralTorqueKiV", 50, ["inconsistent", "small"], { delta: { windows: 5, better: 1, worse: 2 } }),
      cand("LateralAccelLimit", 40, ["no_effect", "small"]), cand("LateralAccelLimit", 30, ["no_effect", "small"]),
      cand("LatTorqueLimitPFilter", 30, ["inconsistent", "small", "saturation"]), cand("LateralTorqueKpV", 170, []),
    ] },
  };
  const rows = settingRows(result);
  assert.deepEqual(rows.map((r) => [r.param, r.now, r.to, r.change, r.reason]), [
    ["LateralAccelLimit", 35, 35, false, "no_effect"],
    ["LateralTorqueKpV", 160, 160, false, "p"],
    ["LateralTorqueKiV", 70, 70, false, "mixed"],
    ["LatTorqueLimitPFilter", 0, 0, false, "worse"],
  ]);
  const changed = settingRows({ ...result, recommendation: { ...result.recommendation, status: "change", param: "LateralAccelLimit", from: 35, to: 40,
    delta: { windows: 5, better: 4 } } });
  assert.deepEqual([changed[0].to, changed[0].change, changed[0].reason], [40, true, "change"]);
  assert.equal(changed.find((r) => r.param === "LateralTorqueKiV").change, false);
});

test("a change that made some curves worse says so, and a passed but unchosen value is not called small", () => {
  const text = recommendationText({ ...change, delta: { better: 8, worse: 2, windows: 10 } });
  assert.match(text.detail, /8 got better and 2 got worse/);
  const result = {
    settings: { LateralAccelLimit: 35, LateralTorqueKiV: 70 },
    recommendation: { status: "change", param: "LateralAccelLimit", from: 35, to: 40, delta: { windows: 10, better: 8, worse: 2 }, candidates: [
      { param: "LateralAccelLimit", from: 35, to: 40, rejected: [], reference_only: false, delta: { windows: 10, better: 8, worse: 2 } },
      { param: "LateralTorqueKiV", from: 70, to: 90, rejected: [], reference_only: false, delta: { windows: 10, better: 10, worse: 0 } },
    ] },
  };
  const rows = settingRows(result);
  assert.deepEqual(rows.map((r) => r.reason), ["change_some", "also_ok"]);
});

test("a setting changed during the drive is named with when, and which part was analysed", () => {
  const at = new Date(2026, 10, 24, 12, 22).getTime() / 1000;
  const result = { settings_changed: { used: "after", changes: [
    { name: "LateralTorqueKiV", prev: 70, next: 60, at }, { name: "LateralTorqueKiV", prev: 60, next: 50, at: at + 5 },
  ] } };
  assert.equal(changedNote(result), "Settings were changed during this drive (Steering I 70 → 50 (12:22)), so only the part after the change was analysed.");
  assert.equal(changedNote({}), null);
});

test("every tuning value reads in its own unit, the torque rates with the car default", () => {
  const ko = (k, v = {}) => ({ car_default: `차 기본값 ${v.v}`, sr_learned: "학습값 사용", seconds: `${v.v}초` })[k] ?? k;
  const defaults = { CustomSteerDeltaUp: 10, CustomSteerDeltaDown: 15 };
  assert.equal(valueText("LateralTorqueKf", 115, ko), "115 (1.15)");
  assert.equal(valueText("LateralTorqueKd", 0, ko), "0 (0.00)");
  assert.equal(valueText("LatJerkMax", 100, ko), "100 (10.0 m/s³)");
  assert.equal(valueText("CustomSteerDeltaUp", 0, ko, defaults), "0 (차 기본값 10)");
  assert.equal(valueText("CustomSteerDeltaDown", 18, ko, defaults), "18");
  assert.equal(rawChange("CustomSteerDeltaUp", 0, 12, ko, defaults), "0 → 12 (차 기본값 10 → 12)");
  assert.equal(valueText("CustomSR", 161, ko), "161 (16.1)");
  assert.equal(valueText("CustomSR", 0, ko), "0 (학습값 사용)");
  assert.equal(valueText("SteerRatioRate", 87, ko), "87 (87%)");
  assert.equal(valueText("SteerActuatorDelay", 30, ko), "30 (0.30초)");
});

test("learned steer ratio and delay rows: check when the ratio is off, info for the delay", () => {
  const result = {
    settings: { SteerActuatorDelay: 30 },
    learned: {
      steer_ratio: { learned: 18.0, used: 16.1, diff_pct: -10.6, param: "CustomSR", now: 161, match: 180, check: true },
      lat_delay: { learned: 0.42, setting: 0.3, uses_learned: true },
    },
  };
  const rows = learnedRows(result);
  assert.deepEqual(rows.map((r) => [r.param, r.now, r.badge, r.reason]), [
    ["CustomSR", 161, "check", "sr_check"],
    ["SteerActuatorDelay", 30, "info", "delay_learned"],
  ]);
  assert.deepEqual(rows[0].values, { learned: "18.0", pct: "10.6", match: "180 (18.0)" });
  const ok = learnedRows({ learned: { steer_ratio: { learned: 16.2, diff_pct: -0.6, param: "CustomSR", now: 161, match: 162, check: false } } });
  assert.deepEqual([ok[0].badge, ok[0].reason], ["keep", "sr_ok"]);
  assert.deepEqual(learnedRows({}), []);
});

test("Kd gets its own reference reason", () => {
  const rows = settingRows({ settings: { LateralTorqueKd: 0 }, recommendation: { status: "keep", candidates: [
    { param: "LateralTorqueKd", from: 0, to: 10, rejected: [], reference_only: true, delta: { windows: 3 } }] } });
  assert.deepEqual([rows[0].param, rows[0].badge, rows[0].reason], ["LateralTorqueKd", "keep", "kd"]);
});

test("a torque limit set back to 0 mid-drive says it was kept", () => {
  const at = new Date(2026, 10, 24, 12, 30).getTime() / 1000;
  const note = changedNote({ settings_changed: { used: "after", changes: [
    { name: "CustomSteerDeltaUp", prev: 0, next: 12, at }, { name: "CustomSteerDeltaUp", prev: 12, next: 0, at: at + 60, effective: 12 },
  ] } });
  assert.equal(note, "Settings were changed during this drive (Torque rise rate 0 → 0 (12:31, kept at 12 for the rest of the drive)), so only the part after the change was analysed.");
});

test("a torque limit flipped back within a second says it is unknown whether it was used", () => {
  const at = new Date(2026, 10, 24, 12, 30).getTime() / 1000;
  const note = changedNote({ settings_changed: { used: "before", unsure: true, changes: [
    { name: "CustomSteerDeltaUp", prev: 0, next: 12, at }, { name: "CustomSteerDeltaUp", prev: 12, next: 0, at: at + 0.1, effective: 12 },
  ] } });
  assert.match(note, /^A GM torque limit was changed and set back within a second/);
});
