"use strict";

// Tools card: tuning advice from the last drive. Server: /api/tuning_advice (features/tuning_advice.py), analysis in
// openpilot/selfdrive/carrot/tuning_advice/.

const POLL_RUNNING_MS = 3000;
const POLL_IDLE_MS = 15000;
let pollTimer = null;
let lastPayload = null;
let busy = false;

const FALLBACK_STRINGS = {
  title: "Tuning from the last drive",
  state_running: "Analysing {pct}%",
  state_change: "Change suggested",
  state_keep: "Keep",
  state_wait: "Waiting",
  state_none: "Not enough data",
  state_error: "Analysis failed",
  drive_line: "Drive {when} · steering {min} min · {segments} segments",
  match_title: "Model intent vs actual",
  match_value: "{pct}% match",
  match_detail: "Curves {curve} · straights {straight} (debug plot 6: green = requested, yellow = actual lateral accel)",
  match_none: "Not enough steering to compare.",
  rec_change: "{change} recommended",
  rec_change_detail: "Same curves replayed: better in {better} of {windows}, curve error {err}, toward the outside {out}; nothing else worse.",
  rec_keep: "Keep the current values",
  rec_keep_detail: "Replaying {windows} curves with each one-step change moved nothing beyond the replay's own noise.",
  rec_one_at_a_time: "Change one value at a time and check it on the next drive.",
  rec_not_enough_data_lat_minutes: "Less than 3 minutes of steering above 8 km/h in the last drive.",
  rec_not_enough_data_curves: "Fewer than 3 curves openpilot steered through without the driver: nothing to compare.",
  rec_no_drive: "No recent drive with at least 3 minutes of steering.",
  rec_no_simulation_unsupported_car: "The replay supports GM torque steering only; the drive's own figures are below.",
  rec_no_simulation_plant: "Too little steady steering to model how this car responds (mostly low-speed streets).",
  rec_no_simulation_no_windows: "No curve or straight long enough to replay.",
  rec_no_simulation_no_car_params: "The drive's log has no car parameters.",
  rec_no_simulation_other: "The replay could not run ({reason}).",
  p_reference: "P is judged from real drives only; its replay is shown for reference.",
  changed_since: "Changed since that drive: now {now}.",
  applied_already: "Already applied.",
  apply: "Apply",
  rerun: "Analyse again",
  applying: "Applying",
  confirm_apply: "Change {change}? It takes effect right away; check it on the next drive.",
  applied: "Saved {change}.",
  onroad_note: "Changes can be applied only while parked with the ignition off.",
  waiting_note: "Runs a minute after parking, or two minutes into the next drive if the comma powers off with the car.",
  error_note: "Analysis failed: {error}",
  interrupted_note: "Stopped because the ignition was turned on. The drive is analysed slowly two minutes into this drive.",
  busy_note: "Put off for this drive: not enough free memory or the comma was too hot. It tries again on the next drive or with Analyse again.",
  resting_note: "Paused until free memory and temperature allow it.",
  gentle_note: "Analysing the previous drive slowly so this drive is not affected.",
  metrics_title: "Last drive",
  m_takeovers: "Driver took over {n}/min ({against} against openpilot, {with} wanting more)",
  m_releases: "Torque let go at the limit {n}/min",
  m_overshoot: "Overshoot then counter-steer {n}/min",
  m_ratio: "Curve tracking (actual / requested): low speed {low}, high speed {high}",
  m_saturation: "At the steering torque limit {n}% of the time",
  m_clip: "Demand cut by the lateral accel limit {n} s",
  m_eps: "EPS refused steering {n} times",
  m_model_over_road: "{n} takeovers at low speed where the model asked for more turn than the lane (a model issue, not tuning)",
  review_title: "Last change: {change}",
  review_better: "Looks better than the drive before",
  review_worse: "Looks worse than the drive before; consider going back",
  review_unclear: "No clear difference yet",
  review_not_comparable: "More than one value changed since the last analysed drive ({names}); not compared.",
  review_row: "{name}: {before} → {after}",
  sim_title: "Replay of the same curves",
  sim_head: "Change · error · outside · better/worse",
  rej_no_effect: "no difference",
  seconds: "{v} s",
  rej_inconsistent: "not consistent",
  rej_small: "too small",
  rej_jitter: "more jitter",
  rej_straight: "worse on straights",
  rej_releases: "more let-go",
  rej_saturation: "more at the limit",
  ok: "worth a drive",
  name_LateralAccelLimit: "Lateral accel limit",
  name_LateralTorqueKiV: "Steering I",
  name_LateralTorqueKpV: "Steering P",
  name_LatTorqueLimitPFilter: "Torque-limit P filter",
  off: "off",
  r_match_pct: "Match",
  r_match_curve_pct: "Curve match",
  r_takeovers_per_min: "Takeovers/min",
  r_release_per_min: "Let-go/min",
  r_err: "Tracking error",
};

function t(key, values = {}) {
  const fallback = FALLBACK_STRINGS[key] ?? key;
  const text = typeof getUIText === "function" ? getUIText(`tuning_advice_${key}`, fallback) : fallback;
  return String(text).replace(/\{(\w+)\}/g, (_m, name) => (name in values ? String(values[name]) : ""));
}

// setting value -> what the user sees (units of the Carrot web setting)
function formatValue(param, value, translate = t) {
  const v = Number(value);
  if (!Number.isFinite(v)) return String(value ?? "-");
  if (param === "LateralAccelLimit") return `${(v / 10).toFixed(1)} m/s²`;
  if (param === "LateralTorqueKpV" || param === "LateralTorqueKiV") return (v / 100).toFixed(2);
  if (param === "LatTorqueLimitPFilter") return v <= 0 ? translate("off") : translate("seconds", { v: (v / 100).toFixed(2) });
  return String(v);
}

function formatChange(param, from, to, translate = t) {
  return `${translate(`name_${param}`)} ${formatValue(param, from, translate)} → ${formatValue(param, to, translate)}`;
}

function pct(value) {
  return value === null || value === undefined || !Number.isFinite(Number(value)) ? "-" : `${Number(value).toFixed(1)}%`;
}

function signed(value, unit) {
  if (value === null || value === undefined || !Number.isFinite(Number(value))) return "-";
  const v = Number(value);
  return `${v > 0 ? "+" : ""}${v.toFixed(1)}${unit}`;
}

// { state, pct } for the heading chip
function describe(payload) {
  const status = payload?.status || {};
  if (payload?.running) return { state: "running", pct: Math.max(0, Math.min(99, Number(status.progress) || 0)) };
  if (status.state === "error" && !payload?.result) return { state: "error" };
  const rec = payload?.result?.recommendation;
  if (!rec) return { state: "wait" };
  if (rec.status === "change") return { state: "change" };
  if (rec.status === "keep") return { state: "keep" };
  return { state: "none" };
}

// the recommendation's headline and detail keys/values
function recommendationText(rec, translate = t) {
  if (!rec) return null;
  if (rec.status === "change") {
    const d = rec.delta || {};
    return {
      headline: translate("rec_change", { change: formatChange(rec.param, rec.from, rec.to, translate) }),
      detail: translate("rec_change_detail", {
        better: d.better ?? 0, windows: d.windows ?? 0, err: signed(d.err_pct, "%"), out: signed(d.outward_cm, " cm"),
      }),
    };
  }
  if (rec.status === "keep") {
    const windows = Math.max(0, ...(rec.candidates || []).map((c) => Number(c.delta?.windows) || 0));
    return { headline: translate("rec_keep"), detail: translate("rec_keep_detail", { windows }) };
  }
  if (rec.status === "not_enough_data") {
    return { headline: translate("state_none"), detail: translate(`rec_not_enough_data_${rec.reason === "curves" ? "curves" : "lat_minutes"}`) };
  }
  if (rec.status === "no_drive") return { headline: translate("state_none"), detail: translate("rec_no_drive") };
  const known = ["unsupported_car", "plant", "no_windows", "no_car_params"];
  const reason = String(rec.reason || "");
  if (reason === "lat_minutes") return { headline: translate("state_none"), detail: translate("rec_not_enough_data_lat_minutes") };
  return {
    headline: translate("state_none"),
    detail: known.includes(reason) ? translate(`rec_no_simulation_${reason}`) : translate("rec_no_simulation_other", { reason }),
  };
}

// apply button: allowed only for a suggested change, parked, and when the value is not already the suggested one
function applyState(payload) {
  const rec = payload?.result?.recommendation;
  if (!rec || rec.status !== "change") return { visible: false };
  const now = payload?.current?.[rec.param];
  const applied = now !== undefined && now !== null && Number(now) === Number(rec.to);
  const changed = now !== undefined && now !== null && !applied && Number(now) !== Number(rec.from);
  return { visible: true, enabled: !payload.onroad && !payload.running && !applied, applied, changed, now };
}

function metricLines(m, translate = t) {
  if (!m) return [];
  const lines = [];
  const tk = m.takeovers || {};
  lines.push(translate("m_takeovers", { n: m.takeovers_per_min ?? 0, against: tk.against ?? 0, with: tk.with ?? 0 }));
  lines.push(translate("m_releases", { n: m.release_per_min ?? 0 }));
  lines.push(translate("m_overshoot", { n: m.overshoot_per_min ?? 0 }));
  const ratio = (x) => (x === null || x === undefined ? "-" : Number(x).toFixed(2));
  lines.push(translate("m_ratio", { low: ratio(m.ratio_low), high: ratio(m.ratio_high) }));
  lines.push(translate("m_saturation", { n: m.sat_pct ?? 0 }));
  if (Number(m.clip_seconds) > 0) lines.push(translate("m_clip", { n: m.clip_seconds }));
  if (Number(m.eps_refusals) > 0) lines.push(translate("m_eps", { n: m.eps_refusals }));
  if (Number(m.takeovers_against_model_over_road) > 0) lines.push(translate("m_model_over_road", { n: m.takeovers_against_model_over_road }));
  return lines;
}

// footnote keys under the card for the job's state
function statusNotes(payload, state = describe(payload).state) {
  const status = payload?.status || {};
  const notes = [];
  if (payload?.running && payload?.gentle) notes.push(status.step === "waiting" ? "resting_note" : "gentle_note");
  else if (state === "error") notes.push("error_note");
  else if (status.state === "interrupted") notes.push(status.step === "busy" ? "busy_note" : "interrupted_note");
  else if (!payload?.result && !payload?.running) notes.push("waiting_note");
  if (payload?.onroad && applyState(payload).visible) notes.push("onroad_note");
  return notes;
}

function el(tag, className, text) {
  const node = document.createElement(tag);
  if (className) node.className = className;
  if (text !== undefined) node.textContent = text;
  return node;
}

function driveWhen(result) {
  const start = Number(result?.drive_start);
  if (!Number.isFinite(start) || start <= 0) return String(result?.route || "").split("--")[0];
  const d = new Date(start * 1000);
  const two = (n) => String(n).padStart(2, "0");
  return `${d.getMonth() + 1}/${d.getDate()} ${two(d.getHours())}:${two(d.getMinutes())}`;
}

function render(payload = lastPayload) {
  const card = document.getElementById("tuningAdviceCard");
  const body = document.getElementById("tuningAdviceBody");
  if (!card || !body || !payload?.ok) return;
  lastPayload = payload;
  card.hidden = false;
  const { state, pct: progress } = describe(payload);
  card.dataset.state = state;
  card.classList.toggle("is-running", state === "running");
  const result = payload.result;
  const m = result?.metrics;
  const rec = result?.recommendation;

  const heading = el("div", "egpu-model-card__heading");
  heading.append(el("strong", "", t("title")), el("span", "egpu-model-card__state", t(`state_${state}`, { pct: progress })));
  const nodes = [heading];

  if (result?.route) {
    nodes.push(el("p", "egpu-model-card__detail", t("drive_line", { when: driveWhen(result), min: Number(m?.lat_minutes ?? 0).toFixed(1), segments: result.segments ?? 0 })));
  }

  // plot 6 agreement, always first
  if (m) {
    const box = el("div", "tuning-advice__match");
    box.append(el("span", "tuning-advice__match-label", t("match_title")));
    if (m.match_pct === null || m.match_pct === undefined) {
      box.append(el("span", "tuning-advice__match-detail", t("match_none")));
    } else {
      box.append(el("strong", "tuning-advice__match-value", t("match_value", { pct: Number(m.match_pct).toFixed(1) })));
      box.append(el("span", "tuning-advice__match-detail", t("match_detail", { curve: pct(m.match_curve_pct), straight: pct(m.match_straight_pct) })));
    }
    nodes.push(box);
  }

  const text = recommendationText(rec);
  if (text) {
    const box = el("div", `tuning-advice__rec tuning-advice__rec--${rec.status}`);
    box.append(el("strong", "tuning-advice__rec-headline", text.headline), el("p", "egpu-model-card__detail", text.detail));
    if (rec.status === "change") box.append(el("p", "egpu-model-card__detail", t("rec_one_at_a_time")));
    const apply = applyState(payload);
    if (apply.changed) box.append(el("p", "egpu-model-card__detail", t("changed_since", { now: formatValue(rec.param, apply.now) })));
    if (apply.applied) box.append(el("p", "egpu-model-card__detail", t("applied_already")));
    nodes.push(box);
  }

  const review = result?.change_review;
  if (review) {
    const box = el("div", "tuning-advice__review");
    if (review.comparable) {
      box.append(el("strong", "", t("review_title", { change: formatChange(review.param, review.before, review.after) })));
      box.append(el("p", "egpu-model-card__detail", t(`review_${review.verdict || "unclear"}`)));
      const list = el("ul", "tuning-advice__list");
      for (const key of ["match_pct", "match_curve_pct", "takeovers_per_min", "release_per_min", "err"]) {
        const row = review.rows?.[key];
        if (!row || (row.before === null && row.after === null)) continue;
        const unit = key.startsWith("match_") ? "%" : "";
        const show = (x) => (x === null || x === undefined ? "-" : `${x}${unit}`);
        list.append(el("li", "", t("review_row", { name: t(`r_${key}`), before: show(row.before), after: show(row.after) })));
      }
      box.append(list);
    } else {
      box.append(el("p", "egpu-model-card__detail", t("review_not_comparable", { names: (review.changed || []).join(", ") })));
    }
    nodes.push(box);
  }

  const lines = metricLines(m);
  if (lines.length) {
    const details = el("details", "tuning-advice__details");
    details.append(el("summary", "", t("metrics_title")));
    const list = el("ul", "tuning-advice__list");
    lines.forEach((line) => list.append(el("li", "", line)));
    details.append(list);
    nodes.push(details);
  }

  const rows = rec?.candidates || [];
  if (rows.length) {
    const details = el("details", "tuning-advice__details");
    details.append(el("summary", "", t("sim_title")));
    const list = el("ul", "tuning-advice__list tuning-advice__sim");
    list.append(el("li", "tuning-advice__sim-head", t("sim_head")));
    for (const row of rows) {
      const d = row.delta || {};
      const verdict = row.reference_only ? t("p_reference")
        : (row.rejected?.length ? row.rejected.map((r) => t(`rej_${r}`)).join(", ") : t("ok"));
      list.append(el("li", "", `${formatChange(row.param, row.from, row.to)} · ${signed(d.err_pct, "%")} · ${signed(d.outward_cm, " cm")} · ${d.better ?? 0}/${d.worse ?? 0} · ${verdict}`));
    }
    details.append(list);
    nodes.push(details);
  }

  const footer = el("div", "egpu-model-card__footer tuning-advice__footer");
  const apply = applyState(payload);
  if (apply.visible) {
    const button = el("button", "btn egpu-model-card__action", busy ? t("applying") : t("apply"));
    button.type = "button";
    button.disabled = busy || !apply.enabled;
    button.addEventListener("click", applyRecommendation);
    footer.append(button);
  }
  const rerun = el("button", "btn egpu-model-card__action", t("rerun"));
  rerun.type = "button";
  rerun.disabled = busy || !payload.can_run;
  rerun.addEventListener("click", rerunAnalysis);
  footer.append(rerun);
  nodes.push(footer);

  const notes = statusNotes(payload, state).map((key) => (key === "error_note" ? t(key, { error: payload.status?.error || "" }) : t(key)));
  notes.forEach((note) => nodes.push(el("span", "egpu-model-card__amount", note)));

  body.replaceChildren(...nodes);
}

async function refresh() {
  try {
    render(await getJson("/api/tuning_advice"));
  } catch (_error) {
    // keep the last card; the next poll retries
  } finally {
    window.clearTimeout(pollTimer);
    pollTimer = window.setTimeout(refresh, lastPayload?.running ? POLL_RUNNING_MS : POLL_IDLE_MS);
  }
}

async function applyRecommendation() {
  const rec = lastPayload?.result?.recommendation;
  if (!rec || rec.status !== "change" || !applyState(lastPayload).enabled) return;
  const change = formatChange(rec.param, rec.from, rec.to);
  if (typeof appConfirm === "function" && !await appConfirm(t("confirm_apply", { change }), { title: t("title") })) return;
  busy = true;
  render();
  try {
    await postJson("/api/param_set", { name: rec.param, value: rec.to, source: "tuning_advice" });
    if (typeof showAppToast === "function") showAppToast(t("applied", { change }));
  } catch (error) {
    if (typeof showAppToast === "function") showAppToast(`${t("state_error")}: ${error?.message || error}`, { tone: "error", duration: 5000 });
  } finally {
    busy = false;
    await refresh();
  }
}

async function rerunAnalysis() {
  busy = true;
  render();
  try {
    render(await postJson("/api/tuning_advice/run", {}));
  } catch (error) {
    if (typeof showAppToast === "function") showAppToast(`${t("state_error")}: ${error?.message || error}`, { tone: "error", duration: 5000 });
  } finally {
    busy = false;
    await refresh();
  }
}

function init() {
  if (!pollTimer) refresh();
}

const CarrotTuningAdvice = Object.freeze({ init, refresh, render });
globalThis.CarrotTuningAdvice = CarrotTuningAdvice;

export { CarrotTuningAdvice, FALLBACK_STRINGS, applyState, describe, formatChange, formatValue, metricLines, recommendationText, statusNotes };
