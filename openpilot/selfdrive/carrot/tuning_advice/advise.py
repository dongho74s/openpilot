"""Last-drive tuning advice, step 4: one recommended change (or "keep") from the drive's own metrics and the replay.

Rules (Traverse tuning of 2026-11-23/24):
- One change at a time, then a real drive to check it before the next.
- The replay compares settings on the same curves; it decides the lateral accel limit, the integrator and the
  torque-limit P filter. P is never recommended from the replay alone: its replay result is shown for reference, and
  a P change is judged by the next drive (change review below).
- Not enough steering in curves -> no recommendation.
"""
from __future__ import annotations

import numpy as np

# (param, step, low, high); LatTorqueLimitPFilter toggles between off and the recommended 30 (0.3 s)
CANDIDATES = (
  ("LateralAccelLimit", +5, 30, 90),
  ("LateralAccelLimit", -5, 30, 90),
  ("LateralTorqueKiV", +20, 0, 200),
  ("LateralTorqueKiV", -20, 0, 200),
  ("LatTorqueLimitPFilter", 0, 0, 60),
  ("LateralTorqueKpV", +10, 0, 200),
  ("LateralTorqueKpV", -10, 0, 200),
)
REFERENCE_ONLY = frozenset({"LateralTorqueKpV"})
CUSTOM_ONLY = frozenset({"LateralTorqueKpV", "LateralTorqueKiV"})   # used only with LateralTorqueCustom on
P_FILTER_ON = 30

MIN_LAT_MINUTES = 3.0
MIN_CURVE_WINDOWS = 3
# a change must move the car at least this much in the curves to be worth a drive
MIN_OUTWARD_GAIN_CM = 3.0
MIN_ERR_GAIN_PCT = 8.0
# and must not make these worse
MAX_JIT_RISE_PCT = 10.0
MAX_STRAIGHT_ERR_RISE_PCT = 5.0
MAX_SAT_RISE_PP = 1.0


def as_int(values: dict, key: str, default: int) -> int:
  try:
    return int(float(values.get(key, default)))
  except (TypeError, ValueError):
    return default


def drive_settings(tuning: dict[str, str]) -> dict[str, int]:
  return {
    "LateralTorqueCustom": as_int(tuning, "LateralTorqueCustom", 0),
    "LateralTorqueKpV": as_int(tuning, "LateralTorqueKpV", 100),
    "LateralTorqueKiV": as_int(tuning, "LateralTorqueKiV", 10),
    "LateralAccelLimit": as_int(tuning, "LateralAccelLimit", 30),
    "LatJerkMax": as_int(tuning, "LatJerkMax", 50),
    "LatTorqueLimitPFilter": as_int(tuning, "LatTorqueLimitPFilter", 0),
  }


def lal_value(setting: int) -> float:
  return setting / 10.0 if 30 <= setting <= 90 else 3.0


def jerk_value(setting: int) -> float:
  return setting / 10.0 if 20 <= setting <= 100 else 5.0


def candidate_list(settings: dict[str, int], p_filter_available: bool) -> list[tuple[str, int, int]]:
  """(param, from, to) for each one-step change that stays inside the setting's range."""
  out = []
  for key, step, lo, hi in CANDIDATES:
    if key in CUSTOM_ONLY and settings["LateralTorqueCustom"] <= 0:
      continue
    if key == "LatTorqueLimitPFilter":
      if not p_filter_available:
        continue
      cur = settings[key]
      out.append((key, cur, 0 if cur > 0 else P_FILTER_ON))
      continue
    cur = settings[key]
    new = cur + step
    if lo <= new <= hi and lo <= cur <= hi:
      out.append((key, cur, new))
  return out


NOISE_FRICTION_SCALE = 1.1   # the noise run: the plant's play band 10 % wider, same setting


def _classify(change, tol):
  if change is None:
    return 0
  return -1 if change < -tol else (1 if change > tol else 0)


def aggregate(per_window: list[dict]) -> dict:
  """per_window: {"kind", "ref", "noise", "cand": {label: metrics}} -> {label: summary vs the drive's setting}.

  A curve replay can flip on a tiny difference (torque at the limit lets go or not), so each window also runs the
  drive's own setting with a 10 % wider play band ("noise"). A change counts in a window only where it moves the result
  by more than twice that, and a recommendation needs most windows to agree (judge())."""
  bends = [w for w in per_window if w["kind"] in ("curve", "turn")]
  straight = [w for w in per_window if w["kind"] == "straight"]
  labels = sorted({lab for w in per_window for lab in w["cand"]})

  def rel(a, b):
    return None if a is None or b is None or b <= 1e-9 else (a - b) / b

  def med(xs, scale=1.0, nd=1):
    xs = [x for x in xs if x is not None and np.isfinite(x)]
    return round(float(np.median(xs)) * scale, nd) if xs else None

  out = {}
  for lab in labels:
    better = worse = rel_up = rel_down = 0
    err_ch, out_ch, jit_ch, sat_ch, str_ch = [], [], [], [], []
    for w in bends:
      r, n, c = w["ref"], w.get("noise") or w["ref"], w["cand"].get(lab)
      if c is None:
        continue
      e = rel(c["err"], r["err"])
      e_tol = max(2 * abs(rel(n["err"], r["err"]) or 0.0), 0.02)
      o = None if c.get("laneOut") is None or r.get("laneOut") is None else c["laneOut"] - r["laneOut"]
      o_noise = 0.0 if n.get("laneOut") is None or r.get("laneOut") is None else abs(n["laneOut"] - r["laneOut"])
      o_tol = max(2 * o_noise, 0.01)
      ec, oc = _classify(e, e_tol), _classify(o, o_tol)
      if (ec < 0 and oc <= 0) or (oc < 0 and ec <= 0):
        better += 1
      elif (ec > 0 and oc >= 0) or (oc > 0 and ec >= 0):
        worse += 1
      d_rel = c["rel"] - r["rel"]
      if abs(d_rel) > abs(n["rel"] - r["rel"]):
        rel_up += d_rel > 0
        rel_down += d_rel < 0
      err_ch.append(e)
      out_ch.append(o)
    for w in per_window:
      c = w["cand"].get(lab)
      if c is None:
        continue
      jit_ch.append(rel(c["jit"], w["ref"]["jit"]))
      sat_ch.append(c["sat"] - w["ref"]["sat"])
      if w["kind"] == "straight":
        str_ch.append(rel(c["err"], w["ref"]["err"]))
    out[lab] = {
      "windows": len(err_ch), "better": better, "worse": worse, "releases_up": int(rel_up), "releases_down": int(rel_down),
      "err_pct": med(err_ch, 100), "outward_cm": med(out_ch, 100), "jit_pct": med(jit_ch, 100),
      "sat_pp": med(sat_ch, 100, 2), "straight_err_pct": med(str_ch, 100),
    }
  out["_ref"] = {
    "err": med([w["ref"]["err"] for w in bends], 1, 3), "outward_cm": med([w["ref"].get("laneOut") for w in bends], 100),
    "releases": int(sum(w["ref"]["rel"] for w in per_window)), "err_straight": med([w["ref"]["err"] for w in straight], 1, 3),
  }
  out["_windows"] = {"bends": len(bends), "straight": len(straight)}
  return out


def judge(delta: dict) -> tuple[float, list[str]]:
  """(benefit score, reasons it is rejected); a change is worth a drive only if it helps in most curve windows, by a
  worthwhile amount, and makes nothing else worse."""
  why = []
  n = delta.get("windows", 0)
  if n > 0 and delta.get("better", 0) == 0 and delta.get("worse", 0) == 0:
    why.append("no_effect")        # inside the replay noise in every curve
  elif n == 0 or delta.get("better", 0) < int(np.ceil(0.6 * n)) or delta.get("worse", 0) > int(np.floor(0.2 * n)):
    why.append("inconsistent")
  out_gain = -(delta.get("outward_cm") or 0.0)
  err_gain = -(delta.get("err_pct") or 0.0)
  if out_gain < MIN_OUTWARD_GAIN_CM and err_gain < MIN_ERR_GAIN_PCT:
    why.append("small")
  if (delta.get("jit_pct") or 0) > MAX_JIT_RISE_PCT:
    why.append("jitter")
  if (delta.get("straight_err_pct") or 0) > MAX_STRAIGHT_ERR_RISE_PCT:
    why.append("straight")
  if delta.get("releases_up", 0) > delta.get("releases_down", 0):
    why.append("releases")
  if (delta.get("sat_pp") or 0) > MAX_SAT_RISE_PP:
    why.append("saturation")
  return out_gain + err_gain / 4 + delta.get("releases_down", 0) - delta.get("releases_up", 0), why


def change_review(current: dict, history: list[dict]) -> dict | None:
  """The last analysed drive that differs from this one in exactly one tuning value: its metrics beside this drive's."""
  cur = current.get("settings") or {}
  for prev in reversed(history):
    if prev.get("route") == current.get("route"):
      continue
    ps = prev.get("settings") or {}
    diff = [k for k in cur if k != "LateralTorqueCustom" and ps.get(k) != cur.get(k)]
    if len(diff) == 0:
      continue
    if len(diff) > 1:
      return {"route": prev.get("route"), "changed": diff, "comparable": False}
    key = diff[0]
    a, b = prev.get("metrics") or {}, current.get("metrics") or {}
    rows = {}
    for m in ("match_pct", "match_curve_pct", "err", "ratio_low", "ratio_high", "curve_outward_cm", "takeovers_per_min",
              "release_per_min", "overshoot_per_min", "wobble_pct", "sat_pct"):
      rows[m] = {"before": a.get(m), "after": b.get(m)}
    better = worse = 0
    lower_is_better = ("err", "takeovers_per_min", "release_per_min", "overshoot_per_min", "wobble_pct")
    for m in lower_is_better:
      x, y = a.get(m), b.get(m)
      if x is None or y is None or max(x, y) <= 0:
        continue
      rel = (y - x) / max(x, 1e-6)
      if rel <= -0.15:
        better += 1
      elif rel >= 0.15:
        worse += 1
    x, y = a.get("match_curve_pct"), b.get("match_curve_pct")   # higher is better, in points
    if x is not None and y is not None:
      better += y - x >= 5
      worse += x - y >= 5
    enough = (a.get("lat_minutes") or 0) >= MIN_LAT_MINUTES and (b.get("lat_minutes") or 0) >= MIN_LAT_MINUTES
    verdict = "unclear"
    if enough and better >= 2 and worse == 0:
      verdict = "better"
    elif enough and worse >= 2 and better == 0:
      verdict = "worse"
    return {"route": prev.get("route"), "changed": [key], "comparable": True, "param": key, "before": ps.get(key),
            "after": cur.get(key), "rows": rows, "better": better, "worse": worse, "verdict": verdict, "enough_data": enough}
  return None


def recommend(metrics: dict, sim: dict | None, settings: dict[str, int], sim_error: str | None = None) -> dict:
  """The single change worth a drive, the replay table, and why nothing else was picked."""
  notes = []
  if (metrics.get("lat_minutes") or 0) < MIN_LAT_MINUTES:
    return {"status": "not_enough_data", "reason": "lat_minutes", "candidates": [], "notes": notes}
  if sim is None:
    return {"status": "no_simulation", "reason": sim_error or "unavailable", "candidates": [], "notes": notes}
  if sim["_windows"]["bends"] < MIN_CURVE_WINDOWS:
    return {"status": "not_enough_data", "reason": "curves", "candidates": table(sim, settings), "notes": notes}
  best, best_score = None, 0.0
  for row in table(sim, settings):
    if row["reference_only"]:
      continue
    if row["score"] > best_score and not row["rejected"]:
      best, best_score = row, row["score"]
  if best is None:
    return {"status": "keep", "candidates": table(sim, settings), "notes": notes}
  return {"status": "change", "param": best["param"], "from": best["from"], "to": best["to"], "delta": best["delta"],
          "candidates": table(sim, settings), "notes": notes}


def label(param: str, frm: int, to: int) -> str:
  return f"{param}:{frm}->{to}"


def table(sim: dict, settings: dict[str, int]) -> list[dict]:
  rows = []
  for lab, delta in sim.items():
    if lab.startswith("_"):
      continue
    param, change = lab.split(":", 1)
    frm, to = (int(x) for x in change.split("->"))
    score, why = judge(delta)
    rows.append({"param": param, "from": frm, "to": to, "delta": delta, "score": round(score, 2), "rejected": why,
                 "reference_only": param in REFERENCE_ONLY})
  rows.sort(key=lambda r: (r["reference_only"], -r["score"]))
  return rows
