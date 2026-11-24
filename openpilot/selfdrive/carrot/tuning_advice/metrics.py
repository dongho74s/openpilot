"""Last-drive tuning advice, step 2: what the logged drive itself says about the steering settings (no simulation).

Signs: desired/actual lateral accel, curvatures and lane offsets are + to the right; steering torques + to the left
(carOutput torque is the controller's, so it carries the controller's sign).
"""
from __future__ import annotations

import numpy as np

DT = 0.01
LAT_MIN_SPEED = 8 / 3.6
LOW_SPEED = 45 / 3.6          # turns and city streets below, curves above
CURVE_LA = 1.0                # m/s^2 of demand that counts as a curve
MIN_STRETCH = 300             # 3 s of continuous lateral control
# "how well the car followed" as on debug plot 6 (Y: torqueState.actualLateralAccel, G: desiredLateralAccel)
MATCH_TOL_ABS = 0.3           # m/s^2
MATCH_TOL_REL = 0.2           # or 20 % of the demand, whichever is larger
STRAIGHT_LA = 0.5


def sm(x: np.ndarray, n: int) -> np.ndarray:
  return np.convolve(x, np.ones(n) / n, mode="same")


def stretches(mask: np.ndarray, minlen: int) -> list[tuple[int, int]]:
  out, k, n = [], 0, len(mask)
  while k < n:
    if mask[k]:
      j = k
      while j < n and mask[j]:
        j += 1
      if j - k >= minlen:
        out.append((k, j))
      k = j
    else:
      k += 1
  return out


def swings(err: np.ndarray, thr1: float, thr2: float, win: int) -> list[int]:
  """Peaks of |err| >= thr1 followed within win samples by an opposite peak >= thr2."""
  ev, k = [], 0
  while k < len(err) - 1:
    e = abs(err[k])
    if e >= thr1 and e >= abs(err[max(k - 1, 0)]) and e >= abs(err[k + 1]):
      seg = err[k:k + win] * -np.sign(err[k])
      if seg.max() >= thr2:
        ev.append(k)
        k += win
        continue
    k += 1
  return ev


def releases(sent: np.ndarray) -> list[int]:
  """|torque| at the limit (>= 0.95) that drops by >= 0.3 within 0.5 s."""
  out, k = [], 0
  while k < len(sent) - 50:
    if abs(sent[k]) >= 0.95 and np.min(sent[k:k + 50] * np.sign(sent[k])) <= abs(sent[k]) - 0.3:
      out.append(k)
      k += 50
      continue
    k += 1
  return out


class _Acc:
  def __init__(self):
    self.n = self.free = self.pressed = self.sat = 0
    self.err_sq = 0.0
    self.ratio = {"low": [], "high": []}
    self.jit, self.wob_windows, self.wob = [], 0, 0
    self.over = self.demand_swings = self.rel_ctrl = 0
    self.clip_rows = self.clip_sat = 0
    self.clip_out, self.curve_out = [], []
    self.bias, self.exit_over = [], 0
    self.takeovers = {"against": 0, "with": 0, "neutral": 0}
    self.with_clipped = 0
    self.against_low_demand_over_road = 0
    self.eps_refusals = 0
    self.match_err = self.match_mag = 0.0
    self.match = {"curve": [0, 0], "straight": [0, 0], "all": [0, 0]}   # [samples within tolerance, samples]


def _stretch(acc: _Acc, a: np.ndarray, C: dict, s: int, e: int, lal: float) -> None:
  seg = slice(s, e)
  v = a[seg, C["v"]]
  pr = a[seg, C["pressed"]] > 0.5
  free = ~pr
  des = a[seg, C["desCurv"]] * v ** 2
  act = a[seg, C["act"]]
  sent = a[seg, C["sent"]]
  mdl = a[seg, C["mdlCurv"]] * v ** 2
  road = a[seg, C["laneCurv"]] * v ** 2
  lane = a[seg, C["laneOff"]]
  lane_ok = (a[seg, C["lp1"]] > 0.5) & (a[seg, C["lp2"]] > 0.5) & np.isfinite(lane)
  n = e - s
  acc.n += n
  acc.free += int(free.sum())
  acc.pressed += int(pr.sum())
  acc.err_sq += float(np.sum(((act - des) ** 2)[free]))

  # plot 6 agreement: the controller's desired lateral accel (green) against the measured one (yellow)
  pd, pa = a[seg, C["des"]][free], a[seg, C["act"]][free]
  okp = np.isfinite(pd) & np.isfinite(pa)
  pd, pa = pd[okp], pa[okp]
  e_abs = np.abs(pa - pd)
  acc.match_err += float(e_abs.sum())
  acc.match_mag += float(np.maximum(np.abs(pa), np.abs(pd)).sum())
  inside = e_abs <= np.maximum(MATCH_TOL_ABS, MATCH_TOL_REL * np.abs(pd))
  for key, m in (("curve", np.abs(pd) > CURVE_LA), ("straight", np.abs(pd) < STRAIGHT_LA), ("all", np.ones_like(inside))):
    acc.match[key][0] += int(np.sum(inside & m))
    acc.match[key][1] += int(np.sum(m))
  acc.sat += int(np.sum((np.abs(sent) >= 0.99)[free]))

  # torque jitter: 0.25 s high-pass spread, and 0.5 s windows whose detrended torque swings by more than 0.1
  if n > 100:
    acc.jit.append(float(np.std((sent - sm(sent, 25))[30:-30])))
  hp = sent - sm(sent, 50)
  for w in range(25, n - 75, 50):
    if free[w:w + 50].all():
      acc.wob_windows += 1
      if np.ptp(hp[w:w + 50]) > 0.1:
        acc.wob += 1

  # curve tracking by speed band
  hi = (np.abs(des) > CURVE_LA) & free
  low = v < LOW_SPEED
  for band, m in (("low", hi & low), ("high", hi & ~low)):
    if m.any():
      acc.ratio[band].append(np.abs(act[m]) / np.abs(des[m]))

  # controller overshoot: the car beyond the demand by >= 0.6, then the other way by >= 0.4 within 2 s
  err = sm(act - des, 15)
  err[pr] = 0
  acc.over += len(swings(err, 0.6, 0.4, 200))
  # demand swing: the demand itself reverses while the road does not
  d = sm(des, 15)
  for k in swings(d - sm(d, 300), 0.6, 0.4, 200):
    rr = road[k:k + 200]
    if np.isfinite(rr).all() and (np.sign(rr.max()) == np.sign(rr.min()) or np.ptp(rr) < 0.5):
      acc.demand_swings += 1
  # torque-limit releases the controller made (steering stayed on, driver off the wheel)
  for k in releases(sent):
    if not pr[k:k + 50].any():
      acc.rel_ctrl += 1

  # demand cut by the lateral accel limit (model asked beyond it) and where the car then sat in the lane
  sgn = np.sign(sm(des, 50))
  fast = v > 30 / 3.6
  clipped = free & fast & (np.abs(mdl) > lal) & (np.abs(mdl) - np.abs(des) > 0.2)
  acc.clip_rows += int(clipped.sum())
  acc.clip_sat += int(np.sum(clipped & (np.abs(sent) >= 0.99)))
  outward = lane * sgn                       # + = toward the outside of the curve
  m = clipped & lane_ok
  if m.any():
    acc.clip_out.append(outward[m])
  m = free & fast & ~clipped & (np.abs(des) > 1.5) & lane_ok
  if m.any():
    acc.curve_out.append(outward[m])

  # steady curves: the remaining under-turn after 1 s (integrator) and an overshoot right after the exit (windup)
  steady = free & (np.abs(des) > 1.2)
  for k, j in stretches(steady, 300):
    if np.ptp(sm(des[k:j], 50)[50:-50]) < 1.0:
      acc.bias.append(float(np.mean(((des - act) * np.sign(des))[k + 100:j])))
      tail = slice(j, min(j + 150, n))
      if j + 50 < n and free[tail].all() and np.abs(des[j:j + 100]).min() < 0.5:
        if np.max((act[tail] - des[tail]) * np.sign(des[j - 1])) > 0.5:
          acc.exit_over += 1


def _takeovers(acc: _Acc, a: np.ndarray, C: dict, lal: float) -> None:
  v = a[:, C["v"]]
  lat = a[:, C["latActive"]] > 0.5
  pr = a[:, C["pressed"]] > 0.5
  sent = a[:, C["sent"]]
  drv = a[:, C["drvTq"]]
  des = a[:, C["desCurv"]] * v ** 2
  mdl = a[:, C["mdlCurv"]] * v ** 2
  road = a[:, C["laneCurv"]] * v ** 2
  k = 150
  while k < len(a) - 30:
    if pr[k] and not pr[k - 1] and lat[k - 150:k].all() and not pr[k - 150:k].any() and v[k] > LAT_MIN_SPEED:
      op = float(np.mean(sent[k - 30:k]))
      osign = np.sign(op) if abs(op) > 0.05 else 0.0
      dsign = np.sign(np.mean(drv[k:k + 30]))
      kind = "against" if osign and dsign == -osign else ("with" if osign and dsign == osign else "neutral")
      acc.takeovers[kind] += 1
      w = slice(k - 50, k)
      if kind == "with" and np.mean(np.abs(mdl[w]) - np.abs(des[w])) > 0.2 and np.mean(np.abs(mdl[w])) > lal:
        acc.with_clipped += 1
      rw = road[w]
      if kind == "against" and v[k] < LOW_SPEED and np.isfinite(rw).all():
        if np.mean((des[w] - rw) * np.sign(np.mean(des[w]) or 1.0)) > 0.5:
          acc.against_low_demand_over_road += 1
      k += 200
      continue
    k += 1


def drive_metrics(segments: list[tuple[np.ndarray, dict]], lal: float, checkpoint=None) -> dict:
  """segments: (rows, column index) per log segment; lal: the lateral accel limit the drive used (m/s^2).
  checkpoint() runs between segments (the runner's pacing)."""
  acc = _Acc()
  for a, C in segments:
    if checkpoint is not None:
      checkpoint()
    if len(a) < MIN_STRETCH:
      continue
    v = a[:, C["v"]]
    ok = (a[:, C["latActive"]] > 0.5) & (v > LAT_MIN_SPEED)
    for s, e in stretches(ok, MIN_STRETCH):
      _stretch(acc, a, C, s, e, lal)
    _takeovers(acc, a, C, lal)
    if "faultTemp" in C:
      f = a[:, C["faultTemp"]] > 0.5
      acc.eps_refusals += int(np.sum(f[1:] & ~f[:-1] & (v[1:] > LAT_MIN_SPEED)))

  minutes = acc.n * DT / 60
  free = max(acc.free, 1)
  ratio = {b: (float(np.median(np.concatenate(r))) if r and sum(len(x) for x in r) >= 200 else None) for b, r in acc.ratio.items()}
  ratio_n = {b: int(sum(len(x) for x in r)) for b, r in acc.ratio.items()}

  def cat_median(parts):
    return float(np.median(np.concatenate(parts))) if parts and sum(len(x) for x in parts) >= 100 else None

  per_min = (lambda x: x / minutes) if minutes > 0 else (lambda x: 0.0)

  def inside_pct(key, minimum=200):
    k, n = acc.match[key]
    return round(k / n * 100, 1) if n >= minimum else None

  return {
    "lat_minutes": round(minutes, 2),
    "pressed_pct": round(acc.pressed / max(acc.n, 1) * 100, 1),
    "curve_seconds": round(sum(ratio_n.values()) * DT, 1),
    "err": round(float(np.sqrt(acc.err_sq / free)), 3) if acc.free else None,
    # 100 x (1 - sum|actual - desired| / sum max(|actual|, |desired|)): the average share of the request the car met
    "match_pct": round(max(0.0, 1 - acc.match_err / acc.match_mag) * 100, 1) if acc.match_mag > 1.0 else None,
    "match_curve_pct": inside_pct("curve"),
    "match_straight_pct": inside_pct("straight"),
    "match_all_pct": inside_pct("all"),
    "ratio_low": None if ratio["low"] is None else round(ratio["low"], 3),
    "ratio_high": None if ratio["high"] is None else round(ratio["high"], 3),
    "curve_seconds_low": round(ratio_n["low"] * DT, 1),
    "curve_seconds_high": round(ratio_n["high"] * DT, 1),
    "sat_pct": round(acc.sat / free * 100, 2),
    "jit": round(float(np.median(acc.jit)), 4) if acc.jit else None,
    "wobble_pct": round(acc.wob / acc.wob_windows * 100, 1) if acc.wob_windows else None,
    "overshoot_per_min": round(per_min(acc.over), 2),
    "demand_swing_per_min": round(per_min(acc.demand_swings), 2),
    "release_per_min": round(per_min(acc.rel_ctrl), 2),
    "clip_seconds": round(acc.clip_rows * DT, 1),
    "clip_sat_pct": round(acc.clip_sat / acc.clip_rows * 100, 1) if acc.clip_rows else None,
    "clip_outward_cm": None if cat_median(acc.clip_out) is None else round(cat_median(acc.clip_out) * 100, 1),
    "curve_outward_cm": None if cat_median(acc.curve_out) is None else round(cat_median(acc.curve_out) * 100, 1),
    "steady_curves": len(acc.bias),
    "steady_bias": round(float(np.median(acc.bias)), 3) if acc.bias else None,
    "exit_overshoots": acc.exit_over,
    "takeovers": dict(acc.takeovers),
    "takeovers_per_min": round(per_min(sum(acc.takeovers.values())), 2),
    "takeovers_with_clipped": acc.with_clipped,
    "takeovers_against_model_over_road": acc.against_low_demand_over_road,
    "eps_refusals": acc.eps_refusals,
  }
