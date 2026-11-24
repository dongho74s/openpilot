"""Last-drive tuning advice, step 3: replay the drive's curves with other settings (same road, same model demand).

The repo's own LatControlTorque, controlsd's curvature filter and clip_curvature, and the GM torque limits run
against a steering plant fitted to this drive (the median torque that held each speed and lateral accel while
openpilot steered). There is no driver and no model reaction in the loop, so only settings compared on the same
windows mean anything. The Mac analysis of 2026-11-24 used it for the lateral accel limit, the integrator and the
torque-limit P filter; P is judged from real drives (the plant got P wrong before).

Runs only in the analysis process (runner.py): it swaps openpilot.common.params.Params for a read-only stand-in
holding the drive's initData values, so building the car interface here never writes the comma's real settings.
"""
from __future__ import annotations

import math
import os
import sys
from types import SimpleNamespace

import numpy as np

from openpilot.selfdrive.carrot.tuning_advice.extract import C

DT = 0.01
PROCESS_ENV = "CARROT_TUNING_ADVICE_PROCESS"
# plant dynamics the Mac fit chose (empirical curve, play band 0.03, 0.05 s dead time, 5 rad/s, damping 1.4)
PLANT_FRICTION, PLANT_DELAY, PLANT_WN, PLANT_ZETA = 0.03, 0.05, 5.0, 1.4
CHECK_EVERY_FRAMES = 500     # about 0.35 s of replay on a comma 4 between the runner's pacing checks
V_GRID = np.arange(4.0, 36.01, 2.0)
LA_GRID = np.arange(-4.5, 4.5001, 0.02)
LA_BINS = np.arange(-4.5, 4.51, 0.25)


class StandInParams:
  """Read-only Params for the simulator: initData values plus the setting under test; writes are dropped."""
  values: dict[str, bytes] = {}

  def __init__(self, *_a, **_k):
    pass

  @classmethod
  def _raw(cls, key):
    v = cls.values.get(key.decode() if isinstance(key, bytes) else str(key))
    return None if v is None else (v.decode(errors="replace") if isinstance(v, bytes) else str(v))

  def get(self, key, block=False, encoding=None, return_default=False):
    v = self._raw(key)
    if v is None:
      return None
    for cast in (int, float):
      try:
        return cast(v)
      except ValueError:
        pass
    return v

  def get_int(self, key, block=False, return_default=False, default=0):
    try:
      return int(float(self._raw(key)))
    except (TypeError, ValueError):
      return default

  def get_float(self, key, block=False, return_default=False, default=0.0):
    try:
      return float(self._raw(key))
    except (TypeError, ValueError):
      return default

  def get_bool(self, key, block=False, default=False):
    v = self._raw(key)
    return default if v is None else v.strip().lower() in ("1", "true")

  def put(self, *_a, **_k):
    pass

  put_int = put_float = put_bool = put_nonblocking = put_bool_nonblocking = remove = put


def install_stand_in_params(values: dict[str, bytes]) -> None:
  if os.environ.get(PROCESS_ENV) != "1":
    raise RuntimeError("the tuning simulator only runs in its own analysis process")
  import openpilot.common.params as params_module
  StandInParams.values = dict(values)
  real = params_module.Params
  if real is StandInParams:
    return
  params_module.Params = StandInParams
  for mod in list(sys.modules.values()):
    if mod is not None and getattr(mod, "Params", None) is real:
      try:
        setattr(mod, "Params", StandInParams)
      except Exception:
        pass


def fit_plant_curve(segments: list[tuple[np.ndarray, dict]], checkpoint=None):
  """Torque that held each (speed, lateral accel) while openpilot steered, the driver was off the wheel and the lateral
  accel changed slowly; monotonic per speed, extended past the data with the outer slope (|torque| <= 1.6).
  None if fewer than 3 speeds have enough samples."""
  V, L, T = [], [], []
  for a, CC in segments:
    if checkpoint is not None:
      checkpoint()
    act = a[:, CC["act"]]
    smooth = np.convolve(np.nan_to_num(act), np.ones(25) / 25, mode="same")
    jerk = np.gradient(smooth) / DT
    ok = (a[:, CC["latActive"]] > .5) & (a[:, CC["pressed"]] < .5) & (a[:, CC["v"]] > 3) & (np.abs(jerk) < 0.4) & np.isfinite(act)
    V.append(a[ok, CC["v"]])
    L.append(smooth[ok])
    T.append(a[ok, CC["sent"]])
  if not V:
    return None
  V, L, T = map(np.concatenate, (V, L, T))
  curve = np.full((len(V_GRID), len(LA_GRID)), np.nan)
  for i, v in enumerate(V_GRID):
    if checkpoint is not None:
      checkpoint()
    sel = np.abs(V - v) < 2.0
    cx, cy = [], []
    for lo, hi in zip(LA_BINS[:-1], LA_BINS[1:], strict=True):
      m = sel & (L >= lo) & (L < hi)
      if m.sum() >= 40:
        cx.append((lo + hi) / 2)
        cy.append(np.median(T[m]))
    if len(cx) < 6:
      continue
    cx, cy = np.array(cx), np.array(cy)
    sgn = np.sign(np.polyfit(cx, cy, 1)[0]) or -1.0
    cy = sgn * np.maximum.accumulate(sgn * cy)
    lo_s = np.polyfit(cx[:4], cy[:4], 1)[0]
    hi_s = np.polyfit(cx[-4:], cy[-4:], 1)[0]
    y = np.interp(LA_GRID, cx, cy)
    y = np.where(LA_GRID < cx[0], cy[0] + lo_s * (LA_GRID - cx[0]), y)
    y = np.where(LA_GRID > cx[-1], cy[-1] + hi_s * (LA_GRID - cx[-1]), y)
    curve[i] = np.clip(y, -1.6, 1.6)
  good = np.where(np.isfinite(curve[:, 0]))[0]
  if len(good) < 3:
    return None
  for i in range(len(V_GRID)):
    if not np.isfinite(curve[i, 0]):
      curve[i] = curve[good[np.argmin(np.abs(good - i))]]
  return curve


class Plant:
  """Steady curve from the drive plus a play band, a dead time and second-order dynamics toward the steady value."""
  def __init__(self, curve: np.ndarray, friction: float = PLANT_FRICTION):
    self.C = curve
    self.fric = friction
    self.d, self.wn, self.z = int(round(PLANT_DELAY / DT)), PLANT_WN, PLANT_ZETA
    self.buf: list[float] = []
    self.la = self.lar = self.eff = 0.0

  def la_ss(self, tq: float, v: float) -> float:
    i = int(np.clip(np.searchsorted(V_GRID, v) - 1, 0, len(V_GRID) - 2))
    w = float(np.clip((v - V_GRID[i]) / 2.0, 0, 1))
    c = (1 - w) * self.C[i] + w * self.C[i + 1]
    if c[-1] < c[0]:
      return float(np.interp(-tq, -c, LA_GRID))
    return float(np.interp(tq, c, LA_GRID))

  def reset(self, la: float, torque: float, lar: float) -> None:
    self.la, self.lar, self.eff = la, lar, torque
    self.buf = [torque] * (self.d + 1)

  def step(self, tq: float, v: float) -> float:
    self.buf.append(tq)
    t = self.buf.pop(0)
    self.eff = min(max(self.eff, t - self.fric), t + self.fric)
    acc = self.wn ** 2 * (self.la_ss(self.eff, v) - self.la) - 2 * self.z * self.wn * self.lar
    self.lar += acc * DT
    self.la += self.lar * DT
    return self.la


def windows(segments: list[tuple[np.ndarray, dict]], checkpoint=None):
  """(segment index, start, end, kind): curve = demand > 1.5 m/s^2 for >= 2 s above 45 km/h, turn = the same at
  15-45 km/h, straight = > 60 km/h with demand < 0.3 for 8 s; with steering on and the driver off the wheel."""
  from openpilot.selfdrive.carrot.tuning_advice.metrics import stretches
  out = []
  for si, (a, CC) in enumerate(segments):
    if checkpoint is not None:
      checkpoint()
    v = a[:, CC["v"]]
    dem = a[:, CC["mdlCurv"]] * v ** 2
    ok = (a[:, CC["latActive"]] > .5) & (a[:, CC["pressed"]] < .5)
    smooth = np.convolve(np.abs(dem), np.ones(20) / 20, mode="same")
    for k, j in stretches(ok & (smooth > 1.5) & (v > 15 / 3.6), 200):
      kind = "turn" if np.median(v[k:j]) * 3.6 < 45 else "curve"
      s, e = max(k - 200, 300), min(j + 200, len(a) - 1)
      if e - s > 300 and (a[s:e, CC["latActive"]] > .5).mean() > .95 and (a[s:e, CC["pressed"]] > .5).mean() < .1:
        out.append((si, s, e, kind))
    for k, j in stretches(ok & (smooth < 0.3) & (v * 3.6 > 60), 800):
      s, e = k + 100, min(k + 1300, j)
      if e - s > 600:
        out.append((si, s, e, "straight"))
  return out


class Simulator:
  def __init__(self, car_params: bytes, params: dict[str, bytes], curve: np.ndarray):
    install_stand_in_params(params)
    import importlib
    from openpilot.cereal import car
    self._cp_bytes = car_params
    with car.CarParams.from_bytes(car_params, traversal_limit_in_words=2 ** 63) as cp:
      self.CP = cp
    if str(self.CP.brand) != "gm" or self.CP.lateralTuning.which() != "torque":
      raise NotImplementedError("the simulator covers GM torque steering only")
    base = importlib.import_module(f"opendbc.car.{self.CP.brand}.interface").CarInterface  # not every brand's module
    self.Interface = type("IsolatedInterface", (base,), {
      "CarState": staticmethod(lambda cp: SimpleNamespace(get_can_parsers=lambda _cp: {})),
      "CarController": staticmethod(lambda _dbc, _cp: None),
    })
    self.base_params = dict(params)
    self.curve = curve

  def controller(self, overrides: dict[str, int]):
    from opendbc.car.gm.values import CarControllerParams
    from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque
    values = dict(self.base_params)
    values.update({k: str(int(v)).encode() for k, v in overrides.items()})
    StandInParams.values = values
    ci = self.Interface(self.CP)
    lac = LatControlTorque(self.CP, ci)
    lac.params = StandInParams()
    lac.read_torque_limit_p_filter()
    return lac, self.torque_limits(CarControllerParams(self.CP))

  def torque_limits(self, gm):
    """The GM car controller's CustomSteerMax / CustomSteerDeltaUp / CustomSteerDeltaDown, clipped as it does."""
    p = StandInParams()
    steer_max, delta_up, delta_down = (p.get_int(k) for k in ("CustomSteerMax", "CustomSteerDeltaUp", "CustomSteerDeltaDown"))
    if steer_max > 0:
      gm.STEER_MAX = min(steer_max, gm.STEER_MAX_EPS)
    if delta_up > 0:
      gm.STEER_DELTA_UP = min(delta_up, gm.max_custom_steer_delta_up())
    if delta_down > 0:
      gm.STEER_DELTA_DOWN = min(delta_down, gm.PANDA_MAX_RATE_DOWN)
    return gm

  def steer_ratio_settings(self) -> tuple[float, float]:
    """(SteerRatioRate, CustomSR) as controlsd reads them for the vehicle model."""
    p = StandInParams()
    StandInParams.values = self.base_params
    return p.get_float("SteerRatioRate"), p.get_float("CustomSR")

  def run(self, a: np.ndarray, models: list, s: int, e: int, overrides: dict[str, int], lal: float, jerk: float,
          warm: int = 200, friction: float = PLANT_FRICTION, checkpoint=None) -> np.ndarray:
    """Rows of (v, model demand, desired, simulated actual, torque sent, logged actual, lane offset, lp1, lp2)."""
    from opendbc.car import apply_driver_steer_torque_limits
    from opendbc.car.vehicle_model import VehicleModel
    from openpilot.selfdrive.controls.lib.drive_helpers import clip_curvature, smooth_value
    from openpilot.selfdrive.controls.lib.steer_ratio import resolve_vehicle_model_steer_ratio
    sr_rate, custom_sr = self.steer_ratio_settings()
    lac, gm = self.controller(overrides)
    parity = bool(getattr(lac, "nnff_frogpilot_parity", False))
    VM = VehicleModel(self.CP)
    plant = Plant(self.curve, friction)
    i0 = max(s - warm, 6)
    last = int(round(a[i0, C["sent"]] * gm.STEER_MAX))
    limited = False
    prev_des = a[i0, C["desCurv"]]
    ang_prev = a[i0, C["angle"]]
    delay = float("nan")
    rec = []
    for k in range(i0, e):
      if checkpoint is not None and (k - i0) % CHECK_EVERY_FRAMES == 0:
        checkpoint()
      row = a[k]
      v = max(row[C["v"]], 0.5)
      roll, off = row[C["roll"]], row[C["angleOffset"]]
      if parity and math.isfinite(row[C["latDelay"]]) and row[C["latDelay"]] != delay:
        delay = row[C["latDelay"]]
        lac.update_live_delay(delay)
      VM.update_params(max(row[C["stiffness"]], 0.1), resolve_vehicle_model_steer_ratio(row[C["sR"]], sr_rate, custom_sr))
      closed = k >= s
      if k == s:
        plant.reset(row[C["act"]], last / gm.STEER_MAX, (a[k, C["act"]] - a[k - 5, C["act"]]) / (5 * DT))
        ang_prev = row[C["angle"]]
      if closed:
        ang = math.degrees(VM.get_steer_from_curvature(-plant.la / v ** 2, v, roll)) + off
        rate = (ang - ang_prev) / DT
        ang_prev = ang
      else:
        ang, rate = row[C["angle"]], row[C["rate"]]
      new = smooth_value(row[C["mdlCurv"]], prev_des, 0.1)
      des, _ = clip_curvature(v, prev_des, new, roll, low_speed_allowance=not parity, max_lat_accel_no_roll=lal,
                              max_lat_jerk=jerk)
      prev_des = des
      m = models[int(max(row[C["model_idx"]], 0))]
      model = SimpleNamespace(acceleration=SimpleNamespace(y=list(m[0])), orientation=SimpleNamespace(x=list(m[1]), y=list(m[2])))
      CS = SimpleNamespace(vEgo=v, aEgo=row[C["a"]], steeringAngleDeg=ang, steeringRateDeg=rate, steeringPressed=False)
      CC = SimpleNamespace(angularVelocity=[0.0, 0.0, row[C["angZ"]]], orientationNED=[0.0, row[C["oriP"]], 0.0])
      steer, _, _ = lac.update(True, CS, VM, SimpleNamespace(roll=roll, angleOffsetDeg=off), limited, des, CC, False,
                               model_data=model)
      if (k - i0) % gm.STEER_STEP == 0:
        last = apply_driver_steer_torque_limits(int(round(steer * gm.STEER_MAX)), last, 0.0, gm)
      sent = last / gm.STEER_MAX
      limited = abs(steer - sent) > 1e-2
      if closed:
        plant.step(sent, v)
        rec.append((v, row[C["mdlCurv"]] * v ** 2, des * v ** 2, plant.la, sent, row[C["act"]], row[C["laneOff"]],
                    row[C["lp1"]], row[C["lp2"]]))
    return np.array(rec)


def _hp(x, n=25):
  return x - np.convolve(x, np.ones(n) / n, mode="same")


def _shift(acc):
  """double integral of a lateral-acceleration difference (m), from no offset and no lateral speed"""
  return np.cumsum(np.cumsum(acc) * DT) * DT


CORRECTION_HORIZON = 200   # 2 s: the model steers a lateral offset back within about this (it is not in the loop)


def _shift_bounded(acc):
  """Lateral offset (m) a lateral-acceleration difference builds within the last 2 s (as if the model removed anything
  older): the double integral over a sliding 2 s horizon, so an offset cannot pile up over a long replay window."""
  kernel = np.arange(CORRECTION_HORIZON) * DT * DT
  return np.convolve(acc, kernel)[:len(acc)]


def window_metrics(r: np.ndarray, ref: np.ndarray | None, kind: str) -> dict:
  v, dem, des, act, sent, _act_log, lane, lp1, lp2 = r.T
  out = {"jit": float(np.std(_hp(sent)[30:-30])), "sat": float(np.mean(np.abs(sent) >= 0.99))}
  from openpilot.selfdrive.carrot.tuning_advice.metrics import releases
  out["rel"] = len(releases(sent))
  out["err"] = float(np.sqrt(np.mean((act - dem) ** 2)))
  if kind == "straight":
    return out
  sgn = np.sign(np.convolve(dem, np.ones(50) / 50, mode="same"))
  under = (dem - act) * sgn
  best = 0.0
  for k in range(0, len(r) - 200, 25):
    best = max(best, float(np.max(_shift(under[k:k + 200]))))
  out["drift2s"] = best
  if ref is not None and len(ref) == len(r):
    shift = _shift_bounded(act - ref[:, 3])
    ok = (lp1 > .5) & (lp2 > .5) & np.isfinite(lane) & (np.abs(dem) > 1.5)
    if ok.sum() > 20:
      out["laneOut"] = float(np.nanpercentile(((lane - shift) * sgn)[ok], 90))
  return out
