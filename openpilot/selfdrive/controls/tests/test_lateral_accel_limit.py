"""LateralAccelLimit (Carrot Web, user request 2026-11-16: make the lateral acceleration limit adjustable from 3.0 to
4.5 m/s^2). The default keeps the previous limits on both paths; the setting replaces the 3.0 everywhere it applied."""
import ast
import json
import math
import re
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

from opendbc.car.vehicle_model import ACCELERATION_DUE_TO_GRAVITY as G
from openpilot.selfdrive.controls.lib.drive_helpers import (
  MAX_LATERAL_ACCEL_NO_ROLL,
  MAX_LATERAL_ACCEL_NO_ROLL_LOW_SPEED,
  clip_curvature,
  lateral_accel_limit,
)

ROOT = Path(__file__).resolve().parents[3]
SPEEDS_KPH = (30, 50, 79, 80, 90, 100, 110, 119, 120, 130, 160)


def limit_at(v_ego, roll=0.0, **kwargs):
  """Largest lateral acceleration clip_curvature lets through (previous request already at the target)."""
  k = 20.0 / v_ego ** 2
  curvature, _ = clip_curvature(v_ego, k, k, roll, **kwargs)
  return curvature * v_ego ** 2


def previous_limit(v_ego, low_speed_allowance):
  """The limit before this setting existed (drive_helpers at 6893ca87)."""
  if not low_speed_allowance:
    return MAX_LATERAL_ACCEL_NO_ROLL
  return float(np.interp(v_ego, [80 / 3.6, 120 / 3.6], [MAX_LATERAL_ACCEL_NO_ROLL_LOW_SPEED, MAX_LATERAL_ACCEL_NO_ROLL]))


@pytest.mark.parametrize("setting, expected", [(30, 3.0), (31, 3.1), (37, 3.7), (45, 4.5)])
def test_setting_reads_in_tenths(setting, expected):
  assert lateral_accel_limit(setting) == pytest.approx(expected)


@pytest.mark.parametrize("setting", [0, 29, 46, 100, -30, 2 ** 31, None, True, "40", "", "abc", 40.0, 45.9,
                                     math.nan, math.inf])
def test_unreadable_or_out_of_range_setting_keeps_the_stock_limit(setting):
  assert lateral_accel_limit(setting) == MAX_LATERAL_ACCEL_NO_ROLL


def read_lat_accel_limit():
  """Controls.read_lat_accel_limit from the production controlsd source (the module needs native messaging)."""
  path = ROOT / "selfdrive" / "controls" / "controlsd.py"
  tree = ast.parse(path.read_text(encoding="utf8"))
  cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "Controls")
  node = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "read_lat_accel_limit")
  ns = {"lateral_accel_limit": lateral_accel_limit, "MAX_LATERAL_ACCEL_NO_ROLL": MAX_LATERAL_ACCEL_NO_ROLL}
  exec(compile(ast.Module(body=[node], type_ignores=[]), str(path), "exec"), ns)
  return ns["read_lat_accel_limit"]


class StoredParams:
  """Params.get for an INT key: Python int() of the stored bytes, None when that fails (params_pyx _cpp2python)."""
  def __init__(self, raw):
    self.raw = raw

  def get(self, key, block=False, return_default=False):
    assert key == "LateralAccelLimit"
    if isinstance(self.raw, Exception):
      raise self.raw
    if self.raw == b"":
      return None
    try:
      return int(self.raw)
    except ValueError:
      return None

  def get_int(self, key, block=False):
    raise AssertionError("get_int parses with the native std::stoi, which aborts the process on a corrupt value")


@pytest.mark.parametrize("raw, expected", [
  (b"30", 3.0), (b"45", 4.5), (b"38", 3.8), (b"", 3.0), (b"46", 3.0), (b"abc", 3.0), (b"nan", 3.0), (b"inf", 3.0),
  (b"2147483648", 3.0), (b"45.9", 3.0), (b"45junk", 3.0), (b"45e99", 3.0), (KeyError("LateralAccelLimit"), 3.0),
])
def test_controlsd_reads_through_params_get_and_falls_back_on_corrupt_values(raw, expected):
  assert read_lat_accel_limit()(NS(params=StoredParams(raw))) == pytest.approx(expected)


@pytest.mark.parametrize("low_speed_allowance", [True, False])
@pytest.mark.parametrize("kph", SPEEDS_KPH)
def test_default_keeps_the_previous_limit_on_both_paths(kph, low_speed_allowance):
  v = kph / 3.6
  expected = previous_limit(max(v, 1.0), low_speed_allowance)
  assert limit_at(v, low_speed_allowance=low_speed_allowance) == pytest.approx(expected, abs=1e-9)
  assert limit_at(v, low_speed_allowance=low_speed_allowance,
                  max_lat_accel_no_roll=lateral_accel_limit(30)) == pytest.approx(expected, abs=1e-9)


@pytest.mark.parametrize("setting", [30, 36, 45])
@pytest.mark.parametrize("kph", SPEEDS_KPH)
def test_frogpilot_parity_path_uses_the_setting_at_every_speed(kph, setting):
  limit = lateral_accel_limit(setting)
  assert limit_at(kph / 3.6, low_speed_allowance=False, max_lat_accel_no_roll=limit) == pytest.approx(limit, abs=1e-9)


@pytest.mark.parametrize("setting", [30, 36, 45])
def test_carrot_path_keeps_4_5_below_80_kph_and_ends_at_the_setting(setting):
  limit = lateral_accel_limit(setting)
  for kph in (30, 50, 80):   # below ~30 km/h MAX_CURVATURE (0.2 1/m) is the tighter limit
    assert limit_at(kph / 3.6, max_lat_accel_no_roll=limit) == pytest.approx(4.5, abs=1e-9)
  assert limit_at(100 / 3.6, max_lat_accel_no_roll=limit) == pytest.approx((4.5 + limit) / 2, abs=1e-9)
  for kph in (120, 160):
    assert limit_at(kph / 3.6, max_lat_accel_no_roll=limit) == pytest.approx(limit, abs=1e-9)


@pytest.mark.parametrize("roll", [-0.05, 0.05])
def test_roll_compensation_and_both_directions_stay(roll):
  v = 25.0
  for low_speed_allowance in (True, False):
    for sign in (1, -1):
      k = sign * 20.0 / v ** 2
      curvature, limited = clip_curvature(v, k, k, roll, low_speed_allowance=low_speed_allowance, max_lat_accel_no_roll=4.0)
      base = float(np.interp(v, [80 / 3.6, 120 / 3.6], [4.5, 4.0])) if low_speed_allowance else 4.0
      assert curvature * v ** 2 == pytest.approx(roll * G + sign * base, abs=1e-9)
      assert limited


def test_jerk_limit_is_unchanged():
  v = 20.0
  curvature, _ = clip_curvature(v, 0.0, 1.0, 0.0, max_lat_accel_no_roll=4.5)
  assert curvature * v ** 2 == pytest.approx(5.0 * 0.01, abs=1e-9)   # MAX_LATERAL_JERK * DT_CTRL


def test_setting_is_registered_with_the_stock_default_and_shown_in_steering_feel():
  settings = json.loads((ROOT / "selfdrive" / "carrot_settings.json").read_text(encoding="utf-8"))
  entry = next(p for p in settings["params"] if p["name"] == "LateralAccelLimit")
  assert (entry["min"], entry["max"], entry["default"], entry["unit"]) == (30, 45, 30, 1)
  assert entry.get("risk") == "high"

  def menus(node):
    if isinstance(node, dict):
      if node.get("ko") == "조향감":
        yield node
      for v in node.values():
        yield from menus(v)
    elif isinstance(node, list):
      for v in node:
        yield from menus(v)
  assert any("LateralAccelLimit" in m["params"] for m in menus(settings["menu"]))

  keys = (ROOT / "common" / "params_keys.h").read_text(encoding="utf-8")
  assert re.search(r'\{"LateralAccelLimit", \{PERSISTENT, INT, "30"\}\}', keys)
