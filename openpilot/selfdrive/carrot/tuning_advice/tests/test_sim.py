import numpy as np
import pytest

from openpilot.selfdrive.carrot.tuning_advice import sim as S
from openpilot.selfdrive.carrot.tuning_advice.extract import COLS, C


def steady_drive(n=60000):
  """Slow lateral accel sweeps at several speeds with torque = -0.3 x lateral accel (torque + left, accel + right)."""
  a = np.zeros((n, len(COLS)))
  t = np.arange(n) * 0.01
  a[:, C["v"]] = 8 + 2 * (np.arange(n) // 6000)        # 8, 10, ... 26 m/s blocks of a minute
  la = 2.5 * np.sin(2 * np.pi * t / 60.0)
  a[:, C["act"]] = la
  a[:, C["sent"]] = -0.3 * la
  a[:, C["latActive"]] = 1
  return a


def test_plant_curve_inverts_the_logged_torque():
  curve = S.fit_plant_curve([(steady_drive(), C)])
  assert curve is not None
  plant = S.Plant(curve, friction=0.0)
  for v in (10.0, 20.0):
    for la in (-1.5, 0.5, 2.0):
      assert abs(plant.la_ss(-0.3 * la, v) - la) < 0.15


def test_plant_settles_at_the_steady_value():
  curve = S.fit_plant_curve([(steady_drive(), C)])
  plant = S.Plant(curve, friction=0.0)
  plant.reset(0.0, 0.0, 0.0)
  for _ in range(500):
    plant.step(-0.3, 15.0)
  assert abs(plant.la - 1.0) < 0.1


def test_no_curve_without_enough_steady_steering():
  assert S.fit_plant_curve([(steady_drive(3000), C)]) is None


def test_bounded_shift_does_not_pile_up():
  diff = np.full(3000, 0.1)                            # 0.1 m/s^2 for 30 s
  assert S._shift(diff)[-1] > 40                         # unbounded double integral: 45 m
  assert abs(S._shift_bounded(diff)[-1] - 0.5 * 0.1 * 2.0 ** 2) < 0.01   # 2 s horizon: 0.2 m


def test_stand_in_params_read_types_and_drop_writes():
  S.StandInParams.values = {"LateralTorqueKpV": b"160", "NNFF": b"1", "Name": b"abc"}
  p = S.StandInParams()
  assert p.get("LateralTorqueKpV") == 160 and p.get_float("LateralTorqueKpV") == 160.0 and p.get_int("LateralTorqueKpV") == 160
  assert p.get_bool("NNFF") and not p.get_bool("Missing") and p.get("Missing") is None and p.get("Name") == "abc"
  p.put_int("LateralTorqueKpV", 1)
  p.put_nonblocking("X", "1")
  p.remove("NNFF")
  assert S.StandInParams.values == {"LateralTorqueKpV": b"160", "NNFF": b"1", "Name": b"abc"}


def test_stand_in_params_only_in_the_analysis_process(monkeypatch):
  monkeypatch.delenv(S.PROCESS_ENV, raising=False)
  with pytest.raises(RuntimeError):
    S.install_stand_in_params({})
