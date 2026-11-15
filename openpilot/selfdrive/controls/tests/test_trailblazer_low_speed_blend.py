"""Trailblazer only (2026-11-16): NNFF's high-lateral-acceleration error blend is cut to 0.3x up to 55 km/h and back
to 1x at 65 km/h, to stop ~1 Hz hunting in 33-47 km/h curves. Every other car keeps the FrogPilot/sunnypilot blend."""
import numpy as np
import pytest

from openpilot.selfdrive.controls.lib import latcontrol_torque as lt


def stock_blend(desired_lateral_accel):
  return float(np.interp(abs(desired_lateral_accel), [1.0, 2.0], [0.0, 1.0]))


@pytest.mark.parametrize("v_kph", [0, 20, 40, 55, 60, 65, 100])
@pytest.mark.parametrize("lat_accel", [-3.0, -1.5, -0.5, 0.0, 0.8, 1.0, 1.4, 2.0, 2.6])
def test_blend_is_unchanged_without_the_trailblazer_scale(v_kph, lat_accel):
  assert lt.get_error_blend_factor(lat_accel, v_kph / 3.6, False) == pytest.approx(stock_blend(lat_accel), abs=1e-12)


@pytest.mark.parametrize("v_kph, scale", [(0, 0.3), (30, 0.3), (40, 0.3), (55, 0.3), (60, 0.65), (65, 1.0), (90, 1.0)])
@pytest.mark.parametrize("lat_accel", [-2.6, -1.5, 0.9, 1.5, 2.0, 3.2])
def test_trailblazer_blend_scale_by_speed(v_kph, scale, lat_accel):
  assert lt.get_error_blend_factor(lat_accel, v_kph / 3.6, True) == pytest.approx(stock_blend(lat_accel) * scale, abs=1e-9)


def test_scale_never_raises_the_blend_and_reaches_stock_at_65_kph():
  for v in np.linspace(0, 40, 401):
    for a in np.linspace(-4, 4, 81):
      scaled = lt.get_error_blend_factor(a, v, True)
      assert 0.0 <= scaled <= stock_blend(a) + 1e-12
      if v >= 65 / 3.6:
        assert scaled == pytest.approx(stock_blend(a), abs=1e-12)


def test_only_the_trailblazer_is_listed():
  assert lt.NNFF_LOW_SPEED_ERROR_BLEND_CARS == ("CHEVROLET_TRAILBLAZER",)
  # the Traverse runs the Trailblazer's NNFF model but is a different fingerprint, so it keeps the stock blend
  assert "CHEVROLET_TRAVERSE" not in lt.NNFF_LOW_SPEED_ERROR_BLEND_CARS
  assert "CHEVROLET_TRAILBLAZER_CC" not in lt.NNFF_LOW_SPEED_ERROR_BLEND_CARS


# --- controller level: the real controller and NNFF model files, NNFF switched on in a throwaway params prefix ---

def make_controller(name):
  from openpilot.common.params import Params
  from openpilot.common.prefix import OpenpilotPrefix
  from opendbc.car.car_helpers import interfaces
  CarInterface = interfaces[name]
  CP = CarInterface.get_non_essential_params(name)
  # Own prefix even if the repo conftest is not loaded (e.g. an overlay run on the device), so /data/params is never written
  with OpenpilotPrefix():
    Params().put_bool("NNFF", True)
    CI = CarInterface(CP)
    if not CI.use_nnff:
      pytest.skip(f"no NNFF model for {name} in this build")
    lc = lt.LatControlTorque(CP.as_reader(), CI)
  return lc, CP


def run_once(name, v_ego, desired_lat_accel, steer_angle_deg):
  """One update in a steady right curve where the car turns less than asked, as at the Trailblazer's hunting curves."""
  from types import SimpleNamespace as NS
  from openpilot.cereal import car, log
  from opendbc.car.vehicle_model import VehicleModel
  lc, CP = make_controller(name)
  VM = VehicleModel(CP.as_reader())
  params = log.LiveParametersData.new_message()
  CS = car.CarState.new_message(vEgo=v_ego, steeringAngleDeg=steer_angle_deg)
  model = NS(orientation=NS(x=[0.0] * 33, y=[0.0] * 33), acceleration=NS(y=[desired_lat_accel] * 33))
  _, _, pid_log = lc.update(True, CS, VM, params, False, desired_lat_accel / v_ego ** 2,
                            NS(angularVelocity=[], orientationNED=[]), False, model_data=model)
  return lc, pid_log.error


@pytest.mark.parametrize("v_kph", [40, 50])
def test_trailblazer_error_mixes_only_0_3_of_the_error_response_at_low_speed(monkeypatch, v_kph):
  v = v_kph / 3.6
  lc, err_scaled = run_once("CHEVROLET_TRAILBLAZER", v, 2.5, -20.0)
  assert lc.low_speed_error_blend
  monkeypatch.setattr(lt, "NNFF_LOW_SPEED_ERROR_BLEND_CARS", ())
  _, err_stock = run_once("CHEVROLET_TRAILBLAZER", v, 2.5, -20.0)      # stock: blend 1 -> the error-input response
  monkeypatch.setattr(lt, "NNFF_LOW_SPEED_ERROR_BLEND_CARS", ("CHEVROLET_TRAILBLAZER",))
  monkeypatch.setattr(lt, "LOW_SPEED_ERROR_BLEND_V", [0.0, 1.0])
  _, err_pure = run_once("CHEVROLET_TRAILBLAZER", v, 2.5, -20.0)       # blend 0 -> the setpoint/measurement difference
  assert abs(err_stock) > abs(err_pure) > 0.0                           # the scenario really uses the blend
  assert err_scaled == pytest.approx(0.7 * err_pure + 0.3 * err_stock, rel=1e-6, abs=1e-9)


def test_trailblazer_error_is_stock_from_65_kph(monkeypatch):
  v = 80 / 3.6
  _, err_scaled = run_once("CHEVROLET_TRAILBLAZER", v, 2.5, -6.0)
  monkeypatch.setattr(lt, "NNFF_LOW_SPEED_ERROR_BLEND_CARS", ())
  _, err_stock = run_once("CHEVROLET_TRAILBLAZER", v, 2.5, -6.0)
  assert err_scaled == pytest.approx(err_stock, abs=1e-12)


def test_traverse_keeps_the_stock_blend(monkeypatch):
  v = 40 / 3.6
  lc, err = run_once("CHEVROLET_TRAVERSE", v, 2.5, -20.0)
  assert not lc.low_speed_error_blend
  monkeypatch.setattr(lt, "NNFF_LOW_SPEED_ERROR_BLEND_CARS", ("CHEVROLET_TRAILBLAZER", "CHEVROLET_TRAVERSE"))
  _, err_if_listed = run_once("CHEVROLET_TRAVERSE", v, 2.5, -20.0)
  assert abs(err_if_listed) < abs(err)   # the same input would change if the Traverse were listed, so it is not
