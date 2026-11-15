"""Traverse full-NNFF behaviour that follows FrogPilot's NNFF. Real controller, donor NN model and
vehicle model; Params are in memory, no device I/O."""
import math
from types import SimpleNamespace as NS

import pytest

from openpilot.cereal import car, log
from opendbc.car.gm.values import CAR
from opendbc.car.interfaces import get_nn_model
from opendbc.car.vehicle_model import VehicleModel
from openpilot.selfdrive.modeld.constants import ModelConstants
from opendbc.car.gm.tests.test_display_speed import MemoryParams, runtime, build_cp  # noqa: F401


@pytest.fixture
def make_controller(runtime, monkeypatch):
  import openpilot.selfdrive.controls.lib.latcontrol_torque as latcontrol_torque
  monkeypatch.setattr(latcontrol_torque, "Params", MemoryParams)

  def make(fingerprint=CAR.CHEVROLET_TRAVERSE, use_nnff=True):
    CP = build_cp(runtime)
    CP.carFingerprint = str(fingerprint)
    CI = runtime.interface.CarInterface.__new__(runtime.interface.CarInterface)
    CI.CP = CP
    CI.lat_torque_nn_model = get_nn_model(str(CAR.CHEVROLET_TRAVERSE), "", allow_substitute=True)
    CI.use_nnff, CI.use_nnff_lite = use_nnff, False
    assert CI.lat_torque_nn_model is not None
    return latcontrol_torque.LatControlTorque(CP.as_reader(), CI), CP
  return make


def model(lat_accel, jerk):
  md = log.ModelDataV2.new_message()
  t = ModelConstants.T_IDXS
  md.init("acceleration")
  md.acceleration.y = [lat_accel + jerk * x for x in t]
  md.init("orientation")
  md.orientation.x = [0.0] * len(t)
  md.orientation.y = [0.0] * len(t)
  return md


def step(controller, CP, rate, plan_jerk, lat_accel=2.0, v=20.0, frames=1):
  VM = VehicleModel(CP)
  VM.update_params(1.0, 18.7)
  desired_curvature = lat_accel / v ** 2
  CS = car.CarState.new_message(vEgo=v, aEgo=0.0, steeringPressed=False, steeringRateDeg=rate,
                                steeringAngleDeg=math.degrees(VM.get_steer_from_curvature(-desired_curvature, v, 0.0)))
  params = log.LiveParametersData.new_message()
  cc = NS(angularVelocity=[], orientationNED=[])
  md = model(lat_accel, plan_jerk)
  for _ in range(frames):
    torque, _, pid_log = controller.update(True, CS, VM, params, False, desired_curvature, cc, False, md)
  return torque, pid_log


CHEVROLETS = [platform for platform in CAR if str(platform).startswith("CHEVROLET_")]
OTHER_GM = [platform for platform in CAR if not str(platform).startswith("CHEVROLET_")]


def test_every_chevrolet_platform_is_listed():
  # user request 2026-11-15: every Chevrolet, not only the Traverse family
  from openpilot.selfdrive.controls.lib.latcontrol_torque import NNFF_FROGPILOT_PARITY_CARS
  assert sorted(NNFF_FROGPILOT_PARITY_CARS) == sorted(str(platform) for platform in CHEVROLETS)
  assert len(CHEVROLETS) >= 16 and {"CHEVROLET_TRAVERSE", "CHEVROLET_EQUINOX", "CHEVROLET_BOLT_EUV",
                                    "CHEVROLET_SILVERADO", "CHEVROLET_TRAX"} <= set(NNFF_FROGPILOT_PARITY_CARS)


@pytest.mark.parametrize("fingerprint", CHEVROLETS)
def test_parity_for_every_chevrolet_full_nnff(make_controller, fingerprint):
  assert make_controller(fingerprint=fingerprint)[0].nnff_frogpilot_parity
  assert not make_controller(fingerprint=fingerprint, use_nnff=False)[0].nnff_frogpilot_parity


@pytest.mark.parametrize("fingerprint", OTHER_GM)
def test_parity_not_for_other_gm_brands(make_controller, fingerprint):
  assert not make_controller(fingerprint=fingerprint)[0].nnff_frogpilot_parity


def test_parity_takes_desired_jerk_over_lateral_delay(make_controller):
  controller, CP = make_controller()
  assert controller.desired_lat_jerk_time == pytest.approx(CP.steerActuatorDelay + 0.2)
  other, _ = make_controller(fingerprint=CAR.GMC_YUKON)
  assert other.desired_lat_jerk_time == pytest.approx(0.3)  # SteerActuatorDelay Param 0 + 0.3 s


def test_measured_jerk_reaches_nn_when_plan_has_jerk(make_controller):
  requests = []
  for rate in (-20.0, 20.0):
    controller, CP = make_controller()
    torque, _ = step(controller, CP, rate, plan_jerk=0.5)
    requests.append(torque)
  assert abs(requests[0] - requests[1]) > 0.02
  assert all(math.isfinite(x) for x in requests)


def test_other_cars_still_drop_steering_angle_jerk(make_controller):
  requests = []
  for rate in (-20.0, 20.0):
    controller, CP = make_controller(fingerprint=CAR.GMC_YUKON)
    torque, _ = step(controller, CP, rate, plan_jerk=0.5)
    requests.append(torque)
  assert requests[0] == pytest.approx(requests[1], abs=1e-9)


def test_measured_jerk_dropped_without_planned_jerk(make_controller):
  requests = []
  for rate in (-20.0, 20.0):
    controller, CP = make_controller()
    torque, _ = step(controller, CP, rate, plan_jerk=0.0)
    requests.append(torque)
  assert requests[0] == pytest.approx(requests[1], abs=1e-9)


def test_non_finite_steering_rate_keeps_controller_finite(make_controller):
  controller, CP = make_controller()
  torque, pid_log = step(controller, CP, float("nan"), plan_jerk=0.5, frames=5)
  assert math.isfinite(torque) and math.isfinite(pid_log.i)
  torque, pid_log = step(controller, CP, 0.0, plan_jerk=0.5, frames=5)
  assert math.isfinite(torque) and math.isfinite(pid_log.i)
