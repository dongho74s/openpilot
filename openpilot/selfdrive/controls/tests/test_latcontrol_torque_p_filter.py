"""Torque-limit P filter of the FrogPilot-parity PID (LatTorqueLimitPFilter)."""
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.common.pid import PIDController
from openpilot.selfdrive.controls.lib.latcontrol_torque import FrogPilotWindupPID, LatControlTorque, P_FILTER_FF_BP, P_FILTER_MAX_CS


def reference_update(pid, error, speed, feedforward, freeze_integrator=False):
  """FrogPilotWindupPID.update as it was before the filter."""
  pid.speed = speed
  pid.p = float(error) * pid.k_p
  pid.f = feedforward * pid.k_f
  pid.d = 0.0
  if not freeze_integrator:
    i = pid.i + error * pid.k_i * pid.i_rate
    test_control = pid.p + i + pid.d + pid.f
    i_upperbound = pid.i if test_control > pid.pos_limit else pid.pos_limit
    i_lowerbound = pid.i if test_control < pid.neg_limit else pid.neg_limit
    pid.i = float(np.clip(i, i_lowerbound, i_upperbound))
  pid.control = float(np.clip(pid.p + pid.i + pid.d + pid.f, pid.neg_limit, pid.pos_limit))
  return pid.control


def make(tau=0.0, kp=1.0, ki=0.7, kf=1.0):
  pid = FrogPilotWindupPID(kp, ki, k_f=kf, pos_limit=1.0, neg_limit=-1.0)
  pid.p_filter_tau = tau
  return pid


def random_drive(seed, n=3000):
  rng = np.random.default_rng(seed)
  ff = np.clip(np.cumsum(rng.normal(0, 0.01, n)) + 0.5, -1.2, 1.2)
  err = rng.normal(0, 0.15, n)
  return err, ff


@pytest.mark.parametrize("seed", [0, 1, 2])
def test_off_is_the_previous_controller(seed):
  err, ff = random_drive(seed)
  new, ref = make(0.0), make(0.0)
  for e, f in zip(err, ff):
    assert new.update(e, speed=20.0, feedforward=f) == reference_update(ref, e, 20.0, f)
    assert (new.p, new.i) == (ref.p, ref.i)


def test_below_the_blend_band_nothing_changes():
  err, ff = random_drive(3)
  ff = np.clip(ff, -P_FILTER_FF_BP[0] + 1e-3, P_FILTER_FF_BP[0] - 1e-3)
  new, ref = make(0.3), make(0.0)
  for e, f in zip(err, ff):
    assert new.update(e, speed=20.0, feedforward=f) == pytest.approx(ref.update(e, speed=20.0, feedforward=f), abs=1e-12)


def test_near_the_limit_p_follows_with_the_time_constant():
  pid = make(0.3)
  for _ in range(200):
    pid.update(0.0, speed=20.0, feedforward=0.95)
  assert pid.p == pytest.approx(0.0)
  for _ in range(30):                       # 0.3 s after an error step of 0.1 (kp 1.0)
    pid.update(0.1, speed=20.0, feedforward=0.95)
  assert pid.p == pytest.approx(0.1 * (1 - (1 - 0.01 / 0.31) ** 30), rel=1e-6)
  assert 0.06 < pid.p < 0.065                # ~63% of the step
  for _ in range(300):
    pid.update(0.1, speed=20.0, feedforward=0.95)
  assert pid.p == pytest.approx(0.1, abs=1e-4)


def test_partial_blend_in_the_middle_of_the_band():
  pid = make(0.3)
  pid.update(0.0, speed=20.0, feedforward=0.825)       # low-pass starts at 0
  out = pid.update(0.2, speed=20.0, feedforward=0.825)
  lowpass = 0.2 * 0.01 / 0.31
  assert pid.p == pytest.approx(0.5 * 0.2 + 0.5 * lowpass)
  assert out == pytest.approx(pid.p + pid.i + 0.825)


def test_integrator_still_sees_the_raw_p():
  err, ff = random_drive(4)
  ff = ff * 0.0 + 0.95
  new, ref = make(0.5), make(0.0)
  for e, f in zip(err, ff):
    new.update(e, speed=20.0, feedforward=f)
    ref.update(e, speed=20.0, feedforward=f)
    assert new.i == ref.i


def test_reset_and_turning_it_off_clear_the_filter():
  pid = make(0.3)
  pid.update(0.5, speed=20.0, feedforward=0.95)
  pid.reset()
  assert pid._p_lowpass is None
  pid.update(0.5, speed=20.0, feedforward=0.95)
  pid.p_filter_tau = 0.0
  pid.update(0.5, speed=20.0, feedforward=0.95)
  assert pid._p_lowpass is None and pid.p == 0.5


class FakeParams:
  """Params.get as the device returns it: the cast value, None when unset or corrupt (never get_int's std::stoi)."""
  def __init__(self, value):
    self.value = value

  def get(self, key):
    assert key == "LatTorqueLimitPFilter"
    if isinstance(self.value, Exception):
      raise self.value
    return self.value

  def get_int(self, key):
    raise AssertionError("get_int aborts controlsd on a corrupt value; use get")


@pytest.mark.parametrize("value, tau", [(0, 0.0), (30, 0.3), (60, 0.6), (250, P_FILTER_MAX_CS * 0.01), (-5, 0.0), (None, 0.0),
                                        (10 ** 30, P_FILTER_MAX_CS * 0.01), (float("nan"), 0.0), ("garbage", 0.0)])
def test_setting_is_read_in_hundredths_of_a_second(value, tau):
  lac = SimpleNamespace(pid=make(), params=FakeParams(value))
  LatControlTorque.read_torque_limit_p_filter(lac)
  assert lac.pid.p_filter_tau == pytest.approx(tau)


def test_unknown_key_or_other_pid_leaves_steering_alone():
  lac = SimpleNamespace(pid=make(0.3), params=FakeParams(KeyError("unknown")))
  LatControlTorque.read_torque_limit_p_filter(lac)
  assert lac.pid.p_filter_tau == 0.0
  plain = SimpleNamespace(pid=PIDController(1.0, 0.7, pos_limit=1.0, neg_limit=-1.0), params=FakeParams(30))
  LatControlTorque.read_torque_limit_p_filter(plain)
  assert not hasattr(plain.pid, "p_filter_tau")


def test_a_non_finite_p_does_not_stick_in_the_filter():
  new, ref = make(0.3), make(0.0)
  for pid in (new, ref):
    pid.update(0.1, speed=20.0, feedforward=0.95, freeze_integrator=True)
    pid.update(float("nan"), speed=20.0, feedforward=0.95, freeze_integrator=True)
  assert new._p_lowpass is not None and np.isfinite(new._p_lowpass)
  for _ in range(5):
    out_new = new.update(0.1, speed=20.0, feedforward=0.5, freeze_integrator=True)
    out_ref = ref.update(0.1, speed=20.0, feedforward=0.5, freeze_integrator=True)
  assert np.isfinite(out_new) and out_new == pytest.approx(out_ref)


def test_an_already_bad_state_is_replaced_by_the_next_finite_p():
  pid = make(0.3)
  pid._p_lowpass = float("inf")
  pid.update(0.2, speed=20.0, feedforward=0.95)
  assert pid.p == pytest.approx(0.2)


def test_no_filter_while_the_driver_steers():
  pid = make(0.3)
  for _ in range(100):
    pid.update(0.0, speed=20.0, feedforward=0.95)
  pid.p_filter_bypass = True
  pid.update(0.3, speed=20.0, feedforward=0.95)
  assert pid.p == pytest.approx(0.3) and pid._p_lowpass is None
  pid.p_filter_bypass = False
  pid.update(0.3, speed=20.0, feedforward=0.95)     # starts fresh from the current P
  assert pid.p == pytest.approx(0.3)
