"""The end of a stop on the engine camera-ACC / SDGM Chevrolets (user report 2026-11-26, Traverse: "the brake goes on and
off as it stops"). Logged stops stepped the brake 41 -> 52 when LongControl entered stopping near 1 km/h and 52 -> 40
(CP.stopAccel's fixed hold) in the instant the car stopped. Real DBC, packer and CarController; no device I/O."""
import pytest

from opendbc.car.gm.values import CAR
from opendbc.car.gm.tests.test_display_speed import build_cp, runtime  # noqa: F401
from opendbc.car.gm.tests.test_bolt_acc_pedal import bolt_cp
from opendbc.car.gm.tests.test_gas_brake_hysteresis import Drive, controller

ENGINE_CARS = [CAR.CHEVROLET_TRAVERSE, CAR.CHEVROLET_TRAILBLAZER, CAR.CHEVROLET_EQUINOX, CAR.CHEVROLET_SILVERADO]


def brakes(pairs):
  return [b for _, b in pairs]


@pytest.mark.parametrize("car", ENGINE_CARS, ids=lambda c: c.name.lower())
def test_engine_cars_smooth_the_stop_and_evs_do_not(runtime, monkeypatch, car):
  assert controller(runtime, monkeypatch, build_cp(runtime, car, True)).smooth_stop
  assert not controller(runtime, monkeypatch, build_cp(runtime, CAR.CHEVROLET_BOLT_EUV, True)).smooth_stop
  assert not controller(runtime, monkeypatch, bolt_cp(runtime, CAR.CHEVROLET_BOLT_ACC_PEDAL, True, False)).smooth_stop


def test_the_brake_does_not_drop_in_the_instant_the_car_stops(runtime, monkeypatch):
  d = Drive(runtime, monkeypatch, build_cp(runtime, CAR.CHEVROLET_TRAVERSE, True))
  d.run(-0.41, 1.0, v=0.4)  # pid, braking 41
  entering = brakes(d.run(-0.52, 0.4, state="stopping", v=0.3))  # StoppingAccel -0.52 -> 52
  assert entering[-1] == 52 and entering == sorted(entering)
  assert max(b2 - b1 for b1, b2 in zip([41] + entering, entering)) <= 3  # was one 41 -> 52 step
  near = brakes(d.run(-0.52, 0.3, state="stopping", v=0.05))  # below NEAR_STOP_BRAKE_PHASE
  assert set(near) == {52}  # was CP.stopAccel's 40
  stopped = brakes(d.run(-0.52, 0.5, state="stopping", v=0.0))  # standstill: the full-stop hold
  assert set(stopped) == {52}


def test_a_weaker_stopping_accel_still_reaches_the_hold_without_a_step(runtime, monkeypatch):
  d = Drive(runtime, monkeypatch, build_cp(runtime, CAR.CHEVROLET_TRAVERSE, True))
  d.run(-0.3, 1.0, v=0.4)
  d.run(-0.3, 0.4, state="stopping", v=0.3)
  near = brakes(d.run(-0.3, 0.6, state="stopping", v=0.05))  # CP.stopAccel's 40 is the floor of the hold
  assert near[-1] == 40 and near == sorted(near)
  assert max(b2 - b1 for b1, b2 in zip([30] + near, near)) <= 3


def test_stopping_brake_rises_quickly_from_nothing(runtime, monkeypatch):
  d = Drive(runtime, monkeypatch, build_cp(runtime, CAR.CHEVROLET_TRAVERSE, True))
  d.run(0.2, 1.0, v=0.6)  # creeping on gas, no brake
  rising = brakes(d.run(-0.5, 0.5, state="stopping", v=0.5))
  assert rising[0] <= 13 and rising[-1] == 50  # a quarter of the gap per command: 50 within half a second
  assert all(b1 <= b2 for b1, b2 in zip(rising, rising[1:]))


def test_pid_braking_is_not_rate_limited(runtime, monkeypatch):
  d = Drive(runtime, monkeypatch, build_cp(runtime, CAR.CHEVROLET_TRAVERSE, True))
  d.run(0.0, 1.0, v=5.0)
  assert set(d.run(-1.2, 0.3, v=5.0)[1:]) == {(-540, 120)}  # urgent braking at once, as before


def test_the_bolts_keep_their_stop_hold(runtime, monkeypatch):
  cp = build_cp(runtime, CAR.CHEVROLET_BOLT_EUV, True)
  d = Drive(runtime, monkeypatch, cp)
  d.run(-0.41, 1.0, v=0.4)
  hold = int(-100 * cp.stopAccel)
  assert set(d.run(-0.52, 0.3, state="stopping", v=0.05)[1:]) == {(-500, hold)}  # CP.stopAccel's hold, unchanged
