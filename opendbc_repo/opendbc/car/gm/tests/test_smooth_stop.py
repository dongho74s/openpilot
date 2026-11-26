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


def test_ascm_and_gateway_long_cars_keep_their_stops(runtime, monkeypatch):
  """Codex review 2026-11-26: only camera ACC / SDGM engine cars (the Malibu / Acadia gateway long and the Malibu ASCM
  with a SASCM are not in scope)."""
  from opendbc.car.gm.tests.test_malibu_ascm import malibu_cp
  for cp in (build_cp(runtime, CAR.CHEVROLET_MALIBU, True), build_cp(runtime, CAR.GMC_ACADIA, True),
             malibu_cp(runtime, True)):
    if cp.openpilotLongitudinalControl:
      assert not controller(runtime, monkeypatch, cp).smooth_stop, cp.carFingerprint


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


def test_a_stop_entered_at_speed_brakes_at_once(runtime, monkeypatch):
  """Codex review 2026-11-26: stopping entered at 8 m/s right after a -2.0 pid request must not ramp (was 0.5 s late)."""
  d = Drive(runtime, monkeypatch, build_cp(runtime, CAR.CHEVROLET_TRAVERSE, True))
  d.run(0.0, 1.0, v=8.0)
  assert brakes(d.run(-2.0, 0.2, state="stopping", v=8.0))[0] == 200


def test_a_strong_stopping_brake_at_low_speed_goes_on_at_once(runtime, monkeypatch):
  d = Drive(runtime, monkeypatch, build_cp(runtime, CAR.CHEVROLET_TRAVERSE, True))
  d.run(0.0, 1.0, v=1.0)
  assert brakes(d.run(-1.5, 0.2, state="stopping", v=1.0))[0] == 150  # above STOP_SMOOTH_MAX_BRAKE


def test_a_standstill_hold_after_a_release_comes_back_at_once(runtime, monkeypatch):
  """Codex review 2026-11-26: after an AutoResume creep release (0 sent) or entering soft hold, the standstill hold must
  be back in the next command, not ramp up from 0."""
  d = Drive(runtime, monkeypatch, build_cp(runtime, CAR.CHEVROLET_TRAVERSE, True))
  d.run(-0.5, 0.5, state="stopping", v=0.0)
  d.c.last_sent_brake = 0  # as after a creep release
  assert brakes(d.run(-0.5, 0.2, state="stopping", v=0.0))[0] == 50
  soft = Drive(runtime, monkeypatch, build_cp(runtime, CAR.CHEVROLET_TRAVERSE, True))
  soft.run(0.2, 0.5, v=0.0)  # standing on no brake
  assert brakes(soft.run(-0.5, 0.2, v=0.0, soft_hold=1))[0] >= 40


def test_rolling_back_brakes_at_once(runtime, monkeypatch):
  """Codex review 2026-11-26: rolling back in drive (negative vEgo) passed the speed check and ramped 12 -> 50."""
  d = Drive(runtime, monkeypatch, build_cp(runtime, CAR.CHEVROLET_TRAVERSE, True))
  d.run(0.2, 0.5, v=0.5)
  assert brakes(d.run(-0.5, 0.2, state="stopping", v=-2.1))[0] == 50
  assert brakes(d.run(-0.5, 0.2, state="stopping", v=-0.3))[0] == 50


# ---------------------------------------------------------------------------------------------------------------------
# Easing the brake off before the stop (StopBrakeEase / StopBrakeEaseSpeed, user request 2026-11-26)

EASE = dict(StopBrakeEase=30, StopBrakeEaseSpeed=4)


def into_a_stop(d, accel=-0.5):
  d.run(accel, 1.0, v=1.5)
  d.run(accel, 0.4, state="stopping", v=1.5)  # above the 4 km/h start speed: not eased


def eased_to_the_stop(d):
  seq = []
  for v in (1.0, 0.8, 0.6, 0.4, 0.2, 0.05):
    seq += brakes(d.run(-0.5, 0.2, state="stopping", v=v, a=-0.3))
  return seq


def test_the_brake_eases_off_as_the_car_slows_and_stays_eased_until_the_car_has_really_stopped(runtime, monkeypatch):
  """User report 2026-11-26 "a thump at the very end": the wheel speed reads 0 below about 0.75 km/h and the car rolls
  0.4-0.5 s more; the hold now comes after STOP_SETTLE_COMMANDS, rising by STOP_BRAKE_RISE, instead of at standstill."""
  from opendbc.car.gm.carcontroller import STOP_SETTLE_COMMANDS, STOP_BRAKE_RISE
  d = Drive(runtime, monkeypatch, build_cp(runtime, CAR.CHEVROLET_TRAVERSE, True), **EASE)
  into_a_stop(d)
  seq = eased_to_the_stop(d)
  assert seq == sorted(seq, reverse=True) and min(seq) >= 30
  assert seq[0] < 50 and seq[-1] <= 38  # 50 * (1 - 0.3 * closeness), near the stop about 36
  stopped = brakes(d.run(-0.5, 1.2, state="stopping", v=0.0))
  assert set(stopped[:STOP_SETTLE_COMMANDS]) == {seq[-1]}  # kept for 0.48 s after standstill
  rise = stopped[STOP_SETTLE_COMMANDS:stopped.index(50)]
  assert rise and all(0 < b2 - b1 <= STOP_BRAKE_RISE for b1, b2 in zip([seq[-1]] + rise, rise + [50]))
  assert set(stopped[stopped.index(50):]) == {50} and stopped.index(50) * 0.04 < 1.0  # before an AutoResume creep release


@pytest.mark.parametrize("setting,commands", [(None, 12), (48, 12), (0, 0), (20, 5), (50, 13), (100, 25), (150, 25),
                                              (-5, 0), ("x", 12)])
def test_the_settle_time_is_a_setting(runtime, monkeypatch, setting, commands):
  """StopBrakeSettleTime (user request 2026-11-26): the eased brake is kept 0-1 s (x0.01 s, one command per 4) after
  standstill, then rises as before; out of range is clipped, unreadable keeps 0.48 s, 0 starts the rise at once."""
  from opendbc.car.gm.carcontroller import STOP_BRAKE_RISE
  params = dict(EASE) if setting is None else dict(EASE, StopBrakeSettleTime=setting)
  d = Drive(runtime, monkeypatch, build_cp(runtime, CAR.CHEVROLET_TRAVERSE, True), **params)
  into_a_stop(d)
  eased = eased_to_the_stop(d)[-1]
  assert d.c.stop_settle_commands == commands
  stopped = brakes(d.run(-0.5, 2.0, state="stopping", v=0.0))
  assert stopped[:commands] == [eased] * commands
  assert stopped[commands] == eased + STOP_BRAKE_RISE
  rise = stopped[commands:stopped.index(50)]
  assert all(0 < b2 - b1 <= STOP_BRAKE_RISE for b1, b2 in zip(rise, rise[1:] + [50]))
  assert set(stopped[stopped.index(50):]) == {50}
  assert stopped.index(50) * 0.04 < 1.4  # at 1 s the hold is there about 1.2 s after standstill


def test_a_creep_release_called_off_during_the_settle_gets_the_hold_back_at_once(runtime, monkeypatch):
  """Codex review 2026-11-26 P1: with StopBrakeSettleTime 1 s, AutoResume's creep release (from 1 s of standstill) sets
  the brake to 0; when it is called off the next command must send the hold, not the eased brake rising from 36."""
  d = Drive(runtime, monkeypatch, build_cp(runtime, CAR.CHEVROLET_TRAVERSE, True), **EASE, StopBrakeSettleTime=100)
  into_a_stop(d)
  eased = eased_to_the_stop(d)[-1]
  assert set(brakes(d.run(-0.5, 0.6, state="stopping", v=0.0))) == {eased}  # settling
  real = d.c.auto_resume.update
  releases = iter([True, False])

  def creep_release_once(CC, CS, frame, gas, brake, *args, **kwargs):
    out = real(CC, CS, frame, gas, brake, *args, **kwargs)
    return (out[0], 0, *out[2:]) if next(releases, False) else out

  monkeypatch.setattr(d.c.auto_resume, "update", creep_release_once)
  assert brakes(d.run(-0.5, 0.12, state="stopping", v=0.0)) == [0, 50, 50]  # released once, then the hold at once


def test_wheels_turning_again_get_the_hold_at_once_and_a_stop_settles_once(runtime, monkeypatch):
  d = Drive(runtime, monkeypatch, build_cp(runtime, CAR.CHEVROLET_TRAVERSE, True), **EASE)
  into_a_stop(d)
  eased_to_the_stop(d)
  assert max(brakes(d.run(-0.5, 0.2, state="stopping", v=0.0))) < 50  # settling
  assert brakes(d.run(-0.5, 0.08, state="stopping", v=0.2, a=0.1)) == [50, 50]  # rolling again: the hold at once
  assert set(brakes(d.run(-0.5, 0.6, state="stopping", v=0.0))) == {50}  # standstill again: no second settle


def test_soft_hold_ends_the_settle_at_once(runtime, monkeypatch):
  d = Drive(runtime, monkeypatch, build_cp(runtime, CAR.CHEVROLET_TRAVERSE, True), **EASE)
  into_a_stop(d)
  eased_to_the_stop(d)
  d.run(-0.5, 0.12, state="stopping", v=0.0)
  assert min(brakes(d.run(-0.5, 0.2, state="stopping", v=0.0, soft_hold=1))) >= 50


@pytest.mark.parametrize("pitch", [0.05, -0.05])
def test_on_a_steep_road_the_hold_comes_at_standstill(runtime, monkeypatch, pitch):
  d = Drive(runtime, monkeypatch, build_cp(runtime, CAR.CHEVROLET_TRAVERSE, True), **EASE)
  into_a_stop(d)
  for v in (1.0, 0.6, 0.2):
    d.run(-0.5, 0.2, state="stopping", v=v, a=-0.3, pitch=pitch)
  assert set(brakes(d.run(-0.5, 0.3, state="stopping", v=0.0, pitch=pitch))) == {50}


def test_no_settle_without_easing_or_for_a_strong_hold(runtime, monkeypatch):
  off = Drive(runtime, monkeypatch, build_cp(runtime, CAR.CHEVROLET_TRAVERSE, True), StopBrakeEase=0)
  off.run(-0.2, 1.0, v=1.0)  # pid braking 20
  off.run(-0.5, 0.2, state="stopping", v=0.3)
  assert set(brakes(off.run(-0.5, 0.3, state="stopping", v=0.0))[1:]) == {50}
  strong = Drive(runtime, monkeypatch, build_cp(runtime, CAR.CHEVROLET_TRAVERSE, True), **EASE)
  into_a_stop(strong, accel=-1.0)
  strong.run(-1.0, 0.3, state="stopping", v=0.4, a=-0.3)
  assert set(brakes(strong.run(-1.0, 0.3, state="stopping", v=0.0))) == {100}  # above STOP_SMOOTH_MAX_BRAKE


def test_easing_ends_for_the_stop_when_the_car_stops_slowing(runtime, monkeypatch):
  d = Drive(runtime, monkeypatch, build_cp(runtime, CAR.CHEVROLET_TRAVERSE, True), **EASE)
  into_a_stop(d)
  assert brakes(d.run(-0.5, 0.2, state="stopping", v=0.4, a=-0.3))[-1] == 40
  rolling = brakes(d.run(-0.5, 0.6, state="stopping", v=0.4, a=0.1))  # a downhill: not slowing for 0.32 s
  assert rolling[-1] == 50 and rolling == sorted(rolling)  # back to the stopping brake, joined up
  assert set(brakes(d.run(-0.5, 0.2, state="stopping", v=0.3, a=-0.3))) == {50}  # until the car has stopped
  d.run(-0.5, 0.2, state="stopping", v=0.0)
  d.run(0.3, 0.5, v=1.5)
  into_a_stop(d)
  assert brakes(d.run(-0.5, 0.2, state="stopping", v=0.4, a=-0.3))[-1] == 40  # the next stop eases again


def test_no_easing_when_off_strong_or_on_an_ev(runtime, monkeypatch):
  off = Drive(runtime, monkeypatch, build_cp(runtime, CAR.CHEVROLET_TRAVERSE, True), StopBrakeEase=0)
  into_a_stop(off)
  assert set(brakes(off.run(-0.5, 0.3, state="stopping", v=0.4, a=-0.3))) == {50}
  strong = Drive(runtime, monkeypatch, build_cp(runtime, CAR.CHEVROLET_TRAVERSE, True), **EASE)
  into_a_stop(strong, accel=-1.0)
  assert set(brakes(strong.run(-1.0, 0.3, state="stopping", v=0.4, a=-0.3))) == {100}  # above STOP_SMOOTH_MAX_BRAKE
  ev = Drive(runtime, monkeypatch, build_cp(runtime, CAR.CHEVROLET_BOLT_EUV, True), **EASE)
  into_a_stop(ev)
  assert set(brakes(ev.run(-0.5, 0.3, state="stopping", v=0.4, a=-0.3))[1:]) == {50}



def test_a_creeping_or_noisy_car_ends_easing(runtime, monkeypatch):
  """Codex review R1: aEgo 0 (creeping) or accelerating with small negative noise must still end easing."""
  d = Drive(runtime, monkeypatch, build_cp(runtime, CAR.CHEVROLET_TRAVERSE, True), **EASE)
  into_a_stop(d)
  assert brakes(d.run(-0.5, 0.6, state="stopping", v=0.4, a=0.0))[-1] == 50
  noisy = Drive(runtime, monkeypatch, build_cp(runtime, CAR.CHEVROLET_TRAVERSE, True), **EASE)
  into_a_stop(noisy)
  out = []
  for _ in range(6):
    out += brakes(noisy.run(-0.5, 0.12, state="stopping", v=0.4, a=0.1))
    out += brakes(noisy.run(-0.5, 0.04, state="stopping", v=0.4, a=-0.1))
  assert out[-1] == 50


def test_a_stop_that_takes_too_long_ends_easing(runtime, monkeypatch):
  d = Drive(runtime, monkeypatch, build_cp(runtime, CAR.CHEVROLET_TRAVERSE, True), **EASE)
  into_a_stop(d)
  assert brakes(d.run(-0.5, 4.0, state="stopping", v=0.4, a=-0.3))[-1] == 40
  assert brakes(d.run(-0.5, 1.6, state="stopping", v=0.4, a=-0.3))[-1] == 50  # past 5 s of easing


def test_no_easing_again_after_standstill_in_the_same_stop(runtime, monkeypatch):
  """Codex review R2: the wheel speed going around the standstill threshold must not toggle the hold and easing."""
  d = Drive(runtime, monkeypatch, build_cp(runtime, CAR.CHEVROLET_TRAVERSE, True), **EASE)
  into_a_stop(d)
  d.run(-0.5, 0.2, state="stopping", v=0.2, a=-0.3)
  seq = []
  for v in (0.0, 0.09, 0.0, 0.09, 0.0):
    seq += brakes(d.run(-0.5, 0.2, state="stopping", v=v, a=-0.1))
  assert seq[-1] == 50 and min(seq[seq.index(50):]) == 50


def test_an_unreadable_setting_turns_easing_off_and_commands_go_on(runtime, monkeypatch):
  """Codex review R3: read through Params.get (None when unreadable), never get_int's C++ parse."""
  from opendbc.car.gm.carcontroller import read_int_param

  class Raising:
    def get(self, key):
      raise RuntimeError("bad")
  assert read_int_param(Raising(), "StopBrakeEase") is None
  assert read_int_param({"StopBrakeEase": None}, "StopBrakeEase") is None
  assert read_int_param({"StopBrakeEase": 35}, "StopBrakeEase") == 35
  d = Drive(runtime, monkeypatch, build_cp(runtime, CAR.CHEVROLET_TRAVERSE, True), StopBrakeEase="x", StopBrakeEaseSpeed=4)
  into_a_stop(d)
  assert set(brakes(d.run(-0.5, 0.3, state="stopping", v=0.4, a=-0.3))) == {50}


def test_a_stronger_hold_or_a_steep_road_ends_the_settle_at_once(runtime, monkeypatch):
  """Codex review 2026-11-26 R1: the hold limit and the pitch were only checked when the settle started."""
  d = Drive(runtime, monkeypatch, build_cp(runtime, CAR.CHEVROLET_TRAVERSE, True), **EASE)
  into_a_stop(d)
  eased_to_the_stop(d)
  assert max(brakes(d.run(-0.5, 0.12, state="stopping", v=0.0))) < 50  # settling
  assert brakes(d.run(-1.0, 0.04, state="stopping", v=0.0)) == [100]  # a stronger stop request: at once
  steep = Drive(runtime, monkeypatch, build_cp(runtime, CAR.CHEVROLET_TRAVERSE, True), **EASE)
  into_a_stop(steep)
  eased_to_the_stop(steep)
  assert max(brakes(steep.run(-0.5, 0.12, state="stopping", v=0.0))) < 50
  assert brakes(steep.run(-0.5, 0.04, state="stopping", v=0.0, pitch=0.031)) == [50]  # the road tilts: at once


def test_a_stop_whose_easing_was_given_up_gets_the_hold_at_standstill(runtime, monkeypatch):
  """Codex review 2026-11-26 R2: easing ended for a creep (aEgo 0), then standstill kept 41 for 0.48 s."""
  d = Drive(runtime, monkeypatch, build_cp(runtime, CAR.CHEVROLET_TRAVERSE, True), **EASE)
  into_a_stop(d)
  d.run(-0.5, 0.2, state="stopping", v=0.4, a=-0.3)
  given_up = brakes(d.run(-0.5, 0.32, state="stopping", v=0.2, a=0.0))
  assert given_up[-1] > given_up[0]  # easing ended, the brake is joining back up
  assert set(brakes(d.run(-0.5, 0.3, state="stopping", v=0.0))) == {50}
