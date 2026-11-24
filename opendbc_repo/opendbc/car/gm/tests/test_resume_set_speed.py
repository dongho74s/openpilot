"""Traverse with openpilot long: RES engages only once the ECM holds a set speed (the ECM faulted its cruise when RES
engaged without one, rlog 0000024c 11:25:20/11:25:29 and 00000252 12:21:33); SET always engages."""
from types import SimpleNamespace

from opendbc.car import DT_CTRL, structs
from opendbc.car.gm.carstate import CarState
from opendbc.car.gm.values import AccState

ButtonType = structs.CarState.ButtonEvent.Type
ONE_SECOND = int(1.0 / DT_CTRL)


def traverse_state(needs_set_speed=True, pcm_cruise=False):
  cs = object.__new__(CarState)
  cs.CP = SimpleNamespace(pcmCruise=pcm_cruise)
  cs.resume_needs_set_speed = needs_set_speed
  cs.ecm_resume_ready = False
  cs.ecm_active_frames = 0
  return cs


def press(button_type, pressed):
  return [structs.CarState.ButtonEvent(type=button_type, pressed=pressed)]


def run(cs, frames, main_on=True, acc_state=AccState.ACTIVE):
  for _ in range(frames):
    cs.update_ecm_resume_ready(main_on, acc_state)


def test_res_does_not_engage_before_the_ecm_holds_a_set_speed():
  cs = traverse_state()
  assert not cs.update_button_enable(press(ButtonType.accelCruise, True))   # first engagement of a drive
  assert cs.update_button_enable(press(ButtonType.decelCruise, False))      # SET still engages on release


def test_res_engages_after_the_ecm_kept_its_cruise_for_a_second():
  cs = traverse_state()
  run(cs, ONE_SECOND - 1)
  assert not cs.update_button_enable(press(ButtonType.accelCruise, True))
  run(cs, 1)
  assert cs.update_button_enable(press(ButtonType.accelCruise, True))
  run(cs, 500, acc_state=AccState.OFF)        # disengaged, MAIN still on: the ECM keeps the speed
  run(cs, 300, acc_state=AccState.STANDBY)
  assert cs.update_button_enable(press(ButtonType.accelCruise, True))
  assert not cs.update_button_enable(press(ButtonType.accelCruise, False))  # RES only on press, as before


def test_a_short_active_spell_before_a_fault_does_not_count():
  cs = traverse_state()
  run(cs, int(0.5 / DT_CTRL))                  # the faulted engagements were active for 0.5 s
  run(cs, 8, acc_state=AccState.FAULTED)
  run(cs, ONE_SECOND - 1)
  assert not cs.update_button_enable(press(ButtonType.accelCruise, True))


def test_main_off_and_a_fault_drop_the_set_speed():
  cs = traverse_state()
  run(cs, ONE_SECOND)
  assert cs.ecm_resume_ready
  run(cs, 1, main_on=False, acc_state=AccState.OFF)
  assert not cs.update_button_enable(press(ButtonType.accelCruise, True))
  run(cs, ONE_SECOND)
  run(cs, 1, acc_state=AccState.FAULTED)
  assert not cs.update_button_enable(press(ButtonType.accelCruise, True))


def test_other_cars_and_stock_long_keep_res_engagement():
  assert traverse_state(needs_set_speed=False).update_button_enable(press(ButtonType.accelCruise, True))
  assert not traverse_state(needs_set_speed=False, pcm_cruise=True).update_button_enable(press(ButtonType.accelCruise, True))
