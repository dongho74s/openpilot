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
  cs.ecm_resume_ready = cs.ecm_acc_active = False
  cs.ecm_active_frames = cs.ecm_acc_msg_age = 0
  return cs


def press(button_type, pressed):
  return [structs.CarState.ButtonEvent(type=button_type, pressed=pressed)]


def run(cs, frames, main_on=1, acc_state=AccState.ACTIVE, acc_every=3, main_every=10):
  """100 Hz updates with AcceleratorPedal2 every 3rd (33 Hz) and ECMEngineStatus every 10th (10 Hz) as received values"""
  for i in range(frames):
    cs.update_ecm_resume_ready([main_on] if i % main_every == 0 else [], [acc_state] if i % acc_every == 0 else [])


def res_engages(cs):
  return cs.update_button_enable(press(ButtonType.accelCruise, True))


def test_res_does_not_engage_before_the_ecm_holds_a_set_speed():
  cs = traverse_state()
  assert not res_engages(cs)                                               # first engagement of a drive
  assert cs.update_button_enable(press(ButtonType.decelCruise, False))     # SET still engages on release


def test_res_engages_after_the_ecm_kept_its_cruise_for_a_second():
  cs = traverse_state()
  run(cs, ONE_SECOND - 1)
  assert not res_engages(cs)
  run(cs, 1)
  assert res_engages(cs)
  run(cs, 500, acc_state=AccState.OFF)        # disengaged, MAIN still on: the ECM keeps the speed
  run(cs, 300, acc_state=AccState.STANDBY)
  assert res_engages(cs)
  assert not cs.update_button_enable(press(ButtonType.accelCruise, False))  # RES only on press, as before


def test_a_short_active_spell_before_a_fault_does_not_count():
  cs = traverse_state()
  run(cs, int(0.5 / DT_CTRL))                  # the faulted engagements were active for 0.5 s
  run(cs, 8, acc_state=AccState.FAULTED)
  run(cs, ONE_SECOND - 1)
  assert not res_engages(cs)


def test_main_off_and_a_fault_drop_the_set_speed():
  cs = traverse_state()
  run(cs, ONE_SECOND)
  assert cs.ecm_resume_ready
  cs.update_ecm_resume_ready([0], [AccState.OFF])
  assert not res_engages(cs)
  run(cs, ONE_SECOND)
  cs.update_ecm_resume_ready([], [AccState.FAULTED])
  assert not res_engages(cs)


def test_a_fault_or_main_off_inside_one_batch_is_not_hidden_by_the_last_value():
  cs = traverse_state()
  run(cs, ONE_SECOND)
  cs.update_ecm_resume_ready([], [AccState.FAULTED, AccState.ACTIVE])
  assert not cs.ecm_resume_ready
  run(cs, ONE_SECOND)
  cs.update_ecm_resume_ready([0, 1], [AccState.ACTIVE])
  assert not cs.ecm_resume_ready


def test_a_message_that_stops_arriving_is_not_counted_again():
  cs = traverse_state()
  run(cs, 50)                                   # 0.5 s of ACTIVE, then AcceleratorPedal2 stops
  for _ in range(200):                          # nothing received: its last value is not counted on
    cs.update_ecm_resume_ready([], [])
    assert not cs.ecm_resume_ready
  assert cs.ecm_active_frames == 0
  run(cs, ONE_SECOND)
  assert cs.ecm_resume_ready
  for _ in range(51):                           # an earlier set speed is dropped too when the message is lost
    cs.update_ecm_resume_ready([], [])
  assert not res_engages(cs)


def test_a_pause_just_before_the_second_does_not_confirm_it():
  cs = traverse_state()
  run(cs, 96)                                   # 33 Hz ACTIVE for 0.96 s (last sample at 0.93 s)
  for _ in range(20):                           # then 0.2 s without AcceleratorPedal2
    cs.update_ecm_resume_ready([], [])
    assert not cs.ecm_resume_ready
  cs.update_ecm_resume_ready([], [AccState.OFF])
  assert not res_engages(cs)


def test_other_cars_and_stock_long_keep_res_engagement():
  assert traverse_state(needs_set_speed=False).update_button_enable(press(ButtonType.accelCruise, True))
  assert not traverse_state(needs_set_speed=False, pcm_cruise=True).update_button_enable(press(ButtonType.accelCruise, True))
  # test fixtures and other callers that pass a bare namespace keep the previous behaviour
  assert CarState.update_button_enable(SimpleNamespace(CP=SimpleNamespace(pcmCruise=False)), press(ButtonType.accelCruise, True))
