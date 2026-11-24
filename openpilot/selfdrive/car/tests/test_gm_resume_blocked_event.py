"""Traverse with openpilot long: a RES press that does not engage (no set speed in the ECM yet) shows its no-entry alert
through the state machine and does not engage; GM shows the EPS retry hold. Other cases are unchanged."""
import pytest

from openpilot.cereal import car, log
from openpilot.selfdrive.car.car_specific import CarSpecificEvents
from openpilot.selfdrive.selfdrived.events import ET
from openpilot.selfdrive.selfdrived.state import StateMachine

EventName = log.OnroadEvent.EventName
ButtonType = car.CarState.ButtonEvent.Type


def car_params(fingerprint="CHEVROLET_TRAVERSE", pcm_cruise=False, brand="gm"):
  return car.CarParams.new_message(brand=brand, carFingerprint=fingerprint, networkLocation="fwdCamera", pcmCruise=pcm_cruise,
                                   openpilotLongitudinalControl=not pcm_cruise)


def car_state(button_enable=False, cruise_enabled=False, pressed=True, button=ButtonType.accelCruise, steer_fault=False):
  CS = car.CarState.new_message(vEgo=15., gearShifter="drive", buttonEnable=button_enable, steerFaultTemporary=steer_fault,
                                buttonEvents=[{"type": button, "pressed": pressed}] if button is not None else [])
  CS.cruiseState.available = True
  CS.cruiseState.enabled = cruise_enabled
  return CS


def events_for(fingerprint="CHEVROLET_TRAVERSE", pcm_cruise=False, **kwargs):
  return CarSpecificEvents(car_params(fingerprint, pcm_cruise)).update(car_state(**kwargs), car.CarState.new_message(),
                                                                       car.CarControl.new_message())


def test_blocked_res_shows_its_no_entry_alert_and_does_not_engage():
  events = events_for()
  assert EventName.resumeBlocked in events.names
  sm = StateMachine()
  enabled, active = sm.update(events)
  assert not enabled and not active
  assert ET.NO_ENTRY in sm.current_alert_types
  alerts = events.create_alerts([ET.NO_ENTRY], [car_params(), car_state(), None, None, 0, None])
  assert [a.alert_text_2 for a in alerts if a.alert_type.startswith("resumeBlocked")] or \
         any("resumeBlocked" in a.alert_type for a in alerts)


@pytest.mark.parametrize("kwargs", [
  dict(button_enable=True),                 # the ECM holds a set speed: RES engages
  dict(cruise_enabled=True),                # engaged: RES raises the speed
  dict(pressed=False),                      # release
  dict(button=ButtonType.decelCruise),      # SET
  dict(fingerprint="CHEVROLET_TRAILBLAZER"),
  dict(pcm_cruise=True),
])
def test_other_cases_keep_their_events(kwargs):
  assert EventName.resumeBlocked not in events_for(**kwargs).names


def test_gm_shows_the_eps_retry_hold():
  car_events = CarSpecificEvents(car_params())
  prev = car.CarState.new_message()
  def names(fault):
    return car_events.update(car_state(button=None, steer_fault=fault), prev, car.CarControl.new_message()).names
  assert EventName.steerTempUnavailableSilent not in names(False)
  names(True)                                          # the EPS refuses: the existing fault handling applies
  held = [EventName.steerTempUnavailableSilent in names(False) for _ in range(300)]
  assert all(held[:299]) and not held[299]


def test_other_brands_have_no_eps_retry_hold():
  car_events = CarSpecificEvents(car_params(fingerprint="HYUNDAI_IONIQ_5", brand="hyundai"))
  prev = car.CarState.new_message()
  car_events.update(car_state(button=None, steer_fault=True), prev, car.CarControl.new_message())
  assert EventName.steerTempUnavailableSilent not in car_events.update(car_state(button=None), prev, car.CarControl.new_message()).names
