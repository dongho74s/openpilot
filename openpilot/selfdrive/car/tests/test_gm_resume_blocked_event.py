"""Traverse with openpilot long: a RES press that does not engage (no set speed in the ECM yet) shows "Press Set to
Engage"; other GM cars, stock long and engaging presses are unchanged."""
import pytest

from openpilot.cereal import car, log
from openpilot.selfdrive.car.car_specific import CarSpecificEvents

EventName = log.OnroadEvent.EventName
ButtonType = car.CarState.ButtonEvent.Type


def event_names(fingerprint="CHEVROLET_TRAVERSE", pcm_cruise=False, button_enable=False, cruise_enabled=False, pressed=True,
                button=ButtonType.accelCruise):
  CP = car.CarParams.new_message(brand="gm", carFingerprint=fingerprint, networkLocation="fwdCamera", pcmCruise=pcm_cruise,
                                 openpilotLongitudinalControl=not pcm_cruise)
  CS = car.CarState.new_message(vEgo=15., gearShifter="drive", buttonEnable=button_enable,
                                buttonEvents=[{"type": button, "pressed": pressed}])
  CS.cruiseState.available = True
  CS.cruiseState.enabled = cruise_enabled
  return CarSpecificEvents(CP).update(CS, car.CarState.new_message(), car.CarControl.new_message()).names


def test_blocked_res_says_set_engages():
  assert EventName.resumeBlocked in event_names()


@pytest.mark.parametrize("kwargs", [
  dict(button_enable=True),                 # the ECM holds a set speed: RES engages
  dict(cruise_enabled=True),                # engaged: RES raises the speed
  dict(pressed=False),                      # release
  dict(button=ButtonType.decelCruise),      # SET
  dict(fingerprint="CHEVROLET_TRAILBLAZER"),
  dict(pcm_cruise=True),
])
def test_other_cases_keep_their_events(kwargs):
  assert EventName.resumeBlocked not in event_names(**kwargs)
