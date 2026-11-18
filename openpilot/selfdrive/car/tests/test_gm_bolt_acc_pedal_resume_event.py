"""The Bolt ACC with pedal keeps the ECM's ACC standstill and leaves it with a pedal tap, so it shows no 'Press Resume';
every other GM car (and this car without the auto-resuming pedal) keeps the event."""
import pytest

from openpilot.cereal import car, log
from openpilot.selfdrive.car.car_specific import CarSpecificEvents

EventName = log.OnroadEvent.EventName


def event_names(fingerprint, auto_resume, standstill):
  CP = car.CarParams.new_message(brand="gm", carFingerprint=fingerprint, networkLocation="fwdCamera", pcmCruise=False,
                                 autoResumeSng=auto_resume)
  events = CarSpecificEvents(CP)
  CS = car.CarState.new_message(vEgo=0., standstill=True, gearShifter="drive")
  CS.cruiseState.available = True
  CS.cruiseState.enabled = True
  CS.cruiseState.standstill = standstill
  return events.update(CS, car.CarState.new_message(), car.CarControl.new_message()).names


def test_bolt_acc_pedal_needs_no_resume():
  assert EventName.resumeRequired not in event_names("CHEVROLET_BOLT_ACC_PEDAL", True, True)


@pytest.mark.parametrize("fingerprint,auto_resume", [("CHEVROLET_BOLT_ACC_PEDAL", False), ("CHEVROLET_BOLT_EUV", True),
                                                     ("CHEVROLET_BOLT_EUV", False), ("CHEVROLET_VOLT", True),
                                                     ("CHEVROLET_TRAVERSE", False)])
def test_other_cases_keep_resume_required(fingerprint, auto_resume):
  assert EventName.resumeRequired in event_names(fingerprint, auto_resume, True)
  assert EventName.resumeRequired not in event_names(fingerprint, auto_resume, False)
