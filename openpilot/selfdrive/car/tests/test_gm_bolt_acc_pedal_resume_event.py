"""The Bolt ACC with pedal keeps the ECM's ACC standstill and leaves it by itself (mode 1: a pedal tap; mode 2, StarPilot:
releasing the pedal stop hold), so it shows no 'Press Resume' in either BoltPedalLongMode. b1e09a9a showed it in the
StarPilot mode; that misleading warning is left out on purpose. Every other GM car (and this car without the
auto-resuming pedal) keeps the event."""
import pytest

from openpilot.cereal import car, log
from openpilot.selfdrive.car.car_specific import CarSpecificEvents

EventName = log.OnroadEvent.EventName


def event_names(fingerprint, auto_resume, standstill, flags=0):
  CP = car.CarParams.new_message(brand="gm", carFingerprint=fingerprint, networkLocation="fwdCamera", pcmCruise=False,
                                 autoResumeSng=auto_resume, flags=flags)
  events = CarSpecificEvents(CP)
  CS = car.CarState.new_message(vEgo=0., standstill=True, gearShifter="drive")
  CS.cruiseState.available = True
  CS.cruiseState.enabled = True
  CS.cruiseState.standstill = standstill
  return events.update(CS, car.CarState.new_message(), car.CarControl.new_message()).names


@pytest.mark.parametrize("flags", [0, 1])  # mode 1, and mode 2 (GMFlags.PEDAL_LONG)
def test_bolt_acc_pedal_needs_no_resume(flags):
  assert EventName.resumeRequired not in event_names("CHEVROLET_BOLT_ACC_PEDAL", True, True, flags)


@pytest.mark.parametrize("fingerprint,auto_resume", [("CHEVROLET_BOLT_ACC_PEDAL", False), ("CHEVROLET_BOLT_EUV", True),
                                                     ("CHEVROLET_BOLT_EUV", False), ("CHEVROLET_VOLT", True),
                                                     ("CHEVROLET_TRAVERSE", False)])
def test_other_cases_keep_resume_required(fingerprint, auto_resume):
  assert EventName.resumeRequired in event_names(fingerprint, auto_resume, True)
  assert EventName.resumeRequired not in event_names(fingerprint, auto_resume, False)


def engage_blocked(min_enable, v, standstill, brake):
  CP = car.CarParams.new_message(brand="gm", carFingerprint="CHEVROLET_BOLT_ACC_PEDAL", networkLocation="fwdCamera",
                                 pcmCruise=False, autoResumeSng=True, minEnableSpeed=min_enable)
  CS = car.CarState.new_message(vEgo=v, standstill=standstill, brake=brake, gearShifter="drive")
  CS.cruiseState.available = True
  return EventName.belowEngageSpeed in CarSpecificEvents(CP).update(CS, car.CarState.new_message(),
                                                                     car.CarControl.new_message()).names


@pytest.mark.parametrize("v,standstill,brake,blocked", [(0.0, True, 0, True),     # rlog 00000021--282be5cb94--1: faulted
                                                        (0.08, True, 0, True), (0.0, True, 15, True),
                                                        (0.0, True, 20, False),   # stopped with the brake: allowed
                                                        (0.8, False, 0, True), (1.5, False, 0, False)])
def test_mode_1_engage_follows_the_ecm_rule(v, standstill, brake, blocked):
  """Mode 1 (minEnableSpeed 5 km/h): engaging needs > 5 km/h or a stop with brake >= 20, as the Bolt's ECM requires."""
  assert engage_blocked(5 / 3.6, v, standstill, brake) == blocked
  assert not engage_blocked(-1, v, standstill, brake)  # mode 2 / other pedal cars (-1) are not limited
