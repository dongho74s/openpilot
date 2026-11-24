"""GM: while the EPS retry hold is on (3 s after a refused lateral request), the hold is shown; other brands show nothing.

Adapted from WIP-FR e23f29cf4: only the EPS-hold tests are ported. The Traverse RES-gate tests are not — the RES gate
(opendbc gm CarState.resume_allowed) is Traverse-only and does not exist on carrot-nnff.
"""
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


def test_gm_shows_the_eps_retry_hold_engaged_or_not():
  car_events = CarSpecificEvents(car_params())
  prev = car.CarState.new_message()
  def update(fault):
    return car_events.update(car_state(button=None, steer_fault=fault), prev, car.CarControl.new_message())
  assert EventName.steerRetryHold not in update(False).names
  update(True)                                         # the EPS refuses: the existing fault handling applies
  held = [update(False) for _ in range(300)]
  assert all(EventName.steerRetryHold in e.names for e in held[:299]) and EventName.steerRetryHold not in held[299].names
  # permanent: selected with nothing engaged (AlwaysLateral) as well as engaged, silently
  sm = StateMachine()
  enabled, _ = sm.update(held[0])
  assert not enabled and ET.PERMANENT in sm.current_alert_types
  alerts = held[0].create_alerts(sm.current_alert_types, [car_params(), car_state(button=None), None, None, 0, None])
  hold = [a for a in alerts if a.alert_type.startswith("steerRetryHold")]
  assert len(hold) == 1 and hold[0].audible_alert == car.CarControl.HUDControl.AudibleAlert.none


def test_other_brands_have_no_eps_retry_hold():
  car_events = CarSpecificEvents(car_params(fingerprint="HYUNDAI_IONIQ_5", brand="hyundai"))
  prev = car.CarState.new_message()
  car_events.update(car_state(button=None, steer_fault=True), prev, car.CarControl.new_message())
  assert EventName.steerRetryHold not in car_events.update(car_state(button=None), prev, car.CarControl.new_message()).names
