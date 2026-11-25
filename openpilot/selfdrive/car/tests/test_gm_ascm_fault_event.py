"""Malibu ASCM harness on openpilot long: when the ASCM gives up its path data (opendbc gm/ascm_fault.py) the driver is
told for 10 s that the stock FCW and AEB are off, engaged or not, with one prompt sound and no disengagement, without
hiding a more important alert or another permanent one."""
from openpilot.cereal import car, log
from openpilot.selfdrive.car.car_specific import ASCM_FAULT_SHOWN_FRAMES, ASCM_FAULT_START_FRAMES, CarSpecificEvents
from openpilot.selfdrive.selfdrived.alertmanager import AlertManager
from openpilot.selfdrive.selfdrived.events import ET, EVENTS, Events
from openpilot.selfdrive.selfdrived.state import StateMachine

EventName = log.OnroadEvent.EventName
AudibleAlert = car.CarControl.HUDControl.AudibleAlert
State = log.SelfdriveState.OpenpilotState
FAULT_EVENTS = {EventName.ascmFault, EventName.ascmFaultStart}


def car_params(brand="gm"):
  return car.CarParams.new_message(brand=brand, carFingerprint="CHEVROLET_MALIBU_ASCM", networkLocation="fwdCamera",
                                   pcmCruise=False, openpilotLongitudinalControl=True)


def car_state(invalid):
  CS = car.CarState.new_message(vEgo=7., gearShifter="drive", ascmPathDataInvalid=invalid)
  CS.cruiseState.available = True
  return CS


def run(states, brand="gm"):
  car_events = CarSpecificEvents(car_params(brand))
  prev = car.CarState.new_message()
  return [car_events.update(car_state(invalid), prev, car.CarControl.new_message()) for invalid in states]


def shown(events, engaged=False, extra=()):
  """Current alert of each frame through the state machine and AlertManager; extra[i] adds events to frame i."""
  sm = StateMachine()
  if engaged:
    sm.state = State.enabled
  am = AlertManager()
  out = []
  for frame, e in enumerate(events):
    merged = Events()
    for name in list(e.names) + list(extra[frame] if frame < len(extra) else ()):
      merged.add(name)
    sm.update(merged)
    am.add_many(frame, merged.create_alerts(sm.current_alert_types, [car_params(), car_state(True), None, None, 0, None]))
    am.process_alerts(frame, set())
    out.append(am.current_alert)
  assert sm.state == (State.enabled if engaged else State.disabled)
  return out


def test_shown_for_ten_seconds_with_the_start_event_for_the_first_second():
  events = run([False] * 10 + [True] * 2000 + [False] * 10)
  assert not any(FAULT_EVENTS & set(e.names) for e in events[:10] + events[2010:])
  fault = [i for i, e in enumerate(events) if EventName.ascmFault in e.names]
  start = [i for i, e in enumerate(events) if EventName.ascmFaultStart in e.names]
  assert fault == list(range(10, 10 + ASCM_FAULT_SHOWN_FRAMES)) and ASCM_FAULT_SHOWN_FRAMES == 1000
  assert start == list(range(10, 10 + ASCM_FAULT_START_FRAMES)) and ASCM_FAULT_START_FRAMES == 100


def test_set_again_after_it_cleared_shows_it_again():
  events = run([True] * 1500 + [False] + [True] * 5)
  assert not FAULT_EVENTS & set(events[1499].names)
  assert FAULT_EVENTS <= set(events[-1].names)


def test_other_brands_ignore_the_field():
  assert not any(FAULT_EVENTS & set(e.names) for e in run([True] * 50, brand="hyundai"))


def test_permanent_only_no_entry_or_disengagement():
  for name in FAULT_EVENTS:
    assert set(EVENTS[name]) == {ET.PERMANENT}


def test_one_prompt_then_silent_then_gone_engaged_or_not():
  events = run([True] * 1200)
  for engaged in (False, True):
    alerts = shown(events, engaged)
    assert all(a.alert_type.startswith("ascmFault") for a in alerts[:ASCM_FAULT_SHOWN_FRAMES])
    sounds = [a.audible_alert for a in alerts]
    assert all(s == AudibleAlert.prompt for s in sounds[:ASCM_FAULT_START_FRAMES])
    assert all(s == AudibleAlert.none for s in sounds[ASCM_FAULT_START_FRAMES + 1:])
    assert len({(a.alert_text_1, a.alert_text_2) for a in alerts[:ASCM_FAULT_SHOWN_FRAMES]}) == 1  # only the sound stops
    # 10 s later nothing is left up (the permanent alert's 0.2 s minimum aside)
    assert not any(a.alert_type for a in alerts[ASCM_FAULT_SHOWN_FRAMES + 30:])


def test_a_low_alert_already_up_stays_on_top():
  # e.g. lane departure (LOW) already up when the fault starts: an alert of the same priority that starts later would
  # take its place, so the fault's start (LOWER) must not
  events = run([False] * 50 + [True] * 300)
  alerts = shown(events, extra=[[EventName.ldw]] * 350)
  assert all(a.alert_type.startswith("ldw") for a in alerts)


def test_another_permanent_alert_comes_back_after_the_first_second():
  # e.g. the GM EPS retry hold (LOWER, permanent): hidden at most for the start, never for the rest
  events = run([False] * 50 + [True] * 400)
  alerts = shown(events, extra=[[EventName.steerRetryHold]] * 450)
  start_end = 50 + ASCM_FAULT_START_FRAMES
  assert all(a.alert_type.startswith("ascmFaultStart") for a in alerts[51:start_end])
  assert all(a.alert_type.startswith("steerRetryHold") for a in alerts[start_end + 30:])
