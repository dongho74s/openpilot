from types import SimpleNamespace

from openpilot.system.manager import process_config as pc

CAR = SimpleNamespace(notCar=False)


class FakeParams:
  def __init__(self, **values):
    self.values = values

  def get_bool(self, key):
    return bool(self.values.get(key, False))


class Clock:
  def __init__(self):
    self.t = 100.0

  def __call__(self):
    return self.t


def fresh_switch(monkeypatch):
  clock = Clock()
  switch = pc._SwitchWhenDisengaged("Rc2RemoteControl", clock=clock)
  monkeypatch.setattr(pc, "rc2_drive", switch)
  return switch, clock


def procs(started, params):
  return {
    "controlsd": pc.not_joystick(started, params, CAR),
    "joystickd": pc.joystick(started, params, CAR),
    "rc2_joystick": pc.rc2_receiver(started, params, CAR),
    "joystick": pc.gamepad_joystick(started, params, CAR),
  }


OPENPILOT = {"controlsd": True, "joystickd": False, "rc2_joystick": False, "joystick": False}
RC2 = {"controlsd": False, "joystickd": True, "rc2_joystick": True, "joystick": False}
OFF = {"controlsd": False, "joystickd": False, "rc2_joystick": False, "joystick": False}


def test_off_by_default(monkeypatch):
  fresh_switch(monkeypatch)
  params = FakeParams()
  assert procs(False, params) == OFF
  assert procs(True, params) == OPENPILOT


def test_switch_on_before_ignition_starts_the_drive_in_rc2_mode(monkeypatch):
  switch, _ = fresh_switch(monkeypatch)
  params = FakeParams(Rc2RemoteControl=True)
  switch.note_engaged(False)
  # offroad: only the receiver, so the RC 2 screen can show the comma
  assert procs(False, params) == {**OFF, "rc2_joystick": True}
  switch.note_engaged(True)  # selfdrived not up yet right after ignition counts as engaged
  assert procs(True, params) == RC2


def test_switch_flipped_while_disengaged_applies_after_a_second(monkeypatch):
  switch, clock = fresh_switch(monkeypatch)
  params = FakeParams()
  switch.note_engaged(False)
  assert procs(True, params) == OPENPILOT
  params.values["Rc2RemoteControl"] = True
  clock.t += 0.5
  assert procs(True, params)["controlsd"]  # not settled yet
  clock.t += 0.6
  assert procs(True, params) == RC2
  params.values["Rc2RemoteControl"] = False
  assert procs(True, params) == OPENPILOT  # still disengaged, so back at once


def test_switch_flipped_while_engaged_waits_for_disengage(monkeypatch):
  switch, clock = fresh_switch(monkeypatch)
  params = FakeParams()
  switch.note_engaged(True)
  assert procs(True, params) == OPENPILOT
  params.values["Rc2RemoteControl"] = True
  clock.t += 30
  assert procs(True, params) == {**OPENPILOT, "rc2_joystick": True}  # receiver up, controls untouched
  switch.note_engaged(False)
  clock.t += 0.9
  assert procs(True, params)["controlsd"]
  clock.t += 0.2
  assert procs(True, params) == RC2

  # and the other way: switched off while driving the RC 2 engaged keeps joystickd until disengaged
  switch.note_engaged(True)
  params.values["Rc2RemoteControl"] = False
  clock.t += 30
  assert procs(True, params) == RC2
  switch.note_engaged(False)
  clock.t += 1.1
  assert procs(True, params) == OPENPILOT


def test_a_brief_disengage_does_not_switch(monkeypatch):
  switch, clock = fresh_switch(monkeypatch)
  params = FakeParams(Rc2RemoteControl=True)
  switch.note_engaged(True)
  procs(True, params)
  switch.note_engaged(False)
  clock.t += 0.5
  switch.note_engaged(True)  # re-engaged within the settle time
  clock.t += 5
  assert procs(True, params)["controlsd"]


def test_without_engagement_reports_a_mid_drive_change_waits_for_ignition(monkeypatch):
  _, clock = fresh_switch(monkeypatch)
  params = FakeParams()
  assert procs(True, params) == OPENPILOT
  params.values["Rc2RemoteControl"] = True
  clock.t += 60
  assert procs(True, params)["controlsd"]
  assert procs(False, params)["rc2_joystick"]
  assert procs(True, params) == RC2


def test_gamepad_joystick_debug_mode_unchanged_and_never_two_publishers(monkeypatch):
  fresh_switch(monkeypatch)
  params = FakeParams(JoystickDebugMode=True)
  assert procs(True, params) == {"controlsd": False, "joystickd": True, "rc2_joystick": False, "joystick": True}
  params.values["Rc2RemoteControl"] = True
  both = procs(True, params)
  assert both["rc2_joystick"] and not both["joystick"]
