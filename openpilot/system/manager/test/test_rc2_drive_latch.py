from types import SimpleNamespace

from openpilot.system.manager import process_config as pc

CAR = SimpleNamespace(notCar=False)


class FakeParams:
  def __init__(self, **values):
    self.values = values

  def get_bool(self, key):
    return bool(self.values.get(key, False))


def fresh_latch(monkeypatch):
  latch = pc._DriveLatch("Rc2RemoteControl")
  monkeypatch.setattr(pc, "rc2_drive", latch)
  return latch


def procs(started, params):
  return {
    "controlsd": pc.not_joystick(started, params, CAR),
    "joystickd": pc.joystick(started, params, CAR),
    "rc2_joystick": pc.rc2_receiver(started, params, CAR),
    "joystick": pc.gamepad_joystick(started, params, CAR),
  }


def test_off_by_default(monkeypatch):
  fresh_latch(monkeypatch)
  params = FakeParams()
  assert procs(False, params) == {"controlsd": False, "joystickd": False, "rc2_joystick": False, "joystick": False}
  assert procs(True, params) == {"controlsd": True, "joystickd": False, "rc2_joystick": False, "joystick": False}


def test_switch_on_before_ignition_runs_the_drive_in_rc2_joystick_mode(monkeypatch):
  fresh_latch(monkeypatch)
  params = FakeParams(Rc2RemoteControl=True)
  # offroad: only the receiver, so the RC 2 screen can show the comma
  assert procs(False, params) == {"controlsd": False, "joystickd": False, "rc2_joystick": True, "joystick": False}
  assert procs(True, params) == {"controlsd": False, "joystickd": True, "rc2_joystick": True, "joystick": False}


def test_switch_flipped_mid_drive_waits_for_the_next_ignition(monkeypatch):
  fresh_latch(monkeypatch)
  params = FakeParams()
  assert procs(True, params)["controlsd"]
  params.values["Rc2RemoteControl"] = True
  on_mid_drive = procs(True, params)
  assert on_mid_drive["controlsd"] and not on_mid_drive["joystickd"]
  assert procs(False, params)["rc2_joystick"]
  assert procs(True, params)["joystickd"]  # next ignition

  params.values["Rc2RemoteControl"] = False
  off_mid_drive = procs(True, params)
  assert off_mid_drive["joystickd"] and off_mid_drive["rc2_joystick"] and not off_mid_drive["controlsd"]
  assert procs(False, params) == {"controlsd": False, "joystickd": False, "rc2_joystick": False, "joystick": False}
  assert procs(True, params)["controlsd"]


def test_gamepad_joystick_debug_mode_unchanged_and_never_two_publishers(monkeypatch):
  fresh_latch(monkeypatch)
  params = FakeParams(JoystickDebugMode=True)
  assert procs(True, params) == {"controlsd": False, "joystickd": True, "rc2_joystick": False, "joystick": True}
  params.values["Rc2RemoteControl"] = True
  both = procs(True, params)
  assert both["rc2_joystick"] and not both["joystick"]
