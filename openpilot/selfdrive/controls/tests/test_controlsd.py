import ast
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from openpilot.cereal import car


def run_lateral_gate(*, brand="tesla", supported=True, speed=0.0, stopped=True, min_speed=0.0,
                     active=True, always_lateral=False, lat_enabled=True, temporary_fault=False,
                     permanent_fault=False, gear="drive"):
  # Execute the production state_control path through its lateral gating call,
  # without starting hardware, IPC, model inference, or actuator controllers.
  path = Path(__file__).parents[1] / "controlsd.py"
  tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
  tree.body = [node for node in tree.body if isinstance(node, (ast.FunctionDef, ast.ClassDef))]
  namespace = {
    "math": math, "car": car, "MIN_LATERAL_CONTROL_SPEED": 0.3,
    "resolve_vehicle_model_steer_ratio": lambda *_args: 15.0,
    "lateral_vehicle_parameters": lambda sm, _cp: sm["liveParameters"],
    "read_live_vehicle_dynamics": lambda *_args: None, "DT_CTRL": 0.01,
  }
  exec(compile(tree, str(path), "exec"), namespace)
  controls = namespace["Controls"].__new__(namespace["Controls"])
  controls.eps_fault_hold = namespace["EpsTemporaryFaultHold"](brand == "gm")
  controls.CP = SimpleNamespace(brand=brand, steerAtStandstill=supported, minSteerSpeed=min_speed,
                               lateralTuning=SimpleNamespace(which=lambda: "angle"))
  controls.params = SimpleNamespace(get_float=lambda _key: 0.0, get_bool=lambda key: always_lateral if key == "AlwaysLateral" else False)
  controls.is_vw_meb = False
  controls.lateral_startup = SimpleNamespace(ready=True, update=lambda *_args: True)
  controls.VM = SimpleNamespace(update_params=lambda *_args: None, calc_curvature=lambda *_args: 0.0)
  state = car.CarState.new_message(vEgo=speed, standstill=stopped, gearShifter=gear, latEnabled=lat_enabled,
                                   steerFaultTemporary=temporary_fault, steerFaultPermanent=permanent_fault)
  controls.sm = {
    "carState": state,
    "liveParameters": SimpleNamespace(stiffnessFactor=1.0, steerRatio=15.0, angleOffsetDeg=0.0, roll=0.0),
    "longitudinalPlan": SimpleNamespace(), "modelV2": SimpleNamespace(),
    "selfdriveState": SimpleNamespace(enabled=active, active=active),
  }

  class ReachedLateralGate(Exception):
    pass

  result = []

  def capture(_state, lat_active):
    result.append(lat_active)
    raise ReachedLateralGate

  controls.carrot_controls = SimpleNamespace(lat_suspend_control=capture)
  with pytest.raises(ReachedLateralGate):
    controls.state_control()
  return result[0]


@pytest.mark.parametrize("active,always_lateral", [(True, False), (False, True)])
def test_tesla_can_steer_at_true_standstill(active, always_lateral):
  assert run_lateral_gate(active=active, always_lateral=always_lateral)


@pytest.mark.parametrize('active,always_lateral,expected', [(False, False, False), (True, False, True), (False, True, True)])
def test_toyota_lateral_selection_requires_engagement_or_always_lateral(active, always_lateral, expected):
  assert run_lateral_gate(brand='toyota', speed=15.0, stopped=False, active=active, always_lateral=always_lateral) == expected


@pytest.mark.parametrize("brand", ["tesla", "ford", "volkswagen", "psa", "hyundai"])
@pytest.mark.parametrize("speed,min_speed", [(0.1, 0.0), (0.25, 0.0), (4.9, 5.0), (5.0, 5.0)])
def test_moving_cars_still_obey_minimum_lateral_speed(brand, speed, min_speed):
  assert not run_lateral_gate(brand=brand, speed=speed, min_speed=min_speed, stopped=False)


@pytest.mark.parametrize("brand", ["ford", "volkswagen", "psa", "hyundai"])
def test_standstill_exception_does_not_change_other_brands(brand):
  assert not run_lateral_gate(brand=brand)


@pytest.mark.parametrize("override", [
  {"supported": False}, {"active": False}, {"lat_enabled": False},
  {"temporary_fault": True}, {"permanent_fault": True},
  {"active": False, "always_lateral": True, "gear": "park"},
  {"active": False, "always_lateral": True, "gear": "reverse"},
  {"active": False, "always_lateral": True, "gear": "neutral"},
])
def test_standstill_preserves_engagement_fault_and_gear_gates(override):
  assert not run_lateral_gate(**override)


@pytest.mark.parametrize("brand", ["tesla", "ford", "volkswagen", "psa", "hyundai"])
def test_lateral_control_remains_available_above_minimum_speed(brand):
  assert run_lateral_gate(brand=brand, speed=5.1, min_speed=5.0, stopped=False)


def test_nothing_actuates_while_modeld_switches_the_internal_model():
  import ast as _ast
  src = (Path(__file__).resolve().parents[1] / "controlsd.py").read_text(encoding="utf8")
  tree = _ast.parse(src)
  fn = next(n for n in _ast.walk(tree) if isinstance(n, _ast.FunctionDef) and n.name == "state_control")
  body = _ast.unparse(fn)
  assert body.count('self.params.get_bool(\'InternalModelSwitching\')') == 1          # read once per frame
  block = "if model_switching:\n        CC.latActive = CC.longActive = False"
  assert block in body and body.index(block) > body.index("CC.longActive = CC.enabled")   # after both are decided
  assert body.index(block) < body.index("actuators = CC.actuators")


def test_no_stock_acc_resume_while_modeld_switches_the_internal_model():
  import ast as _ast
  src = (Path(__file__).resolve().parents[1] / "controlsd.py").read_text(encoding="utf8")
  tree = _ast.parse(src)
  state = _ast.unparse(next(n for n in _ast.walk(tree) if isinstance(n, _ast.FunctionDef) and n.name == "state_control"))
  assert "model_switching = self.model_switching = self.params.get_bool('InternalModelSwitching')" in state
  publish = _ast.unparse(next(n for n in _ast.walk(tree) if isinstance(n, _ast.FunctionDef) and n.name == "publish"))
  resume = next(line for line in publish.splitlines() if "CC.cruiseControl.resume =" in line)
  assert "not getattr(self, 'model_switching', False)" in resume


def eps_fault_hold_class():
  path = Path(__file__).parents[1] / "controlsd.py"
  tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
  tree.body = [node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "EpsTemporaryFaultHold"]
  namespace = {"DT_CTRL": 0.01}
  exec(compile(tree, str(path), "exec"), namespace)
  return namespace["EpsTemporaryFaultHold"]


class RefusingEps:
  """The Traverse EPS at the start of a drive (rlog 00000251, 00000252): it answers a request with status 2 (no torque)
  until it gets one after at least 3 s without requests; from then on it delivers."""
  def __init__(self):
    self.accepting = False
    self.quiet_frames = 0

  def respond(self, requested: bool) -> bool:
    """steerFaultTemporary for this frame's request"""
    if not requested:
      self.quiet_frames += 1
      return False
    if not self.accepting and self.quiet_frames >= 300:
      self.accepting = True
    self.quiet_frames = 0
    return not self.accepting


def drive(hold, frames=1500):
  eps, fault, toggles, last, accepted_at = RefusingEps(), False, 0, False, None
  for frame in range(frames):
    lat_active = not hold.update(fault)
    toggles += lat_active != last
    last = lat_active
    fault = eps.respond(lat_active)
    if eps.accepting and accepted_at is None:
      accepted_at = frame
  return toggles, accepted_at


def test_gm_waits_after_an_eps_refusal_instead_of_toggling():
  cls = eps_fault_hold_class()
  toggles, accepted_at = drive(cls(True))
  assert toggles <= 3 and accepted_at is not None and accepted_at < 400
  old_toggles, old_accepted = drive(cls(False))   # previous behaviour: re-request at once, every 0.1 s
  assert old_toggles > 100 and old_accepted is None


def test_hold_length_and_other_brands():
  cls = eps_fault_hold_class()
  hold = cls(True)
  assert hold.update(True)
  assert all(hold.update(False) for _ in range(299))
  assert not hold.update(False)
  other = cls(False)
  assert other.update(True) and not other.update(False)


def test_gm_temporary_fault_blocks_lateral():
  assert not run_lateral_gate(brand="gm", speed=10.0, stopped=False, temporary_fault=True)
  assert run_lateral_gate(brand="gm", speed=10.0, stopped=False)
