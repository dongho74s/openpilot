"""card applies no lateral/longitudinal command while modeld switches the internal driving model (modeld.LiveSwitch)."""
import ast
from pathlib import Path
from types import SimpleNamespace

from openpilot.cereal import car

CARD = Path(__file__).resolve().parents[1] / "card.py"


def guard():
  tree = ast.parse(CARD.read_text(encoding="utf8"))
  fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "controls_update")
  node = next(n for n in ast.walk(fn) if isinstance(n, ast.If) and "InternalModelSwitching" in ast.unparse(n.test))
  return fn, node, compile(ast.Module(body=[node], type_ignores=[]), "<guard>", "exec")


def active_cc():
  cc = car.CarControl.new_message()
  cc.enabled = cc.latActive = cc.longActive = True
  cc.actuators.torque = 0.8
  cc.actuators.accel = 1.2
  cc.actuators.curvature = 0.01
  cc.actuators.steeringAngleDeg = 30.0
  cc.cruiseControl.resume = True
  cc.cruiseControl.cancel = True
  return cc.as_reader()


def run(switching):
  _, _, code = guard()
  cs = car.CarState.new_message()
  cs.steeringAngleDeg = -4.0
  scope = {"CC": active_cc(), "CS": cs.as_reader(), "car": car,
           "self": SimpleNamespace(params=SimpleNamespace(get_bool=lambda key: switching and key == "InternalModelSwitching"))}
  exec(code, scope)
  return scope["CC"]


def test_a_command_from_before_the_switch_is_neutralised_at_the_final_boundary():
  cc = run(True)
  assert not cc.latActive and not cc.longActive and not cc.cruiseControl.resume
  assert cc.actuators.torque == 0 and cc.actuators.accel == 0 and cc.actuators.curvature == 0
  assert cc.actuators.steeringAngleDeg == -4.0                      # angle cars hold the measured angle
  assert cc.enabled and cc.cruiseControl.cancel                     # left to selfdrived/cancel logic


def test_nothing_changes_without_a_switch():
  cc = run(False)
  assert cc.latActive and cc.longActive and cc.cruiseControl.resume and abs(cc.actuators.torque - 0.8) < 1e-6


def test_the_guard_sits_before_the_startup_gate_and_sending():
  fn, _, _ = guard()
  body = ast.unparse(fn)
  at = body.index("self.params.get_bool('InternalModelSwitching')")
  assert body.index("ImpactDashcamReboot") < at < body.index("self.lateral_startup.update")
