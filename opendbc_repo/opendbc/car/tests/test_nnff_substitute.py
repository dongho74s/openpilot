import contextlib
import io
import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from opendbc.car import gen_empty_fingerprint, interfaces
from opendbc.car.gm import interface as gm_interface
from opendbc.car.gm.values import CAR


class MemoryParams:
  def __init__(self):
    self.values = {"NNFF": 0, "NNFFLite": 0}

  def get(self, key):
    return self.values.get(key)

  def get_bool(self, key):
    return bool(self.values.get(key, 0))

  def get_int(self, key):
    return int(self.values.get(key, 0))

  def get_float(self, key):
    return float(self.values.get(key, 0))

  def put_int(self, key, value):
    self.values[key] = value

  def put_nonblocking(self, key, value):
    self.values[key] = value

  def remove(self, key):
    self.values.pop(key, None)


class IsolatedGMInterface(gm_interface.CarInterface):
  # No CAN parsing, transmitting or device access in the integration fixture.
  CarState = staticmethod(lambda cp: SimpleNamespace(get_can_parsers=lambda cp: {}))
  CarController = staticmethod(lambda dbc, cp: None)


class TestNNFFSubstitute(unittest.TestCase):
  def setUp(self):
    self.params = MemoryParams()
    self.donor = Path(interfaces.TORQUE_NN_MODEL_PATH) / "CHEVROLET_TRAILBLAZER.json"
    self.catalog = Path(interfaces.TORQUE_NN_MODEL_PATH)
    for module in (interfaces, gm_interface):
      self.enterContext(patch.object(module, "Params", return_value=self.params))
    self.enterContext(contextlib.redirect_stdout(io.StringIO()))

  def car_params(self, nnff):
    self.params.values["NNFF"] = int(nnff)
    return IsolatedGMInterface.get_params(CAR.CHEVROLET_TRAVERSE, gen_empty_fingerprint(), [], False, False, False)

  def select(self, car=CAR.CHEVROLET_TRAVERSE, firmware="", enabled=False):
    return interfaces.get_nn_model_path(car, firmware, allow_substitute=enabled)

  def test_substitute_is_opt_in(self):
    self.assertIsNone(self.select())
    self.assertEqual(self.select(enabled=True), str(self.donor))

  def test_firmware_does_not_change_explicit_donor(self):
    self.assertEqual(self.select(firmware="unknown EPS firmware", enabled=True), str(self.donor))

  def test_native_model_has_precedence(self):
    with tempfile.TemporaryDirectory() as directory, patch.object(interfaces, "TORQUE_NN_MODEL_PATH", directory):
      for name in ("CHEVROLET_TRAVERSE", "CHEVROLET_TRAILBLAZER"):
        (Path(directory) / f"{name}.json").touch()
      self.assertEqual(self.select(enabled=True), str(Path(directory) / "CHEVROLET_TRAVERSE.json"))

  def test_missing_donor_is_not_guessed(self):
    with tempfile.TemporaryDirectory() as directory, patch.object(interfaces, "TORQUE_NN_MODEL_PATH", directory):
      (Path(directory) / "CHEVROLET_TRAILBLAZER_CC.json").touch()
      self.assertIsNone(self.select(enabled=True))

  def test_empty_catalog_disables_nnff(self):
    with tempfile.TemporaryDirectory() as directory, patch.object(interfaces, "TORQUE_NN_MODEL_PATH", directory):
      self.assertIsNone(self.select(enabled=True))
      self.params.values["NNFF"] = 1
      cp = self.car_params(True)
      ci = IsolatedGMInterface(cp)
      self.assertFalse(ci.use_nnff)
      self.assertIsNone(ci.lat_torque_nn_model)
      self.assertIsNone(self.params.get("NNFFModelName"))

  def test_substitutes_are_explicit(self):
    self.assertEqual(interfaces.NNFF_MODEL_SUBSTITUTES,
                     {"CHEVROLET_TRAVERSE": "CHEVROLET_TRAILBLAZER", "CHEVROLET_BOLT_CC": "CHEVROLET_BOLT_EUV",
                      "CHEVROLET_BOLT_ACC_PEDAL": "CHEVROLET_BOLT_EUV", "CHEVROLET_MALIBU_ASCM": "CHEVROLET_TRAILBLAZER",
                      "CHEVROLET_MALIBU": "CHEVROLET_TRAILBLAZER", "CHEVROLET_MALIBU_CC": "CHEVROLET_TRAILBLAZER",
                      "CHEVROLET_VOLT_2019": "CHEVROLET_TRAILBLAZER", "CHEVROLET_VOLT_CC": "CHEVROLET_TRAILBLAZER",
                      "CHEVROLET_TRAILBLAZER_CC": "CHEVROLET_TRAILBLAZER", "CHEVROLET_TRAX": "CHEVROLET_TRAILBLAZER"})
    self.assertIsNone(self.select(car="UNSUPPORTED_CAR", enabled=True))
    self.assertIsNone(self.select(car="CHEVROLET_TRAVERSE_OTHER", enabled=True))

  def test_existing_catalog_matches_are_unchanged(self):
    models = sorted(self.catalog.glob("*.json"))
    self.assertGreater(len(models), 100)
    for model in models:
      with self.subTest(model=model.name):
        self.assertEqual(self.select(car=model.stem), self.select(car=model.stem, enabled=True))

  def test_model_loads_and_evaluates_full_inputs(self):
    model = interfaces.get_nn_model(CAR.CHEVROLET_TRAVERSE, "", allow_substitute=True)
    self.assertIsNotNone(model)
    self.assertEqual(model.input_size, 18)
    self.assertEqual(model.output_size, 1)
    for speed in (10.0, 20.0, 30.0):
      for accel in (-1.0, 0.0, 1.0):
        for roll in (-0.02, 0.0, 0.02):
          with self.subTest(speed=speed, accel=accel, roll=roll):
            inputs = [speed, accel, 0.0, roll] + [accel] * 7 + [roll] * 7
            self.assertTrue(math.isfinite(model.evaluate(inputs)))

  def test_enabled_interface_uses_donor(self):
    cp = self.car_params(True)
    ci = IsolatedGMInterface(cp)
    self.assertTrue(ci.use_nnff)
    self.assertFalse(ci.use_nnff_lite)
    self.assertIsNotNone(ci.lat_torque_nn_model)
    self.assertEqual(self.params.get("NNFFModelName"), "CHEVROLET TRAILBLAZER (substitute)")

  def test_disabled_interface_does_not_use_donor(self):
    cp = self.car_params(False)
    ci = IsolatedGMInterface(cp)
    self.assertFalse(ci.use_nnff)
    self.assertIsNone(ci.lat_torque_nn_model)
    self.assertIsNone(self.params.get("NNFFModelName"))

  def test_no_vehicle_or_safety_parameter_substitution(self):
    off = self.car_params(False)
    on = self.car_params(True)
    self.assertEqual(on.to_dict(), off.to_dict())
    self.assertEqual(on.carFingerprint, CAR.CHEVROLET_TRAVERSE)
    self.assertEqual(on.lateralTuning.which(), "torque")

  def test_comma_native_nnff_keeps_precedence(self):
    cp = self.car_params(True)
    with patch.object(IsolatedGMInterface, "check_comma_nn_ff_support", return_value=True):
      self.assertFalse(IsolatedGMInterface(cp).use_nnff)

  def test_lite_setting_is_not_changed(self):
    self.params.values["NNFFLite"] = 1
    off = IsolatedGMInterface(self.car_params(False))
    self.assertFalse(off.use_nnff)
    self.assertTrue(off.use_nnff_lite)
    on = IsolatedGMInterface(self.car_params(True))
    self.assertTrue(on.use_nnff)
    self.assertFalse(on.use_nnff_lite)
    self.assertEqual(self.params.get("NNFFLite"), 1)

  def test_real_torque_controller_selects_donor_only_when_enabled(self):
    from openpilot.selfdrive.controls.lib import latcontrol_torque

    with patch.object(latcontrol_torque, "Params", return_value=self.params):
      for enabled in (False, True):
        with self.subTest(enabled=enabled):
          cp = self.car_params(enabled)
          ci = IsolatedGMInterface(cp)
          controller = latcontrol_torque.LatControlTorque(cp.as_reader(), ci)
          self.assertEqual(controller.use_nnff, enabled)
          self.assertEqual(controller.steer_max, 1.0)
          if enabled:
            self.assertTrue(math.isfinite(controller.torque_from_nn([20.0, 0.0, 0.0, 0.0] + [0.0] * 14)))

  def controller_step(self, active, model_available):
    from openpilot.selfdrive.controls.lib import latcontrol_torque
    from opendbc.car.vehicle_model import VehicleModel

    cp = self.car_params(True)
    ci = IsolatedGMInterface(cp)
    with patch.object(latcontrol_torque, "Params", return_value=self.params):
      controller = latcontrol_torque.LatControlTorque(cp.as_reader(), ci)
      cs = SimpleNamespace(vEgo=20.0, aEgo=0.0, steeringAngleDeg=0.0,
                           steeringRateDeg=0.0, steeringPressed=False)
      cc = SimpleNamespace(angularVelocity=[0.0] * 3, orientationNED=[0.0] * 3)
      params = SimpleNamespace(roll=0.0, angleOffsetDeg=0.0)
      count = len(latcontrol_torque.ModelConstants.T_IDXS)
      model = SimpleNamespace(orientation=SimpleNamespace(x=[0.0] * count, y=[0.0] * count),
                              acceleration=SimpleNamespace(y=[0.4] * count)) if model_available else None
      with patch.object(controller, "torque_from_nn", wraps=controller.torque_from_nn) as evaluate:
        torque, angle, log = controller.update(active, cs, VehicleModel(cp), params, False, 0.001, cc, False, model)
        self.assertTrue(math.isfinite(torque) and math.isfinite(angle))
        self.assertLessEqual(abs(torque), 1.0)
        return torque, log.active, evaluate.call_count

  def test_active_controller_actually_evaluates_nnff(self):
    _, active, calls = self.controller_step(True, True)
    self.assertTrue(active)
    self.assertGreaterEqual(calls, 3)

  def test_inactive_controller_never_issues_torque(self):
    torque, active, calls = self.controller_step(False, True)
    self.assertEqual(torque, 0.0)
    self.assertFalse(active)
    self.assertEqual(calls, 0)

  def test_missing_model_keeps_existing_fallback(self):
    _, active, calls = self.controller_step(True, False)
    self.assertTrue(active)
    self.assertEqual(calls, 0)

  def test_web_toggle_contract(self):
    root = Path(__file__).resolve().parents[4]
    settings_file = root / "openpilot/selfdrive/carrot_settings.json"
    if not settings_file.exists():
      self.skipTest("Openpilot settings are outside this opendbc checkout")
    settings = json.loads(settings_file.read_text())
    entry = next(item for item in settings["params"] if item["name"] == "NNFF")
    self.assertEqual(entry["default"], 0)
    self.assertIn("트레일블레이저", entry["descr"])
    self.assertIn("experimental substitute", entry["edescr"])


BOLT_EUV_MODEL_SHA256 = "6f973c101189318d190575dd3e000b7e43a8332360c8eb11ee7f9241dd58a2fa"  # FrogPilot d73201f2


class TestNNFFOverCommaNN(unittest.TestCase):
  """User request 2026-11-15: the Bolt EUV and Volt 2017-18 use NNFF like FrogPilot, which picks its NNFF whenever it
  has a model, while this fork kept comma's own NN feedforward first for them. User request 2026-11-17: the Bolt
  No-ACC (CHEVROLET_BOLT_CC) does the same with the Bolt EUV model as its substitute."""

  def setUp(self):
    self.params = MemoryParams()
    for module in (interfaces, gm_interface):
      self.enterContext(patch.object(module, "Params", return_value=self.params))
    self.enterContext(contextlib.redirect_stdout(io.StringIO()))

  def interface(self, car, nnff=True):
    self.params.values["NNFF"] = int(nnff)
    cp = IsolatedGMInterface.get_params(car, gen_empty_fingerprint(), [], False, False, False)
    return IsolatedGMInterface(cp)

  def test_exceptions_are_only_the_bolts_and_volt(self):
    self.assertEqual(interfaces.NNFF_OVER_COMMA_NN,
                     ("CHEVROLET_BOLT_EUV", "CHEVROLET_VOLT", "CHEVROLET_BOLT_CC", "CHEVROLET_BOLT_ACC_PEDAL"))

  def test_bolt_acc_pedal_steers_like_the_bolt_cc(self):
    """The Bolt ACC with pedal is the same car and steering as the Bolt No-ACC: same NNFF donor, comma NN and timing."""
    for enabled in (False, True):
      cc = self.interface(CAR.CHEVROLET_BOLT_CC, nnff=enabled)
      acc_pedal = self.interface(CAR.CHEVROLET_BOLT_ACC_PEDAL, nnff=enabled)
      self.assertEqual(cc.use_nnff, acc_pedal.use_nnff)
      self.assertEqual(cc.CP.lateralTuning.to_dict(), acc_pedal.CP.lateralTuning.to_dict())
      self.assertEqual(cc.CP.steerActuatorDelay, acc_pedal.CP.steerActuatorDelay)
      self.assertEqual(cc.CP.steerRatio, acc_pedal.CP.steerRatio)
    self.assertTrue(self.interface(CAR.CHEVROLET_BOLT_ACC_PEDAL).check_comma_nn_ff_support(CAR.CHEVROLET_BOLT_ACC_PEDAL))

  def test_bolt_euv_and_volt_use_nnff_over_comma_nn(self):
    for car, model in ((CAR.CHEVROLET_BOLT_EUV, "CHEVROLET_BOLT_EUV.json"), (CAR.CHEVROLET_VOLT, "CHEVROLET_VOLT.json")):
      with self.subTest(car=car):
        on = self.interface(car)
        self.assertTrue(on.check_comma_nn_ff_support(car))   # comma ships its own NN for it
        self.assertTrue(on.use_nnff)
        self.assertFalse(on.use_nnff_lite)
        self.assertEqual(Path(interfaces.get_nn_model_path(car, "", allow_substitute=True)).name, model)
        self.assertFalse(self.interface(car, nnff=False).use_nnff)

  def test_other_comma_nn_cars_keep_comma_first(self):
    with patch.object(IsolatedGMInterface, "check_comma_nn_ff_support", return_value=True):
      self.assertFalse(self.interface(CAR.CHEVROLET_EQUINOX).use_nnff)   # an NNFF model alone does not override

  def test_bolt_cc_uses_bolt_euv_nnff_only_when_enabled(self):
    on = self.interface(CAR.CHEVROLET_BOLT_CC)
    self.assertTrue(on.check_comma_nn_ff_support(CAR.CHEVROLET_BOLT_CC))   # comma ships its own NN for it
    self.assertTrue(on.use_nnff)
    self.assertFalse(on.use_nnff_lite)
    self.assertEqual(Path(interfaces.get_nn_model_path(CAR.CHEVROLET_BOLT_CC, "", allow_substitute=True)).name,
                     "CHEVROLET_BOLT_EUV.json")
    self.assertEqual(self.params.get("NNFFModelName"), "CHEVROLET BOLT EUV (substitute)")
    # the donor is only borrowed with NNFF on, and an EPS firmware string does not change it
    self.assertIsNone(interfaces.get_nn_model_path(CAR.CHEVROLET_BOLT_CC, ""))
    self.assertEqual(Path(interfaces.get_nn_model_path(CAR.CHEVROLET_BOLT_CC, "unknown EPS firmware",
                                                       allow_substitute=True)).name, "CHEVROLET_BOLT_EUV.json")

  def test_bolt_cc_without_nnff_keeps_comma_nn(self):
    self.params.values.pop("NNFFModelName", None)
    off = self.interface(CAR.CHEVROLET_BOLT_CC, nnff=False)
    self.assertFalse(off.use_nnff)
    self.assertIsNone(off.lat_torque_nn_model)
    self.assertIsNone(self.params.get("NNFFModelName"))
    self.assertEqual(off.torque_from_lateral_accel(), off.torque_from_lateral_accel_neural)

  def test_bolt_cc_substitute_changes_no_vehicle_parameters(self):
    off = IsolatedGMInterface.get_params(CAR.CHEVROLET_BOLT_CC, gen_empty_fingerprint(), [], False, False, False)
    self.params.values["NNFF"] = 1
    on = IsolatedGMInterface.get_params(CAR.CHEVROLET_BOLT_CC, gen_empty_fingerprint(), [], False, False, False)
    self.assertEqual(on.to_dict(), off.to_dict())
    self.assertEqual(on.carFingerprint, CAR.CHEVROLET_BOLT_CC)

  def test_bolt_cc_real_torque_controller_runs_nnff(self):
    from openpilot.selfdrive.controls.lib import latcontrol_torque

    with patch.object(latcontrol_torque, "Params", return_value=self.params):
      for enabled in (False, True):
        with self.subTest(enabled=enabled):
          ci = self.interface(CAR.CHEVROLET_BOLT_CC, nnff=enabled)
          controller = latcontrol_torque.LatControlTorque(ci.CP.as_reader(), ci)
          self.assertEqual(controller.use_nnff, enabled)
          # Chevrolet with NNFF runs FrogPilot's lateral computation, as the Bolt EUV already does
          self.assertEqual(controller.nnff_frogpilot_parity, enabled)
          if enabled:
            for speed in (5.0, 20.0, 35.0):
              for accel in (-2.0, 0.0, 2.0):
                self.assertTrue(math.isfinite(controller.torque_from_nn([speed, accel, 0.0, 0.0] + [accel] * 7 + [0.0] * 7)))

  def test_bolt_euv_model_is_frogpilots_and_evaluates(self):
    import hashlib
    path = Path(interfaces.TORQUE_NN_MODEL_PATH) / "CHEVROLET_BOLT_EUV.json"
    self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), BOLT_EUV_MODEL_SHA256)
    model = interfaces.get_nn_model(CAR.CHEVROLET_BOLT_EUV, "")
    self.assertEqual((model.input_size, model.output_size), (18, 1))
    for speed in (5.0, 20.0, 35.0):
      for accel in (-2.0, 0.0, 2.0):
        inputs = [speed, accel, 0.0, 0.0] + [accel] * 7 + [0.0] * 7
        self.assertTrue(math.isfinite(model.evaluate(inputs)))

  def test_new_model_does_not_capture_other_platforms(self):
    # the Bolt CC gets the Bolt EUV model only through its explicit substitute, never through name matching
    self.assertIsNone(interfaces.get_nn_model_path("CHEVROLET_BOLT_CC", ""))
    for car in ("CHEVROLET_VOLT_2019", "CHEVROLET_VOLT_CC", "CHEVROLET_MALIBU", "CHEVROLET_TRAX"):
      with self.subTest(car=car):
        self.assertIsNone(interfaces.get_nn_model_path(car, ""))
        # with NNFF on they borrow the Trailblazer model (user request 2026-11-19), never the Bolt EUV one
        self.assertEqual(Path(interfaces.get_nn_model_path(car, "", allow_substitute=True)).name, "CHEVROLET_TRAILBLAZER.json")

  def test_real_torque_controller_runs_frogpilot_parity_on_them(self):
    from openpilot.selfdrive.controls.lib import latcontrol_torque
    with patch.object(latcontrol_torque, "Params", return_value=self.params):
      for car in (CAR.CHEVROLET_BOLT_EUV, CAR.CHEVROLET_VOLT):
        with self.subTest(car=car):
          ci = self.interface(car)
          controller = latcontrol_torque.LatControlTorque(ci.CP.as_reader(), ci)
          self.assertTrue(controller.use_nnff and controller.nnff_frogpilot_parity)
          self.assertTrue(math.isfinite(controller.torque_from_nn([20.0, 0.0, 0.0, 0.0] + [0.0] * 14)))


BORROWERS = (CAR.CHEVROLET_MALIBU, CAR.CHEVROLET_MALIBU_CC, CAR.CHEVROLET_MALIBU_ASCM, CAR.CHEVROLET_VOLT_2019,
             CAR.CHEVROLET_VOLT_CC, CAR.CHEVROLET_TRAILBLAZER_CC, CAR.CHEVROLET_TRAX)


class TestChevroletTrailblazerNNFF(unittest.TestCase):
  """User request 2026-11-19: every Chevrolet without an NNFF model of its own (the Malibu ASCM harness first) borrows
  the Trailblazer's with NNFF on, exactly as the Traverse does. The Bolts keep the Bolt EUV donor."""

  def setUp(self):
    self.params = MemoryParams()
    for module in (interfaces, gm_interface):
      self.enterContext(patch.object(module, "Params", return_value=self.params))
    self.enterContext(contextlib.redirect_stdout(io.StringIO()))

  def interface(self, car, nnff=True):
    self.params.values["NNFF"] = int(nnff)
    cp = IsolatedGMInterface.get_params(car, gen_empty_fingerprint(), [], False, False, False)
    return IsolatedGMInterface(cp)

  def test_every_chevrolet_has_nnff_with_it_on(self):
    own, borrowed = [], []
    for car in CAR:
      if not str(car).startswith("CHEVROLET_"):
        continue
      with self.subTest(car=car):
        self.assertTrue(self.interface(car).use_nnff)
        if interfaces.get_nn_model_path(car, "") is not None:
          own.append(str(car))
        else:
          borrowed.append(str(car))
          self.assertIn(str(car), interfaces.NNFF_MODEL_SUBSTITUTES)
    self.assertEqual(sorted(borrowed), sorted(interfaces.NNFF_MODEL_SUBSTITUTES))
    self.assertEqual(sorted(interfaces.TRAILBLAZER_NNFF_BORROWERS), sorted(map(str, BORROWERS + (CAR.CHEVROLET_TRAVERSE,))))
    # a car with its own model is never sent to a donor
    self.assertFalse(set(own) & set(interfaces.NNFF_MODEL_SUBSTITUTES))

  def test_other_gm_brands_are_unchanged(self):
    for car in CAR:
      if str(car).startswith("CHEVROLET_"):
        continue
      with self.subTest(car=car):
        self.assertNotIn(str(car), interfaces.NNFF_MODEL_SUBSTITUTES)
        self.assertEqual(interfaces.get_nn_model_path(car, ""), interfaces.get_nn_model_path(car, "", allow_substitute=True))

  def test_borrowers_use_the_traverse_donor_only_with_nnff_on(self):
    for car in BORROWERS:
      for firmware in ("", "unknown EPS firmware"):
        with self.subTest(car=car, firmware=firmware):
          self.assertIsNone(interfaces.get_nn_model_path(car, firmware))
          donor = interfaces.get_nn_model_path(car, firmware, allow_substitute=True)
          self.assertEqual(Path(donor).name, "CHEVROLET_TRAILBLAZER.json")
          self.assertEqual(donor, interfaces.get_nn_model_path(CAR.CHEVROLET_TRAVERSE, firmware, allow_substitute=True))

      with self.subTest(car=car):
        self.params.values.pop("NNFFModelName", None)
        on = self.interface(car)
        self.assertFalse(on.check_comma_nn_ff_support(car))
        self.assertTrue(on.use_nnff)
        self.assertFalse(on.use_nnff_lite)
        self.assertEqual(self.params.get("NNFFModelName"), "CHEVROLET TRAILBLAZER (substitute)")

        self.params.values.pop("NNFFModelName")
        off = self.interface(car, nnff=False)
        self.assertFalse(off.use_nnff)
        self.assertIsNone(off.lat_torque_nn_model)
        self.assertIsNone(self.params.get("NNFFModelName"))

  def test_substitute_changes_no_vehicle_parameters(self):
    for car in BORROWERS:
      with self.subTest(car=car):
        with patch.dict(interfaces.NNFF_MODEL_SUBSTITUTES, clear=True):
          before = self.interface(car).CP   # NNFF on without any donor: the parameters before this change
        on = self.interface(car).CP
        self.assertEqual(on.to_dict(), before.to_dict())
        self.assertEqual(on.carFingerprint, car)
        self.assertEqual(on.lateralTuning.which(), "torque")

  def test_volt_cc_steers_with_nnff_off(self):
    # it used to get an empty PID with NNFF off, which raised once engaged; it now runs the torque controller
    from openpilot.selfdrive.controls.lib import latcontrol_torque
    from opendbc.car.vehicle_model import VehicleModel

    ci = self.interface(CAR.CHEVROLET_VOLT_CC, nnff=False)
    self.assertEqual(ci.CP.lateralTuning.which(), "torque")
    self.assertFalse(ci.use_nnff)
    with patch.object(latcontrol_torque, "Params", return_value=self.params):
      controller = latcontrol_torque.LatControlTorque(ci.CP.as_reader(), ci)
      cs = SimpleNamespace(vEgo=20.0, aEgo=0.0, steeringAngleDeg=0.0, steeringRateDeg=0.0, steeringPressed=False)
      cc = SimpleNamespace(angularVelocity=[0.0] * 3, orientationNED=[0.0] * 3)
      params = SimpleNamespace(roll=0.0, angleOffsetDeg=0.0)
      for curvature in (0.0, 0.002, -0.002):
        torque, _, log = controller.update(True, cs, VehicleModel(ci.CP), params, False, curvature, cc, False, None)
        self.assertTrue(log.active and math.isfinite(torque) and abs(torque) <= 1.0)

  def test_real_torque_controller_runs_the_traverse_nnff_path(self):
    from openpilot.selfdrive.controls.lib import latcontrol_torque

    with patch.object(latcontrol_torque, "Params", return_value=self.params):
      traverse_ci = self.interface(CAR.CHEVROLET_TRAVERSE)
      traverse = latcontrol_torque.LatControlTorque(traverse_ci.CP.as_reader(), traverse_ci)
      for car in BORROWERS:
        for enabled in (False, True):
          with self.subTest(car=car, enabled=enabled):
            ci = self.interface(car, nnff=enabled)
            self.assertEqual(ci.CP.lateralTuning.which(), "torque")   # torque tuning either way, as on the Traverse
            controller = latcontrol_torque.LatControlTorque(ci.CP.as_reader(), ci)
            self.assertEqual(controller.use_nnff, enabled)
            self.assertEqual(controller.nnff_frogpilot_parity, enabled)
            # like the Traverse, they keep the stock error blend (the low-speed cut is the Trailblazer's alone)
            self.assertFalse(controller.low_speed_error_blend)
            if enabled:
              # the same model as the Traverse: identical output for identical inputs
              for speed in (5.0, 20.0, 35.0):
                for accel in (-2.0, 0.0, 2.0):
                  for roll in (-0.02, 0.0, 0.02):
                    inputs = [speed, accel, 0.0, roll] + [accel] * 7 + [roll] * 7
                    value = controller.torque_from_nn(inputs)
                    self.assertTrue(math.isfinite(value))
                    self.assertEqual(value, traverse.torque_from_nn(inputs))


if __name__ == "__main__":
  unittest.main()
