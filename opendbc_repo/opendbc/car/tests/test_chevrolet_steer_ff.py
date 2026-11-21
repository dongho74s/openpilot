"""ChevroletSteerFF (Carrot Web, user request 2026-11-21): a Chevrolet can pick the Trailblazer NNFF, the Volt NNFF or
the Volt linear torque values for its steering feed-forward. 0 must leave everything exactly as before."""
import contextlib
import io
import math
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from opendbc.car import gen_empty_fingerprint, interfaces, structs
from opendbc.car.gm import interface as gm_interface
from opendbc.car.gm.values import CAR, GMFlags
from opendbc.car.tests.test_nnff_substitute import IsolatedGMInterface, MemoryParams

CHEVROLETS = tuple(car for car in CAR if str(car).startswith("CHEVROLET_"))
OTHER_GM = tuple(car for car in CAR if not str(car).startswith("CHEVROLET_"))
SAMPLE_CARS = (CAR.CHEVROLET_TRAVERSE, CAR.CHEVROLET_MALIBU_ASCM, CAR.CHEVROLET_TRAILBLAZER, CAR.CHEVROLET_BOLT_EUV,
               CAR.CHEVROLET_BOLT_CC, CAR.CHEVROLET_VOLT, CAR.CHEVROLET_VOLT_CC)
MODEL_FILES = {1: "CHEVROLET_TRAILBLAZER.json", 2: "CHEVROLET_VOLT.json"}
NN_INPUTS = [[speed, accel, friction, roll] + [accel] * 7 + [roll] * 7
             for speed in (5.0, 23.6, 35.0) for accel in (-1.0, 0.0, 0.3) for friction in (0.0, 0.1) for roll in (0.0, 0.02)]


class RaisingParams(MemoryParams):
  def get_int(self, key):
    if key == interfaces.CHEVROLET_STEER_FF_PARAM:
      raise KeyError(key)   # what an old params build without the key would do
    return super().get_int(key)


class TestChevroletSteerFF(unittest.TestCase):
  def setUp(self):
    self.params = MemoryParams()
    for module in (interfaces, gm_interface):
      self.enterContext(patch.object(module, "Params", side_effect=lambda: self.params))
    self.enterContext(contextlib.redirect_stdout(io.StringIO()))

  def interface(self, car, choice=None, nnff=False, nnff_lite=False):
    self.params.values.pop(interfaces.CHEVROLET_STEER_FF_PARAM, None)
    if choice is not None:
      self.params.values[interfaces.CHEVROLET_STEER_FF_PARAM] = choice
    self.params.values["NNFF"] = int(nnff)
    self.params.values["NNFFLite"] = int(nnff_lite)
    self.params.values.pop("NNFFModelName", None)
    cp = IsolatedGMInterface.get_params(car, gen_empty_fingerprint(), [], False, False, False)
    return IsolatedGMInterface(cp)

  def state(self, ci):
    model = ci.lat_torque_nn_model
    return (ci.CP.to_dict(), ci.use_nnff, ci.use_nnff_lite, self.params.get("NNFFModelName"),
            None if model is None else [model.evaluate(x) for x in NN_INPUTS[:4]], ci.torque_from_lateral_accel().__name__)

  def control_state(self, ci):
    # everything that steers; NNFFModelName is only the startup notice text, which any get_params() call rewrites
    state = self.state(ci)
    return state[:3] + state[4:]

  def test_zero_and_unset_are_the_old_behaviour(self):
    for car in SAMPLE_CARS:
      for nnff in (False, True):
        for nnff_lite in (False, True):
          with self.subTest(car=car, nnff=nnff, nnff_lite=nnff_lite):
            before = self.state(self.interface(car, None, nnff, nnff_lite))
            with patch.object(interfaces, "get_chevrolet_steer_ff", return_value=0):
              self.assertEqual(self.state(self.interface(car, 0, nnff, nnff_lite)), before)
            self.assertEqual(self.state(self.interface(car, 0, nnff, nnff_lite)), before)

  def test_only_known_choices_on_chevrolets(self):
    for value in (-1, 4, 99):
      self.params.values[interfaces.CHEVROLET_STEER_FF_PARAM] = value
      self.assertEqual(interfaces.get_chevrolet_steer_ff(CAR.CHEVROLET_TRAVERSE), 0)
    for value in (1, 2, 3):
      self.params.values[interfaces.CHEVROLET_STEER_FF_PARAM] = value
      self.assertEqual(interfaces.get_chevrolet_steer_ff(CAR.CHEVROLET_TRAVERSE), value)
      for car in OTHER_GM + ("HYUNDAI_IONIQ_5", "TOYOTA_RAV4"):
        self.assertEqual(interfaces.get_chevrolet_steer_ff(car), 0)

  def test_other_gm_brands_ignore_it(self):
    for car in OTHER_GM:
      if car not in interfaces.get_torque_params():
        continue   # e.g. GMC_YUKON_CC: no torque values at all, so no CarParams either way (unrelated to this setting)
      for choice in (1, 2, 3):
        with self.subTest(car=car, choice=choice):
          self.assertEqual(self.state(self.interface(car, choice, nnff=True)), self.state(self.interface(car, None, nnff=True)))

  def test_missing_key_never_stops_the_car_interface(self):
    self.params = RaisingParams()
    self.assertEqual(interfaces.get_chevrolet_steer_ff(CAR.CHEVROLET_TRAVERSE), 0)
    ci = IsolatedGMInterface(IsolatedGMInterface.get_params(CAR.CHEVROLET_TRAVERSE, gen_empty_fingerprint(), [], False, False, False))
    self.assertEqual(ci.chevrolet_steer_ff, 0)

  def test_nnff_choices_run_that_model_whatever_the_toggle(self):
    for choice, model_file in MODEL_FILES.items():
      reference = interfaces.FluxModel(str(Path(interfaces.TORQUE_NN_MODEL_PATH) / model_file))
      label = f"{model_file.removesuffix('.json').replace('_', ' ')} (selected)"
      for car in CHEVROLETS:
        for nnff in (False, True):
          with self.subTest(car=car, choice=choice, nnff=nnff):
            ci = self.interface(car, choice, nnff=nnff, nnff_lite=True)
            self.assertTrue(ci.use_nnff)          # ahead of comma's own NN too (Bolts, Volt)
            self.assertFalse(ci.use_nnff_lite)
            self.assertEqual(self.params.get("NNFFModelName"), label)
            self.assertEqual([ci.lat_torque_nn_model.evaluate(x) for x in NN_INPUTS], [reference.evaluate(x) for x in NN_INPUTS])
            # the car keeps its own torque values, exactly as with NNFF on today
            tune = ci.CP.lateralTuning
            self.assertEqual(tune.which(), "torque")
            expected = interfaces.get_torque_params()[car]
            self.assertAlmostEqual(tune.torque.latAccelFactor, expected["LAT_ACCEL_FACTOR"])
            self.assertAlmostEqual(tune.torque.friction, expected["FRICTION"])

  def test_volt_linear_choice(self):
    volt = interfaces.get_torque_params()["CHEVROLET_VOLT"]
    for car in CHEVROLETS:
      for nnff in (False, True):
        for nnff_lite in (False, True):
          with self.subTest(car=car, nnff=nnff, nnff_lite=nnff_lite):
            ci = self.interface(car, 3, nnff=nnff, nnff_lite=nnff_lite)
            self.assertFalse(ci.use_nnff)
            self.assertIsNone(ci.lat_torque_nn_model)
            self.assertEqual(ci.use_nnff_lite, nnff_lite)
            self.assertIsNone(self.params.get("NNFFModelName"))   # no NNFF notice
            tune = ci.CP.lateralTuning
            self.assertEqual(tune.which(), "torque")
            self.assertAlmostEqual(tune.torque.latAccelFactor, volt["LAT_ACCEL_FACTOR"])
            self.assertAlmostEqual(tune.torque.friction, volt["FRICTION"])
            for value, expected_gain in zip((tune.torque.kp, tune.torque.ki, tune.torque.kf), (1.0, 0.1, 1.0), strict=True):
              self.assertAlmostEqual(value, expected_gain, places=6)
            # plain linear torque, not comma's NN (Bolts, Volt) or the sigmoid fits (Bolts)
            self.assertEqual(ci.torque_from_lateral_accel(), ci.torque_from_lateral_accel_linear)

  def test_rebuilt_interface_keeps_the_choice_cp_was_built_with(self):
    # controlsd rebuilds the interface from the saved CarParams; a setting changed in between must not mix modes
    # (Codex review 2026-11-21: 0 -> 3 kept the Traverse torque values under the linear path, 3 -> 0 the Volt ones)
    for car in (CAR.CHEVROLET_TRAVERSE, CAR.CHEVROLET_MALIBU_ASCM, CAR.CHEVROLET_BOLT_EUV):
      for built, changed in ((0, 3), (3, 0), (1, 2), (2, 3), (3, 1), (0, 2)):
        with self.subTest(car=car, built=built, changed=changed):
          first = self.interface(car, built, nnff=True)
          expected = self.control_state(first)
          self.params.values[interfaces.CHEVROLET_STEER_FF_PARAM] = changed
          # other get_params() callers in between (docs, another car) must not matter either (Codex review)
          IsolatedGMInterface.get_params(CAR.GMC_ACADIA, gen_empty_fingerprint(), [], False, False, True)
          IsolatedGMInterface.get_params(CAR.CHEVROLET_TRAX, gen_empty_fingerprint(), [], False, False, False)
          rebuilt = IsolatedGMInterface(first.CP)
          self.assertEqual(rebuilt.chevrolet_steer_ff, built)
          self.assertEqual(self.control_state(rebuilt), expected)

  def test_choice_is_read_back_from_carparams_flags(self):
    def cp(car, flags):
      ret = structs.CarParams()
      ret.carFingerprint = car
      ret.flags = int(flags)
      return ret
    base = int(GMFlags.CC_LONG | GMFlags.SPEED_RELATED_MSG)
    for choice, flag in interfaces.CHEVROLET_STEER_FF_FLAGS.items():
      self.assertEqual(interfaces.chevrolet_steer_ff_from_cp(cp(CAR.CHEVROLET_TRAVERSE, base | flag)), choice)
      for car in OTHER_GM:
        self.assertEqual(interfaces.chevrolet_steer_ff_from_cp(cp(car, base | flag)), 0)
    self.assertEqual(interfaces.chevrolet_steer_ff_from_cp(cp(CAR.CHEVROLET_TRAVERSE, base)), 0)
    both = GMFlags.STEER_FF_TRAILBLAZER_NNFF | GMFlags.STEER_FF_VOLT_LINEAR
    self.assertEqual(interfaces.chevrolet_steer_ff_from_cp(cp(CAR.CHEVROLET_TRAVERSE, both)), 0)
    # the choice bits never collide with the GM platform flags
    platform_bits = GMFlags.PEDAL_LONG | GMFlags.CC_LONG | GMFlags.NO_CAMERA | GMFlags.NO_ACCELERATOR_POS_MSG | \
                    GMFlags.SPEED_RELATED_MSG
    for flag in interfaces.CHEVROLET_STEER_FF_FLAGS.values():
      self.assertFalse(flag & platform_bits)

  def test_linear_choice_clears_a_stale_model_name(self):
    # a model name left by an earlier build would show NNFF on screen and in the startup notice (Codex review)
    for car in (CAR.CHEVROLET_TRAVERSE, CAR.CHEVROLET_BOLT_EUV):
      with self.subTest(car=car):
        self.interface(car, 1)
        self.assertEqual(self.params.get("NNFFModelName"), "CHEVROLET TRAILBLAZER (selected)")
        self.params.values[interfaces.CHEVROLET_STEER_FF_PARAM] = 3
        IsolatedGMInterface.get_params(car, gen_empty_fingerprint(), [], False, False, False)
        self.assertIsNone(self.params.get("NNFFModelName"))

  def test_nothing_but_lateral_tuning_changes(self):
    for car in SAMPLE_CARS:
      base = self.interface(car, 0, nnff=True).CP.to_dict()
      base.pop("lateralTuning")
      base_flags = base.pop("flags")
      for choice in (1, 2, 3):
        with self.subTest(car=car, choice=choice):
          cp = self.interface(car, choice, nnff=True).CP.to_dict()
          cp.pop("lateralTuning")
          self.assertEqual(cp.pop("flags"), base_flags | int(interfaces.CHEVROLET_STEER_FF_FLAGS[choice]))
          self.assertEqual(cp, base)

  def test_real_torque_controller(self):
    from openpilot.selfdrive.controls.lib import latcontrol_torque
    from opendbc.car.vehicle_model import VehicleModel

    cs = SimpleNamespace(vEgo=23.6, aEgo=0.0, steeringAngleDeg=0.5, steeringRateDeg=0.0, steeringPressed=False)
    cc = SimpleNamespace(angularVelocity=[0.0] * 3, orientationNED=[0.0] * 3)
    params = SimpleNamespace(roll=0.0, angleOffsetDeg=0.0)
    with patch.object(latcontrol_torque, "Params", side_effect=lambda: self.params):
      for car in (CAR.CHEVROLET_MALIBU_ASCM, CAR.CHEVROLET_TRAVERSE, CAR.CHEVROLET_BOLT_EUV):
        for choice in (1, 2, 3):
          with self.subTest(car=car, choice=choice):
            ci = self.interface(car, choice)
            controller = latcontrol_torque.LatControlTorque(ci.CP.as_reader(), ci)
            self.assertEqual(controller.use_nnff, choice != 3)
            # NNFF keeps FrogPilot's lateral computation; the linear choice runs the stock torque controller
            self.assertEqual(controller.nnff_frogpilot_parity, choice != 3)
            for curvature in (0.0, 0.0005, -0.0005):
              torque, _, log = controller.update(True, cs, VehicleModel(ci.CP), params, False, curvature, cc, False, None)
              self.assertTrue(log.active and math.isfinite(torque) and abs(torque) <= 1.0)

  def test_web_setting_contract(self):
    import json
    root = Path(__file__).resolve().parents[4]
    settings_file = root / "openpilot/selfdrive/carrot_settings.json"
    keys_file = root / "openpilot/common/params_keys.h"
    if not settings_file.exists():
      self.skipTest("Openpilot settings are outside this opendbc checkout")
    settings = json.loads(settings_file.read_text(encoding="utf-8"))
    entry = next(item for item in settings["params"] if item["name"] == interfaces.CHEVROLET_STEER_FF_PARAM)
    self.assertEqual((entry["min"], entry["max"], entry["default"], entry["control"]), (0, 3, 0, "select"))
    for language in ("ko", "en", "zh"):
      self.assertEqual(len(entry["options"][language]), 4)
    self.assertNotIn("gm", entry["hidden_brands"])
    self.assertIn('{"ChevroletSteerFF", {PERSISTENT, INT, "0"}}', keys_file.read_text(encoding="utf-8"))
    self.assertIn('"NNFF", "ChevroletSteerFF"', json.dumps(settings, ensure_ascii=False))   # next to NNFF in the menu


if __name__ == "__main__":
  unittest.main()
