"""The Bolt ACC with pedal engages only from the driver's SET/RES: its panda allows longitudinal control only after that
button (StarPilot's rule), so carrot's other engages (auto cruise, soft hold, remote, carrot command) would end in
Controls Mismatch (Codex review 2026-11-18)."""
from types import SimpleNamespace

import pytest

from openpilot.selfdrive.car.cruise import VCruiseCarrot


def helper(fingerprint, interceptor=True):
  h = VCruiseCarrot.__new__(VCruiseCarrot)
  h.CP = SimpleNamespace(carFingerprint=fingerprint, enableGasInterceptorDEPRECATED=interceptor, pcmCruise=False,
                         openpilotLongitudinalControl=True)
  h._driver_button_engage_only = (fingerprint == "CHEVROLET_BOLT_ACC_PEDAL" and interceptor)
  h._add_log = lambda log: None
  h._cruise_available = True
  h._steering_interlock_active = False
  h._hold_interlock_active = False
  h._cruise_cancel_state = False
  h._cancel_timer = 0
  h.autoCruiseControl = 1
  h.autoCruiseControl_cancel_timer = 0
  h._soft_hold_active = 2
  h._activate_cruise = 0
  return h


@pytest.mark.parametrize('reason,manual', [("Cruise on (soft hold)", False), ("Cruise on (speed)", False),
                                           ("Cruise on (carrot command)", False), ("Cruise on (Bluetooth button)", True)])
def test_bolt_acc_pedal_waits_for_the_button(reason, manual):
  h = helper("CHEVROLET_BOLT_ACC_PEDAL")
  h._cruise_control(1, -1, reason, manual=manual)
  assert h._activate_cruise == 0 and h._soft_hold_active == 0


def test_bolt_acc_pedal_can_still_cancel():
  h = helper("CHEVROLET_BOLT_ACC_PEDAL")
  h._cruise_control(-1, -1, "Cruise off (gas pressed)", allow_cancel_state=True, manual=True)
  assert h._activate_cruise == -1


@pytest.mark.parametrize('fingerprint,interceptor', [("CHEVROLET_BOLT_CC", True), ("CHEVROLET_BOLT_EUV", True),
                                                     ("CHEVROLET_TRAVERSE", False), ("CHEVROLET_BOLT_ACC_PEDAL", False)])
def test_other_setups_keep_carrot_auto_engage(fingerprint, interceptor):
  h = helper(fingerprint, interceptor)
  h._cruise_control(1, -1, "Cruise on (speed)")
  assert h._activate_cruise == 1


def test_flag_comes_from_car_params():
  """__init__ derives the flag the same way; checked on the source line so a rename cannot drift silently."""
  import inspect
  source = inspect.getsource(VCruiseCarrot.__init__)
  assert 'getattr(CP, "carFingerprint", "") == "CHEVROLET_BOLT_ACC_PEDAL"' in source
  assert 'getattr(CP, "enableGasInterceptorDEPRECATED", False)' in source
