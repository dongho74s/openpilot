"""The Bolt ACC with pedal in StarPilot mode (BoltPedalLongMode 2, GMFlags.PEDAL_LONG) engages only from the driver's
SET/RES: its panda allows longitudinal control only after that button (StarPilot's rule), so carrot's other engages (auto
cruise, soft hold, remote, carrot command) would end in Controls Mismatch (Codex review 2026-11-18). Mode 1 (camera long)
keeps carrot's engages, as the panda grants control from 0x2CB there."""
from types import SimpleNamespace

import pytest

from openpilot.selfdrive.car.cruise import VCruiseCarrot, driver_button_engage_only


def helper(fingerprint, interceptor=True, starpilot=True, op_long=True):
  h = VCruiseCarrot.__new__(VCruiseCarrot)
  h.CP = SimpleNamespace(carFingerprint=fingerprint, enableGasInterceptorDEPRECATED=interceptor, pcmCruise=not op_long,
                         openpilotLongitudinalControl=op_long, flags=1 if starpilot else 0)
  h._driver_button_engage_only = driver_button_engage_only(h.CP)
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


@pytest.mark.parametrize('fingerprint,interceptor,starpilot', [("CHEVROLET_BOLT_CC", True, True), ("CHEVROLET_BOLT_EUV", True, True),
                                                               ("CHEVROLET_TRAVERSE", False, False),
                                                               ("CHEVROLET_BOLT_ACC_PEDAL", False, False),
                                                               ("CHEVROLET_BOLT_ACC_PEDAL", True, False)])  # mode 1
def test_other_setups_keep_carrot_auto_engage(fingerprint, interceptor, starpilot):
  h = helper(fingerprint, interceptor, starpilot)
  h._cruise_control(1, -1, "Cruise on (speed)")
  assert h._activate_cruise == 1


def test_flag_comes_from_car_params():
  """__init__ derives the flag from CarParams through driver_button_engage_only."""
  import inspect
  assert "self._driver_button_engage_only = driver_button_engage_only(CP)" in inspect.getsource(VCruiseCarrot.__init__)
  cp = lambda fp, interceptor, flags, op_long: SimpleNamespace(carFingerprint=fp, enableGasInterceptorDEPRECATED=interceptor,
                                                               flags=flags, openpilotLongitudinalControl=op_long)
  assert driver_button_engage_only(cp("CHEVROLET_BOLT_ACC_PEDAL", True, 1, True))
  assert not driver_button_engage_only(cp("CHEVROLET_BOLT_ACC_PEDAL", True, 0, True))   # mode 1
  assert not driver_button_engage_only(cp("CHEVROLET_BOLT_ACC_PEDAL", False, 1, True))
  assert driver_button_engage_only(cp("CHEVROLET_MALIBU_ASCM", False, 0, True))          # SASCM long
  assert not driver_button_engage_only(cp("CHEVROLET_MALIBU_ASCM", False, 0, False))     # stock ACC
  assert not driver_button_engage_only(cp("CHEVROLET_TRAVERSE", False, 0, True))


@pytest.mark.parametrize('reason,manual', [("Cruise on (soft hold)", False), ("Cruise on (speed)", False),
                                           ("Cruise on (carrot command)", False), ("Cruise on (Bluetooth button)", True)])
def test_malibu_ascm_long_waits_for_the_button(reason, manual):
  """Its panda grants control only from the driver's SET/RES, never from a 0x2CB apply (Codex review 2026-11-21)."""
  h = helper("CHEVROLET_MALIBU_ASCM", interceptor=False, starpilot=False)
  h._cruise_control(1, -1, reason, manual=manual)
  assert h._activate_cruise == 0 and h._soft_hold_active == 0
  h._cruise_control(-1, -1, "Cruise off (gas pressed)", allow_cancel_state=True, manual=True)
  assert h._activate_cruise == -1


def test_malibu_ascm_on_the_stock_acc_keeps_carrot_auto_engage():
  h = helper("CHEVROLET_MALIBU_ASCM", interceptor=False, starpilot=False, op_long=False)
  h._cruise_control(1, -1, "Cruise on (speed)")
  assert h._activate_cruise == 1
