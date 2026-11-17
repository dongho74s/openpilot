"""Shared GM test runtime and helpers (desktop checkout; no device I/O).

The Wayon display-mapping tests for this module live on the WIP-FR branch
(cluster_speed is not on carrot-nnff); this file only carries the helpers
that the GM tests on this branch import: MemoryParams, the module-scoped
``runtime`` fixture, build_cp and controller_state.
"""
from collections import defaultdict
import sys
from types import ModuleType, SimpleNamespace as NS

import pytest

from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.gm.values import CAR


class MemoryParams:
  def __init__(self, *args):
    self.values = {"CruiseSpeedUnit": 10, "CruiseSpeedUnitBasic": 1, "CruiseButtonLongDelay": 50,
                   "AutoNaviSpeedSafetyFactor": 100, "AutoGasTokSpeed": 30, "AutoGasCancelSpeed": 30,
                   "LongActuatorDelay": 50, "SpeedFromPCM": 2}

  def get_int(self, key):
    return int(self.values.get(key, 0))

  def get_float(self, key):
    return float(self.values.get(key, 0))

  def get_bool(self, key):
    return bool(self.values.get(key, False))

  def put_int_nonblocking(self, key, value):
    self.values[key] = int(value)

  def put_bool_nonblocking(self, key, value):
    self.values[key] = bool(value)


@pytest.fixture(scope="module")
def runtime():
  # Desktop checkout need not build Params or open a hardware/IPC connection.
  # Only I/O dependencies are replaced; parser, state, KF and controller run whole.
  with pytest.MonkeyPatch.context() as patch:
    params = ModuleType("openpilot.common.params")
    params.Params = MemoryParams
    patch.setitem(sys.modules, params.__name__, params)
    realtime = ModuleType("openpilot.common.realtime")
    realtime.DT_CTRL, realtime.DT_MDL = 0.01, 0.05
    patch.setitem(sys.modules, realtime.__name__, realtime)
    from opendbc.can import CANPacker, CANParser
    import opendbc.car.gm.carstate as state
    import opendbc.car.gm.carcontroller as controller
    import opendbc.car.gm.interface as interface
    import opendbc.car.interfaces as interfaces
    import openpilot.selfdrive.car.cruise as cruise
    for module in (state, controller, interface, interfaces, cruise):
      patch.setattr(module, "Params", MemoryParams)
    patch.setattr(cruise, "CommandReader", lambda *args: NS(read=lambda **kwargs: None, is_repeat=False))
    yield NS(state=state, controller=controller, interface=interface, cruise=cruise,
             CANPacker=CANPacker, CANParser=CANParser)


def build_cp(runtime, candidate=CAR.CHEVROLET_TRAVERSE, alpha=True):
  return runtime.interface.CarInterface.get_params(candidate, gen_empty_fingerprint(), [], alpha, True, False)


def controller_state():
  out = structs.CarState(vEgo=10., standstill=False)
  out.cruiseState.enabled = True
  # Same defaults as GM CarState.__init__, including the Trailblazer camera-long stock references.
  # manual_auto_hold is always present on the real CarState (a hold or None).
  return NS(out=out, loopback_lka_steering_cmd_ts_nanos=0, loopback_lka_steering_cmd_updated=False,
            cam_lka_steering_cmd_counter=0, pt_lka_steering_cmd_counter=0, buttons_counter=0,
            pcm_acc_status=0, pscm_status=defaultdict(int), manual_auto_hold=None,
            cam_ascm_2cb_counter=0, cam_ascm_2cb_counter_updated=False, cam_ascm_2cb_counter_ts_nanos=0,
            cam_stock_long_active=None, cam_stock_long_cancel=False, cam_acc_status=None, cam_fcw_alert=0)
