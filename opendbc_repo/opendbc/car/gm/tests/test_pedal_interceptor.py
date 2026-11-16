"""Comma pedal (gas interceptor) on GM. Real DBC, CAN parser and CarState; no device I/O.

A Bolt EUV with a comma pedal on WIP-FR showed "CAN Error" because card died on its first CarState update:
the GM DBC had no GAS_SENSOR/GAS_COMMAND, so reading the pedal hit the parser's unknown-message assert.
"""
import pytest

from opendbc.car import Bus, create_gas_interceptor_command, gen_empty_fingerprint
from opendbc.car.gm.fingerprints import FINGERPRINTS
from opendbc.car.gm.values import CAR, DBC, GMSafetyFlags
from opendbc.car.gm.tests.test_display_speed import runtime  # noqa: F401

PEDAL_MSG = 0x201
# GAS_SENSOR frames from that Bolt EUV's rlog: pedal released, STATE 5 (FAULT_TIMEOUT) because card was
# dead and nothing commanded the pedal, COUNTER_PEDAL 1..3
BOLT_PEDAL_FRAMES = [bytes.fromhex(h) for h in ("027201415108", "027301435286", "027301435330")]
PEDAL_CARS = [CAR.CHEVROLET_BOLT_CC, CAR.CHEVROLET_BOLT_EUV]


def pedal_fingerprint(candidate):
  fingerprint = gen_empty_fingerprint()
  fingerprint[0] = {**FINGERPRINTS[candidate][0], PEDAL_MSG: 6}
  return fingerprint


@pytest.mark.parametrize("candidate", sorted(CAR))
def test_every_gm_dbc_defines_the_pedal_messages(runtime, candidate):
  packer = runtime.CANPacker(DBC[candidate][Bus.pt])
  addr, dat, bus = create_gas_interceptor_command(packer, 0.2, 3)
  assert (addr, len(dat), bus) == (0x200, 6, 0)

  parser = runtime.CANParser(DBC[candidate][Bus.pt], [("GAS_SENSOR", float("nan"))], 0)
  parser.update([(1_000_000_000, [(PEDAL_MSG, BOLT_PEDAL_FRAMES[0], 0)])])
  assert parser.vl["GAS_SENSOR"]["STATE"] == 5
  assert parser.vl["GAS_SENSOR"]["COUNTER_PEDAL"] == 1
  assert parser.vl["GAS_SENSOR"]["INTERCEPTOR_GAS"] == pytest.approx(2.765, abs=1e-3)


@pytest.mark.parametrize("candidate", PEDAL_CARS)
def test_pedal_car_state_updates_instead_of_crashing(runtime, candidate):
  cp = runtime.interface.CarInterface.get_params(candidate, pedal_fingerprint(candidate), [], False, True, False)
  assert cp.enableGasInterceptorDEPRECATED
  assert cp.safetyConfigs[-1].safetyParam & GMSafetyFlags.GAS_INTERCEPTOR

  ci = runtime.interface.CarInterface.__new__(runtime.interface.CarInterface)
  ci.CS = runtime.state.CarState(cp)
  ci.can_parsers = ci.CS.get_can_parsers(cp)
  ci.v_ego_cluster_seen = False
  ci.CS.update(ci.can_parsers)  # first access registers GAS_SENSOR; this is where card died

  for i, dat in enumerate(BOLT_PEDAL_FRAMES):
    ret = ci.update([(1_000_000_000 + i * 10_000_000, [(PEDAL_MSG, dat, 0)])])
  assert ret.gas == pytest.approx(3.839, abs=1e-3)
  assert not ret.gasPressed
