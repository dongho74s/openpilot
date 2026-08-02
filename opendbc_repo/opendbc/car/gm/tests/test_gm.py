from types import SimpleNamespace

from parameterized import parameterized

from opendbc.can import CANPacker, CANParser
from opendbc.car import structs
from opendbc.car.gm.fingerprints import FINGERPRINTS
from opendbc.car.gm.gmcan import (apply_driver_gas_override, create_acc_dashboard_command, create_friction_brake_command,
                                 create_gas_regen_command, get_longitudinal_command_timing)
from opendbc.car.gm.values import CAR, CAMERA_ACC_CAR, GM_RX_OFFSET

CAMERA_DIAGNOSTIC_ADDRESS = 0x24B
NetworkLocation = structs.CarParams.NetworkLocation


class TestGMFingerprint:
  @parameterized.expand(FINGERPRINTS.items())
  def test_can_fingerprints(self, car_model, fingerprints):
    assert len(fingerprints) > 0

    assert all(len(finger) for finger in fingerprints)

    # The camera can sometimes be communicating on startup
    if car_model in CAMERA_ACC_CAR:
      for finger in fingerprints:
        for required_addr in (CAMERA_DIAGNOSTIC_ADDRESS, CAMERA_DIAGNOSTIC_ADDRESS + GM_RX_OFFSET):
          assert finger.get(required_addr) == 8, required_addr


class TestTrailblazerLongitudinalIntegrity:
  @parameterized.expand([
    ("trailblazer_gas", CAR.CHEVROLET_TRAILBLAZER, True, (-500, 0, False, False)),
    ("trailblazer_released", CAR.CHEVROLET_TRAILBLAZER, False, (-540, 143, True, True)),
    ("other_gm_gas", CAR.CHEVROLET_EQUINOX, True, (-540, 143, True, True)),
  ])
  def test_driver_gas_override_values(self, _, car_fingerprint, gas_pressed, expected):
    result = apply_driver_gas_override(car_fingerprint, gas_pressed, -500, -540, 143, True, True)
    assert result == expected

  def test_driver_gas_override_keeps_counter_group_transmittable(self):
    packer = CANPacker("gm_global_a_powertrain_volt")
    CP = SimpleNamespace(carFingerprint=CAR.CHEVROLET_TRAILBLAZER)
    apply_gas, apply_brake, at_full_stop, near_stop = apply_driver_gas_override(
      CP.carFingerprint, True, -500, -540, 143, False, True,
    )

    gas_msg = create_gas_regen_command(packer, 0, apply_gas, 3, True, at_full_stop)
    brake_msg = create_friction_brake_command(packer, 0, apply_brake, 3, True, near_stop, at_full_stop, CP)

    assert gas_msg[1].hex() == "c142b09000bd4f6d"
    assert brake_msg[1].hex() == "1000effd03"

  @parameterized.expand(
    [
      # Captured stock Trailblazer frames. The first two exercise the lower
      # and upper 24-bit carry/borrow boundaries missed by the old byte-wise
      # checksum implementation.
      ("counter_0_low_byte_zero", 346, 0, True, "0142cb0000bd3500"),
      ("counter_3_low_byte_overflow", 249.75, 3, True, "c142c7fe00bd37ff"),
      ("inactive_checksum_bit", -500, 2, False, "8042b09001bd4f6e"),
    ]
  )
  def test_gas_regen_checksum_matches_stock(self, _, throttle, counter, enabled, expected_payload):
    packer = CANPacker("gm_global_a_powertrain_volt")
    msg = create_gas_regen_command(packer, 0, throttle, counter, enabled, False)
    assert msg[1].hex() == expected_payload

  @parameterized.expand(
    [
      ("counter_0", "002c03d3fd", 0),
      ("counter_1", "402c03d3fc", 1),
      ("counter_2", "802c03d3fb", 2),
      ("counter_3", "c02c03d3fa", 3),
    ]
  )
  def test_ascm_2cd_counter_decode(self, _, payload, expected_counter):
    parser = CANParser("gm_global_a_powertrain_volt", [("ASCM_2CD", 25)], 2)
    parser.update([(1_000_000_000, [(0x2CD, bytes.fromhex(payload), 2)])])
    assert parser.vl["ASCM_2CD"]["RollingCounter"] == expected_counter

  def test_uses_stock_counter_after_first_ascm_2cd(self):
    CP = SimpleNamespace(carFingerprint=CAR.CHEVROLET_TRAILBLAZER, networkLocation=NetworkLocation.fwdCamera)
    CS = SimpleNamespace(cam_ascm_2cd_counter_ts_nanos=1, cam_ascm_2cd_counter_updated=True, cam_ascm_2cd_counter=3)
    assert get_longitudinal_command_timing(CP, CS, frame=41) == (True, 3)

    CS.cam_ascm_2cd_counter_updated = False
    assert get_longitudinal_command_timing(CP, CS, frame=44) == (False, 3)

  def test_uses_original_clock_before_first_ascm_2cd(self):
    CP = SimpleNamespace(carFingerprint=CAR.CHEVROLET_TRAILBLAZER, networkLocation=NetworkLocation.fwdCamera)
    CS = SimpleNamespace(cam_ascm_2cd_counter_ts_nanos=0)
    assert get_longitudinal_command_timing(CP, CS, frame=4) == (True, 1)
    assert get_longitudinal_command_timing(CP, CS, frame=5) == (False, 1)

  @parameterized.expand([
    ("inactive", "000231790000"),
    ("unknown_bit_3", "0802b29f0000"),
    ("unknown_bit_5", "200233290000"),
  ])
  def test_inactive_acc_status_matches_stock_exactly(self, _, stock_payload_hex):
    packer = CANPacker("gm_global_a_powertrain_volt")
    parser = CANParser("gm_global_a_powertrain_volt", [("ASCMActiveCruiseControlStatus", 25)], 2)
    stock_payload = bytes.fromhex(stock_payload_hex)
    parser.update([(1_000_000_000, [(0x370, stock_payload, 2)])])

    hud_control = SimpleNamespace(leadDistanceBars=0, leadVisible=False)
    msg = create_acc_dashboard_command(packer, 0, False, 0, hud_control, False,
                                       dict(parser.vl["ASCMActiveCruiseControlStatus"]))
    assert msg[1] == stock_payload

  def test_active_acc_status_preserves_stock_protocol_state(self):
    packer = CANPacker("gm_global_a_powertrain_volt")
    parser = CANParser("gm_global_a_powertrain_volt", [("ASCMActiveCruiseControlStatus", 25)], 0)
    stock_payload = bytes.fromhex("200233290000")
    parser.update([(1_000_000_000, [(0x370, stock_payload, 0)])])
    stock_values = dict(parser.vl["ASCMActiveCruiseControlStatus"])

    hud_control = SimpleNamespace(leadDistanceBars=2, leadVisible=True)
    msg = create_acc_dashboard_command(packer, 0, True, 100, hud_control, True, stock_values)
    parser.update([(2_000_000_000, [(0x370, msg[1], 0)])])
    values = parser.vl["ASCMActiveCruiseControlStatus"]

    assert values["ACCCruiseState"] == stock_values["ACCCruiseState"] == 2
    assert values["ACCAlwaysOne"] == stock_values["ACCAlwaysOne"] == 0
    assert values["ACCAlwaysOne2"] == stock_values["ACCAlwaysOne2"] == 0
    assert values["ACCUnknownBit5"] == stock_values["ACCUnknownBit5"] == 1
    assert values["ACCCmdActive"] == 1
    assert values["ACCSpeedSetpoint"] == 100
    assert values["ACCGapLevel"] == 2
    assert values["ACCLeadCar"] == 1
    assert values["FCWAlert"] == 3
