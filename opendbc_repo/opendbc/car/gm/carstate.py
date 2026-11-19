import copy
from opendbc.can import CANDefine, CANParser
from openpilot.cereal import car
from openpilot.common.params import Params #kans
import numpy as np
from opendbc.car import Bus, create_button_events, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.gm import gmcan
from opendbc.car.gm.cluster_speed import gm_cluster_cruise_speed_from_raw_ms
from opendbc.car.gm.manual_auto_hold import ManualAutoHold, manual_hold_supported
from opendbc.car.gm.stock_fcw import StockFcw, stock_fcw_supported
from opendbc.car.gm.values import DBC, AccState, CruiseButtons, STEER_THRESHOLD, CAR, DBC, CanBus, GMFlags, CAMERA_ACC_CAR, \
  is_bolt_acc_pedal_starpilot, is_cc_only
from opendbc.car.interfaces import CarStateBase
from opendbc.safety import ALTERNATIVE_EXPERIENCE

ButtonType = structs.CarState.ButtonEvent.Type
TransmissionType = structs.CarParams.TransmissionType
NetworkLocation = structs.CarParams.NetworkLocation
GearShifter = structs.CarState.GearShifter
STANDSTILL_THRESHOLD = 10 * 0.0311 * CV.KPH_TO_MS

BUTTONS_DICT = {CruiseButtons.RES_ACCEL: ButtonType.accelCruise, CruiseButtons.DECEL_SET: ButtonType.decelCruise,
                CruiseButtons.MAIN: ButtonType.mainCruise, CruiseButtons.CANCEL: ButtonType.cancel,
                CruiseButtons.GAP_DIST: ButtonType.gapAdjustCruise}


def create_stock_long_cancel_button_events() -> list[structs.CarState.ButtonEvent]:
  """Create an atomic synthetic cancel click for a stock ACC authority loss."""
  # VCruiseCarrot tracks button duration from matching press/release edges. A
  # press-only synthetic cancel latches that tracker forever, is eventually
  # interpreted as a long press, and prevents later SET/RES button handling.
  # Keep both edges in one carState message so they cannot be lost separately.
  return [
    structs.CarState.ButtonEvent(pressed=True, type=ButtonType.cancel),
    structs.CarState.ButtonEvent(pressed=False, type=ButtonType.cancel),
  ]

# GAS_SENSOR scaling in the GM DBC (INTERCEPTOR_GAS 0.125677/-75.909, INTERCEPTOR_GAS2 0.251976/-76.601)
INTERCEPTOR_GAS_SCALE = ((0.125677, -75.909), (0.251976, -76.601))
GM_PANDA_INTERCEPTOR_GAS_THRESHOLD = 550  # safety_gm.h GM_GAS_INTERCEPTOR_THRESHOLD
BOLT_ACC_PEDAL_PANDA_GAS_THRESHOLD = 595  # safety_gm.h, Bolt ACC with pedal in StarPilot mode (bit 2048)
# Bolt ACC with pedal: a press may be held down to this many raw counts below the panda's threshold. A light foot sits
# right on 550 (rlog 00000023--f5c4a454cd--13: 535-565, the ECM's AcceleratorPedal2 1-13), so the panda-exact threshold
# flipped gasPressed every 20-120 ms; carrot took each flip as a gas tap (+10 km/h, 30 -> 140) and the panda dropped
# the gas frames sent in the gaps. The pedal rests at ~475 (450-486 in both logged segments, AcceleratorPedal2 0).
BOLT_ACC_PEDAL_GAS_RELEASE_MARGIN = 35


def interceptor_raw_average(gas, gas2):
  """The panda's (track1 + track2) / 2 on the raw counts, recovered from the scaled DBC values."""
  (scale1, offset1), (scale2, offset2) = INTERCEPTOR_GAS_SCALE
  return (round((gas - offset1) / scale1) + round((gas2 - offset2) / scale2)) // 2


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])
    self.shifter_values = can_define.dv["ECMPRDNL2"]["PRNDL2"]
    self.cluster_speed_hyst_gap = CV.KPH_TO_MS / 2.
    self.cluster_min_speed = CV.KPH_TO_MS / 2.

    self.loopback_lka_steering_cmd_updated = False
    self.loopback_lka_steering_cmd_ts_nanos = 0
    self.pt_lka_steering_cmd_counter = 0
    self.cam_lka_steering_cmd_counter = 0
    self.cam_ascm_2cb_counter = 0
    self.cam_ascm_2cb_counter_updated = False
    self.cam_ascm_2cb_counter_ts_nanos = 0
    self.cam_stock_long_active = None
    self.cam_stock_long_cancel = False
    self.cam_acc_status = None
    self.stock_fcw = StockFcw() if stock_fcw_supported(CP) else None
    self.cam_fcw_alert = 0
    # Bolt ACC with pedal: mode 1 runs camera long with the pedal only tapping to leave the ECM's ACC standstill; mode 2
    # (StarPilot) runs as a CC-only pedal-long car and replays the camera's 0x370 FCWAlert level on openpilot's 0x370.
    self.is_bolt_acc_pedal = CP.carFingerprint == CAR.CHEVROLET_BOLT_ACC_PEDAL and CP.enableGasInterceptorDEPRECATED
    self.is_bolt_starpilot = self.is_bolt_acc_pedal and is_bolt_acc_pedal_starpilot(CP)
    self.bolt_gas_pressed = False
    self.cc_only = is_cc_only(CP)
    self.stock_fcw_alert = 0
    self.is_metric = False
    self.manual_auto_hold = ManualAutoHold() if manual_hold_supported(CP) else None
    self.manual_hold_controls_active = True  # Wait for an actual CarControl before arming.
    self.brake_pedal_position = 0.

    self.buttons_counter = 0
    self.auto_stop_off = 0
    self.auto_stop_off_ts = 0
    self.button_frame_ts = 0
    self.button_park_brake = 0
    self.button_auto_stop_pressed = False
    self.buttons_idle = False
    self.auto_stop_off_zero_seen = False
    self.single_pedal_mode = False
    self.pedal_steady = 0.
    self.cruise_buttons = 0
    # GAP_DIST
    self.distance_button = 0

    # cruiseMain default(test from nd0706-vision)
    self.cruiseMain_on = True if Params().get_int("AutoEngage") == 2 else False

  def update_button_enable(self, buttonEvents: list[structs.CarState.ButtonEvent]):
    if not self.CP.pcmCruise:
      for b in buttonEvents:
        # The ECM allows enabling on falling edge of set, but only rising edge of resume
        if (b.type == ButtonType.accelCruise and b.pressed) or \
          (b.type == ButtonType.decelCruise and not b.pressed):
          return True
    return False

  def update(self, can_parsers) -> structs.CarState:
    pt_cp = can_parsers[Bus.pt]
    cam_cp = can_parsers[Bus.cam]
    loopback_cp = can_parsers[Bus.loopback]

    ret = structs.CarState()

    prev_cruise_buttons = self.cruise_buttons
    prev_distance_button = self.distance_button
    self.cruise_buttons = pt_cp.vl["ASCMSteeringButton"]["ACCButtons"]
    self.distance_button = pt_cp.vl["ASCMSteeringButton"]["DistanceButton"]
    self.buttons_counter = pt_cp.vl["ASCMSteeringButton"]["RollingCounter"]
    if self.CP.carFingerprint == CAR.CHEVROLET_TRAVERSE:
      # AutoStopOff inputs: auto stop off latch (0x1F4 byte2 bit1) and the real 0x1E1 frame's state.
      # 0x1F4 is registered with ignore-alive in get_can_parsers, so it never affects canValid. The controller
      # checks freshness of these timestamps against the current time (same monotonic clock as CAN logMonoTime).
      self.auto_stop_off = int(pt_cp.vl["SportMode"]["AutoStopOff"])
      self.auto_stop_off_ts = pt_cp.ts_nanos["SportMode"]["AutoStopOff"]
      btn = pt_cp.vl["ASCMSteeringButton"]
      self.button_frame_ts = pt_cp.ts_nanos["ASCMSteeringButton"]["ACCButtons"]
      self.button_park_brake = int(btn["ParkBrakeEngaged"])
      # Several frames can arrive in one update; look at all of them (vl_all), not just the newest (vl).
      # Our own TX comes back as src 128 and is not parsed here, so these reflect the real buttons only.
      btn_all = pt_cp.vl_all["ASCMSteeringButton"]
      self.button_auto_stop_pressed = bool(btn["AutoStopOffButton"]) or any(btn_all["AutoStopOffButton"])
      frames = list(zip(btn_all["ACCButtons"], btn_all["DistanceButton"], btn_all["LKAButton"],
                        btn_all["DriveModeButton"], btn_all["AutoStopOffButton"]))
      frames.append((btn["ACCButtons"], btn["DistanceButton"], btn["LKAButton"], btn["DriveModeButton"],
                     btn["AutoStopOffButton"]))
      self.buttons_idle = all(int(acc) == CruiseButtons.UNPRESS and not (gap or lka or mode or aso)
                              for acc, gap, lka, mode, aso in frames)
      self.auto_stop_off_zero_seen = any(int(v) == 0 for v in pt_cp.vl_all["SportMode"]["AutoStopOff"])

    self.pscm_status = copy.copy(pt_cp.vl["PSCMStatus"])
    # GAP_DIST
    if self.cruise_buttons in [CruiseButtons.UNPRESS, CruiseButtons.INIT] and self.distance_button:
      self.cruise_buttons = CruiseButtons.GAP_DIST

    if self.CP.enableBsm:
      ret.leftBlindspot = pt_cp.vl["BCMBlindSpotMonitor"]["LeftBSM"] == 1
      ret.rightBlindspot = pt_cp.vl["BCMBlindSpotMonitor"]["RightBSM"] == 1

    # Variables used for avoiding LKAS faults
    self.loopback_lka_steering_cmd_updated = len(loopback_cp.vl_all["ASCMLKASteeringCmd"]["RollingCounter"]) > 0
    if self.loopback_lka_steering_cmd_updated:
      self.loopback_lka_steering_cmd_ts_nanos = loopback_cp.ts_nanos["ASCMLKASteeringCmd"]["RollingCounter"]
    if self.CP.networkLocation == NetworkLocation.fwdCamera and not self.CP.flags & GMFlags.NO_CAMERA.value:
      self.pt_lka_steering_cmd_counter = pt_cp.vl["ASCMLKASteeringCmd"]["RollingCounter"]
      self.cam_lka_steering_cmd_counter = cam_cp.vl["ASCMLKASteeringCmd"]["RollingCounter"]

    # The stock 0x2CB counter lags ASCM_2CD by one cycle during some cold starts
    # and aligns with it after ACC activates. Follow 0x2CB itself instead.
    self.cam_ascm_2cb_counter_updated = False
    self.cam_stock_long_cancel = False
    if self.CP.openpilotLongitudinalControl and self.CP.carFingerprint == CAR.CHEVROLET_TRAILBLAZER:
      counters = cam_cp.vl_all["ASCMGasRegenCmd"]["RollingCounter"]
      active_states = cam_cp.vl_all["ASCMGasRegenCmd"]["GasRegenCmdActive"]
      self.cam_ascm_2cb_counter_updated = len(counters) > 0 and len(active_states) > 0
      if self.cam_ascm_2cb_counter_updated:
        previous_stock_long_active = self.cam_stock_long_active
        self.cam_ascm_2cb_counter = int(counters[-1])
        self.cam_ascm_2cb_counter_ts_nanos = cam_cp.ts_nanos["ASCMGasRegenCmd"]["RollingCounter"]
        self.cam_stock_long_active = bool(active_states[-1])
        self.cam_stock_long_cancel = previous_stock_long_active is True and not self.cam_stock_long_active

      # Keep the camera's complete ACC state as the template for the replacement
      # 0x370. This Trailblazer generation expects ACCCruiseState and the two
      # constant bits to retain the stock values (2, 0, 0).
      if len(cam_cp.vl_all["ASCMActiveCruiseControlStatus"]["ACCCruiseState"]) > 0:
        self.cam_acc_status = copy.copy(cam_cp.vl["ASCMActiveCruiseControlStatus"])

    # This is to avoid a fault where you engage while still moving backwards after shifting to D.
    # An Equinox has been seen with an unsupported status (3), so only check if either wheel is in reverse (2)
    left_whl_sign = -1 if pt_cp.vl["EBCMWheelSpdRear"]["RLWheelDir"] == 2 else 1
    right_whl_sign = -1 if pt_cp.vl["EBCMWheelSpdRear"]["RRWheelDir"] == 2 else 1
    ret.wheelSpeeds = self.get_wheel_speeds(
      left_whl_sign * pt_cp.vl["EBCMWheelSpdFront"]["FLWheelSpd"],
      right_whl_sign * pt_cp.vl["EBCMWheelSpdFront"]["FRWheelSpd"],
      left_whl_sign * pt_cp.vl["EBCMWheelSpdRear"]["RLWheelSpd"],
      right_whl_sign * pt_cp.vl["EBCMWheelSpdRear"]["RRWheelSpd"],
    )
    ret.vEgoRaw = float(np.mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr]))
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    # sample rear wheel speeds, standstill=True if ECM allows engagement with brake
    ret.standstill = abs(ret.wheelSpeeds.rl) <= STANDSTILL_THRESHOLD and abs(ret.wheelSpeeds.rr) <= STANDSTILL_THRESHOLD

    if pt_cp.vl["ECMPRDNL2"]["ManualMode"] == 1:
      ret.gearShifter = self.parse_gear_shifter("T")
    else:
      ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(pt_cp.vl["ECMPRDNL2"]["PRNDL2"], None))

    if self.CP.flags & GMFlags.NO_ACCELERATOR_POS_MSG.value:
      ret.brake = pt_cp.vl["EBCMBrakePedalPosition"]["BrakePedalPosition"] / 0xd0
    else:
      ret.brake = pt_cp.vl["ECMAcceleratorPos"]["BrakePedalPos"]
    self.brake_pedal_position = pt_cp.vl["ECMAcceleratorPos"]["BrakePedalPos"] if self.manual_auto_hold is not None else 0.
    if self.CP.networkLocation == NetworkLocation.fwdCamera:
      ret.brakePressed = pt_cp.vl["ECMEngineStatus"]["BrakePressed"] != 0
    else:
      # Some Volt 2016-17 have loose brake pedal push rod retainers which causes the ECM to believe
      # that the brake is being intermittently pressed without user interaction.
      # To avoid a cruise fault we need to use a conservative brake position threshold
      # https://static.nhtsa.gov/odi/tsbs/2017/MC-10137629-9999.pdf
      ret.brakePressed = ret.brake >= 10

    # Regen braking is braking
    if self.CP.transmissionType == TransmissionType.direct:
      ret.regenBraking = pt_cp.vl["EBCMRegenPaddle"]["RegenPaddle"] != 0
      self.single_pedal_mode = ret.gearShifter == GearShifter.low or pt_cp.vl["EVDriveMode"]["SinglePedalModeActive"] == 1
      if self.is_bolt_starpilot:
        self.single_pedal_mode = True  # StarPilot sets it for this car with the pedal (informational only)

    if self.CP.enableGasInterceptorDEPRECATED:
      ret.gas = (pt_cp.vl["GAS_SENSOR"]["INTERCEPTOR_GAS"] + pt_cp.vl["GAS_SENSOR"]["INTERCEPTOR_GAS2"]) / 2.
      threshold = 20 if self.CP.carFingerprint in CAMERA_ACC_CAR else 4
      ret.gasPressed = ret.gas > threshold
      if self.is_bolt_acc_pedal:
        # The panda's own decision (raw track average > 550, or StarPilot's 595 in mode 2) on the same frame is always a
        # press, so the host never releases while the panda sees one. Just under it a press is held (down to
        # BOLT_ACC_PEDAL_GAS_RELEASE_MARGIN below) only while
        #  - the panda ignores gas edges (DisengageOnAccelerator off, carrot's default): with them on it drops control
        #    on every edge of its own and the host must see the same edges, and
        #  - the ECM still reads the driver's pedal as pressed (AcceleratorPedal2, 0 with the foot off), so a pedal
        #    resting inside the band cannot hold a press forever.
        threshold = BOLT_ACC_PEDAL_PANDA_GAS_THRESHOLD if self.is_bolt_starpilot else GM_PANDA_INTERCEPTOR_GAS_THRESHOLD
        raw = interceptor_raw_average(pt_cp.vl["GAS_SENSOR"]["INTERCEPTOR_GAS"], pt_cp.vl["GAS_SENSOR"]["INTERCEPTOR_GAS2"])
        hold = (self.bolt_gas_pressed and raw > threshold - BOLT_ACC_PEDAL_GAS_RELEASE_MARGIN and
                bool(self.CP.alternativeExperience & ALTERNATIVE_EXPERIENCE.DISABLE_DISENGAGE_ON_GAS) and
                pt_cp.vl["AcceleratorPedal2"]["AcceleratorPedal2"] > 0)
        self.bolt_gas_pressed = raw > threshold or hold
        ret.gasPressed = self.bolt_gas_pressed
    else:
      ret.gas = pt_cp.vl["AcceleratorPedal2"]["AcceleratorPedal2"] / 254.
      ret.gasPressed = ret.gas > 1e-5

    ret.steeringAngleDeg = pt_cp.vl["PSCMSteeringAngle"]["SteeringWheelAngle"]
    ret.steeringRateDeg = pt_cp.vl["PSCMSteeringAngle"]["SteeringWheelRate"]
    ret.steeringTorque = pt_cp.vl["PSCMStatus"]["LKADriverAppldTrq"]
    ret.steeringTorqueEps = pt_cp.vl["PSCMStatus"]["LKATorqueDelivered"]
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD

    # 0 inactive, 1 active, 2 temporarily limited, 3 failed
    self.lkas_status = pt_cp.vl["PSCMStatus"]["LKATorqueDeliveredStatus"]
    ret.steerFaultTemporary = self.lkas_status == 2
    ret.steerFaultPermanent = self.lkas_status == 3

    # 1 - open, 0 - closed
    ret.doorOpen = (pt_cp.vl["BCMDoorBeltStatus"]["FrontLeftDoor"] == 1 or
                    pt_cp.vl["BCMDoorBeltStatus"]["FrontRightDoor"] == 1 or
                    pt_cp.vl["BCMDoorBeltStatus"]["RearLeftDoor"] == 1 or
                    pt_cp.vl["BCMDoorBeltStatus"]["RearRightDoor"] == 1)

    # 1 - latched
    ret.seatbeltUnlatched = pt_cp.vl["BCMDoorBeltStatus"]["LeftSeatBelt"] == 0
    ret.leftBlinker = pt_cp.vl["BCMTurnSignals"]["TurnSignals"] == 1
    ret.rightBlinker = pt_cp.vl["BCMTurnSignals"]["TurnSignals"] == 2

    ret.parkingBrake = pt_cp.vl["BCMGeneralPlatformStatus"]["ParkBrakeSwActive"] == 1

    ret.cruiseState.available = pt_cp.vl["ECMEngineStatus"]["CruiseMainOn"] != 0
    self.cruiseMain_on =  ret.cruiseState.available
    ret.espDisabled = pt_cp.vl["ESPStatus"]["TractionControlOn"] != 1
    ret.accFaulted = (pt_cp.vl["AcceleratorPedal2"]["CruiseState"] == AccState.FAULTED or
                      pt_cp.vl["EBCMFrictionBrakeStatus"]["FrictionBrakeUnavailable"] == 1)

    ret.cruiseState.enabled = pt_cp.vl["AcceleratorPedal2"]["CruiseState"] != AccState.OFF
    ret.cruiseState.standstill = pt_cp.vl["AcceleratorPedal2"]["CruiseState"] == AccState.STANDSTILL
    # kans: avoid to accFault
    # The Bolt ACC with pedal keeps the ECM's ACC standstill: in mode 1 the controller taps the pedal while it is set to
    # start from a stop, in mode 2 it releases StarPilot's pedal stop hold (as in StarPilot); controlsd sets
    # cruiseControl.resume from it and LongControl does not let it hold off starting.
    if self.CP.carFingerprint not in CAR.CHEVROLET_VOLT and not self.is_bolt_acc_pedal:
      ret.cruiseState.standstill = False
    if self.CP.networkLocation == NetworkLocation.fwdCamera and not self.CP.flags & GMFlags.NO_CAMERA.value:
      if not self.cc_only:
        ret.cruiseState.speed = cam_cp.vl["ASCMActiveCruiseControlStatus"]["ACCSpeedSetpoint"] * CV.KPH_TO_MS
      ret.stockAeb = False
      if self.is_bolt_starpilot:
        # StarPilot reads the camera's 0x370 on this CC-only platform: its FCW level is replayed on openpilot's 0x370
        # (the panda blocks the camera's), and AEBCmd reports the stock AEB. StarPilot also sets stockFcw, whose event
        # has no alert there; here GM's car_specific turns stockFcw into an audible FCW (meant for the Traverse), so it
        # stays False. The replayed 0x370 level is the same either way (the level is used first).
        self.stock_fcw_alert = int(cam_cp.vl["ASCMActiveCruiseControlStatus"]["FCWAlert"])
        ret.stockAeb = cam_cp.vl["AEBCmd"]["AEBCmdActive"] != 0
      # openpilot controls nonAdaptive when not pcmCruise
      if self.CP.pcmCruise and not self.cc_only:
        ret.cruiseState.nonAdaptive = cam_cp.vl["ASCMActiveCruiseControlStatus"]["ACCCruiseState"] not in (2, 3)
    if self.stock_fcw is not None:
      # The camera 0x370 is already registered by the set-speed read above; a plain
      # dict lookup never adds a parser requirement. Only the dashboard FCW uses this.
      camera_status = cam_cp.vl_all.get("ASCMActiveCruiseControlStatus")
      camera_times = cam_cp.ts_nanos.get("ASCMActiveCruiseControlStatus") or {}
      # Aged against the real boot clock, which keeps running even when no CAN event arrives.
      self.cam_fcw_alert = self.stock_fcw.update(camera_status.get("FCWAlert", ()) if camera_status else (),
                                                 camera_times.get("FCWAlert", 0))
      # 3 is the collision alert seen with the seat pulse (11/4); 1 and 2 are not confirmed as one,
      # so only 3 sets stockFcw, which car_specific turns into the existing forward-collision alert.
      ret.stockFcw = self.cam_fcw_alert == 3
    if self.cc_only:
      ret.accFaulted = False
      ret.cruiseState.speed = pt_cp.vl["ECMCruiseControl"]["CruiseSetSpeed"] * CV.KPH_TO_MS
      ret.cruiseState.enabled = pt_cp.vl["ECMCruiseControl"]["CruiseActive"] != 0
      if self.is_bolt_starpilot and not self.CP.flags & GMFlags.NO_CAMERA.value:
        # The factory camera runs this car's ACC and 0x3D1 stays 0: stock ACC is on when the camera's 0x370 says so.
        # openpilot's controller cancels it from the camera side while this is set (StarPilot carstate.py:428-433).
        ret.cruiseState.enabled = cam_cp.vl["ASCMActiveCruiseControlStatus"]["ACCCmdActive"] != 0
    prev_lkas_enabled = self.lkas_enabled
    self.lkas_enabled = pt_cp.vl["ASCMSteeringButton"]["LKAButton"]

    self.pcm_acc_status = pt_cp.vl["AcceleratorPedal2"]["CruiseState"]
    if self.CP.carFingerprint in (CAR.CHEVROLET_TRAX, CAR.CHEVROLET_TRAILBLAZER, CAR.CHEVROLET_TRAILBLAZER_CC): 
      ret.vCluRatio = 0.96
    elif self.CP.flags & GMFlags.SPEED_RELATED_MSG.value:
      # kans: use cluster speed & vCluRatio(longitudialPlanner)
      self.is_metric = Params().get_bool("IsMetric")
      speed_conv = CV.MPH_TO_MS if self.is_metric else CV.KPH_TO_MS
      cluSpeed = pt_cp.vl["SPEED_RELATED"]["ClusterSpeed"]
      ret.vEgoCluster = cluSpeed * speed_conv
      vEgoClu, aEgoClu = self.update_clu_speed_kf(ret.vEgoCluster)
      if self.CP.carFingerprint in CAR.CHEVROLET_VOLT:
        ret.vCluRatio = 1.0 #(ret.vEgo / vEgoClu) if (vEgoClu > 3. and ret.vEgo > 3.) else 1.0
      else:
        ret.vCluRatio = 0.96

    if self.CP.carFingerprint == CAR.CHEVROLET_TRAVERSE:
      # Wayon's measured display mapping, not a chassis-speed correction.
      if ret.vEgo > 0.0:
        ret.vEgoCluster = gm_cluster_cruise_speed_from_raw_ms(ret.vEgo)
      if ret.cruiseState.speed > 0.0:
        ret.cruiseState.speedCluster = gm_cluster_cruise_speed_from_raw_ms(ret.cruiseState.speed)

    # Don't add event if transitioning from INIT, unless it's to an actual button
    if self.cruise_buttons != CruiseButtons.UNPRESS or prev_cruise_buttons != CruiseButtons.INIT:
      ret.buttonEvents = [
        *create_button_events(self.cruise_buttons, prev_cruise_buttons, BUTTONS_DICT,
                              unpressed_btn=CruiseButtons.UNPRESS),
        *create_button_events(self.distance_button, prev_distance_button,
                              {1: ButtonType.gapAdjustCruise}),
        *create_button_events(self.lkas_enabled, prev_lkas_enabled,
                              {1: ButtonType.lkas})
      ]
    if self.cam_stock_long_cancel:
      # Treat the camera's active-to-inactive edge like a complete momentary
      # cancel click. The controller has already neutralized this cycle's
      # actuation; this edge also prevents openpilot from remaining visually
      # engaged with no ACC without latching the cruise-button state machine.
      ret.buttonEvents = [*ret.buttonEvents, *create_stock_long_cancel_button_events()]

    return ret

  @staticmethod
  def get_can_parsers(CP):
    pt_messages = []
    cam_messages = []
    if CP.networkLocation == NetworkLocation.fwdCamera:
      pt_messages += [
        ("ASCMLKASteeringCmd", float('nan')),
      ]
    cam_messages += gmcan.get_longitudinal_sync_messages(CP)
    if CP.transmissionType == TransmissionType.direct:
      pt_messages += [
        ("EBCMRegenPaddle", 50),
        ("EVDriveMode", float('nan')),
      ]
    if CP.carFingerprint == CAR.CHEVROLET_TRAVERSE:
      # AutoStopOff feedback only; a missing 0x1F4 must not make CAN invalid (freshness is checked by the feature)
      pt_messages += [
        ("SportMode", float('nan')),
      ]
    loopback_messages = [
      ("ASCMLKASteeringCmd", float('nan')),
    ]

    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, 0),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], cam_messages, 2),
      Bus.loopback: CANParser(DBC[CP.carFingerprint][Bus.pt], loopback_messages, 128),
    }
