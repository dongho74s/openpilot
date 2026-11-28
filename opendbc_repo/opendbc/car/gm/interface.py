#!/usr/bin/env python3
import json
import os
from openpilot.cereal import car
from math import fabs, exp
from time import monotonic_ns
from openpilot.common.params import Params
from opendbc.car import Bus, get_safety_config, get_friction, structs
from opendbc.car.common.basedir import BASEDIR
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.gm import bolt_acc_pedal
from opendbc.car.gm.carcontroller import CarController
from opendbc.car.gm.carstate import CarState
from opendbc.car.gm.radar_interface import RadarInterface, RADAR_HEADER_MSG, LAST_RADAR_MSG
from opendbc.car.gm.values import CAR, CarControllerParams, EV_CAR, CAMERA_ACC_CAR, CanBus, GMFlags, CC_ONLY_CAR, SDGM_CAR, CruiseButtons, GMSafetyFlags, BOLT_PEDAL_LONG_MODE_STARPILOT, ALT_ACCS, \
  ASCM_INT_CAR
from opendbc.car.interfaces import CarInterfaceBase, TorqueFromLateralAccelCallbackType, FRICTION_THRESHOLD, LatControlInputs, NanoFFModel, \
  CHEVROLET_STEER_FF_LINEAR

#ButtonType = structs.CarState.ButtonEvent.Type 이 두 줄도 사용되지 않습니다.
#GearShifter = structs.CarState.GearShifter
TransmissionType = structs.CarParams.TransmissionType
NetworkLocation = structs.CarParams.NetworkLocation

CAM_MSG = 0x320  # AEBCmd
                 # TODO: Is this always linked to camera presence?
ACCELERATOR_POS_MSG = 0xbe

NON_LINEAR_TORQUE_PARAMS = {
  CAR.CHEVROLET_BOLT_EUV: [2.6531724862969748, 1.0, 0.1919764879840985, 0.009054123646805178],
  # CAR.CHEVROLET_BOLT_CC: [2.6531724862969748, 1.0, 0.1919764879840985, 0.009054123646805178],
  CAR.CHEVROLET_BOLT_CC: [1.8, 1.1, 0.3, -0.045],
  CAR.CHEVROLET_BOLT_ACC_PEDAL: [1.8, 1.1, 0.3, -0.045],  # same steering as the Bolt No-ACC
  CAR.GMC_ACADIA: [4.78003305, 1.0, 0.3122, 0.05591772],
  CAR.CHEVROLET_SILVERADO: [3.29974374, 1.0, 0.25571356, 0.0465122]
}

NEURAL_PARAMS_PATH = os.path.join(BASEDIR, 'torque_data/neural_ff_weights.json')

PEDAL_MSG = 0x201
SASCM_MSG = 0x2FF  # StarPilot's SASCM_ADDRESS: seen on the powertrain bus when a SASCM is fitted

class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  def _update_manual_hold(self, c=None, now_nanos=None):
    hold = self.CS.manual_auto_hold
    if hold is not None:
      if c is not None:
        self.CS.manual_hold_controls_active = c.enabled or c.longActive
      now_nanos = monotonic_ns() if now_nanos is None else now_nanos
      signals = (("ECMAcceleratorPos", "BrakePedalPos"), ("ECMEngineStatus", "BrakePressed"),
                 ("AcceleratorPedal2", "AcceleratorPedal2"), ("EBCMWheelSpdFront", "FLWheelSpd"),
                 ("EBCMWheelSpdRear", "RLWheelSpd"), ("ECMPRDNL2", "PRNDL2"),
                 ("BCMGeneralPlatformStatus", "ParkBrakeSwActive"))
      timestamps = self.can_parsers[Bus.pt].ts_nanos
      fresh = all(0 < timestamps.get(msg, {}).get(signal, 0) <= now_nanos and
                  now_nanos - timestamps[msg][signal] <= 500_000_000 for msg, signal in signals)
      if fresh:
        hold.update(self.CS.out, self.CS.brake_pedal_position, self.CS.manual_hold_controls_active)
      else:
        hold.reset()
      self.CS.out.brakeHoldActive = hold.active

  def update(self, can_packets):
    ret = super().update(can_packets)
    self._update_manual_hold()
    return ret

  def apply(self, c, now_nanos=None, model_v2=None, radar_state=None):
    self._update_manual_hold(c, now_nanos)
    return super().apply(c, now_nanos, model_v2, radar_state)

  @staticmethod
  def get_pid_accel_limits(CP, current_speed, cruise_speed):
    if CP.carFingerprint == CAR.CHEVROLET_BOLT_ACC_PEDAL and CP.enableGasInterceptorDEPRECATED and \
       CP.flags & GMFlags.PEDAL_LONG.value:
      # StarPilot mode: full -4.0 floor (friction brake), launch-limited ceiling
      return bolt_acc_pedal.get_pid_accel_limits(current_speed)
    return CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX

  # Determined by iteratively plotting and minimizing error for f(angle, speed) = steer.
  @staticmethod
  def get_steer_feedforward_volt(desired_angle, v_ego):
    desired_angle *= 0.02904609
    sigmoid = desired_angle / (1 + fabs(desired_angle))
    return 0.10006696 * sigmoid * (v_ego + 3.12485927)

  def get_steer_feedforward_function(self):
    if self.CP.carFingerprint in (CAR.CHEVROLET_VOLT, CAR.CHEVROLET_VOLT_CC):
      return self.get_steer_feedforward_volt
    else:
      return CarInterfaceBase.get_steer_feedforward_default

  def torque_from_lateral_accel_siglin(self, latcontrol_inputs: LatControlInputs, torque_params: structs.CarParams.LateralTorqueTuning,
                                       lateral_accel_error: float, lateral_accel_deadzone: float, friction_compensation: bool, gravity_adjusted: bool) -> float:
    friction = get_friction(lateral_accel_error, lateral_accel_deadzone, FRICTION_THRESHOLD, torque_params, friction_compensation)

    def sig(val):
      # https://timvieira.github.io/blog/post/2014/02/11/exp-normalize-trick
      if val >= 0:
        return 1 / (1 + exp(-val)) - 0.5
      else:
        z = exp(val)
        return z / (1 + z) - 0.5

    # The "lat_accel vs torque" relationship is assumed to be the sum of "sigmoid + linear" curves
    # An important thing to consider is that the slope at 0 should be > 0 (ideally >1)
    # This has big effect on the stability about 0 (noise when going straight)
    # ToDo: To generalize to other GMs, explore tanh function as the nonlinear
    non_linear_torque_params = NON_LINEAR_TORQUE_PARAMS.get(self.CP.carFingerprint)
    assert non_linear_torque_params, "The params are not defined"
    a, b, c, _ = non_linear_torque_params
    steer_torque = (sig(latcontrol_inputs.lateral_acceleration * a) * b) + (latcontrol_inputs.lateral_acceleration * c)
    return float(steer_torque) + friction

  def torque_from_lateral_accel_neural(self, latcontrol_inputs: LatControlInputs, torque_params: structs.CarParams.LateralTorqueTuning,
                                       lateral_accel_error: float, lateral_accel_deadzone: float, friction_compensation: bool, gravity_adjusted: bool) -> float:
    friction = get_friction(lateral_accel_error, lateral_accel_deadzone, FRICTION_THRESHOLD, torque_params, friction_compensation)
    inputs = list(latcontrol_inputs)
    if gravity_adjusted:
      inputs[0] += inputs[1]
    return float(self.neural_ff_model.predict(inputs)) + friction

  def torque_from_lateral_accel(self) -> TorqueFromLateralAccelCallbackType:
    # ChevroletSteerFF 3 (Volt linear): plain linear torque, not comma's NN or the sigmoid fits of the Bolts/Silverado
    if getattr(self, "chevrolet_steer_ff", 0) == CHEVROLET_STEER_FF_LINEAR:
      return self.torque_from_lateral_accel_linear
    with open(NEURAL_PARAMS_PATH) as f:
      neural_ff_cars = json.load(f).keys()
    if self.CP.carFingerprint in neural_ff_cars:
      self.neural_ff_model = NanoFFModel(NEURAL_PARAMS_PATH, self.CP.carFingerprint)
      return self.torque_from_lateral_accel_neural
    elif self.CP.carFingerprint in NON_LINEAR_TORQUE_PARAMS:
      return self.torque_from_lateral_accel_siglin
    else:
      return self.torque_from_lateral_accel_linear

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "gm"
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.gm)]
    ret.autoResumeSng = False
    ret.enableBsm = 0x142 in fingerprint[CanBus.POWERTRAIN] or 0x142 in fingerprint[CanBus.CAMERA]
    ret.startAccel = 1.0
    ret.radarTimeStep = 0.067
    ret.alternativeExperience = 0

    useEVTables = Params().get_bool("EVTable")

    # The Malibu ASCM harness keeps the stock ACC only; a 0x201 frame on its bus must not switch it to pedal longitudinal
    if PEDAL_MSG in fingerprint[0] and candidate not in ASCM_INT_CAR:
      ret.enableGasInterceptorDEPRECATED = True
      ret.safetyConfigs[0].safetyParam |= GMSafetyFlags.GAS_INTERCEPTOR.value
    # Bolt ACC with a comma pedal: BoltPedalLongMode 2 runs StarPilot's pedal long (as a CC-only pedal car, never camera
    # long); anything else (default 1) runs the Bolt EUV's camera long with the pedal only to start from a stop.
    bolt_acc_pedal_long = candidate == CAR.CHEVROLET_BOLT_ACC_PEDAL and ret.enableGasInterceptorDEPRECATED and \
                          Params().get_int("BoltPedalLongMode") == BOLT_PEDAL_LONG_MODE_STARPILOT
    cc_only = candidate in CC_ONLY_CAR or bolt_acc_pedal_long

    if candidate in EV_CAR:
      ret.transmissionType = TransmissionType.direct
    else:
      ret.transmissionType = TransmissionType.automatic

    ret.longitudinalTuning.kpBP = [0.]
    ret.longitudinalTuning.kiBP = [0.]

    if candidate in (CAMERA_ACC_CAR | SDGM_CAR):
      # Traverse already uses the camera-long controller; keep its opt-in across drives.
      ret.alphaLongitudinalAvailable = (candidate not in SDGM_CAR or candidate == CAR.CHEVROLET_TRAVERSE) and candidate not in ALT_ACCS and \
                                       not bolt_acc_pedal_long
      ret.networkLocation = NetworkLocation.fwdCamera
      # Some Traverse harnesses expose the full stock radar stream on bus 1.
      ret.radarUnavailable = not (
        candidate == CAR.CHEVROLET_TRAVERSE and
        all(fingerprint[CanBus.OBSTACLE].get(msg) == 8 for msg in range(RADAR_HEADER_MSG, LAST_RADAR_MSG + 1))
      )
      ret.pcmCruise = True
      ret.safetyConfigs[0].safetyParam |= GMSafetyFlags.HW_CAM.value
      ret.minEnableSpeed = -1 * CV.KPH_TO_MS
      ret.minSteerSpeed = 10 * CV.KPH_TO_MS

      # Tuning for experimental long
      ret.longitudinalTuning.kiV = [1.7]
      ret.stoppingDecelRate = 2.0  # reach brake quickly after enabling
      ret.vEgoStopping = 0.5
      ret.vEgoStarting = 0.4
      ret.stopAccel = -0.4
      ret.startingState = True
      ret.startAccel = 1.0

      if alpha_long and ret.alphaLongitudinalAvailable:
        ret.pcmCruise = False
        ret.openpilotLongitudinalControl = True
        ret.safetyConfigs[0].safetyParam |= GMSafetyFlags.HW_CAM_LONG.value
        if candidate == CAR.CHEVROLET_TRAVERSE:
          ret.safetyConfigs[0].safetyParam |= GMSafetyFlags.HW_SDGM.value
          if not ret.enableGasInterceptorDEPRECATED:
            ret.safetyConfigs[0].safetyParam |= GMSafetyFlags.MANUAL_AUTO_HOLD.value

      if candidate in ALT_ACCS:
        ret.alphaLongitudinalAvailable = False
        ret.openpilotLongitudinalControl = False
        ret.minEnableSpeed = -1.  # engage speed is decided by PCM

    elif candidate in ASCM_INT_CAR:
      # StarPilot ASCM_INT (interface.py, stable 8589b73f): the comma sits between the ASCM and the powertrain bus. The
      # ASCM stays alive and keeps the stock ACC and radar; the panda forwards both ways like a camera harness except the
      # ASCM's LKAS command (0x180), which openpilot replaces.
      # With a SASCM (0x2FF on the powertrain bus) StarPilot also offers openpilot longitudinal here (alpha long,
      # user request 2026-11-21: the friend's Malibu has one). openpilot then sends gas/regen (0x2CB) and the friction
      # brake (0x315, which the SASCM relays to the EBCM) on the powertrain bus with the ACC status (0x370), and the panda
      # blocks the ASCM's own 0x2CB/0x315/0x370. The ASCM stays alive and keeps running the radar.
      has_sascm = SASCM_MSG in fingerprint[CanBus.POWERTRAIN]
      ret.alphaLongitudinalAvailable = has_sascm
      ret.networkLocation = NetworkLocation.fwdCamera
      # The ASCM runs the radar; read it only when the whole stock radar stream is on bus 1.
      ret.radarUnavailable = not all(fingerprint[CanBus.OBSTACLE].get(msg) == 8
                                     for msg in range(RADAR_HEADER_MSG, LAST_RADAR_MSG + 1))
      ret.pcmCruise = True
      ret.minEnableSpeed = 5 * CV.KPH_TO_MS
      ret.minSteerSpeed = 7 * CV.MPH_TO_MS
      ret.safetyConfigs[0].safetyParam |= GMSafetyFlags.HW_CAM.value | GMSafetyFlags.HW_ASCM_INT.value
      if ACCELERATOR_POS_MSG not in fingerprint[CanBus.POWERTRAIN]:
        # No 0xBE to read the brake from: the brake switch in 0xC9, as StarPilot does with FORCE_BRAKE_C9
        ret.safetyConfigs[0].safetyParam |= GMSafetyFlags.ASCM_INT_C9_BRAKE.value
      if alpha_long and has_sascm:
        ret.pcmCruise = False
        ret.openpilotLongitudinalControl = True
        ret.safetyConfigs[0].safetyParam |= GMSafetyFlags.HW_CAM_LONG.value
        # The same car's tuning where openpilot replaces the ASCM (CHEVROLET_MALIBU, gateway branch). Gas/regen limits
        # are the ASCM ones (CarControllerParams, panda GM_ASCM_LONG_LIMITS): the ASCM's 0x2CB uses the same layout.
        ret.longitudinalTuning.kpV = [1.0]
        ret.longitudinalTuning.kiV = [0.3]

    else:  # ASCM, OBD-II harness
      ret.openpilotLongitudinalControl = True
      ret.networkLocation = NetworkLocation.gateway
      ret.radarUnavailable = False # kans
      ret.pcmCruise = False  # stock non-adaptive cruise control is kept off
      # supports stop and go, but initial engage must (conservatively) be above 18mph
      ret.minEnableSpeed = -1 * CV.MPH_TO_MS
      ret.minSteerSpeed = (6.7 if useEVTables else 7) * CV.MPH_TO_MS

      # Tuning
      ret.longitudinalTuning.kpV = [1.0]
      ret.longitudinalTuning.kiV = [0.3]

      if ret.enableGasInterceptorDEPRECATED:
        # Need to set ASCM long limits when using pedal interceptor, instead of camera ACC long limits
        ret.safetyConfigs[0].safetyParam |= GMSafetyFlags.HW_ASCM_LONG.value

    # These cars have been put into dashcam only due to both a lack of users and test coverage.
    # These cars likely still work fine. Once a user confirms each car works and a test route is
    # added to opendbc/car/tests/routes.py, we can remove it from this list.
    # ret.dashcamOnly = candidate in {CAR.CADILLAC_ATS, CAR.HOLDEN_ASTRA, CAR.CHEVROLET_MALIBU, CAR.BUICK_REGAL} or \
    #                   (ret.networkLocation == NetworkLocation.gateway and ret.radarUnavailable)

    # Start with a baseline tuning for all GM vehicles. Override tuning as needed in each model section below.
    ret.steerActuatorDelay = 0.28  # Default delay, not measured yet

    ret.steerLimitTimer = 0.4
    ret.longitudinalActuatorDelay = Params().get_float("LongActuatorDelay")*0.01 # 0.5  # large delay to initially start braking

    if candidate == CAR.CHEVROLET_VOLT:
      ret.steerActuatorDelay = 0.45 if useEVTables else 0.3
      ret.longitudinalTuning.kpBP = [0.]
      ret.longitudinalTuning.kpV = [1.0]
      ret.longitudinalTuning.kiBP = [0.]
      ret.longitudinalTuning.kiV = [.35]
      ret.longitudinalTuning.kf = 1.0
      ret.stoppingDecelRate = 0.2 # brake_travel/s while trying to stop
      ret.vEgoStopping = 0.25
      ret.vEgoStarting = 0.15
      ret.stopAccel = -0.5
      ret.startingState = True
      ret.startAccel = 1.9

      # softer long tune for ev table
      if useEVTables:
        ret.longitudinalTuning.kpBP = [0.]
        ret.longitudinalTuning.kpV = [1.0]
        ret.longitudinalTuning.kiBP = [0.]
        ret.longitudinalTuning.kiV = [.35]
        ret.longitudinalTuning.kf = 1.0
        ret.stoppingDecelRate = 1.0 # brake_travel/s while trying to stop
        ret.stopAccel = -0.5
        ret.startAccel = 0.6

      useTorque = Params().get_bool("LateralTorqueCustom")
      if useTorque:
        CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
      else:
        ret.lateralTuning.pid.kpBP = [0., 40.]
        ret.lateralTuning.pid.kpV = [0., 0.17]
        ret.lateralTuning.pid.kiBP = [0.]
        ret.lateralTuning.pid.kiV = [0.]
        ret.lateralTuning.pid.kf = 1.

    elif candidate == CAR.CHEVROLET_VOLT_CC:
      # No lateral tuning used to be set here, so with NNFF off the Volt CC got an empty PID and lateral control raised
      # "array of sample points is empty" once engaged (Codex review 2026-11-19). Torque tuning, as on the Traverse.
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate == CAR.CADILLAC_CT6_ACC:
      ret.steerActuatorDelay = 0.3
      ret.longitudinalTuning.kpBP = [0.]
      ret.longitudinalTuning.kpV = [1.0]
      ret.longitudinalTuning.kiBP = [0.]
      ret.longitudinalTuning.kiV = [.3]
      ret.longitudinalTuning.kf = 1.0
      ret.stoppingDecelRate = 0.2 # brake_travel/s while trying to stop
      ret.stopAccel = -0.5
      ret.startingState = True
      ret.startAccel = 1.5

      useTorque = Params().get_bool("LateralTorqueCustom")
      if useTorque:
        CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
      else:
        ret.lateralTuning.pid.kpBP = [0., 40.]
        ret.lateralTuning.pid.kpV = [0., 0.17]
        ret.lateralTuning.pid.kiBP = [0.]
        ret.lateralTuning.pid.kiV = [0.]
        ret.lateralTuning.pid.kf = 1.

    elif candidate == CAR.GMC_ACADIA:
      ret.minEnableSpeed = -1.  # engage speed is decided by pcm
      ret.steerActuatorDelay = 0.2
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate in (CAR.CHEVROLET_MALIBU, CAR.CHEVROLET_MALIBU_CC, CAR.CHEVROLET_MALIBU_ASCM):
      ret.steerActuatorDelay = 0.2
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate == CAR.BUICK_LACROSSE:
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate == CAR.CADILLAC_ESCALADE:
      ret.minEnableSpeed = -1.  # engage speed is decided by pcm
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate in (CAR.CADILLAC_ESCALADE_ESV, CAR.CADILLAC_ESCALADE_ESV_2019):
      ret.minEnableSpeed = -1.  # engage speed is decided by pcm

      if candidate == CAR.CADILLAC_ESCALADE_ESV:
        ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[10., 41.0], [10., 41.0]]
        ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.13, 0.24], [0.01, 0.02]]
        ret.lateralTuning.pid.kf = 0.000045
      else:
        ret.steerActuatorDelay = 0.2
        CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate in (CAR.CHEVROLET_BOLT_EUV, CAR.CHEVROLET_BOLT_CC, CAR.CHEVROLET_BOLT_ACC_PEDAL):
      ret.steerActuatorDelay = 0.2
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

      if ret.enableGasInterceptorDEPRECATED and (candidate != CAR.CHEVROLET_BOLT_ACC_PEDAL or bolt_acc_pedal_long):
        # ACC Bolts use pedal for full longitudinal control, not just sng
        ret.flags |= GMFlags.PEDAL_LONG.value
      # The Bolt ACC with pedal in mode 1 drives through the ECM's ACC (camera long) and uses the pedal only to leave its
      # standstill, so it does not take PEDAL_LONG, which makes the controller cancel the ECM's ACC whenever it is on.

      if candidate in (CAR.CHEVROLET_BOLT_EUV, CAR.CHEVROLET_BOLT_ACC_PEDAL) and ret.openpilotLongitudinalControl and \
         not ret.enableGasInterceptorDEPRECATED:
        # The same car and ECM as the Bolt ACC with pedal in mode 1, without the pedal (user request 2026-11-28: its
        # mode 1 fixes, the pedal parts aside); the pedal block below sets these for mode 1.
        # - The ECM allows its ACC only above 5 km/h or at a stop with the brake pressed: engaging stopped without the
        #   brake faulted it 0.5 s later (rlog 00000021--282be5cb94--1, 2026-11-19). StarPilot also keeps 5 km/h for
        #   this car on camera long. GM's belowEngageSpeed then blocks such an engage unless stopped with brake >= 20.
        # - Starts like mode 1 (StarPilot's start for this car): 1.0 held 0x2CB gas 673 for 0.7 s and the car then
        #   jumped at 1.4 m/s^2 (rlog 00000028--02c4808d1b--21, 2026-11-20); LongControl also follows a slow plan
        #   behind a lead (longcontrol.py BOLT_MODE_1_PLAN_START_MAX_ACCEL).
        ret.minEnableSpeed = 5 * CV.KPH_TO_MS
        ret.startAccel = 0.55
        ret.vEgoStopping = 0.25
        ret.vEgoStarting = 0.25

    elif candidate == CAR.CHEVROLET_SILVERADO:
      # On the Bolt, the ECM and camera independently check that you are either above 5 kph or at a stop
      # with foot on brake to allow engagement, but this platform only has that check in the camera.
      # TODO: check if this is split by EV/ICE with more platforms in the future
      if ret.openpilotLongitudinalControl:
        ret.minEnableSpeed = -1.
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate in (CAR.CHEVROLET_EQUINOX, CAR.CHEVROLET_EQUINOX_CC):
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate in (CAR.CHEVROLET_TRAILBLAZER, CAR.CHEVROLET_TRAILBLAZER_CC):
      ret.steerActuatorDelay = 0.2
      # The Trailblazer ECM faults if camera-long is initially engaged below
      # 5 km/h without sufficient brake input. GM's event handling still
      # permits engagement at a true standstill while the brake is held.
      if candidate == CAR.CHEVROLET_TRAILBLAZER and ret.openpilotLongitudinalControl:
        ret.minEnableSpeed = 5 * CV.KPH_TO_MS
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate in (CAR.CHEVROLET_SUBURBAN, CAR.CHEVROLET_SUBURBAN_CC):
      ret.steerActuatorDelay = 0.075
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate == CAR.GMC_YUKON_CC:
      ret.steerActuatorDelay = 0.2
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate == CAR.CADILLAC_XT4:
      ret.steerActuatorDelay = 0.2
      ret.minEnableSpeed = -1.  # engage speed is decided by pcm
      ret.minSteerSpeed = 30 * CV.MPH_TO_MS
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
    elif candidate == CAR.CHEVROLET_VOLT_2019:
      ret.steerActuatorDelay = 0.2
      ret.minEnableSpeed = -1.  # engage speed is decided by pcm
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate == CAR.CADILLAC_XT5_CC:
      ret.steerActuatorDelay = 0.2
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate == CAR.CHEVROLET_TRAVERSE:
      ret.steerActuatorDelay = 0.2
      ret.minEnableSpeed = -1.  # engage speed is decided by pcm
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
      ret.startAccel = 2.0

    elif candidate == CAR.BUICK_BABYENCLAVE:
      ret.steerActuatorDelay = 0.2
      ret.minEnableSpeed = -1.  # engage speed is decided by pcm
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate == CAR.CADILLAC_CT6_CC:
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate == CAR.CHEVROLET_MALIBU_CC:
      ret.steerActuatorDelay = 0.2
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate == CAR.CHEVROLET_TRAX:
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
      ret.stoppingDecelRate = 0.3
      ret.minEnableSpeed = -1.
      ret.stopAccel = -0.5
      ret.startingState = True
      ret.startAccel = 1.0
    elif candidate == CAR.CHEVROLET_TRAVERSE:
      ret.steerActuatorDelay = 0.2
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate == CAR.GMC_YUKON:
      ret.steerActuatorDelay = 0.5
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
      ret.dashcamOnly = True  # Needs steerRatio, tireStiffness, and lat accel factor tuning

    if ret.enableGasInterceptorDEPRECATED:
      ret.networkLocation = NetworkLocation.fwdCamera
      ret.safetyConfigs[0].safetyParam |= GMSafetyFlags.HW_CAM.value
      ret.minEnableSpeed = -1
      if candidate == CAR.CHEVROLET_BOLT_ACC_PEDAL and not bolt_acc_pedal_long:
        # Mode 1 drives through the ECM's ACC, and the Bolt's ECM and camera allow it only above 5 km/h or at a stop
        # with the brake pressed (see the Silverado note below). Engaging stopped without the brake made the ECM fault
        # 0.5 s later (rlog 00000021--282be5cb94--1, 2026-11-19, twice, behind a lead in one-pedal hold). GM's
        # belowEngageSpeed then blocks such an engage unless stopped with brake >= 20, as on the stock ACC.
        ret.minEnableSpeed = 5 * CV.KPH_TO_MS
      ret.pcmCruise = False
      ret.openpilotLongitudinalControl = True
      ret.autoResumeSng = True

      if cc_only:
        ret.flags |= GMFlags.PEDAL_LONG.value
        ret.safetyConfigs[0].safetyParam |= GMSafetyFlags.PEDAL_LONG.value
        # Note: Low speed, stop and go not tested. Should be fairly smooth on highway
        ret.longitudinalTuning.kpBP = [0., 3., 6., 35.]
        ret.longitudinalTuning.kpV = [0.08, 0.175, 0.225, 0.33]
        ret.longitudinalTuning.kiBP = [0., 35.0]
        ret.longitudinalTuning.kiV = [0.07, 0.07]
        ret.longitudinalTuning.kf = 0.25
        ret.stoppingDecelRate = 0.8
      else:  # Pedal used for SNG, ACC for longitudinal control otherwise
        ret.safetyConfigs[0].safetyParam |= GMSafetyFlags.HW_CAM_LONG.value
        ret.startingState = True
        ret.vEgoStopping = 0.25
        ret.vEgoStarting = 0.25
        if candidate == CAR.CHEVROLET_BOLT_ACC_PEDAL:
          # StarPilot's start for this car (as in mode 2 below): 1.0 held gas 673 for 0.7 s while the ECM let go of its
          # hold, and the car then jumped at 1.4 m/s^2 (rlog 00000028--02c4808d1b--21, 2026-11-20). LongControl also
          # follows a slow plan behind a lead here (longcontrol.py BOLT_MODE_1_PLAN_START_MAX_ACCEL).
          ret.startAccel = 0.55

    elif candidate in CC_ONLY_CAR:
      ret.flags |= GMFlags.CC_LONG.value
      ret.safetyConfigs[0].safetyParam |= GMSafetyFlags.CC_LONG.value
      if alpha_long:
        ret.openpilotLongitudinalControl = True
        ret.flags |= GMFlags.CC_LONG.value
      ret.radarUnavailable = True
      ret.alphaLongitudinalAvailable = True
      ret.minEnableSpeed = 24 * CV.MPH_TO_MS
      ret.pcmCruise = True

      ret.stoppingDecelRate = 11.18  # == 25 mph/s (.04 rate)

      ret.longitudinalTuning.kiBP = [10.7, 10.8, 28.]
      ret.longitudinalTuning.kiV = [0., 20., 20.]  # set lower end to 0 since we can't drive below that speed

    if bolt_acc_pedal_long:
      # StarPilot CHEVROLET_BOLT_ACC_2022_2023_PEDAL (interface.py:588-631, BOLT_PEDAL_LONG_CARS + is_bolt_2022_2023_pedal).
      # LongControl uses kf as the feedforward gain, with this car's own adjustments (longcontrol_bolt_acc_pedal.py).
      # LongTuning* Params only override single-breakpoint tunings, so they leave this one alone.
      ret.longitudinalTuning.kpBP = [0.0, 5.0, 15.0, 35.0]
      ret.longitudinalTuning.kpV = [0.095, 0.085, 0.065, 0.050]
      ret.longitudinalTuning.kiBP = [0.0, 3.0, 6.0, 35.0]
      ret.longitudinalTuning.kiV = [0.07, 0.10, 0.15, 0.24]
      ret.longitudinalTuning.kf = 0.20
      # StarPilot's delay; carrot's planner and modeld take it instead of the LongActuatorDelay Param for this car
      # (bolt_acc_pedal_long_actuator_delay in longcontrol_bolt_acc_pedal.py)
      ret.longitudinalActuatorDelay = 0.6
      ret.startingState = True
      ret.startAccel = 0.55
      ret.vEgoStarting = 0.35
      ret.vEgoStopping = 0.25
      ret.stopAccel = -0.25
      ret.stoppingDecelRate = 0.8
      if ret.openpilotLongitudinalControl and ret.flags & GMFlags.PEDAL_LONG.value:
        ret.safetyConfigs[0].safetyParam |= GMSafetyFlags.BOLT_ACC_PEDAL.value

    if cc_only:
      ret.safetyConfigs[0].safetyParam |= GMSafetyFlags.NO_ACC.value

    # Exception for flashed cars, or cars whose camera was removed. Behind an ASCM the LKAS sender on bus 2 is the ASCM,
    # which need not send AEBCmd (as in StarPilot, ASCM_INT never takes NO_CAMERA).
    if (ret.networkLocation == NetworkLocation.fwdCamera or cc_only) and CAM_MSG not in fingerprint[
      CanBus.CAMERA] and not candidate in SDGM_CAR and candidate not in ASCM_INT_CAR:
      ret.flags |= GMFlags.NO_CAMERA.value
      ret.safetyConfigs[0].safetyParam |= GMSafetyFlags.NO_CAMERA.value

    if ACCELERATOR_POS_MSG not in fingerprint[CanBus.POWERTRAIN]:
      ret.flags |= GMFlags.NO_ACCELERATOR_POS_MSG.value

    if 608 in fingerprint[CanBus.POWERTRAIN]:
      ret.flags |= GMFlags.SPEED_RELATED_MSG.value


    return ret
