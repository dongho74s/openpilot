"""Bolt ACC with a comma pedal (CAR.CHEVROLET_BOLT_ACC_PEDAL): StarPilot's car-side longitudinal control.

Ported from firestar5683/StarPilot 2589caad, CHEVROLET_BOLT_ACC_2022_2023_PEDAL (opendbc/car/gm/carcontroller.py and
values.py). The pedal (0x200) accelerates, a spoofed regen paddle (0xBD with 0x1F5) and the EBCM friction brake (0x315)
decelerate, openpilot sends its own 0x370 to the cluster, and the camera's stock ACC is cancelled from the camera side
(0x1E1 CANCEL on bus 2). Function bodies are StarPilot's, with only the names of the constants changed; StarPilot
features that are off by default or never reach this car (Volt one-pedal and auto hold, truck smoothing, testing
grounds, maneuver mode) are left out.

The panda (BOLT_ACC_PEDAL safety bit) rejects the host's 0xBD/0x1F5 frames and resends the latest one right after each
stock frame, so they show up as rejected in logs; they are still fed every 25 Hz frame as StarPilot does.
"""
import math

import numpy as np

from opendbc.car import DT_CTRL, create_gas_interceptor_command, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.gm import gmcan
from opendbc.car.gm.values import CAR, CanBus, CruiseButtons, GMFlags

LongCtrlState = structs.CarControl.Actuators.LongControlState
VisualAlert = structs.CarControl.HUDControl.VisualAlert
NetworkLocation = structs.CarParams.NetworkLocation

ACCELERATION_DUE_TO_GRAVITY = 9.81  # StarPilot opendbc.car.ACCELERATION_DUE_TO_GRAVITY


class BoltAccPedalParams:
  """StarPilot's CarControllerParams values for this car (values.py: not kaofui, CC-only, EV, so the last branch).
  carrot's CarControllerParams uses a different gas scale (ZERO_GAS 0), so the torque model keeps StarPilot's."""
  ACCEL_MAX = 2.  # m/s^2
  ACCEL_MIN = -4.  # m/s^2
  NEAR_STOP_BRAKE_PHASE = 0.25  # m/s
  SNG_INTERCEPTOR_GAS = 18. / 255.
  CAMERA_KEEPALIVE_STEP = 100

  ZERO_GAS = 6150  # Coasting
  MAX_BRAKE = 400  # ~ -4.0 m/s^2 with regen
  MAX_GAS = 7168
  MAX_ACC_REGEN = 5500
  INACTIVE_REGEN = 5500
  BRAKE_SWITCH_MAX = MAX_ACC_REGEN  # EV
  BRAKE_LOOKUP_BP = [ACCEL_MIN, 0.]
  BRAKE_LOOKUP_V = [MAX_BRAKE, 0.]
  BRAKE_SWITCH_LOOKUP_BP = [0.5, 10]
  BRAKE_SWITCH_LOOKUP_V = [ZERO_GAS, BRAKE_SWITCH_MAX]


BOLT_ACC_PEDAL_FRICTION_RELEASE_FRAMES = 5
BOLT_PEDAL_LONG_ACCEL_LIMIT_BP = [0.0, 1.5, 4.0, 8.0, 15.0, 30.0]
BOLT_PEDAL_LONG_ACCEL_LIMIT_V = [-0.93, -1.28, -1.98, -2.58, -2.86, -2.95]
MAX_UPHILL_GRADE_FF = 0.20

# StarPilot interface.py get_pid_accel_limits for this car
PID_ACCEL_MAX_BP = [0.0, 1.5, 4.0, 8.0, 15.0]
PID_ACCEL_MAX_V = [0.54, 0.74, 1.03, 1.46, BoltAccPedalParams.ACCEL_MAX]


def supports_bolt_acc_pedal_friction_experiment(CP) -> bool:
  return (
    CP.carFingerprint == CAR.CHEVROLET_BOLT_ACC_PEDAL and
    CP.openpilotLongitudinalControl and
    CP.enableGasInterceptorDEPRECATED and
    bool(CP.flags & GMFlags.PEDAL_LONG.value)
  )


def get_pid_accel_limits(current_speed):
  return BoltAccPedalParams.ACCEL_MIN, float(np.interp(current_speed, PID_ACCEL_MAX_BP, PID_ACCEL_MAX_V))


def limit_grade_feedforward(planner_accel: float, pitch_accel: float) -> float:
  if pitch_accel > 0.0 and planner_accel > 0.0:
    return 0.0
  if pitch_accel > MAX_UPHILL_GRADE_FF:
    return MAX_UPHILL_GRADE_FF
  return pitch_accel


def use_interceptor_sng_launch(CP, CS, maneuver_mode=False):
  # Restrict the fixed standstill-launch gas to actual near-zero motion
  # so higher accel requests can take over once the car has started moving.
  launch_speed = max(CP.vEgoStarting, 0.3)
  if maneuver_mode:
    launch_speed = max(launch_speed, 2.0)
  near_stop = CS.out.standstill or CS.out.vEgo < launch_speed
  if (
    getattr(CP, "carFingerprint", None) == CAR.CHEVROLET_BOLT_ACC_PEDAL and
    getattr(CP, "enableGasInterceptorDEPRECATED", False)
  ):
    return near_stop
  return CS.out.cruiseState.standstill and near_stop


def get_acc_dashboard_fcw_alert(hud_alert, CS):
  if hud_alert == VisualAlert.fcw:
    return 0x3

  stock_fcw_alert = int(getattr(CS, "stock_fcw_alert", 0)) & 0x3
  if stock_fcw_alert != 0:
    return stock_fcw_alert

  cs_out = getattr(CS, "out", None)
  if cs_out is not None and (getattr(cs_out, "stockAeb", False) or getattr(cs_out, "stockFcw", False)):
    return 0x3

  return 0


def update_bolt_acc_pedal_stop_hold(CP, CC, CS, holding: bool) -> bool:
  return (
    supports_bolt_acc_pedal_friction_experiment(CP) and
    CC.longActive and
    CC.actuators.longControlState == LongCtrlState.stopping and
    not CC.cruiseControl.resume and
    not CS.out.gasPressed and
    not CS.out.brakePressed and
    (holding or CS.out.standstill)
  )


def get_bolt_acc_pedal_friction_brake(apply_brake, full_brake_accel, v_ego, params) -> int:
  if apply_brake <= 0:
    return 0

  full_brake_accel = min(full_brake_accel, -0.1)
  legacy_full_scale = max(-params.ACCEL_MIN, 0.1)
  corrected_scale = legacy_full_scale / max(-full_brake_accel, 0.1)
  speed_gain = float(np.interp(v_ego, [0.0, 8.0, 15.0, 25.0], [1.0, 1.08, 1.2, 1.35]))
  onset_gain = float(np.interp(
    apply_brake,
    [0.0, 5.0, 20.0, 60.0, 120.0, 240.0, params.MAX_BRAKE],
    [0.0, 1.8, 1.65, 1.4, 1.22, 1.08, 1.0],
  ))

  shaped_brake = apply_brake * corrected_scale * speed_gain * onset_gain
  minimum_brake = float(np.interp(v_ego, [0.0, 6.0, 8.0, 12.0, 18.0, 25.0], [0.0, 0.0, 4.0, 10.0, 20.0, 28.0]))
  shaped_brake = max(shaped_brake, minimum_brake)

  return int(round(np.clip(shaped_brake, 0, params.MAX_BRAKE)))


def shape_bolt_acc_pedal_low_speed_friction(apply_brake: int, v_ego: float, stopping: bool, active: bool):
  if apply_brake <= 0:
    return 0, False

  engage_threshold = float(np.interp(v_ego, [0.0, 1.5, 3.0, 5.0, 8.0], [40.0, 20.0, 12.0, 10.0, 0.0]))
  release_threshold = float(np.interp(v_ego, [0.0, 1.5, 3.0, 5.0, 8.0], [0.0, 8.0, 6.0, 4.0, 0.0]))

  if not active:
    if apply_brake < engage_threshold:
      return 0, False
    active = True
  elif apply_brake < release_threshold:
    return 0, False

  if stopping:
    stop_fade = float(np.interp(v_ego, [0.0, 0.6, 0.9, 1.2, 1.8, 2.8], [0.0, 0.0, 0.05, 0.12, 0.32, 0.78]))
    apply_brake = int(round(apply_brake * stop_fade))
    if apply_brake <= 0 or apply_brake < release_threshold:
      return 0, False

  return apply_brake, active


def get_bolt_pedal_long_accel_limit(v_ego: float) -> float:
  return float(np.interp(v_ego, BOLT_PEDAL_LONG_ACCEL_LIMIT_BP, BOLT_PEDAL_LONG_ACCEL_LIMIT_V))


def get_bolt_acc_pedal_planner_brake_switch(v_ego: float, params, tire_radius: float, mass: float,
                                            coeff_drag: float, frontal_area: float, air_density: float) -> int:
  planner_accel_limit = get_bolt_pedal_long_accel_limit(v_ego)
  aero_drag_force = 0.5 * coeff_drag * frontal_area * air_density * v_ego ** 2
  planner_torque = tire_radius * ((mass * planner_accel_limit) + aero_drag_force)
  return int(round(planner_torque + params.ZERO_GAS))


def get_bolt_acc_pedal_effective_brake_switch(stock_switch: int, planner_switch: int) -> int:
  return max(stock_switch, planner_switch)


def get_bolt_acc_pedal_friction_command_state(apply_brake: int, cruise_main_on: bool, release_frames: int):
  command_brake = apply_brake if cruise_main_on else 0

  if command_brake > 0:
    release_frames = BOLT_ACC_PEDAL_FRICTION_RELEASE_FRAMES
  elif release_frames > 0:
    release_frames -= 1

  should_send = cruise_main_on or release_frames > 0
  return command_brake, release_frames, should_send


def get_interceptor_sng_gas_cmd(CP, interceptor_gas_cmd: float, accel: float, params, maneuver_mode: bool) -> float:
  if maneuver_mode:
    return max(interceptor_gas_cmd, float(np.interp(accel, [0.0, 1.0, 2.0], [params.SNG_INTERCEPTOR_GAS, 0.11, 0.16])))

  if supports_bolt_acc_pedal_friction_experiment(CP):
    return max(interceptor_gas_cmd, params.SNG_INTERCEPTOR_GAS)

  return params.SNG_INTERCEPTOR_GAS


class BoltAccPedalLongController:
  """StarPilot CarController state and per-frame longitudinal flow for this car (carcontroller.py:519-1319)."""

  def __init__(self, CP, packer_pt, packer_ch):
    self.CP = CP
    self.packer_pt = packer_pt
    self.packer_ch = packer_ch
    self.params = BoltAccPedalParams()

    self.apply_gas = 0
    self.apply_brake = 0
    self.last_button_frame = 0
    self.pedal_steady = 0.
    self.regen_paddle_pressed = False
    self.regen_paddle_timer = 0
    self.regen_press_counter = 0
    self.regen_release_counter = 0
    self.regen_min_on_frames = 0
    self.regen_min_off_frames = 0
    self.planner_regen_hold = False
    self.paddle_handoff_frames = 0
    self.pedal_active_last = False
    self.aego = 0.0
    # StarPilot's LongitudinalManeuverPaddleMode Param (default "auto"); carrot has no such Param.
    self.maneuver_paddle_mode = "auto"

    self.mass = CP.mass
    self.tireRadius = 0.075 * CP.wheelbase + 0.1453
    self.frontalArea = 1.05 * CP.wheelbase + 0.0679
    self.coeffDrag = 0.30
    self.airDensity = 1.225

    self.bolt_acc_pedal_friction_release_frames = 0
    self.bolt_acc_pedal_friction_low_speed_active = False
    self.bolt_acc_pedal_stop_hold = False

  def calc_pedal_command(self, accel: float, long_active: bool, v_ego: float):
    if not long_active:
      self.planner_regen_hold = False
      self.regen_paddle_pressed = False
      self.regen_paddle_timer = 0
      self.regen_press_counter = 0
      self.regen_release_counter = 0
      self.regen_min_on_frames = 0
      self.regen_min_off_frames = 0
      self.pedal_active_last = False
      self.pedal_steady = 0.0
      return 0., False

    supports_regen_paddle = True  # CHEVROLET_BOLT_ACC_PEDAL is in CC_REGEN_PADDLE_CAR
    switched_state = False
    press_regen_paddle = False
    if supports_regen_paddle:
      press_cmd_threshold = np.interp(v_ego, [0.0, 4.0, 12.0, 25.0], [-0.90, -0.82, -0.72, -0.65])
      release_cmd_threshold = np.interp(v_ego, [0.0, 4.0, 12.0, 25.0], [-0.10, -0.17, -0.24, -0.30])
      press_aego_threshold = np.interp(v_ego, [0.0, 4.0, 12.0, 25.0], [-0.95, -0.86, -0.76, -0.70])
      release_aego_threshold = np.interp(v_ego, [0.0, 4.0, 12.0, 25.0], [-0.16, -0.23, -0.30, -0.36])

      press_confirm_frames = int(round(np.interp(v_ego, [0.0, 4.0, 12.0, 25.0], [8.0, 6.0, 5.0, 4.0])))
      release_confirm_frames = int(round(np.interp(v_ego, [0.0, 4.0, 12.0, 25.0], [18.0, 15.0, 12.0, 10.0])))
      min_on_frames = int(round(np.interp(v_ego, [0.0, 4.0, 12.0, 25.0], [34.0, 27.0, 20.0, 16.0])))
      min_off_frames = int(round(np.interp(v_ego, [0.0, 4.0, 12.0, 25.0], [16.0, 14.0, 12.0, 10.0])))
      confirm_boost = int(round(np.interp(v_ego, [0.0, 6.0, 8.0, 20.0, 25.0], [0.0, 0.0, 3.0, 3.0, 0.0])))
      press_confirm_frames += confirm_boost
      release_confirm_frames += confirm_boost

      want_press = self.planner_regen_hold or accel <= press_cmd_threshold or self.aego <= press_aego_threshold
      want_release = (not self.planner_regen_hold) and accel >= release_cmd_threshold and self.aego >= release_aego_threshold

      if want_press:
        self.regen_press_counter += 1
      else:
        self.regen_press_counter = max(self.regen_press_counter - 1, 0)

      if want_release:
        self.regen_release_counter += 1
      else:
        self.regen_release_counter = max(self.regen_release_counter - 1, 0)

      if self.planner_regen_hold and accel <= (press_cmd_threshold - 0.30):
        self.regen_press_counter = max(self.regen_press_counter, press_confirm_frames)

      if self.regen_min_on_frames > 0:
        self.regen_min_on_frames -= 1
      if self.regen_min_off_frames > 0:
        self.regen_min_off_frames -= 1

      if self.regen_paddle_pressed:
        if self.regen_min_on_frames == 0 and self.regen_release_counter >= release_confirm_frames:
          self.regen_paddle_pressed = False
          self.regen_min_off_frames = min_off_frames
          self.regen_release_counter = 0
          switched_state = True
      else:
        if self.regen_min_off_frames == 0 and self.regen_press_counter >= press_confirm_frames:
          self.regen_paddle_pressed = True
          self.regen_min_on_frames = min_on_frames
          self.regen_press_counter = 0
          switched_state = True

      self.regen_paddle_timer = self.regen_press_counter
      press_regen_paddle = self.regen_paddle_pressed

      if self.maneuver_paddle_mode == "off":
        self.regen_paddle_pressed = False
        self.regen_press_counter = 0
        self.regen_release_counter = 0
        self.regen_min_on_frames = 0
        press_regen_paddle = False
      elif self.maneuver_paddle_mode == "force":
        forced_press = accel < -0.02
        self.regen_paddle_pressed = forced_press
        press_regen_paddle = forced_press

    speed_mps = [0.559, 1.678, 2.797, 3.916, 5.035, 6.154, 7.273, 8.392, 9.511, 10.63,
                 11.749, 12.868, 13.987, 15.106, 16.225, 17.344, 18.463, 19.582, 20.701, 21.820,
                 22.939, 24.058, 25.177, 26.296]
    regen_gain_ratio = [1.01, 1.01, 1.02, 1.05, 1.08, 1.31, 1.33,
                        1.34, 1.35, 1.36, 1.37, 1.38, 1.39, 1.39,
                        1.40, 1.40, 1.41, 1.42, 1.43, 1.43, 1.44,
                        1.44, 1.45, 1.45]

    gain = np.interp(v_ego, speed_mps, regen_gain_ratio)
    gain *= np.interp(v_ego, [0.0, 2.0, 4.0, 5.5, 8.0, 12.0], [0.92, 0.92, 0.93, 0.94, 0.96, 1.0])
    accel_gain = np.interp(v_ego, [0.0, 3.0, 8.0, 20.0], [0.47, 0.52, 0.57, 0.61])
    pedaloffset = np.interp(v_ego, [0.0, 1.0, 3.0, 6.0, 15.0, 30.0], [0.085, 0.11, 0.17, 0.23, 0.235, 0.23])

    # Suppress tiny planner sign flaps around zero so creep-speed pedal shaping stays stable.
    pedal_accel = 0.0 if abs(accel) < 0.04 else accel

    accel_term_scale = (1.0 / max(gain, 1e-3)) if press_regen_paddle else 1.0
    if pedal_accel >= 0.0:
      small_cmd_scale = np.interp(abs(pedal_accel), [0.0, 0.35, 0.8, 1.5, 2.5], [0.58, 0.68, 0.82, 0.93, 1.0])
    else:
      small_cmd_scale = np.interp(abs(pedal_accel), [0.0, 0.35, 0.8, 1.5, 2.5], [0.44, 0.54, 0.70, 0.89, 1.0])
    accel_cmd = pedal_accel * small_cmd_scale
    if pedal_accel < -2.0:
      accel_cmd *= np.interp(abs(pedal_accel), [2.0, 2.5, 3.0], [1.0, 1.03, 1.06])
    raw_pedal_gas = float(np.clip(pedaloffset + accel_cmd * accel_gain * accel_term_scale, 0.0, 1.0))

    pedal_gas_max = np.interp(v_ego, [0.0, 1.0, 2.5, 4.5, 6.0, 8.0, 12.0], [0.20, 0.235, 0.29, 0.365, 0.52, 0.78, 1.0])
    target_pedal_gas = float(np.clip(raw_pedal_gas, 0.0, pedal_gas_max))

    if not self.pedal_active_last or (switched_state and v_ego > 1.0):
      pedal_gas = target_pedal_gas
      self.pedal_active_last = True
    else:
      urgency = float(np.clip(abs(accel) / 2.0, 0.0, 1.0))
      rate_up = np.interp(v_ego, [0.0, 3.0, 8.0, 20.0], [0.007, 0.012, 0.022, 0.036]) + 0.011 * urgency
      if accel > 0.0 and v_ego > 6.0:
        comfort_factor = np.interp(abs(accel), [0.0, 0.12, 0.25, 0.45, 0.8], [0.55, 0.58, 0.68, 0.82, 1.0])
        rate_up *= comfort_factor
      if accel > 1.2:
        rate_up += np.interp(v_ego, [0.0, 4.0, 12.0, 25.0], [0.006, 0.005, 0.003, 0.002])
      rate_down = np.interp(v_ego, [0.0, 3.0, 8.0, 20.0], [0.008, 0.014, 0.026, 0.045]) + 0.015 * urgency
      pedal_gas = float(np.clip(target_pedal_gas, self.pedal_steady - rate_down, self.pedal_steady + rate_up))

    self.pedal_steady = pedal_gas
    return pedal_gas, press_regen_paddle

  def update(self, CC, CS, frame, long_pitch_enabled, can_sends):
    """One 100 Hz controller frame; appends this car's longitudinal messages and returns (apply_gas, apply_brake).
    apply_gas is in StarPilot's torque scale (5500-7168) and only reported; this car sends no 0x2CB."""
    actuators = CC.actuators
    self.aego = CS.out.aEgo
    accel = actuators.accel
    self.bolt_acc_pedal_stop_hold = update_bolt_acc_pedal_stop_hold(self.CP, CC, CS, self.bolt_acc_pedal_stop_hold)
    press_regen_paddle = False

    # Interceptor + CC_REGEN_PADDLE_CAR + openpilot long hold by construction; only longActive varies.
    if CC.longActive:
      if self.maneuver_paddle_mode == "off":
        self.planner_regen_hold = False
      elif self.maneuver_paddle_mode == "force":
        self.planner_regen_hold = accel < -0.02
      else:
        planner_press_threshold = np.interp(CS.out.vEgo, [0.0, 4.0, 12.0, 25.0], [-0.95, -0.82, -0.70, -0.62])
        planner_release_threshold = np.interp(CS.out.vEgo, [0.0, 4.0, 12.0, 25.0], [-0.14, -0.22, -0.30, -0.36])
        if accel <= planner_press_threshold:
          self.planner_regen_hold = True
        elif accel >= planner_release_threshold:
          self.planner_regen_hold = False
    else:
      self.planner_regen_hold = False

    hud_control = CC.hudControl
    hud_alert = hud_control.visualAlert
    hud_v_cruise = hud_control.setSpeed
    if hud_v_cruise > 70:
      hud_v_cruise = 0

    # use_panda_paddle_sched: feed 0xBD/0x1F5 while enabled and for 2 frames after
    if CC.enabled:
      self.paddle_handoff_frames = 2
    paddle_sched_feed_active = CC.enabled or (self.paddle_handoff_frames > 0)
    if not CC.enabled and self.paddle_handoff_frames > 0:
      self.paddle_handoff_frames -= 1

    bolt_acc_pedal_friction_main_on = CS.out.cruiseState.available
    if not bolt_acc_pedal_friction_main_on:
      self.bolt_acc_pedal_friction_low_speed_active = False

    # Gas/regen, brakes, and UI commands - all at 25Hz
    if frame % 4 == 0:
      stopping = actuators.longControlState == LongCtrlState.stopping
      interceptor_gas_cmd = 0
      at_full_stop = CC.longActive and CS.out.standstill
      near_stop = CC.longActive and (CS.out.vEgo < self.params.NEAR_STOP_BRAKE_PHASE)
      if not CC.longActive:
        # ASCM sends max regen when not enabled
        self.apply_gas = self.params.INACTIVE_REGEN
        self.apply_brake = 0
        self.bolt_acc_pedal_friction_low_speed_active = False
        self.planner_regen_hold = False
        self.regen_paddle_pressed = False
        self.regen_paddle_timer = 0
        self.regen_press_counter = 0
        self.regen_release_counter = 0
        self.regen_min_on_frames = 0
        self.regen_min_off_frames = 0
      else:
        # should_use_fixed_stopping_brake() is always False for this car
        if long_pitch_enabled and len(CC.orientationNED) == 3 and CS.out.vEgo > self.CP.vEgoStopping:
          accel_due_to_pitch = math.sin(CC.orientationNED[1]) * ACCELERATION_DUE_TO_GRAVITY
        else:
          accel_due_to_pitch = 0.0

        gas_max = self.params.MAX_GAS
        accel_max = self.params.ACCEL_MAX

        accel_due_to_pitch = limit_grade_feedforward(actuators.accel, accel_due_to_pitch)
        accel_input = actuators.accel + accel_due_to_pitch

        accel_cmd = float(np.clip(accel_input, self.params.ACCEL_MIN, accel_max))
        drag = 0.5 * self.coeffDrag * self.frontalArea * self.airDensity * CS.out.vEgo ** 2
        torque = self.tireRadius * ((self.mass * accel_cmd) + drag)
        scaled_torque = torque + self.params.ZERO_GAS
        apply_gas_torque = np.clip(scaled_torque, self.params.MAX_ACC_REGEN, gas_max)
        brake_switch = int(round(np.interp(CS.out.vEgo, self.params.BRAKE_SWITCH_LOOKUP_BP, self.params.BRAKE_SWITCH_LOOKUP_V)))
        if bolt_acc_pedal_friction_main_on:
          planner_brake_switch = get_bolt_acc_pedal_planner_brake_switch(
            CS.out.vEgo, self.params, self.tireRadius, self.mass, self.coeffDrag, self.frontalArea, self.airDensity,
          )
          brake_switch = get_bolt_acc_pedal_effective_brake_switch(brake_switch, planner_brake_switch)
        brake_accel = min((scaled_torque - brake_switch) / (self.tireRadius * self.mass), 0)
        self.apply_gas = int(round(apply_gas_torque))
        self.apply_brake = int(round(np.interp(brake_accel, self.params.BRAKE_LOOKUP_BP, self.params.BRAKE_LOOKUP_V)))
        if bolt_acc_pedal_friction_main_on:
          if self.apply_brake > 0:
            full_brake_accel = min(
              self.params.ACCEL_MIN + drag / self.mass +
              (self.params.ZERO_GAS - brake_switch) / (self.tireRadius * self.mass),
              -0.1,
            )
            self.apply_brake = get_bolt_acc_pedal_friction_brake(
              self.apply_brake, full_brake_accel, CS.out.vEgo, self.params,
            )
          self.apply_brake, self.bolt_acc_pedal_friction_low_speed_active = shape_bolt_acc_pedal_low_speed_friction(
            self.apply_brake, CS.out.vEgo, stopping, self.bolt_acc_pedal_friction_low_speed_active,
          )
        if self.apply_brake > 0:
          self.apply_gas = self.params.INACTIVE_REGEN

        if stopping:
          self.apply_gas = self.params.INACTIVE_REGEN

        # gas interceptor only used for full long control on cars without ACC
        interceptor_gas_cmd, press_regen_paddle = self.calc_pedal_command(actuators.accel, CC.longActive, CS.out.vEgo)

      if self.apply_gas > self.params.INACTIVE_REGEN and use_interceptor_sng_launch(self.CP, CS, False):
        interceptor_gas_cmd = get_interceptor_sng_gas_cmd(self.CP, interceptor_gas_cmd, actuators.accel, self.params, False)
        self.apply_brake = 0
        self.apply_gas = self.params.INACTIVE_REGEN

      idx = (frame // 4) % 4

      self.regen_paddle_pressed = press_regen_paddle
      paddle_spoof_pressed = CC.longActive and self.regen_paddle_pressed and (CS.out.vEgo > 2.68)
      if paddle_sched_feed_active:
        can_sends.append(gmcan.create_prndl2_command(self.packer_pt, CanBus.POWERTRAIN, paddle_spoof_pressed, self.CP))
        can_sends.append(gmcan.create_regen_paddle_press_command(self.packer_pt, CanBus.POWERTRAIN, paddle_spoof_pressed))

      if self.bolt_acc_pedal_stop_hold:
        interceptor_gas_cmd = 0.0
        self.pedal_steady = 0.0
      can_sends.append(create_gas_interceptor_command(self.packer_pt, interceptor_gas_cmd, idx))

      # friction_brake_bus: fwdCamera, not SDGM -> powertrain
      if self.CP.networkLocation == NetworkLocation.fwdCamera:
        at_full_stop = at_full_stop and stopping
      experiment_brake, self.bolt_acc_pedal_friction_release_frames, should_send_bolt_acc_pedal_friction = \
        get_bolt_acc_pedal_friction_command_state(
          self.apply_brake,
          bolt_acc_pedal_friction_main_on,
          self.bolt_acc_pedal_friction_release_frames,
        )
      # Never apply stock friction with cruise main off, but send a short explicit zero-brake unwind so the last
      # nonzero EBCM command cannot linger after a disengage or main-off event.
      if should_send_bolt_acc_pedal_friction:
        can_sends.append(gmcan.create_friction_brake_command(
          self.packer_ch, CanBus.POWERTRAIN, experiment_brake, idx, bolt_acc_pedal_friction_main_on and CC.longActive,
          near_stop, at_full_stop, self.CP))

      # should_send_acc_dashboard_status(): always for this car with openpilot long on the pedal
      fcw_alert = get_acc_dashboard_fcw_alert(hud_alert, CS)
      can_sends.append(gmcan.create_starpilot_acc_dashboard_command(
        self.packer_pt, CanBus.POWERTRAIN, CC.enabled, hud_v_cruise * CV.MS_TO_KPH, hud_control, fcw_alert, 1))

    # ADAS keepalive only without the camera (get_adas_keepalive_step)
    if self.CP.flags & GMFlags.NO_CAMERA.value and frame % self.params.CAMERA_KEEPALIVE_STEP == 0:
      can_sends += gmcan.create_adas_keepalive(CanBus.POWERTRAIN)

    # Cancel the camera's stock ACC from the camera side whenever it reports ACCCmdActive, engaged or not
    if CS.out.cruiseState.enabled and (frame - self.last_button_frame) * DT_CTRL > 0.04:
      self.last_button_frame = frame
      can_sends.append(gmcan.create_buttons(self.packer_pt, CanBus.CAMERA, (CS.buttons_counter + 1) % 4, CruiseButtons.CANCEL))

    return self.apply_gas, self.apply_brake
