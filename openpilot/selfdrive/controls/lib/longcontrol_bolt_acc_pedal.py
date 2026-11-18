"""StarPilot LongControl for the Bolt ACC with a comma pedal (CHEVROLET_BOLT_ACC_PEDAL) in StarPilot mode only
(BoltPedalLongMode 2, which the interface marks with GMFlags.PEDAL_LONG; mode 1 runs carrot's LongControl).

Ported from firestar5683/StarPilot 2589caad selfdrive/controls/lib/longcontrol.py and longcontrol_vehicle_tunes.py, as
that LongControl runs for CHEVROLET_BOLT_ACC_2022_2023_PEDAL: the PID closes on acceleration error (a_target - aEgo)
with StarPilot's PID clipping, kf is the feedforward gain, and this car's feedforward restore, friction brake bias and
floor, integrator freeze and start handoff floor apply. Every other car keeps carrot's LongControl unchanged.

Inputs StarPilot takes from its own infrastructure are fixed to its defaults: StarPilot's vEgoStarting / stopAccel /
stoppingDecelRate / startAccel toggles are the CarParams values, no traffic mode, no custom acceleration profile, no
acceleration-profile launch cap, no experimental-mode blending and no Tesla pedal override. Vehicle tunes for other
cars (Volt, GM trucks, Toyota, Hyundai, Subaru, Ford, VW) are no-ops for this car and are not carried.
"""
import numpy as np

from openpilot.cereal import car
from openpilot.common.realtime import DT_CTRL

LongCtrlState = car.CarControl.Actuators.LongControlState
clip = np.clip
interp = np.interp

BOLT_ACC_PEDAL_FINGERPRINT = "CHEVROLET_BOLT_ACC_PEDAL"
GM_FLAG_PEDAL_LONG = 1  # opendbc.car.gm.values.GMFlags.PEDAL_LONG

# longcontrol.py
STOPPING_RELEASE_HYSTERESIS = 0.35
STOPPING_RELEASE_MIN_ACCEL = 0.15
STOPPING_RELEASE_STRONG_ACCEL = 0.45
LEAD_GAP_SETTLE_MAX_START_ACCEL = 0.25
MOVING_STOP_FOLLOW_MIN_GAP = 0.25
NEGATIVE_TARGET_CREEP_GUARD_SPEED = 0.35
NEGATIVE_TARGET_CREEP_GUARD_DECEL = 0.40

# longcontrol_vehicle_tunes.py
BOLT_ACC_PEDAL_REGEN_LIMIT_BP = [0.0, 1.5, 4.0, 8.0, 15.0, 30.0]
BOLT_ACC_PEDAL_REGEN_LIMIT_V = [-0.93, -1.28, -1.98, -2.58, -2.86, -2.95]
BOLT_ACC_PEDAL_START_HANDOFF_TIME = 0.75
BOLT_ACC_PEDAL_START_HANDOFF_MAX_SPEED = 1.25
BOLT_ACC_PEDAL_START_HANDOFF_MIN_TARGET = 0.15
BOLT_ACC_PEDAL_START_HANDOFF_FLOOR_BP = [0.0, 0.5, BOLT_ACC_PEDAL_START_HANDOFF_MAX_SPEED]
BOLT_ACC_PEDAL_START_HANDOFF_FLOOR_V = [0.22, 0.18, 0.10]


def is_bolt_acc_pedal_long(CP) -> bool:
  """LongControlVehicleTuning.is_bolt_acc_pedal_friction_car."""
  return bool(
    getattr(CP, "brand", "") == "gm" and
    getattr(CP, "enableGasInterceptorDEPRECATED", False) and
    str(getattr(CP, "carFingerprint", "")) == BOLT_ACC_PEDAL_FINGERPRINT and
    (int(getattr(CP, "flags", 0)) & GM_FLAG_PEDAL_LONG)
  )


def bolt_acc_pedal_long_actuator_delay(CP):
  """StarPilot plans this car (planner action_t and modeld long_delay) with CP.longitudinalActuatorDelay, 0.6 s from the
  interface. Carrot's planner and modeld read the LongActuatorDelay Param instead, which would quietly replace it, so
  they take this value when it is not None. Every other car returns None and keeps the Param."""
  return float(CP.longitudinalActuatorDelay) if is_bolt_acc_pedal_long(CP) else None


class StarPilotPIDController:
  """StarPilot common/pid.py PIDController: feedforward added as given, integrator held at the limit it pushes into."""
  def __init__(self, k_p, k_i, k_d=0., pos_limit=1e308, neg_limit=-1e308, rate=100):
    self._k_p = k_p
    self._k_i = k_i
    self._k_d = k_d
    if isinstance(self._k_d, (int, float)):
      self._k_d = [[0], [self._k_d]]

    self.pos_limit = pos_limit
    self.neg_limit = neg_limit

    self.i_dt = 1.0 / rate
    self.speed = 0.0

    self.reset()

  @property
  def k_p(self):
    return np.interp(self.speed, self._k_p[0], self._k_p[1])

  @property
  def k_i(self):
    return np.interp(self.speed, self._k_i[0], self._k_i[1])

  @property
  def k_d(self):
    return np.interp(self.speed, self._k_d[0], self._k_d[1])

  def reset(self):
    self.p = 0.0
    self.i = 0.0
    self.d = 0.0
    self.f = 0.0
    self.control = 0

  def update(self, error, error_rate=0.0, speed=0.0, feedforward=0., freeze_integrator=False):
    self.speed = speed
    self.p = self.k_p * float(error)
    self.d = self.k_d * error_rate
    self.f = feedforward

    if not freeze_integrator:
      i = self.i + self.k_i * self.i_dt * error

      # Don't allow windup if already clipping
      test_control = self.p + i + self.d + self.f
      i_upperbound = self.i if test_control > self.pos_limit else self.pos_limit
      i_lowerbound = self.i if test_control < self.neg_limit else self.neg_limit
      self.i = np.clip(i, i_lowerbound, i_upperbound)

    control = self.p + self.i + self.d + self.f
    self.control = np.clip(control, self.neg_limit, self.pos_limit)
    return self.control


def long_control_state_trans(CP, active, long_control_state, v_ego,
                             should_stop, brake_pressed, cruise_standstill,
                             allow_stopping_release=True):
  # Ignore cruise standstill if car has a gas interceptor
  cruise_standstill = cruise_standstill and not CP.enableGasInterceptorDEPRECATED
  stopping_condition = should_stop
  release_condition = not should_stop and not brake_pressed
  starting_condition = release_condition and not cruise_standstill
  # Some stock ACC platforms keep standstill latched until they see positive drive torque.
  # Once the planner has sustained a release request, allow LongControl to leave stopping
  # even if the standstill bit has not dropped yet.
  stopping_release_condition = release_condition and allow_stopping_release
  started_condition = v_ego > CP.vEgoStarting

  if not active:
    long_control_state = LongCtrlState.off

  else:
    if long_control_state == LongCtrlState.off:
      if not starting_condition:
        long_control_state = LongCtrlState.stopping
      else:
        if starting_condition and CP.startingState:
          long_control_state = LongCtrlState.starting
        else:
          long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.stopping:
      if stopping_release_condition and CP.startingState:
        long_control_state = LongCtrlState.starting
      elif stopping_release_condition:
        long_control_state = LongCtrlState.pid

    elif long_control_state in [LongCtrlState.starting, LongCtrlState.pid]:
      if stopping_condition:
        long_control_state = LongCtrlState.stopping
      elif started_condition:
        long_control_state = LongCtrlState.pid

  return long_control_state


def get_bolt_acc_pedal_friction_bias(output_accel, a_target, v_ego):
  if output_accel >= -0.05 or a_target >= -0.80 or v_ego <= 5.0:
    return 0.0

  authority_gap = max(0.0, abs(a_target) - abs(output_accel))
  if authority_gap <= 0.25:
    return 0.0

  speed_factor = interp(v_ego, [5.0, 10.0, 15.0, 25.0], [0.0, 0.55, 0.85, 1.0])
  max_bias = interp(abs(a_target), [0.8, 1.4, 2.2, 3.5], [0.0, 0.14, 0.42, 0.70])
  return float(min(authority_gap * 0.30, max_bias) * speed_factor)


def get_bolt_acc_pedal_friction_floor(a_target, v_ego, pedal_regen_limit):
  if v_ego <= 5.0 or a_target >= (pedal_regen_limit - 0.05):
    return None

  friction_request = max(0.0, pedal_regen_limit - a_target)
  if friction_request <= 0.10:
    return None

  speed_factor = interp(v_ego, [5.0, 8.0, 12.0, 18.0, 25.0], [0.0, 0.45, 0.75, 0.90, 1.0])
  demand_factor = interp(friction_request, [0.10, 0.25, 0.50, 0.90, 1.30], [0.0, 0.22, 0.50, 0.78, 1.0])
  floor_fraction = float(clip(speed_factor * demand_factor, 0.0, 1.0))

  return float(pedal_regen_limit - (friction_request * floor_fraction))


def get_bolt_acc_pedal_feedforward_gain(feedforward_gain, a_target, v_ego, pedal_regen_limit, last_output_accel):
  effective_gain = feedforward_gain
  if a_target >= 0.0:
    return effective_gain

  restore = 0.0

  if a_target < pedal_regen_limit:
    friction_gap = pedal_regen_limit - a_target
    restore = float(interp(friction_gap, [0.0, 0.25, 0.75], [0.0, 0.6, 1.0]))

  if v_ego > 5.0 and a_target < -1.10:
    authority_gap = max(0.0, abs(a_target) - abs(min(last_output_accel, 0.0)))
    target_restore = float(interp(abs(a_target), [1.1, 1.6, 2.2, 3.0], [0.0, 0.25, 0.55, 1.0]))
    gap_restore = float(interp(authority_gap, [0.2, 0.6, 1.0, 1.6], [0.0, 0.25, 0.60, 1.0]))
    speed_factor = float(interp(v_ego, [5.0, 8.0, 12.0, 18.0], [0.0, 0.35, 0.75, 1.0]))
    restore = max(restore, max(target_restore, gap_restore) * speed_factor)

  return float(feedforward_gain + ((1.0 - feedforward_gain) * clip(restore, 0.0, 1.0)))


class BoltAccPedalLongControl:
  def __init__(self, CP):
    self.CP = CP
    self.long_control_state = LongCtrlState.off
    self.pid = StarPilotPIDController((CP.longitudinalTuning.kpBP, CP.longitudinalTuning.kpV),
                                      (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV),
                                      rate=1 / DT_CTRL)
    # Preserve legacy behaviour when no feedforward gain is provided (default of 0.0)
    kf = CP.longitudinalTuning.kf
    self.feedforward_gain = kf if kf != 0.0 else 1.0
    self.last_output_accel = 0.0
    self.stop_release_counter = 0
    self.reset_vehicle_tuning()

  # ---- LongControlVehicleTuning (this car's parts) ----
  def reset_vehicle_tuning(self):
    self.last_a_target = 0.0
    self.integrator_hold_frames = 0
    self.bolt_start_handoff_frames = 0

  def apply_bolt_start_handoff_floor(self, output_accel, last_output_accel, a_target, v_ego,
                                     starting_handoff, should_stop, has_lead):
    if starting_handoff:
      self.bolt_start_handoff_frames = int(round(BOLT_ACC_PEDAL_START_HANDOFF_TIME / DT_CTRL))

    safe_to_hold = (
      self.bolt_start_handoff_frames > 0 and
      has_lead and
      not should_stop and
      a_target > BOLT_ACC_PEDAL_START_HANDOFF_MIN_TARGET and
      v_ego < BOLT_ACC_PEDAL_START_HANDOFF_MAX_SPEED
    )
    if not safe_to_hold:
      self.bolt_start_handoff_frames = 0
      return output_accel

    self.bolt_start_handoff_frames -= 1
    speed_floor = float(interp(v_ego, BOLT_ACC_PEDAL_START_HANDOFF_FLOOR_BP,
                               BOLT_ACC_PEDAL_START_HANDOFF_FLOOR_V))
    target_floor = min(speed_floor, max(0.0, 0.4 * float(a_target)))
    return max(float(output_accel), min(float(last_output_accel), target_floor))

  def get_integrator_freeze(self, last_output_accel, a_target, error, v_ego, accel_limits):
    # is_gm_pedal_long
    handoff_threshold = interp(v_ego, [0.0, 4.0, 12.0, 25.0], [0.35, 0.45, 0.55, 0.70])
    hold_frames = int(round(interp(v_ego, [0.0, 4.0, 12.0, 25.0], [25.0, 20.0, 14.0, 10.0])))

    if abs(a_target - self.last_a_target) > handoff_threshold:
      self.integrator_hold_frames = max(self.integrator_hold_frames, hold_frames)
    self.last_a_target = a_target

    if self.integrator_hold_frames > 0:
      self.integrator_hold_frames -= 1

    sat_buffer = 0.03
    at_neg_sat = last_output_accel <= (accel_limits[0] + sat_buffer)
    at_pos_sat = last_output_accel >= (accel_limits[1] - sat_buffer)
    sat_pushing_lower = at_neg_sat and error < -0.05
    sat_pushing_upper = at_pos_sat and error > 0.05

    return self.integrator_hold_frames > 0 or sat_pushing_lower or sat_pushing_upper

  @staticmethod
  def apply_pedal_long_brake_bias(output_accel, a_target, CS):
    if output_accel >= -0.05 or a_target >= -0.80:
      return output_accel
    if CS.vEgo <= 5.0:
      return output_accel

    # is_bolt_acc_pedal_friction_car
    pedal_regen_limit = float(interp(CS.vEgo, BOLT_ACC_PEDAL_REGEN_LIMIT_BP, BOLT_ACC_PEDAL_REGEN_LIMIT_V))
    bias = get_bolt_acc_pedal_friction_bias(output_accel, a_target, CS.vEgo)
    floor = get_bolt_acc_pedal_friction_floor(a_target, CS.vEgo, pedal_regen_limit)
    if floor is not None:
      bias = max(bias, output_accel - floor)
    return output_accel - float(max(bias, 0.0))

  @staticmethod
  def get_longitudinal_feedforward(feedforward_gain, last_output_accel, a_target, v_ego):
    feedforward = a_target * feedforward_gain
    if a_target >= 0.0:
      return feedforward

    pedal_regen_limit = float(interp(v_ego, BOLT_ACC_PEDAL_REGEN_LIMIT_BP, BOLT_ACC_PEDAL_REGEN_LIMIT_V))
    effective_gain = get_bolt_acc_pedal_feedforward_gain(
      feedforward_gain, a_target, v_ego, pedal_regen_limit, last_output_accel,
    )
    return a_target * effective_gain

  # ---- LongControl ----
  def reset(self, preserve_stop_release=False):
    self.pid.reset()
    self.reset_vehicle_tuning()
    if not preserve_stop_release:
      self.stop_release_counter = 0

  def _stop_release_ready(self, CS, a_target, should_stop, has_lead):
    if self.long_control_state != LongCtrlState.stopping:
      self.stop_release_counter = 0
      return True

    if should_stop or CS.brakePressed:
      self.stop_release_counter = 0
      return False

    if CS.vEgo > self.CP.vEgoStarting:
      self.stop_release_counter = int(round(STOPPING_RELEASE_HYSTERESIS / DT_CTRL))
      return True

    if has_lead and a_target > STOPPING_RELEASE_MIN_ACCEL:
      self.stop_release_counter = int(round(STOPPING_RELEASE_HYSTERESIS / DT_CTRL))
      return True

    if a_target >= STOPPING_RELEASE_STRONG_ACCEL and not CS.cruiseState.standstill:
      self.stop_release_counter = int(round(STOPPING_RELEASE_HYSTERESIS / DT_CTRL))
      return True

    if a_target > STOPPING_RELEASE_MIN_ACCEL:
      max_frames = int(round(STOPPING_RELEASE_HYSTERESIS / DT_CTRL))
      self.stop_release_counter = min(self.stop_release_counter + 1, max_frames)
    else:
      self.stop_release_counter = 0

    return self.stop_release_counter >= int(round(STOPPING_RELEASE_HYSTERESIS / DT_CTRL))

  def _apply_moving_stop_target_follow(self, output_accel, a_target, should_stop, CS):
    follow_min_speed = max(1.5, self.CP.vEgoStopping + 1.0)
    if not should_stop or CS.brakePressed or CS.vEgo <= follow_min_speed:
      return output_accel
    if a_target >= output_accel - MOVING_STOP_FOLLOW_MIN_GAP:
      return output_accel

    follow_step = interp(CS.vEgo, [follow_min_speed, 3.0, 6.0, 10.0], [0.02, 0.03, 0.05, 0.07])
    return max(float(a_target), output_accel - float(follow_step))

  def _trim_positive_overshoot_integrator(self, a_target, error, CS):
    if self.pid.i <= 0.0:
      return
    if a_target >= -0.05 or error >= -0.25:
      return
    if CS.vEgo <= NEGATIVE_TARGET_CREEP_GUARD_SPEED and a_target > -NEGATIVE_TARGET_CREEP_GUARD_DECEL:
      return

    # If the planner has already crossed into decel but the car is still
    # accelerating, bleed stale positive I aggressively so the command can
    # cross back through zero instead of carrying throttle for several seconds.
    bleed = interp(abs(error), [0.25, 0.75, 1.5], [0.55, 0.25, 0.0])
    self.pid.i *= bleed

  @staticmethod
  def _cap_positive_output_on_negative_target(output_accel, a_target, error, CS):
    if output_accel <= 0.0:
      return output_accel
    if a_target >= -0.10 or error >= -0.35:
      return output_accel
    if CS.vEgo <= NEGATIVE_TARGET_CREEP_GUARD_SPEED and a_target > -NEGATIVE_TARGET_CREEP_GUARD_DECEL:
      return output_accel

    # Once the planner is asking for real decel, don't keep feeding positive
    # drive torque while we're still accelerating away from the target.
    positive_cap = interp(a_target, [-1.5, -0.6, -0.1], [0.0, 0.0, 0.05])
    return min(output_accel, float(positive_cap))

  def update(self, active, CS, a_target, should_stop, accel_limits, has_lead=False, soft_hold_active=False):
    """Update longitudinal control. This updates the state machine and runs a PID loop"""
    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1]

    previous_long_control_state = self.long_control_state
    allow_stopping_release = self._stop_release_ready(CS, a_target, should_stop, has_lead)
    self.long_control_state = long_control_state_trans(self.CP, active, self.long_control_state, CS.vEgo,
                                                       should_stop, CS.brakePressed,
                                                       CS.cruiseState.standstill,
                                                       allow_stopping_release=allow_stopping_release)
    # carrot soft hold (driver-armed hold at a stop, carrot's cruise helper) keeps stopping, as for every carrot car
    if active and soft_hold_active:
      self.long_control_state = LongCtrlState.stopping

    if self.long_control_state == LongCtrlState.off:
      self.reset()
      output_accel = 0.

    elif self.long_control_state == LongCtrlState.stopping:
      output_accel = self.last_output_accel
      if soft_hold_active:
        output_accel = self.CP.stopAccel
      if output_accel > self.CP.stopAccel:
        output_accel = min(output_accel, 0.0)
        output_accel -= self.CP.stoppingDecelRate * DT_CTRL
      output_accel = self._apply_moving_stop_target_follow(output_accel, a_target, should_stop, CS)
      self.reset(preserve_stop_release=True)

    elif self.long_control_state == LongCtrlState.starting:
      if has_lead and a_target <= LEAD_GAP_SETTLE_MAX_START_ACCEL:
        output_accel = clip(a_target, 0.0, self.CP.startAccel)
      else:
        output_accel = self.CP.startAccel
      self.reset()

    else:  # LongCtrlState.pid
      error = a_target - CS.aEgo
      self._trim_positive_overshoot_integrator(a_target, error, CS)
      feedforward = self.get_longitudinal_feedforward(
        self.feedforward_gain, self.last_output_accel, a_target, CS.vEgo,
      )
      freeze_integrator = self.get_integrator_freeze(
        self.last_output_accel, a_target, error, CS.vEgo, accel_limits,
      )
      output_accel = self.pid.update(error, speed=CS.vEgo, feedforward=feedforward,
                                     freeze_integrator=freeze_integrator)
      output_accel = self._cap_positive_output_on_negative_target(output_accel, a_target, error, CS)
      output_accel = self.apply_pedal_long_brake_bias(output_accel, a_target, CS)
      output_accel = self.apply_bolt_start_handoff_floor(
        output_accel,
        self.last_output_accel,
        a_target,
        CS.vEgo,
        previous_long_control_state == LongCtrlState.starting,
        should_stop,
        has_lead,
      )

    self.last_output_accel = clip(output_accel, accel_limits[0], accel_limits[1])
    return self.last_output_accel
