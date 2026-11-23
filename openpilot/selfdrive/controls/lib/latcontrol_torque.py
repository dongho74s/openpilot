from collections import deque

import math
import numpy as np

from openpilot.cereal import log
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N

from opendbc.car.interfaces import LatControlInputs
from opendbc.car.vehicle_model import ACCELERATION_DUE_TO_GRAVITY
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.common.pid import PIDController

from openpilot.common.params import Params
from opendbc.car.gm.values import CAR as GM_CAR

# At higher speeds (25+mph) we can assume:
# Lateral acceleration achieved by a specific car correlates to
# torque applied to the steering rack. It does not correlate to
# wheel slip, or to speed.

# This controller applies torque to achieve desired lateral
# accelerations. To compensate for the low speed effects we
# use a LOW_SPEED_FACTOR in the error. Additionally, there is
# friction in the steering wheel that needs to be overcome to
# move it at all, this is compensated for too.

LOW_SPEED_X = [0, 10, 20, 30]
LOW_SPEED_Y = [15, 13, 10, 5]
LOW_SPEED_Y_NN = [12, 3, 1, 0]

LAT_PLAN_MIN_IDX = 5

# FrogPilot parity (2026-11-14) for every Chevrolet platform (user request 2026-11-15; first only the Traverse,
# Trailblazer, Trailblazer CC and Trax). As in FrogPilot it applies only while the car runs NNFF (CI.use_nnff): in this
# fork that is a Chevrolet with its own or a substitute NNFF model, NNFF on and no comma NN feedforward (the Bolt EUV,
# Bolt CC and Volt use comma's). The others keep the torque controller as before. A platform covers every car opendbc
# lists under it, so the GMC Sierra 1500 2020-21 (CHEVROLET_SILVERADO) is included too; the fingerprint cannot tell it
# apart (Codex review).
NNFF_FROGPILOT_PARITY_CARS = tuple(str(platform) for platform in GM_CAR if str(platform).startswith("CHEVROLET_"))

# Trailblazer only (2026-11-16, user request): hold back NNFF's high-lateral-acceleration error blend below 55 km/h.
# Trailblazer logs (11/15 T03-T06, same commit as the Traverse) wind and unwind the wheel at about 0.9 Hz in every
# 33-47 km/h curve, well beyond what the plan asks for. The blend reads the NN at lateral accel = error, whose slope near
# zero is about 4x the setpoint/measurement difference; at 40 km/h and 2 m/s^2 it makes the lateral-accel feedback slope
# (P+F) about 2.1x what a 0.3 blend gives (Codex: -0.84 vs -0.41). The Traverse runs the same controller without hunting;
# that the Trailblazer reacts more strongly to torque at low speed is a hypothesis from closed-loop logs, not identified.
# 0.3 and 55/65 km/h are provisional values from these logs, not a proven stability margin: check on the road.
# Below 1 m/s^2 of desired lateral accel, from 65 km/h, and on other cars (the Traverse included) the computation is the
# stock one; an integrator built up in a slow curve carries over (it has no time decay) until later errors move it.
NNFF_LOW_SPEED_ERROR_BLEND_CARS = ("CHEVROLET_TRAILBLAZER",)
LOW_SPEED_ERROR_BLEND_BP = [55. / 3.6, 65. / 3.6]  # m/s
LOW_SPEED_ERROR_BLEND_V = [0.3, 1.0]

def get_low_speed_factor(v_ego, nn_table):
  # FrogPilot's NNFF (and twilsonco's original) uses the smaller NN low-speed table.
  return np.interp(v_ego, LOW_SPEED_X, LOW_SPEED_Y_NN if nn_table else LOW_SPEED_Y)**2

def get_error_blend_factor(desired_lateral_accel, v_ego, low_speed_scale):
  """Weight of NNFF's error-input response: 0 below 1 m/s^2 of desired lateral accel, 1 above 2 (FrogPilot/
  sunnypilot). With low_speed_scale (Trailblazer) it is cut to 0.3x up to 55 km/h and back to 1x at 65 km/h."""
  factor = float(np.interp(abs(desired_lateral_accel), [1.0, 2.0], [0.0, 1.0]))
  if low_speed_scale:
    factor *= float(np.interp(v_ego, LOW_SPEED_ERROR_BLEND_BP, LOW_SPEED_ERROR_BLEND_V))
  return factor

def get_predicted_lateral_jerk(lat_accels, t_diffs):
  # compute finite difference between subsequent model_data.acceleration.y values
  # this is just two calls of np.diff followed by an element-wise division
  lat_accel_diffs = np.diff(lat_accels)
  lat_jerk = lat_accel_diffs / t_diffs
  # return as python list
  return lat_jerk.tolist()

def sign(x):
  return 1.0 if x > 0.0 else (-1.0 if x < 0.0 else 0.0)

def get_lookahead_value(future_vals, current_val):
  if len(future_vals) == 0:
    return current_val

  same_sign_vals = [v for v in future_vals if sign(v) == sign(current_val)]

  # if any future val has opposite sign of current val, return 0
  if len(same_sign_vals) < len(future_vals):
    return 0.0

  # otherwise return the value with minimum absolute value
  min_val = min(same_sign_vals + [current_val], key=lambda x: abs(x))
  return min_val

# At a given roll, if pitch magnitude increases, the
# gravitational acceleration component starts pointing
# in the longitudinal direction, decreasing the lateral
# acceleration component. Here we do the same thing
# to the roll value itself, then passed to nnff.
def roll_pitch_adjust(roll, pitch):
  return roll * math.cos(pitch)

# Torque-limit P filter (LatTorqueLimitPFilter, user request 2026-11-23): low-pass the P term while the feed-forward
# alone is close to the torque limit. At the limit the feed-forward carries ~0.9 and P the last ~0.1; when the car
# catches up P swings from + to - and takes 0.3 of torque away in 0.5 s, then the car falls behind and P pushes back
# to the limit: the wheel turns and lets go. Traverse rlogs 11/20-11/23: P caused 125 of 129 such drops, the driver
# torque limit 6 of 80. In the closed-loop sim (repo controller, fitted plant, 0.3-0.6 s) the drops fell 20-32% with
# the same tracking and curve exit; lowering P or adding feed-forward/integrator did not help. Blend weight 0 below
# |f| 0.75, full from 0.9, so ordinary steering is untouched; the integrator still sees the raw P as before.
P_FILTER_FF_BP = [0.75, 0.9]
P_FILTER_MAX_CS = 60   # setting cap in 0.01 s


class FrogPilotWindupPID(PIDController):
  """FrogPilot's (stock openpilot's) integrator handling for the FrogPilot-parity cars: while the output is
  clipping the integrator is kept from growing, but it is never cut back to the room the other terms leave.
  carrot's PIDController trims it to that room on every update, so a short P/FF spike at the torque limit
  dumps the integrator and the torque drops when the spike passes. k_f stays (the LateralTorqueKf setting).
  p_filter_tau > 0 adds the torque-limit P filter (see P_FILTER_FF_BP)."""
  p_filter_tau = 0.0
  p_filter_bypass = False

  def reset(self):
    super().reset()
    self._p_lowpass = None

  def update(self, error, error_rate=0.0, speed=0.0, override=False, feedforward=0., freeze_integrator=False):
    self.speed = speed
    self.p = float(error) * self.k_p
    self.f = feedforward * self.k_f
    self.d = error_rate * self.k_d

    if not freeze_integrator:
      i = self.i + error * self.k_i * self.i_rate
      # Don't allow windup if already clipping
      test_control = self.p + i + self.d + self.f
      i_upperbound = self.i if test_control > self.pos_limit else self.pos_limit
      i_lowerbound = self.i if test_control < self.neg_limit else self.neg_limit
      self.i = float(np.clip(i, i_lowerbound, i_upperbound))

    self.p = self._torque_limit_p(self.p)
    control = self.p + self.i + self.d + self.f
    self.control = float(np.clip(control, self.neg_limit, self.pos_limit))
    return self.control

  def _torque_limit_p(self, p):
    if self.p_filter_tau <= 0.0 or self.p_filter_bypass:
      self._p_lowpass = None   # off, or the driver is steering: plain P, and start fresh afterwards
      return p
    if not math.isfinite(p):
      return p                 # never keep a non-finite sample: the next finite P recovers as before (Codex review)
    # keep the low-pass running on every update so it is warm when the feed-forward reaches the limit
    alpha = self.i_rate / (self.p_filter_tau + self.i_rate)
    if self._p_lowpass is None or not math.isfinite(self._p_lowpass):
      self._p_lowpass = p
    else:
      self._p_lowpass += alpha * (p - self._p_lowpass)
    near = float(np.interp(abs(self.f), P_FILTER_FF_BP, [0.0, 1.0]))
    return (1.0 - near) * p + near * self._p_lowpass


class LatControlTorque(LatControl):
  def __init__(self, CP, CI):
    super().__init__(CP, CI)
    self.torque_params = CP.lateralTuning.torque.as_builder()
    # FrogPilot parity (2026-11-11/14): same NNFF path, integrator handling, delay inputs and pitch as FrogPilot
    self.nnff_frogpilot_parity = CI.use_nnff and CP.carFingerprint in NNFF_FROGPILOT_PARITY_CARS
    # Trailblazer: lower error-blend weight below 55 km/h (see NNFF_LOW_SPEED_ERROR_BLEND_CARS)
    self.low_speed_error_blend = bool(CI.use_nnff and CP.carFingerprint in NNFF_LOW_SPEED_ERROR_BLEND_CARS)
    pid_class = FrogPilotWindupPID if self.nnff_frogpilot_parity else PIDController
    self.pid = pid_class(self.torque_params.kp, self.torque_params.ki,
                         k_f=self.torque_params.kf, pos_limit=self.steer_max, neg_limit=-self.steer_max)
    self.torque_from_lateral_accel = CI.torque_from_lateral_accel()
    self.use_steering_angle = self.torque_params.useSteeringAngle
    self.steering_angle_deadzone_deg = self.torque_params.steeringAngleDeadzoneDeg

    # carrot
    self.frame = 0
    self.params = Params()
    self.lateralTorqueCustom = self.params.get_int("LateralTorqueCustom")
    self.read_torque_limit_p_filter()
    self.latAccelFactor_default = self.torque_params.latAccelFactor
    self.latAccelOffset_default = self.torque_params.latAccelOffset
    self.friction_default = self.torque_params.friction

    # Twilsonco's Lateral Neural Network Feedforward
    self.use_nnff = CI.use_nnff
    self.use_nnff_lite = CI.use_nnff_lite

    if self.use_nnff or self.use_nnff_lite:
      # Instantaneous lateral jerk changes very rapidly, making it not useful on its own,
      # however, we can "look ahead" to the future planned lateral jerk in order to guage
      # whether the current desired lateral jerk will persist into the future, i.e.
      # whether it's "deliberate" or not. This lets us simply ignore short-lived jerk.
      # Note that LAT_PLAN_MIN_IDX is defined above and is used in order to prevent
      # using a "future" value that is actually planned to occur before the "current" desired
      # value, which is offset by the steerActuatorDelay.
      self.friction_look_ahead_v = [1.4, 2.0] # how many seconds in the future to look ahead in [0, ~2.1] in 0.1 increments
      self.friction_look_ahead_bp = [9.0, 30.0] # corresponding speeds in m/s in [0, ~40] in 1.0 increments

      # Scaling the lateral acceleration "friction response" could be helpful for some.
      # Increase for a stronger response, decrease for a weaker response.
      self.lat_jerk_friction_factor = 0.4
      self.lat_accel_friction_factor = 0.7 # in [0, 3], in 0.05 increments. 3 is arbitrary safety limit

      # precompute time differences between ModelConstants.T_IDXS
      self.t_diffs = np.diff(ModelConstants.T_IDXS)
      self.desired_lat_jerk_time = self.params.get_float("SteerActuatorDelay") * 0.01 + 0.3

    if self.use_nnff:
      self.pitch = FirstOrderFilter(0.0, 0.5, 0.01)
      # NN model takes current v_ego, lateral_accel, lat accel/jerk error, roll, and past/future/planned data
      # of lat accel and roll
      # Past value is computed using previous desired lat accel and observed roll
      self.torque_from_nn = CI.get_ff_nn
      self.nn_friction_override = CI.lat_torque_nn_model.friction_override

      # setup future time offsets
      # FrogPilot adds the lateral delay alone (learned, see update_live_delay); carrot's own path adds 0.2 s
      self.nn_time_offset = CP.steerActuatorDelay + (0.0 if self.nnff_frogpilot_parity else 0.2)
      future_times = [0.3, 0.6, 1.0, 1.5] # seconds in the future
      self.future_times = future_times
      self.nn_future_times = [i + self.nn_time_offset for i in future_times]
      self.nn_future_times_np = np.array(self.nn_future_times)

      # setup past time offsets
      self.past_times = [-0.3, -0.2, -0.1]
      history_check_frames = [int(abs(i)*100) for i in self.past_times]
      self.history_frame_offsets = [history_check_frames[0] - i for i in history_check_frames]
      self.lateral_accel_desired_deque = deque(maxlen=history_check_frames[0])
      self.roll_deque = deque(maxlen=history_check_frames[0])
      self.error_deque = deque(maxlen=history_check_frames[0])
      self.past_future_len = len(self.past_times) + len(self.nn_future_times)

    # The Traverse runs FrogPilot's donor NNFF model (Trailblazer). On the Traverse, Trailblazer and Trax,
    # match FrogPilot's NNFF where it differs: NN low-speed factor, steering-rate/planned jerk kept on a
    # steering-angle car, and desired jerk taken over the lateral delay instead of SteerActuatorDelay + 0.3 s.
    if self.nnff_frogpilot_parity:
      self.desired_lat_jerk_time = self.nn_time_offset

  def update_live_delay(self, lat_delay):
    """FrogPilot: the NN's future inputs and the desired lateral jerk follow the learned lateral delay
    (liveDelay.lateralDelay; FrogPilot's LAT_SMOOTH_SECONDS is 0). Only for the FrogPilot-parity cars."""
    if not self.nnff_frogpilot_parity or not math.isfinite(lat_delay) or lat_delay <= 0.0:
      return
    self.nn_time_offset = lat_delay
    self.nn_future_times = [t + lat_delay for t in self.future_times]
    self.nn_future_times_np = np.array(self.nn_future_times)
    self.desired_lat_jerk_time = lat_delay


  def update_live_torque_params(self, latAccelFactor, latAccelOffset, friction):
    if self.lateralTorqueCustom > 0:
      return

    self.torque_params.latAccelFactor = latAccelFactor
    self.torque_params.latAccelOffset = latAccelOffset
    self.torque_params.friction = friction

  def read_torque_limit_p_filter(self):
    """LatTorqueLimitPFilter (0.01 s, 0 = off, at most 0.6 s) for the FrogPilot-parity PID; other cars keep their
    PID untouched. Params.get casts in Python (a corrupt value becomes the default); get_int's native std::stoi would
    abort controlsd on one (Codex review). The cap is the range the closed-loop sim checked: a longer filter delays
    the P correction at a curve exit while the feed-forward is still high."""
    if not isinstance(self.pid, FrogPilotWindupPID):
      return
    try:
      value = float(self.params.get("LatTorqueLimitPFilter") or 0)
    except Exception:  # an unknown key (params not rebuilt yet) or a bad value must never stop lateral control
      value = 0.0
    if not math.isfinite(value):
      value = 0.0
    self.pid.p_filter_tau = float(np.clip(value, 0, P_FILTER_MAX_CS)) * 0.01

  def update(self, active, CS, VM, params, steer_limited_by_controls, desired_curvature, CC, curvature_limited, model_data=None):
    self.frame += 1
    if self.frame % 100 == 0:
      self.read_torque_limit_p_filter()
    if self.frame % 10 == 0:
      lateralTorqueCustom = self.params.get_int("LateralTorqueCustom")
      if lateralTorqueCustom > 0:
        self.torque_params.latAccelFactor = self.params.get_float("LateralTorqueAccelFactor")*0.001
        self.torque_params.friction = self.params.get_float("LateralTorqueFriction")*0.001
        lateralTorqueKp = self.params.get_float("LateralTorqueKpV")*0.01
        lateralTorqueKi = self.params.get_float("LateralTorqueKiV")*0.01
        lateralTorqueKf = self.params.get_float("LateralTorqueKf")*0.01
        lateralTorqueKd = self.params.get_float("LateralTorqueKd")*0.01
        self.pid._k_p = [[0], [lateralTorqueKp]]
        self.pid._k_i = [[0], [lateralTorqueKi]]
        self.pid.k_f = lateralTorqueKf
        self.pid._k_d = [[0], [lateralTorqueKd]]
        self.torque_params.latAccelOffset = self.latAccelOffset_default
      elif self.lateralTorqueCustom > 1:  # 1 -> 0, reset to default
        self.torque_params.latAccelFactor = self.latAccelFactor_default
        self.torque_params.friction = self.friction_default
        self.torque_params.latAccelOffset = self.latAccelOffset_default
      self.lateralTorqueCustom = lateralTorqueCustom

    pid_log = log.ControlsState.LateralTorqueState.new_message()
    nn_log = None
    if not active:
      output_torque = 0.0
      pid_log.active = False
      angle_steers_des = float(CS.steeringAngleDeg)
      if isinstance(self.pid, FrogPilotWindupPID):
        self.pid._p_lowpass = None   # the P filter starts fresh when steering resumes
    else:
      angle_steers_des = math.degrees(VM.get_steer_from_curvature(-desired_curvature, CS.vEgo, params.roll))
      angle_steers_des += params.angleOffsetDeg

      actual_curvature_vm = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
      roll_compensation = params.roll * ACCELERATION_DUE_TO_GRAVITY
      actual_lateral_jerk = 0.0
      if self.use_steering_angle:
        actual_curvature = actual_curvature_vm
        curvature_deadzone = abs(VM.calc_curvature(math.radians(self.steering_angle_deadzone_deg), CS.vEgo, 0.0))
        if self.use_nnff or self.use_nnff_lite:
          actual_curvature_rate = -VM.calc_curvature(math.radians(CS.steeringRateDeg), CS.vEgo, 0.0)
          actual_lateral_jerk = actual_curvature_rate * CS.vEgo ** 2
          if not math.isfinite(actual_lateral_jerk):
            actual_lateral_jerk = 0.0
      else:
        if len(CC.angularVelocity) >= 3:
          actual_curvature_llk = CC.angularVelocity[2] / CS.vEgo #llk.angularVelocityCalibrated.value[2] / CS.vEgo
        else:
          actual_curvature_llk = 0
        actual_curvature = np.interp(CS.vEgo, [2.0, 5.0], [actual_curvature_vm, actual_curvature_llk])
        curvature_deadzone = 0.0
      desired_lateral_accel = desired_curvature * CS.vEgo ** 2

      # desired rate is the desired rate of change in the setpoint, not the absolute desired curvature
      # desired_lateral_jerk = desired_curvature_rate * CS.vEgo ** 2
      actual_lateral_accel = actual_curvature * CS.vEgo ** 2
      lateral_accel_deadzone = curvature_deadzone * CS.vEgo ** 2

      low_speed_factor = get_low_speed_factor(CS.vEgo, self.nnff_frogpilot_parity)
      setpoint = desired_lateral_accel + low_speed_factor * desired_curvature
      measurement = actual_lateral_accel + low_speed_factor * actual_curvature
      
      lateral_jerk_setpoint = 0
      lateral_jerk_measurement = 0
      lookahead_lateral_jerk = 0

      model_good = model_data is not None and len(model_data.orientation.x) >= CONTROL_N
      if model_good and (self.use_nnff or self.use_nnff_lite):
        # prepare "look-ahead" desired lateral jerk
        lookahead = np.interp(CS.vEgo, self.friction_look_ahead_bp, self.friction_look_ahead_v)
        friction_upper_idx = next((i for i, val in enumerate(ModelConstants.T_IDXS) if val > lookahead), 16)
        predicted_lateral_jerk = get_predicted_lateral_jerk(model_data.acceleration.y, self.t_diffs)
        desired_lateral_jerk = (np.interp(self.desired_lat_jerk_time, ModelConstants.T_IDXS, model_data.acceleration.y) - desired_lateral_accel) / self.desired_lat_jerk_time
        lookahead_lateral_jerk = get_lookahead_value(predicted_lateral_jerk[LAT_PLAN_MIN_IDX:friction_upper_idx], desired_lateral_jerk)

        # With FrogPilot parity the steering-angle car keeps the steering-rate jerk, as FrogPilot does:
        # the NN sees the planned jerk ahead of a curve and the measured jerk damps winding/unwinding.
        if (self.use_steering_angle and not self.nnff_frogpilot_parity) or lookahead_lateral_jerk == 0.0:
          lookahead_lateral_jerk = 0.0
          actual_lateral_jerk = 0.0
          self.lat_accel_friction_factor = 1.0
        lateral_jerk_setpoint = self.lat_jerk_friction_factor * lookahead_lateral_jerk
        lateral_jerk_measurement = self.lat_jerk_friction_factor * actual_lateral_jerk

      if self.use_nnff and model_good:
        # update past data
        pitch = 0
        roll = params.roll
        if len(CC.orientationNED) > 1:
          pitch = self.pitch.update(CC.orientationNED[1])
          roll = roll_pitch_adjust(roll, pitch)
        self.roll_deque.append(roll)
        self.lateral_accel_desired_deque.append(desired_lateral_accel)

        # prepare past and future values
        # adjust future times to account for longitudinal acceleration
        adjusted_future_times = [t + 0.5*CS.aEgo*(t/max(CS.vEgo, 1.0)) for t in self.nn_future_times]
        past_rolls = [self.roll_deque[min(len(self.roll_deque)-1, i)] for i in self.history_frame_offsets]
        future_rolls = [roll_pitch_adjust(np.interp(t, ModelConstants.T_IDXS, model_data.orientation.x) + roll, np.interp(t, ModelConstants.T_IDXS, model_data.orientation.y) + pitch) for t in adjusted_future_times]
        past_lateral_accels_desired = [self.lateral_accel_desired_deque[min(len(self.lateral_accel_desired_deque)-1, i)] for i in self.history_frame_offsets]
        future_planned_lateral_accels = [np.interp(t, ModelConstants.T_IDXS, model_data.acceleration.y) for t in adjusted_future_times]

        # compute NNFF error response
        nnff_setpoint_input = [CS.vEgo, setpoint, lateral_jerk_setpoint, roll] \
                              + [setpoint] * self.past_future_len \
                              + past_rolls + future_rolls
        # past lateral accel error shouldn't count, so use past desired like the setpoint input
        nnff_measurement_input = [CS.vEgo, measurement, lateral_jerk_measurement, roll] \
                                 + [measurement] * self.past_future_len \
                                 + past_rolls + future_rolls
        torque_from_setpoint = self.torque_from_nn(nnff_setpoint_input)
        torque_from_measurement = self.torque_from_nn(nnff_measurement_input)

        pid_log.error = float(torque_from_setpoint - torque_from_measurement)
        error_blend_factor = get_error_blend_factor(desired_lateral_accel, CS.vEgo, self.low_speed_error_blend)
        if error_blend_factor > 0.0:  # blend in stronger error response when in high lat accel
          nnff_error_input = [CS.vEgo, setpoint - measurement, lateral_jerk_setpoint - lateral_jerk_measurement, 0.0]
          torque_from_error = self.torque_from_nn(nnff_error_input)
          if sign(pid_log.error) == sign(torque_from_error) and abs(pid_log.error) < abs(torque_from_error):
            pid_log.error = float(pid_log.error * (1.0 - error_blend_factor) + torque_from_error * error_blend_factor)

        # compute feedforward (same as nn setpoint output)
        error = setpoint - measurement
        friction_input = self.lat_accel_friction_factor * error + self.lat_jerk_friction_factor * lookahead_lateral_jerk
        nn_input = [CS.vEgo, desired_lateral_accel, friction_input, roll] \
                   + past_lateral_accels_desired + future_planned_lateral_accels \
                   + past_rolls + future_rolls
        ff = self.torque_from_nn(nn_input)

        # apply friction override for cars with low NN friction response. FrogPilot adds the linear torque of
        # zero lateral acceleration here, which is 0 (no friction term), so the parity cars add nothing.
        if self.nn_friction_override and not self.nnff_frogpilot_parity:
          pid_log.error += self.torque_from_lateral_accel(LatControlInputs(0.0, 0.0, CS.vEgo, CS.aEgo), self.torque_params,
                                                          friction_input, lateral_accel_deadzone, friction_compensation=True, gravity_adjusted=False)
        nn_log = nn_input + nnff_setpoint_input + nnff_measurement_input

      else:
        gravity_adjusted_lateral_accel = desired_lateral_accel - roll_compensation
        torque_from_setpoint = self.torque_from_lateral_accel(LatControlInputs(setpoint, roll_compensation, CS.vEgo, CS.aEgo), self.torque_params,
                                                              lateral_jerk_setpoint, lateral_accel_deadzone, friction_compensation=self.use_nnff_lite, gravity_adjusted=False)
        torque_from_measurement = self.torque_from_lateral_accel(LatControlInputs(measurement, roll_compensation, CS.vEgo, CS.aEgo), self.torque_params,
                                                                 lateral_jerk_measurement, lateral_accel_deadzone, friction_compensation=self.use_nnff_lite, gravity_adjusted=False)
        pid_log.error = float(torque_from_setpoint - torque_from_measurement)
        error = desired_lateral_accel - actual_lateral_accel
        if self.use_nnff_lite:
          friction_input = self.lat_accel_friction_factor * error + self.lat_jerk_friction_factor * lookahead_lateral_jerk
        else:
          friction_input = error
        # FrogPilot's fallback (model too short) feed-forward is the plain linear torque, without the friction term
        ff = self.torque_from_lateral_accel(LatControlInputs(gravity_adjusted_lateral_accel, roll_compensation, CS.vEgo, CS.aEgo), self.torque_params,
                                            friction_input, lateral_accel_deadzone, friction_compensation=not self.nnff_frogpilot_parity,
                                            gravity_adjusted=True)

      freeze_integrator = steer_limited_by_controls or CS.steeringPressed or CS.vEgo < 5
      if isinstance(self.pid, FrogPilotWindupPID):
        self.pid.p_filter_bypass = bool(CS.steeringPressed)   # no P filter while the driver steers
      output_torque = self.pid.update(pid_log.error,
                                      feedforward=ff,
                                      speed=CS.vEgo,
                                      freeze_integrator=freeze_integrator)

      pid_log.active = True
      pid_log.p = float(self.pid.p)
      pid_log.i = float(self.pid.i)
      pid_log.d = float(self.pid.d)
      pid_log.f = float(self.pid.f)
      pid_log.output = float(-output_torque)
      pid_log.actualLateralAccel = float(actual_lateral_accel)
      pid_log.desiredLateralAccel = float(desired_lateral_accel)
      pid_log.saturated = bool(self._check_saturation(self.steer_max - abs(output_torque) < 1e-3, CS, steer_limited_by_controls, curvature_limited))
      #if nn_log is not None:
      #  pid_log.nnLog = nn_log

    # TODO left is positive in this convention
    return -output_torque,angle_steers_des, pid_log
