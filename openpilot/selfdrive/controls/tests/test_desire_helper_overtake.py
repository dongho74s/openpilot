"""Tests for the automatic left-lane overtake in DesireHelper.

The overtake triggers when the lead car is at least 20 km/h slower than the
cruise set speed: it announces with the existing lane-change notification
(left blinker), waits 2 seconds, then arms the normal auto-lane-change path.
A solid centre line (left lane_change_available False) blocks the overtake
before it is even announced.

The return (TestAutoOvertakeReturn) covers the trip back to the original
(right) lane after an overtake: once the overtaken car is judged passed
(lead gone/faster + model right zone clear + dead-reckoned advance + 3 s in
the left lane), it waits LaneChangeDelay, announces with the right blinker
for 2 s, arms the auto-lane-change path, and hands over to the FSM. Any
doubt — solid right line, a car seen on the right, driver input, ATC, or the
30 s pass timeout — keeps the car in the left lane.
"""
import sys
from types import ModuleType, SimpleNamespace

# The compiled params extension is not built on this VM (pre-existing: the
# stock test_desire_helper.py fails the same way). Stub it so DesireHelper
# can be imported here; the overtake logic itself does not touch Params.
if "openpilot.common.params_pyx" not in sys.modules:
  _params_pyx = ModuleType("openpilot.common.params_pyx")

  class _Stub:
    pass

  _params_pyx.Params = _Stub
  _params_pyx.ParamKeyFlag = _Stub
  _params_pyx.ParamKeyType = _Stub
  _params_pyx.UnknownKeyName = Exception
  sys.modules["openpilot.common.params_pyx"] = _params_pyx

from openpilot.cereal import log
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.desire_helper import (
  DesireHelper,
  OVERTAKE_ANNOUNCE_S,
  OVERTAKE_RETURN_ANNOUNCE_S,
  OVERTAKE_RETURN_ARMED_TIMEOUT_S,
  OVERTAKE_RETURN_COOLDOWN_S,
  OVERTAKE_RETURN_MIN_LEFT_LANE_S,
  OVERTAKE_RETURN_TIMEOUT_S,
)
from openpilot.selfdrive.controls.lib.desire_lib.constants import LaneChangeDirection


def make_car_state(v_ego_kph=100.0, v_cruise_kph=100.0, left_blinker=False, right_blinker=False,
                   steering_pressed=False, steering_torque=0.0, trailer_connected=False,
                   right_blindspot=False):
  return SimpleNamespace(
    vEgo=v_ego_kph / 3.6,
    vCruise=v_cruise_kph,
    leftBlinker=left_blinker,
    rightBlinker=right_blinker,
    steeringPressed=steering_pressed,
    steeringTorque=steering_torque,
    trailerConnected=trailer_connected,
    rightBlindspot=right_blindspot,
  )


def make_model_data(right_zone_lead=False, lane_lines=True):
  """Fake ModelDataV2: lane lines (0 left outer, 1 left current, 2 right
  current, 3 right outer) and model leads in the right lane ahead."""
  if lane_lines:
    lines = [
      SimpleNamespace(y=[5.4]),    # left outer
      SimpleNamespace(y=[1.8]),    # left current
      SimpleNamespace(y=[-1.8]),   # right current
      SimpleNamespace(y=[-5.4]),   # right outer
    ]
  else:
    lines = []
  if right_zone_lead:
    leads = [SimpleNamespace(prob=0.9, x=[20.0], y=[-3.5])]
  else:
    leads = []
  return SimpleNamespace(laneLines=lines, leadsV3=leads)


def make_carrot_man(atc_type=""):
  return SimpleNamespace(atcType=atc_type)


def make_model_plan(stop_x_m=30.0, end_speed_kph=0.0, ego_kph=70.0, cruising=False):
  """Fake ModelDataV2 plan for the signal-stop guard: position.x ramps to the
  planned stop point (or keeps going when cruising), velocity.x eases from
  ego speed to the end speed, position.y stays on the lane centre."""
  n = 33
  if cruising:
    x = [float(i * 5.0) for i in range(n)]
    v = [ego_kph / 3.6] * n
  else:
    x = [stop_x_m * min(1.0, i / 10.0) for i in range(n)]
    v0 = ego_kph / 3.6
    v1 = end_speed_kph / 3.6
    v = [v0 + (v1 - v0) * (i / (n - 1)) for i in range(n)]
  y = [0.0] * n
  return SimpleNamespace(
    position=SimpleNamespace(x=x, y=y),
    velocity=SimpleNamespace(x=v),
  )


def make_radar_state(lead_speed_kph=None, lead_dist_m=50.0):
  if lead_speed_kph is None:
    lead = SimpleNamespace(status=False, vLead=0.0, dRel=0.0)
  else:
    lead = SimpleNamespace(status=True, vLead=lead_speed_kph / 3.6, dRel=lead_dist_m)
  return SimpleNamespace(leadOne=lead)


class TestAutoOvertake:
  def setup_method(self):
    self.helper = DesireHelper()
    self.helper._update_params_periodic = lambda: None
    # Left lane available (dashed line, clear) by default.
    self.helper.left.lane_change_available = True
    self.helper.left.lane_change_available_geom = True
    self.helper.lane_change_state = log.LaneChangeState.off

  def overtake(self, car, radar, lateral_active=True, signal_stop=False):
    self.helper._update_auto_overtake(car, radar, lateral_active, signal_stop=signal_stop)

  def signal_stop(self, plan, car, radar):
    return self.helper._model_signal_stop(plan, car, radar)

  def test_triggers_when_lead_is_20kph_slower(self):
    car = make_car_state(v_ego_kph=100.0, v_cruise_kph=100.0)
    radar = make_radar_state(lead_speed_kph=80.0)
    self.overtake(car, radar)
    assert self.helper._overtake_state == "announcing"
    assert self.helper.carrot_blinker_state == 1  # BLINKER_LEFT
    assert self.helper.carrot_lane_change_count > 0

  def test_no_trigger_when_lead_only_10kph_slower(self):
    car = make_car_state(v_ego_kph=100.0, v_cruise_kph=100.0)
    radar = make_radar_state(lead_speed_kph=90.0)
    self.overtake(car, radar)
    assert self.helper._overtake_state == "off"

  def test_no_trigger_without_lead(self):
    car = make_car_state(v_ego_kph=100.0, v_cruise_kph=100.0)
    radar = make_radar_state(lead_speed_kph=None)
    self.overtake(car, radar)
    assert self.helper._overtake_state == "off"

  def test_no_trigger_when_left_line_is_solid(self):
    # Solid centre line: left lane change unavailable.
    self.helper.left.lane_change_available = False
    car = make_car_state(v_ego_kph=100.0, v_cruise_kph=100.0)
    radar = make_radar_state(lead_speed_kph=70.0)
    self.overtake(car, radar)
    assert self.helper._overtake_state == "off"
    assert self.helper.carrot_lane_change_count == 0

  def test_no_trigger_when_ego_too_slow(self):
    car = make_car_state(v_ego_kph=50.0, v_cruise_kph=100.0)
    radar = make_radar_state(lead_speed_kph=70.0)
    self.overtake(car, radar)
    assert self.helper._overtake_state == "off"

  def test_no_trigger_when_not_lateral_active(self):
    car = make_car_state(v_ego_kph=100.0, v_cruise_kph=100.0)
    radar = make_radar_state(lead_speed_kph=70.0)
    self.overtake(car, radar, lateral_active=False)
    assert self.helper._overtake_state == "off"

  def test_arms_auto_lane_change_after_2s_announcement(self):
    car = make_car_state(v_ego_kph=100.0, v_cruise_kph=100.0)
    radar = make_radar_state(lead_speed_kph=70.0)
    self.overtake(car, radar)
    assert self.helper._overtake_state == "announcing"
    # Run for the announcement duration.
    frames = int(OVERTAKE_ANNOUNCE_S / DT_MDL) + 1
    for _ in range(frames):
      self.overtake(car, radar)
    assert self.helper.auto_lane_change_enable is True
    assert self.helper._overtake_state == "armed"
    # Once the FSM starts the lane change, the overtake hands over.
    self.helper.lane_change_state = log.LaneChangeState.laneChangeStarting
    self.overtake(car, radar)
    assert self.helper._overtake_state == "off"

  def test_armed_times_out_if_lane_change_never_starts(self):
    from openpilot.selfdrive.controls.lib.desire_helper import OVERTAKE_ARMED_TIMEOUT_S
    car = make_car_state(v_ego_kph=100.0, v_cruise_kph=100.0)
    radar = make_radar_state(lead_speed_kph=70.0)
    self.overtake(car, radar)
    frames = int(OVERTAKE_ANNOUNCE_S / DT_MDL) + 1
    for _ in range(frames):
      self.overtake(car, radar)
    assert self.helper._overtake_state == "armed"
    # The lane change never starts: after the armed timeout the arming is
    # withdrawn instead of announcing forever. Check the moment it times
    # out (the still-slow lead may re-trigger a fresh attempt afterwards).
    timed_out = False
    for _ in range(int(OVERTAKE_ARMED_TIMEOUT_S / DT_MDL) + 5):
      self.overtake(car, radar)
      if self.helper._overtake_state == "off":
        timed_out = True
        break
    assert timed_out
    assert self.helper.auto_lane_change_enable is False

  def test_cancels_when_lead_speeds_up_during_announcement(self):
    car = make_car_state(v_ego_kph=100.0, v_cruise_kph=100.0)
    radar_slow = make_radar_state(lead_speed_kph=70.0)
    radar_fast = make_radar_state(lead_speed_kph=95.0)
    self.overtake(car, radar_slow)
    assert self.helper._overtake_state == "announcing"
    # Lead speeds up: the overtake must cancel before any lane change.
    self.overtake(car, radar_fast)
    assert self.helper._overtake_state == "off"
    assert self.helper.auto_lane_change_enable is False

  def test_cancels_when_solid_line_appears_during_announcement(self):
    car = make_car_state(v_ego_kph=100.0, v_cruise_kph=100.0)
    radar = make_radar_state(lead_speed_kph=70.0)
    self.overtake(car, radar)
    assert self.helper._overtake_state == "announcing"
    # Solid centre line appears: cancel, do not start the lane change.
    self.helper.left.lane_change_available = False
    self.overtake(car, radar)
    assert self.helper._overtake_state == "off"
    assert self.helper.auto_lane_change_enable is False

  def test_cancels_when_driver_uses_blinker(self):
    car = make_car_state(v_ego_kph=100.0, v_cruise_kph=100.0)
    radar = make_radar_state(lead_speed_kph=70.0)
    self.overtake(car, radar)
    assert self.helper._overtake_state == "announcing"
    car_driver = make_car_state(v_ego_kph=100.0, v_cruise_kph=100.0, right_blinker=True)
    self.overtake(car_driver, radar)
    assert self.helper._overtake_state == "off"

  def test_does_not_retrigger_while_lane_change_is_busy(self):
    car = make_car_state(v_ego_kph=100.0, v_cruise_kph=100.0)
    radar = make_radar_state(lead_speed_kph=70.0)
    self.helper.lane_change_state = log.LaneChangeState.laneChangeStarting
    self.overtake(car, radar)
    assert self.helper._overtake_state == "off"


class TestAutoOvertakeSignalStop:
  """Signal-stop guard: a stopped car at a signal (red / solid yellow light,
  stop line) is never overtaken, while a stopped car on the open road still
  is. The guard reads the model's stop plan: it blocks only when the model
  plans to stop well before the lead car, not when it plans to stop behind
  it."""

  def setup_method(self):
    self.helper = DesireHelper()
    self.helper._update_params_periodic = lambda: None
    self.helper.left.lane_change_available = True
    self.helper.left.lane_change_available_geom = True
    self.helper.lane_change_state = log.LaneChangeState.off

  def overtake(self, car, radar, lateral_active=True, signal_stop=False):
    self.helper._update_auto_overtake(car, radar, lateral_active, signal_stop=signal_stop)

  def signal_stop(self, plan, car, radar):
    return self.helper._model_signal_stop(plan, car, radar)

  def test_signal_stop_when_model_plans_stop_before_lead(self):
    # Stopped lead 50 m ahead, model plans to stop at 30 m (stop line well
    # before the queue): a signal stop.
    car = make_car_state(v_ego_kph=70.0, v_cruise_kph=100.0)
    radar = make_radar_state(lead_speed_kph=0.0, lead_dist_m=50.0)
    plan = make_model_plan(stop_x_m=30.0, end_speed_kph=0.0, ego_kph=70.0)
    assert self.signal_stop(plan, car, radar) is True

  def test_no_signal_stop_when_model_stops_behind_lead(self):
    # Same stopped lead, but the model plans to stop 1.5 m behind it: lead
    # following on the open road, not a signal stop.
    car = make_car_state(v_ego_kph=70.0, v_cruise_kph=100.0)
    radar = make_radar_state(lead_speed_kph=0.0, lead_dist_m=50.0)
    plan = make_model_plan(stop_x_m=48.5, end_speed_kph=0.0, ego_kph=70.0)
    assert self.signal_stop(plan, car, radar) is False

  def test_no_signal_stop_when_model_not_stopping(self):
    # Model keeps cruising past the lead: no stop planned at all.
    car = make_car_state(v_ego_kph=70.0, v_cruise_kph=100.0)
    radar = make_radar_state(lead_speed_kph=0.0, lead_dist_m=50.0)
    plan = make_model_plan(ego_kph=70.0, cruising=True)
    assert self.signal_stop(plan, car, radar) is False

  def test_no_signal_stop_when_ego_above_82(self):
    # Same signal-stop plan, but ego is above the model's stop band: the
    # guard is blind up here (the announcement cancel is the safety net).
    car = make_car_state(v_ego_kph=90.0, v_cruise_kph=100.0)
    radar = make_radar_state(lead_speed_kph=0.0, lead_dist_m=50.0)
    plan = make_model_plan(stop_x_m=30.0, end_speed_kph=0.0, ego_kph=90.0)
    assert self.signal_stop(plan, car, radar) is False

  def test_no_trigger_when_signal_stop_ahead(self):
    car = make_car_state(v_ego_kph=70.0, v_cruise_kph=100.0)
    radar = make_radar_state(lead_speed_kph=0.0, lead_dist_m=50.0)
    self.overtake(car, radar, signal_stop=True)
    assert self.helper._overtake_state == "off"
    assert self.helper.carrot_lane_change_count == 0

  def test_triggers_for_stopped_lead_without_signal(self):
    # The core case the revert asked for: a stopped car with no signal stop
    # ahead is still overtaken.
    car = make_car_state(v_ego_kph=70.0, v_cruise_kph=100.0)
    radar = make_radar_state(lead_speed_kph=0.0, lead_dist_m=50.0)
    self.overtake(car, radar, signal_stop=False)
    assert self.helper._overtake_state == "announcing"

  def test_cancels_announcement_when_signal_appears(self):
    # Announcing, then the model starts planning a signal stop: cancel
    # before any lane change starts.
    car = make_car_state(v_ego_kph=70.0, v_cruise_kph=100.0)
    radar = make_radar_state(lead_speed_kph=0.0, lead_dist_m=50.0)
    self.overtake(car, radar, signal_stop=False)
    assert self.helper._overtake_state == "announcing"
    self.overtake(car, radar, signal_stop=True)
    assert self.helper._overtake_state == "off"
    assert self.helper.carrot_lane_change_count == 0


class TestAutoOvertakeReturn:
  def setup_method(self):
    self.helper = DesireHelper()
    self.helper._update_params_periodic = lambda: None
    # Both lanes available (dashed lines, clear) by default.
    self.helper.left.lane_change_available = True
    self.helper.left.lane_change_available_geom = True
    self.helper.right.lane_change_available = True
    self.helper.right.lane_change_available_geom = True
    self.helper.lane_change_state = log.LaneChangeState.off
    self.carrot_man = make_carrot_man()

  def drive_to_overtaking(self):
    """Full flow: overtake trigger -> arm -> FSM handoff -> crossing.

    Ends in the return's "overtaking" state with the entry lane change
    finished (lane_change_state off). The overtaken lead is 70 km/h at 50 m,
    ego 100 km/h, so the dead-reckoned advance grows at 8.33 m/s and the
    pass needs 58 m (50 m anchor + 8 m clearance).
    """
    car = make_car_state(v_ego_kph=100.0, v_cruise_kph=100.0)
    radar = make_radar_state(lead_speed_kph=70.0, lead_dist_m=50.0)
    model = make_model_data()
    self.helper._update_auto_overtake(car, radar, True)
    assert self.helper._overtake_state == "announcing"
    frames = int(OVERTAKE_ANNOUNCE_S / DT_MDL) + 1
    for _ in range(frames):
      self.helper._update_auto_overtake(car, radar, True)
    assert self.helper._overtake_state == "armed"
    # FSM starts the left lane change: the overtake hands over and the
    # return monitor arms.
    self.helper.lane_change_state = log.LaneChangeState.laneChangeStarting
    self.helper.lane_change_direction = LaneChangeDirection.left
    self.helper._update_auto_overtake(car, radar, True)
    assert self.helper._overtake_state == "off"
    assert self.helper._return_pending is True
    # Centre crosses into the left lane: start watching the pass.
    self.helper.crossing.since = 0.0
    self.helper._update_overtake_return(car, model, radar, True, self.carrot_man)
    assert self.helper._return_state == "overtaking"
    # Entry lane change finished.
    self.helper.lane_change_state = log.LaneChangeState.off
    return car, radar, model

  def drive_to_announcing_right(self, delay_s=0.0):
    """drive_to_overtaking, then confirm the pass and reach "announcing_right".

    The pass is confirmed by the lead disappearing (no-lead radar) after the
    dead-reckoned advance already covers the anchor + clearance.
    """
    self.helper.laneChangeDelay = delay_s
    car, radar, model = self.drive_to_overtaking()
    # Accumulate the advance while the slow lead is still ahead (A not met).
    for _ in range(150):
      self.helper._update_overtake_return(car, model, radar, True, self.carrot_man)
      assert self.helper._return_state == "overtaking"
    # Lead gone: A met, B/C/D already met -> pass confirmed -> "delay".
    radar_clear = make_radar_state(lead_speed_kph=None)
    self.helper._update_overtake_return(car, model, radar_clear, True, self.carrot_man)
    assert self.helper._return_state == "delay"
    # LaneChangeDelay elapses -> right-blinker announcement.
    frames = int(delay_s / DT_MDL) + 2
    for _ in range(frames):
      self.helper._update_overtake_return(car, model, radar_clear, True, self.carrot_man)
    assert self.helper._return_state == "announcing_right"
    assert self.helper.carrot_blinker_state == 2  # BLINKER_RIGHT
    return car, radar_clear, model

  def arm_return(self, lead_d0=50.0, lead_v0_kph=70.0):
    """Put the helper straight into "overtaking" for pass-condition tests."""
    h = self.helper
    h._return_state = "overtaking"
    h._return_lead_d0 = lead_d0
    h._return_lead_v0 = lead_v0_kph / 3.6
    h._return_rel_advance = 0.0
    h._return_zone_clear_timer = 0.0
    h._return_left_lane_timer = 0.0
    h._return_pending = False
    h._return_change_started = False
    h.lane_change_state = log.LaneChangeState.off

  def test_full_return_flow(self):
    car, radar, model = self.drive_to_announcing_right()
    # Announce for 2 s, then arm the auto-lane-change path.
    for _ in range(int(OVERTAKE_RETURN_ANNOUNCE_S / DT_MDL) + 1):
      self.helper._update_overtake_return(car, model, radar, True, self.carrot_man)
    assert self.helper._return_state == "armed_right"
    assert self.helper.auto_lane_change_enable is True
    # FSM starts the right lane change: hand over, cooldown starts.
    self.helper.lane_change_state = log.LaneChangeState.laneChangeStarting
    self.helper.lane_change_direction = LaneChangeDirection.right
    self.helper._update_overtake_return(car, model, radar, True, self.carrot_man)
    assert self.helper._return_state == "off"
    assert self.helper._return_cooldown_timer == OVERTAKE_RETURN_COOLDOWN_S

  def test_delay_waits_lane_change_delay(self):
    # LaneChangeDelay = 1 s: after the pass is confirmed the return must
    # wait a full second before announcing the right blinker.
    self.helper.laneChangeDelay = 1.0
    car, radar, model = self.drive_to_overtaking()
    for _ in range(150):
      self.helper._update_overtake_return(car, model, radar, True, self.carrot_man)
    radar_clear = make_radar_state(lead_speed_kph=None)
    self.helper._update_overtake_return(car, model, radar_clear, True, self.carrot_man)
    assert self.helper._return_state == "delay"
    # Half the delay: still waiting, no right blinker yet.
    for _ in range(int(0.5 / DT_MDL)):
      self.helper._update_overtake_return(car, model, radar_clear, True, self.carrot_man)
    assert self.helper._return_state == "delay"
    assert self.helper.carrot_blinker_state != 2
    # Delay elapses: announcement starts.
    for _ in range(int(0.6 / DT_MDL) + 2):
      self.helper._update_overtake_return(car, model, radar_clear, True, self.carrot_man)
    assert self.helper._return_state == "announcing_right"

  def test_pass_needs_lead_gone_or_faster(self):
    # A: the lead ahead must no longer be the overtaken car. With the same
    # slow lead still ahead the pass never confirms, even with B/C/D met.
    self.arm_return()
    self.helper._return_rel_advance = 100.0
    self.helper._return_zone_clear_timer = 2.0
    self.helper._return_left_lane_timer = 5.0
    car = make_car_state(v_ego_kph=100.0, v_cruise_kph=100.0)
    model = make_model_data()
    radar_slow = make_radar_state(lead_speed_kph=70.0)
    for _ in range(10):
      self.helper._update_overtake_return(car, model, radar_slow, True, self.carrot_man)
    assert self.helper._return_state == "overtaking"
    # The lead speeds up past the latched speed + 2 m/s: A met -> pass.
    radar_fast = make_radar_state(lead_speed_kph=90.0)
    self.helper._update_overtake_return(car, model, radar_fast, True, self.carrot_man)
    assert self.helper._return_state == "delay"

  def test_right_zone_car_blocks_pass(self):
    # B: a car seen in the model right zone blocks the pass until the zone
    # has been clear for 1 s. LaneChangeDelay = 1 s so the "delay" state is
    # observable instead of advancing to the announcement at once.
    self.helper.laneChangeDelay = 1.0
    self.arm_return()
    self.helper._return_rel_advance = 100.0
    self.helper._return_left_lane_timer = 5.0
    car = make_car_state(v_ego_kph=100.0, v_cruise_kph=100.0)
    radar_clear = make_radar_state(lead_speed_kph=None)
    model_blocked = make_model_data(right_zone_lead=True)
    for _ in range(40):
      self.helper._update_overtake_return(car, model_blocked, radar_clear, True, self.carrot_man)
    assert self.helper._return_state == "overtaking"
    assert self.helper._return_zone_clear_timer == 0.0
    # Zone clears: after 1 s the pass confirms.
    model_clear = make_model_data()
    for _ in range(int(1.0 / DT_MDL) + 2):
      self.helper._update_overtake_return(car, model_clear, radar_clear, True, self.carrot_man)
    assert self.helper._return_state == "delay"

  def test_right_zone_uses_fallback_without_lane_lines(self):
    # Without lane lines the zone falls back to one lane width to the right.
    model = make_model_data(right_zone_lead=True, lane_lines=False)
    assert self.helper._return_right_zone_occupied(model) is True
    model_clear = make_model_data(lane_lines=False)
    assert self.helper._return_right_zone_occupied(model_clear) is False

  def test_no_return_when_pass_never_confirmed(self):
    # 30 s without a confirmed pass: give up and stay in the left lane.
    car, radar, model = self.drive_to_overtaking()
    timed_out = False
    for _ in range(int(OVERTAKE_RETURN_TIMEOUT_S / DT_MDL) + 10):
      self.helper._update_overtake_return(car, model, radar, True, self.carrot_man)
      if self.helper._return_state == "off":
        timed_out = True
        break
    assert timed_out
    assert self.helper._return_pending is False

  def test_cancels_when_right_lane_solid(self):
    car, radar, model = self.drive_to_announcing_right()
    # Solid centre line on the right: cancel, do not start the return.
    # The return's own right-blinker notification is taken down.
    self.helper.right.lane_change_available = False
    self.helper._update_overtake_return(car, model, radar, True, self.carrot_man)
    assert self.helper._return_state == "off"
    assert self.helper.carrot_lane_change_count == 0

  def test_hands_over_when_fsm_starts_right_change_during_announcement(self):
    # The FSM can start the right lane change during "announcing_right"
    # (the announcement refreshes carrot_lane_change_count, which forces
    # the FSM trigger, and the FSM's own lane_change_delay gate opens
    # first). That is the return's own change, not a foreign one: hand over
    # and start the cooldown instead of cancelling.
    car, radar, model = self.drive_to_announcing_right()
    assert self.helper._return_state == "announcing_right"
    self.helper.lane_change_state = log.LaneChangeState.laneChangeStarting
    self.helper.lane_change_direction = LaneChangeDirection.right
    self.helper._update_overtake_return(car, model, radar, True, self.carrot_man)
    assert self.helper._return_state == "off"
    assert self.helper._return_cooldown_timer == OVERTAKE_RETURN_COOLDOWN_S

  def test_cancels_when_driver_uses_blinker(self):
    car, radar, model = self.drive_to_overtaking()
    car_driver = make_car_state(v_ego_kph=100.0, v_cruise_kph=100.0, right_blinker=True)
    self.helper._update_overtake_return(car_driver, model, radar, True, self.carrot_man)
    assert self.helper._return_state == "off"

  def test_cancels_when_driver_steers_against_return(self):
    car, radar, model = self.drive_to_overtaking()
    # Steering pressed with leftward torque opposes the right return.
    car_driver = make_car_state(v_ego_kph=100.0, v_cruise_kph=100.0,
                                steering_pressed=True, steering_torque=1.0)
    self.helper._update_overtake_return(car_driver, model, radar, True, self.carrot_man)
    assert self.helper._return_state == "off"

  def test_cancels_when_auto_overtake_off(self):
    car, radar, model = self.drive_to_overtaking()
    self.helper.auto_overtake_enabled = False
    self.helper._update_overtake_return(car, model, radar, True, self.carrot_man)
    assert self.helper._return_state == "off"
    # And a disabled toggle never triggers a fresh overtake either.
    radar_slow = make_radar_state(lead_speed_kph=70.0)
    self.helper._update_auto_overtake(car, radar_slow, True)
    assert self.helper._overtake_state == "off"

  def test_bsm_blocks_return(self):
    # Rear-side check: a right blind-spot signal blocks the return even
    # with the pass confirmed. Today the signal is always clear on this car;
    # the code is live so a future low-speed CAN 0x142 feed works unchanged.
    car, radar, model = self.drive_to_announcing_right()
    car_bsm = make_car_state(v_ego_kph=100.0, v_cruise_kph=100.0, right_blindspot=True)
    self.helper._update_overtake_return(car_bsm, model, radar, True, self.carrot_man)
    assert self.helper._return_state == "off"

  def test_bsd_hold_blocks_return(self):
    car, radar, model = self.drive_to_announcing_right()
    self.helper.right.bsd_hold_counter = 5
    self.helper._update_overtake_return(car, model, radar, True, self.carrot_man)
    assert self.helper._return_state == "off"

  def test_cooldown_blocks_reovertake(self):
    car, radar, model = self.drive_to_announcing_right()
    for _ in range(int(OVERTAKE_RETURN_ANNOUNCE_S / DT_MDL) + 1):
      self.helper._update_overtake_return(car, model, radar, True, self.carrot_man)
    assert self.helper._return_state == "armed_right"
    self.helper.lane_change_state = log.LaneChangeState.laneChangeStarting
    self.helper.lane_change_direction = LaneChangeDirection.right
    self.helper._update_overtake_return(car, model, radar, True, self.carrot_man)
    assert self.helper._return_state == "off"
    # The right change finished; the same slow lead must not re-trigger an
    # overtake during the 15 s cooldown.
    self.helper.lane_change_state = log.LaneChangeState.off
    radar_slow = make_radar_state(lead_speed_kph=70.0)
    self.helper._update_auto_overtake(car, radar_slow, True)
    assert self.helper._overtake_state == "off"
    # Cooldown elapses: the overtake may trigger again.
    for _ in range(int(OVERTAKE_RETURN_COOLDOWN_S / DT_MDL) + 5):
      self.helper._update_overtake_return(car, model, radar_slow, True, self.carrot_man)
    self.helper._update_auto_overtake(car, radar_slow, True)
    assert self.helper._overtake_state == "announcing"

  def test_no_return_for_driver_lane_change(self):
    # A left lane change the driver started themselves never arms a return.
    car = make_car_state(v_ego_kph=100.0, v_cruise_kph=100.0)
    radar = make_radar_state(lead_speed_kph=70.0)
    self.helper.lane_change_state = log.LaneChangeState.laneChangeStarting
    self.helper.lane_change_direction = LaneChangeDirection.left
    self.helper._update_auto_overtake(car, radar, True)
    assert self.helper._overtake_state == "off"
    assert self.helper._return_pending is False

  def test_no_return_when_atc_owns_the_change(self):
    # An ATC navigation lane change is never followed by a return.
    car = make_car_state(v_ego_kph=100.0, v_cruise_kph=100.0)
    radar = make_radar_state(lead_speed_kph=70.0)
    self.helper._update_auto_overtake(car, radar, True)
    assert self.helper._overtake_state == "announcing"
    frames = int(OVERTAKE_ANNOUNCE_S / DT_MDL) + 1
    for _ in range(frames):
      self.helper._update_auto_overtake(car, radar, True)
    assert self.helper._overtake_state == "armed"
    self.helper.atc_type = "atc left"
    self.helper.lane_change_state = log.LaneChangeState.laneChangeStarting
    self.helper.lane_change_direction = LaneChangeDirection.left
    self.helper._update_auto_overtake(car, radar, True)
    assert self.helper._overtake_state == "off"
    assert self.helper._return_pending is False

  def test_armed_times_out_if_right_change_never_starts(self):
    car, radar, model = self.drive_to_announcing_right()
    for _ in range(int(OVERTAKE_RETURN_ANNOUNCE_S / DT_MDL) + 1):
      self.helper._update_overtake_return(car, model, radar, True, self.carrot_man)
    assert self.helper._return_state == "armed_right"
    # The right lane change never starts: after the armed timeout the
    # arming is withdrawn instead of announcing forever.
    timed_out = False
    for _ in range(int(OVERTAKE_RETURN_ARMED_TIMEOUT_S / DT_MDL) + 5):
      self.helper._update_overtake_return(car, model, radar, True, self.carrot_man)
      if self.helper._return_state == "off":
        timed_out = True
        break
    assert timed_out
    assert self.helper.auto_lane_change_enable is False
