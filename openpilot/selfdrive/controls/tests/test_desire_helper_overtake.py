"""Tests for the automatic left-lane overtake in DesireHelper.

The overtake triggers when the lead car is at least 20 km/h slower than the
cruise set speed: it announces with the existing lane-change notification
(left blinker), waits 2 seconds, then arms the normal auto-lane-change path.
A solid centre line (left lane_change_available False) blocks the overtake
before it is even announced.
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
)


def make_car_state(v_ego_kph=100.0, v_cruise_kph=100.0, left_blinker=False, right_blinker=False):
  return SimpleNamespace(
    vEgo=v_ego_kph / 3.6,
    vCruise=v_cruise_kph,
    leftBlinker=left_blinker,
    rightBlinker=right_blinker,
  )


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

