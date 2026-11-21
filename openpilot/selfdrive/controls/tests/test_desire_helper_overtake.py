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

  def overtake(self, car, radar, lateral_active=True):
    self.helper._update_auto_overtake(car, radar, lateral_active)

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
