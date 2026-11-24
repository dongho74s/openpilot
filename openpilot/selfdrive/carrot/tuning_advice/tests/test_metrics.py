import numpy as np

from openpilot.selfdrive.carrot.tuning_advice.extract import COLS, C
from openpilot.selfdrive.carrot.tuning_advice import metrics as M


def drive(n=6000, v=20.0, des=None, act=None, sent=None, pressed=None, mdl=None):
  a = np.zeros((n, len(COLS)))
  a[:, C["t"]] = np.arange(n) * 0.01
  a[:, C["v"]] = v
  a[:, C["latActive"]] = 1
  des = np.zeros(n) if des is None else des
  a[:, C["des"]] = des
  a[:, C["desCurv"]] = des / v ** 2
  a[:, C["mdlCurv"]] = (des if mdl is None else mdl) / v ** 2
  a[:, C["act"]] = des if act is None else act
  a[:, C["sent"]] = 0.0 if sent is None else sent
  a[:, C["pressed"]] = 0 if pressed is None else pressed
  a[:, C["laneCurv"]] = a[:, C["desCurv"]]
  a[:, C["laneOff"]] = np.nan
  a[:, C["latDelay"]] = np.nan
  return a


def curve(n=6000, peak=2.0):
  t = np.arange(n) * 0.01
  return peak * np.sin(2 * np.pi * t / 20.0)


def test_perfect_tracking_is_a_full_match():
  m = M.drive_metrics([(drive(des=curve()), C)], 3.0)
  assert m["lat_minutes"] == 1.0
  assert m["match_pct"] == 100.0 and m["match_curve_pct"] == 100.0 and m["match_straight_pct"] == 100.0
  assert m["err"] == 0.0


def test_match_is_the_share_of_the_request_the_car_met():
  d = curve()
  m = M.drive_metrics([(drive(des=d, act=0.8 * d), C)], 3.0)
  assert abs(m["match_pct"] - 80.0) < 0.5           # 1 - sum|0.2 d| / sum|d|
  # inside the 0.3 m/s^2 / 20 % tolerance everywhere (0.2 |d| <= max(0.3, 0.2 |d|))
  assert m["match_curve_pct"] == 100.0
  m = M.drive_metrics([(drive(des=d, act=0.5 * d), C)], 3.0)
  assert abs(m["match_pct"] - 50.0) < 0.5
  assert m["match_curve_pct"] < 10.0                 # half the demand is outside the tolerance in curves


def test_driver_time_is_left_out_of_the_match():
  d = curve()
  act = d.copy()
  pressed = np.zeros(len(d))
  pressed[1000:3000] = 1
  act[1000:3000] = -d[1000:3000]                     # whatever the car did while the driver steered
  m = M.drive_metrics([(drive(des=d, act=act, pressed=pressed), C)], 3.0)
  assert m["match_pct"] == 100.0
  assert 30.0 < m["pressed_pct"] < 40.0


def test_too_little_steering_gives_no_match():
  m = M.drive_metrics([(drive(n=200, des=curve(200)), C)], 3.0)
  assert m["lat_minutes"] == 0.0 and m["match_pct"] is None and m["match_curve_pct"] is None


def test_torque_let_go_at_the_limit_is_counted():
  sent = np.zeros(6000)
  for k in range(1000, 6000, 1000):
    sent[k:k + 100] = 1.0
    sent[k + 100:k + 200] = 0.5
  m = M.drive_metrics([(drive(des=curve(), sent=sent), C)], 3.0)
  assert m["release_per_min"] == 5.0


def test_takeover_direction_against_openpilot():
  sent = np.full(6000, 0.4)
  pressed = np.zeros(6000)
  pressed[3000:3100] = 1
  a = drive(des=curve(), sent=sent, pressed=pressed)
  a[3000:3100, C["drvTq"]] = -1.0                    # driver turns against openpilot's torque
  m = M.drive_metrics([(a, C)], 3.0)
  assert m["takeovers"] == {"against": 1, "with": 0, "neutral": 0}


def test_demand_cut_by_the_lateral_accel_limit():
  d = np.full(6000, 3.0)
  m = M.drive_metrics([(drive(des=d, mdl=np.full(6000, 3.6)), C)], 3.0)
  assert m["clip_seconds"] > 50
  m = M.drive_metrics([(drive(des=d, mdl=np.full(6000, 3.6)), C)], 4.0)
  assert m["clip_seconds"] == 0.0
