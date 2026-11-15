"""Curve speed from the Navdy route preview (TMAP / Naver, 2026-11-15)."""
import math
from types import SimpleNamespace as NS

import pytest

from openpilot.selfdrive.carrot.curve_speed import APPROACH_DECEL, NO_LIMIT_KPH, RELEASE_HOLD, RELEASE_RATE_KPH
from openpilot.selfdrive.carrot.route_preview_speed import RoutePreviewCurveSpeed, preview_curve_speed
from openpilot.selfdrive.navdy.navigation import FeedbackTracker, preview_points


def route(straight_m=200.0, radius=100.0, left=True, total=400.0, step=5.0, heading=0.0):
  """Points every `step` m: `straight_m` along `heading` (rad from north), then a constant-radius arc."""
  points, x, y, h = [(0.0, 0.0)], 0.0, 0.0, heading
  travelled = 0.0
  while travelled + step <= total + 1e-9:
    if travelled >= straight_m and radius:
      h += (-1 if left else 1) * step / radius   # heading measured clockwise from north
    x += step * math.sin(h)
    y += step * math.cos(h)
    travelled += step
    points.append((round(x, 2), round(y, 2)))
  return tuple(points)


def kph(radius, budget=1.9):
  return math.sqrt(budget * radius) * 3.6


def test_straight_route_has_no_limit():
  result = preview_curve_speed(route(radius=0), 0.0, 20.0)
  assert result.approach_kph == NO_LIMIT_KPH and result.curve_kph == NO_LIMIT_KPH


@pytest.mark.parametrize('left', [True, False])
@pytest.mark.parametrize('heading', [0.0, 1.0, -2.5])
def test_curve_target_is_the_vision_physics_whatever_the_heading(left, heading):
  result = preview_curve_speed(route(straight_m=0, left=left, heading=heading), 0.0, 20.0)
  assert result.curve_kph == pytest.approx(kph(100), rel=0.02)
  assert result.direction == (1.0 if left else -1.0)
  # in the curve already: every chord is within the response distance (about 32 m at 20 m/s), so no approach credit
  assert result.distance <= 25.0 and result.approach_kph == pytest.approx(result.curve_kph)


def test_distance_relaxes_approach_with_the_same_envelope_as_vision():
  v = 20.0
  result = preview_curve_speed(route(straight_m=200), 0.0, v)
  assert result.curve_kph == pytest.approx(kph(100), rel=0.02)
  assert 185.0 <= result.distance <= 200.0
  response = v * (1.0 + APPROACH_DECEL / (2 * 0.8))
  expected = math.sqrt((result.curve_kph / 3.6) ** 2 + 2 * APPROACH_DECEL * (result.distance - response)) * 3.6
  assert result.approach_kph == pytest.approx(expected)
  assert result.approach_kph > v * 3.6   # 72 km/h, 195 m before a 100 m radius curve: no slowing needed yet
  nearer = preview_curve_speed(route(straight_m=100), 0.0, v)
  assert nearer.approach_kph < v * 3.6   # 100 m before it: slowing (vision sees only about 130 m of path)


def test_age_counts_the_distance_already_driven():
  fresh = preview_curve_speed(route(straight_m=200), 0.0, 20.0)
  aged = preview_curve_speed(route(straight_m=200), 1.0, 20.0)
  assert aged.distance == pytest.approx(fresh.distance - 20.0)
  assert aged.approach_kph < fresh.approach_kph and aged.curve_kph == fresh.curve_kph


def test_a_passed_curve_no_longer_limits():
  result = preview_curve_speed(route(straight_m=0, total=60, radius=100), 10.0, 20.0)   # 200 m driven, 60 m route
  assert result.approach_kph == NO_LIMIT_KPH


def test_sensitivity_and_floor_follow_the_curve_settings():
  tight = route(straight_m=0, radius=20)
  assert preview_curve_speed(tight, 0.0, 10.0, lower_limit_kph=30).curve_kph == pytest.approx(30.0)
  wide = route(straight_m=0, radius=150)
  base = preview_curve_speed(wide, 0.0, 20.0, sensitivity=1.0).curve_kph
  assert preview_curve_speed(wide, 0.0, 20.0, sensitivity=0.8).curve_kph == pytest.approx(base * math.sqrt(1 / 0.8))
  assert preview_curve_speed(wide, 0.0, 20.0, sensitivity=1.2).curve_kph == pytest.approx(base * math.sqrt(1 / 1.2))


def test_one_bad_vertex_is_rejected_by_the_median():
  points = list(route(radius=0))
  points[40] = (points[40][0] + 3.0, points[40][1])   # 3 m sideways kink in a straight road
  result = preview_curve_speed(tuple(points), 0.0, 20.0)
  assert result.approach_kph == NO_LIMIT_KPH


@pytest.mark.parametrize('args', [
  (route(), math.nan, 20.0), (route(), -1.0, 20.0), (route(), 0.0, -1.0), (route(), 0.0, math.inf), (None, 0.0, 20.0),
  (route()[:6], 0.0, 20.0),
])
def test_invalid_input_gives_no_result(args):
  assert preview_curve_speed(*args) is None


def test_release_is_rate_limited_and_missing_preview_holds():
  limiter = RoutePreviewCurveSpeed()
  result = preview_curve_speed(route(straight_m=0), 0.0, 20.0)
  assert limiter.update(result, 0.0) == pytest.approx(result.approach_kph)
  straight = preview_curve_speed(route(radius=0), 0.0, 20.0)
  assert limiter.update(straight, 0.1) == pytest.approx(result.approach_kph + RELEASE_RATE_KPH * 0.1)
  held = limiter.speed
  assert limiter.update(None, 0.2) == pytest.approx(held)
  assert limiter.update(None, 0.2 + RELEASE_HOLD - 0.01) == pytest.approx(held)
  assert limiter.update(None, 0.2 + RELEASE_HOLD + 0.1) > held
  assert limiter.update(result, 0.7) == pytest.approx(result.approach_kph)   # tightens at once


def packet(**overrides):
  value = {'version': 1, 'session': '12345678-1234-1234-1234-123456789012', 'sequence': 7, 'ageMs': 300,
           'observedAtMs': 1, 'routeId': 3, 'guidance': True, 'reason': 'ok', 'roadLimitKph': 60, 'bumpDistanceM': 0,
           'points': [list(p) for p in route()][:81]}
  value.update(overrides)
  return {'roadPreview': value}


def test_tracker_returns_a_fresh_guided_preview():
  tracker = FeedbackTracker()
  assert tracker.accept(packet(), 100.0, 100.0)
  preview = tracker.route_preview(100.5)
  assert preview is not None and len(preview.points) == 81
  assert preview.age == pytest.approx(0.8) and preview.key[1] == 7
  assert tracker.route_preview(103.0) is None   # older than 2.5 s


@pytest.mark.parametrize('overrides', [
  {'guidance': False}, {'reason': 'off_route_or_heading'}, {'points': []}, {'points': [[0, 0]] * 81},
  {'points': [[0, 0], [5, 0], [10, 0], [15, 0], [20, 0], [25, 0]]}, {'points': [[2, 0]] + [[2 + 5 * i, 0] for i in range(1, 20)]},
  {'points': [[0, 0]] + [[5 * i, 0] for i in range(1, 10)] + [[60, 0]]}, {'points': [[0, 0], [5, 0], ['x', 0]] + [[5 * i, 0] for i in range(3, 10)]},
  {'points': [[0, 0]] + [[5 * i, 0] for i in range(1, 90)]}, {'points': [[0, 0]] + [[0, 600]] * 9},
])
def test_tracker_rejects_unusable_previews(overrides):
  tracker = FeedbackTracker()
  assert tracker.accept(packet(**overrides), 100.0, 100.0)
  assert tracker.route_preview(100.1) is None


def test_points_round_trip_and_bounds():
  assert preview_points([[0, 0]] + [[5 * i, 0] for i in range(1, 7)]) == tuple((5.0 * i, 0.0) for i in range(7))
  assert preview_points([[0, 0]] + [[5 * i, True] for i in range(1, 7)]) is None


# --- carrot integration --------------------------------------------------------------------------------------------

def test_serv_offers_the_preview_as_route_in_route_modes_only(monkeypatch):
  from openpilot.selfdrive.carrot.tests.test_deceleration_candidates import make_serv, publish
  for mode, expected in ((0, False), (1, False), (2, True), (3, True), (4, True)):
    serv, sm, pm, sent = make_serv(monkeypatch, TurnSpeedControlMode=mode, MapTurnSpeedFactor=50)
    msg = publish(serv, sm, pm, sent, route_preview_speed=61.0, route_preview_target=45.0)
    if expected:
      assert msg.desiredSource == 'route' and msg.desiredSpeed == 61   # not scaled by MapTurnSpeedFactor (50 %)
      route_items = [c for c in msg.to_dict()['decelerationCandidates'] if c['source'] == 'route']
      assert route_items and route_items[0]['targetKph'] == 45.0
    else:
      assert msg.desiredSource != 'route'


def test_serv_ignores_no_limit_and_floors_at_the_curve_minimum(monkeypatch):
  from openpilot.selfdrive.carrot.tests.test_deceleration_candidates import make_serv, publish
  serv, sm, pm, sent = make_serv(monkeypatch, TurnSpeedControlMode=2, AutoCurveSpeedLowerLimit=30)
  assert publish(serv, sm, pm, sent, route_preview_speed=250.0).desiredSource != 'route'
  serv, sm, pm, sent = make_serv(monkeypatch, TurnSpeedControlMode=2, AutoCurveSpeedLowerLimit=30)
  msg = publish(serv, sm, pm, sent, route_preview_speed=-12.0)
  assert msg.desiredSource == 'route' and msg.desiredSpeed == 30
  serv, sm, pm, sent = make_serv(monkeypatch, TurnSpeedControlMode=2)
  assert publish(serv, sm, pm, sent, route_preview_speed=math.nan).desiredSource != 'route'


def test_carrot_man_runs_only_in_navdy_route_modes(monkeypatch):
  from openpilot.selfdrive.carrot import carrot_man
  man = object.__new__(carrot_man.CarrotMan)
  man.route_preview_curve_speed = RoutePreviewCurveSpeed()
  man.route_preview_display_target = 0.0
  man.route_preview_logged = 0.0
  man.autoCurveSpeedFactor = 1.0
  events = []
  monkeypatch.setattr(carrot_man.cloudlog, 'event', lambda name, **kw: events.append((name, kw)))
  from openpilot.selfdrive.navdy.navigation import RoutePreview
  preview = RoutePreview(route(straight_m=0), 0.2, ('s', 1))
  reader = NS(route_preview=lambda now: preview)
  sm = {'carState': NS(vEgo=20.0, aEgo=0.0, vCluRatio=1.0)}
  sm = type('SM', (dict,), {'alive': {'carState': True}, 'valid': {'carState': True}})(sm)
  for navdy, mode, limited in ((True, 2, True), (True, 3, True), (True, 1, False), (False, 2, False)):
    man.route_preview_curve_speed = RoutePreviewCurveSpeed()
    man.carrot_serv = NS(accepts_navigation=lambda source, n=navdy: n, turnSpeedControlMode=mode, navdy_navigation=reader,
                         autoCurveSpeedLowerLimit=30)
    speed = man.carrot_route_preview_speed(sm)
    if limited:
      assert abs(speed) == pytest.approx(kph(100), rel=0.02) and man.route_preview_display_target > 0
    else:
      assert abs(speed) == NO_LIMIT_KPH and man.route_preview_display_target == 0.0
  assert events and events[0][0] == 'navdy_route_preview' and events[0][1]['points'] == 81
