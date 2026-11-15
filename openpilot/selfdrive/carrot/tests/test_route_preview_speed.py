"""Curve speed from the Navdy route preview (TMAP / Naver, 2026-11-15, review fixes 2026-11-16)."""
import math
from types import SimpleNamespace as NS

import pytest

from openpilot.selfdrive.carrot.curve_speed import APPROACH_DECEL, NO_LIMIT_KPH, RELEASE_HOLD, RELEASE_RATE_KPH
from openpilot.selfdrive.carrot.route_preview_speed import (
  LAG_MARGIN_S, FrozenPreviewGuard, RoutePreviewCurveSpeed, corners, preview_curve_speed,
)
from openpilot.selfdrive.navdy.navigation import FeedbackTracker, RoutePreview, preview_points

BUDGET = 1.9 / 0.8   # AutoCurveSpeedFactor 80 %, the user's setting


def kph(radius, budget=1.9):
  return math.sqrt(budget * radius) * 3.6


def route(straight_m=200.0, radius=100.0, left=True, total=400.0, step=5.0, heading=0.0):
  """Points every `step` m exactly on the road: `straight_m` along `heading` (rad from north), then a constant-radius
  arc (a densely drawn map)."""
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


def resample(vertices, start=0.0, step=5.0, window=400.0):
  """What the phone sends (TMAP RouteWindow / Naver): the map polyline linearly interpolated every 5 m from the
  matched point `start` m along it, rounded to 1 cm."""
  segments, along = [], 0.0
  for a, b in zip(vertices, vertices[1:]):
    length = math.dist(a, b)
    if length > 0:
      segments.append((a, b, along, length))
      along += length

  def at(s):
    for a, b, s0, length in segments:
      if s <= s0 + length + 1e-9:
        t = (s - s0) / length
        return a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t
    return None

  origin, out, s = at(start), [(0.0, 0.0)], step
  while s <= window and (p := at(start + s)) is not None:
    out.append((round(p[0] - origin[0], 2), round(p[1] - origin[1], 2)))
    s += step
  return tuple(out)


def polygon_curve(radius, spacing, straight=150.0, turn_deg=90.0, phase=0.0, left=False):
  """A map drawing of a curve: straight north, vertices on the circle `spacing` m of chord apart, then straight."""
  sign = -1 if left else 1
  vertices = [(0.0, -straight - phase), (0.0, 0.0)]
  step, angle = 2 * math.asin(spacing / (2 * radius)), 0.0
  while angle + step <= math.radians(turn_deg) + 1e-9:
    angle += step
    vertices.append((sign * radius * (1 - math.cos(angle)), radius * math.sin(angle)))
  last = vertices[-1]
  vertices.append((last[0] + sign * 400 * math.sin(angle), last[1] + 400 * math.cos(angle)))
  return vertices


def test_straight_route_has_no_limit():
  result = preview_curve_speed(route(radius=0), 0.0, 20.0)
  assert result.approach_kph == NO_LIMIT_KPH and result.curve_kph == NO_LIMIT_KPH


@pytest.mark.parametrize('left', [True, False])
@pytest.mark.parametrize('heading', [0.0, 1.0, -2.5])
def test_curve_target_is_the_vision_physics_whatever_the_heading(left, heading):
  result = preview_curve_speed(route(straight_m=0, left=left, heading=heading), 0.0, 20.0)
  assert result.curve_kph == pytest.approx(kph(100), rel=0.02)
  assert result.direction == (1.0 if left else -1.0)
  # in the curve already: the binding corner is within the response distance (about 32 m at 20 m/s)
  assert result.distance <= 32.5 and result.approach_kph == pytest.approx(result.curve_kph)


@pytest.mark.parametrize('radius', [30, 86, 150, 300, 600])
def test_densely_drawn_arcs(radius):
  points = resample(polygon_curve(radius, 5, straight=100, turn_deg=60))
  assert preview_curve_speed(points, 0.0, 20.0, 0.8, 30).curve_kph == pytest.approx(max(30.0, kph(radius, BUDGET)), rel=0.02)


@pytest.mark.parametrize('spacing', [5, 10, 20, 25, 30, 35, 39])
@pytest.mark.parametrize('phase', [0.0, 1.7, 3.3])
def test_map_vertex_spacing_does_not_change_the_curve(spacing, phase):
  """Review R1: an 86 m radius drawn with vertices up to 40 m apart (the producer's limit) and resampled every 5 m is
  still an 86 m radius (51 km/h at the user's 80 %), not a sharp kink at each vertex."""
  points = resample(polygon_curve(86, spacing, phase=phase))
  result = preview_curve_speed(points, 0.0, 20.0, 0.8, 30)
  assert 0.92 * kph(86, BUDGET) <= result.curve_kph <= 1.05 * kph(86, BUDGET)


@pytest.mark.parametrize('start', [0, 2, 4, 8, 12])
def test_starting_inside_a_coarsely_drawn_curve(start):
  points = resample(polygon_curve(86, 39, straight=0), start=start)
  assert preview_curve_speed(points, 0.0, 20.0, 0.8, 30).curve_kph == pytest.approx(kph(86, BUDGET), rel=0.05)


def test_sharp_corners_and_reversals_slow_to_the_floor():
  def speed(vertices):
    return round(preview_curve_speed(resample(vertices), 0.0, 20.0, 0.8, 30).curve_kph, 6)
  assert speed([(0, -100), (0, 100), (300, 100)]) == 30.0                    # a 90 degree junction
  assert speed([(0, -100), (0, 100), (3.54, 103.54), (300, 103.54)]) == 30.0  # two 45 degree vertices 5 m apart
  a, p, split = 0.0, (0.0, 100.0), [(0.0, -100.0), (0.0, 100.0)]
  for _ in range(2):
    a += math.radians(30)
    p = (p[0] + 5 * math.sin(a), p[1] + 5 * math.cos(a))
    split.append(p)
  a += math.radians(30)
  split.append((p[0] + 300 * math.sin(a), p[1] + 300 * math.cos(a)))
  assert speed(split) == 30.0                                                # three 30 degree vertices 5 m apart
  # Review R6: a U-turn drawn back along the same centreline is not a straight road.
  assert speed([(0, 0), (0, 100), (0, -300)]) == 30.0


def test_a_long_hairpin_keeps_its_radius():
  points = resample(polygon_curve(86, 20, straight=50, turn_deg=180))
  assert preview_curve_speed(points, 0.0, 20.0, 0.8, 30).curve_kph == pytest.approx(kph(86, BUDGET), rel=0.03)


def test_small_map_zigzags_and_kinks_are_not_curves():
  points = list(route(radius=0))
  points[40], points[41] = (points[40][0] + 1.0, points[40][1]), (points[41][0] + 1.0, points[41][1])
  assert preview_curve_speed(tuple(points), 0.0, 20.0).approach_kph == NO_LIMIT_KPH   # review R1 artefact
  kink = [(0, -100), (0, 150), (300 * math.sin(math.radians(5)), 150 + 300 * math.cos(math.radians(5)))]
  assert preview_curve_speed(resample(kink, start=100), 0.0, 20.0, 0.8, 30).approach_kph == NO_LIMIT_KPH


def test_distance_relaxes_approach_with_the_same_envelope_as_vision():
  v = 20.0
  result = preview_curve_speed(route(straight_m=200), 0.0, v, lag=0.0)
  assert result.curve_kph == pytest.approx(kph(100), rel=0.02)
  assert 175.0 <= result.distance <= 200.0
  response = v * (1.0 + APPROACH_DECEL / (2 * 0.8))
  expected = math.sqrt((result.curve_kph / 3.6) ** 2 + 2 * APPROACH_DECEL * (result.distance - response)) * 3.6
  assert result.approach_kph == pytest.approx(expected)
  assert result.approach_kph > v * 3.6   # 72 km/h, about 190 m before a 100 m radius curve: no slowing needed yet
  assert preview_curve_speed(route(straight_m=100), 0.0, v).approach_kph < v * 3.6   # 100 m before it: slowing


def test_age_and_lag_count_the_distance_already_driven():
  fresh = preview_curve_speed(route(straight_m=200), 0.0, 20.0, lag=0.0)
  aged = preview_curve_speed(route(straight_m=200), 1.0, 20.0, lag=0.0)
  assert aged.distance == pytest.approx(fresh.distance - 20.0)
  assert aged.approach_kph < fresh.approach_kph and aged.curve_kph == fresh.curve_kph
  lagged = preview_curve_speed(route(straight_m=200), 0.0, 20.0)
  assert lagged.distance == pytest.approx(fresh.distance - 20.0 * LAG_MARGIN_S)   # approaching: assume further along


def test_a_passed_curve_no_longer_limits_but_lag_keeps_it_a_little_longer():
  short = resample([(0, 0), (0, 50), (300, 50)])   # a 90 degree corner 50 m ahead
  end = max(c.end_m for c in corners(short))
  v = 20.0
  assert preview_curve_speed(short, (end + 5) / v, v, lag=0.0).approach_kph == NO_LIMIT_KPH
  assert preview_curve_speed(short, (end + 5) / v, v).approach_kph == pytest.approx(30.0)   # 20 m of lag: may not have passed
  assert preview_curve_speed(short, (end + 25) / v, v).approach_kph == NO_LIMIT_KPH


def test_sensitivity_and_floor_follow_the_curve_settings():
  tight = route(straight_m=0, radius=20)
  assert preview_curve_speed(tight, 0.0, 10.0, lower_limit_kph=30).curve_kph == pytest.approx(30.0)
  wide = route(straight_m=0, radius=150)
  base = preview_curve_speed(wide, 0.0, 20.0, sensitivity=1.0).curve_kph
  assert preview_curve_speed(wide, 0.0, 20.0, sensitivity=0.8).curve_kph == pytest.approx(base * math.sqrt(1 / 0.8))
  assert preview_curve_speed(wide, 0.0, 20.0, sensitivity=1.2).curve_kph == pytest.approx(base * math.sqrt(1 / 1.2))


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


def test_a_frozen_window_with_a_corner_is_rejected_until_it_changes():
  """Review R4: a phone whose matched position stopped keeps sending the same window as new observations."""
  guard = FrozenPreviewGuard()
  curve = RoutePreview(route(straight_m=100), 0.2, ('s', 1))
  t = 0.0
  assert guard.update(curve, 20.0, t) is curve
  for _ in range(3):   # 0.3 s at 20 m/s: 6 m, not yet suspicious
    t += 0.1
    assert guard.update(curve, 20.0, t) is curve
  for _ in range(3):   # 12 m driven with the corner no nearer
    t += 0.1
    guard.update(curve, 20.0, t)
  assert guard.update(curve, 20.0, t + 0.1) is None
  moved = RoutePreview(route(straight_m=80), 0.2, ('s', 2))
  assert guard.update(moved, 20.0, t + 0.2) is moved
  straight = RoutePreview(route(radius=0), 0.2, ('s', 3))   # a straight window is the same however far the car moves
  for i in range(30):
    assert guard.update(straight, 20.0, t + 0.3 + 0.1 * i) is straight
  stopped = FrozenPreviewGuard()
  for i in range(30):
    assert stopped.update(curve, 0.0, 0.1 * i) is curve


# --- tracker ---------------------------------------------------------------------------------------------------------

WALL = 1_800_000_000.0


def packet(observed=WALL - 0.3, guidance_active=True, session='12345678-1234-1234-1234-123456789012', sequence=7,
           with_guidance=True, **overrides):
  value = {'version': 1, 'session': session, 'sequence': sequence, 'ageMs': 300,
           'observedAtMs': int(observed * 1000), 'routeId': 3, 'guidance': True, 'reason': 'ok', 'roadLimitKph': 60,
           'bumpDistanceM': 0, 'points': [list(p) for p in route()][:81]}
  value.update(overrides)
  out = {'roadPreview': value}
  if with_guidance:
    out['navigationFeedback'] = {'version': 1, 'session': 'hud', 'sequence': sequence, 'ageMs': 0, 'validForMs': 2500,
                                 'guidance': {'active': guidance_active}}
  return out


def test_tracker_returns_a_fresh_guided_preview():
  tracker = FeedbackTracker()
  assert tracker.accept(packet(), 100.0, 100.0)
  preview = tracker.route_preview(100.5, WALL + 0.5)
  assert preview is not None and len(preview.points) == 81
  assert preview.age == pytest.approx(0.8) and preview.key[1] == 7
  assert tracker.route_preview(102.5, WALL + 2.5) is None   # observed 2.8 s ago


def test_tracker_age_is_the_comma_clock_not_the_hud_clock():
  """Review R2/R3: the HUD clock was 1.3 s behind the phone (2026-11-16) and clamps the age at 0, and its age leaves
  out the HUD-to-comma leg. The comma's own clock against observedAtMs covers the whole way."""
  tracker = FeedbackTracker()
  assert tracker.accept(packet(observed=WALL - 1.2, ageMs=0), 100.0, 100.0)   # the HUD says 0 ms
  assert tracker.route_preview(100.0, WALL).age == pytest.approx(1.2)
  # The comma clock a little behind the phone: never younger than its time since arrival.
  tracker = FeedbackTracker()
  assert tracker.accept(packet(observed=WALL + 0.5, ageMs=0), 100.0, 100.0)
  assert tracker.route_preview(100.4, WALL + 0.4).age == pytest.approx(0.4)
  # Far behind: no age can be trusted.
  tracker = FeedbackTracker()
  assert tracker.accept(packet(observed=WALL + 1.5, ageMs=0), 100.0, 100.0)
  assert tracker.route_preview(100.1, WALL + 0.1) is None


@pytest.mark.parametrize('kwargs', [{'guidance_active': False}, {'with_guidance': False}])
def test_tracker_needs_active_guidance(kwargs):
  """Review R5: a preview left over after guidance stopped does not limit."""
  tracker = FeedbackTracker()
  assert tracker.accept(packet(**kwargs), 100.0, 100.0)
  assert tracker.route_preview(100.1, WALL) is None


def test_a_replaced_producer_never_comes_back():
  """Review R5: A seq 10, then B seq 2, then a late A seq 9 must not revive A's route."""
  a, b = '12345678-1234-1234-1234-12345678900a', '12345678-1234-1234-1234-12345678900b'
  tracker = FeedbackTracker()
  assert tracker.accept(packet(session=a, sequence=10), 100.0, 100.0)
  assert tracker.route_preview(100.0, WALL).key == (a, 10)
  assert tracker.accept(packet(session=b, sequence=2), 100.1, 100.1)
  assert tracker.route_preview(100.1, WALL).key == (b, 2)
  assert tracker.accept(packet(session=a, sequence=9), 100.2, 100.2)
  assert tracker.route_preview(100.2, WALL) is None
  assert tracker.accept(packet(session=b, sequence=3), 100.3, 100.3)
  assert tracker.route_preview(100.3, WALL).key == (b, 3)


@pytest.mark.parametrize('overrides', [
  {'guidance': False}, {'reason': 'off_route_or_heading'}, {'points': []}, {'points': [[0, 0]] * 81},
  {'points': [[0, 0], [5, 0], [10, 0], [15, 0], [20, 0], [25, 0]]}, {'points': [[2, 0]] + [[2 + 5 * i, 0] for i in range(1, 20)]},
  {'points': [[0, 0]] + [[5 * i, 0] for i in range(1, 10)] + [[60, 0]]}, {'points': [[0, 0], [5, 0], ['x', 0]] + [[5 * i, 0] for i in range(3, 10)]},
  {'points': [[0, 0]] + [[5 * i, 0] for i in range(1, 90)]}, {'points': [[0, 0]] + [[0, 600]] * 9},
  {'observedAtMs': None}, {'observedAtMs': 'x'},
])
def test_tracker_rejects_unusable_previews(overrides):
  tracker = FeedbackTracker()
  assert tracker.accept(packet(**overrides), 100.0, 100.0)
  assert tracker.route_preview(100.1, WALL) is None


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


def test_an_accelerator_floor_does_not_carry_into_the_next_corner(monkeypatch):
  """Review: pressing the accelerator through one previewed corner keeps that speed for that corner only."""
  from openpilot.cereal import car
  from openpilot.selfdrive.carrot.tests.test_deceleration_candidates import make_serv, publish
  serv, sm, pm, sent = make_serv(monkeypatch, TurnSpeedControlMode=2, AutoCurveSpeedLowerLimit=30)
  sm['carState'] = car.CarState.new_message(vEgo=20., vCruise=120., gasPressed=False)
  assert publish(serv, sm, pm, sent, route_preview_speed=50.0, route_preview_curve=1).desiredSpeed == 50
  sm['carState'] = car.CarState.new_message(vEgo=20., vCruise=120., gasPressed=True)
  publish(serv, sm, pm, sent, route_preview_speed=50.0, route_preview_curve=1)
  sm['carState'] = car.CarState.new_message(vEgo=20., vCruise=120., gasPressed=False)
  msg = publish(serv, sm, pm, sent, route_preview_speed=40.0, route_preview_curve=1)
  assert msg.desiredSource == 'gas' and msg.desiredSpeed == 72   # same corner: the driver's floor holds
  msg = publish(serv, sm, pm, sent, route_preview_speed=30.0, route_preview_curve=2)
  assert msg.desiredSource == 'route' and msg.desiredSpeed == 30  # next corner: it does not


def test_carrot_man_runs_only_in_navdy_route_modes_and_forgets_when_off(monkeypatch):
  from openpilot.selfdrive.carrot import carrot_man
  man = object.__new__(carrot_man.CarrotMan)
  man.route_preview_curve_speed = RoutePreviewCurveSpeed()
  man.route_preview_guard = FrozenPreviewGuard()
  man.route_preview_display_target = 0.0
  man.route_preview_logged = 0.0
  man.route_preview_curve = 0
  man.route_preview_corner_at = None
  man.autoCurveSpeedFactor = 1.0
  events = []
  monkeypatch.setattr(carrot_man.cloudlog, 'event', lambda name, **kw: events.append((name, kw)))
  preview = RoutePreview(route(straight_m=0), 0.2, ('s', 1))
  reader = NS(route_preview=lambda now: preview)
  sm = {'carState': NS(vEgo=20.0, aEgo=0.0, vCluRatio=1.0)}
  sm = type('SM', (dict,), {'alive': {'carState': True}, 'valid': {'carState': True}})(sm)

  def run(navdy, mode):
    man.carrot_serv = NS(accepts_navigation=lambda source: navdy, turnSpeedControlMode=mode, navdy_navigation=reader,
                         autoCurveSpeedLowerLimit=30)
    return man.carrot_route_preview_speed(sm)

  assert abs(run(True, 2)) == pytest.approx(kph(100), rel=0.02) and man.route_preview_display_target > 0
  assert man.route_preview_curve == 1
  assert abs(run(True, 3)) == pytest.approx(kph(100), rel=0.02) and man.route_preview_curve == 1   # same corner
  for navdy, mode in ((True, 1), (False, 2), (True, 0)):
    assert abs(run(navdy, mode)) == NO_LIMIT_KPH and man.route_preview_display_target == 0.0
  # Switched back on with no preview: the old cap does not come back (review: mode 2 -> 0 -> 2).
  reader.route_preview = lambda now: None
  assert abs(run(True, 2)) == NO_LIMIT_KPH
  assert events and events[0][0] == 'navdy_route_preview' and events[0][1]['points'] == 81
