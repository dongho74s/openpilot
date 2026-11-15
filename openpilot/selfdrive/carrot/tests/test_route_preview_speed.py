"""Curve speed from the Navdy route preview (TMAP / Naver, 2026-11-15, review fixes 2026-11-16)."""
import math
from types import SimpleNamespace as NS

import pytest

from openpilot.selfdrive.carrot.curve_speed import APPROACH_DECEL, NO_LIMIT_KPH, RELEASE_HOLD, RELEASE_RATE_KPH
from openpilot.selfdrive.carrot.route_preview_speed import (
  LAG_MARGIN_S, FrozenPreviewGuard, RoutePreviewCurveSpeed, corners, preview_curve_speed,
)
from openpilot.selfdrive.navdy.navigation import FeedbackTracker, RoutePreview, preview_points


def split(vertices, gap=39.0):
  """Map vertices no more than `gap` m apart, as the producers require (straights get intermediate vertices)."""
  out = [vertices[0]]
  for a, b in zip(vertices, vertices[1:]):
    n = max(1, math.ceil(math.dist(a, b) / gap))
    out += [(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n) for k in range(1, n + 1)]
  return out

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


def test_curves_beyond_a_straight_are_separate():
  """Review 2 N1: a gentle bend, 200 m of straight, then a junction turn the same way. The turn's limit starts at
  the turn, not at the bend."""
  h = math.radians(20)
  p1, p2 = (0.0, 50.0), (200 * math.sin(h), 50 + 200 * math.cos(h))
  p3 = (p2[0] + 300 * math.sin(h + math.pi / 2), p2[1] + 300 * math.cos(h + math.pi / 2))
  points = resample(split([(0, -50), (0, 0), p1, p2, p3]))
  turn = [c for c in corners(points) if abs(c.turn) > math.radians(80)][0]
  assert turn.start_m > 280 and turn.curve_start_m > 280
  assert preview_curve_speed(points, 0.0, 20.0, 0.8, 30).approach_kph > 72.0


@pytest.mark.parametrize('radius,spacing,turn', [(86, 5, 33.32), (40, 30, 44.05), (86, 20, 40), (60, 15, 50)])
def test_short_arcs_between_straights_keep_their_radius(radius, spacing, turn):
  """Review 2 N7: a short arc drawn with a few vertices between straights."""
  points = resample(polygon_curve(radius, spacing, turn_deg=turn))
  assert preview_curve_speed(points, 0.0, 20.0, 0.8, 5).curve_kph == pytest.approx(kph(radius, BUDGET), rel=0.03)


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
  # Review 2 N5: passing is judged by the clock-free lower bound (time since arrival), never by a wall-clock age that
  # a fast comma clock inflates.
  assert preview_curve_speed(short, (end + 25) / v, v, min_age=0.1).approach_kph == pytest.approx(30.0)


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
  """Review R4 / review 2 N2: new observations with the same cornered window although the car drove on are a
  stalled phone position; the same observation re-read every cycle (normal between 1 Hz updates) is not."""
  guard = FrozenPreviewGuard()
  t = 0.0

  def run(preview, seconds, v=20.0):
    nonlocal t
    out = []
    for _ in range(int(round(seconds / 0.05))):
      t += 0.05
      out.append(guard.update(preview, v, t))
    return out

  # Normal 1 Hz at 72 km/h: every observation is used for its whole second.
  for k in range(4):
    preview = RoutePreview(route(straight_m=150 - 20 * k), 0.1, ('s', k))
    assert all(p is preview for p in run(preview, 1.0))
  # The phone repeats the last window as new observations while the car drives on: rejected.
  for k in range(4, 6):
    stale = RoutePreview(route(straight_m=150 - 20 * 3), 0.1, ('s', k))
    assert all(p is None for p in run(stale, 1.0))
  # ... also with a few centimetres of noise.
  noisy = RoutePreview(tuple((x + 0.06, y) if i == 30 else (x, y) for i, (x, y) in
                             enumerate(route(straight_m=150 - 20 * 3))), 0.1, ('s', 6))
  assert all(p is None for p in run(noisy, 1.0))
  moved = RoutePreview(route(straight_m=60), 0.1, ('s', 7))
  assert all(p is moved for p in run(moved, 1.0))
  # A straight window, or a stopped car, repeats legitimately.
  for k in range(8, 11):
    straight = RoutePreview(route(radius=0), 0.1, ('s', k))
    assert all(p is straight for p in run(straight, 1.0))
  stopped = FrozenPreviewGuard()
  for k in range(3):
    preview = RoutePreview(route(straight_m=100), 0.1, ('t', k))
    assert stopped.update(preview, 0.0, float(k)) is preview


def test_one_continuous_curve_keeps_its_identity_while_the_binding_corner_moves():
  """Review 2 N3: the stretch of road of the curve a corner belongs to identifies it across previews; successive
  previews of one curve overlap (carrot_man counts a new curve only when they do not)."""
  previous, changes = None, 0
  for travelled in range(0, 240, 20):
    points = resample(polygon_curve(86, 20, straight=50, turn_deg=180), start=travelled)
    result = preview_curve_speed(points, 0.0, 20.0, 0.8, 30)
    assert result.approach_kph < NO_LIMIT_KPH
    span = (travelled + result.curve_start, travelled + result.curve_end)
    if previous is not None and (span[0] > previous[1] + 10.0 or span[1] < previous[0] - 10.0):
      changes += 1
    previous = span
  assert changes == 0


# --- tracker ---------------------------------------------------------------------------------------------------------

WALL = 1_800_000_000.0


def tracker():
  out = FeedbackTracker()
  out.clock_synced = lambda: True
  return out


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
  t = tracker()
  assert t.accept(packet(), 100.0, 100.0, WALL)
  preview = t.route_preview(100.5)
  assert preview is not None and len(preview.points) == 81
  assert preview.age == pytest.approx(0.8) and preview.min_age == pytest.approx(0.5) and preview.key[1] == 7
  assert t.route_preview(102.3) is None   # observed 2.6 s ago


def test_tracker_age_is_fixed_on_arrival_by_the_comma_clock():
  """Review R2/R3, review 2 N5/N6: the HUD clock was 1.3 s behind the phone (2026-11-16), clamps the age at 0, and
  its age left out the USB leg; the comma's clock against observedAtMs covers the whole way, once, on arrival."""
  t = tracker()
  assert t.accept(packet(observed=WALL - 1.2, ageMs=0), 100.0, 100.0, WALL)   # the HUD says 0 ms
  assert t.route_preview(100.0).age == pytest.approx(1.2)
  assert t.route_preview(100.5).age == pytest.approx(1.7)   # monotonic from there: a later wall-clock step is moot
  t = tracker()
  assert t.accept(packet(observed=WALL - 0.1, ageMs=3900), 100.0, 100.0, WALL)   # a HUD clock far ahead
  assert t.route_preview(101.9).age == pytest.approx(2.0)
  t = tracker()
  assert t.accept(packet(observed=WALL + 0.4, ageMs=0), 100.0, 100.0, WALL)   # comma a little behind: age 0
  assert t.route_preview(100.2).age == pytest.approx(0.2)
  t = tracker()
  assert t.accept(packet(observed=WALL + 0.8, ageMs=0), 100.0, 100.0, WALL)   # too far behind
  assert t.route_preview(100.1) is None


def test_tracker_needs_an_ntp_synced_comma_clock():
  t = tracker()
  t.clock_synced = lambda: False
  assert t.accept(packet(), 100.0, 100.0, WALL)
  assert t.route_preview(100.1) is None


@pytest.mark.parametrize('kwargs', [{'guidance_active': False}, {'with_guidance': False}])
def test_tracker_needs_active_guidance(kwargs):
  """Review R5: a preview left over after guidance stopped does not limit."""
  t = tracker()
  assert t.accept(packet(**kwargs), 100.0, 100.0, WALL)
  assert t.route_preview(100.1) is None


def test_late_packets_never_bring_back_an_old_route_but_a_returning_app_does():
  """Review R5 / review 2 N4: A10, B20, late A9, C30, late B19: C stays. A coming back with A11 is accepted."""
  a, b, c = ('12345678-1234-1234-1234-12345678900' + x for x in 'abc')
  t = tracker()
  received = 100.0
  for session, sequence, expected in ((a, 10, (a, 10)), (b, 20, (b, 20)), (a, 9, (b, 20)), (c, 30, (c, 30)),
                                      (b, 19, (c, 30)), (a, 11, (a, 11))):
    received += 0.1
    assert t.accept(packet(session=session, sequence=sequence), received, received, WALL)
    assert t.route_preview(received).key == expected


def test_the_newest_observation_without_a_window_clears_it():
  t = tracker()
  assert t.accept(packet(sequence=7), 100.0, 100.0, WALL)
  assert t.accept(packet(sequence=8, reason='off_route_or_heading', points=[]), 100.1, 100.1, WALL)
  assert t.route_preview(100.1) is None


@pytest.mark.parametrize('overrides', [
  {'guidance': False}, {'reason': 'off_route_or_heading'}, {'points': []}, {'points': [[0, 0]] * 81},
  {'points': [[0, 0], [5, 0], [10, 0], [15, 0], [20, 0], [25, 0]]}, {'points': [[2, 0]] + [[2 + 5 * i, 0] for i in range(1, 20)]},
  {'points': [[0, 0]] + [[5 * i, 0] for i in range(1, 10)] + [[60, 0]]}, {'points': [[0, 0], [5, 0], ['x', 0]] + [[5 * i, 0] for i in range(3, 10)]},
  {'points': [[0, 0]] + [[5 * i, 0] for i in range(1, 90)]}, {'points': [[0, 0]] + [[0, 600]] * 9},
  {'observedAtMs': None}, {'observedAtMs': 'x'},
])
def test_tracker_rejects_unusable_previews(overrides):
  t = tracker()
  assert t.accept(packet(**overrides), 100.0, 100.0, WALL)
  assert t.route_preview(100.1) is None


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
  man.route_preview_curve_at = None
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
