"""Curve speed from the Navdy route preview (TMAP / Naver, 2026-11-15, review fixes 2026-11-16)."""
import math
from types import SimpleNamespace as NS

import pytest

from openpilot.selfdrive.carrot.curve_speed import APPROACH_DECEL, NO_LIMIT_KPH, RELEASE_HOLD, RELEASE_RATE_KPH
from openpilot.selfdrive.carrot.route_preview_speed import (
  LAG_MARGIN_S, FrozenPreviewGuard, RoutePreviewCurveSpeed, RouteSpeedMap, corners, preview_curve_speed,
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


def test_the_straight_between_two_curves_changes_the_limit_smoothly():
  """Review 3 T5: a 45-80 m straight drawn with map segments under 40 m keeps the two curves apart; the turn's
  approach limit grows with the straight, without a jump to the floor."""
  limits = []
  for straight in (45, 50, 55, 60, 65, 80):
    h = math.radians(20)
    p1, p2 = (0.0, 50.0), (straight * math.sin(h), 50 + straight * math.cos(h))
    p3 = (p2[0] + 300 * math.sin(h + math.pi / 2), p2[1] + 300 * math.cos(h + math.pi / 2))
    limits.append(preview_curve_speed(resample(split([(0, -50), (0, 0), p1, p2, p3])), 0.0, 20.0, 0.8, 30).approach_kph)
  assert all(b >= a for a, b in zip(limits, limits[1:])) and limits[0] > 50.0


def test_a_metre_more_of_straight_does_not_jump_the_limit():
  """Review 4 U6: 39 m against 40 m of straight between a bend and a junction turn the same way."""
  limits = []
  for straight in (39, 40):
    h = math.radians(20)
    p1, p2 = (0.0, 100.0), (straight * math.sin(h), 100 + straight * math.cos(h))
    p3 = (p2[0] + 300 * math.sin(h + math.pi / 2), p2[1] + 300 * math.cos(h + math.pi / 2))
    points = resample(split([(0, -50), (0, 0), p1, p2, p3]), start=65)
    limits.append(preview_curve_speed(points, 0.0, 14.3, 0.8, 30).approach_kph)
  assert abs(limits[0] - limits[1]) < 3.0


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
  # Review 3 T3: at 30 km/h (8.3 m per 1 Hz observation) the repeats add up across observations.
  slow, results = FrozenPreviewGuard(), []
  for k in range(5):
    preview = RoutePreview(route(straight_m=20), 0.1, ('u', k))
    for i in range(20):
      results.append((k, slow.update(preview, 30 / 3.6, k + 0.05 * (i + 1))))
  assert all(p is not None for k, p in results if k <= 1) and all(p is None for k, p in results if k >= 2)


def drive(vertices, until, v=14.3, start=0.0, switch=None, delay=0.3):
  """Drive along the route at v: a new observation each second (observed 0.3 s before), the map and the limiter
  every 50 ms, as carrot_man runs them. switch=(odometer, vertices): a new route from there.
  Returns [(odometer, applied km/h, map result, curve number)]."""
  speed_map, limiter = RouteSpeedMap(), RoutePreviewCurveSpeed()
  odometer, t, out, k, observation = start, 0.0, [], 0, None
  while odometer < until:
    if abs(t - round(t)) < 1e-9:
      current, route_id = (switch[1], ('r', 2)) if switch and odometer >= switch[0] else (vertices, ('r', 1))
      points = resample(current, start=odometer - v * delay)
      k += 1
      observation = (points, route_id, t, k) if len(points) >= 7 else None
    preview = None
    if observation is not None:
      since = t - observation[2]
      if delay + since <= 2.5:
        preview = RoutePreview(observation[0], delay + since, ('s', observation[3]), since, observation[1])
    result = speed_map.update(preview, t, odometer, v, 0.0, 0.8, 5, 1.0)
    out.append((odometer, abs(limiter.update(result, t)), result, speed_map.number))
    odometer += v * 0.05
    t = round(t + 0.05, 2)
  return out


@pytest.mark.parametrize('cold', [0.0, 120.0])
@pytest.mark.parametrize('radius,spacing,turn,phase', [(86, 5, 33.32, 3.3), (86, 20, 60, 0), (86, 39, 90, 1.7),
                                                       (50, 10, 90, 0), (40, 30, 44.05, 0)])
def test_driving_up_to_and_through_a_curve_keeps_its_speed(radius, spacing, turn, phase, cold):
  """Review 3 T1 / review 4 U3, U7: one curve seen from moving windows, from far away or first seen 30 m before it.
  In it the applied speed stays near the physics, it is not released before the end, and soon after."""
  start = 150 + phase
  end = start + radius * math.radians(turn)
  runs = drive(polygon_curve(radius, spacing, turn_deg=turn, phase=phase), end + 120, start=cold)
  physics = kph(radius, BUDGET)
  inside = [applied for odometer, applied, _, _ in runs if start - 5 <= odometer <= end]
  assert inside and all(0.8 * physics <= applied <= 1.15 * physics for applied in inside)
  free = [odometer for odometer, _, result, _ in runs if result is not None and result.approach_kph >= NO_LIMIT_KPH
          and odometer > start]
  assert free and end < free[0] <= end + 60
  assert {number for *_, number in runs} == {1}


def test_a_u_turn_is_kept_on_the_road_map():
  """Review 5 V1: a U-turn's zero-length corner between 5 m samples, on a diagonal line."""
  h = math.radians(45)
  vertices = split([(0.0, 0.0), (200 * math.sin(h), 200 * math.cos(h)), (-200 * math.sin(h), -200 * math.cos(h))])
  runs = drive(vertices, 190, v=20.0)
  near = [applied for odometer, applied, _, _ in runs if 140 <= odometer <= 185]
  assert near and max(near) <= 31.0


def test_a_nearer_tighter_estimate_is_taken_and_never_loosened():
  """Review 5 V2 (shape per review 6 W3): R30 drawn with four 5 m chords (38.24 degrees, 20 m) between straights.
  Far windows estimate it gently; the map keeps the tight speed through the curve (it had kept 47.5)."""
  vertices = polygon_curve(30, 5, turn_deg=40)
  assert len(vertices) == 2 + 4 + 1   # straight, four chords on the circle, straight
  runs = drive(vertices, 175)
  inside = [applied for odometer, applied, _, _ in runs if 150 <= odometer <= 170]
  assert inside and max(inside) <= 1.1 * kph(30, BUDGET)


@pytest.mark.parametrize('radius,spacing,turn,phase,v', [(150, 34, 135, 3.5, 20.0), (150, 39, 60, 3.3, 14.3)])
def test_one_odd_tight_estimate_does_not_hold_a_gentle_curve_down(radius, spacing, turn, phase, v):
  """Review 6 W1: a constant-radius arc; one window's estimate came out too tight and the map kept it."""
  start = 150 + phase
  end = start + radius * math.radians(turn)
  runs = drive(polygon_curve(radius, spacing, turn_deg=turn, phase=phase), end, v=v)
  inside = [applied for odometer, applied, _, _ in runs if start + 20 <= odometer <= end - 20]
  assert inside and min(inside) >= 0.88 * kph(radius, BUDGET)


def test_one_long_gentle_curve_keeps_one_number():
  """Review 6 W2: a 180 degree R250 arc drawn every 5 m; the binding moves between parts the simplification draws as
  separate curves, but it is one curve for the accelerator floor."""
  runs = drive(polygon_curve(250, 5, turn_deg=180), 150 + 250 * math.pi, v=28.0)
  numbers = {number for odometer, applied, result, number in runs if number}
  assert numbers == {1}


@pytest.mark.parametrize('delay', [0.3, 1.0, 2.4])
def test_a_passed_curve_is_released_soon_even_with_delayed_previews(delay):
  """Review 5 V5: previews arriving up to 2.4 s after their observation (the age is still under 2.5 s)."""
  start, end = 153.3, 153.3 + 86 * math.radians(33.32)
  runs = drive(polygon_curve(86, 5, turn_deg=33.32, phase=3.3), end + 120, delay=delay)
  free = [odometer for odometer, _, result, _ in runs if result is not None and result.approach_kph >= NO_LIMIT_KPH
          and odometer > start]
  assert free and free[0] <= end + 60


def test_a_tighter_part_of_one_curve_gets_its_own_speed():
  """Review 4 U1: R86 for 60 degrees then R40 for 90 degrees, the same way."""
  def arc(start, heading, radius, turn, left):
    points, (x, y), h = [], start, heading
    for _ in range(int(radius * math.radians(turn) / 5)):
      h += (-1 if left else 1) * 5 / radius
      x, y = x + 5 * math.sin(h), y + 5 * math.cos(h)
      points.append((x, y))
    return points, h
  vertices = [(0.0, -150.0), (0.0, 0.0)]
  first, h = arc((0.0, 0.0), 0.0, 86, 60, False)
  second, h = arc(first[-1], h, 40, 90, False)
  vertices += first + second + [(second[-1][0] + 400 * math.sin(h), second[-1][1] + 400 * math.cos(h))]
  split_at = 150 + 86 * math.radians(60)
  end = split_at + 40 * math.radians(90)
  runs = drive(vertices, end)
  tight = [applied for odometer, applied, _, _ in runs if split_at + 5 <= odometer <= end - 5]
  assert tight and max(tight) <= 1.05 * kph(40, BUDGET)


def test_a_curve_the_other_way_is_another_curve():
  """Review 3 T2: right then left with no straight between: two curves (an accelerator floor does not carry)."""
  def arc(start, heading, radius, turn, left):
    points, (x, y), h = [], start, heading
    for _ in range(int(radius * math.radians(turn) / 5)):
      h += (-1 if left else 1) * 5 / radius
      x, y = x + 5 * math.sin(h), y + 5 * math.cos(h)
      points.append((x, y))
    return points, h
  vertices = [(0.0, -150.0), (0.0, 0.0)]
  right, h = arc((0.0, 0.0), 0.0, 86, 60, False)
  left, h = arc(right[-1], h, 50, 90, True)
  vertices += right + left + [(left[-1][0] + 400 * math.sin(h), left[-1][1] + 400 * math.cos(h))]
  directions = {number: result.direction for _, _, result, number in drive(vertices, 300)
                if result is not None and result.approach_kph < NO_LIMIT_KPH}
  assert directions == {1: -1.0, 2: 1.0}


def test_a_new_straight_route_does_not_keep_the_old_turn():
  """Review 4 U2: a turn 150 m ahead, then after 40 m the phone reroutes onto a straight road."""
  runs = drive([(0, -100), (0, 150), (300, 150)], 120, v=20.0, switch=(40.0, [(0, -100), (0, 600)]))
  before = [applied for odometer, applied, _, _ in runs if odometer < 40][-1]
  assert all(applied >= before - 1e-6 for odometer, applied, _, _ in runs if odometer >= 40)


def test_a_missing_preview_gives_no_result_at_once_and_is_dropped_after_a_second():
  """Review 5 V3: the limiter must see a missing preview at once (hold, then release); the cells wait a second."""
  speed_map = RouteSpeedMap()
  preview = RoutePreview(route(straight_m=100), 0.3, ('s', 1), 0.0, ('r', 1))
  t = 0.0
  assert speed_map.update(preview, t, 0.0, 20.0).approach_kph < NO_LIMIT_KPH
  for _ in range(9):
    t += 0.1
    assert speed_map.update(None, t, 2.0 * t * 10, 20.0) is None and speed_map.cells   # kept for a return
  for _ in range(3):
    t += 0.1
    speed_map.update(None, t, 2.0 * t * 10, 20.0)
  assert not speed_map.cells


def test_a_gap_in_updates_starts_the_map_again():
  """Review 5 V4: without updates (car state lost) the odometer cannot follow; old cells must not stay."""
  speed_map = RouteSpeedMap()
  preview = RoutePreview(route(straight_m=100), 0.3, ('s', 1), 0.0, ('r', 1))
  assert speed_map.update(preview, 0.0, 0.0, 20.0).approach_kph < NO_LIMIT_KPH
  later = RoutePreview(route(radius=0), 0.3, ('s', 2), 0.0, ('r', 1))
  assert speed_map.update(later, 4.0, 5.0, 20.0).approach_kph == NO_LIMIT_KPH   # the old turn is not kept


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


def test_an_older_observation_of_another_producer_does_not_replace_a_newer_one():
  """Review 3 T4: A10 observed at 0, B20 at 1.0, then a late A11 observed at 0.2: B stays."""
  a, b = ('12345678-1234-1234-1234-12345678900' + x for x in 'ab')
  t = tracker()
  assert t.accept(packet(session=a, sequence=10, observed=WALL - 1.0), 100.0, 100.0, WALL)
  assert t.accept(packet(session=b, sequence=20, observed=WALL), 100.1, 100.1, WALL + 0.1)
  assert t.accept(packet(session=a, sequence=11, observed=WALL - 0.8), 100.2, 100.2, WALL + 0.2)
  assert t.route_preview(100.2).key == (b, 20)


def test_an_unusable_late_packet_does_not_clear_the_current_window():
  t = tracker()
  assert t.accept(packet(sequence=20), 100.0, 100.0, WALL)
  assert t.accept(packet(session='12345678-1234-1234-1234-12345678900c', sequence=11, observed=WALL - 5.0),
                  100.1, 100.1, WALL + 0.1)
  assert t.route_preview(100.1).key[1] == 20


def test_stopped_guidance_needs_a_new_window_to_resume():
  """Review 3 T4: A1, guidance off with no preview, guidance on with no preview: A1 does not come back."""
  t = tracker()
  assert t.accept(packet(sequence=1), 100.0, 100.0, WALL)
  stopped = packet(sequence=2, guidance_active=False)
  stopped['roadPreview'] = None
  assert t.accept(stopped, 100.1, 100.1, WALL + 0.1)
  resumed = packet(sequence=3)
  resumed['roadPreview'] = None
  assert t.accept(resumed, 100.2, 100.2, WALL + 0.2)
  assert t.route_preview(100.2) is None
  assert t.accept(packet(sequence=4, observed=WALL + 0.2), 100.3, 100.3, WALL + 0.3)
  assert t.route_preview(100.3).key[1] == 4


def test_an_age_fixed_before_the_clock_was_synced_is_not_used():
  """Review 3 T6."""
  t = tracker()
  synced = [False]
  t.clock_synced = lambda: synced[0]
  assert t.accept(packet(), 100.0, 100.0, WALL)
  synced[0] = True
  t.clock_checked = (-math.inf, False)
  assert t.route_preview(100.1) is None
  assert t.accept(packet(sequence=8, observed=WALL + 0.1), 100.2, 100.2, WALL + 0.2)
  assert t.route_preview(100.2).key[1] == 8


def test_a_future_timestamp_does_not_block_later_observations():
  """Review 4 U4: a phone clock an hour ahead for one packet, then right again."""
  t = tracker()
  assert t.accept(packet(sequence=1), 100.0, 100.0, WALL)
  assert t.accept(packet(sequence=2, observed=WALL + 3600), 100.1, 100.1, WALL + 0.1)
  assert t.route_preview(100.1).key[1] == 1
  assert t.accept(packet(sequence=3, observed=WALL + 1.9), 102.0, 102.0, WALL + 2.0)
  assert t.route_preview(102.0).key[1] == 3


def test_a_late_inactive_guidance_packet_does_not_clear_the_window():
  """Review 4 U5: guidance 20 active with a window, then a late guidance 19 inactive."""
  t = tracker()
  assert t.accept(packet(sequence=20), 100.0, 100.0, WALL)
  late = packet(sequence=19, guidance_active=False)
  late['roadPreview']['sequence'] = 20   # the same window again
  assert t.accept(late, 100.1, 100.1, WALL + 0.1)
  assert t.route_preview(100.1).key[1] == 20


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
  man.route_preview_map = RouteSpeedMap()
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
  assert man.route_preview_map.number == 1
  assert abs(run(True, 3)) == pytest.approx(kph(100), rel=0.02) and man.route_preview_map.number == 1   # same curve
  for navdy, mode in ((True, 1), (False, 2), (True, 0)):
    assert abs(run(navdy, mode)) == NO_LIMIT_KPH and man.route_preview_display_target == 0.0
  # Switched back on with no preview: the old cap does not come back (review: mode 2 -> 0 -> 2).
  reader.route_preview = lambda now: None
  assert abs(run(True, 2)) == NO_LIMIT_KPH
  assert events and events[0][0] == 'navdy_route_preview' and events[0][1]['points'] == 81
