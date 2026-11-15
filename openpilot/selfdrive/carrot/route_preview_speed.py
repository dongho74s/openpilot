"""Curve speed from the navigation route preview (TMAP / Naver through the Navdy, 2026-11-15).

The phone matches its position to the planned route and sends the next 400 m as east/north offsets every 5 m, once
per second (selfdrive/navdy/navigation.py RoutePreview). The vision curve speed sees at most about 130 m and 6 s of the
model's path, too late to slow from 75 km/h for a 90 m radius curve (2026-11-15 14:31). This uses the same lateral
budget, approach deceleration and release as curve_speed.py on the route geometry, so it can start slowing earlier.

The 5 m points are a linear resampling of the map's polyline, whose vertices can be up to 40 m apart (the producer's
gap limit): all of a corner's turn sits between two samples, so curvature from neighbouring samples would see a sharp
kink where the road has a 86 m radius (review 2026-11-16). Instead the resampled points are simplified back to their
corners (Douglas-Peucker, SIMPLIFY_M), and each corner's radius comes from its turn and the length of the chords on
either side, as for vertices on a circle, but never more than a path within MAP_TOLERANCE_M of the corner allows.
"""
from __future__ import annotations

import math
from typing import NamedTuple

from openpilot.selfdrive.carrot.curve_speed import (
  APPROACH_DECEL, APPROACH_JERK, NO_LIMIT_KPH, RELEASE_HOLD, RELEASE_RATE_KPH, RESPONSE_TIME, TARGET_LAT_ACCEL,
)

MAP_TOLERANCE_M = 3.0          # how far the drivable curve may be from a map vertex (simplification, corner cut)
SIMPLIFY_M = 1.5               # resampled points within this of a chord are not corners (map zigzags, rounding)
MIN_TURN = math.radians(1.0)   # smaller corners are straight for speed purposes (rounding is 0.01 m per point)
REVERSAL_TURN = math.radians(150.0)   # a U-turn or reversed geometry: no radius can be inferred, the floor applies
SPLIT_VERTEX_M = 5.5            # kept samples one resampling step apart that turn the same way share one map vertex
MERGE_SPAN_M = 60.0            # neighbouring corners closer than this along the route are also checked as one
LANE_FREEDOM_M = 1.5           # how far inside a run of corners the car can cut while staying in its lane
MIN_POINTS = 7
# The age is end to end (comma clock minus the phone's observation time) but the phone's matched position itself
# lags, and the two clocks differ by about 0.1 s (measured 2026-11-16). Approaching, assume the car is this much
# further along (slow earlier); for a passed curve, this much less (keep it longer).
LAG_MARGIN_S = 1.0


class PreviewCurve(NamedTuple):
  approach_kph: float = NO_LIMIT_KPH   # speed allowed now for the binding corner
  curve_kph: float = NO_LIMIT_KPH      # speed in that corner
  distance: float = 0.0                # from the car (lagged) to where the corner's arc starts, >= 0
  direction: float = 1.0               # +1 left, -1 right
  vertex_m: float = math.inf           # from the car now to the corner's vertex (identity of the binding corner)


class Corner(NamedTuple):
  vertex_m: float   # along the preview from its origin
  start_m: float
  end_m: float
  radius: float     # 0 for a reversal
  turn: float       # signed radians, left positive


def _segment_distance(p, a, b):
  ax, ay = b[0] - a[0], b[1] - a[1]
  length2 = ax * ax + ay * ay
  if length2 < 1e-12:
    return math.dist(p, a)
  t = max(0.0, min(1.0, ((p[0] - a[0]) * ax + (p[1] - a[1]) * ay) / length2))
  return math.hypot(p[0] - (a[0] + t * ax), p[1] - (a[1] + t * ay))


def simplify(points, tolerance=SIMPLIFY_M):
  """Douglas-Peucker: indices of the points that stay, ends included (distance to the segment, not the line, so a
  reversal keeps its turning point)."""
  keep = {0, len(points) - 1}
  stack = [(0, len(points) - 1)]
  while stack:
    first, last = stack.pop()
    farthest, index = -1.0, -1
    for i in range(first + 1, last):
      distance = _segment_distance(points[i], points[first], points[last])
      if distance > farthest:
        farthest, index = distance, i
    if index >= 0 and farthest > tolerance:
      keep.add(index)
      stack += [(first, index), (index, last)]
  return sorted(keep)


def _cross(u, w):
  return u[0] * w[1] - u[1] * w[0]


def _turn(a, b, c):
  u, w = (b[0] - a[0], b[1] - a[1]), (c[0] - b[0], c[1] - b[1])
  return math.atan2(_cross(u, w), u[0] * w[0] + u[1] * w[1])


def _meet(a, b, d, c):
  """Where the line a->b meets the line d->c, or None when they are (nearly) parallel."""
  u, w = (b[0] - a[0], b[1] - a[1]), (c[0] - d[0], c[1] - d[1])
  denominator = _cross(u, w)
  if abs(denominator) < 1e-9:
    return None
  t = _cross((d[0] - b[0], d[1] - b[1]), w) / denominator
  return b[0] + t * u[0], b[1] + t * u[1]


def vertices(points):
  """The map polyline behind the resampled points: [(point, metres along the preview)], ends included.

  The resampling puts no sample on a map vertex, so a vertex's turn is shared by the two samples around it, one step
  apart; those are rebuilt into the vertex where the chords on either side meet."""
  along = [0.0]
  for a, b in zip(points, points[1:]):
    along.append(along[-1] + math.dist(a, b))
  kept = [(points[i], along[i]) for i in simplify(points)]
  out = [kept[0]]
  i = 1
  while i < len(kept) - 1:
    j = i
    while (j + 1 < len(kept) - 1 and kept[j + 1][1] - kept[j][1] <= SPLIT_VERTEX_M
           and _turn(kept[j - 1][0], kept[j][0], kept[j + 1][0]) * _turn(kept[j][0], kept[j + 1][0], kept[j + 2][0]) > 0):
      j += 1
    vertex = _meet(out[-1][0], kept[i][0], kept[j][0], kept[j + 1][0]) if j > i else None
    if vertex is not None and _segment_distance(vertex, kept[i][0], kept[j][0]) <= SPLIT_VERTEX_M:
      out.append((vertex, (kept[i][1] + kept[j][1]) / 2.0))
    else:
      out.extend(kept[i:j + 1])
    i = j + 1
  out.append(kept[-1])
  return out


def _merged(verts, singles):
  """Corners from runs of neighbouring same-direction corners: a sharp turn the map draws as a few close vertices
  (three 30 degree vertices 5 m apart) is one tight corner, though each vertex alone looks gentle. The drivable arc
  must turn the run's whole angle while staying within LANE_FREEDOM_M of the run's inner chain."""
  out = []
  for j in range(len(singles)):
    for k in range(j + 1, len(singles)):
      run = singles[j:k + 1]
      first, last = run[0][0], run[-1][0]
      if (any(run[i + 1][0] != run[i][0] + 1 for i in range(len(run) - 1))
          or any(c[1].turn * run[0][1].turn <= 0 for c in run) or verts[last][1] - verts[first][1] > MERGE_SPAN_M):
        break
      total = sum(c[1].turn for c in run)
      if abs(total) >= REVERSAL_TURN:
        break
      vertex = _meet(verts[first - 1][0], verts[first][0], verts[last][0], verts[last + 1][0])   # outer lines
      if vertex is None:
        continue
      chain = [verts[i][0] for i in range(first, last + 1)]
      inset = min(_segment_distance(vertex, p, q) for p, q in zip(chain, chain[1:]))
      radius = (inset + LANE_FREEDOM_M) / (1.0 - math.cos(abs(total) / 2.0))
      if radius < min(c[1].radius for c in run):
        out.append(Corner((run[0][1].vertex_m + run[-1][1].vertex_m) / 2.0, run[0][1].start_m, run[-1][1].end_m,
                          radius, total))
  return out


def corners(points, tolerance=MAP_TOLERANCE_M):
  """The preview's corners (single vertices, then merged runs)."""
  verts = vertices(points)
  singles = []
  for j in range(1, len(verts) - 1):
    a, b, c = verts[j - 1][0], verts[j][0], verts[j + 1][0]
    s1, s2 = math.dist(a, b), math.dist(b, c)
    if s1 < 1e-6 or s2 < 1e-6:
      continue
    turn = _turn(a, b, c)
    if abs(turn) < MIN_TURN:
      continue
    # The first and last chords end at the window edge, not at a route vertex: their real length is unknown.
    if j == 1:
      s1 = max(s1, s2)
    if j == len(verts) - 2:
      s2 = max(s1, s2)
    half = abs(turn) / 2.0
    if abs(turn) >= REVERSAL_TURN:
      radius, reach_in, reach_out = 0.0, 0.0, 0.0
    else:
      # Vertices on a circle of radius R with chords s1, s2 turn by about (s1 + s2) / 2R; a drivable arc must also
      # stay within the tolerance of the corner itself.
      radius = min((s1 + s2) / 4.0 / math.sin(half), tolerance / (1.0 - math.cos(half)))
      tangent = radius * math.tan(half)
      reach_in, reach_out = min(s1 / 2.0, tangent), min(s2 / 2.0, tangent)
    vertex = verts[j][1]
    singles.append((j, Corner(vertex, vertex - reach_in, vertex + reach_out, radius, turn)))
  # Within one curve (neighbouring vertices turning the same way) the road already has this radius at the previous
  # vertex and still at the next: the corner's limit covers both.
  for i, (j, corner) in enumerate(singles):
    start, end = corner.start_m, corner.end_m
    if i > 0 and singles[i - 1][0] == j - 1 and singles[i - 1][1].turn * corner.turn > 0:
      start = min(start, singles[i - 1][1].vertex_m)
    if i + 1 < len(singles) and singles[i + 1][0] == j + 1 and singles[i + 1][1].turn * corner.turn > 0:
      end = max(end, singles[i + 1][1].vertex_m)
    singles[i] = (j, corner._replace(start_m=start, end_m=end))
  return [c for _, c in singles] + _merged(verts, singles)


def preview_curve_speed(points, age, v_ego, sensitivity=1.0, lower_limit_kph=30.0, *, a_ego=0.0, speed_ratio=1.0,
                        lag=LAG_MARGIN_S):
  """The tightest distance-adjusted speed for the previewed route, with the vision curve speed's physics.

  points: ((east, north), ...) metres along the route from where the phone matched the car, age seconds ago; the car
  has since moved about v_ego * age along it. Returns None for invalid input, PreviewCurve() (no limit) when the route
  ahead has no corner that limits.
  """
  if not all(math.isfinite(value) for value in (age, v_ego, sensitivity, lower_limit_kph, speed_ratio, a_ego, lag)):
    return None
  if age < 0.0 or v_ego < 0.0 or sensitivity <= 0.0 or lag < 0.0 or points is None or len(points) < MIN_POINTS:
    return None
  ratio = speed_ratio if 0.5 < speed_ratio <= 1.2 else 1.0
  travelled = v_ego * age
  ahead_of = v_ego * (age + lag)              # for approach: the car may be further along
  passed_at = v_ego * max(0.0, age - lag)     # for a passed corner: it may be less far along
  response_distance = v_ego * (RESPONSE_TIME + (max(0.0, a_ego) + APPROACH_DECEL) / (2.0 * APPROACH_JERK))
  floor_ms = max(5.0, lower_limit_kph) * ratio / 3.6
  lateral_budget = TARGET_LAT_ACCEL / float(min(max(sensitivity, 0.5), 3.0))
  best = PreviewCurve()
  for corner in corners(points):
    if passed_at > corner.end_m + 1e-9:
      continue
    ahead = max(0.0, corner.start_m - ahead_of)
    curve_ms = max(floor_ms, math.sqrt(lateral_budget * corner.radius))
    approach_ms = math.sqrt(curve_ms ** 2 + 2.0 * APPROACH_DECEL * max(0.0, ahead - response_distance))
    approach_kph = min(NO_LIMIT_KPH, approach_ms * 3.6 / ratio)
    if approach_kph < best.approach_kph:
      best = PreviewCurve(approach_kph, min(NO_LIMIT_KPH, curve_ms * 3.6 / ratio), float(ahead),
                          math.copysign(1.0, corner.turn), corner.vertex_m - travelled)
  return best


def same_points(a, b, tolerance=0.05):
  return a is not None and b is not None and len(a) == len(b) and all(
    abs(p[0] - q[0]) <= tolerance and abs(p[1] - q[1]) <= tolerance for p, q in zip(a, b))


class FrozenPreviewGuard:
  """A phone whose matched position stopped updating keeps sending the same window as new observations. On a
  straight road a moving car's window is the same anyway, so only a window with a corner can show it: the corner
  must come nearer as the car moves. Identical windows with a corner after FROZEN_AFTER_M of driving are rejected
  until the window changes."""
  FROZEN_AFTER_M = 10.0

  def __init__(self):
    self.points = None
    self.changed_at_m = 0.0
    self.odometer = 0.0
    self.last_time = None

  def update(self, preview, v_ego, now):
    """preview -> preview, or None while it is frozen."""
    if self.last_time is not None and math.isfinite(v_ego) and v_ego > 0.0:
      self.odometer += v_ego * max(0.0, min(now - self.last_time, 0.2))
    self.last_time = now
    if preview is None:
      return None
    if not same_points(preview.points, self.points):
      self.points = preview.points
      self.changed_at_m = self.odometer
      return preview
    if self.odometer - self.changed_at_m >= self.FROZEN_AFTER_M and corners(preview.points):
      return None
    return preview


class RoutePreviewCurveSpeed:
  """Tighten at once; release at RELEASE_RATE_KPH. The preview is re-evaluated every cycle with its age, so passing a
  curve raises the target smoothly; a missing preview holds the cap for RELEASE_HOLD before releasing it."""

  def __init__(self):
    self.speed = NO_LIMIT_KPH
    self.direction = 1.0
    self.last_time = None
    self.missing_since = None

  def update(self, result, now):
    if not math.isfinite(now):
      return self.speed * self.direction
    dt = 0.0 if self.last_time is None else max(0.0, min(now - self.last_time, 0.2))
    self.last_time = now
    if result is not None and not math.isfinite(result.approach_kph):
      result = None
    if result is not None:
      self.missing_since = None
      if result.approach_kph <= self.speed:
        self.speed = result.approach_kph
        self.direction = result.direction
      else:
        self.speed = min(result.approach_kph, self.speed + RELEASE_RATE_KPH * dt)
    else:
      if self.missing_since is None:
        self.missing_since = now
      if now - self.missing_since >= RELEASE_HOLD:
        self.speed = min(NO_LIMIT_KPH, self.speed + RELEASE_RATE_KPH * dt)
    return self.speed * self.direction
