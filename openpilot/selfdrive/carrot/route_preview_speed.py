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
from collections import deque
from typing import NamedTuple

from openpilot.selfdrive.carrot.curve_speed import (
  APPROACH_DECEL, APPROACH_JERK, NO_LIMIT_KPH, RELEASE_HOLD, RELEASE_RATE_KPH, RESPONSE_TIME, TARGET_LAT_ACCEL,
)

MAP_TOLERANCE_M = 3.0          # how far the drivable curve may be from a map vertex (simplification, corner cut)
SIMPLIFY_M = 1.5               # resampled points within this of a chord are not corners (map zigzags, rounding)
MIN_TURN = math.radians(1.0)   # smaller corners are straight for speed purposes (rounding is 0.01 m per point)
REVERSAL_TURN = math.radians(150.0)   # a U-turn or reversed geometry: no radius can be inferred, the floor applies
SPLIT_VERTEX_M = 5.5            # kept samples one resampling step apart that turn the same way share one map vertex
MERGE_SPAN_M = 60.0            # a curve of corners spanning up to twice this is also checked as one arc
# ... when it turns at least this much: for gentler curves a sample's position error of a metre or two changes the arc
# through it several-fold (a 16 degree bend came out 46 m instead of 156 m), so they keep their vertices' radii.
MERGE_MIN_TURN = math.radians(30.0)
STRAIGHT_M = 0.3               # samples along a straight lie within rounding of it (0.01 m); an arc's start does not
STRAIGHT_SKIP_M = 6.0          # ... except next to the curve, where a sample interval cuts the corner
# Same-direction corners joined by a chord up to this long are one curve. The producers' map segments are at most
# 40 m, so a longer chord is several segments in a line: a straight between two curves. (An arc's chords after
# simplification are about sqrt(8 R SIMPLIFY_M): up to 40 m for a 133 m radius; a gentler arc splits into several
# curves next to each other.)
CURVE_CHORD_M = 40.0
ARC_REACHES = 2.5
CURVE_EXIT_M = 15.0
MIN_POINTS = 7
# The age is end to end (comma clock minus the phone's observation time) but the phone's matched position itself
# lags, and the two clocks differ by about 0.1 s (measured 2026-11-16). Approaching, assume the car is this much
# further along (slow earlier); for a passed curve, this much less (keep it longer).
LAG_MARGIN_S = 1.0
FROZEN_TOLERANCE_M = 0.5


class PreviewCurve(NamedTuple):
  approach_kph: float = NO_LIMIT_KPH   # speed allowed now for the binding corner
  curve_kph: float = NO_LIMIT_KPH      # speed in that corner
  distance: float = 0.0                # from the car (lagged) to where the corner's arc starts, >= 0
  direction: float = 1.0               # +1 left, -1 right
  curve_start: float = math.inf        # from the car now to the start and end of the whole curve the binding corner
  curve_end: float = math.inf          # belongs to (its identity across previews)
  extra_kph2: float = 0.0              # approach_kph**2 - curve_kph**2 before capping: the distance's allowance


class Corner(NamedTuple):
  vertex_m: float   # along the preview from its origin
  start_m: float
  end_m: float
  radius: float     # 0 for a reversal
  turn: float       # signed radians, left positive
  curve_start_m: float = 0.0   # the curve (run of same-direction corners joined by arc chords) it belongs to
  curve_end_m: float = 0.0


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


def _straight(points, along, s0, s1, a, b):
  """Whether the samples between s0 and s1 m along the preview (at least two) lie within STRAIGHT_M of line a-b."""
  inside = [p for p, s in zip(points, along) if s0 - 1e-6 <= s <= s1 + 1e-6]
  if len(inside) < 2:
    return False
  dx, dy = b[0] - a[0], b[1] - a[1]
  length = math.hypot(dx, dy)
  return length > 1e-6 and all(abs((p[0] - a[0]) * dy - (p[1] - a[1]) * dx) / length <= STRAIGHT_M for p in inside)


def _tangent_radius(vertex, a, c, point, half):
  """Radius of the arc tangent to the lines vertex->a and vertex->c (meeting at `vertex`, turning by 2 * half) that
  passes through `point`: the larger root, the arc whose side facing `vertex` goes through the point."""
  ua, uc = (a[0] - vertex[0], a[1] - vertex[1]), (c[0] - vertex[0], c[1] - vertex[1])
  la, lc = math.hypot(*ua), math.hypot(*uc)
  bx, by = ua[0] / la + uc[0] / lc, ua[1] / la + uc[1] / lc
  lb = math.hypot(bx, by)
  if lb < 1e-9:
    return math.inf
  q = (point[0] - vertex[0], point[1] - vertex[1])
  along = (q[0] * bx + q[1] * by) / lb
  secant, tangent2 = 1.0 / math.cos(half), math.tan(half) ** 2
  disc = max(0.0, (secant * along) ** 2 - tangent2 * (q[0] ** 2 + q[1] ** 2))
  return max(0.0, (secant * along + math.sqrt(disc)) / tangent2)


def _merged(points, along, verts, singles, groups):
  """A curve the map draws with a few vertices: a sharp turn split over close vertices (three 30 degree vertices 5 m
  apart) is one tight corner though each vertex alone looks gentle, and a short arc drawn with two or three vertices
  between straights has the radius of its circle. Map vertices lie on the road, so the arc tangent to the lines the
  curve comes from and goes to passes through its vertices."""
  out = []
  turn_at = {j: corner.turn for j, corner in singles}
  for group in groups:
    run = [singles[i] for i in group]
    first, last = run[0][0], run[-1][0]
    total = sum(c[1].turn for c in run)
    if (len(run) < 2 or not MERGE_MIN_TURN <= abs(total) < REVERSAL_TURN
        or verts[last][1] - verts[first][1] > 2 * MERGE_SPAN_M):
      continue
    # A neighbour turning the same way beyond a long chord: a gentle arc the simplification drew with long chords,
    # whose outer chords are not the straights the curve is tangent to.
    if turn_at.get(first - 1, 0.0) * total > 0 or turn_at.get(last + 1, 0.0) * total > 0:
      continue
    a, c = verts[first - 1][0], verts[last + 1][0]
    # The outer chords must be straights the curve is tangent to. The simplification can also draw the start of an
    # arc into a chord (at the window's edges: the car already in the curve, or the window ending in it; or a straight
    # and the arc's first metres together): then the samples along it are not on one line.
    if (not _straight(points, along, verts[first - 1][1], verts[first][1] - STRAIGHT_SKIP_M, a, verts[first][0])
        or not _straight(points, along, verts[last][1] + STRAIGHT_SKIP_M, verts[last + 1][1], verts[last][0], c)):
      continue
    vertex = _meet(a, verts[first][0], verts[last][0], c)   # where the outer lines meet
    if vertex is None:
      continue
    # The median: a vertex rebuilt where two chords meet sits a little outside a densely drawn arc, a sample kept
    # between map vertices a little inside it; a sharp corner's vertices are all near the meeting point.
    radii = sorted(_tangent_radius(vertex, a, c, verts[i][0], abs(total) / 2.0) for i in range(first, last + 1))
    radius = (radii[(len(radii) - 1) // 2] + radii[len(radii) // 2]) / 2.0
    if radius < min(corner.radius for _, corner in run):
      # The arc meets the outer lines radius * tan(half turn) from where they meet: before the first vertex only if
      # that vertex is nearer to the meeting point than that.
      reach = radius * math.tan(abs(total) / 2.0)
      start = run[0][1].vertex_m - max(0.0, reach - math.dist(vertex, verts[first][0]))
      end = run[-1][1].vertex_m + max(0.0, reach - math.dist(vertex, verts[last][0]))
      out.append(Corner((run[0][1].vertex_m + run[-1][1].vertex_m) / 2.0, start, end, radius, total,
                        min(run[0][1].curve_start_m, start), max(run[-1][1].curve_end_m, end)))
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
  # A curve: neighbouring corners turning the same way joined by an arc chord (not a straight between two curves).
  # Within it the road still has a corner's radius up to the next vertex (the limit lasts until there), and already
  # from the previous one when that is as near as an arc's next vertex would be (a longer chord may be a straight
  # between a bend and a sharper turn: review 4 U6). The whole curve is the identity of whichever corner binds.
  groups, group = [], []
  for i, (j, corner) in enumerate(singles):
    joined = (group and singles[i - 1][0] == j - 1 and singles[i - 1][1].turn * corner.turn > 0
              and verts[j][1] - verts[j - 1][1] <= CURVE_CHORD_M)
    if not joined and group:
      groups.append(group)
      group = []
    group.append(i)
  if group:
    groups.append(group)
  for group in groups:
    # Toward the road outside the curve a vertex's reach is its own turn's arc, which for a vertex where an arc meets
    # a straight comes out long and gentle: keep it short there, so the curve's limit ends near its end.
    first_j, last_j = singles[group[0]][0], singles[group[-1]][0]
    for i in (group[0], group[-1]):
      j, corner = singles[i]
      singles[i] = (j, corner._replace(start_m=max(corner.start_m, corner.vertex_m - CURVE_EXIT_M) if j == first_j else corner.start_m,
                                       end_m=min(corner.end_m, corner.vertex_m + CURVE_EXIT_M) if j == last_j else corner.end_m))
    curve_start, curve_end = singles[group[0]][1].start_m, singles[group[-1]][1].end_m
    for position, i in enumerate(group):
      j, corner = singles[i]
      end = singles[group[position + 1]][1].vertex_m if position + 1 < len(group) else corner.end_m
      start = corner.start_m
      if position > 0:
        previous = singles[group[position - 1]][1].vertex_m
        # An arc's vertices are about two reaches apart; a longer chord before the corner may be a straight.
        if corner.vertex_m - previous <= ARC_REACHES * (corner.vertex_m - corner.start_m):
          start = previous
      singles[i] = (j, corner._replace(start_m=min(start, corner.start_m), end_m=max(end, corner.end_m),
                                       curve_start_m=curve_start, curve_end_m=curve_end))
  along = [0.0]
  for a, b in zip(points, points[1:]):
    along.append(along[-1] + math.dist(a, b))
  return [c for _, c in singles] + _merged(points, along, verts, singles, groups)


def preview_curve_speed(points, age, v_ego, sensitivity=1.0, lower_limit_kph=30.0, *, a_ego=0.0, speed_ratio=1.0,
                        lag=LAG_MARGIN_S, min_age=None):
  """The tightest distance-adjusted speed for the previewed route, with the vision curve speed's physics.

  points: ((east, north), ...) metres along the route from where the phone matched the car, age seconds ago; the car
  has since moved about v_ego * age along it. min_age, if given, is a lower bound of the age that needs no clock
  (seconds since the comma received it): a corner counts as passed only by that, less the lag. Returns None for
  invalid input, PreviewCurve() (no limit) when the route ahead has no corner that limits.
  """
  min_age = age if min_age is None else min_age
  if not all(math.isfinite(value) for value in (age, min_age, v_ego, sensitivity, lower_limit_kph, speed_ratio, a_ego,
                                               lag)):
    return None
  if (age < 0.0 or min_age < 0.0 or min_age > age or v_ego < 0.0 or sensitivity <= 0.0 or lag < 0.0 or points is None
      or len(points) < MIN_POINTS):
    return None
  ratio = speed_ratio if 0.5 < speed_ratio <= 1.2 else 1.0
  travelled = v_ego * age
  ahead_of = v_ego * (age + lag)                 # for approach: the car may be further along
  passed_at = v_ego * max(0.0, min_age - lag)    # for a passed corner: it may be less far along
  response_distance = v_ego * (RESPONSE_TIME + (max(0.0, a_ego) + APPROACH_DECEL) / (2.0 * APPROACH_JERK))
  floor_ms = max(5.0, lower_limit_kph) * ratio / 3.6
  lateral_budget = TARGET_LAT_ACCEL / float(min(max(sensitivity, 0.5), 3.0))
  best = PreviewCurve()
  for corner in corners(points):
    if passed_at > corner.end_m + 1e-9:
      continue
    ahead = max(0.0, corner.start_m - ahead_of)
    curve_ms = max(floor_ms, math.sqrt(lateral_budget * corner.radius))
    allowance = 2.0 * APPROACH_DECEL * max(0.0, ahead - response_distance)
    approach_kph = min(NO_LIMIT_KPH, math.sqrt(curve_ms ** 2 + allowance) * 3.6 / ratio)
    if approach_kph < best.approach_kph:
      best = PreviewCurve(approach_kph, min(NO_LIMIT_KPH, curve_ms * 3.6 / ratio), float(ahead),
                          math.copysign(1.0, corner.turn), corner.curve_start_m - travelled,
                          corner.curve_end_m - travelled, allowance * (3.6 / ratio) ** 2)
  return best


def same_points(a, b, tolerance=FROZEN_TOLERANCE_M):
  return a is not None and b is not None and len(a) == len(b) and all(
    abs(p[0] - q[0]) <= tolerance and abs(p[1] - q[1]) <= tolerance for p, q in zip(a, b))


class FrozenPreviewGuard:
  """A phone whose matched position stopped updating keeps sending the same window as new observations. On a
  straight road a moving car's window is the same anyway, so only a window with a clear corner can show it: across
  observations the corner must come nearer as the car moves. New observations (new keys) bringing the same window
  (within FROZEN_TOLERANCE_M) once the car has driven FROZEN_AFTER_M since that window first arrived are rejected;
  the same observation re-read every cycle is not judged again."""
  FROZEN_AFTER_M = 10.0
  CLEAR_TURN = math.radians(10.0)   # moving 10 m shifts the points past such a corner by 1.7 m

  def __init__(self):
    self.key = None
    self.points = None
    self.same_since_m = 0.0   # odometer when the current window first arrived (kept while new keys repeat it)
    self.frozen = False
    self.odometer = 0.0
    self.last_time = None

  def update(self, preview, v_ego, now):
    """preview -> preview, or None while it is frozen."""
    if self.last_time is not None and math.isfinite(v_ego) and v_ego > 0.0:
      self.odometer += v_ego * max(0.0, min(now - self.last_time, 0.2))
    self.last_time = now
    if preview is None:
      return None
    if preview.key != self.key:
      if not same_points(preview.points, self.points):
        self.points, self.same_since_m = preview.points, self.odometer
      self.frozen = (self.key is not None and preview.points is not self.points and
                     self.odometer - self.same_since_m >= self.FROZEN_AFTER_M and
                     any(abs(c.turn) >= self.CLEAR_TURN for c in corners(preview.points)))
      self.key = preview.key
    return None if self.frozen else preview


class _Cell:
  __slots__ = ("good", "partial", "curve", "direction", "early")

  def __init__(self):
    self.good = deque(maxlen=RouteSpeedMap.GOOD_VIEWS)   # speeds from good views (inf: seen straight)
    self.partial = math.inf                               # tightest from partial views
    self.curve = None                                     # (start, end) odometer of its curve
    self.direction = 1.0
    self.early = 0.0

  def speed(self):
    if not self.good:
      return self.partial
    good = sorted(self.good)[(len(self.good) - 1) // 2]   # lower median: one odd view either way does not decide
    return good if not math.isfinite(good) else min(good, self.partial)


class RouteSpeedMap:
  """The previewed curve speeds laid on the road (odometer metres, cells of MAP_STEP_M), kept across previews.

  Each preview simplifies a window that has moved, so a curve's estimate changes from preview to preview, worst where
  the window's start or end cuts into it, and the last metres of a curve the car is in can drop out. A cell takes the
  speed of every corner whose stretch overlaps it (a U-turn's zero-length one too). Road a preview sees well (more
  than NEAR_M ahead of the car and EDGE_M before the window's end) is a vote: the cell's speed is the lower median of
  its last GOOD_VIEWS good views (a curve, or straight), so one view estimating it too tight or too gentle does not
  decide (reviews 5 V2, 6 W1). Partial views only tighten, and only where no good view has seen it or the good
  views see a curve, never road they see straight. The speed for now is the tightest distance-adjusted one over the
  road ahead, until the car is past it.

  A missing preview gives no result at once (the limiter holds, then releases); the cells are kept LOST_S for it to
  come back. A new route, a lost preview, or a gap in the updates (the odometer cannot follow) starts it again.
  """
  NEAR_M = 40.0
  EDGE_M = 40.0
  MAP_STEP_M = 5.0
  LOST_S = 1.0
  MAX_GAP_S = 0.5
  SAME_GAP_M = 10.0
  TIGHTER = 0.85     # a curve this much slower is another curve
  APART_M = 100.0    # ... and so is one this far apart
  GOOD_VIEWS = 7

  def __init__(self):
    self.route = None
    self.key = None
    self.cells = {}        # index -> _Cell
    self.missing_since = None
    self.last_time = None
    self.number = 0        # changes when a different curve binds (an accelerator floor resets)
    self.binding = None    # (start, end, direction, curve m/s) of that curve

  def clear(self):
    self.route = self.key = self.binding = self.missing_since = None
    self.cells = {}

  def _write(self, index, speed, good, curve, direction, early):
    cell = self.cells.get(index)
    if cell is None:
      cell = self.cells[index] = _Cell()
    if good:
      cell.good.append(speed)
    else:
      cell.partial = min(cell.partial, speed)
    if math.isfinite(speed):
      cell.curve, cell.direction = curve, direction
    cell.early = early

  def observe(self, preview, odometer, v_ego, sensitivity=1.0, lower_limit_kph=30.0, speed_ratio=1.0,
              lag=LAG_MARGIN_S):
    """Lay a new observation on the road. preview: RoutePreview with points, age, key, min_age, route."""
    if preview.route != self.route:
      self.clear()
      self.route = preview.route
    if preview.key == self.key:
      return
    self.key = preview.key
    ratio = speed_ratio if 0.5 < speed_ratio <= 1.2 else 1.0
    floor_ms = max(5.0, lower_limit_kph) * ratio / 3.6
    budget = TARGET_LAT_ACCEL / float(min(max(sensitivity, 0.5), 3.0))
    # Odometer of the preview's origin. The clock age, less the lag margin, but never less than the clock-free time
    # since arrival: the road is not placed behind where it is unless the phone's clock is more than LAG_MARGIN_S
    # behind the comma's, nor more than about LAG_MARGIN_S of driving later (review 5 V5). `early` is how much
    # farther it may be (approach side).
    placed_age = max(preview.min_age, preview.age - lag)
    origin = odometer - v_ego * placed_age
    early = v_ego * max(0.0, preview.age - placed_age)
    length = 0.0
    for a, b in zip(preview.points, preview.points[1:]):
      length += math.dist(a, b)
    step = self.MAP_STEP_M

    def good(index):
      s = index * step - origin
      return s - v_ego * preview.age >= self.NEAR_M and s <= length - self.EDGE_M

    first, last = int(math.ceil(origin / step)), int(math.floor((origin + length) / step))
    tightest = {}
    for corner in corners(preview.points):
      speed = max(floor_ms, math.sqrt(budget * corner.radius))
      curve = (origin + corner.curve_start_m, origin + corner.curve_end_m)
      lo = int(math.ceil((origin + corner.start_m - step / 2) / step))
      hi = int(math.floor((origin + corner.end_m + step / 2) / step))
      for index in range(max(lo, first), min(hi, last) + 1):
        if index not in tightest or speed < tightest[index][0]:
          tightest[index] = (speed, curve, math.copysign(1.0, corner.turn))
    for index in range(first, last + 1):
      speed, curve, direction = tightest.get(index, (math.inf, None, 1.0))
      is_good = good(index)
      if is_good or math.isfinite(speed):
        self._write(index, speed, is_good, curve, direction, early)

  def update(self, preview, now, odometer, v_ego, a_ego=0.0, sensitivity=1.0, lower_limit_kph=30.0, speed_ratio=1.0,
             lag=LAG_MARGIN_S):
    """PreviewCurve for now, or None when there is no preview."""
    if self.last_time is not None and not 0.0 <= now - self.last_time <= self.MAX_GAP_S:
      self.clear()
    self.last_time = now
    if preview is None:
      if self.missing_since is None:
        self.missing_since = now
      if now - self.missing_since >= self.LOST_S:
        self.clear()
      return None
    self.missing_since = None
    self.observe(preview, odometer, v_ego, sensitivity, lower_limit_kph, speed_ratio, lag)
    ratio = speed_ratio if 0.5 < speed_ratio <= 1.2 else 1.0
    for index in [i for i in self.cells if i * self.MAP_STEP_M < odometer - self.MAP_STEP_M]:
      del self.cells[index]   # behind the car: passed
    response_distance = v_ego * (RESPONSE_TIME + (max(0.0, a_ego) + APPROACH_DECEL) / (2.0 * APPROACH_JERK))
    best, chosen = PreviewCurve(), None
    for index, cell in self.cells.items():
      speed = cell.speed()
      if not math.isfinite(speed) or cell.curve is None:
        continue
      ahead = max(0.0, index * self.MAP_STEP_M - cell.early - odometer - v_ego * lag)
      allowance = 2.0 * APPROACH_DECEL * max(0.0, ahead - response_distance)
      approach_kph = min(NO_LIMIT_KPH, math.sqrt(speed ** 2 + allowance) * 3.6 / ratio)
      if approach_kph < best.approach_kph:
        best = PreviewCurve(approach_kph, min(NO_LIMIT_KPH, speed * 3.6 / ratio), float(ahead), cell.direction,
                            cell.curve[0] - odometer, cell.curve[1] - odometer, allowance * (3.6 / ratio) ** 2)
        chosen = (cell.curve[0], cell.curve[1], cell.direction, speed)
    if chosen is not None:
      previous = self.binding
      # Another curve (the accelerator floor resets): the other direction; a stretch apart that is clearly tighter
      # (the driver's speed was for the gentler one, review 7 X1); or one far apart. Parts of one long gentle arc
      # (drawn as several curves with similar speeds) and stretches that overlap stay one curve (review 6 W2).
      if previous is None or chosen[2] != previous[2]:
        is_new = True
      elif chosen[0] <= previous[1] + self.SAME_GAP_M and chosen[1] >= previous[0] - self.SAME_GAP_M:
        is_new = False
      else:
        gap = max(chosen[0] - previous[1], previous[0] - chosen[1])
        is_new = chosen[3] < previous[3] * self.TIGHTER or gap > self.APART_M
      if is_new:
        self.number += 1
        self.binding = chosen
      else:
        self.binding = (min(previous[0], chosen[0]), max(previous[1], chosen[1]), chosen[2], min(previous[3], chosen[3]))
    return best


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
