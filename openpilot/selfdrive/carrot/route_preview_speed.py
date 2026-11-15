"""Curve speed from the navigation route preview (TMAP / Naver through the Navdy, 2026-11-15).

The phone matches its position to the planned route and sends the next 400 m as east/north offsets every 5 m, once
per second (selfdrive/navdy/navigation.py RoutePreview). The vision curve speed sees at most about 130 m and 6 s of the
model's path, too late to slow from 75 km/h for a 90 m radius curve (2026-11-15 14:31). This uses the same physics,
lateral budget and approach as curve_speed.py on the route geometry instead, so it can start slowing earlier.
"""
from __future__ import annotations

import math

from openpilot.selfdrive.carrot.curve_speed import (
  APPROACH_DECEL, APPROACH_JERK, NO_LIMIT_KPH, RELEASE_HOLD, RELEASE_RATE_KPH, RESPONSE_TIME, TARGET_LAT_ACCEL, CurveSpeed,
)

CHORD_STEP = 2            # curvature from points i-2, i, i+2: about 20 m of route at 5 m spacing
MIN_CHORD_M = 1.0
MIN_CURVATURE = 0.0005    # 2 km radius: straight for speed purposes


def _curvature(a, b, c):
  """Signed curvature (1/m, left positive in east/north) of the circle through three points, or None."""
  ab, bc, ac = math.dist(a, b), math.dist(b, c), math.dist(a, c)
  if min(ab, bc, ac) < MIN_CHORD_M:
    return None
  cross = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
  return 2.0 * cross / (ab * bc * ac)


def preview_curve_speed(points, age, v_ego, sensitivity=1.0, lower_limit_kph=30.0, *, a_ego=0.0, speed_ratio=1.0):
  """The tightest distance-adjusted speed for the previewed route, as curve_speed() does for the model path.

  points: ((east, north), ...) metres along the route from where the phone matched the car, age seconds ago; the car
  has since moved about v_ego * age along it. A three-sample median rejects a single bad vertex. Returns None for
  invalid input, CurveSpeed() (no limit) when the route ahead is straight.
  """
  if not all(math.isfinite(value) for value in (age, v_ego, sensitivity, lower_limit_kph, speed_ratio, a_ego)):
    return None
  if age < 0.0 or v_ego < 0.0 or sensitivity <= 0.0 or points is None or len(points) < 2 * CHORD_STEP + 3:
    return None
  ratio = speed_ratio if 0.5 < speed_ratio <= 1.2 else 1.0
  distance = [0.0]
  for a, b in zip(points, points[1:]):
    distance.append(distance[-1] + math.dist(a, b))
  raw = [None] * len(points)
  for i in range(CHORD_STEP, len(points) - CHORD_STEP):
    raw[i] = _curvature(points[i - CHORD_STEP], points[i], points[i + CHORD_STEP])
  travelled = v_ego * age
  response_distance = v_ego * (RESPONSE_TIME + (max(0.0, a_ego) + APPROACH_DECEL) / (2.0 * APPROACH_JERK))
  floor_ms = max(5.0, lower_limit_kph) * ratio / 3.6
  lateral_budget = TARGET_LAT_ACCEL / float(min(max(sensitivity, 0.5), 3.0))
  best = CurveSpeed()
  for i in range(CHORD_STEP + 1, len(points) - CHORD_STEP - 1):
    neighbourhood = raw[i - 1:i + 2]
    if any(value is None for value in neighbourhood):
      continue
    curve = sorted(neighbourhood)[1]
    if abs(curve) < MIN_CURVATURE:
      continue
    # Distance from the car now to where this curvature starts (the chord's first point). Until the car passes the
    # chord's middle point it is still in this curve, so it counts at distance 0.
    ahead = distance[i - CHORD_STEP] - travelled
    if ahead < -distance[i] + distance[i - CHORD_STEP] - 1e-9:
      continue
    ahead = max(0.0, ahead)
    curve_ms = max(floor_ms, math.sqrt(lateral_budget / abs(curve)))
    approach_ms = math.sqrt(curve_ms ** 2 + 2.0 * APPROACH_DECEL * max(0.0, ahead - response_distance))
    approach_kph = min(NO_LIMIT_KPH, approach_ms * 3.6 / ratio)
    if approach_kph < best.approach_kph:
      best = CurveSpeed(approach_kph, min(NO_LIMIT_KPH, curve_ms * 3.6 / ratio), float(ahead), math.copysign(1.0, curve))
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
