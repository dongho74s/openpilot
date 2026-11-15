"""Move the car sideways in laneless driving by shifting what the driving model sees.

Laneless steering follows the model's own curvature (controlsd uses modelV2.action), so offsets added to the
lateral planner's path never reach the wheel. Instead, the model input warp is changed so the model sees the road
from a camera moved sideways over the road plane (a virtual lateral translation, like sunnypilot's CameraOffset).
The model then steers back to where it wants to be, which puts the car about that far to the other side. This works
for every internal or external model and in lane mode too.

Two settings (centimetres, + moves the car right):
- LanelessCenterOffset: always applied (after calibration).
- LanelessEdgeOffset: only on an asymmetric road, when one side has a lane next to ours and the other side ends at
  the road edge next to our line, toward that edge; similar to lane mode's AdjustLaneOffset (lane_planner_2.py).
  Both sides: toward the curb in the rightmost lane, toward the median in a first lane next to a median (and away
  from the centre line on a two-lane road). Limited so the car keeps a margin in its lane.

The road shape comes from DesireHelper, evaluated where the lane lines are about 19 m ahead (its "t = 1" sample),
as line-to-line and line-to-edge distances, which do not move with the car or with this offset.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from openpilot.common.transformations.model import medmodel_intrinsics, sbigmodel_intrinsics

MAX_OFFSET_M = 0.35          # sunnypilot allows the same range for its CameraOffset
CENTER_LIMIT_CM = 35
EDGE_LIMIT_CM = 35
DEFAULT_HEIGHT_M = 1.22
HEIGHT_RANGE_M = (0.5, 3.0)
FILTER_TAU_S = 2.5
MAX_RATE_M_S = 0.1           # change of the virtual camera position, also when the calibrated height changes
ZERO_SNAP_M = 0.005          # back at the stock warp below 5 mm once the target is 0

# asymmetric road (side lane widths are line-to-line or line-to-edge distances, so they do not move with the car)
LANE_EXISTS_M = 2.2
NO_ROOM_M = 2.0
MIN_LINE_PROB = 0.5
VEHICLE_WIDTH_M = 2.0             # most cars, the Traverse included, are at most this wide
EDGE_MARGIN_M = 0.15               # kept between the car and our lane line after the edge offset
OWN_WIDTH_MAX_M = (4.2, 4.6)       # a very wide "lane" means the lines are not ours
SPEED_RAMP_KPH = (25.0, 35.0)
HOLD_ON_S = 1.5
HOLD_OFF_S = 3.0                   # release when the road shape is only unclear (grey zone, faint lines)
FAST_RELEASE_S = 0.5               # release when it clearly no longer matches (lane on the edge side, lane side gone)
AFTER_LANE_CHANGE_S = 2.0
OUTER_LINE_PROB = 0.3              # the next line out on the lane side must be seen to count it as a lane
CURVE_START_ACCEL = 1.5            # m/s^2: no new decision in sharper curves, where far lane widths look wider
FILTER_TAU_DOWN_S = 1.0            # faster return toward 0 than build-up


def _finite(*values) -> bool:
  try:
    return all(math.isfinite(float(v)) for v in values)
  except (TypeError, ValueError):
    return False


def shear_matrix(slope: float, bigmodel_frame: bool) -> np.ndarray:
  """Matrix S in model-input pixels for a virtual camera moved right by slope * height.

  In the road-aligned model frame a road point (X ahead, Y right, Z = height down) is at u = fx*Y/X + cx,
  v = fy*h/X + cy. From a camera moved right by d it is at u - fx*d/X = u - (fx/fy)*(d/h)*(v - cy): each pixel of
  the moved view samples the real view (fx/fy)*(d/h)*(v - cy) further right, so get_warp_matrix(...) @ S is that
  view. Exact for the road plane; objects above it shift less, like they do for a real camera move.
  """
  intrinsics = sbigmodel_intrinsics if bigmodel_frame else medmodel_intrinsics
  fx, fy, cy = float(intrinsics[0, 0]), float(intrinsics[1, 1]), float(intrinsics[1, 2])
  k = (fx / fy) * slope
  return np.array([[1.0, k, -k * cy], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])


def shear_transform(transform: np.ndarray, slope: float, bigmodel_frame: bool) -> np.ndarray:
  if slope == 0.0:
    return transform
  return (transform.astype(np.float64) @ shear_matrix(slope, bigmodel_frame)).astype(np.float32)


def offset_slope(offset_right_m: float, height_m: float) -> float:
  """Moving the car right by the offset means showing the model the view from a camera moved left by it."""
  return -offset_right_m / height_m


def shift_transform(transform: np.ndarray, offset_right_m: float, height_m: float, bigmodel_frame: bool) -> np.ndarray:
  return shear_transform(transform, offset_slope(offset_right_m, height_m), bigmodel_frame)


def calibration_height(live_calibration, previous: float) -> float:
  """The calibrated camera height when it is plausible, else the last good one."""
  try:
    height = float(live_calibration.height[0])
  except (AttributeError, IndexError, TypeError, ValueError):
    return previous
  return height if _finite(height) and HEIGHT_RANGE_M[0] <= height <= HEIGHT_RANGE_M[1] else previous


def setting_m(value_cm, limit_cm: int, allow_negative: bool) -> float:
  if not _finite(value_cm):
    return 0.0
  return float(np.clip(float(value_cm), -limit_cm if allow_negative else 0.0, limit_cm)) / 100.0


@dataclass
class EdgeInputs:
  left_width: float      # DesireHelper side lane width: our left line to the next line or road edge
  right_width: float
  left_edge: float       # DesireHelper: our left line to the left road edge
  right_edge: float
  left_prob: float       # our own lane lines
  right_prob: float
  own_width: float       # our lane width
  v_ego: float
  lat_active: bool
  lane_changing: bool
  left_outer_prob: float = 1.0   # the next line out on each side (laneLines[0] / [3])
  right_outer_prob: float = 1.0
  lat_accel: float = 0.0         # |model curvature| * v^2


def _ramp(x: float, lo: float, hi: float) -> float:
  return float(np.clip((x - lo) / (hi - lo), 0.0, 1.0))


class EdgeOffset:
  """Target edge offset in metres (+ right).

  Turns on after the road matched for HOLD_ON_S (not in sharp curves). Releases after FAST_RELEASE_S when the road
  clearly no longer matches, after HOLD_OFF_S when it is only unclear; faint own lane lines keep the last decision
  and its cap for up to HOLD_OFF_S. Missing or non-finite inputs, no steering, low speed and lane changes turn it
  off at once."""

  def __init__(self, dt: float):
    self.dt = dt
    self.side = 0          # -1 left, +1 right, 0 off
    self.pending = 0
    self.pending_t = 0.0
    self.release_t = 0.0
    self.lane_change_t = 0.0
    self.cap = 0.0         # largest edge offset that keeps EDGE_MARGIN_M in our lane
    self.scale = 0.0

  def reset(self) -> None:
    self.side = self.pending = 0
    self.pending_t = self.release_t = 0.0
    self.cap = self.scale = 0.0

  @staticmethod
  def inputs_finite(inp: EdgeInputs) -> bool:
    return _finite(inp.left_width, inp.right_width, inp.left_edge, inp.right_edge, inp.left_prob, inp.right_prob,
                   inp.own_width, inp.v_ego, inp.left_outer_prob, inp.right_outer_prob, inp.lat_accel)

  @staticmethod
  def confident(inp: EdgeInputs) -> bool:
    return min(inp.left_prob, inp.right_prob) > MIN_LINE_PROB

  @staticmethod
  def road_side(inp: EdgeInputs) -> int:
    left_lane = inp.left_width > LANE_EXISTS_M and inp.left_outer_prob >= OUTER_LINE_PROB
    right_lane = inp.right_width > LANE_EXISTS_M and inp.right_outer_prob >= OUTER_LINE_PROB
    # no room: the road edge itself is next to our line (a stray outer line alone does not count)
    if left_lane and inp.right_width < NO_ROOM_M and inp.right_edge < NO_ROOM_M:
      return 1
    if right_lane and inp.left_width < NO_ROOM_M and inp.left_edge < NO_ROOM_M:
      return -1
    return 0

  @staticmethod
  def clearly_not(side: int, inp: EdgeInputs) -> bool:
    """The road clearly no longer asks for `side`: room on the side we move to, or no lane on the other side."""
    lane_width, edge_width = (inp.left_width, inp.right_width) if side > 0 else (inp.right_width, inp.left_width)
    return edge_width > LANE_EXISTS_M or lane_width < NO_ROOM_M

  def update(self, edge_m: float, inp: EdgeInputs | None) -> float:
    lane_changing = inp is not None and bool(inp.lane_changing)
    if lane_changing:
      self.lane_change_t = AFTER_LANE_CHANGE_S
    else:
      self.lane_change_t = max(0.0, self.lane_change_t - self.dt)
    if inp is None or not _finite(edge_m) or edge_m <= 0.0 or not inp.lat_active or lane_changing \
       or self.lane_change_t > 0.0 or not self.inputs_finite(inp) or inp.v_ego * 3.6 < SPEED_RAMP_KPH[0]:
      self.reset()
      return 0.0
    speed_scale = _ramp(inp.v_ego * 3.6, *SPEED_RAMP_KPH)

    if not self.confident(inp):
      # faint lines: no new decision; keep the current one (with the cap from trusted frames) for a while
      self.pending, self.pending_t = 0, 0.0
      if self.side == 0:
        return 0.0
      self.release_t += self.dt
      if self.release_t >= HOLD_OFF_S:
        self.reset()
        return 0.0
      return self.side * min(edge_m, self.cap) * self.scale * speed_scale

    raw = self.road_side(inp)
    if self.side == 0:
      if raw != 0 and inp.lat_accel <= CURVE_START_ACCEL:
        self.pending_t = self.pending_t + self.dt if raw == self.pending else self.dt
        self.pending = raw
        if self.pending_t >= HOLD_ON_S:
          self.side, self.pending, self.pending_t, self.release_t = raw, 0, 0.0, 0.0
      else:
        self.pending, self.pending_t = 0, 0.0
    elif raw == self.side:
      self.release_t = 0.0
    else:
      self.release_t += self.dt
      if self.release_t >= (FAST_RELEASE_S if self.clearly_not(self.side, inp) else HOLD_OFF_S):
        self.reset()
    if self.side == 0:
      return 0.0

    self.cap = max(0.0, (inp.own_width - VEHICLE_WIDTH_M) / 2.0 - EDGE_MARGIN_M)
    self.scale = 1.0 - _ramp(inp.own_width, *OWN_WIDTH_MAX_M)
    return self.side * min(edge_m, self.cap) * self.scale * speed_scale


class ViewOffset:
  """The applied view offset: centre + edge target, limited, low-pass filtered and rate limited.

  `applied` is the offset in metres; `slope` (virtual camera move / height) is what the warp uses. The slope is
  rate limited too, so a change of the calibrated height cannot move the view faster than MAX_RATE_M_S."""

  def __init__(self, dt: float):
    self.dt = dt
    self.edge = EdgeOffset(dt)
    self.applied = 0.0
    self.slope = 0.0
    self.height = DEFAULT_HEIGHT_M
    self.target = 0.0
    self.edge_target = 0.0

  @property
  def warp_offset_m(self) -> float:
    """What the warp applies now, as a car offset over the road plane at the last height (+ right)."""
    return -self.slope * self.height

  def reset(self) -> None:
    self.edge.reset()
    self.applied = self.slope = self.target = self.edge_target = 0.0

  def needs_update(self, center_m: float, edge_m: float) -> bool:
    """False when both settings are 0 and the view is back at the stock warp, so the caller can skip the update.
    The road decision is cleared then too: turning a setting back on starts with a fresh hold, never an old side."""
    if center_m != 0.0 or edge_m != 0.0 or self.applied != 0.0 or self.slope != 0.0:
      return True
    self.reset()
    return False

  def update(self, center_m: float, edge_m: float, inp: EdgeInputs | None, calibrated: bool, height_m: float) -> float:
    if not _finite(height_m) or not HEIGHT_RANGE_M[0] <= height_m <= HEIGHT_RANGE_M[1]:
      height_m = DEFAULT_HEIGHT_M
    if not _finite(center_m):
      center_m = 0.0
    if not calibrated:
      self.edge.reset()
      self.edge_target = 0.0
    else:
      self.edge_target = self.edge.update(edge_m, inp)
      if self.edge_target * center_m > 0.0:  # same direction: together they keep the margin too
        allowed = max(0.0, self.edge.cap - abs(center_m))
        self.edge_target = float(np.sign(self.edge_target)) * min(abs(self.edge_target), allowed)
    target = (center_m + self.edge_target) if calibrated else 0.0
    self.target = float(np.clip(target, -MAX_OFFSET_M, MAX_OFFSET_M)) if _finite(target) else 0.0

    shrinking = abs(self.target) < abs(self.applied) or self.target * self.applied < 0.0
    tau = FILTER_TAU_DOWN_S if shrinking else FILTER_TAU_S
    alpha = self.dt / (tau + self.dt)
    step = float(np.clip(alpha * (self.target - self.applied), -MAX_RATE_M_S * self.dt, MAX_RATE_M_S * self.dt))
    self.applied += step
    if self.target == 0.0 and abs(self.applied) < ZERO_SNAP_M:
      self.applied = 0.0

    # the last step to 0 is within the limit too: slope + (-slope) is exactly 0
    max_slope_step = MAX_RATE_M_S * self.dt / height_m
    self.slope += float(np.clip(offset_slope(self.applied, height_m) - self.slope, -max_slope_step, max_slope_step))
    self.height = height_m
    if not _finite(self.applied, self.slope):
      self.reset()
    return self.applied
