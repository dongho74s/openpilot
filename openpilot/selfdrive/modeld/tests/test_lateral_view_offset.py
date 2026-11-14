"""Laneless view offset: the warp change equals a sideways camera move over the road plane, and the edge rule."""
import numpy as np
import pytest

from openpilot.common.transformations.camera import DEVICE_CAMERAS, view_frame_from_device_frame
from openpilot.common.transformations.model import (calib_from_medmodel, calib_from_sbigmodel, get_warp_matrix,
                                                    medmodel_intrinsics, sbigmodel_intrinsics)
from openpilot.common.transformations.orientation import rot_from_euler
from openpilot.selfdrive.modeld import lateral_view_offset as lvo

DT = 0.05


def project(m, p):
  q = m @ p
  return q[:2] / q[2]


@pytest.mark.parametrize("bigmodel_frame", [False, True])
@pytest.mark.parametrize("camera", ["fcam", "ecam"])
def test_shifted_warp_shows_the_road_from_a_camera_moved_sideways(bigmodel_frame, camera):
  rng = np.random.default_rng(1)
  intrinsics_model = sbigmodel_intrinsics if bigmodel_frame else medmodel_intrinsics
  calib_from_model = calib_from_sbigmodel if bigmodel_frame else calib_from_medmodel
  w, h_px = (512, 256)
  for cam_key in [("mici", "os04c10"), ("tici", "ar0231")]:
    intrinsics = getattr(DEVICE_CAMERAS[cam_key], camera).intrinsics
    for _ in range(5):
      rpy = np.array([rng.uniform(-0.02, 0.02), rng.uniform(-0.05, 0.05), rng.uniform(-0.05, 0.05)])
      height = rng.uniform(1.1, 1.6)
      offset_right = rng.uniform(-0.35, 0.35)
      warp = get_warp_matrix(rpy, intrinsics, bigmodel_frame)
      shifted = lvo.shift_transform(warp.astype(np.float32), offset_right, height, bigmodel_frame).astype(np.float64)
      camera_from_calib = intrinsics @ view_frame_from_device_frame @ rot_from_euler(rpy)
      cam_pos = np.array([0.0, -offset_right, 0.0])  # moved left by the offset (calib frame: x ahead, y right, z down)
      horizon = intrinsics_model[1, 2]
      for u in np.linspace(0, w, 7):
        for v in np.linspace(horizon + 10, h_px, 6):
          ray = calib_from_model @ np.array([u, v, 1.0])
          ground = cam_pos + ray * (height / ray[2])
          expected = project(camera_from_calib, ground)
          np.testing.assert_allclose(project(shifted, np.array([u, v, 1.0])), expected, atol=2e-3)


def test_a_positive_offset_makes_the_road_ahead_look_to_the_right_so_the_car_steers_right():
  # Seen from a camera moved left, a point straight ahead of the car is right of the image centre.
  intrinsics = DEVICE_CAMERAS[("mici", "os04c10")].fcam.intrinsics
  warp = get_warp_matrix(np.zeros(3), intrinsics, False)
  shifted = lvo.shift_transform(warp.astype(np.float32), 0.2, 1.4, False).astype(np.float64)
  camera_from_calib = intrinsics @ view_frame_from_device_frame @ rot_from_euler(np.zeros(3))
  target = project(camera_from_calib, np.array([20.0, 0.0, 1.4]))
  # find the model pixel that samples that camera pixel
  model_px = project(np.linalg.inv(shifted), np.array([*target, 1.0]))
  assert model_px[0] > medmodel_intrinsics[0, 2] + 5


def test_zero_offset_keeps_the_exact_transform():
  warp = np.arange(9, dtype=np.float32).reshape(3, 3)
  assert lvo.shift_transform(warp, 0.0, 1.2, False) is warp


@pytest.mark.parametrize("value,limit,neg,expected", [
  (20, 35, True, 0.20), (-20, 35, True, -0.20), (90, 35, True, 0.35), (-90, 35, True, -0.35),
  (-10, 35, False, 0.0), (float("nan"), 35, True, 0.0), (None, 35, True, 0.0), ("x", 35, True, 0.0),
])
def test_settings_are_clamped_to_their_range(value, limit, neg, expected):
  assert lvo.setting_m(value, limit, neg) == pytest.approx(expected)


class Calib:
  def __init__(self, height):
    self.height = height


@pytest.mark.parametrize("height,expected", [([1.4], 1.4), ([], 1.5), ([0.1], 1.5), ([9.0], 1.5), (None, 1.5),
                                             ([float("nan")], 1.5), ([float("inf")], 1.5)])
def test_calibration_height_keeps_the_last_good_value_when_missing_or_implausible(height, expected):
  assert lvo.calibration_height(Calib(height), 1.5) == pytest.approx(expected)


def two_lane(**changes):
  values = dict(left_width=3.0, right_width=0.6, left_edge=6.0, right_edge=0.6, left_prob=0.9, right_prob=0.9,
                own_width=2.9, v_ego=20.0, lat_active=True, lane_changing=False)
  values.update(changes)
  return lvo.EdgeInputs(**values)


def run(edge, inp, seconds, edge_m=0.3):
  out = 0.0
  for _ in range(int(round(seconds / DT))):
    out = edge.update(edge_m, inp)
  return out


def test_edge_offset_moves_toward_the_road_edge_after_the_hold_time():
  edge = lvo.EdgeOffset(DT)
  assert run(edge, two_lane(), 1.4) == 0.0
  assert run(edge, two_lane(), 0.2) == pytest.approx(0.3)
  mirrored = lvo.EdgeOffset(DT)
  assert run(mirrored, two_lane(left_width=0.5, left_edge=0.5, right_width=3.2, right_edge=6.0), 2.0) == pytest.approx(-0.3)


@pytest.mark.parametrize("changes", [
  {"right_width": 3.0},                       # lanes on both sides
  {"left_width": 1.0},                        # no room on either side
  {"left_width": 2.1},                        # left lane not clearly there
  {"right_width": 2.05},                      # right side not clearly without room
  {"left_prob": 0.4},                         # our lines not visible
  {"own_width": float("nan")},
  {"right_edge": 3.0},                        # a close but stray outer line, the road edge is far
])
def test_edge_offset_stays_off_without_a_clear_asymmetric_road(changes):
  assert run(lvo.EdgeOffset(DT), two_lane(**changes), 5.0) == 0.0


@pytest.mark.parametrize("changes", [{"lat_active": False}, {"lane_changing": True}, {"v_ego": 6.0}])
def test_edge_offset_is_off_immediately_without_steering_during_a_lane_change_or_slowly(changes):
  edge = lvo.EdgeOffset(DT)
  assert run(edge, two_lane(), 2.0) == pytest.approx(0.3)
  assert edge.update(0.3, two_lane(**changes)) == 0.0


def test_edge_offset_waits_after_a_lane_change_and_needs_the_hold_again():
  edge = lvo.EdgeOffset(DT)
  run(edge, two_lane(), 2.0)
  run(edge, two_lane(lane_changing=True), 1.0)
  assert run(edge, two_lane(), 1.9) == 0.0        # 2 s after the lane change
  assert run(edge, two_lane(), 1.4) == 0.0        # then the 1.5 s hold
  assert run(edge, two_lane(), 0.3) == pytest.approx(0.3)


def test_a_short_gap_in_the_road_edge_does_not_release_but_a_long_one_does():
  edge = lvo.EdgeOffset(DT)
  run(edge, two_lane(), 2.0)
  assert run(edge, two_lane(right_width=3.0), 2.5) == pytest.approx(0.3)
  assert run(edge, two_lane(), 0.1) == pytest.approx(0.3)
  assert run(edge, two_lane(right_width=3.0), 3.1) == 0.0


def test_edge_offset_keeps_a_margin_in_narrow_lanes_and_scales_with_speed():
  # (lane - 2.0 m car) / 2 - 0.15 m margin
  assert run(lvo.EdgeOffset(DT), two_lane(own_width=2.3), 2.0) == 0.0
  assert run(lvo.EdgeOffset(DT), two_lane(own_width=2.6), 2.0) == pytest.approx(0.15)
  assert run(lvo.EdgeOffset(DT), two_lane(own_width=2.74), 2.0) == pytest.approx(0.22)
  assert run(lvo.EdgeOffset(DT), two_lane(own_width=4.8), 2.0) == 0.0
  assert run(lvo.EdgeOffset(DT), two_lane(v_ego=30 / 3.6), 2.0) == pytest.approx(0.15)
  assert run(lvo.EdgeOffset(DT), two_lane(), 2.0, edge_m=0.0) == 0.0


def test_view_offset_is_limited_filtered_and_rate_limited():
  view = lvo.ViewOffset(DT)
  steps = [view.update(0.35, 0.35, two_lane(), True, 1.4) for _ in range(int(20 / DT))]
  assert max(abs(b - a) for a, b in zip([0.0, *steps], steps, strict=False)) <= lvo.MAX_RATE_M_S * DT + 1e-9
  assert steps[-1] == pytest.approx(lvo.MAX_OFFSET_M, abs=1e-3)   # 0.35 + 0.35 is limited to 0.35
  assert abs(steps[int(1 / DT) - 1]) <= lvo.MAX_RATE_M_S + 1e-9   # at most 10 cm in the first second


def test_view_offset_returns_to_zero_without_calibration_or_inputs():
  view = lvo.ViewOffset(DT)
  for _ in range(int(20 / DT)):
    view.update(-0.2, 0.0, two_lane(), True, 1.4)
  assert view.applied == pytest.approx(-0.2, abs=1e-3)
  for _ in range(int(20 / DT)):
    view.update(-0.2, 0.3, None, False, 1.4)
  assert view.applied == 0.0 and view.edge.side == 0


def test_centre_and_edge_offsets_add_up():
  view = lvo.ViewOffset(DT)
  for _ in range(int(20 / DT)):
    view.update(-0.1, 0.25, two_lane(), True, 1.4)
  assert view.applied == pytest.approx(0.15, abs=1e-3)


def test_the_side_flips_only_through_off():
  edge = lvo.EdgeOffset(DT)
  run(edge, two_lane(), 2.0)
  flipped = two_lane(left_width=0.5, left_edge=0.5, right_width=3.2, right_edge=6.0)
  assert run(edge, flipped, 2.9) == pytest.approx(0.3)   # still right until released after 3 s
  assert run(edge, flipped, 0.2) == 0.0
  assert run(edge, flipped, 1.5) == pytest.approx(-0.3)


def test_centre_in_the_same_direction_uses_up_the_edge_margin():
  view = lvo.ViewOffset(DT)
  for _ in range(int(30 / DT)):
    view.update(0.15, 0.30, two_lane(own_width=2.74), True, 1.4)   # margin cap 0.22: edge adds only 0.07
  assert view.applied == pytest.approx(0.22, abs=2e-3)
  view = lvo.ViewOffset(DT)
  for _ in range(int(30 / DT)):
    view.update(0.30, 0.30, two_lane(own_width=2.74), True, 1.4)   # the centre alone is the user's choice
  assert view.applied == pytest.approx(0.30, abs=2e-3)


@pytest.mark.parametrize("cam_key", [("mici", "os04c10"), ("tici", "ar0231")])
@pytest.mark.parametrize("bigmodel_frame,camera", [(False, "fcam"), (True, "ecam"), (False, "ecam")])
def test_the_largest_offset_still_samples_inside_the_camera_image(cam_key, bigmodel_frame, camera):
  cam = DEVICE_CAMERAS[cam_key]
  sensor = getattr(cam, camera)
  warp = get_warp_matrix(np.zeros(3), sensor.intrinsics, bigmodel_frame)
  for offset in (-lvo.MAX_OFFSET_M, lvo.MAX_OFFSET_M):
    shifted = lvo.shift_transform(warp.astype(np.float32), offset, 1.1, bigmodel_frame).astype(np.float64)
    for u in (0, 512):
      for v in (0, 256):
        x, y = project(shifted, np.array([u, v, 1.0]))
        _, y0 = project(warp, np.array([u, v, 1.0]))
        # the shift only moves samples sideways: rows stay where the stock warp has them
        assert -1 <= x <= sensor.width + 1 and y == pytest.approx(y0, abs=1e-3), (offset, u, v, x, y)


# --- review fixes: bad inputs, stale inputs, height changes, return time ---------------------------------------------
@pytest.mark.parametrize("changes", [{"left_prob": 0.4}, {"left_prob": 0.5}, {"right_prob": float("nan")},
                                     {"left_width": float("nan")}, {"own_width": float("nan")}, {"right_edge": float("inf")}])
def test_unreliable_inputs_turn_the_edge_offset_off_at_once(changes):
  edge = lvo.EdgeOffset(DT)
  assert run(edge, two_lane(), 2.0) == pytest.approx(0.3)
  assert edge.update(0.3, two_lane(**changes)) == 0.0 and edge.side == 0


def test_missing_inputs_turn_the_edge_offset_off_and_need_the_hold_again():
  edge = lvo.EdgeOffset(DT)
  run(edge, two_lane(), 2.0)
  assert edge.update(0.3, None) == 0.0
  assert run(edge, two_lane(), 1.4) == 0.0


@pytest.mark.parametrize("changes", [{"left_prob": float("nan")}, {"own_width": float("nan")}])
def test_nan_inputs_never_reach_the_view_offset(changes):
  view = lvo.ViewOffset(DT)
  for _ in range(int(5 / DT)):
    view.update(0.1, 0.3, two_lane(**changes), True, 1.4)
  assert np.isfinite(view.applied) and np.isfinite(view.slope) and view.edge.side == 0
  for _ in range(int(30 / DT)):
    view.update(float("nan"), float("nan"), two_lane(), True, float("nan"))
  assert view.applied == 0.0 and view.slope == 0.0


def test_a_height_change_moves_the_view_no_faster_than_the_rate_limit():
  view = lvo.ViewOffset(DT)
  for _ in range(int(30 / DT)):
    view.update(0.35, 0.0, None, True, 1.6)
  assert view.slope == pytest.approx(-0.35 / 1.6, rel=1e-3)
  previous = view.slope
  for _ in range(int(3 / DT)):
    view.update(0.35, 0.0, None, True, 1.22)
    # virtual camera position change over the road at the new height
    assert abs(view.slope - previous) * 1.22 <= lvo.MAX_RATE_M_S * DT + 1e-9
    previous = view.slope
  assert view.slope == pytest.approx(-0.35 / 1.22, rel=1e-3)


@pytest.mark.parametrize("start,max_s", [(0.35, 11.0), (0.24, 10.0)])
def test_turning_it_off_returns_to_the_stock_warp_in_about_ten_seconds(start, max_s):
  view = lvo.ViewOffset(DT)
  for _ in range(int(40 / DT)):
    view.update(start, 0.0, None, True, 1.4)
  frames = 0
  while view.applied != 0.0 or view.slope != 0.0:
    view.update(0.0, 0.0, None, True, 1.4)
    frames += 1
    assert frames < 400
  assert frames * DT <= max_s


def test_without_calibration_the_view_returns_to_stock():
  view = lvo.ViewOffset(DT)
  for _ in range(int(20 / DT)):
    view.update(0.2, 0.0, None, True, 1.4)
  for _ in range(int(15 / DT)):
    view.update(0.2, 0.3, two_lane(), False, 1.4)
  assert view.applied == 0.0 and view.slope == 0.0


def test_the_return_to_stock_including_the_last_step_keeps_the_rate_limit_even_with_a_height_change():
  for height_after in (1.22, 1.40, 1.60):
    view = lvo.ViewOffset(DT)
    for _ in range(int(40 / DT)):
      view.update(0.35, 0.0, None, True, 1.22)
    height, frames = 1.22, 0
    while view.applied != 0.0 or view.slope != 0.0:
      if view.applied < 0.006:
        height = height_after   # the height changes right at the end of the return
      previous = view.slope
      view.update(0.0, 0.0, None, True, height)
      assert abs(view.slope - previous) * height <= lvo.MAX_RATE_M_S * DT + 1e-12
      frames += 1
      assert frames < 600
    assert view.warp_offset_m == 0.0


def test_the_logged_warp_offset_is_what_the_warp_applies():
  view = lvo.ViewOffset(DT)
  for _ in range(int(40 / DT)):
    view.update(0.35, 0.0, None, True, 1.22)
  view.update(0.35, 0.0, None, True, 1.60)
  assert view.applied == pytest.approx(0.35, abs=1e-3)
  assert view.warp_offset_m == pytest.approx(-view.slope * 1.60) and view.warp_offset_m > 0.4   # not yet at 0.35
