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


def test_an_unclear_road_waits_3_s_before_releasing():
  edge = lvo.EdgeOffset(DT)
  run(edge, two_lane(), 2.0)
  grey = two_lane(right_width=2.1, right_edge=2.1)      # neither clearly room nor clearly no room on the right
  assert run(edge, grey, 2.5) == pytest.approx(0.3)
  assert run(edge, two_lane(), 0.1) == pytest.approx(0.3)
  assert run(edge, grey, 3.1) == 0.0


@pytest.mark.parametrize("changes", [
  {"right_width": 3.0, "right_edge": 6.0},     # a lane appeared on the side we move to
  {"left_width": 1.4, "left_edge": 1.3},       # the lane on the other side is gone (e.g. a one-lane ramp)
])
def test_a_road_that_clearly_no_longer_matches_releases_in_half_a_second(changes):
  edge = lvo.EdgeOffset(DT)
  run(edge, two_lane(), 2.0)
  assert run(edge, two_lane(**changes), 0.4) == pytest.approx(0.3)
  assert run(edge, two_lane(**changes), 0.15) == 0.0


def test_a_sharp_curve_does_not_start_it_but_does_not_hold_a_stale_decision_either():
  assert run(lvo.EdgeOffset(DT), two_lane(lat_accel=2.0), 5.0) == 0.0
  edge = lvo.EdgeOffset(DT)
  run(edge, two_lane(), 2.0)
  assert run(edge, two_lane(lat_accel=2.5), 2.0) == pytest.approx(0.3)                         # still the same road
  assert run(edge, two_lane(lat_accel=2.5, left_width=1.4, left_edge=1.3), 0.6) == 0.0   # one-lane ramp in a curve


def test_outer_line_requirement_when_enabled(monkeypatch):
  monkeypatch.setattr(lvo, "OUTER_LINE_PROB", 0.3)
  assert run(lvo.EdgeOffset(DT), two_lane(left_outer_prob=0.1), 3.0) == 0.0
  assert run(lvo.EdgeOffset(DT), two_lane(left_outer_prob=0.6), 3.0) == pytest.approx(0.3)


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
  steps = [view.update(0.6, 0.5, two_lane(), True, 1.4) for _ in range(int(20 / DT))]
  assert max(abs(b - a) for a, b in zip([0.0, *steps], steps, strict=False)) <= lvo.MAX_RATE_M_S * DT + 1e-9
  assert steps[-1] == pytest.approx(lvo.MAX_OFFSET_M, abs=1e-3)   # 0.6 + edge is limited to 0.5
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
  assert run(edge, flipped, 0.4) == pytest.approx(0.3)   # still right until released (clear: 0.5 s)
  assert run(edge, flipped, 0.15) == 0.0
  assert run(edge, flipped, 1.4) == 0.0                  # then a new 1.5 s hold
  assert run(edge, flipped, 0.2) == pytest.approx(-0.3)


def test_centre_in_the_same_direction_uses_up_the_edge_margin():
  view = lvo.ViewOffset(DT)
  for _ in range(int(30 / DT)):
    view.update(0.15, 0.30, two_lane(own_width=2.74), True, 1.4)   # margin cap 0.22: edge adds only 0.07
  assert view.applied == pytest.approx(0.22, abs=2e-3)
  view = lvo.ViewOffset(DT)
  for _ in range(int(30 / DT)):
    view.update(0.30, 0.30, two_lane(own_width=2.74), True, 1.4)   # the centre alone is the user's choice
  assert view.applied == pytest.approx(0.30, abs=2e-3)


def corner_samples(sensor, bigmodel_frame, offset, height=1.1):
  warp = get_warp_matrix(np.zeros(3), sensor.intrinsics, bigmodel_frame)
  shifted = lvo.shift_transform(warp.astype(np.float32), offset, height, bigmodel_frame).astype(np.float64)
  for u in (0, 512):
    for v in (0, 256):
      x, y = project(shifted, np.array([u, v, 1.0]))
      _, y0 = project(warp, np.array([u, v, 1.0]))
      yield x, y, y0


@pytest.mark.parametrize("cam_key,inside_m", [(("mici", "os04c10"), lvo.MAX_OFFSET_M), (("tici", "ar0231"), 0.35)])
@pytest.mark.parametrize("bigmodel_frame,camera", [(False, "fcam"), (True, "ecam"), (False, "ecam")])
def test_the_offset_samples_inside_the_camera_image(cam_key, inside_m, bigmodel_frame, camera):
  """comma 4 (os04c10) stays inside up to the 0.5 m limit; comma 3 (ar0231) up to 0.35 m (user chose 0.5 for all)."""
  sensor = getattr(DEVICE_CAMERAS[cam_key], camera)
  for offset in (-inside_m, inside_m):
    for x, y, y0 in corner_samples(sensor, bigmodel_frame, offset):
      # the shift only moves samples sideways: rows stay where the stock warp has them
      assert -1 <= x <= sensor.width + 1 and y == pytest.approx(y0, abs=1e-3), (offset, x, y)


@pytest.mark.parametrize("bigmodel_frame,camera", [(False, "fcam"), (True, "ecam"), (False, "ecam")])
def test_comma_3_at_the_limit_only_samples_a_little_past_the_bottom_corners(bigmodel_frame, camera):
  sensor = getattr(DEVICE_CAMERAS[("tici", "ar0231")], camera)
  for offset in (-lvo.MAX_OFFSET_M, lvo.MAX_OFFSET_M):
    for x, y, y0 in corner_samples(sensor, bigmodel_frame, offset):
      assert -80 <= x <= sensor.width + 80 and y == pytest.approx(y0, abs=1e-3), (offset, x, y)


# --- review fixes: bad inputs, stale inputs, height changes, return time ---------------------------------------------
@pytest.mark.parametrize("changes", [{"right_prob": float("nan")}, {"left_width": float("nan")}, {"own_width": float("nan")},
                                     {"right_edge": float("inf")}, {"left_outer_prob": float("nan")}, {"lat_accel": float("nan")}])
def test_non_finite_inputs_turn_the_edge_offset_off_at_once(changes):
  edge = lvo.EdgeOffset(DT)
  assert run(edge, two_lane(), 2.0) == pytest.approx(0.3)
  assert edge.update(0.3, two_lane(**changes)) == 0.0 and edge.side == 0


@pytest.mark.parametrize("prob", [0.4, 0.5])
def test_faint_lines_keep_the_last_decision_and_its_cap_for_up_to_3_s(prob):
  edge = lvo.EdgeOffset(DT)
  run(edge, two_lane(own_width=2.74), 2.0)
  faint = two_lane(left_prob=prob, own_width=4.0, left_width=0.5, right_width=3.0)   # untrusted values are not used
  assert run(edge, faint, 2.9) == pytest.approx(0.22)
  assert run(edge, faint, 0.2) == 0.0
  assert run(lvo.EdgeOffset(DT), faint, 5.0) == 0.0                                   # and never start it


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


def test_turning_both_settings_off_clears_the_road_decision_so_turning_it_on_again_needs_a_fresh_hold():
  # centre -30 cm and edge +30 cm cancel: the view is at the stock warp while the edge decision is on
  view = lvo.ViewOffset(DT)
  for _ in range(int(10 / DT)):
    assert view.needs_update(-0.30, 0.30)
    view.update(-0.30, 0.30, two_lane(own_width=3.5), True, 1.4)
  assert view.edge.side == 1 and view.applied == 0.0 and view.slope == 0.0
  for _ in range(int(20 / DT)):                       # both settings off: modeld skips the update
    assert not view.needs_update(0.0, 0.0)
  assert view.edge.side == 0 and view.edge.cap == 0.0 and view.edge_target == 0.0
  faint = two_lane(left_prob=0.4, right_prob=0.4, right_width=3.0, right_edge=6.0, own_width=3.5)
  for _ in range(int(5 / DT)):                        # edge back on with faint lines and lanes on both sides
    assert view.needs_update(0.0, 0.30)
    view.update(0.0, 0.30, faint, True, 1.4)
    assert view.edge_target == 0.0 and view.applied == 0.0 and view.slope == 0.0
  for i in range(int(2 / DT)):                        # clear road again: on only after a new 1.5 s hold
    view.update(0.0, 0.30, two_lane(own_width=3.5), True, 1.4)
    assert (view.edge_target > 0.0) == (i + 1 >= int(round(lvo.HOLD_ON_S / DT)))


def test_with_a_view_left_to_return_the_update_still_runs():
  view = lvo.ViewOffset(DT)
  for _ in range(int(10 / DT)):
    view.update(0.20, 0.0, None, True, 1.4)
  assert view.needs_update(0.0, 0.0) and view.applied > 0.0   # the filtered return to stock is kept


# --- LanelessLineGap: one seen line, the road edge on the other side (user request 2026-11-16) -----------------------
def one_sided(**changes):
  # left line seen 1.0 m left of the plan, right line faint, road edge 2.1 m right of the plan (K8 two-lane road)
  values = dict(left_line=-1.0, right_line=1.53, left_edge=-3.6, right_edge=2.1, plan=0.0, left_prob=0.8, right_prob=0.2,
                left_outer_prob=0.25, right_outer_prob=0.0, v_ego=10.0, lat_active=True, lane_changing=False)
  values.update(changes)
  return lvo.LineGapInputs(**values)


def run_gap(gap, inp, seconds, gap_m=0.3):
  out = 0.0
  for _ in range(int(round(seconds / DT))):
    out = gap.update(gap_m, inp)
  return out


def test_line_gap_moves_away_from_the_seen_line_by_what_the_gap_misses_after_the_hold():
  gap = lvo.LineGapOffset(DT)
  assert run_gap(gap, one_sided(), 1.4) == 0.0
  # car side is 1.0 - 1.0 (half of 2.0 m) = 0 from the line; a 30 cm gap needs 0.3 m to the right
  assert run_gap(gap, one_sided(), 0.2) == pytest.approx(0.3)
  assert gap.side == 1 and gap.need == pytest.approx(0.3)
  assert run_gap(lvo.LineGapOffset(DT), one_sided(plan=-0.1), 2.0) == pytest.approx(0.4)   # plan nearer the line
  mirrored = one_sided(left_line=-1.53, right_line=1.0, left_edge=-2.1, right_edge=3.6, left_prob=0.2, right_prob=0.8,
                       left_outer_prob=0.0, right_outer_prob=0.25)
  assert run_gap(lvo.LineGapOffset(DT), mirrored, 2.0) == pytest.approx(-0.3)


def test_line_gap_never_moves_toward_the_line_and_keeps_room_to_the_road_edge():
  far = lvo.LineGapOffset(DT)
  assert run_gap(far, one_sided(left_line=-1.6), 2.0) == 0.0 and far.side == 1   # already 60 cm from the line
  near_edge = lvo.LineGapOffset(DT)
  # room: 1.5 - 0 - 1.0 (half width) - 0.3 (kept to the edge) = 0.2 m
  assert run_gap(near_edge, one_sided(right_edge=1.5), 2.0, gap_m=0.5) == pytest.approx(0.2)
  assert near_edge.need == pytest.approx(0.5) and near_edge.shift == pytest.approx(0.2)
  assert run_gap(lvo.LineGapOffset(DT), one_sided(right_edge=1.2), 2.0) == 0.0   # no room at all


@pytest.mark.parametrize("changes", [{"right_prob": 0.8}, {"left_prob": 0.4}, {"right_outer_prob": 0.4},
                                     {"right_edge": 3.7}, {"plan": -1.2}, {"plan": 2.2}])
def test_line_gap_needs_one_seen_line_and_the_road_edge_close_on_the_other_side(changes):
  assert run_gap(lvo.LineGapOffset(DT), one_sided(**changes), 3.0) == 0.0


@pytest.mark.parametrize("changes", [{"lat_active": False}, {"lane_changing": True}, {"v_ego": 19.0 / 3.6},
                                     {"plan": float("nan")}, {"right_edge": float("inf")}])
def test_line_gap_is_off_at_once_without_steering_in_a_lane_change_slowly_or_with_bad_inputs(changes):
  gap = lvo.LineGapOffset(DT)
  run_gap(gap, one_sided(), 2.0)
  assert run_gap(gap, one_sided(**changes), DT) == 0.0 and gap.side == 0
  assert run_gap(lvo.LineGapOffset(DT), one_sided(), 2.0, gap_m=0.0) == 0.0
  assert lvo.LineGapOffset(DT).update(0.3, None) == 0.0


def test_line_gap_scales_with_speed_from_20_to_30_kph():
  assert run_gap(lvo.LineGapOffset(DT), one_sided(v_ego=25.0 / 3.6), 2.0) == pytest.approx(0.15)
  assert run_gap(lvo.LineGapOffset(DT), one_sided(v_ego=30.0 / 3.6), 2.0) == pytest.approx(0.3)


def test_line_gap_releases_fast_when_clearly_not_and_holds_its_shift_when_unclear():
  gap = lvo.LineGapOffset(DT)
  run_gap(gap, one_sided(), 2.0)
  assert run_gap(gap, one_sided(right_prob=0.8), 0.4) == pytest.approx(0.3)   # both lines now: EdgeOffset's road
  assert run_gap(gap, one_sided(right_prob=0.8), 0.15) == 0.0
  gap = lvo.LineGapOffset(DT)
  run_gap(gap, one_sided(), 2.0)
  assert run_gap(gap, one_sided(left_prob=0.4, plan=-0.5), 2.9) == pytest.approx(0.3)   # unclear: the last shift
  assert run_gap(gap, one_sided(left_prob=0.4), 0.15) == 0.0


def test_line_gap_does_not_start_in_a_sharp_curve():
  assert run_gap(lvo.LineGapOffset(DT), one_sided(lat_accel=2.0), 3.0) == 0.0


def test_line_gap_replaces_the_centre_offset_and_is_limited_to_the_maximum():
  view = lvo.ViewOffset(DT)
  for _ in range(int(30 / DT)):
    view.update(-0.2, 0.0, None, True, 1.4, 0.3, one_sided())
  assert view.gap.side == 1 and view.applied == pytest.approx(0.3, abs=2e-3)   # not 0.3 - 0.2
  view = lvo.ViewOffset(DT)
  for _ in range(int(30 / DT)):
    view.update(0.0, 0.0, None, True, 1.4, 0.8, one_sided(right_edge=3.0))       # need 0.8, room 1.7
  assert view.applied == pytest.approx(lvo.MAX_OFFSET_M, abs=2e-3)


def test_an_active_edge_offset_keeps_the_line_gap_off_and_it_needs_its_own_hold_afterwards():
  view = lvo.ViewOffset(DT)
  for _ in range(int(3 / DT)):
    view.update(0.0, 0.3, two_lane(), True, 1.4, 0.3, one_sided())
  assert view.edge.side == 1 and view.gap.side == 0 and view.gap_target == 0.0
  for _ in range(int(1.0 / DT)):   # both lines turn faint: the edge holds (3 s), the gap stays off meanwhile
    view.update(0.0, 0.3, two_lane(right_prob=0.2), True, 1.4, 0.3, one_sided())
  assert view.edge.side == 1 and view.gap.side == 0


def test_line_gap_alone_keeps_the_update_running_and_turning_it_off_clears_it():
  view = lvo.ViewOffset(DT)
  assert view.needs_update(0.0, 0.0, 0.3)
  for _ in range(int(2 / DT)):
    view.update(0.0, 0.0, None, True, 1.4, 0.3, one_sided())
  assert view.gap.side == 1
  for _ in range(int(20 / DT)):
    if view.needs_update(0.0, 0.0, 0.0):
      view.update(0.0, 0.0, None, True, 1.4, 0.0, one_sided())
  assert view.applied == 0.0 and view.gap.side == 0


def test_line_gap_inputs_read_lines_and_edges_by_distance_and_the_plan_by_its_own_x():
  from types import SimpleNamespace as NS
  xs = [0.0, 10.0, 19.2, 40.0]
  def line(y19):
    return NS(x=xs, y=[y19 - 0.1, y19 - 0.05, y19, y19 + 0.2])
  md = NS(laneLines=[line(-4.0), line(-1.0), line(1.5), line(4.5)], roadEdges=[line(-3.5), line(2.0)],
          position=NS(x=[0.0, 9.6, 28.8], y=[0.0, 0.1, 0.4]), laneLineProbs=[0.3, 0.8, 0.2, 0.0])
  inp = lvo.line_gap_inputs(md, 10.0, True, False, 0.5)
  assert (inp.left_line, inp.right_line, inp.left_edge, inp.right_edge) == pytest.approx((-1.0, 1.5, -3.5, 2.0))
  assert inp.plan == pytest.approx(0.25)   # halfway between 9.6 m and 28.8 m
  assert (inp.left_prob, inp.right_prob, inp.left_outer_prob, inp.right_outer_prob) == pytest.approx((0.8, 0.2, 0.3, 0.0))


def test_line_gap_setting_range():
  assert lvo.setting_m(80, lvo.LINE_GAP_LIMIT_CM, False) == pytest.approx(0.8)
  assert lvo.setting_m(120, lvo.LINE_GAP_LIMIT_CM, False) == pytest.approx(0.8)
  assert lvo.setting_m(-10, lvo.LINE_GAP_LIMIT_CM, False) == 0.0
  assert lvo.setting_m(None, lvo.LINE_GAP_LIMIT_CM, False) == 0.0   # Params.get of a corrupt value


# --- review fixes (Codex R1-R5) ----------------------------------------------------------------------------------------
def test_a_held_shift_is_cut_to_the_room_the_road_edge_leaves_now():
  gap = lvo.LineGapOffset(DT)
  run_gap(gap, one_sided(), 2.0)
  # unclear (seen line faint) and the edge closer: room 1.35 - 0 - 1.0 - 0.3 = 0.05 m
  assert run_gap(gap, one_sided(left_prob=0.4, right_edge=1.35), 1.0) == pytest.approx(0.05)
  assert run_gap(gap, one_sided(left_prob=0.4, right_edge=1.1), DT) == 0.0   # no room: 0, still held


@pytest.mark.parametrize("changes", [{"right_edge": -0.1}, {"plan": -1.1}])
def test_a_plan_no_longer_between_the_seen_line_and_the_road_edge_releases_fast(changes):
  gap = lvo.LineGapOffset(DT)
  run_gap(gap, one_sided(), 2.0)
  run_gap(gap, one_sided(**changes), 0.5 + 2 * DT)
  assert gap.side == 0


def test_no_line_gap_when_another_lane_fits_past_the_faint_line():
  # our lane 2.2 m, then 2.3 m to the curb: two lanes with a faint divider, not our lane ending at the edge
  two_lanes = one_sided(left_line=-0.8, right_line=1.4, right_edge=3.7)
  assert run_gap(lvo.LineGapOffset(DT), two_lanes, 3.0) == 0.0
  mirrored = one_sided(left_line=-3.7 + 2.3, right_line=0.8, left_edge=-3.7, right_edge=4.0, left_prob=0.2, right_prob=0.8,
                       left_outer_prob=0.0, right_outer_prob=0.25)
  assert run_gap(lvo.LineGapOffset(DT), mirrored, 3.0) == 0.0
  gap = lvo.LineGapOffset(DT)
  run_gap(gap, one_sided(), 2.0)
  run_gap(gap, one_sided(right_line=1.4, right_edge=3.7), 0.5 + 2 * DT)   # a lane fits now: clearly not
  assert gap.side == 0


def test_a_plan_that_does_not_reach_the_lookahead_turns_the_line_gap_off():
  from types import SimpleNamespace as NS
  xs = [0.0, 10.0, 19.2, 40.0]
  def line(y19):
    return NS(x=xs, y=[y19] * 4)
  md = NS(laneLines=[line(-4.0), line(-1.0), line(1.53), line(4.5)], roadEdges=[line(-3.6), line(2.1)],
          position=NS(x=[0.0, 5.0, 10.0], y=[0.0, 0.0, 0.17]), laneLineProbs=[0.25, 0.8, 0.2, 0.0])
  inp = lvo.line_gap_inputs(md, 25 / 3.6, True, False, 0.2)
  assert np.isnan(inp.plan)
  gap = lvo.LineGapOffset(DT)
  run_gap(gap, one_sided(), 2.0)
  assert run_gap(gap, inp, DT) == 0.0 and gap.side == 0
  md.position = NS(x=[0.0, 30.0, 20.0], y=[0.0, 0.1, 0.2])   # not increasing
  assert np.isnan(lvo.line_gap_inputs(md, 10.0, True, False, 0.0).plan)


def test_a_stored_integer_too_large_for_a_float_is_off():
  assert lvo.setting_m(int("9" * 400), lvo.LINE_GAP_LIMIT_CM, False) == 0.0
