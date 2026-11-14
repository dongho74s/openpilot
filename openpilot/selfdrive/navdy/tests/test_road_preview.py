"""Navdy road drawing kept from the camera-view era: road preview, input freshness, short plans and lanes.

Moved from test_camera_view.py when the v180-v182 road drawing came back (2026-11-15); none depends on the projection.
"""
from types import SimpleNamespace as NS

from openpilot.selfdrive.navdy.display import HIDDEN_BAND, navdy_model_geometry, navdy_model_line, navdy_vehicle_view
from openpilot.selfdrive.navdy.tests.test_display import model
from openpilot.selfdrive.navdy.vehicles import vehicle_lane, vehicle_yaw


def test_stopped_short_path_still_draws_lanes_and_vehicles():
  stopped = model()
  stopped.position = NS(x=[0.0, 1.0, 2.0], y=[0.0, 0.0, 0.0])
  geometry = navdy_model_geometry(stopped)
  assert geometry['navLaneLeft'] and geometry['navLaneRight']
  assert len(geometry['navPathLeft']) == 6 and geometry['navPathLeft'][1] == 96.0   # a short band at the bottom row
  one_sample = model()
  one_sample.position = NS(x=[0.0, 120.0], y=[0.0, 0.0])    # a single drawable sample: no band, lanes still drawn
  geometry = navdy_model_geometry(one_sample)
  assert geometry['navLaneLeft'] and geometry['navPathLeft'] == geometry['navPathRight'] == list(HIDDEN_BAND)


def test_every_radar_point_of_a_dense_run_is_drawn():
  # A truck's side and a dense queue look alike to the radar (Codex review 2026-11-13): the overlay merges or drops
  # nothing (sprite_overlap later drops only unconfirmed points inside a camera truck's body).
  from openpilot.selfdrive.navdy.tests.test_display import car, payload as make_payload
  from openpilot.selfdrive.navdy.tests.test_vehicles import data, point
  from openpilot.selfdrive.navdy.vehicles import VehicleOverlay
  radar = data([point(i + 1, 10.0 + 3.0 * i, -3.2, 0.0) for i in range(6)])   # radar left-positive: right lane
  for speed in (0.0, 20.0):
    overlay, m = VehicleOverlay(), model()
    for t in (100.0, 100.2):
      result = make_payload(model=m, car_state=car(vEgo=speed, vEgoCluster=0))
      overlay.update(result, m, radar, timestamp=t, now=t)
    radar_entries = [v for v in result['navVehicles'] if v['source'] == 'radar']
    assert sorted(v['trackId'] for v in radar_entries) == [1, 2, 3, 4, 5, 6]
    assert all('kind' not in v for v in radar_entries)


def test_lane_is_read_at_the_camera_distance_like_the_drawing():
  m = model()
  for line in m.laneLines:
    line.y = [y + 0.2 * x for x, y in zip(line.x, line.y)]
  # dRel 20 m is 21.52 m from the camera: lanes there are 0.304 m further right than at 20 m
  assert vehicle_lane(m, 20.0, 6.004) == 'center'


def preview_payload(**kwargs):
  from openpilot.selfdrive.navdy.tests.test_display import lead, payload, selfdrive
  return payload(model=model(), radar_state=NS(leadOne=lead(), leadTwo=lead(status=False)), **kwargs)


def test_disengaged_onroad_sends_the_road_without_the_path():
  from openpilot.selfdrive.navdy.tests.test_display import selfdrive
  result = preview_payload(selfdrive_state=selfdrive(enabled=False, active=False, state='disabled'), onroad=True)
  assert result['active'] is False and result['roadPreview'] is True
  assert result['navLaneLeft'] and result['navLaneRight'] and len(result['navVehicles']) == 1
  assert result['navPathLeft'] == result['navPathRight'] == list(HIDDEN_BAND)


def test_engaged_keeps_the_path_and_offroad_sends_no_road():
  from openpilot.selfdrive.navdy.tests.test_display import selfdrive
  engaged = preview_payload(onroad=True)
  assert engaged['active'] and 'roadPreview' not in engaged and engaged['navPathLeft'] != list(HIDDEN_BAND)
  offroad = preview_payload(selfdrive_state=selfdrive(enabled=False, active=False, state='disabled'), onroad=False)
  assert 'roadPreview' not in offroad and not offroad['navVehicles'] and 'navLaneLeft' not in offroad


def test_raw_tracks_and_camera_objects_also_show_in_the_preview():
  from openpilot.selfdrive.navdy.tests.test_display import payload, selfdrive
  from openpilot.selfdrive.navdy.tests.test_vehicles import data, point
  from openpilot.selfdrive.navdy.vehicles import VehicleOverlay, annotate_camera
  m, overlay = model(), VehicleOverlay()
  for timestamp in (100.0, 100.2):
    result = payload(model=m, selfdrive_state=selfdrive(enabled=False, active=False, state='disabled'), onroad=True)
    overlay.update(result, m, data([point(7, 30.0, -3.6)]), timestamp=timestamp, now=timestamp)
  annotate_camera(result, m, [{'id': 5, 'kind': 'ped', 'distance': 20.0, 'lateral': 3.0}], None)
  assert {v['trackId'] for v in result['navVehicles']} == {7, 900005}


def test_no_preview_on_stale_car_or_selfdrive_state():
  from openpilot.selfdrive.navdy.tests.test_display import selfdrive
  stale_car = preview_payload(selfdrive_state=selfdrive(enabled=False, active=False, state='disabled'), onroad=True,
                              car_state=None)
  stale_selfdrive = preview_payload(selfdrive_state=None, onroad=True)
  for result in (stale_car, stale_selfdrive):
    assert 'roadPreview' not in result and 'navPathLeft' not in result and result['navVehicles'] == []


def polyline_x_at(points, screen_y):
  pairs = list(zip(points[0::2], points[1::2]))
  for (x0, y0), (x1, y1) in zip(pairs, pairs[1:]):
    if y0 != y1 and min(y0, y1) <= screen_y <= max(y0, y1):
      return x0 + (x1 - x0) * (screen_y - y0) / (y1 - y0)
  return None


def test_a_plan_shorter_than_the_lanes_keeps_vehicles_on_the_drawn_lanes():
  # Codex review 2026-11-15: the plan ends at 60 m and the lanes go on. Right past the end the path correction
  # dropped out for vehicles, 17 px off the drawn lane, and the yaw step across the end hit its 24 deg limit.
  path = NS(x=[0.0, 20.0, 40.0, 60.0], y=[0.0, 0.0, 2.0, 6.0])
  xs = [i * 2.5 for i in range(33)]
  lane = NS(x=xs, y=[0.1 * x - 1.8 for x in xs])
  drawn = navdy_model_line(lane.x, lane.y, 0.0, path.x, path.y)
  m = NS(position=path, laneLines=[lane] * 4, laneLineProbs=[0.9] * 4)
  previous = None
  for camera in (59.5, 59.9, 60.0, 60.1, 60.5, 65.0, 75.0):
    view = navdy_vehicle_view(m, camera - 1.52, 0.1 * camera - 1.8)
    assert abs(view['screenX'] - polyline_x_at(drawn, view['screenY'])) < 1.5
    if previous is not None and camera - previous[0] <= 0.5:
      assert abs(view['screenX'] - previous[1]) < 2.5
    previous = (camera, view['screenX'])
  yaws = [vehicle_yaw(m, 'left', camera - 1.52, 0.1 * camera - 1.8) for camera in (58.5, 59.5, 60.5)]
  assert max(yaws) - min(yaws) < 3.0 and all(abs(yaw) < 20.0 for yaw in yaws)
