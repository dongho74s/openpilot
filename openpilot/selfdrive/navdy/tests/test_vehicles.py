import copy
import json
import math
from types import SimpleNamespace as NS
import zlib

import pytest

from openpilot.selfdrive.navdy import bridge
from openpilot.selfdrive.navdy.display import lateral_scale, navdy_lead_geometry, navdy_model_line, navdy_project_point
from openpilot.selfdrive.navdy.tests.test_bridge import Messages
from openpilot.selfdrive.navdy.tests.test_display import lead, model, payload, selfdrive
from openpilot.selfdrive.navdy.transport import encode_frame
from openpilot.selfdrive.navdy.vehicles import MAX_TRACKS, MAX_VEHICLES, TRACK_MAX_AGE, VehicleOverlay


def point(ident=1, distance=30., lateral=3.6, speed=-1.):
  return NS(trackId=ident, dRel=distance, yRel=lateral, vRel=speed, measured=True)


def data(points=None, **errors):
  return NS(points=[point()] if points is None else points, errors=NS(**errors), radarTrackFlipped=False)


def render(overlay, radar=None, *, timestamp=100., now=None, m=None, leads=None, **kwargs):
  m = model() if m is None else m
  result = payload(model=m, radar_state=leads, **kwargs)
  overlay.update(result, m, data() if radar is None else radar, timestamp=timestamp,
                 now=timestamp if now is None else now)
  return result


def confirmed(overlay=None, radar=None, **kwargs):
  overlay = VehicleOverlay() if overlay is None else overlay
  render(overlay, radar, timestamp=100., **kwargs)
  return render(overlay, radar, timestamp=100.2, **kwargs)


def test_surrounding_left_center_right_requires_two_distinct_samples():
  overlay = VehicleOverlay()
  radar = data([point(1, lateral=3.6), point(2, lateral=0), point(3, lateral=-3.6)])
  original = copy.deepcopy(radar)
  assert render(overlay, radar)['navVehicles'] == []
  assert render(overlay, radar, now=100.1)['navVehicles'] == []
  result = render(overlay, radar, timestamp=100.2)
  cars = {car['lane']: car for car in result['navVehicles']}
  assert len(cars) == 3
  assert cars['left']['screenX'] < 160 < cars['right']['screenX']
  assert cars['left']['lateralM'] == -3.6 and cars['right']['lateralM'] == 3.6
  assert cars['left']['yawDeg'] < 0 < cars['right']['yawDeg']
  assert radar == original


def test_preserves_leads_and_every_non_vehicle_field():
  radar = data([point(42, 40., 0.), point(2, lateral=3.6), point(3, lateral=-3.6)])
  leads = NS(leadOne=lead(), leadTwo=lead())
  plan = NS(longitudinalPlanSource='lead0')
  result = confirmed(radar=radar, leads=leads, longitudinal_plan=plan)
  baseline = payload(model=model(), radar_state=leads, longitudinal_plan=plan)
  assert len(result['navVehicles']) == 3
  assert [car for car in result['navVehicles'] if car['trackId'] == 42] == baseline['navVehicles']
  assert {k: v for k, v in result.items() if k != 'navVehicles'} == {k: v for k, v in baseline.items() if k != 'navVehicles'}


def test_camera_only_lead_is_not_duplicated_by_matching_raw_point():
  result = confirmed(radar=data([point(42, 40., 0.)]), leads=NS(leadOne=lead(radar=False, radarTrackId=-1)))
  assert len(result['navVehicles']) == 1 and result['navVehicles'][0]['colorRole'] == 1


def test_camera_only_lead_is_drawn_at_its_radar_point_so_radard_flips_do_not_move_it():
  # 11/15 drive: radard alternated a far lead between radar track 25 (56 m) and the model's 47.5 m every few ticks.
  overlay = VehicleOverlay()
  radar = data([point(25, 56., 0., -2.)])
  vision = NS(leadOne=lead(radar=False, radarTrackId=-1, dRel=47.5), leadTwo=lead(status=False))
  backed = NS(leadOne=lead(radar=True, radarTrackId=25, dRel=56.), leadTwo=lead(status=False))
  confirmed(overlay, radar, leads=vision)
  frames = [render(overlay, radar, timestamp=100.2 + 0.2 * i, leads=leads)['navVehicles']
            for i, leads in enumerate((vision, backed, vision, backed), start=1)]
  assert all(len(cars) == 1 for cars in frames)
  keys = ('trackId', 'screenX', 'screenY', 'pxPerM', 'distanceM', 'lateralM', 'lane', 'colorRole', 'source')
  assert len({tuple(cars[0][key] for key in keys) for cars in frames}) == 1
  assert frames[0][0]['distanceM'] == 56. and frames[0][0]['colorRole'] == 1 and frames[0][0]['trackId'] == 25


def test_camera_only_lead_without_a_matching_radar_point_keeps_its_own_range():
  leads = NS(leadOne=lead(radar=False, radarTrackId=-1, dRel=47.5), leadTwo=lead(status=False))
  for radar in (data([]), data([point(7, 47.5, 3.6)]), data([point(8, 70., 0.)]), data([point(9, 47.5, 0., 10.)])):
    cars = confirmed(VehicleOverlay(), radar, leads=leads)['navVehicles']
    lead_car = next(car for car in cars if car['colorRole'] == 1)
    assert lead_car['distanceM'] == 47.5 and lead_car['trackId'] == -1 and lead_car['source'] == 'vision'


@pytest.mark.parametrize('distance,expected', [
  (41., [(1, 41.), (2, 44.)]), (43.5, [(1, 40.), (2, 43.5)]), (42., [(1, 42.), (2, 44.)]),  # the closer lead; lead one on a tie
])
def test_one_radar_point_lends_its_position_to_one_camera_only_lead(distance, expected):
  leads = NS(leadOne=lead(radar=False, radarTrackId=-1, dRel=40.), leadTwo=lead(radar=False, radarTrackId=-1, dRel=44.))
  cars = confirmed(radar=data([point(25, distance, 0., -2.)]), leads=leads)['navVehicles']
  assert sorted((car['colorRole'], car['distanceM']) for car in cars) == expected


def test_a_confirmed_point_missing_from_a_snapshot_is_held_and_coasted():
  overlay = VehicleOverlay()
  assert confirmed(overlay, data([point(1, 30., 3.6, -1.)]))['navVehicles']   # seen at 100.0 and 100.2
  held = render(overlay, data([]), timestamp=100.4)['navVehicles']
  assert len(held) == 1 and held[0]['distanceM'] == 29.8 and held[0]['lateralM'] == -3.6
  back = render(overlay, data([point(1, 29.7, 3.6, -1.)]), timestamp=100.5)['navVehicles']
  assert len(back) == 1   # no new confirmation needed after a short gap


def test_hold_runs_from_the_last_sighting_and_never_extends_itself():
  overlay = VehicleOverlay()
  confirmed(overlay)
  for timestamp in (100.3, 100.4, 100.5, 100.55):
    assert render(overlay, data([]), timestamp=timestamp)['navVehicles']
  assert render(overlay, data([]), timestamp=100.6)['navVehicles'] == []
  assert not overlay.tracks


def test_unconfirmed_points_are_not_held_and_holding_never_crosses_the_range_limits():
  overlay = VehicleOverlay()
  assert render(overlay, timestamp=100.)['navVehicles'] == []
  assert render(overlay, data([]), timestamp=100.2)['navVehicles'] == [] and not overlay.tracks
  assert render(overlay, timestamp=100.3)['navVehicles'] == []   # a lone sighting again: still unconfirmed
  near = VehicleOverlay()
  assert confirmed(near, data([point(1, 2.4, 3.6, -5.)]))['navVehicles']
  assert render(near, data([]), timestamp=100.3)['navVehicles'] == []   # would be drawn at 1.9 m


def test_different_radar_ids_are_not_merged_just_because_they_are_close():
  result = confirmed(radar=data([point(43, 44., 0.)]), leads=NS(leadOne=lead()))
  assert len(result['navVehicles']) == 2


def test_eight_vehicle_limit_preserves_leads_and_sorts_far_to_near():
  radar = data([point(i, 10. + i * 2., (i % 3 - 1) * 3.6) for i in range(20)])
  leads = NS(leadOne=lead(dRel=78.), leadTwo=lead(dRel=77., radarTrackId=43))  # camera x within the test lanes (80 m)
  result = confirmed(radar=radar, leads=leads)
  cars = result['navVehicles']
  assert len(cars) == MAX_VEHICLES
  assert {42, 43} <= {car['trackId'] for car in cars}
  assert [car['distanceM'] for car in cars] == sorted([car['distanceM'] for car in cars], reverse=True)
  encoded = encode_frame(result)
  assert json.loads(encoded)['navVehicles'] == cars
  assert len(encoded) < 32768
  assert zlib.decompress(zlib.compress(encoded)) == encoded


@pytest.mark.parametrize('error', ['canError', 'radarFault', 'wrongConfig', 'radarUnavailableTemporary'])
def test_radar_error_clears_overlay_but_not_published_lead(error):
  overlay = VehicleOverlay()
  assert confirmed(overlay)['navVehicles']
  leads = NS(leadOne=lead())
  result = render(overlay, data(**{error: True}), timestamp=100.3, leads=leads)
  assert result['navVehicles'] == navdy_lead_geometry(model(), leads)['navVehicles']
  assert not overlay.tracks


@pytest.mark.parametrize('age', [TRACK_MAX_AGE + .001, 1., -0.01, math.nan, math.inf])
def test_stale_future_or_nonfinite_capture_clears_overlay(age):
  overlay = VehicleOverlay()
  confirmed(overlay)
  assert render(overlay, timestamp=100.2, now=100.2 + age)['navVehicles'] == []
  assert not overlay.tracks


def test_empty_snapshot_disappearance_and_late_reappearance_require_confirmation():
  overlay = VehicleOverlay()
  assert confirmed(overlay)['navVehicles']
  assert render(overlay, data([]), timestamp=100.6)['navVehicles'] == []   # past the hold
  assert render(overlay, timestamp=100.7)['navVehicles'] == []
  assert render(overlay, timestamp=100.9)['navVehicles']
  assert render(overlay, timestamp=101.3)['navVehicles'] == []


@pytest.mark.parametrize('kwargs', [
  {'onroad': False}, {'selfdrive_state': None}, {'car_state': None},
])
def test_invalid_state_clears_immediately(kwargs):
  overlay = VehicleOverlay()
  confirmed(overlay)
  assert render(overlay, timestamp=100.3, **kwargs)['navVehicles'] == []
  assert not overlay.tracks
  assert render(overlay, timestamp=100.4)['navVehicles'] == []


@pytest.mark.parametrize('field,value', [
  ('trackId', -1), ('trackId', math.nan), ('trackId', 2**53), ('trackId', 1.5),
  ('dRel', 1.99), ('dRel', 80.01), ('dRel', math.inf), ('dRel', math.nan),
  ('yRel', math.nan), ('yRel', 16.), ('vRel', math.nan), ('vRel', 101.),
])
def test_bad_points_are_rejected(field, value):
  p = point()
  setattr(p, field, value)
  assert confirmed(radar=data([p]))['navVehicles'] == []


def test_tracker_bounded_and_duplicate_ids_not_counted_as_two_samples():
  overlay = VehicleOverlay()
  assert render(overlay, data([point()] * 500))['navVehicles'] == []
  confirmed(overlay, data([point(i) for i in range(1000)]))
  assert len(overlay.tracks) == MAX_TRACKS


def test_radar_flip_is_not_applied_twice_and_change_resets_smoothing():
  overlay = VehicleOverlay()
  radar = data([point(lateral=-3.6)])
  radar.radarTrackFlipped = True
  assert confirmed(overlay, radar)['navVehicles'][0]['lane'] == 'right'
  radar.radarTrackFlipped = False
  assert render(overlay, radar, timestamp=100.3)['navVehicles'] == []


def test_path_fallback_and_known_outer_boundary():
  m = model()
  m.laneLineProbs = [0.] * 4
  assert confirmed(m=m)['navVehicles'][0]['lane'] == 'left'
  assert confirmed(radar=data([point(lateral=8.)]))['navVehicles'] == []
  m.position.x = []
  assert confirmed(m=m)['navVehicles'] == []


@pytest.mark.parametrize('distance', [10., 30., 51., 52., 70., 78.])  # camera x = dRel + 1.52 <= the 80 m test lanes
@pytest.mark.parametrize('offset,lane', [(-3.6, 'left'), (0., 'center'), (3.6, 'right')])
def test_short_path_vehicles_match_lane_projection(distance, offset, lane):
  m = model()
  m.position.x, m.position.y = [0., 10., 20.], [0., .5, 1.]
  for line in m.laneLines:
    line.y = [y + x * .05 for x, y in zip(line.x, line.y, strict=True)]
  lateral = distance * .05 + offset
  radar = data([point(distance=distance, lateral=-lateral)])
  before = copy.deepcopy(m)
  result = confirmed(radar=radar, m=m)
  assert len(result['navVehicles']) == 1
  actual = result['navVehicles'][0]
  x = distance + 1.52  # radar dRel -> camera; drawn like the lanes at that distance (beyond the plan: its end held)
  path_y = min(x, 20.) * .05
  assert [actual['screenX'], actual['screenY']] == navdy_project_point(x, lateral, path_y)
  assert actual['pxPerM'] == round(lateral_scale(x), 2)
  assert actual['lane'] == lane
  assert actual['lateralM'] == round(lateral, 2)
  assert actual['distanceM'] == distance
  assert m == before


@pytest.mark.parametrize('mutation', [
  'low_probability', 'nonfinite_probability', 'missing_probability', 'missing_lanes',
  'short_left', 'short_right', 'before_lane_start', 'reversed_x', 'nonfinite_y',
])
def test_outside_path_requires_supported_current_lane(mutation):
  m = model()
  m.position.x, m.position.y = [0., 20.], [0., 0.]
  if mutation == 'low_probability':
    m.laneLineProbs[1] = .299
  elif mutation == 'nonfinite_probability':
    m.laneLineProbs[2] = math.nan
  elif mutation == 'missing_probability':
    m.laneLineProbs = m.laneLineProbs[:2]
  elif mutation == 'missing_lanes':
    m.laneLines = m.laneLines[:2]
  elif mutation in ('short_left', 'short_right'):
    line = m.laneLines[1 if mutation == 'short_left' else 2]
    line.x, line.y = line.x[:3], line.y[:3]
  elif mutation == 'before_lane_start':
    for line in m.laneLines:
      line.x, line.y = line.x[16:], line.y[16:]
  elif mutation == 'reversed_x':
    m.laneLines[1].x[1] = -1.
    m.laneLines[1].x[2] = 0.
  elif mutation == 'nonfinite_y':
    m.laneLines[2].y = [math.nan] * len(m.laneLines[2].y)
  assert confirmed(radar=data([point(lateral=0.)]), m=m)['navVehicles'] == []


def test_no_lane_confidence_only_allows_existing_in_path_fallback():
  m = model()
  m.position.x, m.position.y = [0., 20.], [0., 0.]
  m.laneLineProbs = [0.] * 4
  assert confirmed(radar=data([point(distance=10.)]), m=m)['navVehicles']
  assert confirmed(radar=data([point(distance=30.)]), m=m)['navVehicles'] == []


@pytest.mark.parametrize('display_data', [False, True])
def test_short_path_preserves_lead_dedup_and_non_vehicle_fields(display_data):
  m = model()
  m.position.x, m.position.y = [0., 20.], [0., 0.]
  leads = NS(leadOne=lead(dRel=10.))
  matching = point(99 if display_data else 42, 10., 0., -2.)
  points = [matching, matching, point(2, 70., 3.6)]
  radar = data(points)
  radar.hasDisplayData, radar.displayPoints = display_data, points
  kwargs = {'longitudinal_plan': NS(longitudinalPlanSource='lead0'),
            'car_control': NS(latActive=True, longActive=True), 'car_output': NS(actuatorsOutput=NS(accel=-.5))}
  baseline = payload(model=m, radar_state=leads, **kwargs)
  result = confirmed(radar=radar, m=m, leads=leads, **kwargs)
  assert len(result['navVehicles']) == 2
  assert result['navVehicles'][-1] == baseline['navVehicles'][0]
  assert result['navVehicles'][0]['trackId'] == 2
  assert {k: v for k, v in result.items() if k != 'navVehicles'} == {k: v for k, v in baseline.items() if k != 'navVehicles'}


def test_short_path_fallback_holds_old_lane_or_radar_data_only_briefly():
  m = model()
  m.position.x, m.position.y = [0., 20.], [0., 0.]
  overlay = VehicleOverlay()
  assert confirmed(overlay, m=m)['navVehicles']   # lane given at 100.2
  m.laneLineProbs[1] = .1
  assert render(overlay, timestamp=100.3, m=m)['navVehicles'][0]['lane'] == 'left'   # lane held
  assert render(overlay, timestamp=100.55, m=m)['navVehicles']
  assert render(overlay, timestamp=100.6, m=m)['navVehicles'] == []   # 0.4 s since the lane was last given
  m.laneLineProbs[1] = .9
  assert render(overlay, data([]), timestamp=101.0, m=m)['navVehicles'] == []
  assert render(overlay, timestamp=101.1, m=m)['navVehicles'] == []
  assert render(overlay, timestamp=101.2, m=m)['navVehicles']
  assert render(overlay, timestamp=101.2, now=101.551, m=m)['navVehicles'] == []


def test_track_reuse_jump_and_clock_rollback_require_new_confirmation():
  overlay = VehicleOverlay()
  confirmed(overlay)
  assert render(overlay, data([point(distance=70.)]), timestamp=100.3)['navVehicles'] == []
  assert render(overlay, data([point(distance=70.)]), timestamp=100.4)['navVehicles']
  assert render(overlay, timestamp=99.)['navVehicles'] == []


def test_bridge_subscribes_and_applies_only_fresh_tracks(monkeypatch):
  clock = NS(now=100.)
  monkeypatch.setattr(bridge.time, 'monotonic', lambda: clock.now)
  sm = Messages()
  sm.data['liveTracks'] = data([point(1, lateral=3.6), point(2, lateral=-3.6)])
  overlay = VehicleOverlay()
  assert 'liveTracks' in bridge.SERVICES
  assert bridge.live_payload(sm, 1, vehicles=overlay)['navVehicles'] == []
  clock.now = 100.2
  sm.recv_time['liveTracks'] = clock.now
  sm.logMonoTime['liveTracks'] = int(clock.now * 1e9)
  result = bridge.live_payload(sm, 2, vehicles=overlay)
  assert len(result['navVehicles']) == 2
  sm.valid['liveTracks'] = False
  result = bridge.live_payload(sm, 3, vehicles=overlay)
  assert result['active'] and result['navVehicles'] == []
  sm.valid['liveTracks'] = True
  sm.delay('liveTracks', .6)
  assert bridge.live_payload(sm, 4, vehicles=overlay)['navVehicles'] == []


def test_smoothing_uses_fresh_measurements_not_render_ticks():
  overlay = VehicleOverlay()
  confirmed(overlay)
  radar = data([point(distance=32.)])
  first = render(overlay, radar, timestamp=100.3)['navVehicles']
  assert first[0]['distanceM'] == 31.1
  assert render(overlay, radar, timestamp=100.3, now=100.4)['navVehicles'] == first


def test_supported_display_stream_is_authoritative_including_empty():
  overlay = VehicleOverlay()
  radar = data([point(1)])
  radar.hasDisplayData = True
  radar.displayPoints = [point(2, lateral=-3.6)]
  cars = confirmed(overlay, radar)['navVehicles']
  assert [car['trackId'] for car in cars] == [2]
  assert cars[0]['lane'] == 'right'
  radar.displayPoints = []
  assert [car['trackId'] for car in render(overlay, radar, timestamp=100.3)['navVehicles']] == [2]   # held, not points
  assert render(overlay, radar, timestamp=100.6)['navVehicles'] == []
  radar.hasDisplayData = False
  assert render(overlay, radar, timestamp=100.7)['navVehicles'] == []
  assert render(overlay, radar, timestamp=100.9)['navVehicles'][0]['trackId'] == 1


@pytest.mark.parametrize('p,expected_count', [
  (point(42, 40., 3.6), 2),  # Same ID in an independent namespace, different lane.
  (point(99, 40., 0.), 1),   # Different ID, matching lead geometry.
  (point(42, 50., 0.), 2),   # Same ID, same lane, but another vehicle.
  (point(99, 40., 0., 10.), 2),  # Similar position, incompatible velocity.
])
def test_display_stream_dedup_uses_geometry_not_lead_id(p, expected_count):
  radar = data([])
  radar.hasDisplayData = True
  radar.displayPoints = [p]
  leads = NS(leadOne=lead())
  result = confirmed(radar=radar, leads=leads)
  assert len(result['navVehicles']) == expected_count
  assert navdy_lead_geometry(model(), leads)['navVehicles'][0] in result['navVehicles']


def test_changing_track_namespace_requires_two_new_samples():
  overlay = VehicleOverlay()
  radar = data([point()])
  assert confirmed(overlay, radar)['navVehicles']
  radar.hasDisplayData = True
  radar.displayPoints = [point()]
  assert render(overlay, radar, timestamp=100.3)['navVehicles'] == []
  assert render(overlay, radar, timestamp=100.4)['navVehicles']
  radar.hasDisplayData = False
  assert render(overlay, radar, timestamp=100.5)['navVehicles'] == []
  assert render(overlay, radar, timestamp=100.6)['navVehicles']


def test_disengaging_keeps_the_road_preview_tracks():
  overlay = VehicleOverlay()
  confirmed(overlay)
  result = render(overlay, timestamp=100.3, selfdrive_state=selfdrive(enabled=False, active=False))
  assert result['roadPreview'] and len(result['navVehicles']) == 1 and overlay.tracks



def test_every_drawn_vehicle_has_a_colour_role():
  # HUD v185: lead one 1 (sky blue), lead two 2 (pale sky blue), every other vehicle 0 (grey), whatever its source.
  m = model()
  radar = data([point(ident=7, distance=25., lateral=3.6)])
  result = confirmed(radar=radar, m=m)
  assert result['navVehicles'] and all(v['colorRole'] == 0 for v in result['navVehicles'] if v['source'] == 'radar')


def test_a_flickering_lane_check_keeps_the_last_lane_briefly():
  m = model()
  overlay = VehicleOverlay()
  radar = data([point(1, 30., -3.6)])   # right lane
  assert confirmed(overlay, radar, m=m)['navVehicles'][0]['lane'] == 'right'
  outside = copy.deepcopy(m)
  outside.laneLines[3].y = [-1.] * len(outside.laneLines[3].y)   # far right line moved inside: the point is "outside"
  cars = render(overlay, radar, timestamp=100.4, m=outside)['navVehicles']
  assert [car['lane'] for car in cars] == ['right']
  assert render(overlay, radar, timestamp=100.6, m=m)['navVehicles'][0]['lane'] == 'right'   # given again: hold restarts
  assert render(overlay, radar, timestamp=100.9, m=outside)['navVehicles']
  assert render(overlay, radar, timestamp=101.0, m=outside)['navVehicles'] == []


def test_lane_memory_is_per_track_and_cleared_with_the_overlay():
  overlay = VehicleOverlay()
  confirmed(overlay, data([point(1, 30., -3.6)]))
  assert set(overlay.lanes) == {1}
  render(overlay, data([point(2, 30., -3.6)]), timestamp=100.6)
  assert set(overlay.lanes) == set()   # point 1 gone, point 2 not confirmed yet
  confirmed(overlay, data([point(1, 30., -3.6)]))
  overlay.clear()
  assert overlay.lanes == {}
