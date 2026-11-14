"""Navdy sprites covering each other: a truck's own radar points and vehicles hidden behind a nearer one."""
from types import SimpleNamespace as NS

import pytest

from openpilot.selfdrive.navdy import sprite_overlap as so
from openpilot.selfdrive.navdy.display import lateral_scale, navdy_project_point, navdy_vehicle_view


def vehicle(ident, distance, lateral, kind=None, source='radar', lead=False, yaw=0.0):
  x = distance + 1.52
  sx, sy = navdy_project_point(x, lateral, 0.0)
  v = {'trackId': ident, 'screenX': sx, 'screenY': sy, 'pxPerM': round(lateral_scale(x), 2), 'yawDeg': yaw,
       'distanceM': distance, 'lateralM': lateral, 'lane': 'center', 'source': source, 'longitudinalLead': lead,
       'relativeSpeedMps': 0.0, 'confidence': 0.65}
  if kind:
    v['kind'] = kind
  return v


def settle(*vehicles):
  payload = {'navVehicles': list(vehicles)}
  so.settle_overlaps(payload)
  return sorted(v['trackId'] for v in payload['navVehicles'])


def test_a_trucks_own_side_points_are_not_drawn_as_cars():
  truck = vehicle(1, 25.0, 0.2, kind='truck', lead=True)
  side = [vehicle(2, 28.0, 1.1), vehicle(3, 32.0, -0.9), vehicle(4, 34.5, 1.3)]
  assert settle(truck, *side) == [1]


def test_only_unconfirmed_points_inside_the_truck_body_count_as_the_truck():
  truck = vehicle(1, 25.0, 0.0, kind='truck')
  inside = [vehicle(2, 25.5, 0.0), vehicle(3, 35.0, 1.5)]
  outside = [vehicle(4, 25.0, 0.0),                 # level with its rear (another point, not behind it)
             vehicle(5, 27.0, 3.5),                 # next lane
             vehicle(6, 35.5, 0.2),                 # past the body
             vehicle(7, 30.0, 0.5, kind='moto'),    # the camera saw an object of its own there
             vehicle(10, 30.0, 0.5, kind='car'),    # ... also when it is a car
             vehicle(8, 30.0, 0.5, source='vision'),
             vehicle(9, 30.0, 0.5, lead=True),
             dict(vehicle(11, 30.0, 0.5), lane='left')]   # another lane by the model's lanes
  assert all(so._in_truck_body(v, [truck]) for v in inside)
  assert not any(so._in_truck_body(v, [truck]) for v in outside)
  assert settle(truck, vehicle(5, 27.0, 3.5)) == [1, 5]


def test_a_mostly_hidden_vehicle_behind_a_nearer_one_is_left_out():
  near = vehicle(1, 15.0, 0.0)
  hidden = vehicle(2, 20.0, 0.1)
  assert settle(near, hidden) == [1]
  half = vehicle(3, 20.0, 1.6)            # sticks out to the side: still drawn
  assert settle(near, half) == [1, 3]


def test_the_longitudinal_lead_is_always_drawn():
  near = vehicle(1, 15.0, 0.0)
  lead = vehicle(2, 20.0, 0.1, lead=True)
  assert settle(near, lead) == [1, 2]


def test_order_far_to_near_and_bad_entries_are_kept():
  near, far = vehicle(1, 15.0, -3.5), vehicle(2, 40.0, 3.5)
  odd = dict(vehicle(3, 30.0, 0.0), pxPerM=float('nan'))
  payload = {'navVehicles': [near, odd, far]}
  so.settle_overlaps(payload)
  assert [v['trackId'] for v in payload['navVehicles']] == [2, 3, 1]


def test_sprite_box_matches_the_hud_placement():
  v = vehicle(1, 20.0, 0.0)
  left, top, right, bottom = so.sprite_box(v)
  width, height, anchor_x, anchor_y, ppm = so.SPRITES['car'][12]
  scale = v['pxPerM'] / ppm
  assert left == v['screenX'] - anchor_x * scale and top == v['screenY'] - anchor_y * scale
  assert (right - left) == pytest.approx(width * scale) and (bottom - top) == pytest.approx(height * scale)
  assert so.frame_index(-25) == 0 and so.frame_index(1.0) == 13 and so.frame_index(float('nan')) == 12


def curve_model(curvature=0.0035):
  xs = [i * 2.5 for i in range(33)]
  return NS(position=NS(x=xs, y=[curvature * x * x for x in xs]))


def placed(model, ident, distance, lateral, yaw, **extra):
  view = navdy_vehicle_view(model, distance, lateral)
  return {'trackId': ident, 'distanceM': distance, 'lateralM': lateral, 'yawDeg': yaw, 'source': 'radar',
          'longitudinalLead': False, 'relativeSpeedMps': 0.0, 'confidence': 0.65, **view, **extra}


def test_the_next_lane_on_a_curve_is_not_the_truck():
  # Codex review 2026-11-15: raw laterals 0.70 m apart at 40 / 49 m, but 3.6 m apart from the curving path.
  model = curve_model()
  truck = placed(model, 1, 40.0, 6.0389, -9.9, kind='truck', lane='center')
  car = placed(model, 2, 49.0, 5.33655, -14.1, lane='left')
  assert not so._in_truck_body(car, [truck], model)
  payload = {'navVehicles': [truck, car]}
  so.settle_overlaps(payload, model)
  assert sorted(v['trackId'] for v in payload['navVehicles']) == [1, 2]
  same_lane = placed(model, 3, 46.0, 0.0035 * 47.52 ** 2 + 0.4, -10.0, lane='center')
  assert so._in_truck_body(same_lane, [truck], model)


def test_transparent_corners_hide_nothing():
  # Codex review 2026-11-15: their boxes overlap 62%, but only ~4% of the motorcycle's body is behind the car.
  straight = curve_model(0.0)
  car = placed(straight, 1, 50.3, -1.64, -1.6)
  moto = dict(placed(straight, 2, 68.5, -3.85, -4.3, kind='moto'), source='vision')
  assert so._covered_by_boxes(so.sprite_box(moto), so.sprite_box(car)) > 0.6
  assert so.hidden_fraction(so._placement(moto), [so._placement(car)]) < 0.2
  payload = {'navVehicles': [car, moto]}
  so.settle_overlaps(payload, straight)
  assert sorted(v['trackId'] for v in payload['navVehicles']) == [1, 2]


def test_a_body_hidden_by_two_nearer_vehicles_together_is_left_out():
  left, right = vehicle(1, 15.0, -0.9), vehicle(2, 15.0, 0.9)
  behind = vehicle(3, 21.0, 0.0)
  nearer = [so._placement(left), so._placement(right)]
  assert all(so.hidden_fraction(so._placement(behind), [n]) < so.HIDDEN_FRACTION for n in nearer)
  assert so.hidden_fraction(so._placement(behind), nearer) >= so.HIDDEN_FRACTION
  assert settle(left, right, behind) == [1, 2]


def test_alpha_grids_match_the_frame_table():
  grids = so._alpha()
  for kind, frames in so.SPRITES.items():
    assert len(frames) == len(grids[kind]) == 25
    for cells, filled in grids[kind]:
      assert len(cells) == so.CELLS * so.CELLS and all(0.0 <= a <= 1.0 for a in cells)
      assert filled and all(cells[r * so.CELLS + c] == a > 0.0 for r, c, a in filled)


def test_changed_alpha_data_fails_the_pass_not_the_import(monkeypatch):
  monkeypatch.setattr(so, '_alpha_cache', {})
  monkeypatch.setattr(so, 'ALPHA_SHA256', '0' * 64)
  with pytest.raises(ValueError):
    so.settle_overlaps({'navVehicles': [vehicle(1, 15.0, 0.0), vehicle(2, 20.0, 0.1)]})


def test_a_separate_car_beside_the_truck_on_a_curve_is_kept():
  # Codex review 2026-11-15 (N1): same lane, 10 m ahead, 1 m apart from the path, but drawn ~90% beside the truck.
  model = curve_model(0.001)
  truck = placed(model, 1, 60.0, 3.2862, -7.1, kind='truck', lane='center')
  car = placed(model, 2, 70.0, 5.6166, -7.1, lane='center')
  assert so._in_truck_body(car, [truck], model)
  assert so.hidden_fraction(so._placement(car), [so._placement(truck)]) < so.TRUCK_HIDDEN_FRACTION
  payload = {'navVehicles': [truck, car]}
  so.settle_overlaps(payload, model)
  assert sorted(v['trackId'] for v in payload['navVehicles']) == [1, 2]


def test_a_motorcycle_half_behind_a_truck_is_kept():
  # Codex review 2026-11-15 (N2): 43% hidden by the PNGs; a 16x16 opaque-cell estimate said 70%.
  truck = {'trackId': 1, 'kind': 'truck', 'yawDeg': -2.2, 'screenX': 128.9, 'screenY': 29.7, 'pxPerM': 16.89,
           'distanceM': 50.2, 'lateralM': -1.84, 'source': 'vision'}
  moto = {'trackId': 2, 'kind': 'moto', 'yawDeg': -0.7, 'screenX': 152.7, 'screenY': 19.2, 'pxPerM': 12.60,
          'distanceM': 63.3, 'lateralM': -0.58, 'source': 'vision'}
  assert so.hidden_fraction(so._placement(moto), [so._placement(truck)]) == pytest.approx(0.43, abs=0.05)
  payload = {'navVehicles': [truck, moto]}
  so.settle_overlaps(payload)
  assert sorted(v['trackId'] for v in payload['navVehicles']) == [1, 2]


def test_a_truck_left_out_hides_nothing_behind_it():
  # Codex review 2026-11-15 (N3): truck B is hidden behind truck A; car C sat in B's body but ~86% outside A.
  xs = [i * 2.5 for i in range(33)]
  model = NS(position=NS(x=xs, y=[-0.002 * x * x for x in xs]))
  from openpilot.selfdrive.navdy.vehicles import vehicle_yaw
  lane = NS(x=xs, y=[-0.002 * x * x for x in xs])
  lanes = NS(position=model.position, laneLines=[lane] * 4, laneLineProbs=[0.9] * 4)

  def entry(ident, distance, lateral, **extra):
    return placed(model, ident, distance, lateral, vehicle_yaw(lanes, 'center', distance, lateral), lane='center', **extra)

  a = entry(1, 25.5, -1.29210, kind='truck', longitudinalLead=True)
  b = entry(2, 37.75, -2.83685, kind='truck')
  c = entry(3, 43.5, -4.80370)
  assert so.hidden_fraction(so._placement(b), [so._placement(a)]) >= so.HIDDEN_FRACTION
  assert so.hidden_fraction(so._placement(c), [so._placement(a)]) < so.TRUCK_HIDDEN_FRACTION
  payload = {'navVehicles': [a, b, c]}
  so.settle_overlaps(payload, model)
  assert [v['trackId'] for v in payload['navVehicles']] == [3, 1]
