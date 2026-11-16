"""Bounded Navdy-only radar overlay. Never publishes or modifies control inputs."""
from __future__ import annotations

from itertools import islice
import math

from openpilot.selfdrive.navdy.display import (
  _lead_lane, camera_x, drawing_path_y_at, finite_float, model_line_y_at, navdy_project_point, navdy_vehicle_view,
  rounded,
)

MAX_TRACKS = 64
MAX_VEHICLES = 8
TRACK_MAX_AGE = 0.35
# A confirmed radar point missing from a fresh snapshot is kept this long after its last sighting (radar time),
# coasting at its relative speed, like Wayon's NAVDY_RADAR_STALE_SEC. Points drop out of single snapshots often, and
# dropping them at once, then re-confirming over two 5 Hz ticks, blinked icons for 0.4 s (user report 2026-11-16).
TRACK_HOLD = 0.35
# A confirmed point whose lane check fails keeps the lane it was last given for this long (radar time). The check
# flickers for a steady point on a lane line, or while stopped when the plan is too short (11/15: ~100 one-tick
# blinks in 47 minutes). A point that really leaves the lanes goes once this runs out.
LANE_HOLD = 0.35
MAX_DISTANCE = 80.0


def vehicle_lane(model, distance, lateral):
  lane = _lead_lane(model, distance, lateral)
  if lane:
    return lane
  # Wayon's display fallback uses the path and measured/default lane width (at the camera distance, like the view).
  distance = camera_x(distance)
  path_y = model_line_y_at(getattr(model, 'position', None), distance)
  if path_y is None:
    return ''
  lanes = list(islice(getattr(model, 'laneLines', ()), 4))
  probs = list(islice(getattr(model, 'laneLineProbs', ()), 4))
  boundaries = [model_line_y_at(line, distance) for line in lanes]
  if len(boundaries) >= 4 and len(probs) >= 4:
    if boundaries[0] is not None and lateral < boundaries[0] and min(probs[:2]) >= 0.2:
      return ''
    if boundaries[3] is not None and lateral > boundaries[3] and min(probs[2:4]) >= 0.2:
      return ''
  width = 3.6
  if len(boundaries) >= 3 and boundaries[1] is not None and boundaries[2] is not None:
    measured = boundaries[2] - boundaries[1]
    if 2.4 <= measured <= 4.8:
      width = measured
  offset = lateral - path_y
  if abs(offset) <= width / 2:
    return 'center'
  if -1.5 * width <= offset < -width / 2:
    return 'left'
  if width / 2 < offset <= 1.5 * width:
    return 'right'
  return ''


def lane_heading(model, lane, distance):
  """Direction of the vehicle's lane (radians, right-positive) around a radar distance, 0 when unknown."""
  distance = camera_x(distance)
  near, far = max(0.0, distance - 4.0), min(MAX_DISTANCE, distance + 4.0)
  lanes = list(islice(getattr(model, 'laneLines', ()), 4))
  indices = {'left': (0, 1), 'right': (2, 3)}.get(lane)

  def center(d):
    if indices and len(lanes) > indices[1]:
      ys = [model_line_y_at(lanes[index], d) for index in indices]
      if all(y is not None for y in ys):
        return sum(ys) / 2
    return model_line_y_at(getattr(model, 'position', None), d)

  y0, y1 = center(near), center(far)
  if y0 is None or y1 is None or far - near < 1.0:
    return 0.0
  return math.atan2(y1 - y0, far - near)


SPRITE_PITCH_DEG = 2.5  # pitch the Navdy v180 vehicle sprites were rendered at


def vehicle_yaw(model, lane, distance, lateral):
  """Sprite yaw that makes a vehicle's body follow its lane in the Navdy road drawing.

  The HUD road (display.navdy_project_point) squeezes distance far more than width. Step one metre along the lane
  and read the screen motion (dx sideways, dy up). A sprite rendered at a fixed pitch p with yaw y has its long axis
  at tan(tilt) = tan(y) / sin(p) on screen, so matching the lane's tilt dx/dy gives y = atan(sin(p) * dx / dy):
  adjacent lanes come out ~3-4 degrees at every distance. Payload sign: negative for a left-lane vehicle, whose
  front leans right on screen.
  """
  heading = lane_heading(model, lane, distance)
  step = 1.0 if camera_x(distance) + 1.0 <= MAX_DISTANCE else -1.0
  x0 = camera_x(distance)
  start = navdy_project_point(x0, lateral, drawing_path_y_at(model, x0))
  end = navdy_project_point(x0 + step, lateral + math.tan(heading) * step, drawing_path_y_at(model, x0 + step))
  if not start or not end:
    return 0.0
  dx, dy = (end[0] - start[0]) / step, (start[1] - end[1]) / step
  if dy <= 0.0:
    return 0.0
  yaw = math.degrees(math.atan(math.sin(math.radians(SPRITE_PITCH_DEG)) * dx / dy))
  return rounded(max(-24.0, min(24.0, -yaw)))


class VehicleOverlay:
  def __init__(self):
    self.tracks = {}
    self.lanes = {}  # ident -> (lane, radar time it was last given)
    self.timestamp = 0.0
    self.flipped = None
    self.display_data = None
    self.alignment = None

  def clear(self):
    self.tracks.clear()
    self.lanes.clear()
    self.timestamp = 0.0
    self.flipped = None
    self.display_data = None
    self.alignment = None

  def update(self, payload, model, data, *, timestamp, now, radar_alignment=None):
    """radar_alignment (car.radar_display_alignment.RadarDisplayAlignment or None): radar lateral -> camera frame.

    Every confirmed radar point is its own entry. A truck's side can return several points; they are not merged
    here, since radar alone does not tell one long body from a dense queue (Codex review 2026-11-13). With the
    camera's truck, sprite_overlap drops the unconfirmed points inside its body at the end."""
    errors = getattr(data, 'errors', None)
    if (not (payload['active'] or payload.get('roadPreview')) or not payload.get('navPathLeft') or data is None or errors is None
        or any(bool(getattr(errors, name, False)) for name in
               ('canError', 'radarFault', 'wrongConfig', 'radarUnavailableTemporary'))
        or not math.isfinite(timestamp) or not math.isfinite(now) or timestamp <= 0
        or not 0 <= now - timestamp <= TRACK_MAX_AGE):
      self.clear()
      return
    flipped = bool(getattr(data, 'radarTrackFlipped', False))
    display_data = bool(getattr(data, 'hasDisplayData', False))
    alignment = None if radar_alignment is None else radar_alignment.key()
    if (timestamp < self.timestamp or flipped != self.flipped or display_data != self.display_data
        or alignment != self.alignment):
      self.clear()
    self.flipped = flipped
    self.display_data = display_data
    self.alignment = alignment
    self.tracks = {key: value for key, value in self.tracks.items() if now - value['time'] <= TRACK_MAX_AGE}
    if timestamp > self.timestamp:
      self.timestamp = timestamp
      current = {}
      # An advertised display stream is authoritative even when empty.
      points = getattr(data, 'displayPoints', ()) if display_data else getattr(data, 'points', ())
      for point in islice(points, MAX_TRACKS):
        ident = finite_float(getattr(point, 'trackId', None), math.nan)
        distance = finite_float(getattr(point, 'dRel', None), math.nan)
        lateral = finite_float(getattr(point, 'yRel', None), math.nan)
        speed = finite_float(getattr(point, 'vRel', None), math.nan)
        if (not math.isfinite(ident) or not 0 <= ident <= 2**53 - 1 or ident != int(ident)
            or not 2.0 <= distance <= MAX_DISTANCE or not abs(lateral) <= 15.0 or not abs(speed) <= 100.0):
          continue
        ident = int(ident)
        if ident in current:
          continue
        previous = self.tracks.get(ident)
        # Published liveTracks already includes RadarTrackFlip. Convert only
        # radar-left-positive to model-right-positive; never flip a second time.
        lateral = -lateral
        if radar_alignment is not None:
          lateral = radar_alignment.right(distance, lateral)
        values = {'dRel': distance, 'yRel': lateral, 'vRel': speed}
        if (previous and timestamp - previous['time'] <= TRACK_MAX_AGE
            and abs(previous['dRel'] - distance) <= max(8.0, distance * 0.35)
            and abs(previous['yRel'] - lateral) <= 2.5):
          values = {key: previous[key] + 0.55 * (value - previous[key]) for key, value in values.items()}
          samples = min(2, previous['samples'] + 1)
        else:
          samples = 1
        current[ident] = dict(values, time=timestamp, samples=samples)
      # A fresh snapshot removes unconfirmed targets at once; a confirmed one is held until TRACK_HOLD after its
      # last sighting (its time is not refreshed, so the hold never extends itself).
      for ident, track in self.tracks.items():
        if ident not in current and track['samples'] >= 2 and timestamp - track['time'] <= TRACK_HOLD:
          current[ident] = track
      self.tracks = current

    leads = payload['navVehicles']
    candidates = []  # (ident, distance, lateral, lane, vRel) of confirmed points in a lane
    for ident, track in self.tracks.items():
      if track['samples'] < 2:
        continue
      lateral = track['yRel']
      distance = track['dRel'] + track['vRel'] * (self.timestamp - track['time'])  # held: coast to this snapshot
      if not 2.0 <= distance <= MAX_DISTANCE:
        continue
      lane = vehicle_lane(model, distance, lateral)
      if lane:
        self.lanes[ident] = (lane, self.timestamp)
      elif ident in self.lanes and self.timestamp - self.lanes[ident][1] <= LANE_HOLD:
        lane = self.lanes[ident][0]
      if lane:
        candidates.append((ident, distance, lateral, lane, track['vRel']))
    self.lanes = {ident: value for ident, value in self.lanes.items() if ident in self.tracks}

    def same_vehicle(lead, candidate):
      # Display streams may assign independent IDs; use tight geometric matching there, never ID equality across
      # namespaces. A camera-only lead (its range is the model's) is matched loosely.
      _, distance, lateral, lane, speed = candidate
      return (lead['lane'] == lane
              and abs(lead['distanceM'] - distance) <= (max(2.5, distance * 0.06) if display_data else max(5.0, distance * 0.25))
              and abs(lead['lateralM'] - lateral) <= (1.0 if display_data else 1.5)
              and abs(lead['relativeSpeedMps'] - speed) <= (3.0 if display_data else 5.0))

    # A camera-only published lead is drawn where its radar point is. radard flips a far lead between its radar point
    # and the model's own range several times a second (11/15: 56 m vs 47 m, every 0.2-0.4 s), which made the icon
    # blink and jump (user report 2026-11-16); the radar range is the one the stock camera agrees with.
    adopted, moved = set(), set()
    if not display_data:
      pairs = sorted((abs(c[1] - lead['distanceM']) / max(5.0, c[1] * 0.25) + abs(c[2] - lead['lateralM']) / 1.5,
                      lead.get('colorRole', 0) or 3, index, c)
                     for index, lead in enumerate(leads) if lead['source'] == 'vision'
                     for c in candidates if same_vehicle(lead, c))
      for _, _, index, (ident, distance, lateral, lane, speed) in pairs:   # closest pairs first, lead one on a tie
        if index in moved or ident in adopted:
          continue
        view = navdy_vehicle_view(model, distance, lateral)
        if view is None:
          continue
        moved.add(index)
        adopted.add(ident)
        leads[index].update({'trackId': ident, 'screenX': view['screenX'], 'screenY': view['screenY'],
                             'pxPerM': view['pxPerM'], 'distanceM': rounded(distance), 'relativeSpeedMps': rounded(speed),
                             'lateralM': rounded(lateral, 2), 'source': 'radar'})

    surrounding = []
    for candidate in candidates:
      ident, distance, lateral, lane, speed = candidate
      # Preserve actual lead positions/highlights; raw points only fill in additional vehicles.
      if ident in adopted or any((not display_data and lead['trackId'] == ident)
                                 or ((display_data or lead['source'] == 'vision') and same_vehicle(lead, candidate))
                                 for lead in leads):
        continue
      view = navdy_vehicle_view(model, distance, lateral)
      if view is None:
        continue
      surrounding.append({
        'trackId': ident, 'screenX': view['screenX'], 'screenY': view['screenY'], 'pxPerM': view['pxPerM'],
        'yawDeg': vehicle_yaw(model, lane, distance, lateral), 'distanceM': rounded(distance),
        'relativeSpeedMps': rounded(speed), 'lateralM': rounded(lateral, 2),
        'lane': lane, 'source': 'radar', 'longitudinalLead': False, 'confidence': 0.65, 'colorRole': 0,
      })
    surrounding.sort(key=lambda vehicle: vehicle['distanceM'])
    selected = leads + surrounding[:max(0, MAX_VEHICLES - len(leads))]
    payload['navVehicles'] = sorted(selected, key=lambda vehicle: vehicle['distanceM'], reverse=True)


CAMERA_GATE_DISTANCE = 5.0  # metres, same gates as the phone live view
CAMERA_GATE_LATERAL = 1.8
CAMERA_TRACK_ID_BASE = 900000
# The stock camera puts a truck ~10% farther than the radar does (11/14, camera lead vs radar lead, every 10th frame:
# trucks median 1.102, 5-95% 1.04-1.13 over 116 frames; cars 1.015). Beyond ~30 m nearly every lead truck missed the
# 5 m gate, so the radar point stayed a car and the camera truck was drawn again ~6 m behind it (user report
# 2026-11-15: a truck and a car icon on top of each other). Matching therefore runs twice: first on the camera's own
# positions exactly as before; then a truck still unmatched is tried at its position scaled back by this, and only
# lends its kind when exactly one unmatched entry of the same lane is within the gates there. Scaling every truck up
# front picked a nearer car over the truck it had matched (Codex review 2026-11-15, N4); a wider distance gate merged
# another lane on a curve (N5). Camera-only entries keep the camera's own position.
CAMERA_TRUCK_RANGE_SCALE = 1.10


def _camera_pairs(vehicles, objects, model, scale):
  """(score, vehicle index, object index) within the gates, objects' positions divided by scale (trucks only)."""
  pairs = []
  for o_index, obj in objects:
    distance, lateral = obj['distance'] / scale, obj['lateral'] / scale
    lane = vehicle_lane(model, distance, lateral) if scale != 1.0 else None
    if lane == '':
      continue                       # a scaled position outside the lanes vouches for nothing
    for v_index, vehicle in vehicles:
      dd, dl = abs(vehicle['distanceM'] - distance), abs(vehicle['lateralM'] - lateral)
      if dd < CAMERA_GATE_DISTANCE and dl < CAMERA_GATE_LATERAL and (lane is None or lane == vehicle.get('lane')):
        pairs.append((dd / CAMERA_GATE_DISTANCE + dl / CAMERA_GATE_LATERAL, v_index, o_index))
  return pairs


def _camera_vehicle(model, obj):
  distance, lateral = obj['distance'], obj['lateral']
  if not 2.0 <= distance <= MAX_DISTANCE or not abs(lateral) <= 15.0:
    return None
  lane = vehicle_lane(model, distance, lateral)
  if not lane:
    return None
  view = navdy_vehicle_view(model, distance, lateral)
  if view is None:
    return None
  return {
    'trackId': CAMERA_TRACK_ID_BASE + int(obj['id']), 'screenX': view['screenX'], 'screenY': view['screenY'],
    'pxPerM': view['pxPerM'], 'yawDeg': vehicle_yaw(model, lane, distance, lateral), 'distanceM': rounded(distance),
    'relativeSpeedMps': 0.0, 'lateralM': rounded(lateral, 2), 'lane': lane, 'source': 'vision',
    'longitudinalLead': False, 'confidence': 0.6, 'kind': obj['kind'], 'colorRole': 0,
  }


def annotate_camera(payload, model, objects, lamps):
  """Give navVehicles the stock camera's kind and lead lamps, and add camera-only objects.

  Display only. Radar/lead entries keep their positions; a camera object within 5 m / 1.8 m lends
  its kind (an unmatched truck is tried once more on the radar's range, see CAMERA_TRUCK_RANGE_SCALE).
  Track 1's lamps go to whichever entry carries track 1. Camera objects nobody covers (pedestrians,
  motorcycles, vehicles radar missed) are added like the phone live view does.
  """
  vehicles = payload.get('navVehicles')
  if not isinstance(vehicles, list):
    return
  for vehicle in vehicles:
    if 'yawDeg' not in vehicle:
      vehicle['yawDeg'] = vehicle_yaw(model, vehicle.get('lane', ''), vehicle['distanceM'], vehicle['lateralM'])
  matched_vehicles, matched_objects, track1 = set(), set(), None
  track1_id = next((obj['id'] for obj in objects if obj.get('track1')), None)

  def match(v_index, o_index):
    nonlocal track1
    matched_vehicles.add(v_index)
    matched_objects.add(o_index)
    vehicles[v_index]['kind'] = objects[o_index]['kind']   # a radar entry with 'kind' is camera-confirmed (sprite_overlap)
    if objects[o_index].get('track1'):
      track1 = vehicles[v_index]

  for _, v_index, o_index in sorted(_camera_pairs(list(enumerate(vehicles)), list(enumerate(objects)), model, 1.0)):
    if v_index not in matched_vehicles and o_index not in matched_objects:
      match(v_index, o_index)
  trucks = [(o_index, obj) for o_index, obj in enumerate(objects) if o_index not in matched_objects and obj.get('kind') == 'truck']
  free = [(v_index, vehicle) for v_index, vehicle in enumerate(vehicles) if v_index not in matched_vehicles]
  scaled = _camera_pairs(free, trucks, model, CAMERA_TRUCK_RANGE_SCALE)
  for _, v_index, o_index in sorted(scaled):
    candidates = {v for _, v, o in scaled if o == o_index}
    claimants = {o for _, v, o in scaled if v == v_index}
    if len(candidates) == 1 and len(claimants) == 1 and v_index not in matched_vehicles and o_index not in matched_objects:
      match(v_index, o_index)
  extra = []
  if (payload.get('active') or payload.get('roadPreview')) and payload.get('navPathLeft'):
    for o_index, obj in enumerate(objects):
      if o_index in matched_objects:
        continue
      vehicle = _camera_vehicle(model, obj)
      if vehicle is not None:
        extra.append(vehicle)
        if obj.get('track1'):
          track1 = vehicle
  if lamps is not None and track1 is not None and lamps.get('id') == track1_id:
    track1['brakeLight'] = bool(lamps['brake'])
    track1['turnSignal'] = lamps['turn']
  extra.sort(key=lambda vehicle: vehicle['distanceM'])
  selected = vehicles + extra[:max(0, MAX_VEHICLES - len(vehicles))]
  payload['navVehicles'] = sorted(selected, key=lambda vehicle: vehicle['distanceM'], reverse=True)
