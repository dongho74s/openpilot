"""Pure Carrot -> navdy.openpilot.v1 display mapping.

Projection and display conventions adapted from Wayon navdy_op_bridge.py at
b7660d1d403938c464d4f14f842449e3f0e64ebd (helpers, geometry and payload).
No transport, Params, clock, SP services, raw CAN or control writes. The caller
passes None for stale/invalid services and adds any transport timestamp.
All speed fields ending in Kph are km/h regardless of the vehicle's UI units.
"""
from __future__ import annotations

from itertools import islice
import math
from typing import Any

from openpilot.selfdrive.car.outside_air_temp import outside_air_temp_c
from openpilot.selfdrive.car.stock_lane_markings import stock_lane_codes
from openpilot.selfdrive.navdy.deceleration import badge_code

KPH_PER_MS = 3.6
MAX_MODEL_SAMPLES = 64
MAX_PROJECTED_POINTS = 10
MAX_DISTANCE_M = 80.0


def finite_float(value: Any, default: float = 0.0) -> float:
  try:
    number = float(value)
  except (TypeError, ValueError, OverflowError):
    return default
  return number if math.isfinite(number) else default


def rounded(value: Any, digits: int = 1) -> float:
  return round(finite_float(value), digits)


def enum_text(value: Any) -> str:
  return str(value).split(".")[-1]


def _items(value: Any, count: int) -> list:
  return list(islice(value, count)) if value is not None else []


def _pair_text(left: bool, right: bool, both: str) -> str:
  return both if left and right else "left" if left else "right" if right else "off"


def resolve_operating_state(*, connected: bool, onroad: bool, door_open: bool = False,
                            gear: str = "unknown") -> str:
  if not connected:
    return "disconnected"
  if gear == "reverse":
    return "reverse"
  if onroad:
    return "onroad_door" if door_open else "onroad"
  return "offroad_door" if door_open else "offroad"


def _speed_kph(holder: Any, names: tuple[str, ...], scale: float = 1.0) -> float:
  for name in names:
    speed = finite_float(getattr(holder, name, 0.0)) * scale
    if 0.0 < speed < 255.0:
      return speed
  return 0.0


def set_speed_kph(car_state: Any, car_control: Any = None) -> float:
  # card.py publishes vCruise[Cluster] in km/h; CruiseState/HUDControl use m/s.
  return (_speed_kph(car_state, ("vCruiseCluster", "vCruise"))
          or _speed_kph(getattr(car_state, "cruiseState", None), ("speedCluster", "speed"), KPH_PER_MS)
          or _speed_kph(getattr(car_control, "hudControl", None), ("setSpeed",), KPH_PER_MS))


ORIGIN_JITTER_M = 0.001


def _model_points(x_values: Any, y_values: Any) -> list[tuple[float, float]]:
  points = []
  for raw_x, raw_y in zip(_items(x_values, MAX_MODEL_SAMPLES), _items(y_values, MAX_MODEL_SAMPLES), strict=False):
    x, y = finite_float(raw_x, math.nan), finite_float(raw_y, math.nan)
    if not points and -ORIGIN_JITTER_M <= x < 0.0:
      # The first sample is the car itself. Some models (the RDF internal model, 2026-11-15 drive) give it as
      # floating-point noise around 0 (about +-1e-10 m), negative in a third to two thirds of frames; rejecting it
      # emptied the whole line, so the HUD dropped the road, lanes and vehicles every few frames (seen as flicker).
      x = 0.0
    # Preserve sub-millimeter negative origin jitter at standstill, not gaps
    # in an established forward trajectory or invalid/nonfinite samples.
    if points and -ORIGIN_JITTER_M <= x < 0.0 and points[-1][0] <= ORIGIN_JITTER_M and abs(y - points[-1][1]) <= ORIGIN_JITTER_M:
      continue
    if not (math.isfinite(x) and math.isfinite(y) and 0.0 <= x <= 10000.0 and abs(y) <= 100.0):
      break
    if points and x <= points[-1][0]:
      # A stopping/turning trajectory can fold back at its far end. Keep
      # only this frame's unambiguous forward prefix, never sort the tail.
      break
    points.append((x, y))
  return points if len(points) >= 2 else []


STOPPED_PLAN_BACK_M = 1.0


def _stopped_plan(x_values: Any, y_values: Any) -> bool:
  """True for a plan that is the car standing still: it starts at the car (x and y within ORIGIN_JITTER_M of 0), every
  sample is finite and in the usual bounds, and it goes at most STOPPED_PLAN_BACK_M behind the car. Stopped RDF plans
  of 2026-11-15 start at +-5e-11 m and wander at most 0.22 m back (7313 frames, all below 0.21 m/s); a NaN sample, a
  start off the car or a plan reaching metres behind it is a broken plan, not a stopped one (Codex review)."""
  xs, ys = _items(x_values, MAX_MODEL_SAMPLES), _items(y_values, MAX_MODEL_SAMPLES)
  if len(xs) < 2 or len(xs) != len(ys):
    return False
  xs = [finite_float(x, math.nan) for x in xs]
  ys = [finite_float(y, math.nan) for y in ys]
  if not all(math.isfinite(v) for v in xs + ys):
    return False
  return (abs(xs[0]) <= ORIGIN_JITTER_M and abs(ys[0]) <= ORIGIN_JITTER_M and min(xs) >= -STOPPED_PLAN_BACK_M
          and max(xs) <= 10000.0 and max(abs(y) for y in ys) <= 100.0)


def model_line_y_at_values(x_values: Any, y_values: Any, distance_m: float) -> float | None:
  points = _model_points(x_values, y_values)
  if len(points) < 2 or not points[0][0] <= distance_m <= points[-1][0]:
    return None
  for (x0, y0), (x1, y1) in zip(points, points[1:], strict=False):
    if x0 <= distance_m <= x1:
      return y0 + (y1 - y0) * (distance_m - x0) / (x1 - x0)
  return None


def model_line_y_at(line: Any, distance_m: float) -> float | None:
  return model_line_y_at_values(getattr(line, "x", None), getattr(line, "y", None), distance_m)


# Navdy road view: the road drawing of HUD v180-v182 (from Wayon's bridge), restored on 2026-11-15 at the user's
# request after the camera-perspective view (v183, 2026-11-13) drew near vehicles too large and slightly farther ones
# too small. Distance is compressed to a power curve, d = (x / 80 m) ** 0.65, drawn from the bottom row (y = 96) up to
# y = 8, and one metre sideways is 44 px at the bottom shrinking to 8 px at 80 m. Sideways positions are drawn
# relative to the planned path (which itself gets at least 17 px/m so its curve stays visible), so lanes, path and
# vehicles all use this one mapping.
#
# Vehicles stay accurate inside it: each is drawn at its own rear ground point in this mapping (the radar distance
# moved to the camera origin, the radar lateral moved into the camera frame) and at the drawing's own sideways scale
# there (pxPerM), so a car keeps its true size relative to the lanes drawn at its row.
VIEW_CENTER_X = 160.0
VIEW_BOTTOM_Y = 96.0
VIEW_HEIGHT_PX = 88.0
DISTANCE_GAMMA = 0.65
NEAR_SCALE_PX_M = 44.0
FAR_SCALE_PX_M = 8.0
PATH_MIN_SCALE_PX_M = 17.0
VEHICLE_SIZE = 1.0                # sprite size relative to the road drawn at the vehicle (1 = true size)
RADAR_TO_CAMERA_M = 1.52          # radarState/liveTracks dRel is from the radar; the model's origin is the camera
HIDDEN_BAND = (VIEW_CENTER_X, 1000.0, VIEW_CENTER_X, 1000.0)  # a valid, zero-area path far below the HUD's display


def _depth_fraction(x: float) -> float:
  return (x / MAX_DISTANCE_M) ** DISTANCE_GAMMA


def lateral_scale(x: float) -> float:
  """Sideways px per metre of the road drawing at camera distance x (0..MAX_DISTANCE_M)."""
  fraction = _depth_fraction(max(0.0, min(MAX_DISTANCE_M, x)))
  return NEAR_SCALE_PX_M * (1.0 - fraction) + FAR_SCALE_PX_M * fraction


def navdy_project_point(x_value: Any, y_value: Any, path_y_value: Any = None) -> list[float]:
  """One road point in view px, or [] when invalid or beyond the drawing (unchanged from HUD v180-v182)."""
  x, y = finite_float(x_value, math.nan), finite_float(y_value, math.nan)
  if not 0.0 <= x <= MAX_DISTANCE_M or not math.isfinite(y) or abs(y) > 100.0:
    return []
  fraction = _depth_fraction(x)
  scale = NEAR_SCALE_PX_M * (1.0 - fraction) + FAR_SCALE_PX_M * fraction
  if path_y_value is None:
    screen_x = VIEW_CENTER_X + y * scale
  else:
    path_y = finite_float(path_y_value, math.nan)
    if not math.isfinite(path_y) or abs(path_y) > 100.0:
      return []
    screen_x = VIEW_CENTER_X + path_y * max(scale, PATH_MIN_SCALE_PX_M) + (y - path_y) * scale
  return [rounded(screen_x), rounded(VIEW_BOTTOM_Y - VIEW_HEIGHT_PX * fraction)]


def drawing_path_y_at_values(x_values: Any, y_values: Any, distance_m: float) -> float | None:
  """Path lateral for the drawing's sideways correction at distance_m; None without a path.

  Inside the plan it is the path. Beyond its last sample it holds the last value: a plan shorter than the lanes
  (below ~30 km/h it ends before 80 m) otherwise dropped the correction right past its end, moving a vehicle or lane
  point there by up to ~17 px between neighbouring distances (Codex review 2026-11-15)."""
  points = _model_points(x_values, y_values)
  if not points or not math.isfinite(distance_m):
    return None
  if distance_m >= points[-1][0]:
    return points[-1][1]
  if distance_m <= points[0][0]:
    return points[0][1]
  return model_line_y_at_values(x_values, y_values, distance_m)


def drawing_path_y_at(model: Any, distance_m: float) -> float | None:
  position = getattr(model, "position", None)
  return drawing_path_y_at_values(getattr(position, "x", None), getattr(position, "y", None), distance_m)


def camera_x(distance: float) -> float:
  """radarState/liveTracks dRel (from the radar) -> distance from the camera, the model's origin."""
  return distance + RADAR_TO_CAMERA_M


def navdy_vehicle_view(model: Any, distance: float, lateral: float) -> dict[str, float] | None:
  """Rear ground point (screenX/Y) and HUD sprite scale (pxPerM) of a radar-referenced vehicle.

  Projected like the lanes at the same camera distance (same path correction, held past the plan's end as the lanes'
  is), so its position against the drawn lanes is the measured one. None when invalid or beyond 80 m."""
  if not math.isfinite(distance) or not math.isfinite(lateral):
    return None
  x = camera_x(distance)
  point = navdy_project_point(x, lateral, drawing_path_y_at(model, x))
  if not point:
    return None
  return {"screenX": point[0], "screenY": point[1], "pxPerM": rounded(lateral_scale(x) * VEHICLE_SIZE, 2)}


def navdy_model_line(x_values: Any, y_values: Any, lateral_offset: float = 0.0,
                     path_x_values: Any = None, path_y_values: Any = None) -> list[float]:
  """A model line in view px (at most MAX_PROJECTED_POINTS evenly picked samples), drawn relative to the path."""
  points = [(x, y) for x, y in _model_points(x_values, y_values) if x <= MAX_DISTANCE_M]
  if len(points) > MAX_PROJECTED_POINTS:
    last = len(points) - 1
    points = [points[round(i * last / (MAX_PROJECTED_POINTS - 1))] for i in range(MAX_PROJECTED_POINTS)]
  projected = []
  for x, y in points:
    path_y = drawing_path_y_at_values(path_x_values, path_y_values, x)
    projected.extend(navdy_project_point(x, y + lateral_offset, path_y))
  return projected


def _probability(values: list, index: int) -> float:
  return max(0.0, min(1.0, finite_float(values[index]))) if index < len(values) else 0.0


def _lane_type(code: Any, probability: Any) -> str:
  # Hyundai publishes color (0..3) * 10 + type (0..8); Xiaoge replaces
  # only the type with 0/1. Reject reserved codes instead of coercing them.
  if isinstance(code, bool) or not isinstance(code, int) or not 0 <= code <= 38 or code % 10 > 8:
    return "unknown"
  if isinstance(probability, bool) or not isinstance(probability, (int, float)) or not 0.3 < probability <= 1.0:
    return "unknown"
  # Match C4, including double/inner markings. The existing Navdy renderer
  # treats unknown as dashed; do not infer a style from geometry alone.
  return "dashed" if code % 10 == 0 else "solid"


def navdy_model_geometry(model: Any) -> dict[str, Any]:
  position = getattr(model, "position", None)
  lanes = _items(getattr(model, "laneLines", None), 4)
  if position is None or len(lanes) < 3:
    return {}
  path_x, path_y = getattr(position, "x", None), getattr(position, "y", None)

  def line_points(line: Any) -> list[float]:
    return navdy_model_line(getattr(line, "x", None), getattr(line, "y", None), 0.0, path_x, path_y)

  path_left = navdy_model_line(path_x, path_y, -1.0, path_x, path_y)
  path_right = navdy_model_line(path_x, path_y, 1.0, path_x, path_y)
  if not (len(path_left) >= 4 and len(path_right) >= 4) and (_model_points(path_x, path_y) or _stopped_plan(path_x, path_y)):
    # A plan with fewer than two drawable samples (stopped, creeping): keep lanes and vehicles drawn (the HUD draws
    # the road only with a path) with a zero-area band off the view. A stopped RDF plan starts at the car and then
    # wanders millimetres to centimetres behind it, which has no forward line at all; without _stopped_plan such
    # frames dropped the whole road at a standstill while neighbouring frames drew it (2026-11-15 drive).
    path_left, path_right = list(HIDDEN_BAND), list(HIDDEN_BAND)
  geometry = {
    "navPathLeft": path_left,
    "navPathRight": path_right,
    "navLaneLeft": line_points(lanes[1]),
    "navLaneRight": line_points(lanes[2]),
  }
  if not all(len(points) >= 4 for points in geometry.values()):
    return {}
  probs = _items(getattr(model, "laneLineProbs", None), 4)
  for index, side in enumerate(("FarLeft", "Left", "Right", "FarRight")):
    geometry[f"navLane{side}Prob"] = rounded(_probability(probs, index), 2)
  if len(lanes) >= 4:
    geometry["navLaneFarLeft"] = line_points(lanes[0])
    geometry["navLaneFarRight"] = line_points(lanes[3])
  edges = _items(getattr(model, "roadEdges", None), 2)
  stds = _items(getattr(model, "roadEdgeStds", None), 2)
  for index, side in enumerate(("Left", "Right")):
    if index >= len(edges) or index >= len(stds):
      continue
    std = finite_float(stds[index], math.inf)
    confidence = max(0.0, min(1.0, 1.0 - std)) if std >= 0.0 else 0.0
    points = line_points(edges[index])
    if confidence >= 0.5 and len(points) >= 4:
      geometry[f"navRoadEdge{side}"] = points
      geometry[f"navRoadEdge{side}Prob"] = rounded(confidence, 2)
  return geometry


def _lead_lane(model: Any, distance: float, y: float) -> str:
  """Lane of a radar-referenced vehicle, read from the model lanes at its camera distance."""
  lanes = _items(getattr(model, "laneLines", None), 4)
  probs = _items(getattr(model, "laneLineProbs", None), 4)
  boundaries = [model_line_y_at(line, camera_x(distance)) for line in lanes]
  for left, right, lane, threshold in ((1, 2, "center", 0.3), (0, 1, "left", 0.2), (2, 3, "right", 0.2)):
    if right >= len(boundaries) or min(_probability(probs, left), _probability(probs, right)) < threshold:
      continue
    low, high = boundaries[left], boundaries[right]
    if low is not None and high is not None and low < high and low <= y <= high:
      return lane
  return ""


def navdy_lead_geometry(model: Any, radar_state: Any, longitudinal_plan: Any = None,
                        radar_alignment: Any = None) -> dict[str, Any]:
  """Only the two published, visible leads, never inferred raw radar tracks.

  radar_alignment (car.radar_display_alignment.RadarDisplayAlignment or None) moves a radar-measured lead's
  lateral into the camera frame of the drawn lanes, for display only; vision leads already are in it.
  """
  vehicles = []
  source = enum_text(getattr(longitudinal_plan, "longitudinalPlanSource", ""))
  for index, name in enumerate(("leadOne", "leadTwo")):
    lead = getattr(radar_state, name, None)
    if not bool(getattr(lead, "status", False)):
      continue
    distance = finite_float(getattr(lead, "dRel", None), math.nan)
    lateral = -finite_float(getattr(lead, "yRel", None), math.nan)
    relative_speed = finite_float(getattr(lead, "vRel", None), math.nan)
    if not 2.0 <= distance <= MAX_DISTANCE_M or not math.isfinite(relative_speed):
      continue
    if radar_alignment is not None and bool(getattr(lead, "radar", False)):
      lateral = radar_alignment.right(distance, lateral)
    lane = _lead_lane(model, distance, lateral)
    if not lane:
      continue
    view = navdy_vehicle_view(model, distance, lateral)
    if view is None:
      continue
    track_id = int(finite_float(getattr(lead, "radarTrackId", -1), -1))
    if track_id >= 0 and any(vehicle["trackId"] == track_id for vehicle in vehicles):
      continue
    vehicle = {
      "trackId": track_id,
      "screenX": view["screenX"],
      "screenY": view["screenY"],
      "pxPerM": view["pxPerM"],
      "distanceM": rounded(distance),
      "relativeSpeedMps": rounded(relative_speed),
      "lateralM": rounded(lateral, 2),
      "lane": lane,
      "source": "radar" if bool(getattr(lead, "radar", False)) else "vision",
      "longitudinalLead": source == f"lead{index}",
      "confidence": rounded(max(0.0, min(1.0, finite_float(getattr(lead, "modelProb", 0.0)))), 2),
      "colorRole": index + 1,   # HUD v185 colours: lead one sky blue, lead two pale sky blue, everything else grey
    }
    vehicles.append(vehicle)
  return {"navVehicles": sorted(vehicles, key=lambda vehicle: vehicle["distanceM"], reverse=True)}


def _longitudinal_actuator(active: bool, car_control: Any, car_output: Any) -> tuple[str, float]:
  # The GM-specific raw-brake/350 normalization is not portable across Carrot
  # ports. Use the returned SI acceleration, never the requested carControl one.
  if not active or not bool(getattr(car_control, "longActive", False)):
    return "none", 0.0
  accel = finite_float(getattr(getattr(car_output, "actuatorsOutput", None), "accel", None), math.nan)
  if not math.isfinite(accel):
    return "none", 0.0
  if accel > 0.05:
    return "accelerator", min(1.0, accel)
  if accel < -0.05:
    return "brake", min(1.0, -accel)
  return "coast", 0.0


def fuel_percent(car_state: Any) -> float | None:
  """Tank level 0..100 from carState.vehicleCare (Traverse ECM 0x4D1), or None when not valid."""
  care = getattr(car_state, "vehicleCare", None)
  if care is None or not bool(getattr(care, "fuelLevelValid", False)):
    return None
  level = finite_float(getattr(care, "fuelLevel", None), math.nan)
  if not 0.0 <= level <= 1.05:
    return None
  return rounded(min(level, 1.0) * 100.0, 1)


def payload_from_messages(*, car_state: Any = None, selfdrive_state: Any = None,
                          controls_state: Any = None, car_control: Any = None,
                          car_output: Any = None, model: Any = None,
                          radar_state: Any = None, longitudinal_plan: Any = None,
                          carrot_man: Any = None, seq: int = 0, onroad: bool = False,
                          door_open: bool | None = None, radar_alignment: Any = None) -> dict[str, Any]:
  """Map fresh service structs without retaining state or synthesizing alerts.

  Only the display-only deceleration verdict uses carrot_man here; navigation
  feedback is still owned by the bridge separately.
  Alert strings are preserved verbatim, including hold/resume/critical alerts.
  """
  car_valid = (car_state is not None and bool(getattr(car_state, "canValid", False))
               and not bool(getattr(car_state, "canTimeout", False)))
  if onroad:
    # GM speed is signed and its filter can cross zero at rest. Neither is a
    # lost CAN connection; reverse display suppression is handled by gear below.
    car_valid = car_valid and abs(finite_float(getattr(car_state, "vEgo", None), math.inf)) < 100.0
  connected = car_valid and (not onroad or selfdrive_state is not None)
  gear = enum_text(getattr(car_state, "gearShifter", "unknown")).lower() if car_valid else "unknown"
  door = bool(getattr(car_state, "doorOpen", False) if door_open is None else door_open)
  driving = connected and bool(onroad) and gear != "reverse"
  enabled = driving and bool(getattr(selfdrive_state, "enabled", False))
  active = enabled and bool(getattr(selfdrive_state, "active", False))
  engageable = driving and bool(getattr(selfdrive_state, "engageable", False))
  state = enum_text(getattr(selfdrive_state, "state", "unknown")) if driving else "disabled"
  left = driving and bool(getattr(car_state, "leftBlinker", False))
  right = driving and bool(getattr(car_state, "rightBlinker", False))
  left_bsm = driving and bool(getattr(car_state, "leftBlindspot", False))
  right_bsm = driving and bool(getattr(car_state, "rightBlindspot", False))
  soft_hold = int(finite_float(getattr(car_state, "softHoldActive", 0))) if driving else 0
  hold = driving and (bool(getattr(car_state, "brakeHoldActive", False)) or soft_hold == 2)
  standstill = driving and bool(getattr(car_state, "standstill", False))
  stop_icon = driving and (state == "preEnabled" or (standstill and (enabled or hold)))
  speed = _speed_kph(car_state, ("vEgoCluster", "vEgo"), KPH_PER_MS) if driving else 0.0
  physical_acc = _speed_kph(getattr(car_state, "cruiseState", None), ("speedCluster", "speed"), KPH_PER_MS)
  actuator, actuator_level = _longitudinal_actuator(active, car_control, car_output)
  payload = {
    "schema": "navdy.openpilot.v1",
    "seq": max(0, min(2**53 - 1, int(finite_float(seq)))),
    "state": state,
    "enabled": enabled,
    "active": active,
    "engaged": active,
    "disengaged": not enabled,
    "engageable": engageable,
    "opAvailable": engageable,
    "onroad": bool(onroad),
    "doorOpen": door,
    "ambientState": resolve_operating_state(connected=connected, onroad=bool(onroad), door_open=door, gear=gear),
    "gear": gear,
    "standstill": stop_icon,
    "cruiseStandstill": stop_icon,
    "autoHoldActive": hold,
    "softHoldActive": soft_hold,
    "setSpeedKph": rounded(set_speed_kph(car_state, car_control)) if driving else 0.0,
    "actualAccSetKph": rounded(physical_acc) if driving else 0.0,
    "vEgoKph": rounded(speed),
    "speedUnit": "kph",
    "leftBlinker": left,
    "rightBlinker": right,
    "blinkers": _pair_text(left, right, "hazard"),
    "leftBlindspot": left_bsm,
    "rightBlindspot": right_bsm,
    "blindspot": _pair_text(left_bsm, right_bsm, "both"),
    "alertText1": str(getattr(selfdrive_state, "alertText1", "")),
    "alertText2": str(getattr(selfdrive_state, "alertText2", "")),
    "alertType": str(getattr(selfdrive_state, "alertType", "")),
    "alertStatus": enum_text(getattr(selfdrive_state, "alertStatus", "normal")),
    "alertSize": enum_text(getattr(selfdrive_state, "alertSize", "none")),
    "greenLightAlert": False,
    "leadDepartAlert": False,
    "longitudinalActuator": actuator,
    "longitudinalActuatorLevel": rounded(actuator_level, 2),
    "latActive": active and bool(getattr(car_control, "latActive", False)),
    "longActive": active and bool(getattr(car_control, "longActive", False)),
    "activeLaneLine": active and bool(getattr(controls_state, "activeLaneLine", False)),
    "navVehicles": [],
    "decel": 0,
  }
  outside_c = outside_air_temp_c(car_state) if car_valid else None
  if outside_c is not None:
    # Absent key keeps the HUD on its own OBD 0x46 reading.
    payload['outsideTempC'] = rounded(outside_c, 1)
  fuel = fuel_percent(car_state) if car_valid else None
  if fuel is not None:
    # Absent key hides the HUD fuel gauge next to the clock.
    payload['fuelPercent'] = fuel
  if payload['longActive'] and payload['alertSize'] == 'none' and payload['alertStatus'] == 'normal':
    payload['decel'] = badge_code(car_state, selfdrive_state, car_control, longitudinal_plan, carrot_man)
  # Disengaged but onroad (HUD v183): the road preview, lanes and vehicles without the path (user request
  # 2026-11-13). The HUD draws the road only with a path, so the path becomes a zero-area band off the view.
  preview = not active and driving  # fresh, valid inputs, onroad, not reversing: as for every other driving visual
  if active or preview:
    geometry = navdy_model_geometry(model)
    if geometry and preview:
      geometry["navPathLeft"], geometry["navPathRight"] = list(HIDDEN_BAND), list(HIDDEN_BAND)
      payload["roadPreview"] = True
    payload.update(geometry)
    if geometry:
      payload.update(navdy_lead_geometry(model, radar_state, longitudinal_plan, radar_alignment))
      probs = _items(getattr(model, "laneLineProbs", None), 3)
      left_code, right_code = stock_lane_codes(car_state) or (
        getattr(car_state, "leftLaneLine", None), getattr(car_state, "rightLaneLine", None),
      )
      for index, side, code in ((1, "Left", left_code), (2, "Right", right_code)):
        payload[f"navLane{side}Type"] = _lane_type(code, probs[index] if index < len(probs) else None)
  return payload
