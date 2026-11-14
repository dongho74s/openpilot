"""Navdy vehicle sprites that cover each other: draw the one in front (display only).

A truck's side returns several radar points; each was drawn as a car over the truck (user report 2026-11-15). Two
rules, both on the final navVehicles list, both on what the HUD actually draws:
1. A radar point the stock camera did not confirm as an object of its own (no 'kind' from annotate_camera), in the
   same lane as a camera truck, inside its body (behind its rear by at most TRUCK_BODY_M, within TRUCK_HALF_WIDTH_M
   sideways measured from the planned path, so a curve's next lane is not mistaken for it) and drawn at least
   TRUCK_HIDDEN_FRACTION behind the truck's body, is that truck. Being in the body alone is no proof: a separate car
   ahead of a truck on a curve can be drawn beside it, about 90% visible (Codex review 2026-11-15, N1).
2. A vehicle whose drawn body would be mostly hidden behind nearer vehicles' bodies (at least HIDDEN_FRACTION) is left
   out, as it is hidden on the road too. The longitudinal lead is always drawn.
"Hidden" is the alpha-weighted share of the sprite covered by the nearer sprites' alpha (composited), from 32x32 cell
alpha grids of the sprite PNGs. Against the full PNG alpha (bilinear, random overlapping pairs biased to narrow
motorcycles and pedestrians) this is within about +0.05 / -0.08; a 16x16 opaque-cell version was up to +0.25 off
(a 43% hidden motorcycle read as 70%, Codex review 2026-11-15, N2) and a box rule ignored transparent corners.
Sprite boxes follow the HUD's own placement (VehicleSprites.placement with pxPerM) and the v180 frame table and grids
below, from assets/openpilot_vehicles: meta.txt sha256 ead9c7c9b55709a4..., the 100 PNGs (car, truck, moto, ped x
00..24, concatenated) sha256 365ac7f901050fec...; keep them in step. ALPHA_DATA: per frame 32x32 cells split at
integer boundaries (r * h // 32 .. max(+1, (r + 1) * h // 32)), each round(mean alpha * 15), row-major, two cells per
byte (high nibble first), frames in that kind/frame order, zlib then base64.
"""
from __future__ import annotations

import base64
import hashlib
import math
import zlib

from openpilot.selfdrive.navdy.display import camera_x, drawing_path_y_at

TRUCK_BODY_M = 10.0
TRUCK_HALF_WIDTH_M = 1.5
TRUCK_HIDDEN_FRACTION = 0.35
HIDDEN_FRACTION = 0.7
CELLS = 32
KINDS = ('car', 'truck', 'moto', 'ped')

SPRITES = {  # kind: [(width, height, anchor x, anchor y, px per metre)] for payload yaw -24..24 step 2
  'car': [
    (221, 101, 54.9, 99.8, 61.6), (215, 101, 56.4, 99.9, 61.6), (207, 101, 57.1, 100.0, 61.6),
    (199, 101, 57.7, 100.1, 61.6), (192, 101, 58.5, 100.1, 61.6), (184, 101, 59.2, 100.2, 61.6),
    (175, 101, 60.1, 100.2, 61.6), (167, 101, 60.9, 100.3, 61.6), (159, 101, 61.8, 100.3, 61.6),
    (151, 101, 62.7, 100.3, 61.6), (141, 101, 62.6, 100.4, 61.6), (133, 101, 63.6, 100.4, 61.6),
    (127, 100, 63.5, 100.4, 61.6), (133, 101, 69.5, 100.4, 61.6), (141, 101, 78.4, 100.4, 61.6),
    (151, 101, 88.3, 100.3, 61.6), (159, 101, 97.2, 100.3, 61.6), (167, 101, 106.1, 100.3, 61.6),
    (175, 101, 115.0, 100.2, 61.6), (184, 101, 123.8, 100.2, 61.6), (192, 101, 132.6, 100.1, 61.6),
    (199, 101, 141.3, 100.1, 61.6), (207, 101, 150.0, 100.0, 61.6), (215, 101, 158.6, 99.9, 61.6),
    (221, 101, 166.1, 99.8, 61.6),
  ],
  'truck': [
    (234, 166, 53.0, 165.2, 46.2), (225, 166, 53.4, 165.3, 46.2), (216, 166, 53.8, 165.4, 46.2),
    (206, 166, 54.3, 165.5, 46.2), (196, 166, 54.9, 165.5, 46.2), (186, 166, 55.5, 165.6, 46.2),
    (177, 166, 56.1, 165.6, 46.2), (167, 166, 56.8, 165.7, 46.2), (156, 166, 57.5, 165.7, 46.2),
    (147, 166, 58.2, 165.7, 46.2), (136, 166, 59.0, 165.7, 46.2), (126, 166, 58.7, 165.7, 46.2),
    (118, 165, 59.5, 165.7, 46.2), (125, 166, 67.2, 165.7, 46.2), (136, 166, 78.0, 165.7, 46.2),
    (146, 166, 88.7, 165.7, 46.2), (157, 166, 99.4, 165.7, 46.2), (166, 166, 110.1, 165.7, 46.2),
    (177, 166, 120.8, 165.6, 46.2), (187, 166, 131.4, 165.6, 46.2), (196, 166, 142.0, 165.5, 46.2),
    (207, 166, 152.6, 165.5, 46.2), (215, 166, 162.1, 165.4, 46.2), (225, 166, 172.5, 165.3, 46.2),
    (234, 166, 181.9, 165.2, 46.2),
  ],
  'moto': [
    (64, 117, 7.5, 115.6, 61.6), (61, 117, 7.4, 115.6, 61.6), (59, 117, 8.3, 115.6, 61.6),
    (56, 117, 8.3, 115.7, 61.6), (54, 117, 8.3, 115.7, 61.6), (51, 117, 8.3, 115.7, 61.6),
    (48, 117, 8.3, 115.7, 61.6), (46, 117, 9.4, 115.8, 61.6), (45, 117, 10.4, 115.8, 61.6),
    (45, 117, 13.5, 115.8, 61.6), (45, 117, 16.6, 115.8, 61.6), (45, 117, 19.7, 115.8, 61.6),
    (45, 117, 22.8, 115.8, 61.6), (45, 117, 25.8, 115.8, 61.6), (45, 117, 28.9, 115.8, 61.6),
    (45, 117, 32.0, 115.8, 61.6), (44, 117, 34.1, 115.8, 61.6), (46, 117, 37.1, 115.8, 61.6),
    (48, 117, 40.2, 115.7, 61.6), (51, 117, 43.2, 115.7, 61.6), (53, 117, 45.2, 115.7, 61.6),
    (56, 117, 48.2, 115.7, 61.6), (59, 117, 51.2, 115.6, 61.6), (61, 117, 53.1, 115.6, 61.6),
    (64, 117, 57.1, 115.6, 61.6),
  ],
  'ped': [
    (42, 112, 21.0, 110.1, 72.5), (42, 112, 21.0, 110.1, 72.5), (44, 112, 22.0, 110.1, 72.5),
    (44, 112, 22.0, 110.1, 72.5), (44, 112, 22.0, 110.1, 72.5), (44, 112, 22.0, 110.1, 72.5),
    (44, 112, 22.0, 110.1, 72.5), (45, 112, 23.0, 110.1, 72.5), (45, 112, 23.0, 110.1, 72.5),
    (45, 112, 23.0, 110.1, 72.5), (45, 112, 23.0, 110.1, 72.5), (45, 112, 23.0, 110.1, 72.5),
    (45, 112, 23.0, 110.1, 72.5), (45, 112, 23.0, 110.1, 72.5), (45, 112, 23.0, 110.1, 72.5),
    (45, 112, 23.0, 110.1, 72.5), (45, 112, 23.0, 110.1, 72.5), (45, 112, 23.0, 110.1, 72.5),
    (45, 112, 23.0, 110.1, 72.5), (44, 112, 23.0, 110.1, 72.5), (44, 112, 23.0, 110.1, 72.5),
    (44, 112, 23.0, 110.1, 72.5), (43, 112, 22.0, 110.1, 72.5), (43, 112, 22.0, 110.1, 72.5),
    (42, 112, 22.0, 110.1, 72.5),
  ],
}

ALPHA_DATA = (  # see the module docstring
  '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'
  '7UDHfEjzsKo66cd/PO+R9d//BLfSaOs='
)
ALPHA_SHA256 = '6d129210ca17a600ba870b3a8892532cb30c7b446b710f888a791deed569a9e3'  # of the decompressed cells

_alpha_cache: dict = {}


def _alpha() -> dict:
  """kind -> per frame (cell alphas 0..1, row-major; [(row, column, alpha)] of the non-empty cells). Decoded once,
  on first use and not at import, so bad data fails the caller's guarded pass (bridge: every vehicle drawn)."""
  if not _alpha_cache:
    raw = zlib.decompress(base64.b64decode(''.join(ALPHA_DATA)))
    per = CELLS * CELLS // 2
    if len(raw) != len(KINDS) * 25 * per or hashlib.sha256(raw).hexdigest() != ALPHA_SHA256:
      raise ValueError('sprite alpha data changed')
    decoded = {}
    for k, kind in enumerate(KINDS):
      frames = []
      for i in range(25):
        chunk = raw[(k * 25 + i) * per:(k * 25 + i + 1) * per]
        cells = tuple(value / 15.0 for byte in chunk for value in (byte >> 4, byte & 15))
        frames.append((cells, [(n // CELLS, n % CELLS, a) for n, a in enumerate(cells) if a > 0.0]))
      decoded[kind] = frames
    _alpha_cache.update(decoded)
  return _alpha_cache


def frame_index(yaw) -> int:
  """VehicleSprites.frameIndex (Java Math.round: half up)."""
  try:
    value = float(yaw)
  except (TypeError, ValueError):
    value = 0.0
  clamped = max(-24.0, min(24.0, value)) if math.isfinite(value) else 0.0   # Java: NaN -> 0, before clamping
  return max(0, min(24, math.floor((clamped + 24.0) / 2.0 + 0.5)))


def _placement(vehicle: dict):
  try:
    x, y, ppm_view = float(vehicle['screenX']), float(vehicle['screenY']), float(vehicle['pxPerM'])
  except (KeyError, TypeError, ValueError):
    return None
  if not (math.isfinite(x) and math.isfinite(y) and 0.0 < ppm_view < 1000.0):
    return None
  kind = vehicle.get('kind', 'car')
  kind = kind if kind in SPRITES else 'car'
  index = frame_index(vehicle.get('yawDeg', 0.0))
  width, height, anchor_x, anchor_y, ppm = SPRITES[kind][index]
  scale = ppm_view / ppm
  cells, filled = _alpha()[kind][index]
  return x - anchor_x * scale, y - anchor_y * scale, width * scale, height * scale, cells, filled


def sprite_box(vehicle: dict) -> tuple[float, float, float, float] | None:
  """(left, top, right, bottom) of the vehicle's sprite in view px, as the HUD places it with pxPerM."""
  placed = _placement(vehicle)
  if placed is None:
    return None
  left, top, width, height = placed[:4]
  return left, top, left + width, top + height


def _covered_by_boxes(box, other) -> float:
  """Share of box inside other (tests: what a box rule would have said)."""
  width = min(box[2], other[2]) - max(box[0], other[0])
  height = min(box[3], other[3]) - max(box[1], other[1])
  area = (box[2] - box[0]) * (box[3] - box[1])
  return width * height / area if width > 0.0 and height > 0.0 and area > 0.0 else 0.0


def _alpha_at(placed, x: float, y: float) -> float:
  left, top, width, height, cells, _ = placed
  row, column = math.floor((y - top) / height * CELLS), math.floor((x - left) / width * CELLS)
  return cells[row * CELLS + column] if 0 <= row < CELLS and 0 <= column < CELLS else 0.0


def hidden_fraction(placed, nearer: list) -> float:
  """Alpha-weighted share of a sprite under the nearer sprites (their alphas composited)."""
  left, top, width, height, _, filled = placed
  nearer = [n for n in nearer if n[0] < left + width and left < n[0] + n[2] and n[1] < top + height and top < n[1] + n[3]]
  if not nearer:
    return 0.0
  hidden = total = 0.0
  for row, column, alpha in filled:
    x, y = left + (column + 0.5) * width / CELLS, top + (row + 0.5) * height / CELLS
    clear = 1.0
    for other in nearer:
      clear *= 1.0 - _alpha_at(other, x, y)
    hidden += alpha * (1.0 - clear)
    total += alpha
  return hidden / total if total else 0.0


def _path_offset(vehicle: dict, model) -> float:
  """Lateral from the planned path at the vehicle (the raw lateral without a path)."""
  path_y = None if model is None else drawing_path_y_at(model, camera_x(vehicle['distanceM']))
  return vehicle['lateralM'] - (path_y or 0.0)


def _truck_bodies(vehicle: dict, trucks: list, model=None) -> list:
  """Camera trucks whose body holds this radar point (geometry only; settle_overlaps also wants it drawn behind)."""
  # 'kind' on a radar entry means the camera matched it to an object of its own (annotate_camera): keep it.
  if vehicle.get('source') != 'radar' or vehicle.get('longitudinalLead') or 'kind' in vehicle:
    return []
  offset = _path_offset(vehicle, model)
  bodies = []
  for truck in trucks:
    if vehicle is truck or not truck['distanceM'] < vehicle['distanceM'] <= truck['distanceM'] + TRUCK_BODY_M:
      continue
    if vehicle.get('lane') and truck.get('lane') and vehicle['lane'] != truck['lane']:
      continue
    if abs(offset - _path_offset(truck, model)) <= TRUCK_HALF_WIDTH_M:
      bodies.append(truck)
  return bodies


def _in_truck_body(vehicle: dict, trucks: list, model=None) -> bool:
  return bool(_truck_bodies(vehicle, trucks, model))


def settle_overlaps(payload: dict, model=None) -> None:
  vehicles = payload.get('navVehicles')
  if not isinstance(vehicles, list) or len(vehicles) < 2:
    return
  kept, drawn, trucks = [], [], []   # trucks: (entry, placement) of trucks already kept, so drawn in front
  for vehicle in sorted(vehicles, key=lambda v: v['distanceM']):     # nearest first: it is in front
    placed = _placement(vehicle)
    # Only a truck that is itself drawn can hide its own radar points (Codex review 2026-11-15, N3).
    if placed is not None and trucks and any(
        hidden_fraction(placed, [truck_placed]) >= TRUCK_HIDDEN_FRACTION
        for truck, truck_placed in trucks if truck in _truck_bodies(vehicle, [truck], model)):
      continue
    if placed is not None and not vehicle.get('longitudinalLead') and drawn and hidden_fraction(placed, drawn) >= HIDDEN_FRACTION:
      continue
    kept.append(vehicle)
    if placed is not None:
      drawn.append(placed)
      if vehicle.get('kind') == 'truck':
        trucks.append((vehicle, placed))
  payload['navVehicles'] = sorted(kept, key=lambda v: v['distanceM'], reverse=True)
