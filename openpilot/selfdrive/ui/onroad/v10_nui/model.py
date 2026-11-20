"""Lane lines, road edges, path and lead boxes of the v10-nui tici screen (v10-nui qt/onroad/model.cc)."""
import math

import numpy as np
import pyray as rl

from openpilot.cereal import log
from openpilot.selfdrive.locationd.calibrationd import HEIGHT_INIT
from openpilot.selfdrive.ui.onroad.v10_nui import draw as d
from openpilot.selfdrive.ui.onroad.v10_nui import state as s
from openpilot.system.ui.lib import native_geometry
from openpilot.system.ui.lib.application import FontWeight, gui_app
from openpilot.system.ui.lib.shader_polygon import Gradient, draw_polygon
from openpilot.system.ui.lib.text_measure import measure_text_cached

CLIP_MARGIN = 500
MIN_DRAW_DISTANCE = 10.0
MAX_DRAW_DISTANCE = 100.0
PARAM_REFRESH_S = 1.0

LEAD_BOX_ALPHA = 0.85
LEAD_BOX_HALF_WIDTH_M = 1.2
LEAD_BOX_HEIGHT_FACTOR = 0.8
LEAD_BOX_MIN_WIDTH = 120.0
LEAD_BOX_MAX_WIDTH = 800.0
LEAD_BOX_RADIUS = 15.0
LEAD_BOX_STROKE = 3.0
LEAD_BOX_HPAD = 10.0
LEAD_BADGE_OFFSET_X = 80.0
LEAD_BADGE_OFFSET_Y = 60.0
LEAD_BADGE_HEIGHT = 42.0

CARROT_RED: s.RGBA = (255, 0, 0, 255)
CARROT_ORANGE: s.RGBA = (255, 175, 3, 255)
CARROT_BLUE: s.RGBA = (0, 0, 255, 255)
CARROT_OCHRE: s.RGBA = (218, 111, 37, 255)
LEAD1 = log.LongitudinalPlan.LongitudinalPlanSource.lead1


class LeadBox:
  def __init__(self):
    self.visible = False
    self.initialized = False
    self.radar_detected = False
    self.lead_scc = False
    self.selected = False
    self.x = self.y = self.width = 0.0
    self.radar_distance = 0.0


def path_length_idx(xs: np.ndarray, distance: float) -> int:
  """model.cc get_path_length_idx: last index whose x is within distance (at least 0)."""
  if len(xs) == 0:
    return 0
  return max(int(np.searchsorted(xs, distance, side='right')) - 1, 0)


class NuiModel:
  def __init__(self, params):
    self._params = params
    self._font_bold = gui_app.font(FontWeight.BOLD)
    self._transform = np.zeros((3, 3), dtype=np.float32)
    self._throttle = s.ThrottleBlend()
    self._lead_boxes = [LeadBox(), LeadBox()]
    self._show_path_end = 0
    self._param_next = 0.0
    self._last_draw = None
    self._model_frame = -1
    self._path = np.empty((0, 3), dtype=np.float32)
    self._lanes = [np.empty((0, 3), dtype=np.float32) for _ in range(4)]
    self._edges = [np.empty((0, 3), dtype=np.float32) for _ in range(2)]
    self._lane_probs = np.zeros(4, dtype=np.float32)
    self._edge_stds = np.zeros(2, dtype=np.float32)
    self._accel = np.empty((0,), dtype=np.float32)
    self._is_metric = True

  def set_transform(self, transform: np.ndarray) -> None:
    self._transform = transform.astype(np.float32)

  def reset(self) -> None:
    self._lead_boxes = [LeadBox(), LeadBox()]
    self._throttle = s.ThrottleBlend()

  def _refresh_params(self, now: float) -> None:
    if now >= self._param_next:
      self._param_next = now + PARAM_REFRESH_S
      try:
        self._show_path_end = int(self._params.get_int("ShowPathEnd"))
      except Exception:
        self._show_path_end = 0

  def _project(self, line: np.ndarray, half_width: float, z_off: float, max_idx: int, clip: rl.Rectangle,
               allow_invert: bool = True) -> np.ndarray:
    if len(line) == 0:
      return np.empty((0, 2), dtype=np.float32)
    return native_geometry.project_ribbon(line, half_width, z_off, max_idx, self._transform, clip, allow_invert, None)

  def _map_to_screen(self, x: float, y: float, z: float) -> tuple[float, float] | None:
    pt = self._transform @ np.array([x, y, z], dtype=np.float32)
    if abs(pt[2]) < 1e-6:
      return None
    return float(pt[0] / pt[2]), float(pt[1] / pt[2])

  def draw(self, rect: rl.Rectangle, sm, status: str, started_frame: int, now: float, is_metric: bool = True) -> None:
    self._is_metric = is_metric
    dt = 0.05 if self._last_draw is None else max(0.0, min(now - self._last_draw, 0.25))
    self._last_draw = now
    if sm.recv_frame['liveCalibration'] < started_frame or sm.recv_frame['modelV2'] < started_frame:
      return
    if not self._transform.any():  # no camera frame yet, so nothing to project onto
      return
    self._refresh_params(now)

    clip = rl.Rectangle(rect.x - CLIP_MARGIN, rect.y - CLIP_MARGIN, rect.width + 2 * CLIP_MARGIN, rect.height + 2 * CLIP_MARGIN)
    ss, cc, cs = sm['selfdriveState'], sm['carControl'], sm['carState']
    experimental = bool(ss.experimentalMode)
    long_control = bool(sm['carParams'].openpilotLongitudinalControl)
    calib = sm['liveCalibration']
    path_offset_z = float(calib.height[0]) if len(calib.height) else float(HEIGHT_INIT[0])
    lane_mode = bool(sm.alive['controlsState'] and sm['controlsState'].activeLaneLine)
    lateral_only = s.lateral_only_active(bool(ss.enabled), bool(cc.longActive), bool(cc.latActive), bool(cs.latEnabled))
    hidden = status == s.STATUS_DISENGAGED and not lateral_only

    model = sm['modelV2']
    if sm.recv_frame['modelV2'] != self._model_frame:
      self._model_frame = sm.recv_frame['modelV2']
      self._path = np.array([model.position.x, model.position.y, model.position.z], dtype=np.float32).T
      self._lanes = [np.array([ln.x, ln.y, ln.z], dtype=np.float32).T for ln in model.laneLines]
      self._edges = [np.array([e.x, e.y, e.z], dtype=np.float32).T for e in model.roadEdges]
      self._lane_probs = np.array(model.laneLineProbs, dtype=np.float32)
      self._edge_stds = np.array(model.roadEdgeStds, dtype=np.float32)
      self._accel = np.array(model.acceleration.x, dtype=np.float32)
    if len(self._path) == 0 or len(self._lanes) < 4 or len(self._edges) < 2:
      return

    lane_change_state = int(model.meta.laneChangeState.raw)
    direction = int(model.meta.laneChangeDirection.raw)
    radar = sm['radarState']
    lead_one = radar.leadOne

    # update_model
    max_distance = float(np.clip(self._path[-1, 0], MIN_DRAW_DISTANCE, MAX_DRAW_DISTANCE))
    max_idx = path_length_idx(self._lanes[0][:, 0], max_distance)
    lanes = []
    for i in range(4):
      prob = float(self._lane_probs[i]) if i < len(self._lane_probs) else 0.0
      lanes.append(self._project(self._lanes[i], s.lane_line_half_width(prob, i, lane_mode), 0.0, max_idx, clip))
    edges = [self._project(edge, 0.025, 0.0, max_idx, clip) for edge in self._edges[:2]]
    if lead_one.status:
      lead_d = float(lead_one.dRel) * 2.0
      max_distance = float(np.clip(lead_d - min(lead_d * 0.35, 10.0), 0.0, max_distance))
    path_idx = path_length_idx(self._path[:, 0], max_distance)
    track = self._project(self._path, 0.9, path_offset_z, path_idx, clip, allow_invert=False)

    if not hidden:
      self._draw_lane_lines(rect, lanes, edges, lane_change_state, direction, lane_mode)
    self._draw_path(rect, sm, track, experimental, long_control, lateral_only, lane_mode, hidden, dt)

    if self._show_path_end > 0 and sm.alive['radarState'] and sm.alive['longitudinalPlan'] and not hidden:
      self._update_leads(rect, radar, model, sm['longitudinalPlan'], path_offset_z)
      if self._lead_boxes[1].visible:
        self._draw_lead_box(self._lead_boxes[1], True)
      if self._lead_boxes[0].visible:
        self._draw_lead_box(self._lead_boxes[0], False)
        self._draw_lead_badges(self._lead_boxes[0], sm['modelV2'])
    else:
      self._lead_boxes[0].visible = self._lead_boxes[1].visible = False

  def _draw_lane_lines(self, rect, lanes, edges, lane_change_state, direction, lane_mode) -> None:
    for i, pts in enumerate(lanes):
      style = s.lane_line_style(i, lane_change_state, direction, lane_mode)
      if style == s.LANE_SKIP or len(pts) < 3:
        continue
      prob = float(self._lane_probs[i]) if i < len(self._lane_probs) else 0.0
      fill, outline, width = s.lane_line_colors(style, prob)
      draw_polygon(rect, pts, color=d.color(fill))
      if outline is not None:
        d.draw_polygon_outline(pts, outline, width)
    for i, pts in enumerate(edges):
      if len(pts) < 3:
        continue
      std = float(self._edge_stds[i]) if i < len(self._edge_stds) else 1.0
      draw_polygon(rect, pts, color=d.color((255, 0, 0, int(round(s.clamp(1.0 - std, 0.0, 1.0) * 255)))))

  def _draw_path(self, rect, sm, track, experimental, long_control, lateral_only, lane_mode, hidden, dt) -> None:
    allow_throttle = bool(sm['longitudinalPlan'].allowThrottle) or not long_control
    throttle_colors = self._throttle.update(allow_throttle, dt)  # keeps the fade running in every mode
    if len(track) < 3:
      return
    stops = s.path_base_colors(lateral_only, lane_mode)
    if stops is None and experimental:
      half = len(track) // 2
      stops = tuple(s.experimental_path_stops(track[:half, 1], self._accel, rect.y, rect.height)) or ((0.0, (255, 255, 255, 30)),)
    if stops is None:
      stops = tuple(zip((0.0, 0.5, 1.0), throttle_colors, strict=True))
    if hidden:
      stops = tuple((p, s.gray_tone(c)) for p, c in stops)
    gradient = Gradient(start=(0.0, 1.0), end=(0.0, 0.0), colors=[d.color(c) for _, c in stops], stops=[p for p, _ in stops])
    draw_polygon(rect, track, gradient=gradient)

  def _update_leads(self, rect, radar, model, plan, path_offset_z) -> None:
    lead_one, lead_two = radar.leadOne, radar.leadTwo
    self._update_box(rect, lead_one, self._lead_boxes[0], False, path_offset_z)
    if lead_two.radar and (not lead_one.status or lead_two.dRel > lead_one.dRel + 3.0):
      self._update_box(rect, lead_two, self._lead_boxes[1], plan.longitudinalPlanSource == LEAD1, path_offset_z)
    else:
      self._lead_boxes[1].visible = False

  def _update_box(self, rect, lead, box: LeadBox, selected: bool, path_offset_z: float) -> None:
    if not lead.status:
      box.visible = False
      return
    idx = path_length_idx(self._path[:, 0], float(lead.dRel))
    z = float(self._path[min(idx, len(self._path) - 1), 2])
    y = -float(lead.yRel)
    left = self._map_to_screen(float(lead.dRel), y - LEAD_BOX_HALF_WIDTH_M, z + path_offset_z)
    right = self._map_to_screen(float(lead.dRel), y + LEAD_BOX_HALF_WIDTH_M, z + path_offset_z)
    if left is None or right is None:
      box.visible = False
      return
    raw_w = s.clamp(right[0] - left[0], LEAD_BOX_MIN_WIDTH, LEAD_BOX_MAX_WIDTH)
    raw_x = s.clamp((left[0] + right[0]) * 0.5, rect.x + 350.0, rect.x + rect.width - 350.0)
    raw_y = s.clamp((left[1] + right[1]) * 0.5, rect.y + 200.0, rect.y + rect.height - 80.0)
    if not all(math.isfinite(v) for v in (raw_x, raw_y, raw_w)):
      box.visible = False
      return
    if not box.initialized:
      box.x, box.y, box.width, box.initialized = raw_x, raw_y, raw_w, True
    else:
      box.x = box.x * LEAD_BOX_ALPHA + raw_x * (1.0 - LEAD_BOX_ALPHA)
      box.y = box.y * LEAD_BOX_ALPHA + raw_y * (1.0 - LEAD_BOX_ALPHA)
      box.width = box.width * LEAD_BOX_ALPHA + raw_w * (1.0 - LEAD_BOX_ALPHA)
    box.visible = True
    box.selected = selected
    box.radar_detected = int(lead.radarTrackId) >= 0
    box.lead_scc = int(lead.radarTrackId) < 1
    box.radar_distance = float(lead.dRel) if lead.radar else 0.0

  def _draw_lead_box(self, box: LeadBox, secondary: bool) -> None:
    w = box.width + LEAD_BOX_HPAD * 2.0
    h = box.width * LEAD_BOX_HEIGHT_FACTOR
    x = box.x - box.width * 0.5 - LEAD_BOX_HPAD
    y = box.y - h
    stroke, fill = CARROT_BLUE, (0, 0, 0, 20)
    if secondary:
      stroke = CARROT_OCHRE
      fill = (255, 0, 0, 50) if box.selected else (0, 0, 0, 20)
    elif box.radar_detected:
      stroke = CARROT_RED if box.lead_scc else CARROT_ORANGE
    roundness = s.clamp(LEAD_BOX_RADIUS / (min(w, h) / 2), 0.0, 1.0)
    rect = rl.Rectangle(x, y, w, h)
    rl.draw_rectangle_rounded(rect, roundness, 12, d.color(fill))
    rl.draw_rectangle_rounded_lines_ex(rect, roundness, 12, LEAD_BOX_STROKE, d.color(stroke))

  def _draw_lead_badges(self, box: LeadBox, model) -> None:
    vision = 0.0
    if len(model.leadsV3) > 0:
      lead = model.leadsV3[0]
      if lead.prob > 0.5 and len(lead.x) > 0:
        vision = float(lead.x[0]) - 1.52
    scale = 1.0 if self._is_metric else s.METER_TO_FOOT
    y = box.y + LEAD_BADGE_OFFSET_Y
    for center_x, distance, bg in ((box.x - LEAD_BADGE_OFFSET_X, box.radar_distance, CARROT_RED if box.lead_scc else CARROT_ORANGE),
                                   (box.x + LEAD_BADGE_OFFSET_X, vision, CARROT_BLUE)):
      if distance <= 0.0:
        continue
      text = f"{distance * scale:.1f}"
      tw = measure_text_cached(self._font_bold, text, 30).x
      bw = tw + 28
      bx, by = center_x - bw * 0.5, y - 35.0
      d.draw_rounded_rect(bx, by, bw, LEAD_BADGE_HEIGHT, LEAD_BOX_RADIUS, bg)
      d.draw_text(self._font_bold, text, 30, (bx, by, bw, LEAD_BADGE_HEIGHT), (255, 255, 255, 255), d.CENTER, d.CENTER,
                  shadow=(0, 0, 0, 180), shadow_dy=1.5)

