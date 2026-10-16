import colorsys
import numpy as np
import pyray as rl
from openpilot.cereal import messaging
from opendbc.car.structs import car
from dataclasses import dataclass, field
from openpilot.common.params import Params
from openpilot.selfdrive.locationd.calibrationd import HEIGHT_INIT
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.shader_polygon import draw_polygon, Gradient
from openpilot.system.ui.widgets import Widget

CLIP_MARGIN = 500
MIN_DRAW_DISTANCE = 10.0
MAX_DRAW_DISTANCE = 100.0

STOCK_PATH_GRADIENT = Gradient(
  start=(0.0, 1.0),
  end=(0.0, 0.0),
  colors=[rl.Color(13, 248, 122, 102), rl.Color(114, 255, 92, 89), rl.Color(114, 255, 92, 0)],
  stops=[0.0, 0.5, 1.0],
)


@dataclass
class ModelPoints:
  raw_points: np.ndarray = field(default_factory=lambda: np.empty((0, 3), dtype=np.float32))
  projected_points: np.ndarray = field(default_factory=lambda: np.empty((0, 2), dtype=np.float32))


@dataclass
class LeadVehicle:
  glow: list[tuple[float, float]] = field(default_factory=list)
  chevron: list[tuple[float, float]] = field(default_factory=list)
  fill_alpha: int = 0


class ModelRenderer(Widget):
  def __init__(self):
    super().__init__()
    self._longitudinal_control = False
    self._lane_line_probs = np.zeros(4, dtype=np.float32)
    self._road_edge_stds = np.zeros(2, dtype=np.float32)
    self._lead_vehicles = [LeadVehicle(), LeadVehicle()]
    self._path_offset_z = HEIGHT_INIT[0]

    # Initialize ModelPoints objects
    self._path = ModelPoints()
    self._lane_lines = [ModelPoints() for _ in range(4)]
    self._road_edges = [ModelPoints() for _ in range(2)]
    self._acceleration_x = np.empty((0,), dtype=np.float32)

    # Transform matrix (3x3 for car space to screen space)
    self._car_space_transform = np.zeros((3, 3), dtype=np.float32)
    self._transform_dirty = True
    self._clip_region = None

    self._path_gradient = Gradient(
      start=(0.0, 1.0),  # Bottom of path
      end=(0.0, 0.0),  # Top of path
      colors=[],
      stops=[],
    )

    # Get longitudinal control setting from car parameters
    if car_params := Params().get("CarParams"):
      cp = messaging.log_from_bytes(car_params, car.CarParams)
      self._longitudinal_control = cp.openpilotLongitudinalControl

  def set_transform(self, transform: np.ndarray):
    self._car_space_transform = transform.astype(np.float32)
    self._transform_dirty = True

  def _render(self, rect: rl.Rectangle):
    sm = ui_state.sm

    # Check if data is up-to-date
    if (sm.recv_frame["extrinsicsCalibration"] < ui_state.started_frame or
        sm.recv_frame["modelV2"] < ui_state.started_frame):
      return

    # Set up clipping region
    self._clip_region = rl.Rectangle(
      rect.x - CLIP_MARGIN, rect.y - CLIP_MARGIN, rect.width + 2 * CLIP_MARGIN, rect.height + 2 * CLIP_MARGIN
    )

    # Update state
    extrinsics_calibration = sm['extrinsicsCalibration']
    self._path_offset_z = extrinsics_calibration.height[0] if extrinsics_calibration.height else HEIGHT_INIT[0]

    if sm.updated['carParams']:
      self._longitudinal_control = sm['carParams'].openpilotLongitudinalControl

    model = sm['modelV2']
    radar_state = sm['radarState'] if sm.valid['radarState'] else None
    lead_one = radar_state.leadOne if radar_state else None
    render_lead_indicator = self._longitudinal_control and radar_state is not None

    # Update model data when needed
    model_updated = sm.updated['modelV2']
    if model_updated or sm.updated['radarState'] or self._transform_dirty:
      if model_updated:
        self._update_raw_points(model)

      path_x_array = self._path.raw_points[:, 0]
      if path_x_array.size == 0:
        return

      self._update_model(lead_one, path_x_array)
      if render_lead_indicator:
        self._update_leads(radar_state, path_x_array)
      self._transform_dirty = False

    # Draw elements
    self._draw_lane_lines()
    self._draw_path(sm)

    if render_lead_indicator and radar_state:
      self._draw_lead_indicator()

  def _update_raw_points(self, model):
    """Update raw 3D points from model data"""
    self._path.raw_points = np.array([model.position.x, model.position.y, model.position.z], dtype=np.float32).T

    for i, lane_line in enumerate(model.laneLines):
      self._lane_lines[i].raw_points = np.array([lane_line.x, lane_line.y, lane_line.z], dtype=np.float32).T

    for i, road_edge in enumerate(model.roadEdges):
      self._road_edges[i].raw_points = np.array([road_edge.x, road_edge.y, road_edge.z], dtype=np.float32).T

    self._lane_line_probs = np.array(model.laneLineProbs, dtype=np.float32)
    self._road_edge_stds = np.array(model.roadEdgeStds, dtype=np.float32)
    self._acceleration_x = np.array(model.acceleration.x, dtype=np.float32)

  def _update_leads(self, radar_state, path_x_array):
    """Update positions of lead vehicles"""
    self._lead_vehicles = [LeadVehicle(), LeadVehicle()]
    leads = [radar_state.leadOne, radar_state.leadTwo]

    for i, lead_data in enumerate(leads):
      if lead_data and lead_data.present:
        d_rel, y_rel, v_rel = lead_data.dRel, lead_data.yRel, lead_data.vRel
        idx = self._get_path_length_idx(path_x_array, d_rel)

        # Get z-coordinate from path at the lead vehicle position
        z = self._path.raw_points[idx, 2] if idx < len(self._path.raw_points) else 0.0
        point = self._map_to_screen(d_rel, -y_rel, z + self._path_offset_z)
        if point:
          self._lead_vehicles[i] = self._update_lead_vehicle(d_rel, v_rel, point, self._rect)

  def _update_model(self, lead, path_x_array):
    """Update model visualization data based on model message"""
    max_distance = np.clip(path_x_array[-1], MIN_DRAW_DISTANCE, MAX_DRAW_DISTANCE)
    max_idx = self._get_path_length_idx(self._lane_lines[0].raw_points[:, 0], max_distance)

    # Update lane lines using raw points
    for i, lane_line in enumerate(self._lane_lines):
      lane_line.projected_points = self._map_line_to_polygon(
        lane_line.raw_points, 0.025 * self._lane_line_probs[i], 0.0, max_idx, max_distance
      )

    # Update road edges using raw points
    for road_edge in self._road_edges:
      road_edge.projected_points = self._map_line_to_polygon(road_edge.raw_points, 0.025, 0.0, max_idx, max_distance)

    # Update path using raw points
    if lead and lead.present:
      lead_d = lead.dRel * 2.0
      max_distance = np.clip(lead_d - min(lead_d * 0.35, 10.0), 0.0, max_distance)

    max_idx = self._get_path_length_idx(path_x_array, max_distance)
    self._path.projected_points = self._map_line_to_polygon(
      self._path.raw_points, 0.9, self._path_offset_z, max_idx, max_distance, allow_invert=False
    )

    self._update_path_gradient()

  def _update_path_gradient(self):
    if not self._longitudinal_control:
      return

    max_len = min(len(self._path.projected_points) // 2, len(self._acceleration_x))

    segment_colors = []
    gradient_stops = []

    i = 0
    while i < max_len:
      # Some points (screen space) are out of frame (rect space)
      track_y = self._path.projected_points[i][1]
      if track_y < self._rect.y or track_y > (self._rect.y + self._rect.height):
        i += 1
        continue

      # Calculate color based on acceleration (0 is bottom, 1 is top)
      lin_grad_point = 1 - (track_y - self._rect.y) / self._rect.height

      # speed up: 120, slow down: 0
      path_hue = np.clip(60 + self._acceleration_x[i] * 35, 0, 120)

      saturation = min(abs(self._acceleration_x[i] * 1.5), 1)
      lightness = np.interp(saturation, [0.0, 1.0], [0.95, 0.62])
      alpha = np.interp(lin_grad_point, [0.75 / 2.0, 0.75], [0.4, 0.0])

      # Use HSL to RGB conversion
      color = self._hsla_to_color(path_hue / 360.0, saturation, lightness, alpha)

      gradient_stops.append(lin_grad_point)
      segment_colors.append(color)

      # Skip a point, unless next is last
      i += 1 + (1 if (i + 2) < max_len else 0)

    # Store the gradient in the path object
    self._path_gradient = Gradient(
      start=(0.0, 1.0),  # Bottom of path
      end=(0.0, 0.0),  # Top of path
      colors=segment_colors,
      stops=gradient_stops,
    )

  def _update_lead_vehicle(self, d_rel, v_rel, point, rect):
    speed_buff, lead_buff = 10.0, 40.0

    # Calculate fill alpha
    fill_alpha = 0
    if d_rel < lead_buff:
      fill_alpha = 255 * (1.0 - (d_rel / lead_buff))
      if v_rel < 0:
        fill_alpha += 255 * (-1 * (v_rel / speed_buff))
      fill_alpha = min(fill_alpha, 255)

    # Calculate size and position
    sz = np.clip((25 * 30) / (d_rel / 3 + 30), 15.0, 30.0) * 2.35
    x = np.clip(point[0], 0.0, rect.width - sz / 2)
    y = min(point[1], rect.height - sz * 0.6)

    g_xo = sz / 5
    g_yo = sz / 10

    glow = [(x + (sz * 1.35) + g_xo, y + sz + g_yo), (x, y - g_yo), (x - (sz * 1.35) - g_xo, y + sz + g_yo)]
    chevron = [(x + (sz * 1.25), y + sz), (x, y), (x - (sz * 1.25), y + sz)]

    return LeadVehicle(glow=glow, chevron=chevron, fill_alpha=int(fill_alpha))

  def _draw_lane_lines(self):
    """Draw lane lines and road edges"""
    for i, lane_line in enumerate(self._lane_lines):
      if lane_line.projected_points.size == 0:
        continue

      alpha = np.clip(self._lane_line_probs[i], 0.0, 0.7)
      color = rl.Color(255, 255, 255, int(alpha * 255))
      draw_polygon(self._rect, lane_line.projected_points, color)

    for i, road_edge in enumerate(self._road_edges):
      if road_edge.projected_points.size == 0:
        continue

      alpha = np.clip(1.0 - self._road_edge_stds[i], 0.0, 1.0)
      color = rl.Color(255, 0, 0, int(alpha * 255))
      draw_polygon(self._rect, road_edge.projected_points, color)

  def _draw_path(self, sm):
    if not self._path.projected_points.size:
      return

    if not self._longitudinal_control:
      draw_polygon(self._rect, self._path.projected_points, gradient=STOCK_PATH_GRADIENT)
    elif len(self._path_gradient.colors) > 1:
      draw_polygon(self._rect, self._path.projected_points, gradient=self._path_gradient)
    else:
      draw_polygon(self._rect, self._path.projected_points, rl.Color(255, 255, 255, 30))

    # Tire trajectory overlay (port from carrotpilot)
    self._draw_tire_trajectory(sm)

  @staticmethod
  def _tire_grad_color(d: float, a: int) -> rl.Color:
    # danger 0(center)->1(lane touch): green -> yellow -> red
    d = max(0.0, min(1.5, d))
    if d <= 0.10:
      r, g = 0.0, 200.0
    elif d <= 0.60:
      f = (d - 0.10) / 0.50
      r, g = f * 255.0, 200.0
    elif d <= 1.00:
      f = (d - 0.60) / 0.40
      r, g = 255.0, 200.0 - f * 200.0
    else:
      r, g = 255.0, 0.0
    return rl.Color(int(r), int(g), 0, a)

  def _draw_tire_trajectory(self, sm):
    """Tire trajectory: draw tire tracks (±0.9m) as gradient bands on model path.
    Color changes green->red based on deviation from lane center. Pulses when close to lane departure.
    Controlled by TireTrajectory param (1=on, 0=off)."""
    try:
      enabled = int(Params().get("TireTrajectory", encoding='utf8') or "0")
    except (ValueError, TypeError):
      enabled = 0
    if enabled <= 0:
      return
    if not sm.valid['modelV2']:
      return

    model = sm['modelV2']
    lane_lines = model.laneLines
    probs = model.laneLineProbs
    if len(lane_lines) < 3 or len(probs) < 3:
      return

    left_y, right_y = lane_lines[1].y, lane_lines[2].y
    if len(left_y) == 0 or len(right_y) == 0:
      return

    # Distance from vehicle center to left/right lane (m). Model +Y=left, so invert for right.
    left_m = float(left_y[0])
    right_m = -float(right_y[0])
    # If one lane is weakly detected, estimate from the other with default 3.0m width.
    if probs[1] < 0.3 and probs[2] > 0.3:
      left_m = 3.0 - right_m
    elif probs[2] < 0.3 and probs[1] > 0.3:
      right_m = 3.0 - left_m
    total = left_m + right_m
    if total < 0.5:
      total = 3.0

    # drift>0 => vehicle shifted toward left lane (left tire close to left lane).
    drift = (right_m - left_m) / 2.0
    limit = max(0.1, total / 2.0 - 0.9)   # tire (half width 0.9m) touching lane = 1.0
    danger = abs(drift) / limit

    # Model predicted path. Limit to lead distance if present, else 40m max.
    path = self._path.raw_points
    if path.shape[0] < 5:
      return
    max_dist = 40.0
    radar_state = sm['radarState'] if sm.valid['radarState'] else None
    if radar_state is not None and radar_state.leadOne.status:
      d_rel = float(radar_state.leadOne.dRel)
      if 3.0 < d_rel < max_dist:
        max_dist = d_rel
    max_idx = self._get_path_length_idx(path[:, 0], max_dist)
    if max_idx < 5:
      return

    # Colors: deviated side = danger gradient, other side = green. Pulse on severe departure.
    green = rl.Color(0, 200, 0, 255)
    warn = abs(drift) >= 0.5
    pulse = warn and (int(rl.get_time() * 1000) % 800 < 400)
    if warn:
      side = rl.Color(255, 0, 0, 255) if pulse else rl.Color(255, 100, 0, 220)
      color_left = side if drift > 0.0 else green
      color_right = side if drift < 0.0 else green
    elif drift > 0.02:
      color_left, color_right = self._tire_grad_color(danger, 255), green
    elif drift < -0.02:
      color_left, color_right = green, self._tire_grad_color(danger, 255)
    else:
      color_left = color_right = green

    # Tire track bands: thin ribbons at ±0.9m from path center. Fade with distance.
    band_half = 0.16
    for y_shift, col in ((0.9, color_left), (-0.9, color_right)):
      ribbon = self._map_line_to_polygon(path, band_half, 1.22, max_idx, max_dist, True, y_shift)
      if ribbon.size == 0:
        continue
      grad = Gradient(
        start=(0.0, 1.0),  # bottom of path (near vehicle)
        end=(0.0, 0.0),    # top of path (far)
        colors=[rl.Color(col.r, col.g, col.b, col.a), rl.Color(col.r, col.g, col.b, 0)],
        stops=[0.0, 1.0],
      )
      draw_polygon(self._rect, ribbon, gradient=grad)

  def _draw_lead_indicator(self):
    # Draw lead vehicles if available
    for lead in self._lead_vehicles:
      if not lead.glow or not lead.chevron:
        continue

      rl.draw_triangle_fan(lead.glow, len(lead.glow), rl.Color(218, 202, 37, 255))
      rl.draw_triangle_fan(lead.chevron, len(lead.chevron), rl.Color(201, 34, 49, lead.fill_alpha))

  @staticmethod
  def _get_path_length_idx(pos_x_array: np.ndarray, path_distance: float) -> int:
    """Get the index corresponding to the given path distance"""
    if len(pos_x_array) == 0:
      return 0
    indices = np.where(pos_x_array <= path_distance)[0]
    return indices[-1] if indices.size > 0 else 0

  def _map_to_screen(self, in_x, in_y, in_z):
    """Project a point in car space to screen space"""
    input_pt = np.array([in_x, in_y, in_z])
    pt = self._car_space_transform @ input_pt

    if abs(pt[2]) < 1e-6:
      return None

    x, y = pt[0] / pt[2], pt[1] / pt[2]

    clip = self._clip_region
    if not (clip.x <= x <= clip.x + clip.width and clip.y <= y <= clip.y + clip.height):
      return None

    return (x, y)

  def _map_line_to_polygon(self, line: np.ndarray, y_off: float, z_off: float, max_idx: int, max_distance: float, allow_invert: bool = True, y_shift: float = 0.0) -> np.ndarray:
    """Convert 3D line to 2D polygon for rendering."""
    if line.shape[0] == 0:
      return np.empty((0, 2), dtype=np.float32)

    # Slice points and filter non-negative x-coordinates
    points = line[:max_idx + 1]

    # Interpolate around max_idx so path end is smooth (max_distance is always >= p0.x)
    if 0 < max_idx < line.shape[0] - 1:
      p0 = line[max_idx]
      p1 = line[max_idx + 1]
      x0, x1 = p0[0], p1[0]
      interp_y = np.interp(max_distance, [x0, x1], [p0[1], p1[1]])
      interp_z = np.interp(max_distance, [x0, x1], [p0[2], p1[2]])
      interp_point = np.array([max_distance, interp_y, interp_z], dtype=points.dtype)
      points = np.concatenate((points, interp_point[None, :]), axis=0)

    points = points[points[:, 0] >= 0]
    if points.shape[0] == 0:
      return np.empty((0, 2), dtype=np.float32)

    N = points.shape[0]
    # Generate left and right 3D points in one array using broadcasting
    # y_shift offsets the band laterally (for tire tracks)
    offsets = np.array([[0, -y_off + y_shift, z_off], [0, y_off + y_shift, z_off]], dtype=np.float32)
    points_3d = points[None, :, :] + offsets[:, None, :]  # Shape: 2xNx3
    points_3d = points_3d.reshape(2 * N, 3)  # Shape: (2*N)x3

    # Transform all points to projected space in one operation
    proj = self._car_space_transform @ points_3d.T  # Shape: 3x(2*N)
    proj = proj.reshape(3, 2, N)
    left_proj = proj[:, 0, :]
    right_proj = proj[:, 1, :]

    # Filter points where z is sufficiently large
    valid_proj = (np.abs(left_proj[2]) >= 1e-6) & (np.abs(right_proj[2]) >= 1e-6)
    if not np.any(valid_proj):
      return np.empty((0, 2), dtype=np.float32)

    # Compute screen coordinates
    left_screen = left_proj[:2, valid_proj] / left_proj[2, valid_proj][None, :]
    right_screen = right_proj[:2, valid_proj] / right_proj[2, valid_proj][None, :]

    # Define clip region bounds
    clip = self._clip_region
    x_min, x_max = clip.x, clip.x + clip.width
    y_min, y_max = clip.y, clip.y + clip.height

    # Filter points within clip region
    left_in_clip = (
      (left_screen[0] >= x_min) & (left_screen[0] <= x_max) &
      (left_screen[1] >= y_min) & (left_screen[1] <= y_max)
    )
    right_in_clip = (
      (right_screen[0] >= x_min) & (right_screen[0] <= x_max) &
      (right_screen[1] >= y_min) & (right_screen[1] <= y_max)
    )
    both_in_clip = left_in_clip & right_in_clip

    if not np.any(both_in_clip):
      return np.empty((0, 2), dtype=np.float32)

    # Select valid and clipped points
    left_screen = left_screen[:, both_in_clip]
    right_screen = right_screen[:, both_in_clip]

    # Handle Y-coordinate inversion on hills
    if not allow_invert and left_screen.shape[1] > 1:
      y = left_screen[1, :]  # y-coordinates
      keep = y == np.minimum.accumulate(y)
      if not np.any(keep):
        return np.empty((0, 2), dtype=np.float32)
      left_screen = left_screen[:, keep]
      right_screen = right_screen[:, keep]

    return np.vstack((left_screen.T, right_screen[:, ::-1].T)).astype(np.float32)

  @staticmethod
  def _hsla_to_color(h, s, l, a):
    rgb = colorsys.hls_to_rgb(h, l, s)
    return rl.Color(
      int(rgb[0] * 255),
      int(rgb[1] * 255),
      int(rgb[2] * 255),
      int(a * 255)
    )
