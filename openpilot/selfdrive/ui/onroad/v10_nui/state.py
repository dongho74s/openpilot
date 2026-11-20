"""Pure state and colour rules of the v10-nui tici screen (no raylib, no messaging).

Each rule cites the v10-nui Qt source it mirrors (traverse/v10-nui 0e57e272, selfdrive/ui/qt/onroad/).
"""
import colorsys
import math

RGBA = tuple[int, int, int, int]

SET_SPEED_NA = 255
KM_TO_MILE = 0.621371
MS_TO_KPH = 3.6
MS_TO_MPH = 2.236936
METER_TO_FOOT = 3.28084

STANDSTILL_TIMER_MIN_S = 60       # hud.cc: the parked timer replaces the speed after a minute
LEAD_DEPART_HOLD_S = 3.0          # alerts.cc: lead_departing_frames = 3 * UI_FREQ
TURN_SIGNAL_CYCLE_MS = 800
TURN_SIGNAL_ON_MS = 420
BLINDSPOT_BLINK_MS = 120
CONE_MAX_ROTATION_DEG = 38.0
CAMERA_DIM_BRIGHTNESS = 0.62      # annotated_camera.cc setFrameFilter(1.0, 0.62)

STATUS_DISENGAGED = "disengaged"
STATUS_ENGAGED = "engaged"
STATUS_OVERRIDE = "override"

# LaneChangeState / LaneChangeDirection raw values (cereal/log.capnp)
LANE_CHANGE_STARTING = 2
LANE_CHANGE_FINISHING = 3
DIRECTION_LEFT = 1
DIRECTION_RIGHT = 2

LATERAL_ONLY_MINT: RGBA = (0x67, 0xF5, 0xD1, 255)
LANE_CHANGE_MINT: RGBA = (0x53, 0xEE, 0xB6, 255)
OVERRIDE_BLUE: RGBA = (0x4F, 0x8D, 0xFF, 255)
ENGAGED_GREEN: RGBA = (0x49, 0xD2, 0x83, 255)


def clamp(value: float, lo: float, hi: float) -> float:
  return lo if value < lo else hi if value > hi else value


def with_alpha(color: RGBA, alpha: int) -> RGBA:
  return color[0], color[1], color[2], alpha


def blend(a: RGBA, b: RGBA, t: float) -> RGBA:
  """hud.cc blendColor: channel-wise mix with std::lround."""
  t = clamp(t, 0.0, 1.0)
  return tuple(int(math.floor(x + (y - x) * t + 0.5)) for x, y in zip(a, b, strict=True))  # type: ignore[return-value]


def gray_tone(color: RGBA) -> RGBA:
  """model.cc grayToneColor: luminance * 0.82, alpha kept."""
  r, g, b, a = color
  toned = clamp((0.299 * r + 0.587 * g + 0.114 * b) / 255.0 * 0.82, 0.0, 1.0)
  v = int(round(toned * 255))
  return v, v, v, a


def hsla(h_deg: float, s: float, light: float, a: float) -> RGBA:
  r, g, b = colorsys.hls_to_rgb(h_deg / 360.0, light, s)
  return int(round(r * 255)), int(round(g * 255)), int(round(b * 255)), int(round(a * 255))


# model.cc updatePathGradient
THROTTLE_COLORS: tuple[RGBA, ...] = (hsla(148, 0.94, 0.51, 0.4), hsla(112, 1.0, 0.68, 0.35), hsla(112, 1.0, 0.68, 0.0))
NO_THROTTLE_COLORS: tuple[RGBA, ...] = (hsla(148, 0.62, 0.70, 0.4), hsla(112, 0.72, 0.72, 0.35), hsla(112, 0.72, 0.72, 0.0))


def lateral_only_active(selfdrive_enabled: bool, long_active: bool, lat_active: bool, lat_enabled: bool) -> bool:
  """Steering assist without openpilot engaged (always-on lateral). Drawn in mint instead of grey."""
  return not selfdrive_enabled and not long_active and (lat_active or lat_enabled)


def camera_filter(started: bool, status: str, lateral_only: bool) -> tuple[float, float]:
  """(grayscale mix, dim amount) for the road camera. Disengaged and not steering: grey and 62% bright."""
  if started and status == STATUS_DISENGAGED and not lateral_only:
    return 1.0, 1.0 - CAMERA_DIM_BRIGHTNESS
  return 0.0, 0.0


def steering_torque_pct(lateral_control_state) -> float:
  """hud.cc: |output| of the active lateral controller, 1.0 when saturated."""
  try:
    which = lateral_control_state.which()
  except Exception:
    return 0.0
  if which not in ("torqueState", "pidState", "angleState", "debugState"):
    return 0.0
  state = getattr(lateral_control_state, which)
  if getattr(state, "saturated", False):
    return 1.0
  output = float(getattr(state, "output", 0.0))
  return clamp(abs(output), 0.0, 1.0) if math.isfinite(output) else 0.0


def wheel_style(torque: float, steering_active: bool, lateral_override: bool) -> tuple[RGBA, RGBA]:
  """hud.cc drawSteeringWheelIcon: (button background, wheel tint)."""
  if lateral_override:
    return (0x0A, 0x12, 0x1F, 0xB8), OVERRIDE_BLUE
  if steering_active:
    white, warm, hot = (0xF4, 0xF7, 0xFB, 0xF4), (0xFF, 0xC7, 0x58, 0xF6), (0xFF, 0x5D, 0x57, 0xF8)
    torque = clamp(torque, 0.0, 1.0)
    tint = blend(white, warm, torque / 0.65 * 0.35) if torque < 0.65 else blend(warm, hot, (torque - 0.65) / 0.35)
    return (0x0D, 0x16, 0x12, 0xB6), tint
  return (0x0A, 0x10, 0x16, 0xA8), (0x9F, 0xA8, 0xB2, 0xE6)


def lfa_style(long_override: bool, lateral_only: bool, enabled: bool, engageable: bool) -> tuple[RGBA, RGBA | None]:
  """hud.cc drawLfaIcon: (icon tint, glow colour or None)."""
  if long_override:
    return OVERRIDE_BLUE, OVERRIDE_BLUE
  if lateral_only:
    return LATERAL_ONLY_MINT, LATERAL_ONLY_MINT
  if enabled:
    return ENGAGED_GREEN, ENGAGED_GREEN
  if engageable:
    return (0xF4, 0xF7, 0xFB, 0xF4), None
  return (0x92, 0x9D, 0xA8, 0xE6), None


def set_speed_colors(cruise_set: bool, status: str) -> tuple[RGBA, RGBA]:
  """hud.cc drawSetSpeed: (value colour, 'SET' label colour)."""
  if not cruise_set or status == STATUS_DISENGAGED:
    return (0xD2, 0xD9, 0xE1, 0xCA), (0xA7, 0xB0, 0xBA, 0xB8)
  if status == STATUS_OVERRIDE:
    return (0xF3, 0xE6, 0xCB, 0xE2), (0xD6, 0xC3, 0x98, 0xBA)
  return (0xF2, 0xF5, 0xF9, 0xF2), (0xD1, 0xD8, 0xDF, 0xD0)


def current_speed_colors(status: str) -> tuple[RGBA, RGBA]:
  """hud.cc drawCurrentSpeed: (speed colour, unit colour)."""
  if status == STATUS_DISENGAGED:
    return (0xE2, 0xE7, 0xEC, 0xE6), (0xC5, 0xCF, 0xD8, 0xC8)
  if status == STATUS_OVERRIDE:
    return (0xF6, 0xF8, 0xFB, 0xF4), (0xE8, 0xDC, 0xC2, 0xC4)
  return (0xF6, 0xF8, 0xFB, 0xF4), (0xDF, 0xE5, 0xEA, 0xD4)


def gap_tint(status: str) -> RGBA:
  if status == STATUS_DISENGAGED:
    return 0xD4, 0xDD, 0xE6, 0xDA
  if status == STATUS_OVERRIDE:
    return 0xF2, 0xE6, 0xC9, 0xEA
  return 0xF6, 0xF8, 0xFB, 0xEE


def gap_level(personality: int | None, fallback_personality: int) -> int:
  """1..4 bars from the longitudinal personality (aggressive=0 .. moreRelaxed=3)."""
  value = fallback_personality if personality is None else personality
  return int(clamp(int(value) + 1, 1, 4))


def set_speed(v_cruise_cluster: float, v_cruise_fallback: float, is_metric: bool) -> tuple[float, bool, bool]:
  """(displayed set speed, cruise set, cruise available). km/h in, km/h or mph out."""
  speed = v_cruise_fallback if v_cruise_cluster == 0.0 else v_cruise_cluster
  cruise_set = 0 < speed < SET_SPEED_NA
  available = speed != -1
  if cruise_set and not is_metric:
    speed *= KM_TO_MILE
  return speed, cruise_set, available


def format_minutes_seconds(seconds: int) -> str:
  seconds = max(0, int(seconds))
  return f"{seconds // 60}:{seconds % 60:02d}"


class StandstillTimer:
  """hud.cc: seconds stopped (not in reverse), shown only after a minute; reset by resumeRequired."""

  def __init__(self):
    self._since: float | None = None

  def reset(self) -> None:
    self._since = None

  def update(self, standstill: bool, resume_required: bool, now: float) -> int:
    if resume_required or not standstill:
      self._since = None
      return 0
    if self._since is None:
      self._since = now
    elapsed = int(now - self._since)
    return elapsed if elapsed >= STANDSTILL_TIMER_MIN_S else 0


class ElapsedTimer:
  """alerts.cc resumeRequiredTimer: seconds since an alert type started."""

  def __init__(self):
    self._since: float | None = None

  def update(self, active: bool, now: float) -> int:
    if not active:
      self._since = None
      return 0
    if self._since is None:
      self._since = now
    return int(now - self._since)


def lead_departing_detected(standstill: bool, lead_status: bool, d_rel: float, v_rel: float,
                            v_lead_k: float, v_ego: float) -> bool:
  """alerts.cc: stopped behind a lead 1-30 m away that starts to move off."""
  return (standstill and lead_status and 1.0 < d_rel < 30.0 and
          (v_rel > 0.8 or v_lead_k > v_ego + 0.8))


class LeadDepartHold:
  """Keeps the lead-departed notice for three seconds after the last detection."""

  def __init__(self):
    self._until = -1.0

  def reset(self) -> None:
    self._until = -1.0

  def update(self, detected: bool, now: float) -> bool:
    if detected:
      self._until = now + LEAD_DEPART_HOLD_S
    return now < self._until


ALERT_NORMAL = "normal"
ALERT_STEER_SATURATED = "steer_saturated"
ALERT_RESUME_REQUIRED = "resume_required"
ALERT_COLLISION = "collision"
ALERT_LANE_DEPARTURE = "lane_departure"
ALERT_LEAD_DEPARTING = "lead_departing"


def alert_kind(alert_type: str) -> str:
  """alerts.cc type matching (case-insensitive substrings of selfdriveState.alertType)."""
  t = (alert_type or "").lower()
  if "leaddeparting" in t:
    return ALERT_LEAD_DEPARTING
  if "steersaturated" in t:
    return ALERT_STEER_SATURATED
  if "resumerequired" in t:
    return ALERT_RESUME_REQUIRED
  if "fcw" in t or "aeb" in t:
    return ALERT_COLLISION
  if "ldw" in t:
    return ALERT_LANE_DEPARTURE
  return ALERT_NORMAL


def turn_signal_on(now_ms: int) -> bool:
  return now_ms % TURN_SIGNAL_CYCLE_MS < TURN_SIGNAL_ON_MS


def blindspot_blink_visible(now_ms: int) -> bool:
  return (now_ms // BLINDSPOT_BLINK_MS) % 2 == 0


def dm_cone_rotation(yaw: float) -> float:
  if not math.isfinite(yaw):
    return 0.0
  return clamp(-yaw * 57.2957795, -CONE_MAX_ROTATION_DEG, CONE_MAX_ROTATION_DEG)


def dm_cone_tint(enabled: bool, engageable: bool) -> RGBA:
  if enabled:
    return ENGAGED_GREEN
  if engageable:
    return 0xF2, 0xF5, 0xF8, 0xFF
  return 0x8A, 0x8F, 0x96, 0xFF


LANE_PLAIN = "plain"
LANE_SKIP = "skip"
LANE_CHANGE = "change"
LANE_MODE = "mode"


def lane_line_style(index: int, lane_change_state: int, direction: int, lane_mode_active: bool) -> str:
  """model.cc drawLaneLines: hide the line being crossed, highlight the current and target lane edges."""
  changing = lane_change_state in (LANE_CHANGE_STARTING, LANE_CHANGE_FINISHING) and direction in (DIRECTION_LEFT, DIRECTION_RIGHT)
  if changing:
    crossing = 1 if direction == DIRECTION_LEFT else 2
    highlight = (2, 0) if direction == DIRECTION_LEFT else (1, 3)
    if index == crossing:
      return LANE_SKIP
    if index in highlight:
      return LANE_CHANGE
  if lane_mode_active and index in (1, 2):
    return LANE_MODE
  return LANE_PLAIN


def lane_line_half_width(prob: float, index: int, lane_mode_active: bool) -> float:
  """model.cc update_model: lane ribbons are thicker than stock and ego edges thicker in lane mode."""
  width = 0.025 * prob
  if lane_mode_active and index in (1, 2):
    return max(width * 1.7, 0.034)
  return max(width * 1.25, 0.028)


def lane_line_colors(style: str, prob: float) -> tuple[RGBA, RGBA | None, float]:
  """(fill, outline or None, outline width px) for one lane line."""
  if style == LANE_CHANGE:
    alpha = clamp(0.22 + prob * 0.62, 0.22, 0.88)
    return with_alpha(LANE_CHANGE_MINT, int(round(alpha * 255))), with_alpha(LANE_CHANGE_MINT, 128), 9.0
  if style == LANE_MODE:
    alpha = clamp(0.15 + prob * 0.65, 0.15, 0.8)
    glow_alpha = clamp(alpha * 0.5, 0.16, 0.34)
    fill = (int(round(0.14 * 255)), int(round(0.90 * 255)), int(round(0.38 * 255)), int(round(alpha * 255)))
    glow = (int(round(0.24 * 255)), int(round(0.95 * 255)), int(round(0.46 * 255)), int(round(glow_alpha * 255)))
    return fill, glow, 10.0
  return (255, 255, 255, int(round(clamp(prob, 0.0, 0.7) * 255))), None, 0.0


def path_base_colors(lateral_only: bool, lane_mode_active: bool) -> tuple[tuple[float, RGBA], ...] | None:
  """model.cc drawPath: fixed path gradients (bottom=0 .. top=1); None means throttle/experimental colours."""
  if lateral_only:
    return (0.0, with_alpha(LATERAL_ONLY_MINT, 110)), (0.55, with_alpha(LATERAL_ONLY_MINT, 84)), (1.0, with_alpha(LATERAL_ONLY_MINT, 8))
  if lane_mode_active:
    white = (0xF6, 0xF8, 0xFB, 0)
    return (0.0, with_alpha(white, 102)), (0.55, with_alpha(white, 58)), (1.0, white)
  return None


def _map_val(x: float, x0: float, x1: float, y0: float, y1: float) -> float:
  """common/util.h map_val (input clamped to [x0, x1])."""
  return y0 + (y1 - y0) * ((clamp(x, x0, x1) - x0) / (x1 - x0))


def experimental_path_stops(near_to_far_ys, accelerations, top: float, height: float) -> list[tuple[float, RGBA]]:
  """model.cc drawPath experimental branch: hue from acceleration along one edge of the path.

  near_to_far_ys are screen y values of one path edge, nearest first (Qt walks the right edge bottom-up).
  """
  stops: dict[float, RGBA] = {}
  max_len = min(len(near_to_far_ys), len(accelerations))
  i = 0
  while i < max_len:
    track_y = float(near_to_far_ys[i])
    if not top <= track_y <= top + height:
      i += 1  # Qt `continue` skips the extra step below
      continue
    point = (top + height - track_y) / height
    accel = float(accelerations[i])
    hue = max(min(60 + accel * 35, 120), 0)
    hue = int(hue * 100 + 0.5) // 100  # C++ integer division: whole degrees
    saturation = min(abs(accel * 1.5), 1.0)
    lightness = _map_val(saturation, 0.0, 1.0, 0.95, 0.62)
    alpha = _map_val(point, 0.75 / 2.0, 0.75, 0.4, 0.0)
    stops[point] = hsla(hue, saturation, lightness, alpha)  # QGradient: a later stop at the same point wins
    i += 1 + (1 if (i + 2) < max_len else 0)
  return sorted(stops.items())


class ThrottleBlend:
  """model.cc: 0.5 s cross-fade between throttle and no-throttle path colours."""

  def __init__(self):
    self.allow_throttle = True
    self.factor = 1.0

  def update(self, allow_throttle: bool, dt: float) -> tuple[RGBA, ...]:
    if allow_throttle != self.allow_throttle:
      self.allow_throttle = allow_throttle
      self.factor = max(1.0 - self.factor, 0.0)
    begin = NO_THROTTLE_COLORS if allow_throttle else THROTTLE_COLORS
    end = THROTTLE_COLORS if allow_throttle else NO_THROTTLE_COLORS
    if self.factor < 1.0:
      self.factor = min(self.factor + 2.0 * max(dt, 0.0), 1.0)
    if self.factor >= 1.0:
      return end
    return tuple(_blend_float(b, e, self.factor) for b, e in zip(begin, end, strict=True))


def _blend_float(a: RGBA, b: RGBA, t: float) -> RGBA:
  return tuple(int(round((1 - t) * x + t * y)) for x, y in zip(a, b, strict=True))  # type: ignore[return-value]
