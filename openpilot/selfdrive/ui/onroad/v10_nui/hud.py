"""Bottom HUD of the v10-nui tici screen (v10-nui selfdrive/ui/qt/onroad/hud.cc)."""
import math

import pyray as rl

from openpilot.cereal import car, log
from openpilot.selfdrive.ui.onroad.v10_nui import draw as d
from openpilot.selfdrive.ui.onroad.v10_nui import state as s
from openpilot.system.ui.lib.application import FontWeight, gui_app

STEERING_ICON_SIZE = 156
STEERING_ICON_WARNING_SIZE = 188
LFA_ICON_SIZE = 104
GAP_ICON_WIDTH = 124
GAP_ICON_HEIGHT = 84
CARROT_BADGE_WIDTH = 112
CARROT_BADGE_HEIGHT = 52
CARROT_BADGE_GAP = 18
TURN_SIGNAL_ICON_SIZE = 72
TURN_SIGNAL_GAP = 28
WARNING_ICON_SIZE = 76
HUD_SIDE_MARGIN = 104
HUD_BOTTOM_MARGIN = 96
SET_SPEED_GAP = 28
SET_SPEED_WIDTH = 236
SET_SPEED_HEIGHT = 118
SPEED_GROUP_FROM_BOTTOM = 246
PARAM_REFRESH_S = 1.0

OVERRIDING = log.SelfdriveState.OpenpilotState.overriding
REVERSE = car.CarState.GearShifter.reverse

TURN_SIGNAL_GREEN: s.RGBA = (0x1E, 0xFF, 0x5C, 255)
WHITE: s.RGBA = (255, 255, 255, 255)


def set_speed_rect(x0: float, y0: float, w: float, h: float) -> tuple[float, float, float, float]:
  return (x0 + w - HUD_SIDE_MARGIN - STEERING_ICON_SIZE - SET_SPEED_GAP - SET_SPEED_WIDTH,
          y0 + h - HUD_BOTTOM_MARGIN - SET_SPEED_HEIGHT, SET_SPEED_WIDTH, SET_SPEED_HEIGHT)


def speed_rect(x0: float, y0: float, w: float, h: float) -> tuple[float, float, float, float]:
  group_top = y0 + h - SPEED_GROUP_FROM_BOTTOM
  return x0 + w / 2 - 230, group_top - 18, 460, 162


def gap_rect(x0: float, y0: float, w: float, h: float) -> tuple[float, float, float, float]:
  sx, sy, _, sh = set_speed_rect(x0, y0, w, h)
  return sx - GAP_ICON_WIDTH - 34, sy + (sh - GAP_ICON_HEIGHT) // 2 + 2, GAP_ICON_WIDTH, GAP_ICON_HEIGHT


def wheel_center(x0: float, y0: float, w: float, h: float, size: int) -> tuple[float, float]:
  return x0 + w - HUD_SIDE_MARGIN - size / 2, y0 + h - HUD_BOTTOM_MARGIN - size / 2


class NuiHud:
  def __init__(self, params):
    self._params = params
    self._font_bold = gui_app.font(FontWeight.BOLD)
    self._font_semi = gui_app.font(FontWeight.SEMI_BOLD)
    self._turn_left = d.ThresholdTexture("turn_signal_left", TURN_SIGNAL_ICON_SIZE, TURN_SIGNAL_ICON_SIZE)
    self._turn_right = d.ThresholdTexture("turn_signal_right", TURN_SIGNAL_ICON_SIZE, TURN_SIGNAL_ICON_SIZE)
    self._standstill = s.StandstillTimer()
    self._param_personality = 1
    self._param_next = 0.0
    self._started_frame = -1
    self.reset()

  def reset(self) -> None:
    self.status = s.STATUS_DISENGAGED
    self.is_metric = True
    self.speed = 0.0
    self.set_speed = float(s.SET_SPEED_NA)
    self.is_cruise_set = False
    self.is_cruise_available = True
    self.enabled = False
    self.engageable = False
    self.long_override = False
    self.lat_override = False
    self.lateral_only = False
    self.steer_limit_warning = False
    self.torque = 0.0
    self.steering_angle = 0.0
    self.gap_level = 0
    self.carrot_level = 0
    self.left_signal = False
    self.right_signal = False
    self.standstill_seconds = 0
    self._v_ego_cluster_seen = False
    self._standstill.reset()

  def close(self) -> None:
    self._turn_left.close()
    self._turn_right.close()

  def _fallback_personality(self, now: float) -> int:
    if now >= self._param_next:
      self._param_next = now + PARAM_REFRESH_S
      try:
        self._param_personality = int(self._params.get_int("LongitudinalPersonality"))
      except Exception:
        self._param_personality = 1
    return self._param_personality

  def update(self, sm, started_frame: int, status: str, is_metric: bool, now: float) -> None:
    if started_frame != self._started_frame:
      self._started_frame = started_frame
      self.reset()
    self.status = status
    self.is_metric = is_metric

    ss_fresh = sm.recv_frame['selfdriveState'] >= started_frame
    ss = sm['selfdriveState']
    alert_type = str(ss.alertType) if ss_fresh else ""
    self.enabled = bool(ss.enabled) if ss_fresh else False
    self.engageable = (bool(ss.engageable) or self.enabled) if ss_fresh else False
    self.steer_limit_warning = s.alert_kind(alert_type) == s.ALERT_STEER_SATURATED
    personality = int(ss.personality.raw) if sm.recv_frame['selfdriveState'] > 0 else None
    self.gap_level = s.gap_level(personality, self._fallback_personality(now) if personality is None else 0)
    self.carrot_level = int(sm['carrotMan'].activeCarrot) if sm.recv_frame['carrotMan'] > 0 else 0
    self.long_override = self.lat_override = self.lateral_only = False
    self.torque = 0.0
    self.steering_angle = 0.0
    self.left_signal = self.right_signal = False

    if sm.recv_frame['carState'] < started_frame:
      self.is_cruise_set = False
      self.set_speed = float(s.SET_SPEED_NA)
      self.speed = 0.0
      self.standstill_seconds = 0
      self._standstill.reset()
      return

    cs = sm['carState']
    cc = sm['carControl']
    controls = sm['controlsState']
    self.left_signal = bool(cs.leftBlinker)
    self.right_signal = bool(cs.rightBlinker)
    self.lateral_only = s.lateral_only_active(self.enabled, bool(cc.longActive), bool(cc.latActive), bool(cs.latEnabled))
    if ss_fresh and ss.state == OVERRIDING:
      self.long_override = bool(cs.gasPressed)
      self.lat_override = bool(cs.steeringPressed)
    angle = float(cs.steeringAngleDeg)
    self.steering_angle = -angle if math.isfinite(angle) else 0.0
    self.torque = s.steering_torque_pct(controls.lateralControlState)

    standstill = bool(cs.standstill) and cs.gearShifter != REVERSE
    self.standstill_seconds = self._standstill.update(standstill, s.alert_kind(alert_type) == s.ALERT_RESUME_REQUIRED, now)

    fallback = float(getattr(getattr(controls, 'deprecated', None), 'vCruise', 0.0) or 0.0)
    self.set_speed, self.is_cruise_set, self.is_cruise_available = s.set_speed(float(cs.vCruiseCluster), fallback, is_metric)

    v_ego_cluster = float(cs.vEgoCluster)
    self._v_ego_cluster_seen = self._v_ego_cluster_seen or v_ego_cluster != 0.0
    v_ego = v_ego_cluster if self._v_ego_cluster_seen else float(cs.vEgo)
    self.speed = max(0.0, v_ego * (s.MS_TO_KPH if is_metric else s.MS_TO_MPH))

  def draw(self, rect: rl.Rectangle) -> None:
    x0, y0, w, h = rect.x, rect.y, rect.width, rect.height
    if self.is_cruise_available:
      self._draw_set_speed(x0, y0, w, h)
    if self.standstill_seconds == 0:
      self._draw_current_speed(x0, y0, w, h)
    else:
      self._draw_standstill_timer(x0, y0, w, h)
    self._draw_turn_signals(x0, y0, w, h)
    self._draw_carrot_badge(x0, y0, w, h)
    self._draw_gap_icon(x0, y0, w, h)
    self._draw_lfa_icon(x0, y0, w, h)
    self._draw_steer_limit_warning(x0, y0, w, h)
    self._draw_steering_wheel(x0, y0, w, h)

  def _draw_set_speed(self, x0, y0, w, h) -> None:
    value_color, label_color = s.set_speed_colors(self.is_cruise_set, self.status)
    text = str(int(round(self.set_speed))) if self.is_cruise_set else "–"
    sx, sy, sw, sh = set_speed_rect(x0, y0, w, h)
    d.draw_text(self._font_semi, "SET", 28, (sx, sy + 44, 72, 34), label_color)
    d.draw_text(self._font_bold, text, 88, (sx + 72, sy - 6, sw - 72, sh), value_color)

  def _draw_current_speed(self, x0, y0, w, h) -> None:
    speed_color, unit_color = s.current_speed_colors(self.status)
    rx, ry, rw, rh = speed_rect(x0, y0, w, h)
    d.draw_text(self._font_bold, str(int(round(self.speed))), 148, (rx, ry, rw, rh), speed_color, d.CENTER, d.BOTTOM)
    cx = x0 + w / 2
    d.draw_text(self._font_semi, "KM/H" if self.is_metric else "MPH", 34, (cx - 116, ry + rh + 1, 232, 40), unit_color,
                d.CENTER, d.TOP)

  def _draw_standstill_timer(self, x0, y0, w, h) -> None:
    group_top = y0 + h - SPEED_GROUP_FROM_BOTTOM
    cx = x0 + w / 2
    d.draw_text(self._font_bold, s.format_minutes_seconds(self.standstill_seconds), 132, (cx - 260, group_top - 6, 520, 150),
                WHITE, d.CENTER, d.BOTTOM)

  def _draw_turn_signals(self, x0, y0, w, h) -> None:
    if not (self.left_signal or self.right_signal):
      return
    rx, ry, rw, rh = speed_rect(x0, y0, w, h)
    icon_y = ry + (rh - 1) // 2 - TURN_SIGNAL_ICON_SIZE // 2 + 2
    opacity = 0.96 if s.turn_signal_on(d.blink_ms()) else 0.14
    tint = d.color(TURN_SIGNAL_GREEN, opacity)
    if self.left_signal:
      d.draw_texture_rect(self._turn_left.get(), rx - TURN_SIGNAL_ICON_SIZE - TURN_SIGNAL_GAP, icon_y,
                          TURN_SIGNAL_ICON_SIZE, TURN_SIGNAL_ICON_SIZE, tint)
    if self.right_signal:
      d.draw_texture_rect(self._turn_right.get(), rx + rw - 1 + TURN_SIGNAL_GAP, icon_y,
                          TURN_SIGNAL_ICON_SIZE, TURN_SIGNAL_ICON_SIZE, tint)

  def _draw_gap_icon(self, x0, y0, w, h) -> None:
    if not 1 <= self.gap_level <= 4:
      return
    tex = d.texture(f"gap{self.gap_level}", GAP_ICON_WIDTH, GAP_ICON_HEIGHT)
    gx, gy, gw, gh = gap_rect(x0, y0, w, h)
    d.draw_texture_rect(tex, gx, gy + 4, gw, gh, d.color((0, 0, 0, 0x74)))
    d.draw_texture_rect(tex, gx, gy, gw, gh, d.color(s.gap_tint(self.status)))

  def _draw_carrot_badge(self, x0, y0, w, h) -> None:
    if self.carrot_level < 1:
      return
    gx, gy, gw, _ = gap_rect(x0, y0, w, h)
    bx = gx + (gw - CARROT_BADGE_WIDTH) // 2
    by = gy - CARROT_BADGE_HEIGHT - CARROT_BADGE_GAP
    badge = (0x2F, 0xC8, 0x67, 0xF2) if self.carrot_level >= 2 else (0x4F, 0x8D, 0xFF, 0xEC)
    d.draw_rounded_rect(bx, by + 4, CARROT_BADGE_WIDTH, CARROT_BADGE_HEIGHT, 18, (0, 0, 0, 0x56))
    d.draw_rounded_rect(bx, by, CARROT_BADGE_WIDTH, CARROT_BADGE_HEIGHT, 18, badge)
    d.draw_text(self._font_bold, "APN", 28, (bx, by, CARROT_BADGE_WIDTH, CARROT_BADGE_HEIGHT), WHITE, d.CENTER, d.CENTER)

  def _draw_lfa_icon(self, x0, y0, w, h) -> None:
    rx, ry, rw, _ = speed_rect(x0, y0, w, h)
    ix, iy = rx + rw - 1 - 84, ry + 2
    tint, glow = s.lfa_style(self.long_override, self.lateral_only, self.enabled, self.engageable)
    if glow is not None:
      gw, gh = LFA_ICON_SIZE + 52, LFA_ICON_SIZE + 46
      d.draw_radial_glow((ix - 26, iy - 22, gw, gh), gw * 0.55, d.glow_stops(glow, 76, 34))
    d.draw_texture_rect(d.texture("lfa", LFA_ICON_SIZE, LFA_ICON_SIZE), ix, iy, LFA_ICON_SIZE, LFA_ICON_SIZE, d.color(tint))

  def _draw_steering_wheel(self, x0, y0, w, h) -> None:
    button = STEERING_ICON_WARNING_SIZE if self.steer_limit_warning else STEERING_ICON_SIZE
    icon = 178 if self.steer_limit_warning else 150
    cx, cy = wheel_center(x0, y0, w, h, button)
    steering_active = self.enabled or self.long_override or self.lat_override or self.lateral_only
    bg, tint = s.wheel_style(self.torque, steering_active, self.lat_override)
    rl.draw_circle(int(round(cx)), int(round(cy)), button / 2, d.color(bg))
    tex = d.texture("steeringwheel", icon, icon, keep_aspect=True)
    d.draw_texture_rotated(tex, cx, cy, icon, icon, self.steering_angle, d.color(tint))

  def _draw_steer_limit_warning(self, x0, y0, w, h) -> None:
    if not self.steer_limit_warning:
      return
    cx, cy = wheel_center(x0, y0, w, h, STEERING_ICON_WARNING_SIZE)
    ix = cx - STEERING_ICON_WARNING_SIZE / 2 - 84
    iy = cy - STEERING_ICON_WARNING_SIZE / 2 - 20
    gw, gh = WARNING_ICON_SIZE + 52, WARNING_ICON_SIZE + 44
    d.draw_radial_glow((ix - 26, iy - 22, gw, gh), gw * 0.55, d.glow_stops((0xFF, 0x68, 0x5B, 255), 64, 26))
    d.draw_texture_rect(d.texture("warning", WARNING_ICON_SIZE, WARNING_ICON_SIZE), ix, iy, WARNING_ICON_SIZE, WARNING_ICON_SIZE)
