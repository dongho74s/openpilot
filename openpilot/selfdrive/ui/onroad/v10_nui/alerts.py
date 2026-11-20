"""Alerts of the v10-nui tici screen (v10-nui qt/onroad/alerts.cc).

v10-nui also rewrote some events in selfdrived (Korean texts, FCW/LDW/resumeRequired as mid alerts). Those are
not part of this display toggle, so the same looks are produced here from the alerts WIP-FR sends.
"""
import time
from dataclasses import dataclass

import pyray as rl

from openpilot.cereal import car, log
from openpilot.selfdrive.ui.onroad.alert_renderer import ALERT_CRITICAL_REBOOT, ALERT_CRITICAL_TIMEOUT, ALERT_STARTUP_PENDING, \
  SELFDRIVE_STATE_TIMEOUT, SELFDRIVE_UNRESPONSIVE_TIMEOUT
from openpilot.selfdrive.ui.onroad.v10_nui import draw as d
from openpilot.selfdrive.ui.onroad.v10_nui import state as s
from openpilot.system.hardware import TICI
from openpilot.system.ui.lib.application import FontWeight, TextAlignment, TextAlignmentVertical, gui_app
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.widgets.label import Label

AlertSize = log.SelfdriveState.AlertSize
AlertStatus = log.SelfdriveState.AlertStatus
REVERSE = car.CarState.GearShifter.reverse

SIZE_NONE, SIZE_SMALL, SIZE_MID, SIZE_FULL = 0, 1, 2, 3
STATUS_NORMAL, STATUS_USER_PROMPT, STATUS_CRITICAL = 0, 1, 2
ALERT_HEIGHTS = {SIZE_SMALL: 271, SIZE_MID: 420}
ALERT_MARGIN = 40
ALERT_RADIUS = 30
ALERT_COLORS: dict[int, s.RGBA] = {
  STATUS_NORMAL: (0x15, 0x15, 0x15, 0xF1),
  STATUS_USER_PROMPT: (0xDA, 0x6F, 0x25, 0xF1),
  STATUS_CRITICAL: (0xC9, 0x22, 0x31, 0xF1),
}
WHITE: s.RGBA = (255, 255, 255, 255)
AUTOHOLD_ICON = 184
LEAD_DEPART_ICON = 320


@dataclass(frozen=True)
class NuiAlert:
  text1: str = ""
  text2: str = ""
  alert_type: str = ""
  size: int = SIZE_NONE
  status: int = STATUS_NORMAL


def _from_stock(alert) -> NuiAlert:
  return NuiAlert(alert.text1, alert.text2, "", int(alert.size), int(alert.status))


def alert_icon(lane_departure: bool) -> tuple[str, int, int, int, float, tuple]:
  """(icon, size, glow width, glow height, glow radius, stops) for the FCW/AEB or LDW icon: rect adjusted by -58, -44, 58, 52."""
  size = 292 if lane_departure else 348
  gw, gh = size + 116, size + 96
  glow = (0xFF, 0xC3, 0x53, 255) if lane_departure else (0xFF, 0x5D, 0x57, 255)
  return "lanecrossing" if lane_departure else "collision", size, gw, gh, gw * 0.56, d.glow_stops(glow, 82, 34)


def layout_size(alert: NuiAlert) -> int:
  """The alert size drawn here.

  v10-nui sends FCW/AEB, LDW and auto-hold as mid alerts and draws lead departed full screen. A box alert with only
  one line of text uses the one-line (small) box, so a single line never sits in a two-line box.
  """
  kind = s.alert_kind(alert.alert_type)
  if kind == s.ALERT_LEAD_DEPARTING:
    return SIZE_FULL
  if kind == s.ALERT_RESUME_REQUIRED and alert.size != SIZE_NONE:
    return SIZE_MID  # auto-hold layout (icon, timer, one description line), not a box
  size = SIZE_MID if kind in (s.ALERT_COLLISION, s.ALERT_LANE_DEPARTURE) and alert.size != SIZE_NONE else alert.size
  if size == SIZE_MID and not alert.text2.strip():
    return SIZE_SMALL
  return size


class NuiAlerts:
  def __init__(self):
    self._font_bold = gui_app.font(FontWeight.BOLD)
    self._font_semi = gui_app.font(FontWeight.SEMI_BOLD)
    self._font_regular = gui_app.font(FontWeight.NORMAL)
    self._resume_timer = s.ElapsedTimer()
    self._lead_depart = s.LeadDepartHold()
    self._started_frame = -1
    self._full_title = Label("", font_size=0, font_weight=FontWeight.BOLD, text_alignment=TextAlignment.CENTER,
                             text_alignment_vertical=TextAlignmentVertical.TOP, text_color=rl.WHITE)
    self._full_subtitle = Label("", font_size=88, text_alignment=TextAlignment.CENTER,
                                text_alignment_vertical=TextAlignmentVertical.TOP, text_color=rl.WHITE)

  def prewarm_steps(self) -> list:
    steps = [lambda: d.texture("lead_depart", LEAD_DEPART_ICON, LEAD_DEPART_ICON, keep_aspect=True),
             lambda: d.texture("autohold", AUTOHOLD_ICON, AUTOHOLD_ICON, keep_aspect=True)]
    for lane_departure in (False, True):
      name, size, gw, gh, radius, stops = alert_icon(lane_departure)
      steps += [lambda n=name, z=size: d.texture(n, z, z), lambda a=(gw, gh, radius, stops): d.bake_radial_glow(*a)]
    return steps

  def get_alert(self, sm, started_frame: int, started_time: float, now: float) -> NuiAlert | None:
    if started_frame != self._started_frame:
      self._started_frame = started_frame
      self._lead_depart.reset()
    ss = sm['selfdriveState']
    recv_frame = sm.recv_frame['selfdriveState']
    alert = None
    if recv_frame >= started_frame and ss.alertSize != AlertSize.none:
      alert = NuiAlert(str(ss.alertText1), str(ss.alertText2), str(ss.alertType), int(ss.alertSize.raw), int(ss.alertStatus.raw))

    departing = False
    if sm.recv_frame['carState'] >= started_frame and sm.recv_frame['radarState'] >= started_frame:
      cs, lead = sm['carState'], sm['radarState'].leadOne
      standstill = bool(cs.standstill) and cs.gearShifter != REVERSE
      departing = s.lead_departing_detected(standstill, bool(lead.status), float(lead.dRel), float(lead.vRel),
                                            float(lead.vLeadK), float(cs.vEgo))
    if self._lead_depart.update(departing, now) and (alert is None or s.alert_kind(alert.alert_type) == s.ALERT_RESUME_REQUIRED):
      alert = NuiAlert("", tr("Lead vehicle departed"), "leadDepartingUi", SIZE_FULL, STATUS_NORMAL)

    # Same selfdriveState timeouts as the stock tici AlertRenderer.
    if not sm.updated['selfdriveState']:
      waiting_for_startup = recv_frame < started_frame
      if waiting_for_startup and now - started_time > 5:
        return _from_stock(ALERT_STARTUP_PENDING)
      if TICI and not waiting_for_startup:
        ss_missing = now - sm.recv_time['selfdriveState']
        if ss_missing > SELFDRIVE_STATE_TIMEOUT:
          if ss.enabled and (ss_missing - SELFDRIVE_STATE_TIMEOUT) < SELFDRIVE_UNRESPONSIVE_TIMEOUT:
            return _from_stock(ALERT_CRITICAL_TIMEOUT)
          return _from_stock(ALERT_CRITICAL_REBOOT)
    return alert

  def covers_hud(self, alert: NuiAlert | None) -> bool:
    """True when an alert is drawn (DM hides under any visible alert)."""
    return alert is not None and s.alert_kind(alert.alert_type) != s.ALERT_STEER_SATURATED and layout_size(alert) != SIZE_NONE

  def render(self, rect: rl.Rectangle, alert: NuiAlert | None, now: float | None = None) -> None:
    now = time.monotonic() if now is None else now
    kind = s.alert_kind(alert.alert_type) if alert is not None else s.ALERT_NORMAL
    resume_seconds = self._resume_timer.update(kind == s.ALERT_RESUME_REQUIRED, now)
    if not self.covers_hud(alert):
      return
    assert alert is not None
    size = layout_size(alert)
    x0, y0, w, h = rect.x, rect.y, rect.width, rect.height
    height = ALERT_HEIGHTS.get(size, h)
    margin, radius = (0, 0) if size == SIZE_FULL else (ALERT_MARGIN, ALERT_RADIUS)
    r = (x0 + margin, y0 + h - height + margin, w - margin * 2, height - margin * 2)
    lead_departing = kind == s.ALERT_LEAD_DEPARTING
    resume = kind == s.ALERT_RESUME_REQUIRED
    icon_alert = kind in (s.ALERT_COLLISION, s.ALERT_LANE_DEPARTURE)
    status = STATUS_NORMAL if resume else alert.status  # v10-nui sends auto-hold as a normal alert
    alert_color = ALERT_COLORS.get(status, ALERT_COLORS[STATUS_NORMAL])

    if lead_departing:
      d.draw_vertical_gradient(x0, y0, w, h, ((0.0, (0x20, 0x22, 0x25, 188)), (0.45, (0x16, 0x18, 0x1C, 206)),
                                              (1.0, (0x10, 0x12, 0x16, 224))))
    if resume:
      ar, ag, ab, _ = alert_color
      d.draw_vertical_gradient(x0, y0, w, h, ((0.0, (ar, ag, ab, 178)), (0.45, (ar, ag, ab, 194)),
                                              (1.0, (max(ar - 18, 0), max(ag - 18, 0), max(ab - 18, 0), 212))))
    if icon_alert:
      d.draw_vertical_gradient(x0, y0, w, h, ((0.0, (0x2C, 0x33, 0x3A, 0x74)), (0.25, (0x25, 0x2B, 0x32, 0x98)),
                                              (0.55, (0x1E, 0x24, 0x2A, 0xBF)), (1.0, (0x16, 0x1A, 0x1F, 0xD8))))
    if not resume and not lead_departing:
      # The darkening gradient is composited under the box (DestinationOver), then the box itself.
      d.draw_rounded_rect(*r, radius, (0, 0, 0, 51))
      d.draw_rounded_rect(*r, radius, alert_color)

    if icon_alert:
      self._draw_alert_icon(x0, y0, w, kind == s.ALERT_LANE_DEPARTURE, now)
    if lead_departing:
      self._draw_lead_departing(x0, y0, w, h, alert)
      return

    cy = r[1] + r[3] / 2
    if size == SIZE_SMALL:
      d.draw_text(self._font_semi, alert.text1, 74, r, WHITE, d.CENTER, d.CENTER)
    elif size == SIZE_MID:
      if resume:
        self._draw_auto_hold(x0, y0, w, h, alert, resume_seconds)
      else:
        d.draw_text(self._font_bold, alert.text1, 88, (x0, cy - 125, w, 150), WHITE, d.CENTER, d.TOP)
        d.draw_text(self._font_regular, alert.text2, 66, (x0, cy + 21, w, 90), WHITE, d.CENTER, d.TOP)
    elif size == SIZE_FULL:
      is_long = len(alert.text1) > 15
      self._full_title.set_font_size(132 if is_long else 177)
      self._full_title.set_text(alert.text1)
      self._full_title.render(rl.Rectangle(x0, r[1] + (240 if is_long else 270), w, 600))
      self._full_subtitle.set_text(alert.text2)
      self._full_subtitle.render(rl.Rectangle(x0, y0 + r[3] - (361 if is_long else 420), w, 300))

  def _draw_alert_icon(self, x0: float, y0: float, w: float, lane_departure: bool, now: float) -> None:
    name, size, gw, gh, radius, stops = alert_icon(lane_departure)
    interval_ms = 320 if lane_departure else 120
    if (d.blink_ms() // interval_ms) % 2 != 0:
      return
    ix, iy = x0 + (w - size) / 2, y0 + (188 if lane_departure else 164)
    d.draw_radial_glow((ix - 58, iy - 44, gw, gh), radius, stops)
    d.draw_texture_rect(d.texture(name, size, size), ix, iy, size, size)

  def _draw_lead_departing(self, x0: float, y0: float, w: float, h: float, alert: NuiAlert) -> None:
    tex = d.texture("lead_depart", LEAD_DEPART_ICON, LEAD_DEPART_ICON, keep_aspect=True)
    box_x = x0 + (w - LEAD_DEPART_ICON) / 2
    box_y = y0 + (h - LEAD_DEPART_ICON) / 2 - 70
    d.draw_texture_rect(tex, box_x + (LEAD_DEPART_ICON - tex.width) / 2, box_y + (LEAD_DEPART_ICON - tex.height) / 2,
                        tex.width, tex.height)
    d.draw_text(self._font_bold, alert.text2, 82, (x0, box_y + LEAD_DEPART_ICON - 1 + 52, w, 110), WHITE, d.CENTER, d.TOP)

  def _draw_auto_hold(self, x0: float, y0: float, w: float, h: float, alert: NuiAlert, seconds: int) -> None:
    gap, timer_w = 28, 260
    group_left = x0 + (w - (AUTOHOLD_ICON + gap + timer_w)) / 2
    group_top = y0 + (h - 1) // 2 - 150
    tex = d.texture("autohold", AUTOHOLD_ICON, AUTOHOLD_ICON, keep_aspect=True)
    d.draw_texture_rect(tex, group_left, group_top + (AUTOHOLD_ICON - tex.height) / 2, tex.width, tex.height)
    d.draw_text(self._font_bold, s.format_minutes_seconds(seconds), 112,
                (group_left + AUTOHOLD_ICON - 1 + gap, group_top + 10, timer_w, AUTOHOLD_ICON - 20), WHITE, d.CENTER, d.CENTER)
    description = alert.text2 or alert.text1  # WIP-FR sends one line ("Press Resume to Exit Standstill")
    d.draw_text(self._font_regular, description, 66, (x0, group_top + AUTOHOLD_ICON + 26, w, 90), WHITE, d.CENTER, d.TOP)
