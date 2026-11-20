"""Everything drawn over the road camera on the v10-nui tici screen (v10-nui annotated_camera.cc + alerts)."""
import time

import pyray as rl

from openpilot.selfdrive.ui.onroad.v10_nui import draw as d
from openpilot.selfdrive.ui.onroad.v10_nui import state as s
from openpilot.selfdrive.ui.onroad.v10_nui.alerts import NuiAlerts
from openpilot.selfdrive.ui.onroad.v10_nui.driver_monitor import NuiDriverMonitor
from openpilot.selfdrive.ui.onroad.v10_nui.hud import NuiHud
from openpilot.selfdrive.ui.onroad.v10_nui.model import NuiModel

CHROME_GRADIENT = ((0.0, (0x0E, 0x1C, 0x29, 0x34)), (0.30, (0x12, 0x2A, 0x3B, 0x18)), (1.0, (0x12, 0x2A, 0x3B, 0x00)))
FOOTER_GRADIENT = ((0.0, (0x05, 0x09, 0x0F, 0x00)), (0.35, (0x05, 0x09, 0x0F, 0x34)), (0.72, (0x05, 0x09, 0x0F, 0x92)),
                   (1.0, (0x05, 0x09, 0x0F, 0xD8)))
HUD_GRADIENT = ((0.0, (0x03, 0x05, 0x09, 0x00)), (0.22, (0x03, 0x05, 0x09, 0x30)), (0.58, (0x03, 0x05, 0x09, 0x88)),
                (1.0, (0x03, 0x05, 0x09, 0xE8)))
BLINDSPOT_W, BLINDSPOT_H = 196, 224
SCREEN_CORNER_RADIUS = 60  # the stock C3 border's inner rounding (roundness 0.12 of a 1080px screen)
SCREEN_CORNER_COLOR: s.RGBA = (0, 0, 0, 255)
BLINDSPOT_GLOW: s.RGBA = (0xFF, 0x67, 0x4A, 255)


def blindspot_icon(name: str):
  """(texture, glow width, glow height, radius, stops) for one blind-spot icon: rect adjusted by -34, -28, 34, 28."""
  tex = d.texture(name, BLINDSPOT_W, BLINDSPOT_H, keep_aspect=True)
  gw, gh = tex.width + 68, tex.height + 56
  return tex, gw, gh, gw * 0.56, d.glow_stops(BLINDSPOT_GLOW, 82, 34)


def status_name(status) -> str:
  return str(getattr(status, "value", status))


def lateral_only(sm, started_frame: int) -> bool:
  if not all(sm.recv_frame[svc] >= started_frame for svc in ('selfdriveState', 'carControl', 'carState')):
    return False
  ss, cc, cs = sm['selfdriveState'], sm['carControl'], sm['carState']
  return s.lateral_only_active(bool(ss.enabled), bool(cc.longActive), bool(cc.latActive), bool(cs.latEnabled))


class NuiOnroad:
  def __init__(self, params):
    self.model = NuiModel(params)
    self.hud = NuiHud(params)
    self.dm = NuiDriverMonitor()
    self.alerts = NuiAlerts()
    # Decode icons and bake glows one per frame from the first frame on, instead of at first use mid-drive.
    self._prewarm = (self.hud.prewarm_steps() + self.alerts.prewarm_steps() + self.dm.prewarm_steps() +
                     [lambda n=n: d.bake_radial_glow(*blindspot_icon(n)[1:]) for n in ("blindspot_left", "blindspot_right")] +
                     [lambda: d.bake_rounded_corner(SCREEN_CORNER_RADIUS, SCREEN_CORNER_COLOR)])

  def reset(self) -> None:
    """Back from another NewUi value: drop timers and smoothing that kept running only while this was drawn."""
    self.hud.reset()
    self.alerts.reset()
    self.model.reset()

  def set_transform(self, transform) -> None:
    self.model.set_transform(transform)

  def camera_filter(self, ui) -> tuple[float, float]:
    return s.camera_filter(ui.started, status_name(ui.status), lateral_only(ui.sm, ui.started_frame))

  def close(self) -> None:
    self.hud.close()
    d.release_baked_textures()

  def render(self, rect: rl.Rectangle, ui, draw_model: bool = True, now: float | None = None) -> dict[str, float]:
    """Draw the overlay; returns per-part milliseconds for uiDebug."""
    now = time.monotonic() if now is None else now
    sm = ui.sm
    status = status_name(ui.status)
    timings = {}

    x, y, w, h = rect.x, rect.y, rect.width, rect.height
    d.draw_vertical_gradient(x, y, w, h * 0.55, CHROME_GRADIENT)
    d.draw_vertical_gradient(x, y + h * 0.56, w, h * 0.44, FOOTER_GRADIENT)
    d.draw_vertical_gradient(x, y + h * 0.66, w, h * 0.34, HUD_GRADIENT)

    t = time.monotonic()
    if draw_model:
      self.model.draw(rect, sm, status, ui.started_frame, now, ui.is_metric)
    timings['model'] = (time.monotonic() - t) * 1000.0

    t = time.monotonic()
    self.dm.update(sm, ui.started_frame, now)
    self.dm.draw(rect)
    timings['driver_state'] = (time.monotonic() - t) * 1000.0

    t = time.monotonic()
    self.hud.update(sm, ui.started_frame, status, ui.is_metric, now)
    self.hud.draw(rect)
    self._draw_blindspots(rect, sm, ui.started_frame)
    timings['hud'] = (time.monotonic() - t) * 1000.0

    t = time.monotonic()
    alert = self.alerts.get_alert(sm, ui.started_frame, ui.started_time, now)
    self.alerts.render(rect, alert, now)
    timings['alert'] = (time.monotonic() - t) * 1000.0
    if self._prewarm:
      self._prewarm.pop(0)()
    return timings

  @staticmethod
  def draw_screen_corners(screen: rl.Rectangle) -> None:
    """Round the four corners of the whole screen (sidebar included when it is open), over everything drawn."""
    d.draw_rounded_corners((screen.x, screen.y, screen.width, screen.height), SCREEN_CORNER_RADIUS, SCREEN_CORNER_COLOR)

  def _draw_blindspots(self, rect: rl.Rectangle, sm, started_frame: int) -> None:
    if sm.recv_frame['carState'] < started_frame:
      return
    cs = sm['carState']
    left, right = bool(cs.leftBlindspot), bool(cs.rightBlindspot)
    if not (left or right):
      return
    blink_on = s.blindspot_blink_visible(d.blink_ms())
    y = rect.y + (rect.height - 1) // 2 - 180
    for show, blink, name, x in ((left, bool(cs.leftBlinker), "blindspot_left", rect.x + 44),
                                 (right, bool(cs.rightBlinker), "blindspot_right", rect.x + rect.width - 40 - BLINDSPOT_W)):
      if not show or (blink and not blink_on):
        continue
      tex, gw, gh, radius, stops = blindspot_icon(name)
      d.draw_radial_glow((x - 34, y - 28, gw, gh), radius, stops)
      d.draw_texture_rect(tex, x, y, tex.width, tex.height)
