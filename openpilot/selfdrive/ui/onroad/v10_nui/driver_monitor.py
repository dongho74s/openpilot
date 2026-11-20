"""Bottom-left driver monitoring badge of the v10-nui tici screen (v10-nui qt/onroad/driver_monitoring.cc)."""
import math

import pyray as rl

from openpilot.cereal import log
from openpilot.selfdrive.ui.dm_preview import sample_fresh
from openpilot.selfdrive.ui.onroad.v10_nui import draw as d
from openpilot.selfdrive.ui.onroad.v10_nui import state as s

AlertSize = log.SelfdriveState.AlertSize
VISION_POLICY = log.DriverMonitoringState.MonitoringPolicy.vision
DM_SIZE = 156
CONE_SIZE = 132
PERSON_SIZE = DM_SIZE - 16
LEFT_MARGIN = 104
BOTTOM_MARGIN = 96


class NuiDriverMonitor:
  def __init__(self):
    self.visible = False
    self.active = False
    self.enabled = False
    self.engageable = False
    self.rotation = 0.0

  @staticmethod
  def prewarm_steps() -> list:
    return [lambda: d.texture("dm_background", DM_SIZE, DM_SIZE, folder=d.DM_DIR),
            lambda: d.texture("dm_cone", CONE_SIZE, CONE_SIZE, folder=d.DM_DIR),
            lambda: d.texture("dm_person", PERSON_SIZE, PERSON_SIZE, folder=d.DM_DIR)]

  def update(self, sm, started_frame: int, now: float) -> None:
    ss = sm['selfdriveState']
    self.visible = ss.alertSize == AlertSize.none and sm.recv_frame['driverStateV2'] > started_frame
    if not self.visible:
      return
    cc, cs = sm['carControl'], sm['carState']
    lateral_only = (sm.recv_frame['carControl'] >= started_frame and sm.recv_frame['carState'] >= started_frame and
                    s.lateral_only_active(bool(ss.enabled), bool(cc.longActive), bool(cc.latActive), bool(cs.latEnabled)))
    self.enabled = bool(ss.enabled) or lateral_only
    self.engageable = bool(ss.engageable) or self.enabled

    # Stale monitoring must not keep a bright cone or an old head direction.
    fresh = all(sm.alive[svc] and sample_fresh(now, sm.logMonoTime[svc] / 1e9) for svc in ('driverMonitoringState', 'driverStateV2'))
    if not fresh:
      self.active = False
      self.rotation = 0.0
      self.enabled = self.engageable = False
      return
    dm = sm['driverMonitoringState']
    self.active = dm.activePolicy == VISION_POLICY  # v10 isActiveMode: the camera policy is in charge
    driver = sm['driverStateV2'].rightDriverData if dm.isRHD else sm['driverStateV2'].leftDriverData
    orientation = list(driver.faceOrientation)
    yaw = float(orientation[1]) if len(orientation) > 1 else 0.0
    self.rotation = s.dm_cone_rotation(yaw if math.isfinite(yaw) else 0.0)

  def draw(self, rect: rl.Rectangle) -> None:
    if not self.visible:
      return
    cx = rect.x + LEFT_MARGIN + DM_SIZE / 2
    cy = rect.y + rect.height - BOTTOM_MARGIN - DM_SIZE / 2
    opacity = 0.95 if self.active else 0.50
    white = d.color((255, 255, 255, 255), opacity)
    bg = d.texture("dm_background", DM_SIZE, DM_SIZE, folder=d.DM_DIR)
    d.draw_texture_rect(bg, cx - DM_SIZE / 2, cy - DM_SIZE / 2, DM_SIZE, DM_SIZE, white)
    cone = d.texture("dm_cone", CONE_SIZE, CONE_SIZE, folder=d.DM_DIR)
    d.draw_texture_rotated(cone, cx, cy, CONE_SIZE, CONE_SIZE, self.rotation,
                           d.color(s.dm_cone_tint(self.enabled, self.engageable), opacity), offset_y=-18)
    person = d.texture("dm_person", PERSON_SIZE, PERSON_SIZE, folder=d.DM_DIR)
    d.draw_texture_rect(person, cx - PERSON_SIZE / 2, cy - PERSON_SIZE / 2 + 4, PERSON_SIZE, PERSON_SIZE, white)
