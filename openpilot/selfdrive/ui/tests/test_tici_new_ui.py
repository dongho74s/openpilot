"""NewUi 1 on C3/C3X: the v10-nui screen port (selfdrive/ui/onroad/v10_nui) and how the tici view routes to it.

State rules run on real code; drawing code runs against a recording raylib stand-in. Full renders of the real view
are produced by preview_tici_new_ui.py.
"""
import ast
import importlib
import sys
from collections import namedtuple
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

from openpilot.cereal import car, log
from openpilot.selfdrive.ui.onroad.v10_nui import state as s

UI_DIR = Path(__file__).parents[1]
ROAD_VIEW = UI_DIR / "onroad" / "augmented_road_view.py"
CAMERA_VIEW = UI_DIR / "onroad" / "cameraview.py"
STARTED = 100


# ---------- pure rules ----------

def test_lateral_only_and_camera_filter():
  assert s.lateral_only_active(False, False, True, False)
  assert s.lateral_only_active(False, False, False, True)
  assert not s.lateral_only_active(True, True, True, True)
  assert not s.lateral_only_active(False, True, True, False)
  assert s.camera_filter(True, s.STATUS_DISENGAGED, False) == (1.0, pytest.approx(0.38))
  assert s.camera_filter(True, s.STATUS_DISENGAGED, True) == (0.0, 0.0)
  assert s.camera_filter(True, s.STATUS_ENGAGED, False) == (0.0, 0.0)
  assert s.camera_filter(False, s.STATUS_DISENGAGED, False) == (0.0, 0.0)


def test_wheel_and_lfa_colours_follow_v10():
  assert s.wheel_style(0.0, False, False) == ((0x0A, 0x10, 0x16, 0xA8), (0x9F, 0xA8, 0xB2, 0xE6))
  assert s.wheel_style(0.0, True, False)[1] == (0xF4, 0xF7, 0xFB, 0xF4)
  assert s.wheel_style(0.65, True, False)[1] == (0xFF, 0xC7, 0x58, 0xF6)
  assert s.wheel_style(1.0, True, False)[1] == (0xFF, 0x5D, 0x57, 0xF8)
  assert s.wheel_style(1.0, True, True)[1] == s.OVERRIDE_BLUE
  assert s.lfa_style(True, True, True, True) == (s.OVERRIDE_BLUE, s.OVERRIDE_BLUE)
  assert s.lfa_style(False, True, True, True) == (s.LATERAL_ONLY_MINT, s.LATERAL_ONLY_MINT)
  assert s.lfa_style(False, False, True, True) == (s.ENGAGED_GREEN, s.ENGAGED_GREEN)
  assert s.lfa_style(False, False, False, True)[1] is None
  assert s.lfa_style(False, False, False, False) == ((0x92, 0x9D, 0xA8, 0xE6), None)


def test_torque_from_every_lateral_controller():
  for which in ("torqueState", "pidState", "angleState", "debugState"):
    controls = log.ControlsState.new_message()
    controls.lateralControlState.init(which)
    getattr(controls.lateralControlState, which).output = -0.4
    assert s.steering_torque_pct(controls.lateralControlState) == pytest.approx(0.4)
    getattr(controls.lateralControlState, which).saturated = True
    assert s.steering_torque_pct(controls.lateralControlState) == 1.0


def test_set_speed_units_and_states():
  assert s.set_speed(100.0, 0.0, True) == (100.0, True, True)
  speed, cruise_set, available = s.set_speed(100.0, 0.0, False)
  assert speed == pytest.approx(62.1371) and cruise_set and available
  assert s.set_speed(0.0, 255.0, True)[1:] == (False, True)
  assert s.set_speed(-1.0, 0.0, True)[2] is False
  assert s.set_speed(0.0, 80.0, True)[0] == 80.0


def test_standstill_timer_appears_after_a_minute_and_resets_on_auto_hold():
  timer = s.StandstillTimer()
  assert timer.update(True, False, 0.0) == 0
  assert timer.update(True, False, 59.9) == 0
  assert timer.update(True, False, 75.2) == 75
  assert timer.update(True, True, 76.0) == 0
  assert timer.update(True, False, 77.0) == 0
  assert timer.update(False, False, 200.0) == 0
  assert s.format_minutes_seconds(75) == "1:15"
  assert s.format_minutes_seconds(3600) == "60:00"


def test_lead_departed_detection_and_three_second_hold():
  assert s.lead_departing_detected(True, True, 8.0, 1.0, 0.0, 0.0)
  assert s.lead_departing_detected(True, True, 8.0, 0.0, 1.0, 0.0)
  assert not s.lead_departing_detected(False, True, 8.0, 1.0, 1.0, 0.0)
  assert not s.lead_departing_detected(True, True, 0.5, 1.0, 1.0, 0.0)
  assert not s.lead_departing_detected(True, True, 31.0, 1.0, 1.0, 0.0)
  assert not s.lead_departing_detected(True, True, 8.0, 0.5, 0.5, 0.0)
  hold = s.LeadDepartHold()
  assert hold.update(True, 10.0)
  assert hold.update(False, 12.9)
  assert not hold.update(False, 13.0)


@pytest.mark.parametrize(("alert_type", "kind"), [
  ("steerSaturated/warning", s.ALERT_STEER_SATURATED), ("resumeRequired/warning", s.ALERT_RESUME_REQUIRED),
  ("fcw/permanent", s.ALERT_COLLISION), ("stockAeb/permanent", s.ALERT_COLLISION), ("ldw/permanent", s.ALERT_LANE_DEPARTURE),
  ("leadDepartingUi", s.ALERT_LEAD_DEPARTING), ("promptDriverDistracted/warning", s.ALERT_NORMAL), ("", s.ALERT_NORMAL),
])
def test_alert_kinds(alert_type, kind):
  assert s.alert_kind(alert_type) == kind


@pytest.mark.parametrize(("direction", "skipped", "highlighted"), [(s.DIRECTION_LEFT, 1, (0, 2)), (s.DIRECTION_RIGHT, 2, (1, 3))])
def test_lane_change_hides_the_crossed_line_and_highlights_both_lanes(direction, skipped, highlighted):
  for state in (s.LANE_CHANGE_STARTING, s.LANE_CHANGE_FINISHING):
    styles = [s.lane_line_style(i, state, direction, False) for i in range(4)]
    assert styles[skipped] == s.LANE_SKIP
    assert [i for i, style in enumerate(styles) if style == s.LANE_CHANGE] == list(highlighted)
  assert [s.lane_line_style(i, 1, direction, False) for i in range(4)] == [s.LANE_PLAIN] * 4  # preLaneChange
  assert [s.lane_line_style(i, s.LANE_CHANGE_STARTING, 0, False) for i in range(4)] == [s.LANE_PLAIN] * 4


def test_lane_mode_widens_and_greens_the_ego_lane_edges():
  assert [s.lane_line_style(i, 0, 0, True) for i in range(4)] == [s.LANE_PLAIN, s.LANE_MODE, s.LANE_MODE, s.LANE_PLAIN]
  assert s.lane_line_half_width(1.0, 1, True) == pytest.approx(0.0425)
  assert s.lane_line_half_width(0.0, 1, True) == pytest.approx(0.034)
  assert s.lane_line_half_width(1.0, 0, True) == pytest.approx(0.03125)
  assert s.lane_line_half_width(0.0, 0, False) == pytest.approx(0.028)
  fill, outline, width = s.lane_line_colors(s.LANE_MODE, 1.0)
  assert fill[1] > 200 and outline is not None and width == 10.0
  assert s.lane_line_colors(s.LANE_PLAIN, 1.0) == ((255, 255, 255, 178), None, 0.0)


def test_path_colours():
  assert s.path_base_colors(True, True)[0][1] == s.with_alpha(s.LATERAL_ONLY_MINT, 110)
  assert s.path_base_colors(False, True)[2][1][3] == 0
  assert s.path_base_colors(False, False) is None
  blend = s.ThrottleBlend()
  assert blend.update(True, 0.05) == s.THROTTLE_COLORS
  midway = blend.update(False, 0.25)
  assert midway not in (s.THROTTLE_COLORS, s.NO_THROTTLE_COLORS)
  assert blend.update(False, 0.25) == s.NO_THROTTLE_COLORS
  assert s.gray_tone((255, 0, 0, 77)) == (63, 63, 63, 77)
  assert s.THROTTLE_COLORS[0] == (13, 248, 122, 102)  # same as the stock C3 path colour


def test_experimental_path_stops_follow_qt_skipping():
  ys = np.linspace(1000, 300, 12)
  stops = s.experimental_path_stops(ys, np.full(12, -2.0), 0.0, 1080.0)
  points = [p for p, _ in stops]
  assert points == sorted(points) and len(points) == 7  # indices 0, 2, 4, 6, 8, 10 and the last one
  assert all(c[0] > c[1] for _, c in stops)  # braking reads red
  off_screen = s.experimental_path_stops(np.array([2000.0, 900.0, 800.0]), np.zeros(3), 0.0, 1080.0)
  assert [round(p, 3) for p, _ in off_screen] == [round(180 / 1080, 3), round(280 / 1080, 3)]


def test_dm_cone_rotation_and_tint():
  assert s.dm_cone_rotation(0.2) == pytest.approx(-11.459, abs=1e-3)
  assert s.dm_cone_rotation(-2.0) == s.CONE_MAX_ROTATION_DEG
  assert s.dm_cone_rotation(float('nan')) == 0.0
  assert s.dm_cone_tint(True, True) == s.ENGAGED_GREEN
  assert s.dm_cone_tint(False, False) == (0x8A, 0x8F, 0x96, 0xFF)


def test_blinks():
  assert s.turn_signal_on(0) and s.turn_signal_on(419) and not s.turn_signal_on(420) and s.turn_signal_on(800)
  assert s.blindspot_blink_visible(0) and not s.blindspot_blink_visible(120)


# ---------- drawing code with a recording raylib ----------

class SM(dict):
  def __init__(self, **services):
    super().__init__(services)
    names = list(services)
    self.frame = STARTED + 5
    self.recv_frame = dict.fromkeys(names, STARTED + 5)
    self.updated = dict.fromkeys(names, True)
    self.alive = dict.fromkeys(names, True)
    self.valid = dict.fromkeys(names, True)
    self.recv_time = dict.fromkeys(names, 50.0)
    self.logMonoTime = dict.fromkeys(names, int(50.0 * 1e9))


def make_sm():
  model = log.ModelDataV2.new_message()
  for field in ("position", "acceleration"):
    getattr(model, field).x = np.linspace(0, 90, 33).tolist()
    getattr(model, field).y = [0.0] * 33
    getattr(model, field).z = [0.0] * 33
  model.init("laneLines", 4)
  model.init("roadEdges", 2)
  for line in list(model.laneLines) + list(model.roadEdges):
    line.x, line.y, line.z = np.linspace(0, 90, 33).tolist(), [1.8] * 33, [0.0] * 33
  model.laneLineProbs = [0.5, 0.9, 0.9, 0.5]
  model.roadEdgeStds = [0.3, 0.3]
  calib = log.LiveCalibrationData.new_message(height=[1.22])
  cs = car.CarState.new_message(vEgo=20.0, vEgoCluster=20.0, vCruiseCluster=100.0, steeringAngleDeg=12.0)
  ss = log.SelfdriveState.new_message(enabled=True, engageable=True, personality="standard")
  cc = car.CarControl.new_message(latActive=True, longActive=True)
  dm = log.DriverMonitoringState.new_message(activePolicy="vision")
  ds = log.DriverStateV2.new_message(frameId=10)
  ds.leftDriverData.faceOrientation = [0.0, 0.2, 0.0]
  return SM(carState=cs, selfdriveState=ss, carControl=cc, controlsState=log.ControlsState.new_message(),
            carrotMan=log.Event.new_message().init("carrotMan"),
            modelV2=model, liveCalibration=calib, carParams=car.CarParams.new_message(openpilotLongitudinalControl=True),
            radarState=log.RadarState.new_message(), longitudinalPlan=log.LongitudinalPlan.new_message(allowThrottle=True),
            driverMonitoringState=dm, driverStateV2=ds)


@pytest.fixture
def nui(monkeypatch):
  calls = []
  Color = namedtuple("Color", "r g b a")
  rect_t = namedtuple("Rectangle", "x y width height")

  def rec(name):
    return lambda *args, **kwargs: calls.append((name, *args))

  rl = NS(Rectangle=rect_t, Vector2=namedtuple("Vector2", "x y"), Color=Color, WHITE=Color(255, 255, 255, 255),
          BLACK=Color(0, 0, 0, 255), RL_TRIANGLES=4, Texture=object, Font=object)
  for name in ("draw_texture_pro", "draw_text_ex", "draw_circle", "draw_rectangle_rounded", "draw_rectangle_rounded_lines_ex",
               "draw_rectangle_gradient_v", "draw_line_ex", "rl_begin", "rl_end", "rl_color4ub", "rl_vertex2f", "draw_rectangle_rec"):
    setattr(rl, name, rec(name))
  font = NS(name="font")
  requests = []

  def texture(path, w, h, **kw):
    requests.append((path, w, h, kw.get("keep_aspect_ratio", True)))
    return NS(path=path, width=w, height=h)
  gui = NS(texture=texture, font=lambda weight: font)

  class Label:
    def __init__(self, *a, **k):
      self.text = ""
    def set_text(self, text):
      self.text = text
    def set_font_size(self, size):
      self.size = size
    def render(self, rect):
      calls.append(("label", self.text, rect))

  def draw_polygon(rect, points, color=None, gradient=None):
    calls.append(("polygon", len(points), color, gradient))

  class Gradient:
    def __init__(self, start, end, colors, stops):
      self.start, self.end, self.colors, self.stops = start, end, colors, stops

  ribbon = np.array([[900, 1000], [950, 600], [1000, 500], [1100, 500], [1150, 600], [1200, 1000]], dtype=np.float32)
  stubs = {
    "pyray": rl,
    "openpilot.system.ui.lib.application": NS(gui_app=gui, FontWeight=NS(BOLD="bold", SEMI_BOLD="semi", NORMAL="normal"),
                                              ASSETS_DIR=Path("/assets"), TextAlignment=NS(CENTER=1),
                                              TextAlignmentVertical=NS(TOP=0)),
    "openpilot.system.ui.lib.text_measure": NS(measure_text_cached=lambda f, t, size, spacing=0: rl.Vector2(len(t) * size / 2, size)),
    "openpilot.system.ui.widgets.label": NS(Label=Label),
    "openpilot.system.ui.lib.multilang": NS(tr=lambda text: text),
    "openpilot.system.ui.lib.shader_polygon": NS(draw_polygon=draw_polygon, Gradient=Gradient),
    "openpilot.system.ui.lib.native_geometry": NS(project_ribbon=lambda *a, **k: ribbon.copy()),
    "openpilot.system.ui.lib": NS(native_geometry=NS(project_ribbon=lambda *a, **k: ribbon.copy())),
    "openpilot.system.hardware": NS(TICI=True),
    "openpilot.selfdrive.locationd.calibrationd": NS(HEIGHT_INIT=np.array([1.22])),
    "openpilot.selfdrive.ui.onroad.alert_renderer": NS(
      ALERT_STARTUP_PENDING=NS(text1="openpilot Unavailable", text2="Waiting to start", size=2, status=0),
      ALERT_CRITICAL_TIMEOUT=NS(text1="TAKE CONTROL IMMEDIATELY", text2="System Unresponsive", size=3, status=2),
      ALERT_CRITICAL_REBOOT=NS(text1="System Unresponsive", text2="Reboot Device", size=2, status=0),
      SELFDRIVE_STATE_TIMEOUT=5, SELFDRIVE_UNRESPONSIVE_TIMEOUT=10),
  }
  for name, value in stubs.items():
    monkeypatch.setitem(sys.modules, name, value)
  for name in [m for m in sys.modules if m.startswith("openpilot.selfdrive.ui.onroad.v10_nui.") and not m.endswith(".state")]:
    monkeypatch.delitem(sys.modules, name)
  modules = {name: importlib.import_module(f"openpilot.selfdrive.ui.onroad.v10_nui.{name}")
             for name in ("draw", "hud", "model", "alerts", "driver_monitor", "view")}
  for name in modules:
    monkeypatch.delitem(sys.modules, f"openpilot.selfdrive.ui.onroad.v10_nui.{name}")
  monkeypatch.setattr(modules["draw"], "blink_ms", lambda: 0)  # every blink in its visible phase
  baked = []

  def fake_baked(key, make):
    pixels = make()
    baked.append((key, pixels))
    requests.append(key)
    return NS(path=f"baked:{key[0]}", width=pixels.shape[1], height=pixels.shape[0])
  monkeypatch.setattr(modules["draw"], "_baked_texture", fake_baked)
  params = NS(get_int=lambda key: {"ShowPathEnd": 1, "LongitudinalPersonality": 2}.get(key, 0))
  monkeypatch.setattr(modules["draw"].ThresholdTexture, "get",
                      lambda self: requests.append(("threshold", self._path)) or NS(path=self._path, width=72, height=72))
  return NS(m=modules, calls=calls, rl=rl, params=params, rect=rect_t(0, 0, 2160, 1080), baked=baked, requests=requests)


def textures(calls):
  return [c[1].path.rsplit("/", 1)[-1] for c in calls if c[0] == "draw_texture_pro"]


def texts(calls):
  return [c[2] for c in calls if c[0] == "draw_text_ex"]


def test_hud_engaged_layout(nui):
  hud = nui.m["hud"].NuiHud(nui.params)
  sm = make_sm()
  hud.update(sm, STARTED, s.STATUS_ENGAGED, True, 100.0)
  assert (hud.speed, hud.set_speed, hud.gap_level, hud.steering_angle) == (pytest.approx(72.0), 100.0, 2, -12.0)
  hud.draw(nui.rect)
  assert texts(nui.calls) == ["SET", "100", "72", "KM/H"]
  assert textures(nui.calls) == ["gap2.png", "gap2.png", "baked:glow", "lfa.png", "steeringwheel.png"]
  wheel = [c for c in nui.calls if c[0] == "draw_texture_pro"][-1]
  assert wheel[3] == (2160 - 104 - 78, 1080 - 96 - 78, 150, 150) and wheel[5] == -12.0
  assert not any(c[0] == "rl_begin" for c in nui.calls)  # glows are baked textures, not per-frame triangles


def test_hud_stale_car_state_and_parked_timer(nui):
  hud = nui.m["hud"].NuiHud(nui.params)
  sm = make_sm()
  sm.recv_frame["carState"] = STARTED - 1
  hud.update(sm, STARTED, s.STATUS_DISENGAGED, True, 100.0)
  assert (hud.speed, hud.is_cruise_set, hud.set_speed) == (0.0, False, 255.0)
  sm = make_sm()
  sm["carState"].standstill = True
  hud.update(sm, STARTED, s.STATUS_ENGAGED, True, 100.0)
  hud.update(sm, STARTED, s.STATUS_ENGAGED, True, 175.0)
  hud.draw(nui.rect)
  assert "1:15" in texts(nui.calls) and "KM/H" not in texts(nui.calls)
  sm["selfdriveState"].alertType = "resumeRequired/warning"
  hud.update(sm, STARTED, s.STATUS_ENGAGED, True, 176.0)
  assert hud.standstill_seconds == 0


def test_hud_override_signals_and_steer_limit(nui):
  hud = nui.m["hud"].NuiHud(nui.params)
  sm = make_sm()
  sm["selfdriveState"].state = "overriding"
  sm["carState"].gasPressed = True
  sm["carState"].leftBlinker = True
  sm["selfdriveState"].alertType = "steerSaturated/warning"
  hud.update(sm, STARTED, s.STATUS_OVERRIDE, False, 100.0)
  assert hud.long_override and not hud.lat_override and hud.steer_limit_warning
  hud._turn_left.get = lambda: NS(path="turn_signal_left.png", width=72, height=72)
  hud.draw(nui.rect)
  assert "MPH" in texts(nui.calls) and "62" in texts(nui.calls)
  names = [n for n in textures(nui.calls) if not n.startswith("baked:")]
  assert names[0] == "turn_signal_left.png" and "warning.png" in names
  lfa = next(c for c in nui.calls if c[0] == "draw_texture_pro" and c[1].path.endswith("lfa.png"))
  assert lfa[-1] == nui.rl.Color(*s.OVERRIDE_BLUE)
  wheel = [c for c in nui.calls if c[0] == "draw_texture_pro"][-1]
  assert wheel[1].width == 178


def test_dm_hides_under_alerts_and_drops_stale_pose(nui):
  dm = nui.m["driver_monitor"].NuiDriverMonitor()
  sm = make_sm()
  dm.update(sm, STARTED, 50.1)
  assert dm.visible and dm.active and dm.enabled and dm.rotation == pytest.approx(-11.459, abs=1e-3)
  dm.draw(nui.rect)
  assert textures(nui.calls) == ["dm_background.png", "dm_cone.png", "dm_person.png"]
  cone = nui.calls[1]
  assert cone[5] == pytest.approx(-11.459, abs=1e-3) and cone[4] == (66, 66 + 18)
  dm.update(sm, STARTED, 51.0)  # 1 s old samples
  assert dm.visible and not dm.active and dm.rotation == 0.0 and not dm.enabled
  sm["selfdriveState"].alertSize = "small"
  dm.update(sm, STARTED, 50.1)
  assert not dm.visible


def test_alerts_layouts(nui):
  alerts_mod = nui.m["alerts"]
  alerts = alerts_mod.NuiAlerts()
  sm = make_sm()
  ss = sm["selfdriveState"]
  ss.alertType, ss.alertSize, ss.alertStatus, ss.alertText1, ss.alertText2 = "fcw/permanent", "full", "critical", "BRAKE!", "Risk"
  alert = alerts.get_alert(sm, STARTED, 0.0, 50.0)
  assert alerts_mod.layout_size(alert) == alerts_mod.SIZE_MID and alerts.covers_hud(alert)
  alerts.render(nui.rect, alert, 50.0)
  assert "collision.png" in textures(nui.calls) and texts(nui.calls)[:2] == ["BRAKE!", "Risk"]
  nui.calls.clear()
  ss.alertType = "steerSaturated/warning"
  alert = alerts.get_alert(sm, STARTED, 0.0, 50.0)
  assert not alerts.covers_hud(alert)
  alerts.render(nui.rect, alert, 50.0)
  assert nui.calls == []


def test_auto_hold_timer_and_lead_departed(nui):
  alerts = nui.m["alerts"].NuiAlerts()
  sm = make_sm()
  ss, cs, lead = sm["selfdriveState"], sm["carState"], sm["radarState"].leadOne
  ss.alertType, ss.alertSize, ss.alertStatus, ss.alertText1 = "resumeRequired/warning", "small", "userPrompt", "Press Resume"
  cs.standstill, cs.vEgo = True, 0.0
  alerts.render(nui.rect, alerts.get_alert(sm, STARTED, 0.0, 10.0), 10.0)
  nui.calls.clear()
  alerts.render(nui.rect, alerts.get_alert(sm, STARTED, 0.0, 52.5), 52.5)
  assert textures(nui.calls) == ["autohold.png"] and texts(nui.calls) == ["0:42", "Press Resume"]
  lead.status, lead.dRel, lead.vRel = True, 8.0, 1.5
  alert = alerts.get_alert(sm, STARTED, 0.0, 53.0)
  assert alert.text2 == "Lead vehicle departed" and alert.alert_type == "leadDepartingUi"
  lead.vRel = 0.0
  assert alerts.get_alert(sm, STARTED, 0.0, 55.9).alert_type == "leadDepartingUi"
  assert alerts.get_alert(sm, STARTED, 0.0, 56.1).alert_type == "resumeRequired/warning"


def test_alert_timeouts_match_stock(nui):
  alerts = nui.m["alerts"].NuiAlerts()
  sm = make_sm()
  sm.updated["selfdriveState"] = False
  sm.recv_frame["selfdriveState"] = STARTED - 1
  assert alerts.get_alert(sm, STARTED, 0.0, 6.0).text1 == "openpilot Unavailable"
  sm.recv_frame["selfdriveState"] = STARTED + 1
  assert alerts.get_alert(sm, STARTED, 0.0, 56.0).text1 == "TAKE CONTROL IMMEDIATELY"
  assert alerts.get_alert(sm, STARTED, 0.0, 70.0).text1 == "System Unresponsive"


def polygons(calls):
  return [c for c in calls if c[0] == "polygon"]


def test_model_hides_lanes_when_disengaged_and_greys_the_path(nui):
  model = nui.m["model"].NuiModel(nui.params)
  model.set_transform(np.eye(3, dtype=np.float32))
  sm = make_sm()
  model.draw(nui.rect, sm, s.STATUS_ENGAGED, STARTED, 10.0)
  engaged = polygons(nui.calls)
  assert len(engaged) == 7  # 4 lane lines, 2 road edges, path
  nui.calls.clear()
  sm["selfdriveState"].enabled = False
  sm["carControl"].latActive = sm["carControl"].longActive = False
  model.draw(nui.rect, sm, s.STATUS_DISENGAGED, STARTED, 10.05)
  path = polygons(nui.calls)
  assert len(path) == 1
  assert all(c.r == c.g == c.b for c in path[0][3].colors)


def test_model_lane_change_and_lead_box(nui):
  model = nui.m["model"].NuiModel(nui.params)
  model.set_transform(np.eye(3, dtype=np.float32))
  sm = make_sm()
  sm["modelV2"].meta.laneChangeState = "laneChangeStarting"
  sm["modelV2"].meta.laneChangeDirection = "right"
  lead = sm["radarState"].leadOne
  lead.status, lead.dRel, lead.radar, lead.radarTrackId = True, 30.0, True, 0
  model.draw(nui.rect, sm, s.STATUS_ENGAGED, STARTED, 10.0)
  assert len(polygons(nui.calls)) == 6  # crossed line skipped
  outlines = [c for c in nui.calls if c[0] == "draw_line_ex"]
  assert outlines and all(c[4] == nui.rl.Color(0x53, 0xEE, 0xB6, 128) for c in outlines)
  box = next(c for c in nui.calls if c[0] == "draw_rectangle_rounded_lines_ex")
  assert box[-1] == nui.rl.Color(255, 0, 0, 255)  # SCC radar lead
  assert "30.0" in texts(nui.calls)


def test_model_skips_without_camera_transform(nui):
  model = nui.m["model"].NuiModel(nui.params)
  model.draw(nui.rect, make_sm(), s.STATUS_ENGAGED, STARTED, 10.0)
  assert polygons(nui.calls) == []


# ---------- routing in the tici view and the camera filter ----------

def _function(tree, name, cls=None):
  scope = tree.body if cls is None else next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == cls).body
  return next(n for n in scope if isinstance(n, ast.FunctionDef) and n.name == name)


def test_only_new_ui_1_takes_the_v10_path_before_any_carrot_drawing():
  tree = ast.parse(ROAD_VIEW.read_text(encoding="utf-8"))
  nui_active = _function(tree, "nui_active")
  compare = nui_active.body[-1].value
  assert ast.unparse(compare) == "ui_state.new_ui_mode == NEW_UI_WAYON"
  render = _function(tree, "_render", "AugmentedRoadView")
  branch = next(n for n in render.body if isinstance(n, ast.If) and ast.unparse(n.test) == "nui_active()")
  assert ast.unparse(branch.body[0]) == "self._render_nui(rect, start_draw)" and isinstance(branch.body[1], ast.Return)
  first_carrot_line = next(i for i, n in enumerate(render.body) if "timing" in ast.unparse(n))
  assert render.body.index(branch) < first_carrot_line
  assert ast.unparse(render.body[render.body.index(branch) + 1]) == "self.set_frame_filter(0.0, 0.0)"
  matrix = ast.unparse(_function(tree, "_calc_frame_matrix", "AugmentedRoadView"))
  assert "self.stream_type,\n nui)" in matrix.replace("  ", " ") or "self.stream_type, nui)" in matrix


def test_carrot_shaders_are_unchanged_and_filter_is_a_separate_program():
  tree = ast.parse(CAMERA_VIEW.read_text(encoding="utf-8"))
  source = CAMERA_VIEW.read_text(encoding="utf-8")
  assert "fragColor = vec4(pow(color.rgb, vec3(1.0/1.28)), color.a);\n    }" in source
  assert "uGrayscaleMix" not in ast.unparse(next(n for n in tree.body if isinstance(n, ast.If) and ast.unparse(n.test) == "TICI"))
  frame_shader = _function(tree, "_frame_shader", "CameraView")
  first = next(n for n in frame_shader.body if isinstance(n, ast.If))
  assert ast.unparse(first.test) == "self._frame_filter == (0.0, 0.0) or self._filter_failed"
  assert ast.unparse(first.body[0]) == "return (self.shader, self._texture1_loc)"


def test_filter_shader_text_appends_grey_and_dim_to_the_frame_shader():
  tree = ast.parse(CAMERA_VIEW.read_text(encoding="utf-8"))
  ns = {}
  keep = [n for n in tree.body if (isinstance(n, ast.Assign) and any(getattr(t, "id", "") in ("_FILTER_UNIFORMS", "_FILTER_STEP")
                                                                      for t in n.targets)) or
          (isinstance(n, ast.FunctionDef) and n.name == "_with_filter")]
  exec(compile(ast.Module(body=keep, type_ignores=[]), "cameraview", "exec"), ns)
  tici_branch = next(n for n in tree.body if isinstance(n, ast.If) and ast.unparse(n.test) == "TICI")
  for branch in (tici_branch.body, tici_branch.orelse):  # C3/C3X shader, then the PC shader
    local = dict(ns, VERSION="#version 300 es\n")
    exec(compile(ast.Module(body=branch, type_ignores=[]), "cameraview", "exec"), local)
    original = local["FRAME_FRAGMENT_SHADER"]
    filtered = ns["_with_filter"](original)
    assert filtered.count("uniform float uGrayscaleMix;") == 1 and filtered.count("uniform float uDim;") == 1
    assert original.split("void main() {")[1].rstrip()[:-1].rstrip() in filtered
    assert filtered.rstrip().endswith("* (1.0 - uDim);\n    }")


def test_one_line_alerts_use_the_one_line_box(nui):
  alerts_mod = nui.m["alerts"]
  A, small, mid = alerts_mod.NuiAlert, alerts_mod.SIZE_SMALL, alerts_mod.SIZE_MID
  assert alerts_mod.layout_size(A("Lane Departure Detected", "", "ldw/permanent", 1, 1)) == small
  assert alerts_mod.layout_size(A("BRAKE!", "", "fcw/permanent", 3, 2)) == small
  assert alerts_mod.layout_size(A("BRAKE!", "Risk of Collision", "fcw/permanent", 3, 2)) == mid
  assert alerts_mod.layout_size(A("Take Control", " ", "promptDriverDistracted/warning", 2, 1)) == small
  assert alerts_mod.layout_size(A("Take Control", "Hands on wheel", "promptDriverDistracted/warning", 2, 1)) == mid
  assert alerts_mod.layout_size(A("Press Resume", "", "resumeRequired/warning", 1, 1)) == mid  # auto-hold layout
  assert alerts_mod.layout_size(A("TAKE CONTROL", "", "x/permanent", 3, 2)) == alerts_mod.SIZE_FULL
  alerts = alerts_mod.NuiAlerts()
  alerts.render(nui.rect, A("Lane Departure Detected", "", "ldw/permanent", 1, 1), 1.0)
  box = [c for c in nui.calls if c[0] == "draw_rectangle_rounded"][-1][1]
  assert (box.y, box.height) == (1080 - 271 + 40, 271 - 80)
  assert texts(nui.calls) == ["Lane Departure Detected"]


def test_screen_corners_are_one_baked_corner_drawn_four_ways(nui):
  nui.m["draw"].draw_rounded_corners((300, 0, 1860, 1080), 60, (0, 0, 0, 255))
  quads = [c for c in nui.calls if c[0] == "draw_texture_pro"]
  assert [(c[3].x, c[3].y, c[3].width, c[3].height) for c in quads] == [
    (300, 0, 60, 60), (2100, 0, 60, 60), (300, 1020, 60, 60), (2100, 1020, 60, 60)]
  assert [(c[2].width, c[2].height) for c in quads] == [(60, 60), (-60, 60), (60, -60), (-60, -60)]  # mirrored
  assert not any(c[0] == "rl_begin" for c in nui.calls)
  corner = nui.baked[-1][1]
  assert corner.shape == (60, 60, 4) and corner[0, 0, 3] == 255 and corner[59, 59, 3] == 0 and corner[0, 59, 3] <= 1


def test_glow_pixels_match_the_qt_radial_gradient(nui):
  d = nui.m["draw"]
  px = d.glow_rgba(156, 150, 85.8, d.glow_stops((73, 210, 131, 255), 76, 34))
  assert px.shape == (150, 156, 4)
  assert abs(int(px[75, 78, 3]) - 76) <= 1  # centre
  assert abs(int(px[75, 78 + 39, 3]) - 34) <= 2  # 0.45 of the radius out
  assert px[0, 0, 3] == 0  # outside the ellipse
  assert abs(int(px[75, 0, 3]) - 6) <= 1  # the ellipse (78 px) clips the 85.8 px gradient, as in Qt
  assert (px[..., :3][px[..., 3] > 0] == (73, 210, 131)).all()
  assert not any(c[0] == "rl_begin" for c in nui.calls)
def test_screen_corners_round_the_whole_screen_after_the_sidebar():
  tree = ast.parse(ROAD_VIEW.read_text(encoding="utf-8"))
  body = [ast.unparse(n) for n in _function(tree, "_render_nui", "AugmentedRoadView").body]
  corners = body.index("nui.draw_screen_corners(rl.Rectangle(0, 0, gui_app.width, gui_app.height))")
  assert body.index("rl.end_scissor_mode()") < corners  # outside the camera-view clip, over the sidebar too
  assert not any("draw_rounded_corners" in line for line in body)
  view = ast.unparse(_function(ast.parse((UI_DIR / "onroad" / "v10_nui" / "view.py").read_text(encoding="utf-8")), "render", "NuiOnroad"))
  assert "corner" not in view  # never per camera view: with the sidebar open its inner corners stay square


MAIN_LAYOUT = UI_DIR / "layouts" / "main.py"


@pytest.fixture
def sidebar_layout():
  """The real MainLayout sidebar methods with stand-in UI state, sidebar and touches."""
  tree = ast.parse(MAIN_LAYOUT.read_text(encoding="utf-8"))
  methods = [_function(tree, name, "MainLayout") for name in ("_on_onroad_clicked", "_auto_hide_sidebar")]
  clock = NS(now=100.0)
  ui = NS(started=True, new_ui_mode=1)
  gui = NS(mouse_events=[])
  ns = {"time": NS(monotonic=lambda: clock.now), "ui_state": ui, "gui_app": gui, "NEW_UI_WAYON": 1,
        "SIDEBAR_AUTO_HIDE_S": 10.0, "MainState": NS(ONROAD=2)}
  exec(compile(ast.Module(body=methods, type_ignores=[]), str(MAIN_LAYOUT), "exec"), ns)

  class Sidebar:
    is_visible = False
    def set_visible(self, visible):
      self.is_visible = visible

  layout = NS(_sidebar=Sidebar(), _sidebar_hide_at=None, _current_mode=2, _sidebar_rect=NS(x=0, width=300))
  layout.click = lambda: ns["_on_onroad_clicked"](layout)
  layout.tick = lambda: ns["_auto_hide_sidebar"](layout, clock.now)
  return NS(layout=layout, clock=clock, ui=ui, gui=gui)


def test_new_ui_1_sidebar_closes_ten_seconds_after_opening(sidebar_layout):
  t = sidebar_layout
  t.layout.click()
  assert t.layout._sidebar.is_visible
  t.clock.now = 109.9
  t.layout.tick()
  assert t.layout._sidebar.is_visible
  t.clock.now = 110.0
  t.layout.tick()
  assert not t.layout._sidebar.is_visible
  t.layout.click()  # a tap opens it again, with a fresh ten seconds
  t.clock.now = 119.0
  t.layout.tick()
  assert t.layout._sidebar.is_visible
  t.layout.click()  # tapping the road view while open still closes it at once
  assert not t.layout._sidebar.is_visible and t.layout._sidebar_hide_at is None


def test_touching_the_sidebar_keeps_it_open(sidebar_layout):
  t = sidebar_layout
  t.layout.click()
  t.clock.now = 108.0
  t.gui.mouse_events = [NS(pos=NS(x=150, y=500))]
  t.layout.tick()
  t.gui.mouse_events = []
  t.clock.now = 117.9
  t.layout.tick()
  assert t.layout._sidebar.is_visible
  t.clock.now = 118.0
  t.layout.tick()
  assert not t.layout._sidebar.is_visible


@pytest.mark.parametrize("mode", [0, 2, 3])
def test_other_new_ui_modes_keep_the_sidebar_until_tapped(sidebar_layout, mode):
  t = sidebar_layout
  t.ui.new_ui_mode = mode
  t.layout.click()
  t.clock.now = 1000.0
  t.layout.tick()
  assert t.layout._sidebar.is_visible


def test_auto_hide_stops_when_leaving_onroad_or_new_ui_1(sidebar_layout):
  t = sidebar_layout
  t.layout.click()
  t.ui.new_ui_mode = 0  # switched off while open: back to the stock behaviour
  t.clock.now = 200.0
  t.layout.tick()
  assert t.layout._sidebar.is_visible and t.layout._sidebar_hide_at is None
  t.ui.new_ui_mode = 1
  t.layout._sidebar.set_visible(False)
  t.layout.click()
  t.layout._current_mode = 1  # settings opened from the sidebar
  t.layout.tick()
  assert t.layout._sidebar_hide_at is None


def test_main_layout_checks_the_sidebar_deadline_every_frame_before_drawing():
  body = [ast.unparse(n) for n in _function(ast.parse(MAIN_LAYOUT.read_text(encoding="utf-8")), "_render", "MainLayout").body]
  assert body.index("self._auto_hide_sidebar(time.monotonic())") < body.index("self._render_main_content()")


def test_prewarm_covers_every_texture_the_screen_can_draw(nui):
  view = nui.m["view"].NuiOnroad(nui.params)
  while view._prewarm:
    view._prewarm.pop(0)()
  warmed = set(nui.requests)
  nui.requests.clear()
  hud, dm, alerts = view.hud, view.dm, view.alerts
  for status, setup in ((s.STATUS_ENGAGED, {}), (s.STATUS_DISENGAGED, {"enabled": False}),
                        (s.STATUS_OVERRIDE, {"state": "overriding", "gas": True}), (s.STATUS_DISENGAGED, {"lateral": True}),
                        (s.STATUS_ENGAGED, {"steer_limit": True})):
    for personality in ("aggressive", "standard", "relaxed", "moreRelaxed"):
      sm = make_sm()
      ss, cs, cc = sm["selfdriveState"], sm["carState"], sm["carControl"]
      ss.personality = personality
      cs.leftBlinker = cs.rightBlinker = cs.leftBlindspot = cs.rightBlindspot = True
      if setup.get("enabled") is False or setup.get("lateral"):
        ss.enabled, cc.longActive = False, False
        cc.latActive = bool(setup.get("lateral"))
      if setup.get("state"):
        ss.state, cs.gasPressed = setup["state"], True
      if setup.get("steer_limit"):
        ss.alertType = "steerSaturated/warning"
      hud.update(sm, STARTED, status, True, 100.0)
      hud.draw(nui.rect)
      dm.update(sm, STARTED, 50.1)
      dm.draw(nui.rect)
      view._draw_blindspots(nui.rect, sm, STARTED)
  for alert in (nui.m["alerts"].NuiAlert("BRAKE!", "Risk", "fcw/permanent", 3, 2),
                nui.m["alerts"].NuiAlert("Lane Departure", "", "ldw/permanent", 1, 1),
                nui.m["alerts"].NuiAlert("Press Resume", "", "resumeRequired/warning", 1, 1),
                nui.m["alerts"].NuiAlert("", "Lead vehicle departed", "leadDepartingUi", 3, 0)):
    alerts.render(nui.rect, alert, 10.0)
  view.draw_screen_corners(nui.rect)
  drawn = set(nui.requests)
  assert drawn and drawn <= warmed, drawn - warmed


@pytest.mark.parametrize("name", ["state", "draw", "hud", "model", "driver_monitor"])
def test_modules_import_with_the_real_pyray(name, monkeypatch):
  """pyray's types are functions, so e.g. `rl.Texture | None` in a signature only fails with the real library."""
  pytest.importorskip("pyray")
  if name == "model":
    pytest.importorskip("openpilot.selfdrive.locationd.calibrationd")  # needs native msgq, absent on a Mac
  for module in [m for m in sys.modules if m.startswith("openpilot.selfdrive.ui.onroad.v10_nui.") and not m.endswith(".state")]:
    monkeypatch.delitem(sys.modules, module)
  importlib.import_module(f"openpilot.selfdrive.ui.onroad.v10_nui.{name}")
  for module in [m for m in sys.modules if m.startswith("openpilot.selfdrive.ui.onroad.v10_nui.") and not m.endswith(".state")]:
    monkeypatch.delitem(sys.modules, module)
