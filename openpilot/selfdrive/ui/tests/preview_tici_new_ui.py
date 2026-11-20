"""Offline C3/C3X (tici) onroad preview for NewUi 0 and 1. No device, publishers or camerad.

Draws the real AugmentedRoadView (camera shader, Carrot or v10-nui overlays) at 2160x1080 from one recorded scene:
  --scene  concatenated cereal Event bytes, the latest message of each service at one moment
  --frame  the matching road camera frame as NV12 (width x height from --frame-size)
Only messaging, visionipc, Params and ui_state are replaced by in-process stand-ins.

  SCALE=1 python preview_tici_new_ui.py --scene scene.bin --frame frame.nv12 --frame-size 1344x760 \\
    --output out/ --case nui-engaged --case carrot-engaged

Fonts follow a real C3/C3X: the device type reads as tici (big layout) while BIG stays unset, which is how the
device starts the UI (Pretendard, FONT_SCALE 1.16). Set BIG=1 to preview the Inter fonts instead.
"""
import argparse
import enum
import os
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace

os.environ.setdefault("SCALE", "1")

import numpy as np

CASES = ('carrot-engaged', 'carrot-disengaged',
         'nui-engaged', 'nui-disengaged', 'nui-lateral-only', 'nui-override', 'nui-lane-change', 'nui-blindspot',
         'nui-steer-limit', 'nui-fcw', 'nui-ldw', 'nui-mid-alert', 'nui-small-alert', 'nui-autohold', 'nui-lead-departed',
         'nui-parked', 'nui-sidebar', 'nui-mph', 'nui-experimental', 'nui-lane-mode', 'nui-newui2')


class FakeClock:
  """Deterministic time for both runs: every frame advances 50 ms."""
  def __init__(self):
    self.t = 1000.0

  def monotonic(self):
    return self.t



def install_stubs(params_values: dict, clock: FakeClock):
  from openpilot.cereal import log

  class Params:
    def __init__(self, *_a, **_k):
      pass

    def get(self, key, *a, **k):
      return params_values.get(key)

    def get_bool(self, key, *a, **k):
      return bool(params_values.get(key, False))

    def get_int(self, key, *a, **k):
      try:
        return int(params_values.get(key, 0) or 0)
      except (TypeError, ValueError):
        return 0

    def get_float(self, key, *a, **k):
      return float(params_values.get(key, 0.0) or 0.0)

    def put(self, *_a, **_k):
      pass

    put_bool = put_int = put_nonblocking = put_bool_nonblocking = put_int_nonblocking = put

  params_mod = ModuleType('openpilot.common.params')
  params_mod.Params = Params
  params_mod.ParamKeyType = SimpleNamespace(PERSISTENT=1, CLEAR_ON_MANAGER_START=2)
  sys.modules[params_mod.__name__] = params_mod

  def new_message(service=None, valid=True, **_):
    msg = log.Event.new_message(valid=valid, logMonoTime=int(clock.monotonic() * 1e9))
    if service is not None:
      msg.init(service)
    return msg

  class PubMaster:
    def __init__(self, *_a, **_k):
      self.sent = []

    def send(self, service, msg):
      self.sent.append(service)

  def log_from_bytes(data, cls=log.Event):
    return cls.from_bytes(data).__enter__()

  messaging = ModuleType('openpilot.cereal.messaging')
  messaging.new_message = new_message
  messaging.PubMaster = PubMaster
  messaging.SubMaster = object
  messaging.log_from_bytes = log_from_bytes
  sys.modules[messaging.__name__] = messaging
  import openpilot.cereal as cereal_pkg
  cereal_pkg.messaging = messaging

  class VisionStreamType(enum.IntEnum):
    VISION_STREAM_ROAD = 0
    VISION_STREAM_DRIVER = 1
    VISION_STREAM_WIDE_ROAD = 2

  class VisionBuf(SimpleNamespace):
    pass

  vipc = ModuleType('msgq.visionipc')
  vipc.VisionStreamType = VisionStreamType
  vipc.VisionBuf = VisionBuf
  vipc.VisionIpcClient = None  # set by main once the frame is loaded
  msgq = sys.modules.get('msgq') or ModuleType('msgq')
  msgq.visionipc = vipc
  sys.modules['msgq'] = msgq
  sys.modules['msgq.visionipc'] = vipc
  return vipc, log


def make_client_class(frame_bytes: np.ndarray, width: int, height: int, vipc):
  frame = vipc.VisionBuf(data=frame_bytes, width=width, height=height, stride=width, uv_offset=width * height, idx=0, fd=-1)

  class FakeVisionIpcClient:
    def __init__(self, name, stream_type, conflate=True):
      self.stream_type = stream_type
      self._connected = False
      self.width, self.height, self.stride, self.num_buffers = width, height, width, 4

    def is_connected(self):
      return self._connected

    def connect(self, _blocking):
      self._connected = True
      return True

    @staticmethod
    def available_streams(_name, block=False):
      return [vipc.VisionStreamType.VISION_STREAM_ROAD]

    def recv(self, timeout_ms=0):
      return frame if self.stream_type == vipc.VisionStreamType.VISION_STREAM_ROAD else None

  return FakeVisionIpcClient


class FakeSubMaster(dict):
  def __init__(self, services, frame: int, clock: FakeClock):
    super().__init__(services)
    self.frame = frame
    self._clock = clock
    names = list(services)
    self.recv_frame = dict.fromkeys(names, frame)
    self.updated = dict.fromkeys(names, True)
    self.alive = dict.fromkeys(names, True)
    self.valid = dict.fromkeys(names, True)
    self.seen = dict.fromkeys(names, True)
    self.recv_time = dict.fromkeys(names, clock.monotonic())
    self.logMonoTime = dict.fromkeys(names, int(clock.monotonic() * 1e9))

  def tick(self):
    self.frame += 1
    for name in self.recv_frame:
      self.recv_frame[name] = self.frame
      self.recv_time[name] = self._clock.monotonic()
      self.logMonoTime[name] = int(self._clock.monotonic() * 1e9)

  def all_checks(self, services=None):
    return True


def load_scene(path: Path, log):
  """Latest message per service as mutable builders, plus defaults for everything else the UI reads."""
  services = {}
  with path.open('rb') as f:
    data = f.read()
  for event in log.Event.read_multiple_bytes(data):
    builder = event.as_builder()
    services[builder.which()] = getattr(builder, builder.which())
  for name in ('modelV2', 'controlsState', 'onroadEvents', 'liveCalibration', 'radarState', 'liveTracks', 'deviceState',
               'pandaStates', 'carParams', 'driverMonitoringState', 'carState', 'driverStateV2', 'roadCameraState',
               'wideRoadCameraState', 'managerState', 'selfdriveState', 'longitudinalPlan', 'gpsLocationExternal',
               'carOutput', 'carControl', 'liveParameters', 'rawAudioData', 'carrotMan', 'peripheralState',
               'navInstruction', 'liveDelay', 'lateralPlan', 'liveTorqueParameters', 'uiDebug'):
    if name not in services:
      try:
        msg = log.Event.new_message()
        msg.init(name)
        services[name] = getattr(msg, name)
      except Exception:
        pass
  return services


def apply_case(case: str, sm, ui, log, car):
  """Edit the recorded scene for one presentation case."""
  ss, cs, cc, model = sm['selfdriveState'], sm['carState'], sm['carControl'], sm['modelV2']
  status = ui.UIStatus
  ui.new_ui_mode = 0 if case.startswith('carrot') else 2 if case == 'nui-newui2' else 1
  ui.new_ui = ui.new_ui_mode >= 1
  ui.status = status.ENGAGED if ss.enabled else status.DISENGAGED
  ui.is_metric = case != 'nui-mph'
  ss.alertSize = 'none'
  ss.alertType = ''
  if case in ('carrot-disengaged', 'nui-disengaged'):
    ss.enabled = False
    cc.latActive = cc.longActive = False
    cs.latEnabled = False
    ui.status = status.DISENGAGED
  elif case == 'nui-lateral-only':
    ss.enabled = False
    cc.latActive, cc.longActive = True, False
    ui.status = status.DISENGAGED
  elif case == 'nui-override':
    ss.state = 'overriding'
    cs.gasPressed = True
    ui.status = status.OVERRIDE
  elif case == 'nui-lane-change':
    model.meta.laneChangeState = 'laneChangeStarting'
    model.meta.laneChangeDirection = 'left'
    cs.leftBlinker = True
  elif case == 'nui-blindspot':
    cs.leftBlindspot = cs.rightBlindspot = True
    cs.rightBlinker = True
  elif case == 'nui-steer-limit':
    ss.alertType, ss.alertSize, ss.alertStatus = 'steerSaturated/warning', 'small', 'userPrompt'
    ss.alertText1, ss.alertText2 = '핸들을 조작하세요', ''
  elif case == 'nui-fcw':
    ss.alertType, ss.alertSize, ss.alertStatus = 'fcw/permanent', 'full', 'critical'
    ss.alertText1, ss.alertText2 = '브레이크!', '추돌 위험'
  elif case == 'nui-ldw':
    ss.alertType, ss.alertSize, ss.alertStatus = 'ldw/permanent', 'small', 'userPrompt'
    ss.alertText1, ss.alertText2 = '차선 이탈 감지', ''
  elif case == 'nui-mid-alert':
    ss.alertType, ss.alertSize, ss.alertStatus = 'preEnableStandstill/warning', 'mid', 'userPrompt'
    ss.alertText1, ss.alertText2 = '핸들을 잡아 주세요', '운전자 주의가 필요합니다'
  elif case == 'nui-small-alert':
    ss.alertType, ss.alertSize, ss.alertStatus = 'promptDriverDistracted/warning', 'small', 'normal'
    ss.alertText1, ss.alertText2 = '전방을 주시하세요', ''
  elif case in ('nui-autohold', 'nui-lead-departed', 'nui-parked'):
    cs.standstill, cs.vEgo, cs.vEgoCluster = True, 0.0, 0.0
    if case == 'nui-autohold':
      ss.alertType, ss.alertSize, ss.alertStatus = 'resumeRequired/warning', 'small', 'userPrompt'
      ss.alertText1, ss.alertText2 = '정지 상태를 종료하려면 RES버튼을 누르세요', ''
    if case == 'nui-lead-departed':
      lead = sm['radarState'].leadOne
      lead.status, lead.dRel, lead.vRel, lead.vLeadK = True, 8.0, 1.6, 1.6
  elif case == 'nui-experimental':
    ss.experimentalMode = True
  elif case == 'nui-lane-mode':
    sm['controlsState'].activeLaneLine = True
  if ss.state == 'overriding' and case != 'nui-override':
    ss.state = 'enabled'


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('--scene', type=Path, required=True)
  parser.add_argument('--frame', type=Path, required=True)
  parser.add_argument('--frame-size', default='1928x1208')
  parser.add_argument('--output', type=Path, required=True)
  parser.add_argument('--case', choices=CASES, action='append')
  parser.add_argument('--frames', type=int, default=8)
  parser.add_argument('--language', default='ko')
  args = parser.parse_args()
  args.output.mkdir(parents=True, exist_ok=True)

  clock = FakeClock()
  params_values = {'ShowPathEnd': 1, 'LongitudinalPersonality': 1, 'IsMetric': True, 'NewUi': 1}
  vipc, log = install_stubs(params_values, clock)
  from openpilot.cereal import car
  width, height = (int(v) for v in args.frame_size.split('x'))
  frame_bytes = np.fromfile(args.frame, dtype=np.uint8)
  vipc.VisionIpcClient = make_client_class(frame_bytes, width, height, vipc)

  class UIStatus(enum.Enum):
    DISENGAGED = "disengaged"
    ENGAGED = "engaged"
    OVERRIDE = "override"

  ui = SimpleNamespace(
    sm=None, started=True, started_frame=1, started_time=clock.monotonic() - 60, status=UIStatus.ENGAGED, is_metric=True,
    new_ui_mode=1, new_ui=True, params=sys.modules['openpilot.common.params'].Params(), params_memory=None,
    lat_active=True, always_on_dm=True, show_camera_with_cluster=False, engaged=True, has_longitudinal_control=True,
    usbgpu_active=False, usbgpu_loading=False, usbgpu_startup_failed=False, usbgpu_recovery_badge=None, usbgpu_present=False,
    usbgpu_delivery_badge=None, usbgpu_compiled=False, usbgpu_compile_pending=False, jetlink_badge=None, jetlink_detail=None,
    jetlink_detail_since=0.0, UIStatus=UIStatus, recording_audio=False, panda_type=None,
    add_offroad_transition_callback=lambda cb: None, is_onroad=lambda: True, update=lambda: None,
  )
  state_mod = ModuleType('openpilot.selfdrive.ui.ui_state')
  state_mod.ui_state = ui
  state_mod.UIStatus = UIStatus
  state_mod.NEW_UI_CARROT, state_mod.NEW_UI_WAYON, state_mod.NEW_UI_CLOCK, state_mod.NEW_UI_ROBOT = 0, 1, 2, 3
  state_mod.device = SimpleNamespace()
  sys.modules[state_mod.__name__] = state_mod

  import time
  time.monotonic = clock.monotonic

  import pyray as rl
  from openpilot.system.hardware import HARDWARE
  HARDWARE.get_device_type = lambda: 'tici'  # big C3 layout; fonts still follow the BIG environment variable as on device
  from openpilot.system.ui.lib.application import FontWeight, gui_app
  from openpilot.system.ui.lib.multilang import multilang
  multilang._language = args.language
  multilang.setup()
  gui_app.init_window('tici new ui preview', font_weights=list(FontWeight))
  from openpilot.selfdrive.ui.onroad.augmented_road_view import AugmentedRoadView
  try:  # absent on trees without NewUi 1 for tici (used for before/after comparisons)
    from openpilot.selfdrive.ui.onroad.v10_nui import draw as nui_draw
    nui_draw.blink_ms = lambda: 0  # every blink in its visible phase
  except ImportError:
    pass

  selected = args.case or list(CASES)
  ui.panda_type = log.PandaState.PandaType.tres
  from openpilot.selfdrive.ui.layouts.sidebar import SIDEBAR_WIDTH, Sidebar
  keep_alive = []
  for case in selected:
    services = load_scene(args.scene, log)
    sm = FakeSubMaster(services, frame=100, clock=clock)
    ui.sm = sm
    ui.started_frame = 1
    apply_case(case, sm, ui, log, car)
    view = AugmentedRoadView()
    sidebar = Sidebar() if case == 'nui-sidebar' else None
    left = SIDEBAR_WIDTH if sidebar is not None else 0
    rect = rl.Rectangle(left, 0, 2160 - left, 1080)
    for frame in range(args.frames):
      clock.t += 0.05
      sm.tick()
      if frame == 1 and getattr(view, "_nui", None) is not None:
        # Long stops and auto-hold timers start in the past so the preview shows them running.
        view._nui.hud._standstill._since = clock.monotonic() - 75
        view._nui.alerts._resume_timer._since = clock.monotonic() - 42
      rl.begin_drawing()
      rl.clear_background(rl.Color(0, 0, 0, 255))
      if sidebar is not None:  # drawn first, as MainLayout does
        sidebar.render(rl.Rectangle(0, 0, SIDEBAR_WIDTH, 1080))
      view.render(rect)
      if frame == args.frames - 1:
        rl.rl_draw_render_batch_active()
        image = rl.load_image_from_screen()
        try:
          if image.width != 2160:  # HiDPI framebuffer
            rl.image_resize(image, 2160, 1080)
          rl.export_image(image, str(args.output / f'{case}.png'))
        finally:
          rl.unload_image(image)
      rl.end_drawing()
    keep_alive.append((view, sidebar))  # CameraView.__del__ closes again; never let a closed view be collected mid-run
    print('rendered', case, flush=True)
  sys.stdout.flush()
  os._exit(0)  # skip atexit window teardown, which aborts on some macOS raylib builds


if __name__ == '__main__':
  main()
