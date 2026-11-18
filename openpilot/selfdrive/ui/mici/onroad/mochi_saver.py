"""모찌 화면보호기 (Mochi screensaver) — 주행 화면 대체 모드.

검은 화면에 눈만 크게, 입은 작게, 코 없음. 각 표정은 한 화면 안에서
계속 반복해 움직이는 애니메이션 (사용자 승인한 시뮬레이션과 동일 구성).

동작 (OnroadMochiMode=1일 때만, 주행 화면에서만):
- 모찌 화면을 터치하면 주행 화면으로 복귀.
- 주행 화면을 650ms 길게 누르면 모찌 화면으로 전환 (게이지 표시).
- 주행 화면 짧은 터치는 아무 동작 없음 (주행 중 오동작 방지).

표정 트리거 (우선순위 순):
  긴급제동(aEgo <= -4) > 급감속(aEgo <= -2.5) > 급가속(aEgo >= 2)
  > 후진(R) > 신호대기(빨간불 + 정지) > 오토홀드
  > 인게이지 / 인게이지 해제 / 정지 중 출발 (전환 직후 잠깐)
  > 차선변경(desire) > 깜빡이 > 주행중(기본)

주의 — 오토홀드 감지는 근사치:
  실제 홀드 상태(CS.manual_auto_hold.active)는 UI로 전달되는 메시지에
  없어서, 정지 + D/low + 미인게이지 + 크루즈 꺼짐 상태에서 브레이크를
  밟았다 뗀 뒤로 추정한다. 가볍게 브레이크를 밟았다 뗐을 때도 홀드
  표정이 나올 수 있고, UI 시작 전에 이미 걸린 홀드는 다음 브레이크
  조작 때부터 보인다.
"""

import math
import time

import pyray as rl

from openpilot.cereal import car, log
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import FontWeight, MouseEvent, gui_app
from openpilot.system.ui.lib.text_measure import measure_text_cached

LONG_PRESS_S = 0.65  # 주행 화면 길게 누르기 시간 (초)
TRANSIENT_S = 2.5  # 인게이지/해제 표정 유지 시간 (초)
START_S = 3.0  # 정지 중 출발 표정 유지 시간 (초)
HOLD_SHOW_S = 30.0  # 오토홀드 표정 최대 유지 시간 (초)

EMERGENCY_DECEL = -4.0  # m/s^2, 긴급제동
DECEL = -2.5  # m/s^2, 급감속
ACCEL = 2.0  # m/s^2, 급가속
STOP_SPEED = 0.5  # m/s, 정지 판정
HOLD_SPEED = 0.05  # m/s, 오토홀드 정지 판정

# 표정 이름
EXPR_DRIVING = "driving"  # 주행중
EXPR_DECEL = "decel"  # 급감속
EXPR_EMERGENCY = "emergency"  # 긴급제동
EXPR_ACCEL = "accel"  # 급가속
EXPR_LANE_LEFT = "lane_left"  # 좌측 차선변경
EXPR_LANE_RIGHT = "lane_right"  # 우측 차선변경
EXPR_BLINK_LEFT = "blink_left"  # 좌측 깜빡이
EXPR_BLINK_RIGHT = "blink_right"  # 우측 깜빡이
EXPR_START = "start"  # 정지 중 출발
EXPR_ENGAGE = "engage"  # 인게이지
EXPR_DISENGAGE = "disengage"  # 인게이지 해제
EXPR_REVERSE = "reverse"  # 후진
EXPR_TRAFFIC = "traffic"  # 신호대기
EXPR_AUTOHOLD = "autohold"  # 오토홀드

EYE_WHITE = rl.Color(255, 255, 255, 255)
PUPIL = rl.Color(35, 35, 45, 255)
PUPIL_HI = rl.Color(255, 255, 255, 255)
MOUTH = rl.Color(225, 225, 225, 255)
C_ORANGE = rl.Color(255, 150, 0, 255)
C_RED = rl.Color(255, 60, 60, 255)
C_YELLOW = rl.Color(255, 210, 63, 255)
C_GREEN = rl.Color(40, 190, 110, 255)
C_SWEAT = rl.Color(120, 200, 255, 255)
C_DARK = rl.Color(40, 40, 45, 255)
C_GAUGE_BG = rl.Color(255, 255, 255, 60)
C_GAUGE_FG = rl.Color(0, 220, 180, 255)


class MochiSaver:
  def __init__(self):
    self.active = False  # 모찌 화면 표시 중
    self._pressing = False  # 주행 화면 길게 누르는 중
    self._press_start = 0.0
    self._press_x = 0.0
    self._press_y = 0.0
    self._press_fired = False

    self._expr = EXPR_DRIVING
    self._prev_engaged = False
    self._prev_v_ego = 0.0
    self._prev_brake = False
    self._transient = ("", 0.0)  # (표정, 유지 종료 시각)
    self._brake_held_stop = False
    self._hold_until = 0.0

    self._font_bold: rl.Font | None = None
    self._font_medium: rl.Font | None = None

  # ---------------------------------------------------------------
  # 외부 연결: AugmentedRoadView에서 호출
  # ---------------------------------------------------------------

  @property
  def enabled(self) -> bool:
    return ui_state.onroad_mochi

  def handle_mouse_event(self, ev: MouseEvent) -> None:
    """주행 화면 터치 처리. 짧은 터치는 무시, 길게 누르기만 인식."""
    if not self.enabled or not ui_state.started:
      self._cancel_press()
      return

    if self.active:
      # 모찌 화면: 터치하면 바로 주행 화면으로 복귀
      if ev.left_pressed:
        self.active = False
        self._cancel_press()
      return

    # 주행 화면: 길게 누르기만 인식 (짧은 터치는 오동작 방지용 무동작)
    if ev.left_pressed:
      self._pressing = True
      self._press_start = time.monotonic()
      self._press_x = ev.pos.x
      self._press_y = ev.pos.y
      self._press_fired = False
    elif ev.left_released:
      self._cancel_press()

  def update(self) -> None:
    """매 프레임 호출: 길게 누르기 판정 + 표정 선택."""
    if not self.enabled or not ui_state.started:
      self.active = False
      self._cancel_press()
      return

    now = time.monotonic()

    # 길게 누르기: 손가락을 가만히 들고 있어도 중간 이벤트가 안 오므로
    # 시간으로 직접 판정한다. 손을 뗐으면 release 이벤트가 취소한다.
    if self._pressing and not self._press_fired:
      if not gui_app.last_mouse_event.left_down:
        self._cancel_press()
      elif now - self._press_start >= LONG_PRESS_S:
        self.active = True
        self._press_fired = True
        self._pressing = False

    self._select_expression(now)

  def render(self, rect: rl.Rectangle) -> None:
    """모찌 화면 또는 길게 누르기 게이지 그리기."""
    if not self.enabled or not ui_state.started:
      return

    if self._pressing and not self.active:
      progress = min(1.0, (time.monotonic() - self._press_start) / LONG_PRESS_S)
      self._draw_long_press_gauge(self._press_x, self._press_y, progress)
      return

    if not self.active:
      return

    # 검은 화면 전체를 덮는다 (주행 화면 대체)
    rl.draw_rectangle(int(rect.x), int(rect.y), int(rect.width), int(rect.height), rl.BLACK)
    self._draw_face(rect, rl.get_time())

  # ---------------------------------------------------------------
  # 표정 선택
  # ---------------------------------------------------------------

  def _select_expression(self, now: float) -> None:
    sm = ui_state.sm
    cs = sm["carState"]
    lp = sm["lateralPlan"]
    lon = sm["longitudinalPlan"]
    try:
      v_ego = float(cs.vEgo)
      a_ego = float(cs.aEgo)
    except Exception:
      v_ego, a_ego = 0.0, 0.0

    engaged = ui_state.engaged

    # 인게이지 / 해제 전환 직후 잠깐 보여주는 표정
    if engaged != self._prev_engaged:
      self._transient = ("engage" if engaged else "disengage", now + TRANSIENT_S)
      self._prev_engaged = engaged
    # 정지에서 출발 직후 잠깐 보여주는 표정
    if self._prev_v_ego < 0.3 and v_ego > STOP_SPEED:
      self._transient = ("start", now + START_S)
    self._prev_v_ego = v_ego

    # 오토홀드 근사 감지: 정지 + D/low + 미인게이지 + 크루즈 꺼짐에서
    # 브레이크를 밟았다 뗀 뒤 (실제 홀드 상태는 UI 메시지에 없음)
    try:
      brake = bool(cs.brakePressed)
      gas = bool(cs.gasPressed)
      standstill = bool(cs.standstill)
      gear = cs.gearShifter
      cruise_on = bool(cs.cruiseState.enabled)
    except Exception:
      brake, gas, standstill, gear, cruise_on = False, False, False, None, False
    hold_ok = (standstill and abs(v_ego) < HOLD_SPEED
               and gear in (car.CarState.GearShifter.drive, car.CarState.GearShifter.low)
               and not engaged and not cruise_on)
    if brake and hold_ok:
      self._brake_held_stop = True
    if self._brake_held_stop and not brake and hold_ok:
      self._hold_until = now + HOLD_SHOW_S
      self._brake_held_stop = False
    if not hold_ok or gas:
      self._hold_until = 0.0
      if not hold_ok:
        self._brake_held_stop = False
    self._prev_brake = brake

    try:
      desire = lp.desire
    except Exception:
      desire = log.Desire.none
    try:
      traffic_red = int(lon.trafficState) == 1
    except Exception:
      traffic_red = False
    try:
      left_blinker = bool(cs.leftBlinker)
      right_blinker = bool(cs.rightBlinker)
    except Exception:
      left_blinker, right_blinker = False, False

    # 우선순위 순으로 표정 결정
    if a_ego <= EMERGENCY_DECEL:
      expr = EXPR_EMERGENCY
    elif a_ego <= DECEL:
      expr = EXPR_DECEL
    elif a_ego >= ACCEL:
      expr = EXPR_ACCEL
    elif gear == car.CarState.GearShifter.reverse:
      expr = EXPR_REVERSE
    elif traffic_red and v_ego < STOP_SPEED:
      expr = EXPR_TRAFFIC
    elif now < self._hold_until:
      expr = EXPR_AUTOHOLD
    elif self._transient[0] and now < self._transient[1]:
      expr = self._transient[0]
    elif desire == log.Desire.laneChangeLeft:
      expr = EXPR_LANE_LEFT
    elif desire == log.Desire.laneChangeRight:
      expr = EXPR_LANE_RIGHT
    elif left_blinker:
      expr = EXPR_BLINK_LEFT
    elif right_blinker:
      expr = EXPR_BLINK_RIGHT
    else:
      expr = EXPR_DRIVING
    self._expr = expr

  def _cancel_press(self) -> None:
    self._pressing = False
    self._press_fired = False

  # ---------------------------------------------------------------
  # 그리기: 얼굴
  # ---------------------------------------------------------------

  def _fonts(self):
    if self._font_bold is None:
      self._font_bold = gui_app.font(FontWeight.BOLD)
      self._font_medium = gui_app.font(FontWeight.MEDIUM)
    return self._font_bold, self._font_medium

  def _draw_long_press_gauge(self, x: float, y: float, progress: float) -> None:
    """길게 누르는 동안 손가락 위치에 차오르는 원형 게이지."""
    r = 70
    rl.draw_ring(rl.Vector2(x, y), r - 8, r, 0.0, 360.0, 48, C_GAUGE_BG)
    if progress > 0.0:
      rl.draw_ring(rl.Vector2(x, y), r - 8, r, -90.0, -90.0 + 360.0 * progress, 48, C_GAUGE_FG)
    rl.draw_circle(int(x), int(y), int(10 + 26 * progress), C_GAUGE_FG)

  def _draw_face(self, rect: rl.Rectangle, t: float) -> None:
    s = min(rect.width / 2160.0, rect.height / 1080.0)
    cx = rect.x + rect.width / 2.0
    cy = rect.y + rect.height / 2.0
    expr = self._expr

    # 표정별 전체 움직임 (통통 튀기 / 떨림 / 흔들림 / 숨쉬기)
    bounce = 0.0
    jx = jy = 0.0
    breathe = 1.0
    if expr == EXPR_START:
      bounce = -26.0 * s * max(0.0, math.sin(t * math.pi * 2.0 / 1.1))
    elif expr == EXPR_ENGAGE:
      bounce = -18.0 * s * max(0.0, math.sin(t * math.pi * 2.0 / 0.9))
    elif expr == EXPR_EMERGENCY:
      jx = 8.0 * s * math.sin(t * math.pi * 2.0 * 13.0)
      jy = 6.0 * s * math.sin(t * math.pi * 2.0 * 17.0)
    elif expr == EXPR_DECEL:
      jx = 5.0 * s * math.sin(t * math.pi * 2.0 * 9.0)
    elif expr == EXPR_AUTOHOLD:
      breathe = 1.0 + 0.02 * math.sin(t * math.pi * 2.0 / 4.0)

    face_cy = cy + bounce
    eye_r = 165.0 * s * breathe
    eye_dx = 250.0 * s * breathe
    eye_y = -70.0 * s * breathe
    pupil_r = 62.0 * s
    mouth_y = 250.0 * s * breathe

    # 표정별 눈동자 위치·눈꺼풀·입 모양
    pupil_dx = pupil_dy = 0.0
    lid = 1.0  # 1=활짝, 0.45=반쯤 감음
    mouth = "smile"
    if expr == EXPR_LANE_LEFT or expr == EXPR_BLINK_LEFT:
      pupil_dx = -70.0 * s if expr == EXPR_LANE_LEFT else -45.0 * s
    elif expr == EXPR_LANE_RIGHT or expr == EXPR_BLINK_RIGHT:
      pupil_dx = 70.0 * s if expr == EXPR_LANE_RIGHT else 45.0 * s
    elif expr == EXPR_REVERSE:
      pupil_dx, pupil_dy = 30.0 * s, -50.0 * s
    elif expr == EXPR_TRAFFIC:
      pupil_dy = -55.0 * s
      lid = 0.5
      mouth = "flat"
    elif expr == EXPR_DISENGAGE:
      pupil_dy = 35.0 * s
      lid = 0.55
      mouth = "flat"
    elif expr == EXPR_AUTOHOLD:
      pupil_dy = 40.0 * s
      lid = 0.8
      mouth = "calm"
    elif expr == EXPR_DECEL:
      pupil_dy = -30.0 * s
      mouth = "o"
    elif expr == EXPR_EMERGENCY:
      pupil_dy = -35.0 * s
      pupil_r = 48.0 * s
      mouth = "wavy"
    elif expr == EXPR_ACCEL:
      pupil_dy = -40.0 * s
      mouth = "open"
    elif expr == EXPR_START or expr == EXPR_ENGAGE:
      mouth = "open"

    blink = self._blink_scale(t)
    lid = min(lid, blink) if expr not in (EXPR_TRAFFIC, EXPR_DISENGAGE) else lid

    # 눈 (좌·우)
    for side in (-1, 1):
      ex = cx + side * eye_dx + jx
      ey = face_cy + eye_y + jy
      self._draw_eye(ex, ey, eye_r, lid, pupil_dx, pupil_dy, pupil_r, t, expr, side)

    # 입
    self._draw_mouth(cx, face_cy + mouth_y, mouth, s, t)

    # 표정별 배지·글자·소품
    if expr == EXPR_DECEL:
      self._draw_badge_text("stop", cx, face_cy - 430.0 * s, 110, C_ORANGE, s)
    elif expr == EXPR_EMERGENCY:
      self._draw_badge_text("STOP", cx, face_cy - 430.0 * s, 130, C_RED, s)
      self._draw_sweat(cx, face_cy, s, t)
    elif expr == EXPR_START:
      self._draw_badge_text("GO~~", cx, face_cy - 430.0 * s, 130, C_YELLOW, s)
    elif expr == EXPR_REVERSE:
      self._draw_r_badge(cx + 640.0 * s, face_cy - 400.0 * s, s)
    elif expr == EXPR_TRAFFIC:
      self._draw_traffic_light(cx + 700.0 * s, face_cy - 260.0 * s, s)
    elif expr == EXPR_AUTOHOLD:
      self._draw_hold_badge(cx, face_cy - 460.0 * s, s)

  def _blink_scale(self, t: float, period: float = 2.3) -> float:
    """깜빡임: 2.3초 주기로 0.12초간 감았다 뜬다. 1=활짝."""
    p = (t % period) / period
    if p < 0.12:
      return 0.12 + 0.88 * abs(p - 0.06) / 0.06
    return 1.0

  def _draw_eye(self, ex: float, ey: float, r: float, lid: float,
                pupil_dx: float, pupil_dy: float, pupil_r: float,
                t: float, expr: str, side: int) -> None:
    # 흰자 (원)
    rl.draw_circle(int(ex), int(ey), int(r), EYE_WHITE)
    # 눈동자
    px = ex + pupil_dx
    py = ey + pupil_dy
    rl.draw_circle(int(px), int(py), int(pupil_r), PUPIL)
    # 눈동자 반짝이
    rl.draw_circle(int(px - pupil_r * 0.3), int(py - pupil_r * 0.3), int(pupil_r * 0.22), PUPIL_HI)
    # 눈꺼풀: 검은 배경 위라 위에서 덮으면 감은 것처럼 보인다
    if lid < 1.0:
      cover_h = 2.0 * r * (1.0 - lid)
      rl.draw_rectangle(int(ex - r), int(ey - r), int(2.0 * r), int(cover_h), rl.BLACK)
    # 깜빡이 표정: 깜빡이는 쪽 눈을 번갈아 감는다
    if expr in (EXPR_BLINK_LEFT, EXPR_BLINK_RIGHT):
      wink_side = -1 if expr == EXPR_BLINK_LEFT else 1
      if side == wink_side and (int(t * 2.0) % 2 == 0):
        rl.draw_rectangle(int(ex - r), int(ey - r), int(2.0 * r), int(2.0 * r * 0.55), rl.BLACK)

  def _draw_mouth(self, cx: float, cy: float, kind: str, s: float, t: float) -> None:
    w = 90.0 * s
    thick = max(6.0, 10.0 * s)
    if kind == "smile" or kind == "calm":
      # 아래쪽 호 (미소)
      rl.draw_ring(rl.Vector2(cx, cy - w * 0.35), w * 0.5 - thick, w * 0.5, 20.0, 160.0, 32, MOUTH)
    elif kind == "o":
      rl.draw_ring(rl.Vector2(cx, cy), w * 0.22, w * 0.32, 0.0, 360.0, 32, MOUTH)
    elif kind == "open":
      # 활짝 웃는 입 (채운 부채꼴)
      rl.draw_ring(rl.Vector2(cx, cy - w * 0.4), 1.0, w * 0.55, 20.0, 160.0, 32, MOUTH)
    elif kind == "flat":
      rl.draw_line_ex(rl.Vector2(cx - w * 0.5, cy), rl.Vector2(cx + w * 0.5, cy), thick, MOUTH)
    elif kind == "wavy":
      # 떨리는 입 (지그재그)
      pts = [(-0.5, 0.0), (-0.25, -0.18), (0.0, 0.0), (0.25, -0.18), (0.5, 0.0)]
      for i in range(len(pts) - 1):
        x1 = cx + pts[i][0] * w
        y1 = cy + pts[i][1] * w
        x2 = cx + pts[i + 1][0] * w
        y2 = cy + pts[i + 1][1] * w
        rl.draw_line_ex(rl.Vector2(x1, y1), rl.Vector2(x2, y2), thick * 0.8, MOUTH)

  def _draw_badge_text(self, text: str, cx: float, y: float, size: int,
                       color: rl.Color, s: float) -> None:
    font, _ = self._fonts()
    font_size = int(size * s)
    size_vec = measure_text_cached(font, text, font_size)
    pos = rl.Vector2(cx - size_vec.x / 2.0, y)
    rl.draw_text_ex(font, text, pos, font_size, 0, color)

  def _draw_r_badge(self, x: float, y: float, s: float) -> None:
    """후진: 커다란 노란 R 배지."""
    font, _ = self._fonts()
    w, h = 200.0 * s, 200.0 * s
    rl.draw_rectangle_rounded(rl.Rectangle(x, y, w, h), 0.25, 12, C_YELLOW)
    size_vec = measure_text_cached(font, "R", int(150 * s))
    pos = rl.Vector2(x + (w - size_vec.x) / 2.0, y + (h - size_vec.y) / 2.0 - 8 * s)
    rl.draw_text_ex(font, "R", pos, int(150 * s), 0, rl.BLACK)

  def _draw_traffic_light(self, x: float, y: float, s: float) -> None:
    """신호대기: 커다란 신호등 (빨간불 켜짐)."""
    w, h = 170.0 * s, 420.0 * s
    rl.draw_rectangle_rounded(rl.Rectangle(x, y, w, h), 0.18, 12, C_DARK)
    lamp_r = 52.0 * s
    lamp_cx = x + w / 2.0
    gap = 130.0 * s
    top = y + 78.0 * s
    rl.draw_circle(int(lamp_cx), int(top), int(lamp_r), C_RED)
    rl.draw_circle(int(lamp_cx), int(top + gap), int(lamp_r), rl.Color(90, 80, 40, 255))
    rl.draw_circle(int(lamp_cx), int(top + gap * 2), int(lamp_r), rl.Color(40, 90, 50, 255))

  def _draw_hold_badge(self, cx: float, y: float, s: float) -> None:
    """오토홀드: 초록 HOLD 알약 배지."""
    font, _ = self._fonts()
    w, h = 360.0 * s, 95.0 * s
    x = cx - w / 2.0
    rl.draw_rectangle_rounded(rl.Rectangle(x, y, w, h), 0.5, 12, C_GREEN)
    size_vec = measure_text_cached(font, "HOLD", int(64 * s))
    pos = rl.Vector2(cx - size_vec.x / 2.0, y + (h - size_vec.y) / 2.0 - 4 * s)
    rl.draw_text_ex(font, "HOLD", pos, int(64 * s), 0, rl.WHITE)

  def _draw_sweat(self, cx: float, cy: float, s: float, t: float) -> None:
    """긴급제동: 얼굴 양옆에서 떨어지는 땀방울."""
    for i, side in enumerate((-1, 1)):
      fall = ((t * 260.0 + i * 130.0) % 260.0) * s
      sx = cx + side * 480.0 * s
      sy = cy - 260.0 * s + fall
      rl.draw_circle(int(sx), int(sy), int(16 * s), C_SWEAT)
