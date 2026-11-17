"""화면 꺼짐 시 빨간 아날로그 시계 (기기 화면만, 외부 HUD는 건드리지 않음).

사용자가 보낸 아이폰 대기(StandBy) 시계 사진 디자인: 검은 배경에 빨간 숫자(12/3/6/9),
빨간 눈금, 빨간 시침·분침. 초침은 없음. 왼쪽에는 현재 시각(12시간제), 오른쪽에는
날짜(요일 + 일)를 작은 빨간 글씨로 표시한다. 기기에 알람 기능이 없어 사진 속
"6:30 AM" 알람 자리는 현재 시각으로 대신한다.

화면이 꺼졌을 때(ShowCustomBrightness=0) 백라이트를 완전히 끄면 시계가 안 보이므로,
시계가 켜진 상태에서는 ui_state.Device._update_brightness가 백라이트를
CLOCK_BACKLIGHT_FLOOR(%)까지만 낮춘다. 시계 밝기는 화면 밝기 파이프라인을 그대로
따른다 (밝기를 0보다 크게 올리면 시계가 사라지고 일반 화면으로 돌아온다).
"""

import math
from datetime import datetime

import pyray as rl

from openpilot.system.ui.lib.application import FontWeight, gui_app
from openpilot.system.ui.lib.text_draw import draw_text_ui_style

# 아이폰 대기 시계 빨간색.
CLOCK_RED = rl.Color(255, 59, 48, 255)

# 시계 표시 중 백라이트 하한(%). 0이면 시계가 안 보이므로, 희미하게 보이도록
# 최소 밝기를 유지한다. 화면 밝기 파이프라인(_update_brightness)을 통해 적용된다.
CLOCK_BACKLIGHT_FLOOR = 8


def hand_angles(now: datetime) -> tuple[float, float]:
  """시침·분침 각도(도, 12시 방향=0, 시계 방향+). 테스트용 순수 함수."""
  minute_angle = now.minute * 6.0
  hour_angle = ((now.hour % 12) + now.minute / 60.0) * 30.0
  return hour_angle, minute_angle


def _polar(cx: float, cy: float, radius: float, angle_deg: float) -> tuple[float, float]:
  """각도(0도=12시, 시계 방향+)를 화면 좌표로 변환. y축은 아래가 +."""
  angle = math.radians(angle_deg - 90.0)
  return cx + radius * math.cos(angle), cy + radius * math.sin(angle)


def draw_screen_off_clock(rect: rl.Rectangle, now: datetime | None = None) -> None:
  """검은 배경 위에 빨간 아날로그 시계를 그린다. rect 전체를 덮는다."""
  if now is None:
    now = datetime.now()

  x = int(rect.x)
  y = int(rect.y)
  w = int(rect.width)
  h = int(rect.height)
  rl.draw_rectangle(x, y, w, h, rl.Color(0, 0, 0, 255))

  cx = rect.x + rect.width / 2.0
  cy = rect.y + rect.height / 2.0
  radius = min(rect.width, rect.height) * 0.42

  font = gui_app.font(FontWeight.DISPLAY)

  # 눈금 60개: 12/3/6/9(15분 단위)는 길고 굵게, 5분 단위는 중간, 나머지는 짧고 가늘게.
  for minute in range(60):
    angle_deg = minute * 6.0
    if minute % 15 == 0:
      inner = radius * 0.78
      thick = 10.0
    elif minute % 5 == 0:
      inner = radius * 0.84
      thick = 6.0
    else:
      inner = radius * 0.88
      thick = 3.0
    x1, y1 = _polar(cx, cy, inner, angle_deg)
    x2, y2 = _polar(cx, cy, radius * 0.96, angle_deg)
    rl.draw_line_ex(rl.Vector2(x1, y1), rl.Vector2(x2, y2), thick, CLOCK_RED)

  # 숫자 12/3/6/9 (사진과 같은 위치).
  num_size = int(radius * 0.20)
  for num, angle_deg in (("12", 0.0), ("3", 90.0), ("6", 180.0), ("9", 270.0)):
    nx, ny = _polar(cx, cy, radius * 0.70, angle_deg)
    draw_text_ui_style(num, nx, ny, num_size, CLOCK_RED, font=font,
                       border_width=0.0, shadow_offset=0.0, align="center", y_offset=0.0)

  # 왼쪽: 현재 시각(12시간제, "8:20 PM"), 오른쪽: 날짜("MON 5").
  info_size = int(radius * 0.11)
  time_text = now.strftime("%I:%M %p").lstrip("0")
  date_text = now.strftime("%a").upper() + " " + str(now.day)
  draw_text_ui_style(time_text, cx - radius * 0.38, cy, info_size, CLOCK_RED, font=font,
                     border_width=0.0, shadow_offset=0.0, align="center", y_offset=0.0)
  draw_text_ui_style(date_text, cx + radius * 0.38, cy, info_size, CLOCK_RED, font=font,
                     border_width=0.0, shadow_offset=0.0, align="center", y_offset=0.0)

  # 시침·분침 (초침 없음, 사진과 동일). 글씨 위에 그린다.
  hour_angle, minute_angle = hand_angles(now)
  hx, hy = _polar(cx, cy, radius * 0.52, hour_angle)
  rl.draw_line_ex(rl.Vector2(cx, cy), rl.Vector2(hx, hy), 16.0, CLOCK_RED)
  mx, my = _polar(cx, cy, radius * 0.80, minute_angle)
  rl.draw_line_ex(rl.Vector2(cx, cy), rl.Vector2(mx, my), 11.0, CLOCK_RED)

  # 중심점.
  rl.draw_circle(int(cx), int(cy), 14, CLOCK_RED)
