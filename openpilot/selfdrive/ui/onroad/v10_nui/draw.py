"""raylib helpers for the v10-nui tici screen (Qt QPainter equivalents)."""
import math
import time

import numpy as np
import pyray as rl

from openpilot.selfdrive.ui.onroad.v10_nui.state import RGBA, clamp
from openpilot.system.ui.lib.application import ASSETS_DIR, gui_app
from openpilot.system.ui.lib.text_measure import measure_text_cached

ICON_DIR = "icons/v10_nui"
DM_DIR = "icons_mici/onroad/driver_monitoring"

LEFT, CENTER, RIGHT = "left", "center", "right"
TOP, BOTTOM = "top", "bottom"

_GLOW_SEGMENTS = 36
_GLOW_RINGS = (0.0, 0.15, 0.3, 0.45, 0.6, 0.75, 0.9, 1.0)


def blink_ms() -> int:
  """Clock for blink phases (turn signals, blind spot, alert icons)."""
  return int(time.monotonic() * 1000)


def color(rgba: RGBA, opacity: float = 1.0) -> rl.Color:
  r, g, b, a = rgba
  return rl.Color(int(r), int(g), int(b), int(round(clamp(a * opacity, 0, 255))))


def texture(name: str, width: int, height: int, *, folder: str = ICON_DIR, keep_aspect: bool = False) -> rl.Texture:
  """Qt drawPixmap(rect, loadPixmap(path, size)) stretches into rect, so most icons load at the rect size."""
  return gui_app.texture(f"{folder}/{name}.png", width, height, keep_aspect_ratio=keep_aspect)


class ThresholdTexture:
  """hud.cc tintTurnSignalIcon: alpha < 140 becomes clear, the rest opaque white (tinted at draw)."""

  def __init__(self, name: str, width: int, height: int, threshold: int = 140):
    self._path = str(ASSETS_DIR.joinpath(f"{ICON_DIR}/{name}.png"))
    self._size = (width, height)
    self._threshold = threshold
    self._texture: rl.Texture | None = None

  def get(self) -> rl.Texture:
    if self._texture is None:
      image = rl.load_image(self._path)
      rl.image_format(image, rl.PixelFormat.PIXELFORMAT_UNCOMPRESSED_R8G8B8A8)
      rl.image_resize(image, *self._size)
      w, h = image.width, image.height
      pixels = np.frombuffer(rl.ffi.buffer(image.data, w * h * 4), dtype=np.uint8).reshape(h, w, 4).copy()
      opaque = pixels[..., 3] >= self._threshold
      pixels[..., :3] = 255
      pixels[..., 3] = np.where(opaque, 255, 0)
      rl.ffi.memmove(image.data, pixels.tobytes(), w * h * 4)
      self._texture = rl.load_texture_from_image(image)
      rl.set_texture_filter(self._texture, rl.TextureFilter.TEXTURE_FILTER_BILINEAR)
      rl.set_texture_wrap(self._texture, rl.TextureWrap.TEXTURE_WRAP_CLAMP)
      rl.unload_image(image)
    return self._texture

  def close(self) -> None:
    if self._texture is not None:
      rl.unload_texture(self._texture)
      self._texture = None


def draw_texture_rect(tex: rl.Texture, x: float, y: float, w: float, h: float, tint: rl.Color = rl.WHITE) -> None:
  rl.draw_texture_pro(tex, rl.Rectangle(0, 0, tex.width, tex.height), rl.Rectangle(x, y, w, h), rl.Vector2(0, 0), 0.0, tint)


def draw_texture_rotated(tex: rl.Texture, cx: float, cy: float, w: float, h: float, angle_deg: float,
                         tint: rl.Color = rl.WHITE, offset_y: float = 0.0) -> None:
  """Rotate about (cx, cy); the texture centre sits offset_y below it before rotation (Qt translate+rotate)."""
  rl.draw_texture_pro(tex, rl.Rectangle(0, 0, tex.width, tex.height), rl.Rectangle(cx, cy, w, h),
                      rl.Vector2(w / 2, h / 2 - offset_y), angle_deg, tint)


def draw_text(font: rl.Font, text: str, size: float, rect: tuple[float, float, float, float], rgba: RGBA,
              h_align: str = LEFT, v_align: str = CENTER, shadow: RGBA | None = None, shadow_dy: float = 0.0) -> None:
  """Qt drawText(rect, alignment, text) for a single line."""
  if not text:
    return
  x, y, w, h = rect
  measured = measure_text_cached(font, text, int(size))
  tx = x if h_align == LEFT else x + (w - measured.x) / 2 if h_align == CENTER else x + w - measured.x
  ty = y if v_align == TOP else y + (h - measured.y) / 2 if v_align == CENTER else y + h - measured.y
  if shadow is not None:
    rl.draw_text_ex(font, text, rl.Vector2(tx, ty + shadow_dy), size, 0, color(shadow))
  rl.draw_text_ex(font, text, rl.Vector2(tx, ty), size, 0, color(rgba))


def draw_vertical_gradient(x: float, y: float, w: float, h: float, stops: tuple[tuple[float, RGBA], ...]) -> None:
  """QLinearGradient(0, y, 0, y + h) filled over the same span (pad beyond the ends is transparent here)."""
  for (p0, c0), (p1, c1) in zip(stops, stops[1:], strict=False):
    top = y + h * p0
    bottom = y + h * p1
    if bottom - top < 0.5:
      continue
    rl.draw_rectangle_gradient_v(int(round(x)), int(round(top)), int(round(w)), int(round(bottom)) - int(round(top)),
                                 color(c0), color(c1))


def _gradient_at(stops: tuple[tuple[float, RGBA], ...], t: float) -> RGBA:
  t = clamp(t, 0.0, 1.0)
  if t <= stops[0][0]:
    return stops[0][1]
  for (p0, c0), (p1, c1) in zip(stops, stops[1:], strict=False):
    if t <= p1:
      f = (t - p0) / max(p1 - p0, 1e-6)
      return tuple(int(round(a + (b - a) * f)) for a, b in zip(c0, c1, strict=True))  # type: ignore[return-value]
  return stops[-1][1]


def _emit(points, colors) -> None:
  """One triangle in raylib's own front-face order (as DrawCircleGradient), so GPU culling never drops it."""
  (ax, ay), (bx, by), (cx, cy) = points
  if (bx - ax) * (cy - ay) - (by - ay) * (cx - ax) > 0:
    points = (points[0], points[2], points[1])
    colors = (colors[0], colors[2], colors[1])
  for (px, py), c in zip(points, colors, strict=True):
    rl.rl_color4ub(c[0], c[1], c[2], c[3])
    rl.rl_vertex2f(px, py)


def draw_radial_glow(rect: tuple[float, float, float, float], radius: float, stops: tuple[tuple[float, RGBA], ...]) -> None:
  """QPainter drawEllipse(rect) with QRadialGradient(rect centre, radius, stops)."""
  x, y, w, h = rect
  cx, cy, rx, ry = x + w / 2, y + h / 2, w / 2, h / 2
  if radius <= 0 or rx <= 0 or ry <= 0:
    return
  dirs = [(math.cos(2 * math.pi * j / _GLOW_SEGMENTS), math.sin(2 * math.pi * j / _GLOW_SEGMENTS)) for j in range(_GLOW_SEGMENTS)]
  edge = [math.hypot(rx * c, ry * s) for c, s in dirs]
  rl.rl_begin(rl.RL_TRIANGLES)
  for k in range(len(_GLOW_RINGS) - 1):
    u0, u1 = _GLOW_RINGS[k], _GLOW_RINGS[k + 1]
    for j in range(_GLOW_SEGMENTS):
      j2 = (j + 1) % _GLOW_SEGMENTS
      (c0, s0), (c1, s1) = dirs[j], dirs[j2]
      p00 = (cx + u0 * rx * c0, cy + u0 * ry * s0)
      p01 = (cx + u0 * rx * c1, cy + u0 * ry * s1)
      p10 = (cx + u1 * rx * c0, cy + u1 * ry * s0)
      p11 = (cx + u1 * rx * c1, cy + u1 * ry * s1)
      k00 = _gradient_at(stops, u0 * edge[j] / radius)
      k01 = _gradient_at(stops, u0 * edge[j2] / radius)
      k10 = _gradient_at(stops, u1 * edge[j] / radius)
      k11 = _gradient_at(stops, u1 * edge[j2] / radius)
      if u0 > 0:
        _emit((p00, p01, p11), (k00, k01, k11))
      _emit((p00, p11, p10), (k00, k11, k10))
  rl.rl_end()


def glow_stops(base: RGBA, inner: int, middle: int) -> tuple[tuple[float, RGBA], ...]:
  r, g, b, _ = base
  return (0.0, (r, g, b, inner)), (0.45, (r, g, b, middle)), (1.0, (r, g, b, 0))


def draw_rounded_rect(x: float, y: float, w: float, h: float, radius: float, rgba: RGBA) -> None:
  if w <= 0 or h <= 0:
    return
  roundness = clamp(radius / (min(w, h) / 2), 0.0, 1.0) if radius > 0 else 0.0
  rl.draw_rectangle_rounded(rl.Rectangle(x, y, w, h), roundness, 12, color(rgba))


def draw_polygon_outline(points: np.ndarray, rgba: RGBA, width: float) -> None:
  """QPen(width, RoundCap, RoundJoin) along a closed ribbon outline."""
  if len(points) < 2 or width <= 0:
    return
  c = color(rgba)
  closed = np.vstack((points, points[:1]))
  for (x0, y0), (x1, y1) in zip(closed[:-1], closed[1:], strict=True):
    rl.draw_line_ex(rl.Vector2(float(x0), float(y0)), rl.Vector2(float(x1), float(y1)), width, c)


def draw_rounded_corners(rect: tuple[float, float, float, float], radius: float, rgba: RGBA, segments: int = 16) -> None:
  """Cover the four outside corners so the view reads as one rounded rectangle."""
  x, y, w, h = rect
  radius = min(radius, w / 2, h / 2)
  if radius <= 0:
    return
  corners = (  # (corner point, arc centre, start angle in degrees; screen y points down)
    ((x, y), (x + radius, y + radius), 180.0),
    ((x + w, y), (x + w - radius, y + radius), 270.0),
    ((x + w, y + h), (x + w - radius, y + h - radius), 0.0),
    ((x, y + h), (x + radius, y + h - radius), 90.0),
  )
  c = (rgba, rgba, rgba)
  rl.rl_begin(rl.RL_TRIANGLES)
  for corner, (cx, cy), start in corners:
    arc = [(cx + radius * math.cos(math.radians(start + 90.0 * i / segments)),
            cy + radius * math.sin(math.radians(start + 90.0 * i / segments))) for i in range(segments + 1)]
    for p0, p1 in zip(arc[:-1], arc[1:], strict=True):
      _emit((corner, p0, p1), c)
  rl.rl_end()
