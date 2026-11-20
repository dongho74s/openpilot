"""raylib helpers for the v10-nui tici screen (Qt QPainter equivalents)."""
import time

import numpy as np
import pyray as rl

from openpilot.selfdrive.ui.onroad.v10_nui.state import RGBA, clamp
from openpilot.system.ui.lib.application import ASSETS_DIR, gui_app
from openpilot.system.ui.lib.text_measure import measure_text_cached

ICON_DIR = "icons/v10_nui"
DM_DIR = "icons_mici/onroad/driver_monitoring"

LEFT, CENTER, RIGHT = "left", "center", "right"
GLOW_DOWNSCALE = 2
TOP, BOTTOM = "top", "bottom"



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


def glow_rgba(width: int, height: int, radius: float, stops: tuple[tuple[float, RGBA], ...]) -> np.ndarray:
  """Pixels of QPainter drawEllipse(0, 0, width, height) filled with QRadialGradient(centre, radius, stops)."""
  ys, xs = np.mgrid[0:height, 0:width].astype(np.float32) + 0.5
  rx, ry = width / 2.0, height / 2.0
  dx, dy = xs - rx, ys - ry
  t = np.hypot(dx, dy) / max(radius, 1e-6)
  positions = np.array([p for p, _ in stops], dtype=np.float32)
  channels = np.array([c for _, c in stops], dtype=np.float32)
  rgba = np.stack([np.interp(t, positions, channels[:, i]) for i in range(4)], axis=-1)
  # Antialiased ellipse edge, about one pixel wide.
  edge = (1.0 - np.hypot(dx / rx, dy / ry)) * min(rx, ry) + 0.5
  rgba[..., 3] *= np.clip(edge, 0.0, 1.0)
  return np.clip(np.rint(rgba), 0, 255).astype(np.uint8)


def corner_rgba(radius: int, rgba: RGBA) -> np.ndarray:
  """Top-left screen corner outside a quarter circle of this radius (antialiased), clear inside."""
  ys, xs = np.mgrid[0:radius, 0:radius].astype(np.float32) + 0.5
  coverage = np.clip(np.hypot(radius - xs, radius - ys) - radius + 0.5, 0.0, 1.0)
  out = np.empty((radius, radius, 4), dtype=np.uint8)
  out[..., :3] = rgba[:3]
  out[..., 3] = np.rint(coverage * rgba[3]).astype(np.uint8)
  return out


_baked: dict = {}


def _baked_texture(key, make) -> rl.Texture:
  """Upload generated pixels once; drawing is then one textured quad per frame."""
  tex = _baked.get(key)
  if tex is None:
    pixels = make()
    h, w = pixels.shape[:2]
    image = rl.gen_image_color(w, h, rl.BLANK)
    rl.image_format(image, rl.PixelFormat.PIXELFORMAT_UNCOMPRESSED_R8G8B8A8)
    rl.ffi.memmove(image.data, np.ascontiguousarray(pixels).tobytes(), w * h * 4)
    tex = rl.load_texture_from_image(image)
    rl.set_texture_filter(tex, rl.TextureFilter.TEXTURE_FILTER_BILINEAR)
    rl.set_texture_wrap(tex, rl.TextureWrap.TEXTURE_WRAP_CLAMP)
    rl.unload_image(image)
    _baked[key] = tex
  return tex


def release_baked_textures() -> None:
  for tex in _baked.values():
    rl.unload_texture(tex)
  _baked.clear()


def bake_radial_glow(width: float, height: float, radius: float, stops: tuple[tuple[float, RGBA], ...]) -> "rl.Texture | None":  # pyray types are functions
  # A soft gradient: half resolution, stretched with bilinear filtering, is visually the same and 4x cheaper to bake.
  iw, ih = int(round(width / GLOW_DOWNSCALE)), int(round(height / GLOW_DOWNSCALE))
  if iw <= 0 or ih <= 0 or radius <= 0:
    return None
  key = ("glow", iw, ih, round(float(radius), 2), tuple(stops))
  return _baked_texture(key, lambda: glow_rgba(iw, ih, radius / GLOW_DOWNSCALE, stops))


def draw_radial_glow(rect: tuple[float, float, float, float], radius: float, stops: tuple[tuple[float, RGBA], ...]) -> None:
  """QPainter drawEllipse(rect) with QRadialGradient(rect centre, radius, stops)."""
  x, y, w, h = rect
  tex = bake_radial_glow(w, h, radius, stops)
  if tex is not None:
    draw_texture_rect(tex, x, y, w, h)


def bake_rounded_corner(radius: float, rgba: RGBA) -> rl.Texture:
  r = int(round(radius))
  return _baked_texture(("corner", r, tuple(rgba)), lambda: corner_rgba(r, rgba))


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


def draw_rounded_corners(rect: tuple[float, float, float, float], radius: float, rgba: RGBA) -> None:
  """Cover the four outside corners so the area reads as one rounded rectangle."""
  x, y, w, h = rect
  r = int(round(min(radius, w / 2, h / 2)))
  if r <= 0:
    return
  tex = bake_rounded_corner(r, rgba)
  for cx, cy, flip_x, flip_y in ((x, y, 1, 1), (x + w - r, y, -1, 1), (x, y + h - r, 1, -1), (x + w - r, y + h - r, -1, -1)):
    rl.draw_texture_pro(tex, rl.Rectangle(0, 0, flip_x * r, flip_y * r), rl.Rectangle(cx, cy, r, r), rl.Vector2(0, 0), 0.0,
                        rl.WHITE)
