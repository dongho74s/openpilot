#!/usr/bin/env python3
"""Generate the C4 Wanted Sans SemiBold atlases, leaving every bundled atlas untouched.

- WantedSans-SemiBold: 200px, the Latin/numeric set fonts/process.py uses for its base atlases.
- WantedSans-KR-SemiBold: 48px, that set plus all modern Hangul and the CJK-locale characters.

Wanted Sans has no glyph for a few characters the UI uses, notably U+2011 in the Korean "Wi‑Fi"
strings and some keyboard symbols. Pretendard SemiBold shares its 2048 UPM, 1950/-494 vertical metrics
and weight, so those glyphs are rasterized from the bundled Pretendard into the same atlas instead of
pushing the whole string to unifont. Anything neither face has still falls back to unifont at runtime.

The source TTF lives in fonts/wanted/ so the generic process.py glob (and its SCons command) never sees
it: adding it there would regenerate, and dirty, the tracked Pretendard atlases on the device.
"""
from pathlib import Path

import pyray as rl

from openpilot.selfdrive.assets.fonts import process

FONT_DIR = Path(__file__).resolve().parent
SOURCE = FONT_DIR / "wanted" / "WantedSans-SemiBold.ttf"
FILL = FONT_DIR / "Pretendard-SemiBold.ttf"
LATIN_STEM = "WantedSans-SemiBold"
KOREAN_STEM = "WantedSans-KR-SemiBold"


def _load_glyphs(path: Path, font_size: int, codepoints: list[int]):
  data = path.read_bytes()
  file_buf = rl.ffi.new("unsigned char[]", data)
  cp_buffer = rl.ffi.new("int[]", codepoints)
  glyph_count = rl.ffi.new("int *", len(codepoints))
  glyphs = rl.load_font_data(rl.ffi.cast("unsigned char *", file_buf), len(data), font_size,
                             rl.ffi.cast("int *", cp_buffer), len(codepoints), rl.FontType.FONT_DEFAULT, glyph_count)
  if glyphs == rl.ffi.NULL:
    raise RuntimeError(f"raylib failed to load font data from {path.name}")
  # raylib 6 returns only the codepoints the face has; older builds kept missing ones as empty entries.
  real = [glyphs[i] for i in range(glyph_count[0])
          if glyphs[i].advanceX > 0 or (glyphs[i].image.width > 0 and glyphs[i].image.height > 0)]
  return real, glyphs


def build(stem: str, font_size: int, padding: int, codepoints, output: Path, source: Path = SOURCE, fill: Path = FILL):
  codepoints = sorted(set(codepoints))
  print(f"Processing {stem} from {source.name}...")
  glyphs, keep = _load_glyphs(source, font_size, codepoints)
  found = {int(g.value) for g in glyphs}
  missing = [cp for cp in codepoints if cp not in found]
  keep_alive = [keep]
  if missing and not fill.exists():
    print(f"  WARNING: {fill} is missing; {len(missing)} codepoints stay without a glyph (unifont at runtime)")
  elif missing:
    filled, keep = _load_glyphs(fill, font_size, missing)
    keep_alive.append(keep)
    print(f"  {len(filled)} of {len(missing)} missing codepoints taken from {fill.name}")
    glyphs += filled

  merged = rl.ffi.new("GlyphInfo[]", len(glyphs))
  for i, glyph in enumerate(glyphs):
    merged[i] = glyph

  rects_ptr = rl.ffi.new("Rectangle **")
  image = rl.gen_image_font_atlas(rl.ffi.cast("GlyphInfo *", merged), rects_ptr, len(glyphs), font_size, padding, 0)
  if image.width == 0 or image.height == 0:
    raise RuntimeError("raylib returned an empty atlas")

  atlas_name = f"{stem}.png"
  entries, line_height, base = process._glyph_metrics(merged, rects_ptr[0], len(glyphs))
  if not rl.export_image(image, (output / atlas_name).as_posix()):
    raise RuntimeError("Failed to export atlas image")
  process._write_bmfont(output / f"{stem}.fnt", font_size, stem, atlas_name, line_height, base,
                        (image.width, image.height), entries)


def generate(output: Path = FONT_DIR, source: Path = SOURCE, fill: Path = FILL) -> None:
  base, international = process._char_sets()
  build(LATIN_STEM, 200, process.GLYPH_PADDING, base, output, source, fill)
  build(KOREAN_STEM, 48, 2, set(base) | set(international), output, source, fill)


if __name__ == "__main__":
  generate()
