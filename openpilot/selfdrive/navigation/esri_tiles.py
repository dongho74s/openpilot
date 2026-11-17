"""Esri World Imagery raster tiles for the satellite map background; no API key.

Tiles are decoded to a single luminance channel for the grayscale model
map, or kept as RGB for the color display preview. Tiles are cached in
memory and on disk (tmpfs) so a satellite view is fetched once and reused;
a slow or failed fetch returns None and the caller falls back to the
vector-only render.
"""
import io
import os
import time
from pathlib import Path

import numpy as np
import requests

TILE_SIZE = 256
ESRI_URL = "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}"
CACHE_TTL_S = 7 * 24 * 3600  # imagery changes rarely
CACHE_LIMIT = 512  # tiles on disk
MEMORY_LIMIT = 64  # tiles in memory
FETCH_TIMEOUT = (2, 3)  # connect, read — never block the map
MAX_TILE_BYTES = 1024 * 1024


def _default_cache_dir():
  for base in ("/dev/shm", "/tmp"):
    try:
      path = Path(base) / "esri_tiles"
      path.mkdir(parents=True, exist_ok=True)
      return path
    except OSError:
      continue
  return None


class EsriTiles:
  """Esri World Imagery tiles as grayscale or RGB arrays, cached in memory and on disk.

  The on-disk cache stores the raw JPEG bytes (already color), so a tile is
  fetched once and decoded either way. The in-memory cache is keyed by the
  color flag so a grayscale and an RGB decode of the same tile coexist.
  """
  min_zoom, max_zoom = 0, 19

  def __init__(self, cache_dir=None, session=None):
    if cache_dir is not None:
      try:
        self.cache_dir = Path(cache_dir)
        self.cache_dir.mkdir(parents=True, exist_ok=True)
      except OSError:
        self.cache_dir = None
    else:
      self.cache_dir = _default_cache_dir()
    self.session = session if session is not None else requests.Session()
    self._memory: dict[tuple[int, int, int, bool], np.ndarray] = {}

  def _path(self, zoom, x, y):
    if self.cache_dir is None:
      return None
    return self.cache_dir / f"{zoom}_{x}_{y}.jpg"

  @staticmethod
  def _decode(data, color=False):
    from PIL import Image
    try:
      with Image.open(io.BytesIO(data)) as image:
        if image.size != (TILE_SIZE, TILE_SIZE):
          return None
        if color:
          return np.asarray(image.convert("RGB"), dtype=np.uint8).copy()
        return np.asarray(image.convert("L"), dtype=np.uint8).copy()
    except (OSError, ValueError):
      return None

  def _read_disk(self, zoom, x, y, color=False):
    path = self._path(zoom, x, y)
    if path is None:
      return None
    try:
      if not path.exists() or time.time() - path.stat().st_mtime > CACHE_TTL_S:
        return None
      return self._decode(path.read_bytes(), color=color)
    except OSError:
      return None

  def _store_disk(self, zoom, x, y, data):
    path = self._path(zoom, x, y)
    if path is None:
      return
    try:
      temporary = path.with_suffix(".tmp")
      temporary.write_bytes(data)
      os.replace(temporary, path)
      self._trim_disk()
    except OSError:
      pass

  def _trim_disk(self):
    if self.cache_dir is None:
      return
    try:
      files = [(cached.stat().st_mtime, cached) for cached in self.cache_dir.glob("*.jpg")]
    except OSError:
      return
    for _, cached in sorted(files)[:max(0, len(files) - CACHE_LIMIT)]:
      try:
        cached.unlink()
      except OSError:
        pass

  def _fetch(self, zoom, x, y):
    url = ESRI_URL.format(z=zoom, y=y, x=x)
    try:
      started = time.monotonic()
      with self.session.get(url, timeout=FETCH_TIMEOUT, stream=True) as response:
        response.raise_for_status()
        body = bytearray()
        for chunk in response.iter_content(65536):
          body.extend(chunk)
          if len(body) > MAX_TILE_BYTES or time.monotonic() - started > 8:
            return None
        return bytes(body)
    except requests.RequestException:
      return None

  def get(self, zoom, x, y, color=False):
    """A (256, 256) uint8 grayscale tile, or a (256, 256, 3) uint8 RGB tile
    when color=True; None if unavailable."""
    key = (zoom, x, y, color)
    tile = self._memory.get(key)
    if tile is not None:
      return tile
    tile = self._read_disk(zoom, x, y, color=color)
    if tile is None:
      data = self._fetch(zoom, x, y)
      if data is None:
        return None
      tile = self._decode(data, color=color)
      if tile is None:
        return None
      self._store_disk(zoom, x, y, data)
    if len(self._memory) >= MEMORY_LIMIT:
      self._memory.clear()
    self._memory[key] = tile
    return tile

  def close(self):
    self.session.close()
