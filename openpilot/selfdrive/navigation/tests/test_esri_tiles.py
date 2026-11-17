import io
from types import SimpleNamespace as NS

import numpy as np
import pytest
import requests
from PIL import Image

from openpilot.selfdrive.navigation.esri_tiles import EsriTiles, ESRI_URL


def jpeg_bytes(color=(10, 20, 30)):
  buffer = io.BytesIO()
  Image.new("RGB", (256, 256), color).save(buffer, format="JPEG")
  return buffer.getvalue()


class FakeResponse:
  def __init__(self, body, status=200):
    self._body = body
    self.status_code = status

  def __enter__(self):
    return self

  def __exit__(self, *args):
    return False

  def raise_for_status(self):
    if self.status_code != 200:
      raise requests.HTTPError(f"status {self.status_code}")

  def iter_content(self, chunk_size):
    yield self._body


def test_esri_tile_fetch_decode_and_cache(monkeypatch, tmp_path):
  calls = []

  def fake_get(url, **kwargs):
    calls.append(url)
    return FakeResponse(jpeg_bytes())

  monkeypatch.setattr(requests, "Session", lambda: NS(get=fake_get, close=lambda: None))
  tiles = EsriTiles(cache_dir=tmp_path)
  tile = tiles.get(17, 12345, 67890)
  assert tile is not None
  assert tile.shape == (256, 256) and tile.dtype == np.uint8
  # solid (10, 20, 30) is about 18 in luminance; JPEG is near-lossless for a flat color
  assert abs(int(tile.mean()) - 18) <= 2
  assert calls == [ESRI_URL.format(z=17, y=67890, x=12345)]
  # a second request for the same tile is served from the disk cache, no network call
  again = tiles.get(17, 12345, 67890)
  assert np.array_equal(again, tile)
  assert len(calls) == 1
  tiles.close()


def test_esri_tile_network_error_returns_none(monkeypatch, tmp_path):
  def fail(*args, **kwargs):
    raise requests.RequestException("down")

  monkeypatch.setattr(requests, "Session", lambda: NS(get=fail, close=lambda: None))
  tiles = EsriTiles(cache_dir=tmp_path)
  assert tiles.get(17, 1, 2) is None
  tiles.close()


def test_esri_tile_bad_image_returns_none(monkeypatch, tmp_path):
  monkeypatch.setattr(requests, "Session",
                      lambda: NS(get=lambda *a, **k: FakeResponse(b"not an image"), close=lambda: None))
  tiles = EsriTiles(cache_dir=tmp_path)
  assert tiles.get(17, 1, 2) is None
  tiles.close()
