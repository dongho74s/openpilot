from __future__ import annotations

import asyncio
import gzip
import importlib.util
import json

from aiohttp import web
from aiohttp.test_utils import TestClient, TestServer
import pytest

from openpilot.selfdrive.carrot.server.services import static_assets

HAS_BROTLI = importlib.util.find_spec("brotli") is not None
MODEL_PAYLOAD = json.dumps({"models": [{"name": "suv", "vertices": list(range(4000))}]}).encode()
SCRIPT_PAYLOAD = b"export const answer = 42;\n" * 400


@pytest.fixture
def web_dir(tmp_path):
  (tmp_path / "assets" / "traverse_live").mkdir(parents=True)
  (tmp_path / "assets" / "traverse_live" / "models.json").write_bytes(MODEL_PAYLOAD)
  (tmp_path / "assets" / "traverse_live" / "README.md").write_text("models\n")
  (tmp_path / "js" / "generated").mkdir(parents=True)
  (tmp_path / "js" / "generated" / "drive.js").write_bytes(SCRIPT_PAYLOAD)
  (tmp_path / "index.html").write_text("<!doctype html>")
  return tmp_path


def _assert_sidecars(source, payload):
  assert gzip.decompress(source.with_name(source.name + ".gz").read_bytes()) == payload
  brotli_sidecar = source.with_name(source.name + ".br")
  if HAS_BROTLI:
    import brotli
    assert brotli.decompress(brotli_sidecar.read_bytes()) == payload
  else:
    assert not brotli_sidecar.exists()


def test_precompress_covers_json_under_assets_and_keeps_js(web_dir):
  static_assets.precompress_static_assets(str(web_dir))

  _assert_sidecars(web_dir / "assets" / "traverse_live" / "models.json", MODEL_PAYLOAD)
  _assert_sidecars(web_dir / "js" / "generated" / "drive.js", SCRIPT_PAYLOAD)
  assert not (web_dir / "assets" / "traverse_live" / "README.md.gz").exists()
  assert not (web_dir / "index.html.gz").exists()


def test_requested_json_refreshes_a_stale_sidecar(web_dir):
  model = web_dir / "assets" / "traverse_live" / "models.json"
  static_assets.precompress_static_assets(str(web_dir))
  updated = MODEL_PAYLOAD.replace(b"suv", b"van")
  model.write_bytes(updated)

  static_assets._refresh_requested_gzip(str(web_dir), "/assets/traverse_live/models.json")

  _assert_sidecars(model, updated)


def test_json_is_compressed_but_never_fingerprinted(web_dir):
  assert static_assets.fingerprint_static_asset(web_dir, "/assets/traverse_live/models.json") is None
  assert static_assets.fingerprint_static_asset(web_dir, "/js/generated/drive.js") is not None


def test_json_outside_the_web_root_is_ignored(web_dir, tmp_path_factory):
  outside = tmp_path_factory.mktemp("outside") / "secret.json"
  outside.write_bytes(b'{"secret": true}')

  static_assets._refresh_requested_gzip(str(web_dir), "/assets/../../" + outside.parent.name + "/secret.json")

  assert not outside.with_name("secret.json.gz").exists()


def test_served_json_is_compressed(web_dir):
  async def fetch():
    app = web.Application(middlewares=[static_assets.create_static_cache_middleware(str(web_dir))])
    app.router.add_static("/", str(web_dir), show_index=False)
    async with TestClient(TestServer(app)) as client:
      response = await client.get("/assets/traverse_live/models.json",
                                  headers={"Accept-Encoding": "gzip, br"}, auto_decompress=False)
      body = await response.read()
      return response.status, response.headers.get("Content-Encoding"), response.headers.get("Cache-Control"), body

  status, encoding, cache_control, body = asyncio.run(fetch())

  assert status == 200
  assert encoding == ("br" if HAS_BROTLI else "gzip")
  if HAS_BROTLI:
    import brotli
    assert brotli.decompress(body) == MODEL_PAYLOAD
  else:
    assert gzip.decompress(body) == MODEL_PAYLOAD
  assert len(body) < len(MODEL_PAYLOAD) / 2
  # .json is never fingerprinted, so it must not be marked immutable
  assert cache_control is None
