import ast
import json
import time
from pathlib import Path
from types import SimpleNamespace as NS

import pytest
from aiohttp import web
from aiohttp.test_utils import TestClient, TestServer

from openpilot.selfdrive.carrot.bluetooth import model as b1_model
from openpilot.selfdrive.carrot.bluetooth.b1_monitor import Intent, Monitor
from openpilot.selfdrive.carrot.server.tests.test_assistant_settings_http import load_routes


class FakeMonitor:
  def __init__(self, result=None):
    self.result = result if result is not None else {'op': 0, 'seq': 1, 'ttl_ms': 350, 'stamp': 0.0}
    self.acks = []

  def wifi_intent(self, ack):
    self.acks.append(ack)
    return dict(self.result)


def _app(monitor):
  routes = load_routes('b1_wifi')
  app = web.Application()
  if monitor is not None:
    app[routes.B1] = monitor
  routes.register(app)
  return app, routes


@pytest.mark.asyncio
async def test_intent_returns_monitor_result_and_validates_ack():
  monitor = FakeMonitor({'op': 1, 'seq': 9, 'ttl_ms': 200, 'stamp': 12.5})
  app, _routes = _app(monitor)
  async with TestClient(TestServer(app)) as client:
    response = await client.get('/api/b1/intent?ack=7')
    assert response.status == 200
    assert await response.json() == {'op': 1, 'seq': 9, 'ttl_ms': 200, 'stamp': 12.5}
    assert monitor.acks == [7]
    response = await client.get('/api/b1/intent')
    assert response.status == 200
    assert monitor.acks == [7, 0]
    assert (await client.get('/api/b1/intent?ack=abc')).status == 400
    assert (await client.get('/api/b1/intent?ack=-1')).status == 400
    assert (await client.get('/api/b1/intent?ack=70000')).status == 400


@pytest.mark.asyncio
async def test_intent_without_monitor_is_unavailable():
  app, _routes = _app(None)
  async with TestClient(TestServer(app)) as client:
    assert (await client.get('/api/b1/intent')).status == 503


@pytest.mark.asyncio
async def test_bsm_posts_state_and_rejects_bad_bodies(tmp_path, monkeypatch):
  state_path = tmp_path / 'bsm.json'
  monkeypatch.setattr(b1_model, 'B1_BSM_PATH', state_path)
  app, _routes = _app(FakeMonitor())
  async with TestClient(TestServer(app)) as client:
    response = await client.post('/api/b1/bsm', json={'left': True, 'right': False, 't': 1.23})
    assert response.status == 200
    assert await response.json() == {'ok': True}
    state = json.loads(state_path.read_text())
    assert state['left'] is True and state['right'] is False
    assert state['time'] > 0
    assert (await client.post('/api/b1/bsm', json={'left': 1, 'right': False})).status == 400
    assert (await client.post('/api/b1/bsm', json={'left': True})).status == 400
    assert (await client.post('/api/b1/bsm', data='{not json')).status == 400
    assert (await client.post('/api/b1/bsm', data='x' * 2048)).status == 413


def test_port80_app_has_only_b1_routes():
  routes = load_routes('b1_wifi')
  app = routes.create_port80_app(FakeMonitor())
  paths = sorted({resource.canonical for resource in app.router.resources()})
  assert paths == ['/api/b1/bsm', '/api/b1/intent']


def test_bsm_state_expires_after_timeout(tmp_path, monkeypatch):
  state_path = tmp_path / 'bsm.json'
  monkeypatch.setattr(b1_model, 'B1_BSM_PATH', state_path)
  assert b1_model.read_b1_bsm(now=100.0) == (False, False)
  b1_model.write_b1_bsm(True, False, now=100.0)
  assert b1_model.read_b1_bsm(now=101.5) == (True, False)
  assert b1_model.read_b1_bsm(now=102.5) == (False, False)
  state_path.write_text('{broken json')
  assert b1_model.read_b1_bsm(now=101.0) == (False, False)
  state_path.write_text('{"left": true}')
  assert b1_model.read_b1_bsm(now=101.0) == (False, False)


def _monitor_with_fake_worker(source):
  monitor = Monitor.__new__(Monitor)
  monitor.intent = Intent()
  monitor.source = None
  monitor.params = NS(get_bool=lambda name: name == 'IsOnroad')
  monitor.output_owner = NS(acquire=lambda ready: ready, release=lambda: None)
  monitor.wifi_last_poll = 0.
  monitor.last_command = None
  monitor.last_signal_at = 0.
  sent = []
  monitor.worker = NS(
    process=NS(poll=lambda: None),
    send=lambda value: sent.append(value) or True,
    latest=lambda: dict(source),
    close=lambda wait=True: None,
  )
  monitor._sent = sent
  return monitor


def _fresh_source(**overrides):
  source = {'ok': True, 'stamp': time.monotonic(), 'events': [], 'hold': None,
            'devices': [], 'b1_hold': True}
  source.update(overrides)
  return source


def test_wifi_intent_mints_a_fresh_seq_every_poll():
  monitor = _monitor_with_fake_worker(_fresh_source())
  first = monitor.wifi_intent(0)
  second = monitor.wifi_intent(first['seq'])
  assert first['op'] == 0 and second['op'] == 0
  assert 0 < first['seq'] < second['seq'] <= 65535
  assert 1 <= first['ttl_ms'] <= 350 and 1 <= second['ttl_ms'] <= 350


def test_wifi_intent_dispatches_one_event_then_keepalive():
  monitor = _monitor_with_fake_worker(_fresh_source())
  event = {'id': 'tap-1', 'time': time.monotonic(), 'direction': 'left'}
  monitor.worker.latest = lambda: _fresh_source(events=[event])
  first = monitor.wifi_intent(0)
  assert first['op'] == 1
  assert monitor.intent.inflight[0] == first['seq']
  assert monitor.last_command['id'] == 'tap-1'
  # The board acked the dispatched seq: the same event must not re-fire.
  second = monitor.wifi_intent(first['seq'])
  assert second['op'] == 0
  assert second['seq'] != first['seq']


def test_wifi_intent_stops_when_source_is_stale_or_output_blocked():
  stale = _fresh_source(stamp=time.monotonic() - 5)
  monitor = _monitor_with_fake_worker(stale)
  assert monitor.wifi_intent(0)['op'] == 3
  monitor = _monitor_with_fake_worker(_fresh_source())
  monitor.params = NS(get_bool=lambda name: False)
  assert monitor.wifi_intent(0)['op'] == 3


def test_stop_source_keeps_worker_while_wifi_is_polling():
  monitor = _monitor_with_fake_worker(_fresh_source())
  closed = []
  monitor.worker = NS(process=NS(poll=lambda: None), send=lambda v: True,
                      latest=lambda: None, close=lambda wait=True: closed.append(True))
  monitor.wifi_last_poll = time.monotonic()
  monitor.source = {'ok': True}
  monitor.stop_source()
  assert closed == [] and monitor.worker is not None
  assert monitor.source is None
  monitor.wifi_last_poll = 0.
  monitor.stop_source()
  assert closed == [True] and monitor.worker is None


def test_card_state_update_merges_b1_bsm_after_xiaoge():
  path = Path(__file__).parents[3] / 'car' / 'card.py'
  tree = ast.parse(path.read_text())
  method = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == 'state_update')
  calls = [n for n in ast.walk(method) if isinstance(n, ast.Call)]
  xiaoge_line = next(n.lineno for n in calls if isinstance(n.func, ast.Name) and n.func.id == 'apply_xiaoge_vision_result')
  bsm_line = next(n.lineno for n in calls if isinstance(n.func, ast.Name) and n.func.id == 'read_b1_bsm')
  assert bsm_line > xiaoge_line
