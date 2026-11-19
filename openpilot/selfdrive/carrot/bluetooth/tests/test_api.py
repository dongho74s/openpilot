import asyncio
import importlib.util
from pathlib import Path
import time

from aiohttp import web
from aiohttp.test_utils import TestClient, TestServer

from openpilot.selfdrive.carrot.bluetooth.model import DEFAULT_MAPPING, atomic_json, config, read_json

MAC = '66:C0:0C:7B:6E:71'


def test_stationary_origin_mapping_and_no_http_command_endpoint(tmp_path, monkeypatch):
  spec = importlib.util.spec_from_file_location('bluetooth_feature_test', Path(__file__).parents[2] / 'server/features/bluetooth.py')
  feature = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(feature)
  monkeypatch.setattr(feature, 'RUNTIME', tmp_path)
  monkeypatch.setattr(feature, 'CONFIG_PATH', tmp_path / 'config.json')
  monkeypatch.setattr(feature, 'config', lambda: config(tmp_path / 'config.json'))
  second = '00:11:22:33:44:55'

  class Client:
    async def snapshot(self):
      return {'devices': [{'address': MAC, 'paired': True}, {'address': second, 'paired': True}]}

    async def close(self):
      pass

  monkeypatch.setattr(feature, 'Bluez', Client)
  async def enabled():
    return True
  monkeypatch.setattr(feature, 'radio_enabled', enabled)

  async def run():
    app = web.Application()
    feature.register(app)
    async with TestClient(TestServer(app)) as client:
      result = await client.get('/api/bluetooth')
      assert result.status == 200
      assert (await result.json())['radioEnabled']
      payload = {'devices': {MAC: {'profile': 'yiser-j6', 'enabled': True, 'mapping': DEFAULT_MAPPING}}}
      assert (await client.post('/api/bluetooth/config', json=payload)).status == 409
      atomic_json(tmp_path / 'status.json', {'time': time.monotonic(), 'stationary': True})
      assert (await client.post('/api/bluetooth/config', json=payload, headers={'Origin': 'https://evil.example'})).status == 403
      assert (await client.post('/api/bluetooth/config', data='{}')).status == 415
      assert (await client.post('/api/bluetooth/config', json=payload)).status == 200
      second_device = {'profile': 'generic', 'enabled': False, 'mapping': {'key:115@double': 'carrotCruise'}}
      assert (await client.post('/api/bluetooth/device-config', json={'address': second, 'device': second_device})).status == 200
      assert feature.config()['devices'][MAC]['mapping'] == DEFAULT_MAPPING
      assert feature.config()['devices'][second]['mapping'] == second_device['mapping']
      # Saving one device cannot replace the other device's mapping.
      payload['devices'][MAC]['mapping'] = {'1@long': 'carrotCruise'}
      assert (await client.post('/api/bluetooth/device-config', json={'address': MAC, 'device': payload['devices'][MAC]})).status == 200
      assert feature.config()['devices'][second]['mapping'] == second_device['mapping']
      atomic_json(tmp_path / 'cruise.json', {'events': [{'address': MAC}, {'address': second}]})
      assert (await client.post('/api/bluetooth/learn', json={'address': MAC, 'enabled': True})).status == 200
      assert read_json(tmp_path / 'cruise.json')['events'] == [{'address': MAC}, {'address': second}]
      assert MAC in read_json(tmp_path / 'cancelled.json')
      assert (await client.post('/api/bluetooth/fire', json={'action': 'accelCruise'})).status == 404
      payload['devices'][MAC]['mapping'] = {'up': 'unrestrictedCommand'}
      assert (await client.post('/api/bluetooth/config', json=payload)).status == 400
      atomic_json(tmp_path / 'status.json', {'time': time.monotonic() - 3, 'stationary': True})
      assert (await client.post('/api/bluetooth/config', json=payload)).status == 409
  asyncio.run(run())

def test_auto_reconnect_connects_paired_disconnected_device(tmp_path, monkeypatch):
  spec = importlib.util.spec_from_file_location('bluetooth_feature_reconnect_test', Path(__file__).parents[2] / 'server/features/bluetooth.py')
  feature = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(feature)
  monkeypatch.setattr(feature, 'RUNTIME', tmp_path)
  monkeypatch.setattr(feature, 'CONFIG_PATH', tmp_path / 'config.json')
  monkeypatch.setattr(feature, 'config', lambda: config(tmp_path / 'config.json'))

  class Client:
    def __init__(self):
      self.actions = []
      self.devices = [{'address': MAC, 'paired': True, 'connected': False}]
      self.adapters = [{'powered': True}]

    async def snapshot(self):
      return {'devices': list(self.devices), 'adapters': list(self.adapters)}

    async def device_action(self, mac, operation):
      self.actions.append((mac, operation))

    async def close(self):
      pass

  def run_loop(client, seconds=0.12):
    async def run():
      task = asyncio.create_task(feature.auto_reconnect({feature.CLIENT: client}, interval_s=0.02))
      await asyncio.sleep(seconds)
      task.cancel()
      await asyncio.gather(task, return_exceptions=True)
    asyncio.run(run())

  client = Client()
  atomic_json(tmp_path / 'config.json', {'devices': {MAC: {'profile': 'yiser-j6', 'enabled': True, 'mapping': DEFAULT_MAPPING}}})
  run_loop(client)
  assert (MAC, 'connect') in client.actions

  # Already connected: no reconnect attempt.
  client.actions.clear()
  client.devices = [{'address': MAC, 'paired': True, 'connected': True}]
  run_loop(client)
  assert client.actions == []

  # Disabled in config: no reconnect attempt.
  client.actions.clear()
  client.devices = [{'address': MAC, 'paired': True, 'connected': False}]
  atomic_json(tmp_path / 'config.json', {'devices': {MAC: {'profile': 'yiser-j6', 'enabled': False, 'mapping': DEFAULT_MAPPING}}})
  run_loop(client)
  assert client.actions == []

  # Manually disconnected moments ago: suppressed for a while.
  client.actions.clear()
  atomic_json(tmp_path / 'config.json', {'devices': {MAC: {'profile': 'yiser-j6', 'enabled': True, 'mapping': DEFAULT_MAPPING}}})
  atomic_json(tmp_path / 'cancelled.json', {MAC: time.monotonic()})
  run_loop(client)
  assert client.actions == []

  # Adapter not powered: no reconnect attempt.
  client.actions.clear()
  (tmp_path / 'cancelled.json').unlink(missing_ok=True)
  client.adapters = [{'powered': False}]
  run_loop(client)
  assert client.actions == []
