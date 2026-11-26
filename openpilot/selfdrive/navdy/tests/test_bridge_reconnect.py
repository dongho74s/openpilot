import json
from types import SimpleNamespace as NS

import pytest

from openpilot.selfdrive.navdy import bridge
from openpilot.selfdrive.navdy.tests.test_bridge import Messages


def healthy_session(reconnect, start=100.):
  for offset in (0., .5, 1., 1.5, 2.):
    reconnect.acknowledge(start + offset, start + offset)


@pytest.mark.parametrize('arrivals', [[], [100.], [100., 100.5, 101., 101.5],
                                    [100., 102.], [100., 100., 100., 100.]])
def test_uptime_sparse_or_duplicate_acks_cannot_earn_fast_retry(arrivals):
  reconnect = bridge.BluetoothReconnect()
  for received in arrivals:
    reconnect.acknowledge(received, received)
  assert not reconnect.take_fast_retry(102.1)


@pytest.mark.parametrize('now,allowed', [(102., True), (104., True), (104.001, False), (101.9, False)])
def test_fast_retry_requires_recent_healthy_ack_history(now, allowed):
  reconnect = bridge.BluetoothReconnect()
  healthy_session(reconnect)
  assert reconnect.take_fast_retry(now) is allowed


@pytest.mark.parametrize('received', [float('nan'), float('inf'), 103., 99.])
def test_invalid_ack_time_discards_health(received):
  reconnect = bridge.BluetoothReconnect()
  healthy_session(reconnect)
  reconnect.acknowledge(received, 102.)
  assert not reconnect.take_fast_retry(102.)


def test_new_session_must_requalify_without_resetting_fast_retry_cooldown():
  reconnect = bridge.BluetoothReconnect()
  healthy_session(reconnect)
  assert reconnect.take_fast_retry(102.)
  assert not reconnect.take_fast_retry(102.)
  reconnect.reset_session()
  assert not reconnect.healthy(102.)
  healthy_session(reconnect, 128.)
  assert not reconnect.take_fast_retry(131.999)
  assert reconnect.take_fast_retry(132.)
  reconnect.reset_session()
  reconnect.acknowledge(164., 164.)
  assert not reconnect.take_fast_retry(164.)


def run_wireless_bridge(monkeypatch, tmp_path, exchange, *, connect=None, duration=5., bluetooth_only=True, usb_at=None):
  from openpilot import cereal

  state = NS(now=100., attempts=[], sent=[], statuses=[], feedback_writes=[], usb_attempts=[], closes=[],
             feedback=tmp_path / 'feedback.json', sm=Messages(), freeze_sources=False)
  state.reply = lambda: [({'navigationFeedback': None}, state.now, 0.)]
  monkeypatch.setattr(bridge.time, 'monotonic', lambda: state.now)
  monkeypatch.setattr(bridge.time, 'monotonic_ns', lambda: round(state.now * 1e9))
  monkeypatch.setattr(bridge.time, 'sleep', lambda seconds: setattr(state, 'now', round(state.now + seconds, 9)))
  monkeypatch.setattr(bridge, 'read_override', lambda: None)
  monkeypatch.setattr(bridge, 'DoorMonitor', lambda camera=None: NS(camera_error='', start=lambda: None, close=lambda: None,
                                                     door_open=False, gm=True, error=''))
  monkeypatch.setattr(bridge, 'configured_connection', lambda _: ('54:ED:A3:00:48:23', bluetooth_only))
  monkeypatch.setattr(bridge, 'jetson_candidates', list)

  def update(_):
    if not state.freeze_sources:
      state.sm.recv_time = dict.fromkeys(bridge.SERVICES, state.now)
      state.sm.logMonoTime = dict.fromkeys(bridge.SERVICES, round(state.now * 1e9))
      state.sm.data['carState'].vEgo = state.sm.data['carState'].vEgoCluster = state.now - 100.

  state.sm.update = update
  monkeypatch.setattr(cereal, 'messaging', NS(SubMaster=lambda _: state.sm), raising=False)

  class Wireless:
    def __init__(self, peer):
      self.peer = peer

    def connect(self):
      self.attempt = len(state.attempts) + 1
      state.attempts.append(state.now)
      if connect is not None:
        connect(self, state)
      self.connected_at = state.now

    def exchange_batch(self, payload, *, captured_ns):
      assert captured_ns == round(state.now * 1e9)
      state.sent.append((self.attempt, state.now, payload))
      return exchange(self, state)

    def close(self):
      state.closes.append(self.attempt)

  class Usb:
    def connect(self):
      assert not bluetooth_only
      state.usb_attempts.append(state.now)
      if usb_at is None or state.now < usb_at:
        raise ConnectionError('No cable')

    def exchange(self, payload):
      return {'navigationFeedback': None}

    def close(self):
      pass

  monkeypatch.setattr(bridge, 'BluetoothTransport', Wireless)
  monkeypatch.setattr(bridge, 'AdbTransport', Usb)
  monkeypatch.setattr(bridge, 'UsbProbe', lambda _: NS(poll=lambda now: None, close=lambda: None))
  original_write = bridge.atomic_json

  def write(path, value):
    if path.name == 'status.json':
      state.statuses.append(value)
      if not value.get('connected'):
        assert not state.feedback.exists()
    elif path == state.feedback:
      state.feedback_writes.append(value)
    original_write(path, value)

  monkeypatch.setattr(bridge, 'atomic_json', write)
  bridge.run(feedback=state.feedback, status=tmp_path / 'status.json', duration=duration)
  assert not state.feedback.exists()
  assert json.loads((tmp_path / 'status.json').read_text())['reason'] == 'bridge_stopped'
  state.failures = [s for s in state.statuses if 'retryInSeconds' in s]
  state.delays = [s['retryInSeconds'] for s in state.failures]
  return state


@pytest.mark.parametrize('fault,delay', [(ConnectionError, .2), (TimeoutError, .2), (OSError, .2), (ValueError, 2.)])
def test_healthy_exchange_loss_gets_one_fast_retry_but_protocol_errors_do_not(monkeypatch, tmp_path, fault, delay):
  def exchange(link, state):
    if link.attempt == 1 and state.now >= 102.6:
      raise fault('Link interrupted')
    return state.reply()

  state = run_wireless_bridge(monkeypatch, tmp_path, exchange)
  assert state.delays == [delay]
  assert state.attempts == pytest.approx([100., 102.6 + delay])
  assert state.failures[0]['diagnostics']['lastFailure']['phase'] == 'exchange'
  assert state.closes == [1, 2]


@pytest.mark.parametrize('mode', ['empty', 'one_ack', 'short', 'old_ack', 'sparse'])
def test_no_fast_retry_for_unproven_or_silent_sessions(monkeypatch, tmp_path, mode):
  def exchange(link, state):
    elapsed = state.now - link.connected_at
    if elapsed >= (1. if mode == 'short' else 3.):
      raise TimeoutError('No healthy session')
    if mode == 'short' or (mode == 'one_ack' and elapsed == 0):
      return state.reply()
    if mode == 'old_ack':
      return [({}, state.now - 2.1, 0.)]
    if mode == 'sparse' and elapsed in (0., 2.):
      return state.reply()
    return []

  state = run_wireless_bridge(monkeypatch, tmp_path, exchange, duration=4.)
  assert state.delays == [2.]


@pytest.mark.parametrize('healthy_first', [False, True])
def test_failed_connects_and_empty_polls_keep_bounded_backoff(monkeypatch, tmp_path, healthy_first):
  def connect(link, state):
    if not healthy_first or link.attempt > 2:
      raise ConnectionError('Peer unavailable')

  def exchange(link, state):
    if state.now - link.connected_at >= 2.6:
      raise TimeoutError('Radio lost')
    return state.reply() if link.attempt == 1 else []

  state = run_wireless_bridge(monkeypatch, tmp_path, exchange, connect=connect, duration=35.)
  assert state.delays == ([.2, 4., 8., 10., 10.] if healthy_first else [2., 4., 8., 10., 10., 10.])
  for failure, attempt in zip(state.failures, state.attempts[1:], strict=False):
    assert attempt - failure['updatedAt'] == pytest.approx(failure['retryInSeconds'])


def test_repeated_healthy_flaps_cannot_rearm_fast_retry_inside_cooldown(monkeypatch, tmp_path):
  def exchange(link, state):
    if state.now - link.connected_at >= 2.6:
      raise ConnectionError('Flapping peer')
    return state.reply()

  state = run_wireless_bridge(monkeypatch, tmp_path, exchange, duration=42.)
  fast = [s['updatedAt'] for s in state.failures if s['retryInSeconds'] == .2]
  assert len(fast) == 2
  assert fast[1] - fast[0] >= bridge.BLUETOOTH_FAST_RETRY_COOLDOWN
  assert all(s['retryInSeconds'] == 2. for s in state.failures if s['updatedAt'] not in fast)


@pytest.mark.parametrize('freeze_sources', [False, True])
def test_fast_reconnect_refreshes_payload_without_replaying_feedback(monkeypatch, tmp_path, freeze_sources):
  def connect(link, state):
    if link.attempt == 2:
      assert not state.feedback.exists()
      state.now += 1.

  def exchange(link, state):
    if link.attempt == 1 and state.now >= 102.6:
      state.freeze_sources = freeze_sources
      raise ConnectionError('Transient reset')
    return state.reply() if link.attempt == 1 else []

  state = run_wireless_bridge(monkeypatch, tmp_path, exchange, connect=connect)
  assert state.delays == [.2]
  first_reconnected = next(payload for attempt, _, payload in state.sent if attempt == 2)
  assert first_reconnected['active'] is not freeze_sources
  assert first_reconnected['vEgoKph'] == pytest.approx(0. if freeze_sources else round(3.8 * 3.6, 1))
  assert state.feedback_writes[-1]['receivedAt'] == pytest.approx(102.4)
  assert all(s['extendedFeedback'] is False for s in state.statuses if s.get('connectedAt') == 103.8)


def test_fast_wireless_retry_still_checks_usb_first(monkeypatch, tmp_path):
  def exchange(link, state):
    if state.now >= 102.6:
      raise ConnectionError('Wireless reset as cable arrives')
    return state.reply()

  state = run_wireless_bridge(monkeypatch, tmp_path, exchange, bluetooth_only=False, usb_at=102.7)
  assert state.delays == [.2]
  assert state.attempts == [100.]
  assert state.usb_attempts == pytest.approx([100., 102.8])
  assert state.statuses[-2]['transport'] == 'usb'


@pytest.mark.parametrize('phase', ['payload', 'feedback', 'status'])
def test_non_exchange_errors_do_not_consume_healthy_fast_retry(monkeypatch, tmp_path, phase):
  failed = False

  def exchange(link, state):
    if link.attempt == 2 and state.now - link.connected_at >= 2.6:
      raise ConnectionError('Later transient radio failure')
    return state.reply()

  def fail_once():
    nonlocal failed
    if bridge.time.monotonic() >= 102.6 and not failed:
      failed = True
      raise OSError('Non-radio failure')

  if phase == 'payload':
    original_payload = bridge.live_payload

    def payload(*args, **kwargs):
      fail_once()
      return original_payload(*args, **kwargs)

    monkeypatch.setattr(bridge, 'live_payload', payload)
  elif phase == 'feedback':
    original_age = bridge.age_model_feedback

    def age(*args):
      fail_once()
      return original_age(*args)

    monkeypatch.setattr(bridge, 'age_model_feedback', age)
  else:
    original_write = bridge.atomic_json

    def write(path, value):
      if path.name == 'status.json' and value.get('connected'):
        fail_once()
      original_write(path, value)

    monkeypatch.setattr(bridge, 'atomic_json', write)

  state = run_wireless_bridge(monkeypatch, tmp_path, exchange, duration=8.)
  assert state.delays == [2., .2]
  assert state.failures[0]['diagnostics']['lastFailure']['phase'] == phase


def run_usb_bridge(monkeypatch, tmp_path, *, drop_at, down_for=0., duration=8., healthy=True, peer=None, replies=True,
                   close_takes=0.):
  """USB-only bridge: the cable session drops once at drop_at ('Navdy closed its socket', an adb Protocol error on the
  road, 2026-11-26) and the device is back after down_for."""
  from openpilot import cereal

  state = NS(now=100., usb_attempts=[], usb_opened=[], relay_attempts=[], statuses=[], sm=Messages(), dropped=False)
  monkeypatch.setattr(bridge.time, 'monotonic', lambda: state.now)
  monkeypatch.setattr(bridge.time, 'monotonic_ns', lambda: round(state.now * 1e9))
  monkeypatch.setattr(bridge.time, 'sleep', lambda seconds: setattr(state, 'now', round(state.now + seconds, 9)))
  monkeypatch.setattr(bridge, 'read_override', lambda: None)
  monkeypatch.setattr(bridge, 'DoorMonitor', lambda camera=None: NS(camera_error='', start=lambda: None, close=lambda: None,
                                                     door_open=False, gm=True, error=''))
  monkeypatch.setattr(bridge, 'configured_connection', lambda _: (peer, False))
  monkeypatch.setattr(bridge, 'jetson_candidates', lambda: ['jetson-nx'])
  state.bt_attempts = []

  def update(_):
    state.sm.recv_time = dict.fromkeys(bridge.SERVICES, state.now)
    state.sm.logMonoTime = dict.fromkeys(bridge.SERVICES, round(state.now * 1e9))

  state.sm.update = update
  monkeypatch.setattr(cereal, 'messaging', NS(SubMaster=lambda _: state.sm), raising=False)

  class Usb:
    def connect(self):
      state.usb_attempts.append(state.now)
      if state.dropped and state.now < drop_at + down_for:
        raise ConnectionError('Navdy USB not found')
      state.usb_opened.append(state.now)
      self.opened = state.now

    def exchange(self, payload):
      if not state.dropped and state.now >= drop_at and (healthy or state.now - self.opened < 2):
        state.dropped = True
        raise ConnectionError('Navdy closed its socket')
      if not replies:
        state.now = round(state.now + 0.8, 9)
        raise TimeoutError('timed out')
      return {'navigationFeedback': None}

    def close(self):
      state.now = round(state.now + close_takes, 9)

  class Wireless:
    def __init__(self, target):
      state.bt_attempts.append(state.now)

    def connect(self):
      raise ConnectionError('peer unavailable')

    def close(self):
      pass

  monkeypatch.setattr(bridge, 'BluetoothTransport', Wireless)

  class Relay:
    def __init__(self, host):
      state.relay_attempts.append(state.now)

    def connect(self):
      state.now = round(state.now + 4., 9)  # a Jetson connect that waits 4 s and fails
      raise ConnectionError('Jetson relay unavailable')

    def close(self):
      pass

  monkeypatch.setattr(bridge, 'AdbTransport', Usb)
  monkeypatch.setattr(bridge, 'SshTransport', Relay)
  monkeypatch.setattr(bridge, 'UsbProbe', lambda _: NS(poll=lambda now: None, close=lambda: None))
  original_write = bridge.atomic_json

  def write(path, value):
    if path.name == 'status.json':
      state.statuses.append(value)
    original_write(path, value)

  monkeypatch.setattr(bridge, 'atomic_json', write)
  bridge.run(feedback=tmp_path / 'feedback.json', status=tmp_path / 'status.json', duration=duration)
  state.delays = [s['retryInSeconds'] for s in state.statuses if 'retryInSeconds' in s]
  return state


def test_a_healthy_usb_drop_is_retried_within_a_quarter_second(monkeypatch, tmp_path):
  state = run_usb_bridge(monkeypatch, tmp_path, drop_at=103.)
  assert state.delays == [bridge.USB_QUICK_RETRY]
  assert state.usb_opened[1] - 103. <= bridge.USB_QUICK_RETRY + bridge.DISPLAY_INTERVAL + 1e-6  # was 2 s + connect


def test_while_the_cable_is_coming_back_only_the_cable_is_retried(monkeypatch, tmp_path):
  """No Jetson relay detour (a 4 s connect) inside USB_QUICK_WINDOW; the cable is back after 0.8 s."""
  state = run_usb_bridge(monkeypatch, tmp_path, drop_at=103., down_for=0.8)
  assert not [t for t in state.relay_attempts if t < 103. + bridge.USB_QUICK_WINDOW]
  assert set(state.delays) == {bridge.USB_QUICK_RETRY}
  assert state.usb_opened[1] - 103. < 1.3


def test_an_unproven_usb_session_keeps_the_backoff(monkeypatch, tmp_path):
  state = run_usb_bridge(monkeypatch, tmp_path, drop_at=100.5, healthy=False)
  assert state.delays[0] == 2.


def test_a_trusted_hud_reconnects_without_the_package_query_and_service_start(monkeypatch):
  from openpilot.selfdrive.navdy import transport as t
  calls = []

  def command(cmd, **kwargs):
    calls.append(cmd[3:] if cmd[3] == '-s' else cmd[3:])
    args = cmd[cmd.index('-s') + 2:] if '-s' in cmd else cmd[3:]
    out = {'forward': '45123\n', 'shell': 'package:/data/app/hud.apk\n'}.get(args[0], '')
    return NS(stdout=out, returncode=0)

  class Conn:
    def __init__(self, *a, **k): self.sent = []
    def settimeout(self, _): pass
    def sendall(self, data): self.sent.append(data)
    def recv(self, _): return b'{"navigationFeedback":null}\n'
    def close(self): pass

  monkeypatch.setattr(t.socket, 'create_connection', lambda *a, **k: Conn())
  monkeypatch.setitem(t._trusted, 'serial', None)
  inventory = lambda port: 'List of devices attached\nNAVDY01 device usb:3-1\n' if port == 5038 else ''

  def session(n):
    calls.clear()
    link = t.AdbTransport(adb='/usr/bin/adb', command=command, inventory=inventory)
    link.connect()
    for _ in range(n):
      link.exchange({'x': 1})
    link.close()
    return [c for c in calls if 'shell' in c]

  assert len(session(t.TRUSTED_EXCHANGES)) == 2  # pm path + am startservice the first time
  assert session(t.TRUSTED_EXCHANGES) == []  # trusted: straight to the forward and the socket
  assert session(1) == [] and t._trusted['serial'] is None  # still trusted, but it ended early: trust cleared
  assert len(session(1)) == 2  # so the next connect does the full checks again


def test_bluetooth_fallback_is_not_held_back_by_the_quick_usb_window(monkeypatch, tmp_path):
  """Codex review 2026-11-26 P2: usb_first with a paired peer still tries Bluetooth right after a cable drop."""
  state = run_usb_bridge(monkeypatch, tmp_path, drop_at=103., down_for=3., peer='54:ED:A3:00:48:23')
  assert state.bt_attempts and state.bt_attempts[0] - 103. < 0.6
  assert not [t for t in state.relay_attempts if t < 103. + bridge.USB_QUICK_WINDOW]


def test_a_session_without_replies_does_not_open_the_quick_window(monkeypatch, tmp_path):
  """Codex review 2026-11-26 P2: time spent in a slow cleanup is not a healthy session."""
  state = run_usb_bridge(monkeypatch, tmp_path, drop_at=1e9, replies=False, close_takes=3., duration=30.)
  assert bridge.USB_QUICK_RETRY not in state.delays and state.delays[:3] == [2, 4, 8]


def test_a_briefly_absent_hud_keeps_its_trust(monkeypatch):
  """Codex review 2026-11-26 P2: a discovery that does not find the trusted HUD (or a connect that never opened a
  session) leaves the trust alone."""
  from openpilot.selfdrive.navdy import transport as t
  monkeypatch.setitem(t._trusted, 'serial', 'NAVDY01')
  link = t.AdbTransport(adb='/usr/bin/adb', command=lambda *a, **k: NS(stdout='', returncode=0),
                        inventory=lambda port: 'List of devices attached\n' if port == 5038 else '')
  with pytest.raises(ConnectionError):
    link.connect()
  assert t._trusted['serial'] == 'NAVDY01'


def test_a_failing_bluetooth_fallback_keeps_its_backoff_inside_the_quick_window(monkeypatch, tmp_path):
  """Codex review 2026-11-26 P2: the quarter-second retry is for the cable only."""
  state = run_usb_bridge(monkeypatch, tmp_path, drop_at=103., down_for=6., peer='54:ED:A3:00:48:23', duration=12.)
  assert len([t for t in state.bt_attempts if t < 103. + bridge.USB_QUICK_WINDOW]) <= 2
  assert bridge.USB_QUICK_RETRY not in state.delays[1:]
