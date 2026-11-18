"""Weak Wi-Fi guard's D-Bus calls, through the real jeepney codec and blocking transport (no system bus needed)."""
import socket
import threading
import time

import pytest

jeepney = pytest.importorskip("jeepney")
from jeepney import HeaderFields, Message, new_error, new_method_return

from openpilot.system.hardware import weak_wifi_guard as g

DISC = int(g.DEV_DISCONNECTED)
HOME = '집 Wi-Fi'.encode()


class Sock:
  def __init__(self):
    self.timeouts = []

  def settimeout(self, t):
    self.timeouts.append(t)


class Wire:
  def __init__(self, handler, delay=0.0):
    self.handler, self.calls, self.delay, self.sock = handler, [], delay, Sock()

  def send_and_get_reply(self, msg, timeout=None):
    assert 0 < timeout <= g.CALL_TIMEOUT_S
    time.sleep(self.delay)
    request = Message.from_buffer(msg.serialise(serial=1))
    self.calls.append(request)
    return Message.from_buffer(self.handler(request).serialise(serial=2))

  def close(self):
    pass


def wire_nm(handler, boot=1000.0, delay=0.0, cancelled=lambda: False):
  nm = g.NMWifi.__new__(g.NMWifi)
  nm._conn, nm._device, nm._boottime = Wire(handler, delay), '/dev/1', lambda: boot
  nm.cancelled, nm.deadline = cancelled, None
  nm._saved, nm._saved_at, nm._events, nm._filter, nm._ap_ssid = set(), -1e9, None, None, {}
  return nm


def denied(request):
  return new_error(request, 'org.freedesktop.DBus.Error.AccessDenied', 's', ('Not authorized',))


def air(aps, state=DISC, saved=HOME, gone=()):
  """A NetworkManager answering for one saved network and the given access points {path: (ssid, strength, seen)}."""
  settings = {'802-11-wireless': {'mode': ('s', 'infrastructure'), 'ssid': ('ay', saved)}}

  def handler(r):
    member = r.header.fields[HeaderFields.member]
    if member == 'ListConnections':
      return new_method_return(r, 'ao', (['/saved/1'],))
    if member == 'GetSettings':
      return new_method_return(r, 'a{sa{sv}}', (settings,))
    path, prop = r.header.fields[HeaderFields.path], r.body[1]
    if prop == 'State':
      return new_method_return(r, 'v', (('u', state),))
    if prop == 'AccessPoints':
      return new_method_return(r, 'v', (('ao', list(aps)),))
    if path in gone:
      return new_error(r, gone[path], 's', ('gone',))
    ssid, strength, seen = aps[path]
    value = {'Ssid': ('ay', ssid), 'Strength': ('y', strength), 'LastSeen': ('i', seen)}[prop]
    return new_method_return(r, 'v', (value,))
  return handler


def test_error_replies_raise():
  nm = wire_nm(denied)
  with pytest.raises(jeepney.wrappers.DBusErrorResponse):
    nm.autoconnect()
  with pytest.raises(jeepney.wrappers.DBusErrorResponse):
    nm.set_autoconnect(False)


def test_set_autoconnect_message():
  nm = wire_nm(lambda r: new_method_return(r))
  nm.set_autoconnect(False)
  msg = nm._conn.calls[0]
  assert msg.header.fields[HeaderFields.signature] == 'ssv'
  assert msg.body == ('org.freedesktop.NetworkManager.Device', 'Autoconnect', ('b', False))


def test_autoconnect_must_be_a_bool():
  nm = wire_nm(lambda r: new_method_return(r, 'v', (('s', 'yes'),)))
  with pytest.raises(TypeError):
    nm.autoconnect()


def test_view_reads_saved_fresh_access_points_by_raw_ssid():
  aps = {'/ap/1': (HOME, 22, 990), '/ap/2': (HOME, 70, 100),      # 70 is a stale scan result: ignored
         '/ap/3': (b'home\xff', 90, 995), '/ap/4': (HOME, 25, -1)}  # unsaved raw bytes; never seen
  nm = wire_nm(air(aps))
  assert nm.view(0) == g.WifiView(DISC, 22)
  n = len(nm._conn.calls)
  assert nm.view(1) == g.WifiView(DISC, 22)
  assert not [c for c in nm._conn.calls[n:] if c.body[1:] == ('Ssid',)]   # SSIDs are read once per access point


def test_view_skips_the_air_while_connected():
  nm = wire_nm(lambda r: new_method_return(r, 'v', (('u', 100),)))
  assert nm.view(0) == g.WifiView(100, None)
  assert len(nm._conn.calls) == 1


def test_view_skips_access_points_that_went_away_but_not_refusals():
  aps = {'/ap/1': (HOME, 22, 990), '/ap/2': (HOME, 70, 990)}
  nm = wire_nm(air(aps, gone={'/ap/2': 'org.freedesktop.DBus.Error.UnknownObject'}))
  assert nm.view(0) == g.WifiView(DISC, 22)
  nm = wire_nm(air(aps, gone={'/ap/2': 'org.freedesktop.DBus.Error.AccessDenied'}))
  with pytest.raises(jeepney.wrappers.DBusErrorResponse):
    nm.view(0)                                              # a refusal is not "out of sight"


def test_calls_stop_at_the_deadline():
  aps = {f'/ap/{i}': (HOME, 22, 990) for i in range(20)}
  nm = wire_nm(air(aps), delay=0.05)
  nm.deadline = time.monotonic() + 0.3
  with pytest.raises(TimeoutError):
    nm.view(0)


def test_stop_request_cuts_a_long_view_short_and_the_restore_still_runs():
  """hardwared's SIGINT path: a step reading 40 slow access points must not eat the restore's time, and an off left
  by a killed guard is restored although the first step never got to read it."""
  end = threading.Event()
  world = {'auto': False}

  def handler(r):
    member = r.header.fields[HeaderFields.member]
    if member == 'Set':
      world['auto'] = r.body[2][1]
      return new_method_return(r)
    if r.body[1:] == ('Autoconnect',):
      return new_method_return(r, 'v', (('b', world['auto']),))
    return air({f'/ap/{i}': (HOME, 22, 990) for i in range(40)})(r)

  def factory(cancelled, deadline):
    nm = wire_nm(handler, delay=0.1, cancelled=cancelled)
    nm.deadline = deadline
    return nm
  guard = g.WeakWifiGuard(nm_factory=factory, cellular=lambda now, connected: True, end_event=end)
  worker = threading.Thread(target=g.guard_thread, args=(end, guard), daemon=True)
  worker.start()
  time.sleep(0.5)                                            # mid-view (40 APs x 0.1 s)
  t0 = time.monotonic()
  end.set()
  worker.join(g.STOP_JOIN_S)
  assert not worker.is_alive() and time.monotonic() - t0 < 1.5
  assert world['auto'] is True


def test_calls_bound_the_send_too():
  nm = wire_nm(lambda r: new_method_return(r, 'v', (('b', True),)))
  nm.deadline = time.monotonic() + 0.4
  nm.autoconnect()
  assert 0 < nm._conn.sock.timeouts[-1] <= 0.4


def slow_auth(monkeypatch, a, auth_s):
  def prep_socket(addr, enable_fds=False, timeout=2.0):
    assert timeout <= g.CALL_TIMEOUT_S
    time.sleep(auth_s)
    return a
  monkeypatch.setattr('jeepney.io.blocking.prep_socket', prep_socket)
  monkeypatch.setattr('jeepney.bus.get_bus', lambda addr: 'unused')


def test_open_system_bus_bounds_auth_and_hello_together(monkeypatch):
  a, b = socket.socketpair()
  slow_auth(monkeypatch, a, 0.15)
  t0 = time.monotonic()
  with pytest.raises(TimeoutError):
    g.open_system_bus(time.monotonic() + 0.25)               # auth takes 0.15 s, the bus never answers Hello
  assert time.monotonic() - t0 < 0.4
  b.close()


def test_open_system_bus_gives_up_after_auth_when_cancelled(monkeypatch):
  a, b = socket.socketpair()
  slow_auth(monkeypatch, a, 0.05)
  calls = iter([False, True])
  with pytest.raises(g.Cancelled):
    g.open_system_bus(time.monotonic() + 1.0, cancelled=lambda: next(calls))
  b.close()


def test_open_system_bus_says_hello(monkeypatch):
  a, b = socket.socketpair()
  slow_auth(monkeypatch, a, 0.0)

  def bus():
    hello = Message.from_buffer(b.recv(65536))
    b.sendall(new_method_return(hello, 's', (':1.42',)).serialise(serial=1))
  t = threading.Thread(target=bus)
  t.start()
  conn = g.open_system_bus(time.monotonic() + 1.0)
  t.join(1)
  assert conn.unique_name == ':1.42'
  conn.close()
  b.close()
