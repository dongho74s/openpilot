"""Weak Wi-Fi guard: stop NetworkManager from retrying a saved Wi-Fi network it can barely hear while the SIM's cellular
data already works.

Why (2026-11-18): wifi_manager saves networks with autoconnect-retries 0, which NetworkManager reads as "retry
forever", so the comma keeps reconnecting at home. Parked at the edge of the home Wi-Fi (signal about 22) it
associated, failed and fell back to LTE about once a minute; every attempt moved the default route, Tailscale rebound
("LinkChange: major, rebinding") and SSH and Carrot Web dropped, although LTE itself was fine (4G, no ping loss).

What: while cellular data works and no Wi-Fi is connected, a saved network whose best access point is weaker than
WEAK_BELOW turns the Wi-Fi device's autoconnect off (Device.Autoconnect, not the saved connection: the networks and
passwords are untouched and NetworkManager resets it at boot). Scans go on (the UI requests them every 5 s), and once
a saved network is at STRONG_AT or more for STRONG_HOLD_S, or no saved network has been in sight for ABSENT_CLEAR_S,
autoconnect is turned back on. FAIL_LIMIT failed attempts within FAIL_WINDOW_S also rest it for FAIL_REST_S, then it
tries again; a connection clears all of that.

Safety, as the comma is often reached only over that cellular link:
- Wi-Fi is never kept off without working cellular data: a default route over the modem that has received something
  within CELL_QUIET_S while Wi-Fi was off (Tailscale and athena keep traffic going). Cellular is checked first in every
  step, from /proc and /sys, and without it autoconnect is turned back on before anything else is read, on its own
  budget. Only a device state read in the same step counts as "Wi-Fi connected" for the quiet clock.
- Every D-Bus exchange is bounded, opening the connection (connect, auth, Hello together) included, sends too, and
  one step has STEP_BUDGET_S for reading and deciding.
- Any autoconnect=off the guard sees (its own, or one a killed guard left) is its to turn back on: when the policy
  allows, and when the guard stops (hardwared exits; a stop request cuts the step short and the restore gets
  STOP_BUDGET_S). A killed hardwared leaves it to the next start; a reboot resets it.
- A connection the user starts from the UI still works and a connected Wi-Fi is never touched.
"""
from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass

from openpilot.common.swaglog import cloudlog
from openpilot.system.ui.lib.networkmanager import (NM, NM_PATH, NM_IFACE, NM_DEVICE_IFACE, NM_DEVICE_TYPE_WIFI,
                                                    NM_SETTINGS_PATH, NM_SETTINGS_IFACE, NM_CONNECTION_IFACE,
                                                    NM_WIRELESS_IFACE, NM_ACCESS_POINT_IFACE, NMDeviceState)

WEAK_BELOW = 30          # NetworkManager signal (0-100): below this a saved network is not worth trying on cellular
STRONG_AT = 45           # and from this, held STRONG_HOLD_S, it is again
STRONG_HOLD_S = 10.0
ABSENT_CLEAR_S = 30.0    # no saved network in sight this long: forget the weak one
AP_FRESH_S = 60.0        # an access point missing from scans this long is ignored (weak ones skip some scans)
FAIL_LIMIT = 3           # failed attempts ...
FAIL_WINDOW_S = 300.0    # ... within this long ...
FAIL_REST_S = 300.0      # ... rest autoconnect this long (cellular only), then try again
CELL_QUIET_S = 120.0     # cellular route with nothing received this long while Wi-Fi is off: treat as not working
CELL_IFACES = ('ppp', 'rmnet', 'wwan')
PERIOD_S = 2.0
CALL_TIMEOUT_S = 1.0     # any one D-Bus exchange (connect + auth, Hello, a call)
STEP_BUDGET_S = 4.0      # one whole step
STOP_BUDGET_S = 2.5      # turning autoconnect back on when stopping
STOP_JOIN_S = CALL_TIMEOUT_S + STOP_BUDGET_S + 0.5   # hardwared waits this long (manager allows 5 s before SIGKILL)
SAVED_REFRESH_S = 60.0
ERROR_LOG_S = 60.0
GONE_ERRORS = ('org.freedesktop.DBus.Error.UnknownObject', 'org.freedesktop.DBus.Error.UnknownMethod')

DEV_DISCONNECTED, DEV_ACTIVATED, DEV_FAILED = NMDeviceState.DISCONNECTED, NMDeviceState.ACTIVATED, NMDeviceState.FAILED


class Cancelled(Exception):
  pass


@dataclass(frozen=True)
class WifiView:
  state: int                    # the Wi-Fi device's NMDeviceState
  saved_signal: int | None      # strongest freshly seen access point of a saved network (None: none in sight)
  events: tuple[int, ...] = ()  # new states from StateChanged signals since the last view, in order


class WeakWifiPolicy:
  """Decides whether the Wi-Fi device may autoconnect. Pure: update() takes the time, whether cellular data works and
  what NetworkManager shows, and returns (allow, reason)."""

  def __init__(self):
    self.allow, self.reason = True, "start"
    self.weak = False                       # a saved network heard only weakly (hysteresis latch)
    self._strong_since: float | None = None
    self._absent_since: float | None = None
    self._fails: list[float] = []
    self._rest_until: float | None = None

  def _track_signal(self, now: float, signal: int | None) -> None:
    if signal is None:
      self._strong_since = None
      if self._absent_since is None:
        self._absent_since = now
      if now - self._absent_since >= ABSENT_CLEAR_S:
        self.weak = False
      return
    self._absent_since = None
    if signal < WEAK_BELOW:
      self.weak, self._strong_since = True, None
    elif signal >= STRONG_AT:
      if self._strong_since is None:
        self._strong_since = now
      if now - self._strong_since >= STRONG_HOLD_S:
        self.weak = False
    else:
      self._strong_since = None

  def _connected(self) -> None:
    self._fails.clear()                     # it connected: earlier failures, their rest and the latch no longer count
    self._rest_until = None
    self.weak = False

  def _track_events(self, now: float, events: tuple[int, ...]) -> None:
    self._fails = [t for t in self._fails if now - t < FAIL_WINDOW_S]
    for state in events:                    # in order: a failure after a connection still counts
      if state == DEV_ACTIVATED:
        self._connected()
      elif state == DEV_FAILED:
        self._fails.append(now)
        if len(self._fails) >= FAIL_LIMIT:
          self._fails.clear()
          self._rest_until = now + FAIL_REST_S

  def update(self, now: float, cellular: bool, view: WifiView) -> tuple[bool, str]:
    self._track_events(now, view.events)    # what happened since the last view, then what the air shows now
    self._track_signal(now, view.saved_signal)
    if view.state == DEV_ACTIVATED:
      self._connected()
    if self._rest_until is not None and now >= self._rest_until:
      self._rest_until = None
    if not cellular:
      return self._decide(True, "no working cellular data: Wi-Fi is the only way online")
    if view.state == DEV_ACTIVATED:
      return self._decide(True, "connected")
    if self._rest_until is not None:
      return self._decide(False, "resting after repeated failed attempts")
    if self.weak:
      return self._decide(False, f"saved network too weak ({view.saved_signal}) on cellular")
    return self._decide(True, "saved network usable" if view.saved_signal is not None else "no saved network in sight")

  def _decide(self, allow: bool, reason: str) -> tuple[bool, str]:
    self.allow, self.reason = allow, reason
    return allow, reason


def cellular_default_iface(route_file: str = "/proc/net/route") -> str | None:
  """The modem interface a default route goes out over, if any (it stays while Wi-Fi is up, with a higher metric)."""
  try:
    with open(route_file) as f:
      for line in f.readlines()[1:]:
        cols = line.split()
        if (len(cols) > 7 and cols[1] == "00000000" and cols[7] == "00000000" and int(cols[3], 16) & 0x1 and
            cols[0].startswith(CELL_IFACES)):
          return cols[0]
  except (OSError, ValueError):
    pass
  return None


class CellularLink:
  """Whether cellular data works: a default route over the modem that has received something within CELL_QUIET_S of
  Wi-Fi being off. While Wi-Fi is connected the modem is idle by design, so the quiet clock only runs without it. A new
  modem session (another interface, or the counter starting over) gets the same CELL_QUIET_S of grace as the first
  sample; unreadable counters prove nothing and let the clock run out."""

  def __init__(self, route_file: str = "/proc/net/route", sys_net: str = "/sys/class/net"):
    self._route_file, self._sys_net = route_file, sys_net
    self._iface: str | None = None
    self._rx: int | None = None
    self._heard_at = 0.0

  def _rx_bytes(self, iface: str) -> int | None:
    try:
      with open(f"{self._sys_net}/{iface}/statistics/rx_bytes") as f:
        return int(f.read())
    except (OSError, ValueError):
      return None

  def ok(self, now: float, wifi_connected: bool) -> bool:
    iface = cellular_default_iface(self._route_file)
    if iface is None:
      self._iface = None
      return False
    rx = self._rx_bytes(iface)
    new_session = iface != self._iface or (rx is not None and self._rx is not None and rx < self._rx)
    if new_session or wifi_connected or (rx is not None and self._rx is not None and rx > self._rx):
      self._heard_at = now
    self._iface = iface
    if rx is not None:
      self._rx = rx
    return now - self._heard_at < CELL_QUIET_S


def open_system_bus(end_by: float, cancelled=lambda: False):
  """jeepney's open_dbus_connection(bus="SYSTEM"), but all of it (connect, auth, Hello: the stock Hello waits forever)
  done by end_by (time.monotonic), and given up between steps once cancelled() says so."""
  from jeepney.bus import get_bus
  from jeepney.bus_messages import message_bus
  from jeepney.io.blocking import DBusConnection, DBusConnectionBase, prep_socket
  from jeepney.io.common import MessageFilters
  from jeepney.wrappers import unwrap_msg

  def left() -> float:
    if cancelled():
      raise Cancelled
    t = end_by - time.monotonic()
    if t <= 0:
      raise TimeoutError("weak_wifi_guard: connecting to the system bus took too long")
    return t
  sock = prep_socket(get_bus("SYSTEM"), enable_fds=False, timeout=left())
  try:
    t = left()
    sock.settimeout(t)                      # bounds the send; the reply wait goes through select with a deadline
    conn = DBusConnection.__new__(DBusConnection)
    DBusConnectionBase.__init__(conn, sock, enable_fds=False)
    conn._filters = MessageFilters()
    conn.unique_name = unwrap_msg(conn.send_and_get_reply(message_bus.Hello(), timeout=t))[0]
  except BaseException:
    sock.close()
    raise
  return conn


class NMWifi:
  """The bit of NetworkManager the guard needs, over D-Bus (jeepney, as wifi_manager). Every call is cut at the
  `deadline` the guard sets (time.monotonic) and at CALL_TIMEOUT_S, raises Cancelled once `cancelled()` says so, and
  error replies raise."""

  def __init__(self, cancelled=lambda: False, deadline: float | None = None, boottime=None):
    self.cancelled, self.deadline = cancelled, deadline
    self._boottime = boottime or (lambda: time.clock_gettime(time.CLOCK_BOOTTIME))
    self._conn = open_system_bus(time.monotonic() + self._timeout(), cancelled)
    self._device: str | None = None
    self._saved: set[bytes] = set()
    self._saved_at = -1e9
    self._ap_ssid: dict[str, bytes] = {}
    self._events: deque | None = None
    self._filter = None

  def close(self) -> None:
    try:
      if self._filter is not None:
        self._filter.close()
      self._conn.close()
    except Exception:
      pass

  def _timeout(self) -> float:
    if self.cancelled():
      raise Cancelled
    timeout = CALL_TIMEOUT_S
    if self.deadline is not None:
      timeout = min(timeout, self.deadline - time.monotonic())
      if timeout <= 0:
        raise TimeoutError("weak_wifi_guard: time budget spent")
    return timeout

  def _call(self, msg):
    from jeepney.wrappers import unwrap_msg
    timeout = self._timeout()
    self._conn.sock.settimeout(timeout)     # the send too, not only the reply wait
    return unwrap_msg(self._conn.send_and_get_reply(msg, timeout=timeout))

  def _get(self, path: str, iface: str, prop: str):
    from jeepney import DBusAddress
    from jeepney.wrappers import Properties
    return self._call(Properties(DBusAddress(path, bus_name=NM, interface=iface)).get(prop))[0][1]

  def _method(self, path: str, iface: str, member: str):
    from jeepney import DBusAddress, new_method_call
    return self._call(new_method_call(DBusAddress(path, bus_name=NM, interface=iface), member))[0]

  @staticmethod
  def _gone(e: Exception) -> bool:
    """An object that went away mid-read (an access point out of range, a removed connection), not a failure."""
    return getattr(e, 'name', None) in GONE_ERRORS

  def device(self) -> str:
    if self._device is None:
      dev = None
      for p in self._method(NM_PATH, NM_IFACE, 'GetDevices'):
        if self._get(p, NM_DEVICE_IFACE, 'DeviceType') == NM_DEVICE_TYPE_WIFI:
          dev = str(p)
          break
      if dev is None:
        raise RuntimeError("no Wi-Fi device")
      self._watch_state(dev)
      self._device = dev
    return self._device

  def _watch_state(self, dev: str) -> None:
    from jeepney.bus_messages import MatchRule, message_bus
    rule = MatchRule(type="signal", interface=NM_DEVICE_IFACE, member="StateChanged", path=dev)
    self._call(message_bus.AddMatch(rule))
    self._events = deque(maxlen=64)
    self._filter = self._conn.filter(rule, queue=self._events)

  def _take_events(self) -> tuple[int, ...]:
    events = []
    while self._events:
      events.append(int(self._events.popleft().body[0]))
    return tuple(events)

  def saved_ssids(self, now: float) -> set[bytes]:
    if now - self._saved_at >= SAVED_REFRESH_S:
      saved = set()
      for c in self._method(NM_SETTINGS_PATH, NM_SETTINGS_IFACE, 'ListConnections'):
        try:
          settings = self._method(c, NM_CONNECTION_IFACE, 'GetSettings')
        except Exception as e:
          if self._gone(e):
            continue
          raise
        wifi = dict(settings.get('802-11-wireless', {}))
        if wifi and wifi.get('mode', ('s', 'infrastructure'))[1] != 'ap' and 'ssid' in wifi:
          ssid = bytes(wifi['ssid'][1])
          if ssid:
            saved.add(ssid)
      self._saved, self._saved_at = saved, now
    return self._saved

  def view(self, now: float) -> WifiView:
    dev = self.device()
    state = int(self._get(dev, NM_DEVICE_IFACE, 'State'))
    if state == DEV_ACTIVATED:
      return WifiView(state, None, self._take_events())   # connected: left alone, no need to read the air
    saved = self.saved_ssids(now)
    best = None
    boot = self._boottime()
    aps = [str(ap) for ap in self._get(dev, NM_WIRELESS_IFACE, 'AccessPoints')]
    self._ap_ssid = {ap: self._ap_ssid[ap] for ap in aps if ap in self._ap_ssid}
    for ap in aps:
      try:
        if ap not in self._ap_ssid:         # one object per BSSID: read its SSID once (80+ in a city)
          self._ap_ssid[ap] = bytes(self._get(ap, NM_ACCESS_POINT_IFACE, 'Ssid'))
        if self._ap_ssid[ap] not in saved:
          continue
        seen = int(self._get(ap, NM_ACCESS_POINT_IFACE, 'LastSeen'))
        if seen < 0 or boot - seen > AP_FRESH_S:
          continue                          # not in a recent scan: its strength is stale
        strength = int(self._get(ap, NM_ACCESS_POINT_IFACE, 'Strength'))
        best = strength if best is None else max(best, strength)
      except Exception as e:
        if self._gone(e):
          continue
        raise                               # a timeout or a refusal is not "out of sight"
    return WifiView(state, best, self._take_events())

  def autoconnect(self) -> bool:
    value = self._get(self.device(), NM_DEVICE_IFACE, 'Autoconnect')
    if not isinstance(value, bool):
      raise TypeError(f"Autoconnect is {value!r}")
    return value

  def set_autoconnect(self, on: bool) -> None:
    from jeepney import DBusAddress
    from jeepney.wrappers import Properties
    self._call(Properties(DBusAddress(self.device(), bus_name=NM, interface=NM_DEVICE_IFACE)).set('Autoconnect', 'b', on))


class WeakWifiGuard:
  """One step every PERIOD_S. Keeps, apart from any D-Bus connection, whether autoconnect may be off, so an error, a
  lost reply, a reconnect or a restart never forgets to turn it back on."""

  def __init__(self, nm_factory=NMWifi, cellular=None, clock=time.monotonic, end_event: threading.Event | None = None):
    self._nm_factory, self._clock = nm_factory, clock
    self._cellular = cellular or CellularLink().ok
    self._end_event = end_event
    self._restoring = False
    self.policy = WeakWifiPolicy()
    self.nm = None
    self.may_be_off = True                  # until autoconnect reads back on: unknown at start (a killed guard may
                                            # have left it off), seen off, or we wrote off
    self._wifi_connected = False
    self._error_logged = -1e9

  def _cancelled(self) -> bool:
    return self._end_event is not None and self._end_event.is_set() and not self._restoring

  def _drop(self, what: str) -> None:
    now = time.monotonic()
    if now - self._error_logged >= ERROR_LOG_S:
      cloudlog.exception(f"weak_wifi_guard: {what}")
      self._error_logged = now
    if self.nm is not None:
      self.nm.close()
    self.nm = None

  def _nm(self, deadline: float):
    if self.nm is None:
      self.nm = self._nm_factory(cancelled=self._cancelled, deadline=deadline)
    self.nm.deadline = deadline
    return self.nm

  def _apply(self, allow: bool, reason: str, view: WifiView | None, deadline: float) -> None:
    nm = self._nm(deadline)
    current = nm.autoconnect()
    if not current:
      self.may_be_off = True                # ours to turn back on, also when a killed guard left it off
    if current != allow:
      if not allow:
        self.may_be_off = True              # before the write: a lost reply may still have applied it
      nm.set_autoconnect(allow)
      current = nm.autoconnect()
      if current != allow:
        raise RuntimeError(f"Autoconnect still {current} after setting {allow}")
      cloudlog.event("weak_wifi_guard", autoconnect=allow, reason=reason,
                     signal=view.saved_signal if view else None, state=view.state if view else None)
    if current:
      self.may_be_off = False

  def _try_apply(self, allow: bool, reason: str, view: WifiView | None, deadline: float) -> None:
    try:
      self._apply(allow, reason, view, deadline)
    except Cancelled:
      pass
    except Exception:
      self._drop("setting autoconnect")

  def step(self) -> None:
    now = self._clock()
    cellular = self._cellular(now, self._wifi_connected)   # first, and never waits on D-Bus
    self._wifi_connected = False            # until this step reads the device again: a stale "connected" must not
                                            # hold the cellular quiet clock
    if not cellular and self.may_be_off:
      # free Wi-Fi before reading anything else, on its own budget, so reading the air never uses up its time
      self._try_apply(True, "no working cellular data", None, time.monotonic() + STOP_BUDGET_S)
    deadline = time.monotonic() + STEP_BUDGET_S
    try:
      view = self._nm(deadline).view(now)
    except Cancelled:
      return
    except Exception:
      self._drop("reading NetworkManager")
      return                                # with cellular: keep what is set; without: the restore above ran
    self._wifi_connected = view.state == DEV_ACTIVATED
    allow, reason = self.policy.update(now, cellular, view)
    self._try_apply(allow, reason, view, deadline)

  def stop(self, budget_s: float = STOP_BUDGET_S) -> None:
    """Turn autoconnect back on if it may be off, within budget_s."""
    self._restoring = True
    deadline = time.monotonic() + budget_s
    while self.may_be_off and time.monotonic() < deadline:
      try:
        self._apply(True, "guard stopped", None, deadline)
      except Exception:
        self._drop("restoring autoconnect")
        time.sleep(0.05)
    if self.nm is not None:
      self.nm.close()
      self.nm = None


def guard_thread(end_event: threading.Event, guard: WeakWifiGuard | None = None) -> None:
  """hardwared runs this, and on the way out joins it for up to STOP_JOIN_S: setting end_event cuts a step short
  (within one call) and the restore then has STOP_BUDGET_S."""
  guard = guard or WeakWifiGuard(end_event=end_event)
  try:
    while not end_event.is_set():
      guard.step()
      end_event.wait(PERIOD_S)
  finally:
    guard.stop()
