"""Weak Wi-Fi guard (user request 2026-11-18): on cellular data, a barely heard saved Wi-Fi stops being retried, and
the guard never leaves Wi-Fi autoconnect off without working cellular data or after it stops."""
import threading
import time

from openpilot.system.hardware import weak_wifi_guard as g

DISC, CONFIG, ACT = int(g.DEV_DISCONNECTED), 50, int(g.DEV_ACTIVATED)


FAILED = int(g.DEV_FAILED)


def run(policy, seconds, cellular, state, signal, t0=0.0, dt=g.PERIOD_S, failed_at_start=0, events=()):
  t, out = t0, None
  for i in range(int(round(seconds / dt))):
    t += dt
    first = tuple(events) + (FAILED,) * failed_at_start if i == 0 else ()
    out = policy.update(t, cellular, g.WifiView(state, signal, first))
  return out, t


# ---- policy ----

def test_weak_saved_network_on_cellular_stops_autoconnect():
  p = g.WeakWifiPolicy()
  (allow, reason), _ = run(p, 4, True, DISC, 22)          # the comma at the edge of the home Wi-Fi (11/18)
  assert not allow and "too weak (22)" in reason


def test_it_comes_back_only_after_a_strong_signal_holds():
  p = g.WeakWifiPolicy()
  _, t = run(p, 4, True, DISC, 22)
  (allow, _), t = run(p, 6, True, DISC, 40, t0=t)          # between the two thresholds: still off
  assert not allow
  (allow, _), t = run(p, 8, True, DISC, 60, t0=t)          # strong, but not for 10 s yet
  assert not allow
  (allow, _), t = run(p, 4, True, DISC, 60, t0=t)
  assert allow
  (allow, _), t = run(p, 4, True, DISC, 38, t0=t)          # dips to 38: above the weak line, keeps trying
  assert allow


def test_out_of_sight_for_a_while_forgets_the_weak_network():
  p = g.WeakWifiPolicy()
  _, t = run(p, 4, True, DISC, 22)
  (allow, _), t = run(p, 20, True, DISC, None, t0=t)       # drove off: briefly out of sight, still off
  assert not allow
  (allow, _), t = run(p, 12, True, DISC, None, t0=t)
  assert allow
  (allow, _), t = run(p, 2, True, DISC, 38, t0=t)          # another saved network at 38 is tried normally
  assert allow


def test_without_cellular_it_never_blocks():
  p = g.WeakWifiPolicy()
  _, t = run(p, 4, True, DISC, 22)
  (allow, reason), _ = run(p, 2, False, DISC, 22, t0=t)
  assert allow and "only way online" in reason


def test_a_connected_wifi_is_left_alone():
  p = g.WeakWifiPolicy()
  (allow, _), t = run(p, 10, True, ACT, 20)               # connected, even weakly: not touched
  assert allow
  _, t = run(p, 4, True, DISC, 22, t0=t)                   # dropped and weak: off
  (allow, reason), t = run(p, 2, True, ACT, 25, t0=t)      # the user connects from the UI: allowed
  assert allow and reason == "connected"
  (allow, _), _ = run(p, 2, True, DISC, 40, t0=t)          # a connection clears the latch: 40 is tried again
  assert allow


def test_no_saved_network_in_sight_changes_nothing():
  p = g.WeakWifiPolicy()
  (allow, _), _ = run(p, 20, True, DISC, None)
  assert allow


def test_failed_attempts_rest_it_then_it_tries_again_at_a_middling_signal():
  p = g.WeakWifiPolicy()
  t = 0.0
  for _ in range(g.FAIL_LIMIT - 1):
    (allow, _), t = run(p, 20, True, DISC, 40, t0=t, failed_at_start=1)
    assert allow
  (allow, reason), t = run(p, 2, True, DISC, 40, t0=t, failed_at_start=1)
  assert not allow and "repeated failed" in reason
  (allow, _), t = run(p, g.FAIL_REST_S - 10, True, DISC, 40, t0=t)
  assert not allow
  (allow, _), t = run(p, 12, True, DISC, 40, t0=t)         # the rest ends: tries again, even at 40 or out of sight
  assert allow


def test_failures_counted_from_events_not_samples():
  p = g.WeakWifiPolicy()
  (allow, _), _ = run(p, 2, True, CONFIG, 40, failed_at_start=g.FAIL_LIMIT)   # three failures between two polls
  assert not allow


def test_old_failures_age_out():
  p = g.WeakWifiPolicy()
  t = 0.0
  for _ in range(g.FAIL_LIMIT):
    (allow, _), t = run(p, g.FAIL_WINDOW_S / 2 + 2, True, DISC, 40, t0=t, failed_at_start=1)
  assert allow


def test_a_short_connection_between_two_polls_clears_failures():
  p = g.WeakWifiPolicy()
  _, t = run(p, 2, True, DISC, 50, failed_at_start=g.FAIL_LIMIT - 1)
  (allow, _), t = run(p, 2, True, DISC, 50, t0=t, events=(FAILED, ACT, DISC))   # failed, then connected and dropped
  assert allow
  (allow, _), t = run(p, 2, True, DISC, 50, t0=t, failed_at_start=g.FAIL_LIMIT - 1)
  assert allow                                             # only what failed after the connection counts


def test_a_failure_after_a_short_connection_still_counts():
  p = g.WeakWifiPolicy()
  _, t = run(p, 2, True, DISC, 50, events=(ACT, DISC) + (FAILED,) * g.FAIL_LIMIT)
  assert not p.allow


def test_a_past_connection_does_not_erase_what_the_air_shows_now():
  p = g.WeakWifiPolicy()
  _, t = run(p, 4, True, DISC, 22)
  (allow, _), _ = run(p, 2, True, DISC, 22, t0=t, events=(ACT, DISC))   # connected briefly, dropped, still at 22
  assert not allow


def test_a_short_connection_clears_the_weak_latch():
  p = g.WeakWifiPolicy()
  _, t = run(p, 4, True, DISC, 22)
  (allow, _), _ = run(p, 2, True, DISC, 40, t0=t, events=(ACT, DISC))
  assert allow


def test_a_success_clears_failures_and_rest():
  p = g.WeakWifiPolicy()
  _, t = run(p, 2, True, DISC, 50, failed_at_start=g.FAIL_LIMIT)
  (allow, _), t = run(p, 2, True, ACT, 50, t0=t)
  (allow, _), t = run(p, 2, True, DISC, 50, t0=t)          # dropped again later: no leftover rest
  assert allow
  (allow, _), t = run(p, 2, True, DISC, 50, t0=t, failed_at_start=g.FAIL_LIMIT - 1)
  assert allow


# ---- cellular ----

HDR = "Iface\tDestination\tGateway\tFlags\tRefCnt\tUse\tMetric\tMask\tMTU\tWindow\tIRTT\n"


def route(dest="00000000", mask="00000000", iface="ppp0", flags="0003"):
  return f"{iface}\t{dest}\t40404040\t{flags}\t0\t0\t1000\t{mask}\t0\t0\t0\n"


def test_cellular_default_iface(tmp_path):
  f = tmp_path / "route"
  f.write_text(HDR + route())
  assert g.cellular_default_iface(str(f)) == "ppp0"
  f.write_text(HDR + route(iface="wlan0") + route(dest="40404040", mask="FFFFFFFF"))
  assert g.cellular_default_iface(str(f)) is None           # ppp0 up but no default route through it
  f.write_text(HDR + route(flags="0002"))
  assert g.cellular_default_iface(str(f)) is None           # route not up
  assert g.cellular_default_iface(str(tmp_path / "missing")) is None


def link(tmp_path, rx="100"):
  (tmp_path / "route").write_text(HDR + route())
  stats = tmp_path / "net" / "ppp0" / "statistics"
  stats.mkdir(parents=True, exist_ok=True)
  if rx is not None:
    (stats / "rx_bytes").write_text(rx)
  return g.CellularLink(str(tmp_path / "route"), str(tmp_path / "net")), stats / "rx_bytes"


def test_cellular_link_quiet_modem_frees_wifi(tmp_path):
  cell, rx = link(tmp_path)
  assert cell.ok(0, False)
  assert cell.ok(g.CELL_QUIET_S - 1, False)
  assert not cell.ok(g.CELL_QUIET_S + 1, False)              # nothing received for 2 min with Wi-Fi off: dead SIM
  rx.write_text("200")
  assert cell.ok(g.CELL_QUIET_S + 3, False)                  # traffic again


def test_cellular_link_quiet_clock_waits_while_wifi_connected(tmp_path):
  cell, _ = link(tmp_path)
  assert cell.ok(0, True)
  assert cell.ok(1000, True)                                  # modem idle behind a connected Wi-Fi: not counted
  assert cell.ok(1000 + g.CELL_QUIET_S - 1, False)            # Wi-Fi dropped: the clock starts now


def test_cellular_link_unreadable_counters_run_the_clock_out(tmp_path):
  cell, _ = link(tmp_path, rx=None)
  assert cell.ok(0, False)                                    # a first sample gets the grace ...
  assert not cell.ok(g.CELL_QUIET_S + 1, False)               # ... but unreadable counters prove nothing after it


def test_cellular_link_new_session_gets_grace(tmp_path):
  cell, rx = link(tmp_path, rx="5000")
  assert cell.ok(0, False)
  assert not cell.ok(g.CELL_QUIET_S + 1, False)
  rx.write_text("10")                                         # counter started over: a new PPP session
  assert cell.ok(g.CELL_QUIET_S + 3, False)


def test_cellular_link_without_route(tmp_path):
  cell, _ = link(tmp_path)
  assert cell.ok(0, False)
  (tmp_path / "route").write_text(HDR)
  assert not cell.ok(2, False)


# ---- guard ----

class FakeNM:
  """Autoconnect lives in `world`, like the real device, so it survives a new connection."""
  def __init__(self, world, views):
    self.world, self.views, self.deadline = world, views, None

  def view(self, now):
    self.world['order'].append('view')
    v = self.views[min(self.world['views'], len(self.views) - 1)]
    self.world['views'] += 1
    if isinstance(v, Exception):
      raise v
    return v

  def autoconnect(self):
    if self.world.get('get_error'):
      raise RuntimeError("Get failed")
    return self.world['auto']

  def set_autoconnect(self, on):
    self.world['order'].append('set')
    if self.world.get('set_error'):
      raise RuntimeError("Set failed")
    self.world['auto'] = on
    self.world['sets'].append(on)
    if self.world.get('lose_reply'):
      raise TimeoutError("reply lost")

  def close(self):
    self.world['closed'] += 1


def make_guard(views, cellular=None, end_event=None, **world):
  w = {'auto': True, 'sets': [], 'views': 0, 'closed': 0, 'connects': 0, 'order': [], 'cell': True, **world}
  clock = iter(float(t) for t in range(0, 10 ** 6, 2))

  def cell(now, connected):
    w['order'].append('cellular')
    return w['cell']

  def factory(cancelled, deadline):
    w['connects'] += 1
    if w.get('no_bus'):
      raise RuntimeError("no system bus")
    return FakeNM(w, views)
  return g.WeakWifiGuard(nm_factory=factory, cellular=cellular or cell, clock=lambda: next(clock),
                         end_event=end_event), w


def test_guard_turns_autoconnect_off_and_back_on_when_it_stops():
  guard, w = make_guard([g.WifiView(DISC, 22)])
  guard.step()
  assert w['sets'] == [False] and guard.may_be_off
  guard.stop()
  assert w['auto'] and w['sets'] == [False, True] and not guard.may_be_off


def test_guard_checks_cellular_before_dbus():
  guard, w = make_guard([g.WifiView(DISC, 22)])
  guard.step()
  assert w['order'][:2] == ['cellular', 'view']


def test_guard_takes_over_an_off_left_by_a_killed_guard():
  guard, w = make_guard([g.WifiView(DISC, 22)], auto=False)
  guard.step()                                               # its own policy also wants it off: nothing to write
  assert w['sets'] == [] and guard.may_be_off
  guard.stop()                                               # but it is still its to turn back on
  assert w['auto']


def test_guard_restores_what_a_killed_guard_left_off():
  guard, w = make_guard([g.WifiView(DISC, 60)], auto=False)
  guard.step()
  assert w['auto']


def test_guard_remembers_off_across_errors_and_reconnects():
  guard, w = make_guard([g.WifiView(DISC, 22), RuntimeError("bus went away")])
  guard.step()
  guard.step()                                               # error: connection dropped, but not the duty
  assert guard.nm is None and guard.may_be_off
  guard.stop()
  assert w['auto'] and w['connects'] == 2


def test_guard_frees_wifi_when_cellular_goes_even_if_access_points_cannot_be_read():
  guard, w = make_guard([g.WifiView(DISC, 22), RuntimeError("AccessPoints failed")])
  guard.step()
  w['cell'] = False
  guard.step()
  assert w['auto'] and not guard.may_be_off


def test_guard_without_cellular_frees_wifi_before_reading_the_air():
  guard, w = make_guard([TimeoutError("40 slow access points")], auto=False, cell=False)
  guard.step()
  assert w['auto'] and w['order'].index('set') < w['order'].index('view')


def test_guard_stale_connected_state_does_not_hold_the_cellular_clock(tmp_path):
  cell, _ = link(tmp_path)                                   # modem route, counter never moves
  guard, w = make_guard([g.WifiView(ACT, None), RuntimeError("NetworkManager unreadable")],
                        cellular=cell.ok, auto=False, set_error=True)
  guard.step()                                               # connected; turning autoconnect on fails once
  w['set_error'] = False
  for _ in range(int(g.CELL_QUIET_S / 2) + 5):               # then NetworkManager stops answering views
    guard.step()
  assert w['auto']                                           # the quiet modem freed Wi-Fi anyway


def test_guard_keeps_the_setting_while_cellular_works_and_nm_is_unreadable():
  guard, w = make_guard([g.WifiView(DISC, 22), RuntimeError("AccessPoints failed")])
  guard.step()
  guard.step()
  assert w['sets'] == [False]


def test_guard_lost_reply_still_counts_as_maybe_off():
  guard, w = make_guard([g.WifiView(DISC, 22)], lose_reply=True)
  guard.step()
  assert not w['auto'] and guard.may_be_off                  # the write landed although the reply never came
  w['lose_reply'] = False
  guard.stop()
  assert w['auto']


def test_guard_does_not_trust_a_failed_read_as_on():
  guard, w = make_guard([g.WifiView(DISC, 22), g.WifiView(DISC, 60)])
  guard.step()
  w['get_error'] = True
  for _ in range(10):
    guard.step()
  assert guard.may_be_off                                    # never marked restored without reading it back on
  w['get_error'] = False
  guard.stop()
  assert w['auto']


def test_guard_restore_gives_up_at_its_budget():
  guard, w = make_guard([g.WifiView(DISC, 22)])
  guard.step()
  w['set_error'] = True
  t0 = time.monotonic()
  guard.stop(0.3)                                            # gives up at the deadline instead of hanging
  assert time.monotonic() - t0 < 1.0
  assert not w['auto'] and w['connects'] > 2


def test_guard_thread_stops_and_restores():
  end = threading.Event()
  guard, w = make_guard([g.WifiView(DISC, 22)], end_event=end)
  orig_step = guard.step

  def step():
    orig_step()
    end.set()
  guard.step = step
  g.guard_thread(end, guard)
  assert w['sets'] == [False, True] and w['auto']


def test_guard_stopped_before_its_first_read_still_checks():
  guard, w = make_guard([RuntimeError("slow")], auto=False)
  guard.stop()
  assert w['auto']


def test_guard_stop_with_autoconnect_on_writes_nothing():
  guard, w = make_guard([RuntimeError("slow")])
  guard.stop()
  assert w['sets'] == []


def test_guard_survives_a_missing_bus():
  guard, w = make_guard([g.WifiView(DISC, 22)], no_bus=True)
  for _ in range(3):
    guard.step()
  assert w['connects'] == 3 and w['sets'] == []
