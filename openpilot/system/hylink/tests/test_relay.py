import socket

import pytest
from websocket import ABNF, WebSocketTimeoutException

from openpilot.system.hylink.relay import Relay, TARGET
from openpilot.system.hylink.live import BoundedArchiveExecutor, ClipFrameStore, FRAME_TYPE_WIDE


class WebSocket:
  def __init__(self, frames):
    self.frames = iter(frames)
  def settimeout(self, timeout):
    assert timeout <= 0.5
  def recv_data(self, **kwargs):
    return next(self.frames)


def test_relay_only_targets_loopback_video():
  assert TARGET == ("127.0.0.1", 8765)


def test_stale_forward_thread_cannot_close_new_viewer():
  relay = Relay(lambda: True)
  old, old_peer = socket.socketpair()
  new, new_peer = socket.socketpair()
  try:
    relay.local = new
    relay.close_local(old)
    assert relay.local is new
    relay.close_local(new)
    assert relay.local is None
  finally:
    old.close()
    old_peer.close()
    new.close()
    new_peer.close()


def test_onroad_prevents_local_connection(monkeypatch):
  relay = Relay(lambda: False)
  def forbidden(*a, **k):
    pytest.fail("Must not connect onroad")
  monkeypatch.setattr(socket, "create_connection", forbidden)
  relay.open_local(None)
  assert relay.local is None


def test_no_arbitrary_ssh_or_vehicle_command(monkeypatch):
  relay = Relay(lambda: True)
  monkeypatch.setattr(relay, "open_local", lambda ws: pytest.fail("Unknown command must not connect"))
  relay.connected(WebSocket([(ABNF.OPCODE_TEXT, b"wayon-ssh-authorize-v1.untrusted"),
                             (ABNF.OPCODE_TEXT, b"remote-control"), (ABNF.OPCODE_CLOSE, b"")]))


def test_peer_close_and_ignition_closes_local():
  allowed = [True]
  relay = Relay(lambda: allowed[0])
  local, peer = socket.socketpair()
  relay.local = local
  class WS(WebSocket):
    def recv_data(self, **kwargs):
      allowed[0] = False
      raise WebSocketTimeoutException()
  try:
    relay.connected(WS([]))
    assert relay.local is None
    assert peer.recv(1) == b""
  finally:
    peer.close()


def test_clip_buffer_memory_is_bounded():
  store = ClipFrameStore()
  for i in range(40):
    store.append(FRAME_TYPE_WIDE, (i, b"x" * (1024 * 1024), i % 10 == 0, i))
  assert sum(len(frame[1]) for frame in store.buffers[FRAME_TYPE_WIDE]) <= 16 * 1024 * 1024


def test_archive_queue_bounded_across_multiple_sessions():
  import threading
  executor = BoundedArchiveExecutor()
  done = threading.Event()
  try:
    futures = [executor.submit(done.wait, 1) for _ in range(3)]
    with pytest.raises(RuntimeError, match="archive_queue_full"):
      executor.submit(lambda: None)
    done.set()
    for future in futures:
      future.result(timeout=2)
    assert executor.submit(lambda: True).result(timeout=2)
  finally:
    done.set()
    executor.pool.shutdown(wait=True, cancel_futures=True)
