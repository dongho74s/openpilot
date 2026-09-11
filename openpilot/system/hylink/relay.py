"""Authenticated cloud relay for parking video only. No SSH or actuation endpoint."""
import socket
import threading
import time

from websocket import ABNF, WebSocketException, WebSocketTimeoutException, create_connection

from openpilot.common.params import Params
from openpilot.system.hylink.policy import UploadBackoff
from openpilot.system.hylink.runtime import ENDPOINT, media_ready, read_config

TARGET = ("127.0.0.1", 8765)


class Relay:
  def __init__(self, allowed, target=TARGET):
    self.allowed = allowed
    self.target = target
    self.local = None
    self.lock = threading.Lock()

  def close_local(self, expected=None):
    with self.lock:
      if expected is not None and self.local is not expected:
        return
      local, self.local = self.local, None
    if local:
      try:
        local.shutdown(socket.SHUT_RDWR)
      except OSError:
        pass
      local.close()

  def open_local(self, ws):
    self.close_local()
    if not self.allowed():
      return
    local = socket.create_connection(self.target, timeout=1)
    local.settimeout(0.5)
    with self.lock:
      self.local = local

    def forward():
      try:
        while self.allowed():
          try:
            data = local.recv(65536)
          except TimeoutError:
            continue
          if not data:
            break
          ws.send(data, opcode=ABNF.OPCODE_BINARY)
      except (OSError, WebSocketException):
        pass
      finally:
        with self.lock:
          unexpected_end = self.local is local
        self.close_local(local)
        if unexpected_end:
          # Propagate a camera/SSH EOF to the phone. Otherwise a five-minute
          # session can leave its WebSocket open forever with no further bytes.
          ws.close(timeout=0.5)
    threading.Thread(target=forward, name="hylink-live-forward", daemon=True).start()

  def tick(self):
    pass

  def command(self, command):
    pass

  def connected(self, ws):
    ws.settimeout(0.5)
    next_ping = time.monotonic() + 20
    try:
      while self.allowed():
        self.tick()
        try:
          opcode, data = ws.recv_data(control_frame=True)
        except WebSocketTimeoutException:
          if time.monotonic() >= next_ping:
            ws.ping(b"wayon-live")
            next_ping = time.monotonic() + 20
          continue
        if opcode == ABNF.OPCODE_TEXT:
          command = data.decode() if isinstance(data, bytes) else data
          if command == "wayon-peer-open":
            self.open_local(ws)
          elif command == "wayon-peer-close":
            self.close_local()
          else:
            self.command(command)
        elif opcode == ABNF.OPCODE_BINARY:
          if len(data) > 8 * 1024 * 1024 + 9:
            raise ValueError("Oversized live control frame")
          with self.lock:
            local = self.local
          if local and self.allowed():
            local.sendall(data)
        elif opcode == ABNF.OPCODE_CLOSE:
          return
    finally:
      self.close_local()


def run_relay(relay, params, kind="live"):
  allowed = relay.allowed
  backoff = UploadBackoff()
  while allowed():
    ws = None
    connected_at = time.monotonic()
    try:
      config = read_config(params)
      ws = create_connection(ENDPOINT.replace("https://", "wss://") + "/api/device/relay/" + kind,
                             header=["Authorization: Bearer " + config["token"]],
                             timeout=5, enable_multithread=True, redirect_limit=0)
      relay.connected(ws)
    except (OSError, WebSocketException, ValueError) as exc:
      print(f"Hylink relay: {type(exc).__name__}; retry delayed", flush=True)
    finally:
      relay.close_local()
      if ws:
        ws.close(timeout=0.5)
    if time.monotonic() - connected_at > 60:
      backoff.success()
    retry_at = time.monotonic() + backoff.failure_delay()
    while allowed() and time.monotonic() < retry_at:
      time.sleep(0.5)


def main():
  params = Params()
  run_relay(Relay(lambda: media_ready(False, params)), params)


if __name__ == "__main__":
  main()
