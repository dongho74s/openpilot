#!/usr/bin/env python3
import json
import math
import select
import socket
import struct
import threading
import time

from openpilot.system.hylink.camera import CameraRequest
from openpilot.system.hylink.runtime import media_ready
from openpilot.common.params import Params

LISTEN_HOST = "127.0.0.1"
LISTEN_PORT = 8765
LIVE_BITRATE = 600_000
DEFAULT_MAX_SESSION_S = 300.0
DEFAULT_CAMERA_WAIT_S = 30.0
DEFAULT_PROCESS_WAIT_S = 10.0
MIN_SESSION_S = 30.0
MAX_SESSION_S = 900.0
CLIENT_HEARTBEAT_TIMEOUT_S = 12.0

CLIENT_HEARTBEAT_MAGIC = b"WLP1"
FRAME_MAGIC = b"WLV1"
FRAME_HEADER = struct.Struct(">4sBBHIQI")
FRAME_TYPE_METADATA = 0
FRAME_TYPE_WIDE = 1
FRAME_TYPE_DRIVER = 2
FRAME_TYPE_STATUS = 3
FRAME_FLAG_KEY = 1

STREAM_SERVICES = {
  "livestreamWideRoadEncodeData": FRAME_TYPE_WIDE,
  "livestreamDriverEncodeData": FRAME_TYPE_DRIVER,
}


class ClientControlState:
  """Only viewer heartbeats are accepted; live capture/recording is removed."""
  def __init__(self):
    self.heartbeat_seen = False
    self.last_heartbeat = time.monotonic()
    self.buffer = bytearray()

  def feed(self, data: bytes) -> None:
    self.buffer.extend(data)
    while len(self.buffer) >= len(CLIENT_HEARTBEAT_MAGIC):
      if not self.buffer.startswith(CLIENT_HEARTBEAT_MAGIC):
        raise ValueError("Unsupported live control")
      del self.buffer[:len(CLIENT_HEARTBEAT_MAGIC)]
      self.heartbeat_seen = True
      self.last_heartbeat = time.monotonic()
    if not CLIENT_HEARTBEAT_MAGIC.startswith(self.buffer):
      raise ValueError("Unsupported live control")


def read_client_control(client: socket.socket, state: ClientControlState, timeout_s: float = 0.0) -> bool:
  try:
    readable, _, _ = select.select([client], [], [], max(0.0, timeout_s))
    while readable:
      data = client.recv(4096)
      if not data:
        return False
      state.feed(data)
      readable, _, _ = select.select([client], [], [], 0.0)
    return True
  except (ConnectionError, OSError, ValueError):
    return False


class ClientHeartbeatMonitor:
  def __init__(self, client: socket.socket, state: ClientControlState,
               timeout_s: float = CLIENT_HEARTBEAT_TIMEOUT_S, poll_s: float = 0.25):
    self.client = client
    self.state = state
    self.timeout_s = timeout_s
    self.poll_s = poll_s
    self.client_alive = True
    self.timed_out = False
    self.stop_event = threading.Event()
    self.thread = threading.Thread(target=self._run, name="wayon-live-heartbeat", daemon=True)

  def start(self) -> None:
    self.thread.start()

  def stop(self) -> None:
    self.stop_event.set()
    if self.thread.is_alive():
      self.thread.join(timeout=max(1.0, self.poll_s * 2.0))

  def _close_client(self) -> None:
    try:
      self.client.shutdown(socket.SHUT_RDWR)
    except OSError:
      pass

  def _run(self) -> None:
    while not self.stop_event.is_set():
      if not read_client_control(self.client, self.state, self.poll_s):
        self.client_alive = False
        self._close_client()
        return

      if time.monotonic() - self.state.last_heartbeat > self.timeout_s:
        self.client_alive = False
        self.timed_out = True
        print("Wayon live: viewer heartbeat timed out", flush=True)
        self._close_client()
        return


def bounded_number(value, fallback: float, minimum: float, maximum: float) -> float:
  try:
    number = float(value)
    return min(maximum, max(minimum, number)) if math.isfinite(number) else fallback
  except (TypeError, ValueError):
    return fallback


def pack_frame(frame_type: int, payload: bytes, sequence: int = 0,
               timestamp_us: int = 0, key_frame: bool = False) -> bytes:
  flags = FRAME_FLAG_KEY if key_frame else 0
  header = FRAME_HEADER.pack(
    FRAME_MAGIC,
    frame_type,
    flags,
    0,
    sequence & 0xFFFFFFFF,
    timestamp_us & 0xFFFFFFFFFFFFFFFF,
    len(payload),
  )
  return header + payload


def json_frame(frame_type: int, data: dict) -> bytes:
  return pack_frame(frame_type, json.dumps(data, separators=(",", ":")).encode("utf-8"))


def encoded_payload(encoded) -> tuple[bytes, bool]:
  codec_header = bytes(encoded.header)
  return codec_header + bytes(encoded.data), bool(codec_header)


def stream_metadata(bitrate: int, max_session_s: float) -> dict:
  return {
    "schema": "wayon-live-v1",
    "codec": "avc1.640020",
    "annexB": True,
    "width": 1344,
    "height": 760,
    "fps": 20,
    "bitratePerCamera": bitrate,
    "maxSessionSeconds": int(max_session_s),
    "cameras": ["wide", "driver"],
    "panorama": {
      "projectionModel": "equidistant-sphere-v1",
      "wideYawDeg": 0.0,
      "wideFovDeg": 205.0,
      "widePitchDeg": -6.5,
      "wideRollDeg": 0.0,
      "driverYawDeg": 180.0,
      "driverFovDeg": 205.0,
      "driverPitchDeg": -14.0,
      "driverRollDeg": 0.0,
      "driverMirror": False,
      "wideVerticalFovDeg": 128.0,
      "driverVerticalFovDeg": 128.0,
      "wideRadialDistortion": [-0.018, 0.006],
      "driverRadialDistortion": [-0.018, 0.006],
      "wideFocalScale": [0.31640625, 0.55953947],
      "driverFocalScale": [0.31640625, 0.55953947],
      "wideFisheyeDistortion": [-0.035, 0.0],
      "driverFisheyeDistortion": [-0.035, 0.0],
      "wideMaxThetaDeg": 102.5,
      "driverMaxThetaDeg": 102.5,
      "wideMaxThetaBiasDeg": 0.0,
      "driverMaxThetaBiasDeg": 0.0,
      "widePositionM": [-0.2, 0.0, 0.0],
      "driverPositionM": [0.0, -0.435, 0.03],
      "sphereRadiusM": 10.0,
      "wideOpticalCenter": [0.5, 0.5],
      "driverOpticalCenter": [0.5, 0.5],
      "wideVignetteCompensation": 0.045,
      "driverVignetteCompensation": 0.045,
      "blendDeg": 24.0,
    },
  }


def send_terminal(client: socket.socket, state: str, message: str = "") -> None:
  payload = {"state": state}
  if message:
    payload["message"] = message
  client.sendall(json_frame(FRAME_TYPE_STATUS, payload))
  # Give the TCP/WebSocket bridge time to forward a short final frame before EOF.
  time.sleep(0.25)


def send_stream_start(client, metadata):
  # The Android viewer treats metadata and live status as separate frame types.
  # Send this only after both cameras have yielded a decodable key frame.
  client.sendall(json_frame(FRAME_TYPE_METADATA, metadata) + json_frame(FRAME_TYPE_STATUS, {"state": "live"}))


def run_stream(client):
  from openpilot.cereal import messaging

  params = Params()
  if not media_ready(False, params):
    send_terminal(client, "onroad", "Parking media is unavailable")
    return
  max_session_s = 300
  control = ClientControlState()
  monitor = ClientHeartbeatMonitor(client, control)
  try:
    with CameraRequest(params, stream=True) as camera:
      monitor.start()
      poller = messaging.Poller()
      sockets = [messaging.sub_sock(name, poller=poller, conflate=True) for name in STREAM_SERVICES]
      first = {}
      deadline = time.monotonic() + 20
      while monitor.client_alive and len(first) < 2 and time.monotonic() < deadline:
        camera.renew()
        for sock in poller.poll(200):
          event = messaging.recv_one_or_none(sock)
          if event is not None:
            encoded = getattr(event, event.which())
            if encoded.header:
              first[event.which()] = event
      if len(first) != 2:
        send_terminal(client, "error", "Camera startup timed out")
        return
      sizes = {(int(getattr(event, name).width), int(getattr(event, name).height)) for name, event in first.items()}
      if len(sizes) != 1 or any(w <= 0 or h <= 0 for w, h in sizes):
        send_terminal(client, "error", "Unsupported camera dimensions")
        return
      width, height = next(iter(sizes))
      metadata = {**stream_metadata(LIVE_BITRATE, max_session_s), "width": width, "height": height, "state": "live"}
      send_stream_start(client, metadata)
      started = last_frame = time.monotonic()
      sequence = 0

      def send_event(event):
        nonlocal sequence, last_frame
        encoded = getattr(event, event.which())
        payload, key_frame = encoded_payload(encoded)
        client.sendall(pack_frame(STREAM_SERVICES[event.which()], payload, sequence,
                                  int(encoded.idx.timestampEof // 1000), key_frame))
        sequence += 1
        last_frame = time.monotonic()

      for event in first.values():
        send_event(event)
      while monitor.client_alive and time.monotonic() - started < max_session_s:
        camera.renew()
        if time.monotonic() - last_frame > 5:
          send_terminal(client, "error", "Camera frames stopped")
          return
        for sock in poller.poll(200):
          event = messaging.recv_one_or_none(sock)
          if event is not None:
            send_event(event)
      if monitor.client_alive:
        send_terminal(client, "expired")
      assert sockets  # Keep subscriptions alive for the session.
  except (BlockingIOError, InterruptedError):
    send_terminal(client, "onroad" if not media_ready(False, params) else "busy")
  finally:
    monitor.stop()


def main():
  params = Params()
  with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((LISTEN_HOST, LISTEN_PORT))
    server.listen(1)
    server.settimeout(0.5)
    while media_ready(False, params):
      try:
        client, _ = server.accept()
      except TimeoutError:
        continue
      with client:
        client.settimeout(1)
        client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        try:
          run_stream(client)
        except Exception as exc:
          print(f"Hylink live: {type(exc).__name__}", flush=True)


if __name__ == "__main__":
  main()
