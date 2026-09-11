#!/usr/bin/env python3
import json
import math
import queue
import select
import socket
import struct
import threading
import time
from collections import deque
from concurrent.futures import ThreadPoolExecutor

from openpilot.system.hylink.camera import CameraRequest
from openpilot.system.hylink.live_archive import (
  build_dual_h264_archive,
  frame_duration_s,
  select_recent_frames,
  upload_live_capture,
  utc_now,
)


from openpilot.system.hylink.runtime import read_config, media_ready
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
CLIENT_CONTROL_MAGIC = b"WLC1"
CLIENT_CONTROL_HEADER = struct.Struct(">4sBI")
CLIENT_CONTROL_PHOTO = 1
CLIENT_CONTROL_CLIP = 2
MAX_CONTROL_PAYLOAD = 8 * 1024 * 1024
MAX_CLIP_BUFFER_S = 36.0
MAX_PENDING_ARCHIVES = 3

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

class BoundedArchiveExecutor:
  def __init__(self):
    self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="hylink-archive")
    self.slots = threading.BoundedSemaphore(3)

  def submit(self, fn, *args):
    if not self.slots.acquire(blocking=False):
      raise RuntimeError("archive_queue_full")
    try:
      future = self.pool.submit(fn, *args)
    except BaseException:
      self.slots.release()
      raise
    future.add_done_callback(lambda _: self.slots.release())
    return future


ARCHIVE_EXECUTOR = BoundedArchiveExecutor()


class ClientControlState:
  def __init__(self):
    self.heartbeat_seen = False
    self.last_heartbeat = time.monotonic()
    self.buffer = bytearray()
    self.commands = queue.SimpleQueue()

  def feed(self, data: bytes) -> None:
    if not data:
      return

    self.buffer.extend(data)
    while self.buffer:
      if self.buffer.startswith(CLIENT_HEARTBEAT_MAGIC):
        del self.buffer[:len(CLIENT_HEARTBEAT_MAGIC)]
        self.heartbeat_seen = True
        self.last_heartbeat = time.monotonic()
        continue

      if self.buffer.startswith(CLIENT_CONTROL_MAGIC):
        if len(self.buffer) < CLIENT_CONTROL_HEADER.size:
          return
        _magic, command_type, payload_size = CLIENT_CONTROL_HEADER.unpack_from(self.buffer)
        if payload_size > MAX_CONTROL_PAYLOAD:
          del self.buffer[:len(CLIENT_CONTROL_MAGIC)]
          continue
        frame_size = CLIENT_CONTROL_HEADER.size + payload_size
        if len(self.buffer) < frame_size:
          return
        payload = bytes(self.buffer[CLIENT_CONTROL_HEADER.size:frame_size])
        del self.buffer[:frame_size]
        self.commands.put((command_type, payload))
        continue

      marker_indexes = [
        index for index in (
          self.buffer.find(CLIENT_HEARTBEAT_MAGIC, 1),
          self.buffer.find(CLIENT_CONTROL_MAGIC, 1),
        ) if index >= 0
      ]
      if marker_indexes:
        del self.buffer[:min(marker_indexes)]
        continue

      keep = b""
      payload = bytes(self.buffer)
      for length in range(min(3, len(payload)), 0, -1):
        suffix = payload[-length:]
        if CLIENT_HEARTBEAT_MAGIC.startswith(suffix) or CLIENT_CONTROL_MAGIC.startswith(suffix):
          keep = suffix
          break
      self.buffer = bytearray(keep)
      return

  def pop_commands(self):
    commands = []
    while True:
      try:
        commands.append(self.commands.get_nowait())
      except queue.Empty:
        return commands


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


class ClipFrameStore:
  def __init__(self, max_buffer_s: float = MAX_CLIP_BUFFER_S):
    self.max_buffer_s = max_buffer_s
    self.lock = threading.Lock()
    self.buffers = {
      FRAME_TYPE_WIDE: deque(),
      FRAME_TYPE_DRIVER: deque(),
    }

  def append(self, frame_type: int, frame) -> None:
    received_at = frame[0]
    with self.lock:
      frame_buffer = self.buffers[frame_type]
      frame_buffer.append(frame)
      cutoff = received_at - self.max_buffer_s
      # Bound memory even if the encoder bitrate or timestamps are abnormal.
      while frame_buffer and (frame_buffer[0][0] < cutoff or len(frame_buffer) > 800
                              or sum(len(item[1]) for item in frame_buffer) > 16 * 1024 * 1024):
        frame_buffer.popleft()

  def recent_pair(self, duration_s: float, now_s: float):
    with self.lock:
      wide = list(self.buffers[FRAME_TYPE_WIDE])
      driver = list(self.buffers[FRAME_TYPE_DRIVER])
    return (
      select_recent_frames(wide, duration_s, now_s),
      select_recent_frames(driver, duration_s, now_s),
    )


class ClipFrameCollector:
  def __init__(self, messaging, frame_store: ClipFrameStore):
    self.messaging = messaging
    self.frame_store = frame_store
    self.stop_event = threading.Event()
    self.thread = threading.Thread(target=self._run, name="wayon-live-clip-buffer", daemon=True)

  def start(self) -> None:
    self.thread.start()

  def stop(self) -> None:
    self.stop_event.set()
    if self.thread.is_alive():
      self.thread.join(timeout=2.0)

  def _run(self) -> None:
    poller = self.messaging.Poller()
    _stream_sockets = [
      self.messaging.sub_sock(service, poller=poller, conflate=False)
      for service in STREAM_SERVICES
    ]
    try:
      while not self.stop_event.is_set():
        for stream_socket in poller.poll(250):
          event = self.messaging.recv_one_or_none(stream_socket)
          if event is None:
            continue
          service = event.which()
          encoded = getattr(event, service)
          payload, key_frame = encoded_payload(encoded)
          frame_type = STREAM_SERVICES[service]
          timestamp_us = int(encoded.idx.timestampEof // 1000)
          self.frame_store.append(
            frame_type,
            (time.monotonic(), payload, key_frame, timestamp_us),
          )
    except Exception as exc:
      print(f"Wayon live: clip collector failed: {exc}", flush=True)


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


def send_capture_status(client: socket.socket, capture_state: str, kind: str,
                        message: str = "", duration_s: float | None = None,
                        capture_id: str | None = None) -> None:
  payload = {
    "state": "capture",
    "captureState": capture_state,
    "kind": kind,
  }
  if message:
    payload["message"] = message
  if duration_s is not None:
    payload["durationSeconds"] = round(duration_s, 3)
  if capture_id:
    payload["captureId"] = capture_id
  client.sendall(json_frame(FRAME_TYPE_STATUS, payload))


def upload_clip_archive(config: dict, device_id: str, captured_at: str, duration_s: float,
                        requested_duration_s: int, wide_frames, driver_frames,
                        metadata: dict) -> dict:
  archive = build_dual_h264_archive(
    wide_frames,
    driver_frames,
    captured_at,
    requested_duration_s,
    metadata,
  )
  try:
    return upload_live_capture(
      config,
      device_id,
      "clip",
      captured_at,
      "application/zip",
      "dual_h264_360",
      archive,
      duration_s,
    )
  except Exception as exc:
    raise RuntimeError(f"archive_bytes={len(archive)} {type(exc).__name__}: {exc}") from exc


def process_capture_commands(client: socket.socket, control: ClientControlState,
                             config: dict, device_id: str, frame_store: ClipFrameStore,
                             metadata: dict, pending_archives: list) -> None:
  for command_type, payload in control.pop_commands():
    if len(pending_archives) >= MAX_PENDING_ARCHIVES:
      send_capture_status(client, "error", "unknown", "archive_queue_full")
      continue

    captured_at = utc_now()
    if command_type == CLIENT_CONTROL_PHOTO:
      if len(payload) < 4 or not payload.startswith(b"\xff\xd8") or not payload.endswith(b"\xff\xd9"):
        send_capture_status(client, "error", "photo", "invalid_jpeg")
        continue
      future = ARCHIVE_EXECUTOR.submit(
        upload_live_capture,
        config,
        device_id,
        "photo",
        captured_at,
        "image/jpeg",
        "equirectangular_360",
        payload,
      )
      pending_archives.append((future, {"kind": "photo", "duration": None}))
      send_capture_status(client, "uploading", "photo")
      continue

    if command_type != CLIENT_CONTROL_CLIP or len(payload) != 1 or payload[0] not in (10, 30):
      send_capture_status(client, "error", "unknown", "invalid_capture_command")
      continue

    requested_duration = int(payload[0])
    now = time.monotonic()
    wide_frames, driver_frames = frame_store.recent_pair(requested_duration, now)
    actual_duration = min(
      frame_duration_s(wide_frames),
      frame_duration_s(driver_frames),
    ) if wide_frames and driver_frames else 0.0
    if actual_duration < requested_duration * 0.9:
      send_capture_status(client, "buffering", "clip", duration_s=actual_duration)
      continue

    future = ARCHIVE_EXECUTOR.submit(
      upload_clip_archive,
      config,
      device_id,
      captured_at,
      actual_duration,
      requested_duration,
      wide_frames,
      driver_frames,
      metadata,
    )
    pending_archives.append((future, {"kind": "clip", "duration": actual_duration}))
    send_capture_status(client, "uploading", "clip", duration_s=actual_duration)


def process_archive_results(client: socket.socket, pending_archives: list) -> None:
  remaining = []
  for future, info in pending_archives:
    if not future.done():
      remaining.append((future, info))
      continue
    try:
      result = future.result()
      send_capture_status(
        client,
        "saved",
        info["kind"],
        duration_s=info["duration"],
        capture_id=str(result.get("id") or ""),
      )
    except Exception as exc:
      print(f"Wayon live: archive upload failed: {exc}", flush=True)
      send_capture_status(client, "error", info["kind"], str(exc)[:220], duration_s=info["duration"])
  pending_archives[:] = remaining


def run_stream(client):
  from openpilot.cereal import messaging

  params = Params()
  if not media_ready(False, params):
    send_terminal(client, "onroad", "Parking media is unavailable")
    return
  config = read_config(params)
  max_session_s = 300
  control = ClientControlState()
  monitor = ClientHeartbeatMonitor(client, control)
  collector = None
  pending_archives = []
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
      client.sendall(json_frame(FRAME_TYPE_METADATA, metadata))
      store = ClipFrameStore()
      collector = ClipFrameCollector(messaging, store)
      collector.start()
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
        process_archive_results(client, pending_archives)
        process_capture_commands(client, control, config, config["device_id"], store, metadata, pending_archives)
      if monitor.client_alive:
        send_terminal(client, "expired")
      assert sockets  # Keep subscriptions alive for the session.
  except (BlockingIOError, InterruptedError):
    send_terminal(client, "onroad" if not media_ready(False, params) else "busy")
  finally:
    if collector:
      collector.stop()
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
