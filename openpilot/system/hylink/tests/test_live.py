import json
import socket
import time
from types import SimpleNamespace

import pytest

from openpilot.system.hylink.live import (
  CLIENT_HEARTBEAT_MAGIC,
  ClientHeartbeatMonitor,
  ClientControlState,
  FRAME_FLAG_KEY,
  FRAME_HEADER,
  FRAME_MAGIC,
  FRAME_TYPE_METADATA,
  FRAME_TYPE_STATUS,
  bounded_number,
  encoded_payload,
  json_frame,
  pack_frame,
  read_client_control,
  stream_metadata,
  send_stream_start,
)


def unpack_frame(frame):
  header = FRAME_HEADER.unpack(frame[:FRAME_HEADER.size])
  magic, frame_type, flags, _, sequence, timestamp_us, payload_size = header
  return magic, frame_type, flags, sequence, timestamp_us, frame[FRAME_HEADER.size:FRAME_HEADER.size + payload_size]


def test_pack_frame_preserves_header_and_payload():
  frame = pack_frame(2, b"video", sequence=42, timestamp_us=123_456, key_frame=True)
  magic, frame_type, flags, sequence, timestamp_us, payload = unpack_frame(frame)

  assert magic == FRAME_MAGIC
  assert frame_type == 2
  assert flags & FRAME_FLAG_KEY
  assert sequence == 42
  assert timestamp_us == 123_456
  assert payload == b"video"


def test_json_frame_uses_compact_utf8_payload():
  frame = json_frame(FRAME_TYPE_METADATA, {"state": "live", "fps": 20})
  *_, payload = unpack_frame(frame)
  assert json.loads(payload) == {"state": "live", "fps": 20}


def test_live_start_includes_separate_status_after_metadata():
  packets = []
  send_stream_start(SimpleNamespace(sendall=packets.append), {"width": 1344, "height": 760})
  wire = packets[0]
  *_, payload = unpack_frame(wire)
  assert FRAME_HEADER.unpack(wire[:FRAME_HEADER.size])[1] == FRAME_TYPE_METADATA
  assert json.loads(payload) == {"width": 1344, "height": 760}
  status = wire[FRAME_HEADER.size + len(payload):]
  *_, body = unpack_frame(status)
  assert FRAME_HEADER.unpack(status[:FRAME_HEADER.size])[1] == FRAME_TYPE_STATUS
  assert json.loads(body) == {"state": "live"}


def test_encoded_payload_prepends_codec_header_on_key_frame():
  payload, key_frame = encoded_payload(SimpleNamespace(header=b"sps-pps", data=b"frame"))
  assert payload == b"sps-ppsframe"
  assert key_frame is True

  payload, key_frame = encoded_payload(SimpleNamespace(header=b"", data=b"delta"))
  assert payload == b"delta"
  assert key_frame is False


def test_bounded_number_clamps_and_falls_back():
  assert bounded_number("800", 10, 100, 1000) == 800
  assert bounded_number(10, 50, 20, 100) == 20
  assert bounded_number("bad", 50, 20, 100) == 50








def test_driver_camera_uses_correct_horizontal_orientation():
  metadata = stream_metadata(800_000, 300)
  assert metadata["panorama"]["driverMirror"] is False
  assert metadata["panorama"]["blendDeg"] == 24.0
  assert metadata["panorama"]["wideRadialDistortion"] == [-0.018, 0.006]


def test_panorama_uses_mici_equidistant_projection():
  panorama = stream_metadata(800_000, 300)["panorama"]

  assert panorama["projectionModel"] == "equidistant-sphere-v1"
  assert panorama["wideFocalScale"] == [0.31640625, 0.55953947]
  assert panorama["driverFocalScale"] == [0.31640625, 0.55953947]
  assert panorama["widePitchDeg"] == -6.5
  assert panorama["driverPitchDeg"] == -14.0
  assert panorama["widePositionM"] == [-0.2, 0.0, 0.0]
  assert panorama["driverPositionM"] == [0.0, -0.435, 0.03]


def test_client_control_accepts_fragmented_heartbeat():
  state = ClientControlState()

  state.feed(CLIENT_HEARTBEAT_MAGIC[:2])
  assert not state.heartbeat_seen
  assert bytes(state.buffer) == CLIENT_HEARTBEAT_MAGIC[:2]

  state.feed(CLIENT_HEARTBEAT_MAGIC[2:])
  assert state.heartbeat_seen
  assert not state.buffer


def test_read_client_control_detects_heartbeat_and_disconnect():
  client, peer = socket.socketpair()
  state = ClientControlState()
  try:
    peer.sendall(CLIENT_HEARTBEAT_MAGIC)
    assert read_client_control(client, state, timeout_s=0.1)
    assert state.heartbeat_seen

    peer.close()
    assert not read_client_control(client, state, timeout_s=0.1)
  finally:
    client.close()


def test_heartbeat_monitor_closes_stale_client():
  client, peer = socket.socketpair()
  state = ClientControlState()
  monitor = ClientHeartbeatMonitor(client, state, timeout_s=0.05, poll_s=0.01)
  try:
    monitor.start()
    deadline = time.monotonic() + 0.5
    while monitor.client_alive and time.monotonic() < deadline:
      time.sleep(0.01)

    assert monitor.timed_out
    assert not monitor.client_alive
    assert peer.recv(1) == b""
  finally:
    monitor.stop()
    peer.close()
    client.close()


@pytest.mark.parametrize("command", [bytes.fromhex("574c433101000000046a706567"), bytes.fromhex("574c433102000000011e")])
def test_removed_capture_commands_are_rejected(command):
  state = ClientControlState()
  with pytest.raises(ValueError, match="Unsupported live control"):
    state.feed(command)
  assert not state.heartbeat_seen


def test_heartbeat_buffer_is_bounded_and_multiple_heartbeats_are_accepted():
  state = ClientControlState()
  state.feed(CLIENT_HEARTBEAT_MAGIC * 100 + b"WL")
  assert state.heartbeat_seen
  assert state.buffer == b"WL"
  state.feed(b"P1")
  assert not state.buffer


def test_live_module_has_no_recording_buffer_or_upload_worker():
  from openpilot.system.hylink import live
  for name in ("ClipFrameStore", "ClipFrameCollector", "ARCHIVE_EXECUTOR",
               "process_capture_commands", "upload_live_capture"):
    assert not hasattr(live, name)
