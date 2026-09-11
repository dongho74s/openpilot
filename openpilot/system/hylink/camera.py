"""Camera requests only. The openpilot manager remains the sole camerad owner."""
import base64
import io
import os
import time

from openpilot.common.params import Params
from openpilot.system.hylink.camera_lease import CameraLease
from openpilot.system.hylink.runtime import CAMERA_PATH, media_ready, read_json, write_json


class CameraRequest:
  def __init__(self, params, stream=False):
    self.params, self.stream = params, stream
    self.lease = CameraLease("hylink", 2.5)

  def allowed(self):
    return (media_ready(False, self.params) and not self.params.get_bool("IsDriverViewEnabled")
            and not self.params.get_bool("IsTakingSnapshot"))

  def __enter__(self):
    if not self.allowed() or not self.lease.acquire():
      raise BlockingIOError("Camera unavailable")
    try:
      self.renew()
    except BaseException:
      self.lease.release()
      raise
    return self

  def renew(self):
    if not self.allowed():
      raise InterruptedError("Parking camera no longer permitted")
    write_json(CAMERA_PATH, {"pid": os.getpid(), "at": time.monotonic(), "active": True, "stream": self.stream})

  def __exit__(self, *args):
    try:
      if read_json(CAMERA_PATH).get("pid") == os.getpid():
        CAMERA_PATH.unlink(missing_ok=True)
    finally:
      self.lease.release()


def capture_images():
  from msgq.visionipc import VisionIpcClient, VisionStreamType
  from openpilot.system.camerad.snapshot import extract_image
  from openpilot.cereal import messaging

  params = Params()
  with CameraRequest(params) as request:
    clients = {name: VisionIpcClient("camerad", stream, True) for name, stream in (
      ("wide", VisionStreamType.VISION_STREAM_WIDE_ROAD), ("driver", VisionStreamType.VISION_STREAM_DRIVER))}
    connected = set()
    images = {}
    started = time.monotonic()
    sm = messaging.SubMaster(["wideRoadCameraState", "driverCameraState"])
    while time.monotonic() - started < 20:
      request.renew()
      sm.update(0)
      for name, client in clients.items():
        if name not in connected:
          if not client.connect(False):
            continue
          connected.add(name)
        frame = client.recv(100)
        # Allow exposure/focus to settle; never wait forever for a missing camera.
        if frame is not None and time.monotonic() - started >= 4:
          images[name] = extract_image(frame)
      if len(images) == len(clients):
        return images
    return images


def jpeg_b64(image):
  from PIL import Image
  buffer = io.BytesIO()
  frame = Image.fromarray(image)
  frame.thumbnail((1344, 1008))
  frame.save(buffer, "JPEG", quality=72)
  return base64.b64encode(buffer.getvalue()).decode("ascii")


def snapshot_payload(config):
  from openpilot.system.hylink.telemetry import utc_now
  images = capture_images()
  if not images:
    return None
  return {"deviceId": config["device_id"], "capturedAt": utc_now(),
          "wideJpegBase64": jpeg_b64(images["wide"]) if "wide" in images else None,
          "driverJpegBase64": jpeg_b64(images["driver"]) if "driver" in images else None}
