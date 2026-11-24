"""Last-drive tuning advice, step 1: a drive's logs -> 100 Hz rows of what lateral control did.

One row per controlsState with the latest carState, carControl, carOutput, liveParameters and modelV2 values (the
same columns the Mac tuning analysis of 2026-11-24 used). The qlog (10 Hz) is read first so segments in which
openpilot never steered are not decoded at full rate.
"""
from __future__ import annotations

import os
import re
from dataclasses import dataclass, field

import numpy as np

REALDATA = "/data/media/0/realdata"
COLS = ("t", "v", "a", "angle", "rate", "drvTq", "pressed", "latActive", "sent", "desCurv", "curv", "out", "p", "i", "f",
        "des", "act", "roll", "angleOffset", "stiffness", "sR", "oriP", "angZ", "model_idx", "mdlCurv", "laneOff", "lp1",
        "lp2", "laneW", "laneCurv", "enabled", "faultTemp", "latDelay")
C = {k: i for i, k in enumerate(COLS)}
# settings shown in the result (initData carries every Param at loggerd start; tokens and keys stay in memory only)
TUNING_PARAMS = ("LateralTorqueCustom", "LateralTorqueKpV", "LateralTorqueKiV", "LateralTorqueKf", "LateralTorqueKd",
                 "LateralTorqueFriction", "LateralTorqueAccelFactor", "LateralAccelLimit", "LatJerkMax",
                 "LatTorqueLimitPFilter", "ChevroletSteerFF", "NNFF", "NNFFLite", "SteerActuatorDelay")
SEGMENT_RE = re.compile(r"^([0-9a-f]{8}--[0-9a-f]{10})--(\d+)$")
LAT_MIN_SPEED = 8 / 3.6


@dataclass
class Segment:
  route: str
  number: int
  a: np.ndarray                       # rows x COLS
  models: list = field(default_factory=list)  # per modelV2: (acceleration.y, orientation.x, orientation.y)


@dataclass
class Drive:
  route: str
  segments: list[Segment]
  params: dict[str, bytes]            # initData params (all of them, for the simulator's stand-in Params)
  car_params: bytes | None
  car_fingerprint: str
  wall_start: float | None            # unix seconds of the route start
  partial: list[int] = field(default_factory=list)  # segments whose log ended early (power cut)

  def tuning(self) -> dict[str, str]:
    return {k: self.params[k].decode(errors="replace") for k in TUNING_PARAMS if k in self.params}


STREAM_RESERVE_MB = 32.0   # a segment's decode needs about a chunk and a batch of messages beyond the rows it keeps


def route_counter(route: str) -> int:
  try:
    return int(route.split("--")[0], 16)
  except ValueError:
    return -1


def list_routes(root: str = REALDATA) -> dict[str, list[int]]:
  """{route: sorted segment numbers}, routes ordered newest first (the route counter, not the clock)."""
  routes: dict[str, list[int]] = {}
  try:
    names = os.listdir(root)
  except OSError:
    return {}
  for name in names:
    m = SEGMENT_RE.match(name)
    if m:
      routes.setdefault(m.group(1), []).append(int(m.group(2)))
  return {r: sorted(routes[r]) for r in sorted(routes, key=route_counter, reverse=True)}


def segment_file(root: str, route: str, n: int, kind: str) -> str | None:
  for ext in (".zst", ""):
    p = os.path.join(root, f"{route}--{n}", f"{kind}{ext}")
    if os.path.isfile(p):
      return p
  return None


READ_CHUNK = 1 << 20   # decompressed bytes per read: memory stays at about a chunk plus one batch of messages


def _complete_length(buf, pos: int) -> int:
  """Length of the Cap'n Proto message (segment table framing) starting at pos, or 0 if not all of it is in buf."""
  if len(buf) - pos < 8:
    return 0
  count = int.from_bytes(buf[pos:pos + 4], "little") + 1
  header = 4 + 4 * count
  header += (8 - header % 8) % 8
  if count > 512 or len(buf) - pos < header:
    return 0 if count <= 512 else -1
  words = sum(int.from_bytes(buf[pos + 4 + 4 * i:pos + 8 + 4 * i], "little") for i in range(count))
  total = header + 8 * words
  return total if len(buf) - pos >= total else 0


def read_events(path: str):
  """Events of a log file, decoded a chunk at a time (never the whole file in memory). A log cut short by a power loss
  yields what was written."""
  from openpilot.cereal import log
  with open(path, "rb") as f:
    if path.endswith(".zst"):
      import zstandard
      reader = zstandard.ZstdDecompressor().stream_reader(f, read_size=READ_CHUNK // 4)
    else:
      reader = f
    buf = bytearray()
    done = False
    while not done:
      chunk = reader.read(READ_CHUNK)
      if chunk:
        buf += chunk
      else:
        done = True
      end = 0
      while True:
        n = _complete_length(buf, end)
        if n <= 0:
          if n < 0:
            done, buf = True, bytearray()   # not a message boundary: stop rather than misread the rest
          break
        end += n
      if end:
        batch = bytes(buf[:end])
        del buf[:end]
        yield from log.Event.read_multiple_bytes(batch, traversal_limit_in_words=2 ** 63, nesting_limit=1000)


CHECK_EVERY_EVENTS = 20000   # about half a second of decoding on a comma 4 between the runner's pacing checks


def qlog_lateral_seconds(path: str) -> float:
  """Seconds in which openpilot steered above 8 km/h, from the 10 Hz qlog."""
  n = 0
  v = 0.0
  try:
    for e in read_events(path):
      w = e.which()
      if w == "carState":
        v = e.carState.vEgo
      elif w == "carControl" and e.carControl.latActive and v > LAT_MIN_SPEED:
        n += 1
  except Exception:
    pass
  return n / 10.0


def lane_fit(x, y1, y2, v):
  """Lane centre offset at the car (m, + = centre to the right), lane width and curvature from laneLines 1 and 2."""
  xc = np.asarray(x)
  yc = (np.asarray(y1) + np.asarray(y2)) / 2
  hi = max(20.0, min(60.0, v * 2.5))
  m = (xc >= 3) & (xc <= hi)
  if m.sum() < 4:
    return np.nan, np.nan, np.nan
  a2, _b, c = np.polyfit(xc[m], yc[m], 2)
  return c, float(np.median((np.asarray(y2) - np.asarray(y1))[m])), 2 * a2


def read_segment(path: str, route: str, number: int, want_init: bool = False, checkpoint=None):
  """(Segment or None, initData params or None, carParams bytes or None, route wall start or None, cut_short)"""
  rows, models = [], []
  cs = cc = co = lp = None
  lat_delay = float("nan")
  mc = 0.0
  lane = (np.nan, np.nan, np.nan)
  probs = (0.0, 0.0)
  params = car_params = wall = None
  cut = False
  try:
    for count, e in enumerate(read_events(path)):
      if checkpoint is not None and count % CHECK_EVERY_EVENTS == CHECK_EVERY_EVENTS - 1:
        checkpoint()
      w = e.which()
      if w == "carState":
        cs = e.carState
      elif w == "carControl":
        cc = e.carControl
      elif w == "carOutput":
        co = e.carOutput
      elif w == "liveParameters":
        lp = e.liveParameters
      elif w == "liveDelay":
        lat_delay = e.liveDelay.lateralDelay
      elif w == "modelV2":
        m = e.modelV2
        models.append((np.array(m.acceleration.y, dtype=np.float32), np.array(m.orientation.x, dtype=np.float32),
                       np.array(m.orientation.y, dtype=np.float32)))
        mc = m.action.desiredCurvature
        try:
          ll = m.laneLines
          lane = lane_fit(ll[1].x, ll[1].y, ll[2].y, cs.vEgo if cs is not None else 10.0)
          probs = (m.laneLineProbs[1], m.laneLineProbs[2])
        except Exception:
          lane, probs = (np.nan, np.nan, np.nan), (0.0, 0.0)
      elif w == "controlsState" and cs is not None and cc is not None and co is not None and lp is not None:
        c = e.controlsState
        ts = c.lateralControlState.torqueState
        ori = list(cc.orientationNED)
        ang = list(cc.angularVelocity)
        rows.append((e.logMonoTime / 1e9, cs.vEgo, cs.aEgo, cs.steeringAngleDeg, cs.steeringRateDeg, cs.steeringTorque,
                     cs.steeringPressed, cc.latActive, co.actuatorsOutput.torque, c.desiredCurvature, c.curvature, ts.output,
                     ts.p, ts.i, ts.f, ts.desiredLateralAccel, ts.actualLateralAccel, lp.roll, lp.angleOffsetDeg,
                     lp.stiffnessFactor, lp.steerRatio, ori[1] if len(ori) > 1 else 0.0, ang[2] if len(ang) > 2 else 0.0,
                     len(models) - 1, mc, lane[0], probs[0], probs[1], lane[1], lane[2], cc.enabled, cs.steerFaultTemporary,
                     lat_delay))
      elif want_init and w == "initData" and params is None:
        ini = e.initData
        params = {ent.key: bytes(ent.value) for ent in ini.params.entries}
        wall = ini.wallTimeNanos / 1e9 if ini.wallTimeNanos else None
      elif want_init and w == "carParams" and car_params is None:
        car_params = e.carParams.as_builder().to_bytes()
  except InterruptedError:
    raise
  except Exception:
    cut = True
  seg = Segment(route, number, np.array(rows, dtype=np.float64), models) if len(rows) >= 100 else None
  return seg, params, car_params, wall, cut


def load_drive(route: str, numbers: list[int], root: str = REALDATA, checkpoint=None, progress=None) -> Drive:
  """Decode the segments of a route in which openpilot steered (per qlog; segments without a qlog are decoded).
  checkpoint() runs before each segment (the runner's pacing; it may raise InterruptedError)."""
  segments, partial = [], []
  params: dict[str, bytes] = {}
  car_params = None
  wall = None
  fingerprint = ""
  for k, n in enumerate(numbers):
    if checkpoint is not None:
      checkpoint()
    if progress is not None:
      progress(k, len(numbers))
    rlog = segment_file(root, route, n, "rlog")
    if rlog is None:
      continue
    need_init = not params or car_params is None
    q = segment_file(root, route, n, "qlog")
    if not need_init and q is not None and qlog_lateral_seconds(q) < 3.0:
      continue
    if checkpoint is not None:
      checkpoint(need_mb=STREAM_RESERVE_MB)   # checked before the segment is read
    seg, p, cp, w, cut = read_segment(rlog, route, n, want_init=need_init, checkpoint=checkpoint)
    if p and not params:
      params = p
      wall = w        # loggerd writes the route's own initData (route start) into every segment
    if cp and car_params is None:
      car_params = cp
    if cut:
      partial.append(n)
    if seg is not None and (seg.a[:, C["latActive"]] > 0.5).any():
      segments.append(seg)
  if car_params is not None:
    try:
      from openpilot.cereal import car
      with car.CarParams.from_bytes(car_params) as cp_reader:
        fingerprint = str(cp_reader.carFingerprint)
    except Exception:
      fingerprint = ""
  return Drive(route, segments, params, car_params, fingerprint, wall, partial)
