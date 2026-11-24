"""Analyse the last drive and write the tuning advice for Carrot web (features/tuning_advice.py starts this process).

  python -m openpilot.selfdrive.carrot.tuning_advice.runner [--route ROUTE] [--root DIR] [--state-dir DIR]

Always at the lowest CPU and I/O priority, and it never stops because the ignition comes on: between log segments and
replay windows it looks at the car and sets its pace.
- ignition off, or on with the gear in P at a standstill: full pace;
- driving: it rests as long as it worked after every step (half of one little core at most).
With the ignition on it also waits while free memory or the CPU/GPU temperature is tight and gives up after 10 minutes
of waiting. Started with the ignition on (a comma that powers off with the car is never parked long enough), it
analyses the drive before the one being recorded.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
import traceback

VERSION = 1
STATE_DIR = os.environ.get("CARROT_STATE_DIR", "/data/carrot/state")
RESULT_NAME = "tuning_advice.json"
STATUS_NAME = "tuning_advice_status.json"
HISTORY_NAME = "tuning_advice_history.json"
HISTORY_LIMIT = 20
ROUTES_TO_SEARCH = 6
MIN_ROUTE_LAT_SECONDS = 180.0
# With the ignition on: selfdrived raises lowMemory above 90 % memory use (about 360 MB free of 3.6 GB). Traverse drives 00000250/252
# ran at 76 % (p50, about 860 MB free), 81 % at most (about 690 MB free); this run peaks at about 190 MB (comma 4,
# 2026-11-24). Below 600 MB free it waits, which leaves that peak plus a margin above the alert.
GENTLE_MIN_AVAILABLE_MB = 600
GENTLE_MAX_TEMP_C = 75.0
GENTLE_REST_RATIO = 1.0          # rest as long as the work since the last check took
GENTLE_MAX_REST_S = 30.0
CHECK_MIN_INTERVAL_S = 0.25      # checks come every ~0.3-0.5 s of work (segment decode, replay frames, qlog scan)
LOCK_NAME = "tuning_advice.lock"
GENTLE_WAIT_S = 10.0
GENTLE_GIVE_UP_S = 600.0
DESIRED_MISMATCH_ABS = 0.1      # m/s^2 RMS between the replayed and the logged desired lateral accel
DESIRED_MISMATCH_REL = 0.25     # or a quarter of the logged demand's RMS
THERMAL_ZONE_TYPES = tuple(f"cpu{i}-{c}-usr" for c in ("silver", "gold") for i in range(4)) + ("gpu0-usr", "gpu1-usr")


def mem_available_mb() -> float | None:
  try:
    with open("/proc/meminfo") as f:
      for line in f:
        if line.startswith("MemAvailable:"):
          return int(line.split()[1]) / 1024
  except Exception:
    pass
  return None


_thermal_paths: list[str] | None = None


def max_temp_c() -> float | None:
  global _thermal_paths
  import glob
  if _thermal_paths is None:
    _thermal_paths = []
    for zone in glob.glob("/sys/class/thermal/thermal_zone*"):
      try:
        with open(os.path.join(zone, "type")) as f:
          if f.read().strip() in THERMAL_ZONE_TYPES:
            _thermal_paths.append(os.path.join(zone, "temp"))
      except Exception:
        pass
  temps = []
  for p in _thermal_paths:
    try:
      with open(p) as f:
        temps.append(int(f.read().strip()) / 1000)
    except Exception:
      pass
  return max(temps) if temps else None


def resources() -> tuple[float | None, float | None]:
  return mem_available_mb(), max_temp_c()


def write_json(path: str, data: dict) -> None:
  tmp = f"{path}.tmp{os.getpid()}"
  with open(tmp, "w", encoding="utf-8") as f:
    json.dump(data, f, ensure_ascii=False, separators=(",", ":"), allow_nan=False, default=_plain)
  os.replace(tmp, path)


def read_json(path: str, default):
  try:
    with open(path, encoding="utf-8") as f:
      return json.load(f)
  except Exception:
    return default


def _plain(x):
  try:
    import numpy as np
    if isinstance(x, np.integer):
      return int(x)
    if isinstance(x, np.floating):
      return float(x) if np.isfinite(x) else None
  except Exception:
    pass
  raise TypeError(type(x).__name__)


def _finite(obj):
  """NaN/inf -> None all the way down (the JSON is read by the browser)."""
  import math
  if isinstance(obj, float):
    return obj if math.isfinite(obj) else None
  if isinstance(obj, dict):
    return {k: _finite(v) for k, v in obj.items()}
  if isinstance(obj, (list, tuple)):
    return [_finite(v) for v in obj]
  return obj


OFFROAD, PARKED, DRIVING = "offroad", "parked", "driving"
CAR_STATE_MAX_AGE_S = 2.0


def make_drive_state(params):
  """offroad / parked (ignition on, gear P, standstill) / driving, from IsOnroad and the latest carState. Without a
  recent carState the car counts as driving."""
  sock = None
  last, last_t = None, 0.0

  def drive_state() -> str:
    nonlocal sock, last, last_t
    try:
      if not params.get_bool("IsOnroad"):
        return OFFROAD
    except Exception:
      return DRIVING
    try:
      import openpilot.cereal.messaging as messaging
      if sock is None:
        sock = messaging.sub_sock("carState", conflate=True)
      msg = messaging.recv_one_or_none(sock)
      if msg is not None:
        last, last_t = msg.carState, time.monotonic()
      if last is not None and time.monotonic() - last_t < CAR_STATE_MAX_AGE_S:
        if last.gearShifter == "park" and last.standstill:
          return PARKED
    except Exception:
      pass
    return DRIVING
  return drive_state


class Job:
  def __init__(self, state_dir: str, drive_state, sleep=time.sleep, clock=time.monotonic, read_resources=resources):
    self.state_dir = state_dir
    self.drive_state = drive_state
    self.sleep, self.clock, self.read_resources = sleep, clock, read_resources
    self.last_mark = clock()
    self.started = time.time()
    self.status = {"state": "running", "step": "searching", "progress": 0, "started_at": int(self.started), "pid": os.getpid()}

  def update(self, **kw) -> None:
    self.status.update(kw)
    self.status["updated_at"] = int(time.time())
    write_json(os.path.join(self.state_dir, STATUS_NAME), _finite(self.status))

  def checkpoint(self) -> None:
    """Called every few hundred ms of work: set the pace for where the car is (driving: rest as long as the work since
    the last check took), wait while memory or temperature is tight (ignition on), and raise InterruptedError("busy")
    after 10 minutes of waiting."""
    if self.clock() - self.last_mark < CHECK_MIN_INTERVAL_S and "drive_state" in self.status:
      return
    state = self._state()
    if state == DRIVING:
      worked = self.clock() - self.last_mark
      if worked > 0:
        self.sleep(min(worked * GENTLE_REST_RATIO, GENTLE_MAX_REST_S))
    waiting_since = None
    step = self.status.get("step")
    while state != OFFROAD:
      mem, temp = self.read_resources()
      tight = (mem is not None and mem < GENTLE_MIN_AVAILABLE_MB) or (temp is not None and temp >= GENTLE_MAX_TEMP_C)
      if not tight:
        break
      if waiting_since is None:
        waiting_since = self.clock()
      elif self.clock() - waiting_since > GENTLE_GIVE_UP_S:
        raise InterruptedError("busy")
      self.update(step="waiting", mem_mb=None if mem is None else round(mem), temp_c=temp)
      self.sleep(GENTLE_WAIT_S)
      state = self._state()
    if waiting_since is not None:
      self.update(step=step)
    self.last_mark = self.clock()

  def _state(self) -> str:
    try:
      state = self.drive_state()
    except Exception:
      state = DRIVING
    if state != self.status.get("drive_state"):
      self.update(drive_state=state, pace="gentle" if state == DRIVING else "full")
    return state


def pick_route(root: str, job: Job, route: str | None, exclude_current: bool = False):
  """(route to analyse, its segments, newest finished route); exclude_current leaves out the route being recorded."""
  from openpilot.selfdrive.carrot.tuning_advice import extract
  routes = extract.list_routes(root)
  if exclude_current and routes:
    routes.pop(next(iter(routes)))
  newest = next(iter(routes), None)
  if route:
    return route, routes.get(route, []), newest
  for r in list(routes)[:ROUTES_TO_SEARCH]:
    seconds = 0.0
    for n in routes[r]:
      job.checkpoint()
      q = extract.segment_file(root, r, n, "qlog")
      if q is not None:
        seconds += extract.qlog_lateral_seconds(q)
      if seconds >= MIN_ROUTE_LAT_SECONDS:
        return r, routes[r], newest
  return None, [], newest


def simulate(drive, settings: dict, job: Job):
  """Replay every window with the drive's setting and each one-step change. (aggregate, info) or (None, info)."""
  from openpilot.selfdrive.carrot.tuning_advice import advise, sim
  from openpilot.selfdrive.carrot.tuning_advice.extract import C
  info = {"error": None, "windows": {}, "seconds": None, "plant_rms": None}
  if drive.car_params is None:
    info["error"] = "no_car_params"
    return None, info
  segs = [(s.a, C) for s in drive.segments]
  job.checkpoint()
  curve = sim.fit_plant_curve(segs)
  job.checkpoint()
  if curve is None:
    info["error"] = "plant"
    return None, info
  try:
    simulator = sim.Simulator(drive.car_params, drive.params, curve)
  except NotImplementedError:
    info["error"] = "unsupported_car"
    return None, info
  W = sim.windows(segs)
  info["windows"] = {k: sum(1 for w in W if w[3] == k) for k in ("curve", "turn", "straight")}
  from openpilot.selfdrive.controls.lib.latcontrol_torque import FrogPilotWindupPID
  p_filter = isinstance(simulator.controller({})[0].pid, FrogPilotWindupPID)   # the only PID that has the P filter
  cands = advise.candidate_list(settings, p_filter)
  lal, jerk = advise.lal_value(settings["LateralAccelLimit"]), advise.jerk_value(settings["LatJerkMax"])
  t0 = time.time()
  per_window, rms = [], []
  skipped = 0
  total = max(len(W) * (len(cands) + 2), 1)
  done = 0
  import numpy as np
  for si, s, e, kind in W:
    seg = drive.segments[si]
    job.checkpoint()
    ref = simulator.run(seg.a, seg.models, s, e, {}, lal, jerk, checkpoint=job.checkpoint)
    done += 2
    if len(ref) == 0:
      continue
    # the replay feeds the model's curvature through controlsd's filter and clip; lane mode, the minimum-distance clamp
    # or anything else it does not reproduce shows up as a desired curvature unlike the logged one: leave that window out
    logged = seg.a[s:e, C["desCurv"]] * seg.a[s:e, C["v"]] ** 2
    if len(logged) == len(ref) and np.isfinite(logged).all():
      if np.sqrt(np.mean((ref[:, 2] - logged) ** 2)) > max(DESIRED_MISMATCH_ABS, DESIRED_MISMATCH_REL * np.sqrt(np.mean(logged ** 2))):
        skipped += 1
        done += len(cands)
        continue
    rms.append(float(np.sqrt(np.mean((ref[:, 3] - ref[:, 5]) ** 2))))
    noise = simulator.run(seg.a, seg.models, s, e, {}, lal, jerk, friction=sim.PLANT_FRICTION * advise.NOISE_FRICTION_SCALE,
                          checkpoint=job.checkpoint)
    w = {"kind": kind, "ref": sim.window_metrics(ref, ref, kind), "noise": sim.window_metrics(noise, ref, kind), "cand": {}}
    for key, frm, to in cands:
      job.checkpoint()
      over = {key: to}
      c_lal = advise.lal_value(to) if key == "LateralAccelLimit" else lal
      r = simulator.run(seg.a, seg.models, s, e, {} if key == "LateralAccelLimit" else over, c_lal, jerk,
                        checkpoint=job.checkpoint)
      w["cand"][advise.label(key, frm, to)] = sim.window_metrics(r, ref, kind)
      done += 1
      job.update(step="simulating", progress=50 + int(49 * done / total))
    per_window.append(w)
  info["seconds"] = round(time.time() - t0, 1)
  info["plant_rms"] = round(float(np.median(rms)), 3) if rms else None
  info["skipped_windows"] = skipped
  if not per_window:
    info["error"] = "mismatch" if skipped else "no_windows"
    return None, info
  return advise.aggregate(per_window), info


def analyse(root: str, state_dir: str, route: str | None, drive_state, exclude_current: bool = False,
            job: Job | None = None) -> dict:
  from openpilot.selfdrive.carrot.tuning_advice import advise, extract, metrics
  job = job or Job(state_dir, drive_state)
  job.update()
  job.checkpoint()
  route, numbers, newest = pick_route(root, job, route, exclude_current)
  base = {"version": VERSION, "newest_route": newest, "analyzed_at": int(time.time())}
  if route is None:
    result = dict(base, route=None, recommendation={"status": "no_drive", "candidates": [], "notes": []})
    write_json(os.path.join(state_dir, RESULT_NAME), _finite(result))
    job.update(state="done", step="done", progress=100)
    return result
  job.update(step="reading", route=route, progress=1)
  drive = extract.load_drive(route, numbers, root, checkpoint=job.checkpoint,
                             progress=lambda k, n: job.update(step="reading", progress=1 + int(48 * k / max(n, 1)),
                                                              segment=k + 1, segments=n))
  tuning = drive.tuning()
  settings = advise.drive_settings(tuning)
  segs = [(s.a, extract.C) for s in drive.segments]
  m = metrics.drive_metrics(segs, advise.lal_value(settings["LateralAccelLimit"]))
  job.update(step="simulating", progress=50)
  sim_result, sim_info = (None, {"error": "lat_minutes"})
  if m["lat_minutes"] >= advise.MIN_LAT_MINUTES:
    try:
      sim_result, sim_info = simulate(drive, settings, job)
    except InterruptedError:
      raise
    except Exception as e:  # the advice still shows the drive's own figures
      sim_info = {"error": f"exception:{type(e).__name__}"}
      traceback.print_exc()
  rec = advise.recommend(m, sim_result, settings, sim_info.get("error"))
  result = dict(base, route=route, car=drive.car_fingerprint, drive_start=drive.wall_start, settings=settings,
                segments=len(drive.segments), partial_segments=drive.partial, metrics=m,
                simulation=dict(sim_info, reference=(sim_result or {}).get("_ref")), recommendation=rec)
  history_path = os.path.join(state_dir, HISTORY_NAME)
  history = read_json(history_path, [])
  if not isinstance(history, list):
    history = []
  result["change_review"] = advise.change_review(result, history)
  history = [h for h in history if isinstance(h, dict) and h.get("route") != route]
  history.append({"route": route, "drive_start": drive.wall_start, "settings": settings, "metrics": m,
                  "analyzed_at": result["analyzed_at"]})
  write_json(history_path, _finite(history[-HISTORY_LIMIT:]))
  write_json(os.path.join(state_dir, RESULT_NAME), _finite(result))
  job.update(state="done", step="done", progress=100, route=route, seconds=round(time.time() - job.started, 1),
             max_rss_mb=max_rss_mb())
  return result


def max_rss_mb() -> float | None:
  try:
    import resource
    rss = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    return round(rss / 1024 if sys.platform.startswith("linux") else rss / 2 ** 20, 1)
  except Exception:
    return None


def acquire_lock(state_dir: str):
  """One analysis at a time (also across a carrot_server restart): an exclusive lock held for the process's life, with
  its pid in the file. None if another runner holds it."""
  import fcntl
  fd = os.open(os.path.join(state_dir, LOCK_NAME), os.O_RDWR | os.O_CREAT, 0o644)
  try:
    fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
  except OSError:
    os.close(fd)
    return None
  os.ftruncate(fd, 0)
  os.write(fd, str(os.getpid()).encode())
  return fd


def lock_holder(state_dir: str) -> int | None:
  """pid of the runner holding the lock, or None when no analysis runs."""
  import fcntl
  path = os.path.join(state_dir, LOCK_NAME)
  try:
    fd = os.open(path, os.O_RDONLY)
  except OSError:
    return None
  try:
    try:
      fcntl.flock(fd, fcntl.LOCK_SH | fcntl.LOCK_NB)
    except OSError:
      try:
        return int(os.pread(fd, 32, 0).decode().strip() or 0) or None
      except ValueError:
        return None
    fcntl.flock(fd, fcntl.LOCK_UN)
    return None
  finally:
    os.close(fd)


def main(argv=None) -> int:
  ap = argparse.ArgumentParser()
  ap.add_argument("--route")
  ap.add_argument("--root", default=None)
  ap.add_argument("--state-dir", default=STATE_DIR)
  args = ap.parse_args(argv)
  os.environ["CARROT_TUNING_ADVICE_PROCESS"] = "1"
  try:
    os.nice(19)
  except OSError:
    pass
  if sys.platform.startswith("linux") and os.uname().machine == "aarch64":
    try:  # idle I/O class: ioprio_set(IOPRIO_WHO_PROCESS, pid, IOPRIO_CLASS_IDLE), syscall 30 on arm64
      import ctypes
      ctypes.CDLL(None).syscall(30, 1, os.getpid(), 3 << 13)
    except Exception:
      pass
  from openpilot.common.params import Params
  from openpilot.selfdrive.carrot.tuning_advice import extract
  drive_state = make_drive_state(Params())   # bound before the simulator swaps the class for its read-only stand-in

  os.makedirs(args.state_dir, exist_ok=True)
  job_status = os.path.join(args.state_dir, STATUS_NAME)
  lock = acquire_lock(args.state_dir)
  if lock is None:
    print("another tuning analysis is running", file=sys.stderr)
    return 4
  try:
    # with the ignition on the newest route is the one being recorded
    analyse(args.root or extract.REALDATA, args.state_dir, args.route, drive_state, exclude_current=drive_state() != OFFROAD)
    return 0
  except InterruptedError as e:
    prev = read_json(job_status, {})
    write_json(job_status, _finite(dict(prev, state="interrupted", step=str(e) or "busy", updated_at=int(time.time()))))
    return 3
  except Exception as e:
    prev = read_json(job_status, {})
    write_json(job_status, _finite(dict(prev, state="error", error=f"{type(e).__name__}: {e}"[:300], updated_at=int(time.time()))))
    traceback.print_exc()
    return 1


if __name__ == "__main__":
  sys.exit(main())
