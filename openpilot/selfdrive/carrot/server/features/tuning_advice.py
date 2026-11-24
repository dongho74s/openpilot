"""Carrot web: tuning advice from the last drive (openpilot/selfdrive/carrot/tuning_advice/).

The analysis runs as its own low-priority process (runner.py):
- parked for a minute after a new drive (permanent power): a parked run, killed the moment the car is switched on;
- otherwise two minutes into the next drive (the comma lost power with the ignition): a gentle run of the drive before
  the one being recorded, which rests between steps and waits while memory or temperature is tight;
- or when the card's button asks for it (gentle while driving).
"""
from __future__ import annotations

import asyncio
import os
import signal
import subprocess
import sys
import time
from typing import Any

from aiohttp import web

from openpilot.selfdrive.carrot.tuning_advice import runner
from openpilot.selfdrive.carrot.tuning_advice.advise import CANDIDATES
from openpilot.selfdrive.carrot.tuning_advice.extract import REALDATA, list_routes

from ..config import CARROT_STATE_DIR
from ..services.params import HAS_PARAMS, Params, get_param_values

LOOP_INTERVAL = 2.0             # the ignition check that kills a running analysis
AUTO_CHECK_INTERVAL = 30.0      # looking for a new drive
PARKED_BEFORE_START = 60.0      # seconds offroad before a new drive is analysed on its own
DRIVING_BEFORE_START = 120.0    # seconds into the next drive before the previous one is analysed gently
RUN_TIMEOUT = 30 * 60
GENTLE_RUN_TIMEOUT = 60 * 60
TUNED_PARAMS = tuple(sorted({c[0] for c in CANDIDATES} | {"LateralTorqueCustom", "LatJerkMax"}))
LOG_NAME = "tuning_advice.log"


def _path(name: str) -> str:
  return os.path.join(CARROT_STATE_DIR, name)


def _params() -> Any | None:
  return Params() if HAS_PARAMS and Params is not None else None


def _bool(params: Any | None, key: str) -> bool:
  if params is None:
    return False
  try:
    return bool(params.get_bool(key))
  except Exception:
    return False


def _target_route(onroad: bool) -> str | None:
  """The newest finished drive: while driving the newest route is the one being recorded."""
  routes = list(list_routes(REALDATA))
  if onroad:
    routes = routes[1:]
  return routes[0] if routes else None


class Job:
  """The one analysis process this server may run."""
  def __init__(self):
    self.proc: subprocess.Popen | None = None
    self.gentle = False
    self.started = 0.0
    self.offroad_since: float | None = None
    self.onroad_since: float | None = None
    self.auto_tried: str | None = None   # newest route an automatic run already started for
    self.next_check = 0.0

  def running(self) -> bool:
    return self.proc is not None and self.proc.poll() is None

  def start(self, gentle: bool = False) -> bool:
    if self.running():
      return False
    os.makedirs(CARROT_STATE_DIR, exist_ok=True)
    env = dict(os.environ, CARROT_TUNING_ADVICE_PROCESS="1")
    log = open(_path(LOG_NAME), "w", encoding="utf-8")
    try:
      self.proc = subprocess.Popen([sys.executable, "-m", "openpilot.selfdrive.carrot.tuning_advice.runner",
                                    "--state-dir", CARROT_STATE_DIR] + (["--gentle"] if gentle else []),
                                   stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True)
    finally:
      log.close()
    self.gentle = gentle
    self.started = time.monotonic()
    return True

  def stop(self) -> None:
    if not self.running():
      return
    try:
      os.killpg(self.proc.pid, signal.SIGTERM)
      self.proc.wait(timeout=2)
    except Exception:
      try:
        os.killpg(self.proc.pid, signal.SIGKILL)
      except Exception:
        pass
    status = runner.read_json(_path(runner.STATUS_NAME), {})
    if status.get("state") == "running":
      runner.write_json(_path(runner.STATUS_NAME), dict(status, state="interrupted", step="onroad", updated_at=int(time.time())))


JOB = Job()


def needs_run(result: dict | None, target: str | None) -> bool:
  return target is not None and (result is None or result.get("newest_route") != target)


def read_result() -> dict | None:
  result = runner.read_json(_path(runner.RESULT_NAME), None)
  return result if isinstance(result, dict) and result.get("version") == runner.VERSION else None


def build_payload(params: Any | None = None) -> dict[str, Any]:
  result = read_result()
  status = runner.read_json(_path(runner.STATUS_NAME), {})
  running = JOB.running()
  if not running and status.get("state") == "running":
    status = dict(status, state="interrupted")   # the process died without saying so
  onroad = _bool(params, "IsOnroad")
  target = _target_route(onroad)
  current = {}
  try:
    current = get_param_values(list(TUNED_PARAMS), {})
  except Exception:
    pass
  return {
    "ok": True,
    "onroad": onroad,
    "running": running,
    "status": status,
    "result": result,
    "newest_route": target,
    "needs_run": needs_run(result, target),
    "can_run": not running,
    "gentle": running and JOB.gentle,
    "current": current,
  }


async def api_status(_request: web.Request) -> web.Response:
  return web.json_response(build_payload(_params()))


async def api_run(_request: web.Request) -> web.Response:
  params = _params()
  JOB.start(gentle=_bool(params, "IsOnroad"))   # while driving: the drive before this one, gently
  return web.json_response(build_payload(params))


def tick(params: Any | None, now: float) -> None:
  """One pass of the loop. Parked for a minute after a new drive: a parked run. Switching the car on kills a parked run;
  two minutes into a drive, a gentle run of the previous drive if it has no advice yet. One automatic run per drive
  (the card's button runs it again)."""
  onroad = _bool(params, "IsOnroad")
  if onroad:
    JOB.offroad_since = None
    if JOB.onroad_since is None:
      JOB.onroad_since = now
    if JOB.running() and not JOB.gentle:
      JOB.stop()
      JOB.auto_tried = None    # the drive it was analysing gets its gentle run in this drive
      return
    wait, delay = JOB.onroad_since, DRIVING_BEFORE_START
  else:
    JOB.onroad_since = None
    if JOB.offroad_since is None:
      JOB.offroad_since = now
    wait, delay = JOB.offroad_since, PARKED_BEFORE_START
  if JOB.running():
    if now - JOB.started > (GENTLE_RUN_TIMEOUT if JOB.gentle else RUN_TIMEOUT):
      JOB.stop()
    return
  if params is None or now - wait < delay or now < JOB.next_check:
    return
  JOB.next_check = now + AUTO_CHECK_INTERVAL
  target = _target_route(onroad)
  if needs_run(read_result(), target) and JOB.auto_tried != target:
    JOB.auto_tried = target
    JOB.start(gentle=onroad)


async def tuning_advice_loop(app: web.Application) -> None:
  while True:
    try:
      tick(app.get("params"), time.monotonic())
    except asyncio.CancelledError:
      raise
    except Exception:
      pass
    await asyncio.sleep(LOOP_INTERVAL)


def register(app: web.Application) -> None:
  app.router.add_get("/api/tuning_advice", api_status)
  app.router.add_post("/api/tuning_advice/run", api_run)
