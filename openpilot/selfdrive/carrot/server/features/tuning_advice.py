"""Carrot web: tuning advice from the last drive (openpilot/selfdrive/carrot/tuning_advice/).

The analysis runs as its own low-priority process (runner.py) a minute after a new drive ends with the comma still on
(permanent power), or a minute into the next drive for a comma that powers off with the car (then it analyses the drive
before the one being recorded), or when the card's button asks for it. It is not stopped when the ignition comes on:
the runner keeps full pace in P at a standstill and rests between steps while driving.
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

LOOP_INTERVAL = 2.0
AUTO_CHECK_INTERVAL = 30.0      # looking for a new drive
PARKED_BEFORE_START = 60.0      # seconds offroad before a new drive is analysed on its own
ONROAD_BEFORE_START = 60.0      # seconds after the ignition comes on (boot and model start settle first)
RUN_TIMEOUT = 60 * 60
TUNED_PARAMS = tuple(sorted({c[0] for c in CANDIDATES} | {"LateralTorqueCustom", "LatJerkMax"}))
LOG_NAME = "tuning_advice.log"


def _path(name: str) -> str:
  return os.path.join(CARROT_STATE_DIR, name)


def _params() -> Any | None:
  return Params() if HAS_PARAMS and Params is not None else None


def _parked(params: Any | None) -> bool:
  """True only when the comma says the ignition is off and nothing is engaged; any read failure is not parked."""
  if params is None:
    return False
  try:
    return not params.get_bool("IsOnroad") and not params.get_bool("IsEngaged")
  except Exception:
    return False


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
  """The one analysis process. The runner holds a lock file with its pid, so a runner left over from before a
  carrot_server restart still counts as running and can still be stopped."""
  def __init__(self):
    self.proc: subprocess.Popen | None = None
    self.started = 0.0
    self.offroad_since: float | None = None
    self.onroad_since: float | None = None
    self.last_onroad: bool | None = None
    self.auto_tried: str | None = None   # newest route an automatic run already started for (per ignition state)
    self.next_check = 0.0

  def own_running(self) -> bool:
    return self.proc is not None and self.proc.poll() is None

  def other_pid(self) -> int | None:
    """pid of a runner this server did not start, verified to be that runner (None if unknown)."""
    return None if self.own_running() else runner.lock_holder(CARROT_STATE_DIR)

  def running(self) -> bool:
    return self.own_running() or runner.lock_held(CARROT_STATE_DIR)

  def elapsed(self) -> float:
    if self.own_running():
      return time.monotonic() - self.started
    started = runner.read_json(_path(runner.STATUS_NAME), {}).get("started_at")
    return time.time() - started if isinstance(started, (int, float)) else 0.0

  def start(self) -> bool:
    if self.running():
      return False
    os.makedirs(CARROT_STATE_DIR, exist_ok=True)
    env = dict(os.environ, CARROT_TUNING_ADVICE_PROCESS="1")
    log = open(_path(LOG_NAME), "w", encoding="utf-8")
    try:
      self.proc = subprocess.Popen([sys.executable, "-m", "openpilot.selfdrive.carrot.tuning_advice.runner",
                                    "--state-dir", CARROT_STATE_DIR],
                                   stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True)
    finally:
      log.close()
    self.started = time.monotonic()
    return True

  def stop(self) -> None:
    pid = self.proc.pid if self.own_running() else self.other_pid()
    if pid is None:
      return
    for sig, wait in ((signal.SIGTERM, 2.0), (signal.SIGKILL, 1.0)):
      try:
        os.killpg(pid, sig)          # the runner leads its own session (start_new_session)
      except ProcessLookupError:
        break
      except Exception:
        try:
          os.kill(pid, sig)
        except Exception:
          break
      deadline = time.monotonic() + wait
      while time.monotonic() < deadline and self.running():
        time.sleep(0.05)
      if not self.running():
        break
    status = runner.read_json(_path(runner.STATUS_NAME), {})
    if status.get("state") == "running":
      runner.write_json(_path(runner.STATUS_NAME), dict(status, state="interrupted", step="stopped", updated_at=int(time.time())))


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
    "current": current,
  }


async def api_status(_request: web.Request) -> web.Response:
  return web.json_response(build_payload(_params()))


async def api_run(_request: web.Request) -> web.Response:
  params = _params()
  JOB.start()   # with the ignition on the runner analyses the drive before this one and paces itself
  return web.json_response(build_payload(params))


def tick(params: Any | None, now: float) -> None:
  """One pass of the loop: a minute after the ignition goes off or on, analyse the newest finished drive if it has no
  advice yet (one automatic run per drive; the card's button runs it again). A run is never stopped for the ignition,
  only after an hour."""
  onroad = _bool(params, "IsOnroad")
  if onroad != JOB.last_onroad:
    JOB.last_onroad = onroad
    JOB.auto_tried = None          # a run put off for memory or heat is tried again after the ignition changes
  if onroad:
    JOB.offroad_since = None
    if JOB.onroad_since is None:
      JOB.onroad_since = now
    wait, delay = JOB.onroad_since, ONROAD_BEFORE_START
  else:
    JOB.onroad_since = None
    if JOB.offroad_since is None:
      JOB.offroad_since = now
    wait, delay = JOB.offroad_since, PARKED_BEFORE_START
  if JOB.running():
    if JOB.elapsed() > RUN_TIMEOUT:
      JOB.stop()
    return
  if params is None or now - wait < delay or now < JOB.next_check:
    return
  JOB.next_check = now + AUTO_CHECK_INTERVAL
  target = _target_route(onroad)
  if needs_run(read_result(), target) and JOB.auto_tried != target:
    JOB.auto_tried = target
    JOB.start()


async def tuning_advice_loop(app: web.Application) -> None:
  while True:
    try:
      tick(app.get("params"), time.monotonic())
    except asyncio.CancelledError:
      raise
    except Exception:
      pass
    await asyncio.sleep(LOOP_INTERVAL)


async def api_apply(request: web.Request) -> web.Response:
  """Write the recommended value: parked only, only the current recommendation, only while it is about the newest drive.
  The check is here, right before the write, not only in the browser (the ignition may come on while the confirm
  dialog is open)."""
  try:
    body = await request.json()
  except Exception:
    return web.json_response({"ok": False, "error": "invalid json"}, status=400)
  params = _params()
  if not _parked(params):
    return web.json_response({"ok": False, "error": "park and turn the ignition off first"}, status=409)
  result = read_result()
  rec = (result or {}).get("recommendation") or {}
  name = body.get("name") if isinstance(body, dict) else None
  value = body.get("value") if isinstance(body, dict) else None
  if rec.get("status") != "change" or name != rec.get("param") or value != rec.get("to"):
    return web.json_response({"ok": False, "error": "that is not the current recommendation"}, status=409)
  if JOB.running() or needs_run(result, _target_route(False)):
    return web.json_response({"ok": False, "error": "the newest drive is not analysed yet"}, status=409)
  from ..services.param_changes import append_param_change
  from ..services.params import set_param_value
  from ..services.settings import get_settings_cached
  try:
    setting = get_settings_cached()[2].get(name)
  except Exception:
    setting = None
  previous = get_param_values([name], {name: None}).get(name)
  if not _parked(params):   # again right before the write: the reads above take time
    return web.json_response({"ok": False, "error": "park and turn the ignition off first"}, status=409)
  try:
    set_param_value(name, int(value), setting)
  except Exception as e:
    return web.json_response({"ok": False, "error": str(e)}, status=500)
  if previous != value:
    append_param_change(name, previous, int(value), source="tuning_advice", engaged=False)
  return web.json_response(build_payload(params))


def register(app: web.Application) -> None:
  app.router.add_get("/api/tuning_advice", api_status)
  app.router.add_post("/api/tuning_advice/run", api_run)
  app.router.add_post("/api/tuning_advice/apply", api_apply)
