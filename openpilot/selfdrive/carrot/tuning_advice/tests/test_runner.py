import json

import pytest

from openpilot.selfdrive.carrot.tuning_advice import runner as R


class Clock:
  def __init__(self):
    self.t = 0.0
    self.slept = []

  def __call__(self):
    return self.t

  def sleep(self, s):
    self.slept.append(s)
    self.t += s


def make_job(tmp_path, readings, state=R.DRIVING):
  clock = Clock()
  it = iter(readings)
  last = [readings[-1]]
  where = [state]

  def read():
    try:
      last[0] = next(it)
    except StopIteration:
      pass
    return last[0]
  job = R.Job(str(tmp_path), lambda: where[0], sleep=clock.sleep, clock=clock, read_resources=read)
  return job, clock, where


def status(tmp_path):
  return json.loads((tmp_path / R.STATUS_NAME).read_text())


def test_driving_rests_as_long_as_it_worked(tmp_path):
  job, clock, _ = make_job(tmp_path, [(2000, 50.0)])
  clock.t += 2.0                                   # a 2 s step
  job.checkpoint()
  assert clock.slept == [2.0]
  assert status(tmp_path)["pace"] == "gentle" and status(tmp_path)["drive_state"] == "driving"
  clock.t += 30.0                                  # a long step: the rest is capped
  job.checkpoint()
  assert clock.slept[-1] == R.GENTLE_MAX_REST_S


def test_parked_in_p_and_ignition_off_keep_full_pace(tmp_path):
  job, clock, where = make_job(tmp_path, [(2000, 50.0)], state=R.PARKED)
  clock.t += 2.0
  job.checkpoint()
  assert clock.slept == [] and status(tmp_path)["pace"] == "full"
  where[0] = R.DRIVING                             # drives off: the same run goes on, gently
  clock.t += 2.0
  job.checkpoint()
  assert clock.slept == [2.0]
  where[0] = R.OFFROAD
  clock.t += 2.0
  job.checkpoint()
  assert clock.slept == [2.0] and status(tmp_path)["drive_state"] == "offroad"


def test_ignition_on_waits_while_memory_or_temperature_is_tight(tmp_path):
  job, clock, _ = make_job(tmp_path, [(500, 50.0), (2000, 80.0), (2000, 50.0)], state=R.PARKED)
  job.update(step="reading", progress=30)
  job.checkpoint()
  assert clock.slept == [R.GENTLE_WAIT_S, R.GENTLE_WAIT_S]
  assert status(tmp_path)["step"] == "reading" and status(tmp_path)["progress"] == 30   # back to its step after waiting


def test_ignition_off_does_not_wait_for_memory(tmp_path):
  job, clock, _ = make_job(tmp_path, [(500, 90.0)], state=R.OFFROAD)
  job.checkpoint()
  assert clock.slept == []


def test_gives_up_when_it_stays_tight(tmp_path):
  job, clock, _ = make_job(tmp_path, [(500, 50.0)])
  with pytest.raises(InterruptedError, match="busy"):
    job.checkpoint()
  assert clock.t > R.GENTLE_GIVE_UP_S


def test_drive_state_from_ignition_and_gear(monkeypatch):
  import sys
  import types
  msgs = []
  fake = types.ModuleType("openpilot.cereal.messaging")
  fake.sub_sock = lambda name, conflate=False: object()
  fake.recv_one_or_none = lambda sock: msgs.pop(0) if msgs else None
  monkeypatch.setitem(sys.modules, "openpilot.cereal.messaging", fake)
  import openpilot.cereal as cereal_pkg
  monkeypatch.setattr(cereal_pkg, "messaging", fake, raising=False)
  flags = {"IsOnroad": False}
  params = types.SimpleNamespace(get_bool=lambda k: flags[k])
  state = R.make_drive_state(params)
  assert state() == R.OFFROAD
  flags["IsOnroad"] = True
  assert state() == R.DRIVING                      # no carState yet
  import time
  cs = lambda gear, still: types.SimpleNamespace(valid=True, logMonoTime=int(time.monotonic() * 1e9),
                                                 carState=types.SimpleNamespace(gearShifter=gear, standstill=still))
  msgs.append(cs("park", True))
  assert state() == R.PARKED
  assert state() == R.PARKED                       # the latest message still counts
  msgs.append(cs("drive", False))
  assert state() == R.DRIVING
  msgs.append(cs("park", False))                   # P but still rolling
  assert state() == R.DRIVING


def test_while_driving_the_route_being_recorded_is_left_out(tmp_path, monkeypatch):
  for name in ("00000251--ec2aec2b03--0", "00000252--3d51466b37--0", "00000253--121bd0b631--0"):
    (tmp_path / name).mkdir()
  monkeypatch.setattr("openpilot.selfdrive.carrot.tuning_advice.extract.qlog_lateral_seconds", lambda p: 600.0)
  for name in ("00000252--3d51466b37--0", "00000253--121bd0b631--0"):
    (tmp_path / name / "qlog.zst").write_bytes(b"")
  job = R.Job(str(tmp_path), lambda: False)
  assert R.pick_route(str(tmp_path), job, None)[0] == "00000253--121bd0b631"
  route, segs, newest = R.pick_route(str(tmp_path), job, None, exclude_current=True)
  assert (route, segs, newest) == ("00000252--3d51466b37", [0], "00000252--3d51466b37")


def test_one_runner_at_a_time(tmp_path, monkeypatch):
  import os
  monkeypatch.setattr(R, "is_runner", lambda pid: pid == os.getpid())
  assert R.lock_holder(str(tmp_path)) is None and not R.lock_held(str(tmp_path))
  fd = R.acquire_lock(str(tmp_path))
  assert fd is not None
  assert R.lock_held(str(tmp_path)) and R.lock_holder(str(tmp_path)) == os.getpid()
  assert R.acquire_lock(str(tmp_path), attempts=2) is None     # a second runner gives up after its retries
  os.close(fd)
  assert R.lock_holder(str(tmp_path)) is None and not R.lock_held(str(tmp_path))


def test_a_status_check_does_not_stop_a_runner_from_starting(tmp_path):
  import fcntl
  import os
  import threading
  path = tmp_path / R.LOCK_NAME
  path.write_text("")
  fd = os.open(path, os.O_RDONLY)
  fcntl.flock(fd, fcntl.LOCK_SH)                   # a status check holding the lock shared for a moment
  threading.Timer(0.12, lambda: (fcntl.flock(fd, fcntl.LOCK_UN), os.close(fd))).start()
  got = R.acquire_lock(str(tmp_path))
  assert got is not None
  os.close(got)


def test_a_stale_or_invalid_park_message_is_not_parked(monkeypatch):
  import sys
  import time
  import types
  msgs = []
  fake = types.ModuleType("openpilot.cereal.messaging")
  fake.sub_sock = lambda name, conflate=False: object()
  fake.recv_one_or_none = lambda sock: msgs.pop(0) if msgs else None
  monkeypatch.setitem(sys.modules, "openpilot.cereal.messaging", fake)
  import openpilot.cereal as cereal_pkg
  monkeypatch.setattr(cereal_pkg, "messaging", fake, raising=False)
  state = R.make_drive_state(types.SimpleNamespace(get_bool=lambda k: True))
  def park(age, valid=True):
    return types.SimpleNamespace(valid=valid, logMonoTime=int((time.monotonic() - age) * 1e9),
                                 carState=types.SimpleNamespace(gearShifter="park", standstill=True))
  msgs.append(park(90.0))                          # read now, but written 90 s ago
  assert state() == R.DRIVING
  msgs.append(park(0.0, valid=False))
  assert state() == R.DRIVING
  msgs.append(park(0.0))
  assert state() == R.PARKED


def test_memory_for_the_next_step_is_checked_before_it_is_taken(tmp_path):
  job, clock, _ = make_job(tmp_path, [(700, 50.0), (900, 50.0)], state=R.PARKED)
  job.checkpoint(need_mb=250)                      # 700 < 600 + 250: wait, then 900 >= 850
  assert clock.slept == [R.GENTLE_WAIT_S]


def test_checks_are_spaced_but_the_rest_matches_the_work(tmp_path):
  job, clock, _ = make_job(tmp_path, [(2000, 50.0)])
  job.checkpoint()                                 # first check always runs
  clock.t += 0.1
  job.checkpoint()                                 # too soon: nothing
  assert clock.slept == []
  clock.t += 0.3                                   # 0.4 s of work since the last check
  job.checkpoint()
  assert clock.slept == [pytest.approx(0.4)]
