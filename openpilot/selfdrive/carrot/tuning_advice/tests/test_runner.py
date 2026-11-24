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


def gentle_job(tmp_path, readings):
  clock = Clock()
  it = iter(readings)
  last = [readings[-1]]

  def read():
    try:
      last[0] = next(it)
    except StopIteration:
      pass
    return last[0]
  job = R.Job(str(tmp_path), lambda: True, gentle=True, sleep=clock.sleep, clock=clock, read_resources=read)
  return job, clock


def test_parked_run_stops_when_the_car_starts(tmp_path):
  onroad = [False]
  job = R.Job(str(tmp_path), lambda: onroad[0])
  assert not job.stop_requested()
  onroad[0] = True
  assert job.stop_requested()


def test_gentle_run_rests_as_long_as_it_worked_and_ignores_the_ignition(tmp_path):
  job, clock = gentle_job(tmp_path, [(2000, 50.0)])
  clock.t += 2.0                                   # a 2 s step
  assert not job.stop_requested()
  assert clock.slept == [2.0]
  clock.t += 30.0                                  # a long step: the rest is capped
  job.stop_requested()
  assert clock.slept[-1] == R.GENTLE_MAX_REST_S


def test_gentle_run_waits_while_memory_or_temperature_is_tight(tmp_path):
  job, clock = gentle_job(tmp_path, [(500, 50.0), (2000, 80.0), (2000, 50.0)])
  job.update(step="reading", progress=30)
  assert not job.stop_requested()
  assert clock.slept == [R.GENTLE_WAIT_S, R.GENTLE_WAIT_S]
  status = json.loads((tmp_path / R.STATUS_NAME).read_text())
  assert status["step"] == "reading" and status["progress"] == 30   # back to its step after waiting


def test_gentle_run_gives_up_when_it_stays_tight(tmp_path):
  job, clock = gentle_job(tmp_path, [(500, 50.0)])
  with pytest.raises(InterruptedError, match="busy"):
    job.stop_requested()
  assert clock.t > R.GENTLE_GIVE_UP_S


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
