import pytest

from openpilot.selfdrive.carrot.tuning_advice import extract as E

zstandard = pytest.importorskip("zstandard")


def log_bytes(n=300):
  from openpilot.cereal import log
  out = []
  for i in range(n):
    e = log.Event.new_message(logMonoTime=i, valid=True)
    cs = e.init("carState")
    cs.vEgo = float(i)
    out.append(e.to_bytes())
  return b"".join(out)


def test_logs_are_read_a_chunk_at_a_time_across_message_boundaries(tmp_path, monkeypatch):
  monkeypatch.setattr(E, "READ_CHUNK", 97)        # tiny chunks: messages split across reads
  raw = log_bytes()
  path = tmp_path / "rlog.zst"
  path.write_bytes(zstandard.ZstdCompressor().compress(raw))
  events = list(E.read_events(str(path)))
  assert [e.logMonoTime for e in events] == list(range(300))
  assert events[-1].carState.vEgo == 299.0
  plain = tmp_path / "rlog"
  plain.write_bytes(raw)
  assert len(list(E.read_events(str(plain)))) == 300


def test_a_log_cut_short_yields_what_was_written(tmp_path):
  raw = log_bytes()
  path = tmp_path / "rlog"
  path.write_bytes(raw[:len(raw) - 7])            # last message incomplete (power cut)
  assert [e.logMonoTime for e in E.read_events(str(path))] == list(range(299))
