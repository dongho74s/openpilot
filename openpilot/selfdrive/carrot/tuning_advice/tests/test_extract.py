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


def read_until_cut(path):
  got = []
  with pytest.raises(E.LogCut):
    for e in E.read_events(str(path)):
      got.append(e.logMonoTime)
  return got


def test_a_log_cut_short_yields_what_was_written_then_says_so(tmp_path):
  raw = log_bytes()
  path = tmp_path / "rlog"
  path.write_bytes(raw[:len(raw) - 7])            # last message incomplete (power cut)
  assert read_until_cut(path) == list(range(299))
  zst = tmp_path / "rlog.zst"
  zst.write_bytes(zstandard.ZstdCompressor().compress(raw[:len(raw) - 7]))
  assert read_until_cut(zst) == list(range(299))


def test_a_damaged_header_keeps_the_messages_before_it(tmp_path):
  raw = log_bytes(10)
  path = tmp_path / "rlog"
  path.write_bytes(raw + (5000).to_bytes(4, "little") + b"\0" * 4)   # 5001 segments: not a log message
  assert read_until_cut(path) == list(range(10))


def test_a_huge_declared_size_stops_without_buffering_it(tmp_path, monkeypatch):
  monkeypatch.setattr(E, "READ_CHUNK", 4096)
  raw = log_bytes(10)
  path = tmp_path / "rlog"
  bogus = (0).to_bytes(4, "little") + (1 << 27).to_bytes(4, "little")  # one segment of 1 GiB declared
  path.write_bytes(raw + bogus + b"\0" * (1 << 20))
  assert read_until_cut(path) == list(range(10))


def test_a_cut_segment_is_marked_partial_with_its_rows_kept(tmp_path):
  raw = log_bytes()
  path = tmp_path / "rlog"
  path.write_bytes(raw[:len(raw) - 7])
  _seg, _p, _cp, _w, cut = E.read_segment(str(path), "r", 0)
  assert cut is True
