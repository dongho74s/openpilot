import asyncio
import json
import subprocess
from pathlib import Path

import pytest

from openpilot.selfdrive.carrot.server.services import git_status
from openpilot.selfdrive.carrot.server.services.update_notes import (
  MAX_ITEMS,
  NOTES_PATH,
  build_notes,
  parse_log,
  parse_notes,
)

REPO_NOTES = Path(__file__).resolve().parents[2] / "update_notes.json"


def test_notes_file_in_the_repo_is_valid_and_every_entry_is_used():
  data = json.loads(REPO_NOTES.read_text(encoding="utf-8"))
  assert data["version"] == 1
  assert len(parse_notes(REPO_NOTES.read_text(encoding="utf-8"))) == len(data["notes"])
  prefixes = [p for entry in data["notes"] for p in entry["commits"]]
  assert len(prefixes) == len(set(prefixes)), "a commit must belong to one note"
  assert all(entry["ko"].strip() for entry in data["notes"]), "every note needs Korean"


def test_malformed_entries_are_skipped_not_fatal():
  text = json.dumps({"version": 1, "notes": [
    {"commits": ["abcdef1"], "ko": "  두   칸  "},
    {"commits": ["xyz1234"], "ko": "bad hash"},
    {"commits": ["abc"], "ko": "too short"},
    {"commits": [], "ko": "no commits"},
    {"commits": ["abcdef2"], "ko": "", "en": ""},
    "not an object",
    {"commits": ["ABCDEF3"], "en": "only english"},
  ]})
  assert parse_notes(text) == [
    {"commits": ["abcdef1"], "ko": "두 칸", "en": ""},
    {"commits": ["abcdef3"], "ko": "", "en": "only english"},
  ]
  assert parse_notes("{oops") == []
  assert parse_notes(json.dumps([1, 2])) == []
  assert parse_notes("") == []


def _commit(h, subject, files, ko=""):
  return {"hash": h, "subject": subject, "ko": ko, "files": files}


def test_one_note_per_change_trailer_next_and_the_rest_undescribed():
  commits = [
    _commit("a" * 40, "Add notes", [NOTES_PATH]),                     # notes only: not listed
    _commit("b" * 40, "Bolt: hold", ["x.py"]),
    _commit("c" * 40, "Bolt: engage gate", ["y.py"]),
    _commit("d" * 40, "Web: theme", ["z.js"], ko="테마 버튼"),
    _commit("e" * 40, "Fix typo", ["w.md"]),
    _commit("f" * 40, "Merge-ish", []),                               # no files listed: still a change
  ]
  notes = [{"commits": ["ccccccc", "bbbbbbbb"], "ko": "볼트 정지", "en": "Bolt stop"}]
  result = build_notes(commits, notes, behind=6)
  assert result["items"] == [
    {"ko": "볼트 정지", "en": "Bolt stop", "commits": 2},
    {"ko": "테마 버튼", "en": "", "commits": 1},
  ]
  assert result["undescribed"] == [{"hash": "eeeeeeee", "subject": "Fix typo"},
                                   {"hash": "ffffffff", "subject": "Merge-ish"}]
  assert result["undescribed_count"] == 2
  assert result["more_items"] == 0 and result["truncated"] is False
  assert build_notes(commits[:2], notes, behind=300)["truncated"] is True


def test_items_are_capped_with_a_count_of_the_rest():
  commits = [_commit(f"{i:07x}" + "0" * 33, f"c{i}", ["f"], ko=f"변경 {i}") for i in range(MAX_ITEMS + 5)]
  result = build_notes(commits, [], behind=len(commits))
  assert len(result["items"]) == MAX_ITEMS and result["more_items"] == 5


def test_log_records_with_trailers_and_files():
  text = ("\x1e" + "1" * 40 + "\x1fFirst\x1f한국어 설명\n\nfile/a.py\nfile/b.py\n"
          "\x1e" + "2" * 40 + "\x1fSecond\x1f\n\n" + NOTES_PATH)
  assert parse_log(text) == [
    {"hash": "1" * 40, "subject": "First", "ko": "한국어 설명", "files": ["file/a.py", "file/b.py"]},
    {"hash": "2" * 40, "subject": "Second", "ko": "", "files": [NOTES_PATH]},
  ]


def _git(cwd, *args):
  return subprocess.run(["git", *args], cwd=cwd, check=True, capture_output=True, text=True).stdout.strip()


@pytest.fixture
def repos(tmp_path):
  remote = tmp_path / "remote.git"
  work = tmp_path / "work"
  device = tmp_path / "device"
  _git(tmp_path, "init", "--bare", "-b", "main", str(remote))
  _git(tmp_path, "init", "-b", "main", str(work))
  for repo in (work,):
    _git(repo, "config", "user.email", "t@example.com")
    _git(repo, "config", "user.name", "t")
  (work / "a.txt").write_text("1")
  _git(work, "add", "a.txt")
  _git(work, "commit", "-m", "base")
  _git(work, "remote", "add", "origin", str(remote))
  _git(work, "push", "-q", "origin", "main")
  _git(tmp_path, "clone", "-q", str(remote), str(device))
  return work, device


def test_device_status_lists_the_notes_of_the_waiting_commits(repos, monkeypatch):
  work, device = repos
  (work / "a.txt").write_text("2")
  _git(work, "commit", "-qam", "Bolt: hold a stop")
  first = _git(work, "rev-parse", "HEAD")
  (work / "b.txt").write_text("x")
  _git(work, "add", "b.txt")
  _git(work, "commit", "-qm", "Web: theme switch\n\nKo: 도구 화면에 테마 버튼")
  (work / "c.txt").write_text("y")
  _git(work, "add", "c.txt")
  _git(work, "commit", "-qm", "Fix a typo")
  notes = work / NOTES_PATH
  notes.parent.mkdir(parents=True)
  notes.write_text(json.dumps({"version": 1, "notes": [{"commits": [first[:8]], "ko": "볼트 정지 유지"}]}),
                   encoding="utf-8")
  _git(work, "add", NOTES_PATH)
  _git(work, "commit", "-qm", "Add update notes")
  _git(work, "push", "-q", "origin", "main")

  monkeypatch.setattr(git_status, "REPO_DIR", str(device))
  status = asyncio.run(git_status._read_status())
  assert status["state"] == "ok" and status["behind"] == 4
  assert status["notes"]["items"] == [
    {"ko": "도구 화면에 테마 버튼", "en": "", "commits": 1},
    {"ko": "볼트 정지 유지", "en": "", "commits": 1},
  ]
  assert [c["subject"] for c in status["notes"]["undescribed"]] == ["Fix a typo"]


def test_up_to_date_device_has_no_notes_and_a_bad_notes_file_only_drops_them(repos, monkeypatch):
  work, device = repos
  monkeypatch.setattr(git_status, "REPO_DIR", str(device))
  assert asyncio.run(git_status._read_status())["notes"] is None

  notes = work / NOTES_PATH
  notes.parent.mkdir(parents=True)
  notes.write_text("{broken", encoding="utf-8")
  (work / "a.txt").write_text("3")
  _git(work, "add", "-A")
  _git(work, "commit", "-qm", "Change and a broken notes file")
  _git(work, "push", "-q", "origin", "main")
  status = asyncio.run(git_status._read_status())
  assert status["state"] == "ok" and status["behind"] == 1
  assert status["notes"]["items"] == []
  assert [c["subject"] for c in status["notes"]["undescribed"]] == ["Change and a broken notes file"]
