"""Plain-language notes for the commits a device has not pulled yet, shown under Carrot Web's update row (2026-11-28).

Commit subjects are short English lines written for developers. The notes are written for drivers, in Korean (and
optionally English), and live in the repository in NOTES_PATH:

  {"version": 1, "notes": [{"commits": ["6339565a", "d7964887"], "ko": "...", "en": "..."}]}

One entry can describe several commits (hash prefixes of at least 7 characters). The file is read from the upstream
ref right after the fetch, so a note written after its commits were pushed, or reworded later, still reaches devices
that have not updated yet. A commit's own `Ko:` trailer is used when the file has no entry for it. Commits that change
only the notes file are not listed. Anything malformed is skipped: the update row never depends on the notes.
"""
from __future__ import annotations

import json
import re
from collections.abc import Awaitable, Callable
from typing import Any

NOTES_PATH = "openpilot/selfdrive/carrot/update_notes.json"
MAX_COMMITS = 200
MAX_ITEMS = 30
MAX_UNDESCRIBED = 30
MAX_NOTES_BYTES = 512 * 1024
MAX_TEXT = 600
# One record per commit: hash, subject and the Ko trailer, then the changed files (--name-only).
LOG_FORMAT = "%x1e%H%x1f%s%x1f%(trailers:key=Ko,valueonly,separator=%x20)"
_HASH = re.compile(r"^[0-9a-f]{7,40}$")

GitRunner = Callable[[list[str]], Awaitable[tuple[int, str]]]


def _text(value: Any) -> str:
  return " ".join(value.split())[:MAX_TEXT] if isinstance(value, str) else ""


def parse_notes(text: str) -> list[dict[str, Any]]:
  """The valid entries of the notes file, in file order."""
  if not text or len(text.encode("utf-8", "replace")) > MAX_NOTES_BYTES:
    return []
  try:
    data = json.loads(text)
  except ValueError:
    return []
  raw = data.get("notes") if isinstance(data, dict) else None
  if not isinstance(raw, list):
    return []
  notes = []
  for entry in raw:
    if not isinstance(entry, dict):
      continue
    commits = entry.get("commits")
    if not isinstance(commits, list) or not commits:
      continue
    prefixes = [c.strip().lower() for c in commits if isinstance(c, str) and _HASH.match(c.strip().lower())]
    if len(prefixes) != len(commits):
      continue
    ko, en = _text(entry.get("ko")), _text(entry.get("en"))
    if ko or en:
      notes.append({"commits": prefixes, "ko": ko, "en": en})
  return notes


def parse_log(text: str) -> list[dict[str, Any]]:
  """`git log --format=LOG_FORMAT --name-only` output, newest first."""
  commits = []
  for record in text.split("\x1e"):
    head, _, rest = record.partition("\n")
    fields = head.split("\x1f")
    if len(fields) < 2:
      continue
    commit = fields[0].strip().lower()
    if not _HASH.match(commit):
      continue
    commits.append({
      "hash": commit,
      "subject": _text(fields[1]),
      "ko": _text(fields[2]) if len(fields) > 2 else "",
      "files": [line.strip() for line in rest.splitlines() if line.strip()],
    })
  return commits


def build_notes(commits: list[dict[str, Any]], notes: list[dict[str, Any]], behind: int) -> dict[str, Any]:
  """What the update row lists: described changes (one per note entry), then the commits nobody described yet."""
  items: list[dict[str, Any]] = []
  by_entry: dict[int, dict[str, Any]] = {}
  undescribed = []
  for commit in commits:
    if commit["files"] and all(name == NOTES_PATH for name in commit["files"]):
      continue
    entry = next((e for e in notes if any(commit["hash"].startswith(p) for p in e["commits"])), None)
    if entry is not None:
      item = by_entry.get(id(entry))
      if item is not None:
        item["commits"] += 1
        continue
      item = {"ko": entry["ko"], "en": entry["en"], "commits": 1}
      by_entry[id(entry)] = item
      items.append(item)
    elif commit["ko"]:
      items.append({"ko": commit["ko"], "en": "", "commits": 1})
    else:
      undescribed.append({"hash": commit["hash"][:8], "subject": commit["subject"]})
  return {
    "items": items[:MAX_ITEMS],
    "more_items": max(0, len(items) - MAX_ITEMS),
    "undescribed": undescribed[:MAX_UNDESCRIBED],
    "undescribed_count": len(undescribed),
    "truncated": behind > len(commits),
  }


async def collect_update_notes(git: GitRunner, upstream: str, behind: int) -> dict[str, Any] | None:
  """Notes for HEAD..upstream (already fetched), or None when there is nothing to pull or git fails."""
  if behind <= 0 or not upstream:
    return None
  rc, log = await git(["log", "--no-merges", f"-n{MAX_COMMITS}", f"--format={LOG_FORMAT}", "--name-only",
                       f"HEAD..{upstream}", "--"])
  if rc != 0:
    return None
  rc, text = await git(["show", f"{upstream}:{NOTES_PATH}"])
  return build_notes(parse_log(log), parse_notes(text) if rc == 0 else [], behind)
