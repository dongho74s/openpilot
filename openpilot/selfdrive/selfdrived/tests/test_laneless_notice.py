"""Laneless offsets are announced when they start moving the car (user request 2026-11-16: with sound, kept 3 s).

The event is added on one frame; its alert lasts 3 s from its start frame (AlertManager, Alert duration)."""
import ast
from pathlib import Path

import numpy as np

from openpilot.selfdrive.selfdrived.laneless_notice import NOTICE_S, LanelessOffsetNotice

DT = 0.01


def frames(notice, seconds, edge=(0, 0.0), gap=(0, 0.0)):
  return [notice.update(edge[0], edge[1], gap[0], gap[1]) for _ in range(int(round(seconds / DT)))]


def test_moving_the_car_is_announced_once_per_decision():
  notice = LanelessOffsetNotice()
  assert not any(e for e, _ in frames(notice, 2.0, edge=(1, 0.0)))     # on, nothing to move: silent
  out = frames(notice, NOTICE_S + 2.0, edge=(1, 0.1))
  assert out[0] == (True, False) and sum(e for e, _ in out) == 1 and not any(g for _, g in out)
  assert not any(e for e, _ in frames(notice, 1.0, edge=(1, 0.01)) + frames(notice, 1.0, edge=(1, 0.2)))  # hovering


def test_a_new_decision_or_side_is_announced_again():
  notice = LanelessOffsetNotice()
  frames(notice, 5.0, gap=(1, 0.2))
  assert frames(notice, 0.5, gap=(-1, -0.2))[0] == (False, True)
  frames(notice, 1.0)
  assert frames(notice, DT, gap=(-1, -0.2))[0] == (False, True)


def test_offsets_are_announced_independently_and_bad_targets_are_silent():
  notice = LanelessOffsetNotice()
  frames(notice, 1.0, edge=(1, 0.2))
  out = frames(notice, 1.0, gap=(1, 0.05))
  assert out[0] == (False, True) and not any(e for e, _ in out)
  assert frames(notice, DT, edge=(1, float("nan")))[0][0] is False


def test_exactly_3_cm_through_a_float32_message_field_is_announced():
  target = float(np.float32(0.03))   # 0.029999999329447746
  for edge, gap in (((1, target), (0, 0.0)), ((0, 0.0), (-1, -target))):
    assert frames(LanelessOffsetNotice(), DT, edge=edge, gap=gap)[0] != (False, False)


def test_the_alerts_last_3_s_from_their_start_even_if_the_event_is_added_once():
  """events.py imports native messaging, so read the alert duration from its source; AlertManager keeps an alert
  until start_frame + duration (alertmanager.add_many)."""
  src = (Path(__file__).resolve().parents[1] / "events.py").read_text(encoding="utf8")
  tree = ast.parse(src)
  value = next(n.value.value for n in tree.body if isinstance(n, ast.Assign)
               and any(isinstance(t, ast.Name) and t.id == "LANELESS_NOTICE_S" for t in n.targets))
  assert value == NOTICE_S
  for name in ("laneless_edge_offset_alert", "laneless_line_gap_alert"):
    fn = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == name)
    call = next(n for n in ast.walk(fn) if isinstance(n, ast.Call) and getattr(n.func, "id", "") == "Alert")
    assert ast.unparse(call.args[-1]) == "LANELESS_NOTICE_S + LANELESS_SOUND_S"   # text visible 3 s after the sound
    assert ast.unparse(call.args[-2]) == "AudibleAlert.none"   # the sound is its own short alert
  am = (Path(__file__).resolve().parents[1] / "alertmanager.py").read_text(encoding="utf8")
  assert "min_end_frame = entry.start_frame + alert.duration" in am


def test_the_sound_alert_follows_each_announcement_a_frame_later_and_never_outranks_another_warning():
  """soundd restarts a sound only when selfdriveState.alertSound changes; the sound alert lasts 0.2 s, has no text and
  the lowest priority, and starts a frame after the text alert, so alertSound goes prompt -> none -> prompt for each
  announcement while any other warning (LOWER and up) still wins."""
  src = (Path(__file__).resolve().parents[1] / "events.py").read_text(encoding="utf8")
  block = src[src.index("EventName.lanelessOffsetSound: {"):]
  block = block[:block.index("},") + 2]
  assert "AudibleAlert.prompt, LANELESS_SOUND_S)" in block and "AlertSize.none" in block and "Priority.LOWEST," in block
  sd = (Path(__file__).resolve().parents[1] / "selfdrived.py").read_text(encoding="utf8")
  assert ("    if self.laneless_sound_due:   # a frame after the text alert: its newer start wins the lowest-priority tie\n"
          "      self.events.add(EventName.lanelessOffsetSound)\n"
          "    self.laneless_sound_due = edge_notice or gap_notice\n") in sd
  soundd = (Path(__file__).resolve().parents[2] / "ui" / "soundd.py").read_text(encoding="utf8")
  assert "if self.current_alert != new_alert and" in soundd
