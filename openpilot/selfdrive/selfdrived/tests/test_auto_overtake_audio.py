"""Auto-overtake lane-change voice announcement.

SelfdriveD.update_auto_overtake_events watches modelV2.meta.desire (the
DesireHelper's desire, published by modeld). When it flips to
laneChangeLeft/Right a lane change has started; the event rings the
"차선변경 합니다" voice (EventName.audioLaneChange -> AudibleAlert.laneChange
-> audio_lane_change.wav) exactly once per change.

The method is exercised by AST-extracting it from selfdrived.py, the same
way test_model_startup_events.py tests update_events without device drivers.
"""
import ast
from pathlib import Path
from types import SimpleNamespace

from openpilot.cereal import log

ROOT = Path(__file__).resolve().parents[1]
EventName = log.OnroadEvent.EventName


def load_method():
  tree = ast.parse((ROOT / 'selfdrived.py').read_text(encoding='utf8'))
  cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'SelfdriveD')
  method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'update_auto_overtake_events')
  wrapper = ast.parse('def check(self, CS): pass')
  wrapper.body[0].body = method.body
  ns = {'log': log, 'EventName': EventName}
  exec(compile(ast.fix_missing_locations(wrapper), '<auto overtake audio>', 'exec'), ns)
  return ns['check']


class FakeSM(dict):
  def __init__(self):
    super().__init__(
      modelV2=SimpleNamespace(meta=SimpleNamespace(desire=log.Desire.none)),
      carrotMan=SimpleNamespace(atcType='none'),
    )
    self.alive = {'modelV2': True, 'carrotMan': True}


def drive():
  return SimpleNamespace(sm=FakeSM(), events=set(), _desire_prev=log.Desire.none)


def step(state, desire, *, left_blinker=False, right_blinker=False, atc_type='none', model_alive=True):
  """Feed one frame; returns True if the voice event rang."""
  state.sm['modelV2'].meta.desire = desire
  state.sm['carrotMan'].atcType = atc_type
  state.sm.alive['modelV2'] = model_alive
  cs = SimpleNamespace(leftBlinker=left_blinker, rightBlinker=right_blinker)
  state.events.clear()
  _METHOD(state, cs)
  return EventName.audioLaneChange in state.events


_METHOD = load_method()


def test_auto_overtake_left_rings_once():
  state = drive()
  # No change yet: silence.
  assert not step(state, log.Desire.none)
  # Auto-overtake left starts: rings.
  assert step(state, log.Desire.laneChangeLeft)
  # Desire held through starting/finishing: no repeat.
  assert not step(state, log.Desire.laneChangeLeft)
  assert not step(state, log.Desire.laneChangeLeft)
  # Change finished: silence.
  assert not step(state, log.Desire.none)
  # A new left change rings again.
  assert step(state, log.Desire.laneChangeLeft)


def test_auto_overtake_return_right_rings():
  state = drive()
  # Return to the original (right) lane: also an automatic change, rings.
  assert step(state, log.Desire.laneChangeRight)
  assert not step(state, log.Desire.laneChangeRight)
  assert not step(state, log.Desire.none)


def test_lever_lane_change_silent():
  state = drive()
  # Driver's own lever change: physical stalk blinker on, no voice.
  assert not step(state, log.Desire.laneChangeLeft, left_blinker=True)
  assert not step(state, log.Desire.laneChangeRight, right_blinker=True)
  # A lever change must not consume the announcement: the next automatic
  # change still rings.
  assert step(state, log.Desire.laneChangeLeft)


def test_atc_lane_change_silent():
  state = drive()
  # ATC navigation change: it already rings its own audioLaneChange in
  # update_atc_events, so this must not double-announce.
  for atc_type in ('fork left', 'fork right', 'atc left', 'atc right', 'turn left', 'turn right'):
    assert not step(state, log.Desire.laneChangeLeft, atc_type=atc_type), atc_type
    assert not step(state, log.Desire.none, atc_type=atc_type), atc_type
  # After the ATC maneuver ends, an automatic change rings again.
  assert step(state, log.Desire.laneChangeLeft, atc_type='none')


def test_modelv2_not_alive_skips_and_recovers():
  state = drive()
  # modelV2 down: no ring, and the previous desire is not consumed.
  assert not step(state, log.Desire.laneChangeLeft, model_alive=False)
  assert state._desire_prev == log.Desire.none
  # modelV2 back: the change in progress still rings (caught late, once).
  assert step(state, log.Desire.laneChangeLeft, model_alive=True)
  assert not step(state, log.Desire.laneChangeLeft, model_alive=True)
