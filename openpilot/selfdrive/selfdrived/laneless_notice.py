"""Announce the laneless road offsets (LanelessEdgeOffset, LanelessLineGap) when they start moving the car.

modeld puts each decision and its target in modelV2.meta (lanelessEdgeSide / lanelessEdgeTarget, lanelessGapSide /
lanelessGapTarget). The event is added on one frame per decision, when its target first reaches NOTICE_MIN_M, and
its alert lasts NOTICE_S from then (Alert duration, AlertManager keeps it from its start frame) even if the decision
turns off meanwhile. A decision can be on with nothing to move (the car is already far enough from the line, or
there is no room to the road edge), and a target hovering near 0 must not announce again and again; a decision
that turns off or switches side can announce again. It is not shown for as long as the offset stays on: on comma 4
an alert covers the top of the road view, and these offsets can stay on for most of a drive on a two-lane road.
"""
NOTICE_S = 3.0
NOTICE_MIN_M = 0.03
FLOAT32_SLACK_M = 1e-6   # the targets come through Float32 message fields: 0.03 arrives as 0.0299999993


class LanelessOffsetNotice:
  def __init__(self):
    self.sides = [0, 0]              # edge, gap
    self.announced = [False, False]  # this decision was announced already

  def update(self, edge_side: int, edge_target: float, gap_side: int, gap_target: float) -> tuple[bool, bool]:
    """(add the edge offset event, add the line gap event) for this frame: True on the announcing frame only."""
    out = []
    for i, (side, target) in enumerate(((int(edge_side), edge_target), (int(gap_side), gap_target))):
      if side == 0 or side != self.sides[i]:
        self.announced[i] = False
      self.sides[i] = side
      try:
        moving = abs(float(target)) >= NOTICE_MIN_M - FLOAT32_SLACK_M
      except (TypeError, ValueError):
        moving = False
      announce = side != 0 and not self.announced[i] and moving
      if announce:
        self.announced[i] = True
      out.append(announce)
    return out[0], out[1]
