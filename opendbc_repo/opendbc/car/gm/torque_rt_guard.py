from collections import deque


class PandaRtGuard:
  """Keep each sent LKA torque inside the panda's real-time band, whatever the send timing.

  The panda (safety.h rt_rate_limit_check) holds a reference command and accepts a new one only within
  [min(ref, 0) - max_rt_delta, max(ref, 0) + max_rt_delta]; it replaces the reference with the first command that
  arrives more than 250 ms after it, checking that command before replacing. So the reference used for the next
  command was sent at most 250 ms before the previous command. A fixed step limit only bounds this at the nominal
  command rate: a late control loop that catches up with short gaps stacks more rises on one reference (Codex
  review 2026-11-17: 80 ms late, then 7.5 ms calls, broke the check at 12/16 per command). Checking the band of every
  command sent within 250 ms + margin before the previous one covers any timing; the margin absorbs the difference
  between our send time and the panda's receive time. The band always contains [-max_rt_delta, max_rt_delta] and
  the previous command, so clipping only slows a rise and never blocks moving toward zero.
  """

  def __init__(self, max_rt_delta: int, interval_ms: int, margin_ms: int = 40):
    self.max_rt_delta = max_rt_delta
    self.window_ns = int((interval_ms + margin_ms) * 1e6)
    self.sent: deque[tuple[int, int]] = deque()

  def clip(self, torque: int, now_nanos: int) -> int:
    if self.sent:
      cutoff = self.sent[-1][0] - self.window_ns
      while self.sent[0][0] < cutoff:
        self.sent.popleft()
      highest = min(max(v, 0) for _, v in self.sent) + self.max_rt_delta
      lowest = max(min(v, 0) for _, v in self.sent) - self.max_rt_delta
      torque = int(min(max(torque, lowest), highest))
    self.sent.append((now_nanos, torque))
    return torque
