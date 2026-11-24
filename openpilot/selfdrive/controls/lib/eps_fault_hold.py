from openpilot.common.realtime import DT_CTRL


class EpsTemporaryFaultHold:
  """GM: after the EPS reports LKA torque temporarily limited (PSCMStatus LKATorqueDeliveredStatus 2, carState
  steerFaultTemporary), do not request lateral again for a while.

  The Traverse EPS refuses the first LKA request of nearly every drive (status 2, no torque delivered). Re-requesting
  as soon as the status cleared kept it refusing: lateral toggled every 0.1 s for up to 19 s (rlog 00000251
  11:35:29-35 and 00000252 12:20:20-39). After a 3.5 s or 9 s pause it accepted at once, after 1.2 s it refused again.
  Over seven drives every fault but three was such an activation refusal; the other three were at 10-24 km/h.

  controlsd keeps lateral off with it; selfdrived runs its own copy on the same carState stream to show the hold."""
  RETRY_S = 3.0

  def __init__(self, enabled: bool, hold_s: float | None = None):
    self.enabled = enabled
    self.frames = int(round((self.RETRY_S if hold_s is None else hold_s) / DT_CTRL))
    self.remaining = 0

  def update(self, steer_fault_temporary: bool) -> bool:
    """True while lateral must stay off: the fault itself, then the hold after it."""
    if not self.enabled:
      return steer_fault_temporary
    if steer_fault_temporary:
      self.remaining = self.frames
    elif self.remaining > 0:
      self.remaining -= 1
    return steer_fault_temporary or self.remaining > 0
