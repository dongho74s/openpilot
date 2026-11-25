"""Gas/brake side hysteresis for GM openpilot longitudinal control through 0x2CB and 0x315.

On the camera ACC and SDGM cars (CarControllerParams max_regen_acceleration 0) any accel below 0 switches 0x2CB from
gas (>= ZERO_GAS) to MAX_ACC_REGEN and starts the friction brake, as the stock camera does. carrot's LongControl then
moved its command across 0 about once a second while it held a speed: rlog 00000028--02c4808d1b--18 (Bolt ACC with
pedal, mode 1, about 4 degrees downhill, 2026-11-20) switched sides 48 times in 54 s. Fitted on that log, the Bolt's
gas side gives about -0.26 m/s^2 at ZERO_GAS and its brake side about -0.88 m/s^2 at MAX_ACC_REGEN with no friction
brake, so every switch was a step of about 0.6 m/s^2. A Traverse log (000001ea--ba62d04ea0--6) showed a much smaller
step but the same switching.

This keeps the side it is on while the command stays near 0: the gas side coasts (ZERO_GAS, no brake) down to
BRAKE_ENTER and the brake side holds MAX_ACC_REGEN up to GAS_ENTER; after a switch the new side is held for HOLD_FRAMES
commands unless the command passes BRAKE_NOW or GAS_NOW, which switch at once. The brake side is judged on both the
gas lookup's and the brake lookup's input: with LongPitch on they differ (below 10 m/s the brake keeps less of the
pitch), and a strong raw brake request must not be lost to a pitch-compensated gas input near 0 (Codex review
2026-11-20 R1). Only pid commands use it; the CarController resets it on every other frame.

Blend (user request 2026-11-25, every car that uses this hysteresis): the coast is wanted, but leaving it put
MAX_ACC_REGEN and the friction brake for the whole request on at once. Traverse rlogs after this hysteresis reached the car (11-21, routes 245-257
against 231/23c from 11-18..20) braked later and harder at the start: the 0.3 s decel jerk of brake starts with a
-0.3..-0.6 request went from -0.72 to -1.05 m/s^3 (10th percentile -1.07 to -2.44), with -0.6..-1.2 from -1.11 to
-1.52 (-1.73 to -3.85). With blend on, a switch from the gas side to the brake side fades the brake side in over
BLEND_FRAMES commands for a request down to BRAKE_NOW, fewer for a deeper one, and at once from BLEND_URGENT down. The
coast itself is unchanged. Review 2026-11-25 (independent reviewer, Codex login expired):
- the friction brake rises from what was actually sent last, never from 0: with LongPitch on a slow uphill the gas
  side already sends a brake (gas input >= 0, brake input < 0), and restarting at 0 dropped 60 to 11 just as the
  request deepened. Only rises are faded; a lower lookup value is sent as it is.
- a brake let off for up to QUICK_RETURN_FRAMES gas commands resumes where it was; longer gas phases empty the fade
  at half the fill rate (2 * BLEND_FRAMES commands), so a back-and-forth always fills it. A +0.35/-0.45
  command-by-command back-and-forth kept the brake near 11 % before, and 3 gas + 1 brake commands near 20 %.
- 0x2CB between ZERO_GAS and MAX_ACC_REGEN is faded only on the EVs (blend_regen), whose EVTable path already sends
  such values; the stock camera sends only the two ends, and how an engine car's ECM takes the values in between is
  unknown, so there 0x2CB switches at once and only the friction brake fades.
"""

import numpy as np

BRAKE_ENTER = -0.2  # m/s^2: leave the gas side below this (after HOLD_FRAMES)
GAS_ENTER = 0.1  # m/s^2: leave the brake side above this (after HOLD_FRAMES)
BRAKE_NOW = -0.4  # m/s^2: always brake at once below this
GAS_NOW = 0.3  # m/s^2: always accelerate at once above this
HOLD_FRAMES = 12  # 0x2CB commands (25 Hz): 0.48 s
BLEND_FRAMES = 10  # 0x2CB commands (25 Hz): 0.4 s fade-in of the brake side for a request down to BRAKE_NOW
BLEND_URGENT = -1.0  # m/s^2: no fade-in from here down (urgent braking keeps the immediate brake)
QUICK_RETURN_FRAMES = 2  # gas commands a brake let-off may last and still resume where it was


def has_gas_brake_step(params) -> bool:
  """True when the gas lookup jumps from ZERO_GAS to MAX_ACC_REGEN at 0 (camera ACC / SDGM cars)."""
  return params.GAS_LOOKUP_BP[0] == params.GAS_LOOKUP_BP[1] == 0.


class GasBrakeHysteresis:
  def __init__(self, blend: bool = False, blend_regen: bool = False):
    self.blend_enabled = blend
    self.blend_regen = blend_regen
    self.reset()

  def reset(self):
    self.gas_side = None
    self.frames = 0
    self.blend = 1.0  # brake-side share sent: 1 = the brake side as looked up
    self.blend_left = 0.0  # the share sent in the last brake-side command (what a quick return resumes from)
    self.gas_frames = 0  # gas-side commands since the last switch
    self.last_brake = 0  # friction brake in the last command this returned
    self.start_brake = 0  # friction brake sent just before the brake side began fading in

  def update(self, accel: float, brake_accel: float, apply_gas: int, apply_brake: int, zero_gas: float,
             max_acc_regen: float):
    """accel and brake_accel are the gas and brake lookups' inputs; apply_gas/apply_brake are what they gave."""
    want_gas = accel >= 0.
    brake_request = min(accel, brake_accel)
    if self.gas_side is None:
      self.gas_side = want_gas
      self.frames = 0
    self.frames += 1

    if want_gas != self.gas_side:
      if want_gas:
        switch = accel > GAS_NOW or (accel > GAS_ENTER and self.frames >= HOLD_FRAMES)
      else:
        switch = brake_request < BRAKE_NOW or (brake_request < BRAKE_ENTER and self.frames >= HOLD_FRAMES)
      if switch:
        if self.blend_enabled and self.gas_side and not want_gas:
          # back on the brake side: fade in after a real gas phase; a quick return resumes where it was let off
          # (emptied at half the fill rate, so any gas/brake back-and-forth still fills it: 3 gas + 1 brake did not)
          let_off = max(0, self.gas_frames - QUICK_RETURN_FRAMES) / (2 * BLEND_FRAMES)
          self.blend = max(0.0, self.blend_left - let_off)
          self.start_brake = self.last_brake
        self.gas_side = want_gas
        self.frames = 0
        self.gas_frames = 0

    if self.gas_side:
      self.gas_frames += 1
      return self._sent((zero_gas, 0) if not want_gas else (apply_gas, apply_brake))

    gas, brake = (max_acc_regen, apply_brake) if want_gas else (apply_gas, apply_brake)
    if self.blend < 1.0:
      frames = float(np.interp(brake_request, [BLEND_URGENT, BRAKE_NOW], [0., BLEND_FRAMES]))
      self.blend = 1.0 if frames < 1. else min(1.0, self.blend + 1.0 / frames)
      if self.blend_regen:
        gas = zero_gas + self.blend * (gas - zero_gas)
      if brake > self.start_brake:  # only a rise is faded, from what was sent
        brake = self.start_brake + self.blend * (brake - self.start_brake)
    self.blend_left = self.blend
    return self._sent((gas, brake))

  def _sent(self, command):
    gas, brake = int(round(command[0])), int(round(command[1]))
    self.last_brake = brake
    return gas, brake
