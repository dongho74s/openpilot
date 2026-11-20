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
commands unless the command passes BRAKE_NOW or GAS_NOW, which switch at once. Stops and inactive frames bypass it.
"""

BRAKE_ENTER = -0.2  # m/s^2: leave the gas side below this (after HOLD_FRAMES)
GAS_ENTER = 0.1  # m/s^2: leave the brake side above this (after HOLD_FRAMES)
BRAKE_NOW = -0.4  # m/s^2: always brake at once below this
GAS_NOW = 0.3  # m/s^2: always accelerate at once above this
HOLD_FRAMES = 12  # 0x2CB commands (25 Hz): 0.48 s


def has_gas_brake_step(params) -> bool:
  """True when the gas lookup jumps from ZERO_GAS to MAX_ACC_REGEN at 0 (camera ACC / SDGM cars)."""
  return params.GAS_LOOKUP_BP[0] == params.GAS_LOOKUP_BP[1] == 0.


class GasBrakeHysteresis:
  def __init__(self):
    self.reset()

  def reset(self):
    self.gas_side = None
    self.frames = 0

  def update(self, accel: float, apply_gas: int, apply_brake: int, zero_gas: float, max_acc_regen: float):
    """accel is the gas lookup's input; apply_gas/apply_brake are what the lookups gave for it."""
    want_gas = accel >= 0.
    if self.gas_side is None:
      self.gas_side = want_gas
      self.frames = 0
    self.frames += 1

    if want_gas != self.gas_side:
      if want_gas:
        switch = accel > GAS_NOW or (accel > GAS_ENTER and self.frames >= HOLD_FRAMES)
      else:
        switch = accel < BRAKE_NOW or (accel < BRAKE_ENTER and self.frames >= HOLD_FRAMES)
      if switch:
        self.gas_side = want_gas
        self.frames = 0

    if self.gas_side and not want_gas:
      return int(round(zero_gas)), 0
    if not self.gas_side and want_gas:
      return int(round(max_acc_regen)), apply_brake
    return apply_gas, apply_brake
