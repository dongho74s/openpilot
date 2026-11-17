"""Disengaged GM pedal hold. A gas release of an active hold requests a
one-shot cruise resume (RES); every other release clears it silently."""
import math

from opendbc.car import structs
from opendbc.car.gm.values import CAR, GMSafetyFlags

ARM_BRAKE = 80  # ECMAcceleratorPos.BrakePedalPos raw units, not percent.
ARM_SPEED = 1.5  # m/s; arming is not a brake command.
HOLD_BRAKE = 400
HOLD_SPEED = 0.05


# Car -> brakes through the SDGM camera-bus harness. The Traverse uses that
# harness; the 2021-22 Trailblazer and 2024 Trax use the regular camera-long
# powertrain brake path. Cruise-only variants have no ACC brake path.
MANUAL_HOLD_CARS = {
  CAR.CHEVROLET_TRAVERSE: True,
  CAR.CHEVROLET_TRAILBLAZER: False,
  CAR.CHEVROLET_TRAX: False,
}


def manual_hold_supported(CP):
  sdgm = MANUAL_HOLD_CARS.get(CP.carFingerprint)
  if sdgm is None:
    return False
  required = GMSafetyFlags.HW_CAM | GMSafetyFlags.HW_CAM_LONG | GMSafetyFlags.MANUAL_AUTO_HOLD
  incompatible = (GMSafetyFlags.CC_LONG | GMSafetyFlags.NO_ACC | GMSafetyFlags.PEDAL_LONG | GMSafetyFlags.NO_CAMERA |
                  GMSafetyFlags.GAS_INTERCEPTOR | GMSafetyFlags.HW_ASCM_LONG | GMSafetyFlags.EV)
  if sdgm:
    required |= GMSafetyFlags.HW_SDGM
  else:
    incompatible |= GMSafetyFlags.HW_SDGM
  return (CP.openpilotLongitudinalControl and not CP.enableGasInterceptorDEPRECATED and bool(CP.safetyConfigs) and
          (CP.safetyConfigs[0].safetyParam & required) == required and
          not CP.safetyConfigs[0].safetyParam & incompatible)


def eligible(cs, controls_active):
  return (not controls_active and not cs.cruiseState.enabled and cs.canValid and not cs.canTimeout and
          not cs.accFaulted and not cs.gasPressed and not cs.regenBraking and not cs.parkingBrake and
          cs.gearShifter in (structs.CarState.GearShifter.drive, structs.CarState.GearShifter.low) and
          math.isfinite(cs.vEgo) and math.isfinite(cs.vEgoRaw))


def stopped(cs):
  return cs.standstill and abs(cs.vEgo) < HOLD_SPEED and abs(cs.vEgoRaw) < HOLD_SPEED


class ManualAutoHold:
  def __init__(self):
    # One-shot cruise resume request. NOT part of reset(): update() runs
    # twice per control cycle (interface update, then apply) and the second
    # call resets the hold while gas is still pressed, so a flag cleared by
    # reset() would never reach the car controller. It is a latch set only
    # by a gas release and cleared only by the controller (after sending
    # RES) or by an explicit cancellation below / a stale-CAN reset.
    self.resume_requested = False
    self.reset()

  def reset(self):
    self.armed = False
    self.active = False
    self.pedal_released = False
    self.press_peak = 0.

  def update(self, cs, pedal, controls_active):
    # Cancel a pending resume if the driver brakes or the car leaves the
    # resumable state (gear, CAN, fault, cruise already on, openpilot
    # engaged, parking brake). Gas alone does not cancel: the driver is
    # still pressing it after a gas release, and the car moving off is
    # expected, so neither clears the latch.
    if (cs.brakePressed or cs.parkingBrake or cs.accFaulted or not cs.canValid or cs.canTimeout or
        cs.cruiseState.enabled or controls_active or
        cs.gearShifter not in (structs.CarState.GearShifter.drive, structs.CarState.GearShifter.low)):
      self.resume_requested = False

    # Gas release of an active, stopped hold in D/low: the driver wants to go,
    # so request one-shot RES. Brake re-press, gear change and CAN loss clear
    # the hold without a resume (gas_release stays False for those).
    gas_release = (self.active and cs.gasPressed and not cs.brakePressed and stopped(cs) and
                   cs.gearShifter in (structs.CarState.GearShifter.drive, structs.CarState.GearShifter.low))

    if (not eligible(cs, controls_active) or not math.isfinite(pedal) or not 0 <= pedal <= 255 or
        max(abs(cs.vEgo), abs(cs.vEgoRaw)) > ARM_SPEED or (self.active and not stopped(cs))):
      self.reset()
      if gas_release:
        self.resume_requested = True
      return

    if cs.brakePressed and pedal >= ARM_BRAKE:
      self.armed = True
    elif not self.active and not cs.brakePressed:
      self.armed = False
    if not self.active and self.armed and cs.brakePressed and stopped(cs):
      self.active = True
      self.pedal_released = False
      self.press_peak = 0.

    if self.active:
      if cs.brakePressed:
        if self.pedal_released:
          self.press_peak = max(self.press_peak, pedal)
      else:
        if self.pedal_released and 0 < self.press_peak < ARM_BRAKE:
          self.reset()
          return
        self.pedal_released = True
        self.press_peak = 0.

  def command(self, cs, controls_active):
    return HOLD_BRAKE if self.active and eligible(cs, controls_active) and stopped(cs) else 0
