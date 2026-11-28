from dataclasses import dataclass, field
from enum import Enum, IntFlag

import numpy as np
from opendbc.car import Bus, PlatformConfig, DbcDict, Platforms, CarSpecs
from opendbc.car.structs import CarParams
from opendbc.car.docs_definitions import CarHarness, CarDocs, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = CarParams.Ecu


class CarControllerParams:
  STEER_MAX = 300  # GM limit is 3Nm. Used by carcontroller to generate LKA output
  # Ceiling for CustomSteerMax. In Traverse logs the reported LKATorqueDelivered stayed at about 3.00
  # once commands passed ~300 while up to 400 was sent, so the extra range only delayed torque release.
  STEER_MAX_EPS = 300
  STEER_STEP = 4  # Active control frames per command (25hz)
  INACTIVE_STEER_STEP = 10  # Inactive control frames per command (10hz)
  STEER_DELTA_UP = 5  # Delta rates require review due to observed EPS weakness
  STEER_DELTA_DOWN = 7
  STEER_DRIVER_ALLOWANCE = 65
  STEER_DRIVER_MULTIPLIER = 4
  STEER_DRIVER_FACTOR = 100
  NEAR_STOP_BRAKE_PHASE = 0.1
  SNG_INTERCEPTOR_GAS = 18. / 255.
  SNG_TIME = 30  # frames until the above is reached

  # Heartbeat for dash "Service Adaptive Cruise" and "Service Front Camera"
  ADAS_KEEPALIVE_STEP = 100
  CAMERA_KEEPALIVE_STEP = 100

  # Allow small margin below -3.5 m/s^2 from ISO 15622:2018 since we
  # perform the closed loop control, and might need some
  # to apply some more braking if we're on a downhill slope.
  # Our controller should still keep the 2 second average above
  # -3.5 m/s^2 as per planner limits
  ACCEL_MAX = 2.  # m/s^2
  ACCEL_MIN = -4.  # m/s^2

  def __init__(self, CP):
    # Gas/brake lookups
    self.ZERO_GAS = 0.0  # Coasting
    self.MAX_BRAKE = 400  # ~ -4.0 m/s^2 with regen

    if CP.carFingerprint in (CAMERA_ACC_CAR | SDGM_CAR) and not is_cc_only(CP):
      self.MAX_GAS = 1346.0
      self.MAX_ACC_REGEN = -540.0
      self.INACTIVE_REGEN = -500.0
      # Camera ACC vehicles have no regen while enabled.
      # Camera transitions to MAX_ACC_REGEN from ZERO_GAS and uses friction brakes instantly
      max_regen_acceleration = 0.

    else:
      self.MAX_GAS = 1018.0  # Safety limit, not ACC max. Stock ACC >2042 from standstill.
      self.MAX_ACC_REGEN = -650.0  # Max ACC regen is slightly less than max paddle regen
      self.INACTIVE_REGEN = -650.0
      # ICE has much less engine braking force compared to regen in EVs,
      # lower threshold removes some braking deadzone
      max_regen_acceleration = -1. if CP.carFingerprint in EV_CAR else -0.1

    self.GAS_LOOKUP_BP = [max_regen_acceleration, 0., self.ACCEL_MAX]
    self.GAS_LOOKUP_V = [self.MAX_ACC_REGEN, self.ZERO_GAS, self.MAX_GAS]

    self.BRAKE_LOOKUP_BP = [self.ACCEL_MIN, max_regen_acceleration]
    self.BRAKE_LOOKUP_V = [self.MAX_BRAKE, 0.]

    if CP.carFingerprint in FROGPILOT_STEER_TIMING_CAR:
      # Same LKA command timing as upstream openpilot and FrogPilot GM: ~33 Hz, +10/-15 per command.
      # At 25 Hz a +5 ramp needs 2.4 s for 0->300 (2.0 s with CustomSteerDeltaUp 6), which kept
      # large-curve requests rate limited long enough for the torque loop to wind and unwind the wheel.
      self.STEER_STEP = 3
      self.STEER_DELTA_UP = 10
      self.STEER_DELTA_DOWN = 15

  # Panda GM torque limits (safety_gm.h GM_STEERING_LIMITS); a CustomSteerDelta* setting is clipped to stay inside them.
  PANDA_MAX_RATE_UP = 20
  PANDA_MAX_RATE_DOWN = 25
  PANDA_MAX_RT_DELTA = 128
  PANDA_RT_INTERVAL_MS = 250

  def max_custom_steer_delta_up(self) -> int:
    """Largest per-command rise whose sum stays inside the panda's real-time check at this command rate.
    The panda compares each command with a reference it resets only after 250 ms have passed, and it checks before
    resetting, so 250 // period + 1 rises land on one reference (9 at 3 frames, 7 at 4). One more command is allowed
    for timing jitter. 3 frames: 12 (15 broke the check after the Bolt CC moved to 3 frames); 4 frames: 16."""
    rises = self.PANDA_RT_INTERVAL_MS // (self.STEER_STEP * 10) + 2
    return min(self.PANDA_MAX_RATE_UP, self.PANDA_MAX_RT_DELTA // rises)

  # determined by letting Volt regen to a stop in L gear from 89mph,
  # and by letting off gas and allowing car to creep, for determining
  # the positive threshold values at very low speed
  EV_GAS_BRAKE_THRESHOLD_BP = [1.29, 1.52, 1.55, 1.6, 1.7, 1.8, 2.0, 2.2, 2.5, 5.52, 9.6, 20.5, 23.5, 35.0] # [m/s]
  EV_GAS_BRAKE_THRESHOLD_V = [0.0, -0.14, -0.16, -0.18, -0.215, -0.255, -0.32, -0.41, -0.5, -0.72, -0.905, -1.14, -1.16, -1.175] # [m/s^s]

  def update_ev_gas_brake_threshold(self, v_ego):
    gas_brake_threshold = np.interp(v_ego, self.EV_GAS_BRAKE_THRESHOLD_BP, self.EV_GAS_BRAKE_THRESHOLD_V)
    self.EV_GAS_LOOKUP_BP = [gas_brake_threshold, max(0., gas_brake_threshold), self.ACCEL_MAX]
    self.EV_BRAKE_LOOKUP_BP = [self.ACCEL_MIN, gas_brake_threshold]


class GMSafetyFlags(IntFlag):
  HW_CAM = 1
  HW_CAM_LONG = 2
  CC_LONG = 4
  NO_CAMERA = 8
  HW_ASCM_LONG = 16
  NO_ACC = 32
  PEDAL_LONG = 64  # TODO: This can be inferred
  GAS_INTERCEPTOR = 128
  EV = 256
  HW_SDGM = 512  # Camera-side friction brake routing for the Traverse SDGM harness
  MANUAL_AUTO_HOLD = 1024  # Disengaged, driver-armed stationary friction brake only
  # Bolt with factory ACC + comma pedal in StarPilot mode (BoltPedalLongMode 2; StarPilot's BOLT_2022_PEDAL +
  # PANDA_PADDLE_SCHED in one bit): regen paddle and PRNDL2 sent by the panda right after the stock frames, friction brake
  # on PT, CANCEL always allowed toward the camera. Mode 1 (camera long) does not set it.
  BOLT_ACC_PEDAL = 2048
  # Comma between the ASCM and the powertrain bus (StarPilot's ASCM_INT, CHEVROLET_MALIBU_ASCM): forwarded like HW_CAM,
  # with the brake taken from ECMAcceleratorPos (0xBE) as on the ASCM path instead of ECMEngineStatus (0xC9).
  HW_ASCM_INT = 4096
  # HW_ASCM_INT on a car without ECMAcceleratorPos (StarPilot's FORCE_BRAKE_C9 there): brake from 0xC9 and the RX checks
  # take EBCMBrakePedalPosition (0xF1) instead of 0xBE
  ASCM_INT_C9_BRAKE = 8192

@dataclass
class GMCarDocs(CarDocs):
  package: str = "Adaptive Cruise Control (ACC)"

  def init_make(self, CP: CarParams):
    if CP.networkLocation == CarParams.NetworkLocation.fwdCamera:
      self.car_parts = CarParts.common([CarHarness.gm])
    else:
      self.car_parts = CarParts.common([CarHarness.obd_ii])


@dataclass(frozen=True, kw_only=True)
class GMCarSpecs(CarSpecs):
  tireStiffnessFactor: float = 0.444  # not optimized yet


@dataclass
class GMPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {
    Bus.pt: 'gm_global_a_powertrain_volt',
    Bus.radar: 'gm_global_a_object',
    Bus.chassis: 'gm_global_a_chassis',
  })


@dataclass
class GMASCMPlatformConfig(GMPlatformConfig):
  def init(self):
    # ASCM is supported, but due to a janky install and hardware configuration, we are not showing in the car docs
    #self.car_docs = []
    pass



class CAR(Platforms):
  HOLDEN_ASTRA = GMASCMPlatformConfig(
    [GMCarDocs("Holden Astra 2017")],
    GMCarSpecs(mass=1363, wheelbase=2.662, steerRatio=15.7, centerToFrontRatio=0.4),
  )
  CHEVROLET_VOLT = GMASCMPlatformConfig(
    [GMCarDocs("Chevrolet Volt 2017-18", min_enable_speed=0, video="https://youtu.be/QeMCN_4TFfQ")],
    GMCarSpecs(mass=1607, wheelbase=2.69, steerRatio=17.7, centerToFrontRatio=0.55, tireStiffnessFactor=0.469, minEnableSpeed=-1),
  )
  CADILLAC_ATS = GMASCMPlatformConfig(
    [GMCarDocs("Cadillac ATS Premium Performance 2018")],
    GMCarSpecs(mass=1601, wheelbase=2.78, steerRatio=15.3),
  )
  CHEVROLET_MALIBU = GMASCMPlatformConfig(
    [GMCarDocs("Chevrolet Malibu Premier 2017")],
    GMCarSpecs(mass=1496, wheelbase=2.83, steerRatio=15.8, centerToFrontRatio=0.4),
  )
  # StarPilot CHEVROLET_MALIBU_ASCM: the ASCM stays in the car and keeps the stock ACC, the comma sits between it and the
  # powertrain bus and steers. Same CAN as CHEVROLET_MALIBU, so it is chosen from the car list, never fingerprinted.
  CHEVROLET_MALIBU_ASCM = GMPlatformConfig(
    [GMCarDocs("Chevrolet Malibu ASCM Harness 2017-19")],
    CHEVROLET_MALIBU.specs,
  )
  GMC_ACADIA = GMASCMPlatformConfig(
    [GMCarDocs("GMC Acadia 2018", video="https://www.youtube.com/watch?v=0ZN6DdsBUZo")],
    GMCarSpecs(mass=1975, wheelbase=2.86, steerRatio=14.4, centerToFrontRatio=0.4),
  )
  BUICK_LACROSSE = GMASCMPlatformConfig(
    [GMCarDocs("Buick LaCrosse 2017-19", "Driver Confidence Package 2")],
    GMCarSpecs(mass=1712, wheelbase=2.91, steerRatio=15.8, centerToFrontRatio=0.4),
  )
  BUICK_REGAL = GMASCMPlatformConfig(
    [GMCarDocs("Buick Regal Essence 2018")],
    GMCarSpecs(mass=1714, wheelbase=2.83, steerRatio=14.4, centerToFrontRatio=0.4),
  )
  CADILLAC_ESCALADE = GMASCMPlatformConfig(
    [GMCarDocs("Cadillac Escalade 2017", "Driver Assist Package")],
    GMCarSpecs(mass=2564, wheelbase=2.95, steerRatio=17.3),
  )
  CADILLAC_ESCALADE_ESV = GMASCMPlatformConfig(
    [GMCarDocs("Cadillac Escalade ESV 2016", "Adaptive Cruise Control (ACC) & LKAS")],
    GMCarSpecs(mass=2739, wheelbase=3.302, steerRatio=17.3, tireStiffnessFactor=1.0),
  )
  CADILLAC_ESCALADE_ESV_2019 = GMASCMPlatformConfig(
    [GMCarDocs("Cadillac Escalade ESV 2019", "Adaptive Cruise Control (ACC) & LKAS")],
    CADILLAC_ESCALADE_ESV.specs,
  )
  CHEVROLET_BOLT_EUV = GMPlatformConfig(
    [
      GMCarDocs("Chevrolet Bolt EUV 2022-23", "Premier or Premier Redline Trim without Super Cruise Package", video="https://youtu.be/xvwzGMUA210"),
      GMCarDocs("Chevrolet Bolt EV 2022-23", "2LT Trim with Adaptive Cruise Control Package"),
    ],
    GMCarSpecs(mass=1669, wheelbase=2.63779, steerRatio=16.8, centerToFrontRatio=0.4, tireStiffnessFactor=1.0),
  )
  CHEVROLET_SILVERADO = GMPlatformConfig(
    [
      GMCarDocs("Chevrolet Silverado 1500 2020-21", "Safety Package II"),
      GMCarDocs("GMC Sierra 1500 2020-21", "Driver Alert Package II", video="https://youtu.be/5HbNoBLzRwE"),
    ],
    GMCarSpecs(mass=2450, wheelbase=3.75, steerRatio=16.3, tireStiffnessFactor=1.0),
  )
  CHEVROLET_EQUINOX = GMPlatformConfig(
    [GMCarDocs("Chevrolet Equinox 2019-22")],
    GMCarSpecs(mass=1588, wheelbase=2.72, steerRatio=14.4, centerToFrontRatio=0.4),
  )
  CHEVROLET_TRAILBLAZER = GMPlatformConfig(
    [GMCarDocs("Chevrolet Trailblazer 2021-22")],
    GMCarSpecs(mass=1345, wheelbase=2.64, steerRatio=16.8, centerToFrontRatio=0.4, tireStiffnessFactor=1.0),
  )
  CADILLAC_XT4 = GMPlatformConfig(
    [GMCarDocs("Cadillac XT4 2023", "Driver Assist Package")],
    CarSpecs(mass=1660, wheelbase=2.78, steerRatio=14.4, centerToFrontRatio=0.4),
  )
  CHEVROLET_VOLT_2019 = GMPlatformConfig(
    [GMCarDocs("Chevrolet Volt 2019", "Adaptive Cruise Control (ACC) & LKAS")],
    GMCarSpecs(mass=1607, wheelbase=2.69, steerRatio=15.7, centerToFrontRatio=0.45),
  )
  CHEVROLET_TRAVERSE = GMPlatformConfig(
    [GMCarDocs("Chevrolet Traverse 2022-23", "RS, Premier, or High Country Trim")],
    CarSpecs(mass=2090, wheelbase=3.07, steerRatio=17.9, centerToFrontRatio=0.7),
  )
  # Separate car def is required when there is no ASCM
  # (for now) unless there is a way to detect it when it has been unplugged...
  CHEVROLET_VOLT_CC = GMPlatformConfig(
    [GMCarDocs("Chevrolet Volt LT 2017-18")],
    CHEVROLET_VOLT.specs,
  )
  CHEVROLET_BOLT_CC = GMPlatformConfig(
    [
      GMCarDocs("Chevrolet Bolt EUV 2022-23 - No-ACC"),
      GMCarDocs("Chevrolet Bolt EV 2017-23 - No-ACC"),
    ],
    CHEVROLET_BOLT_EUV.specs,
  )
  # A Bolt with the factory camera ACC and a comma pedal. The BoltPedalLongMode Param picks the longitudinal path when
  # CarParams are built (user request 2026-11-19):
  # 1 (default): the Bolt EUV's camera long (0x2CB gas/regen, 0x315 friction brake, 0x370, carrot's LongControl and
  #   settings), with the pedal only tapping the accelerator to leave the ECM's ACC standstill when starting.
  # 2: StarPilot's CHEVROLET_BOLT_ACC_2022_2023_PEDAL (firestar5683/StarPilot 2589caad): the pedal for gas, the spoofed
  #   regen paddle and the friction brake for deceleration, StarPilot's LongControl, the camera's ACC cancelled.
  # Steering is the Bolt No-ACC's (33 Hz, Bolt EUV NNFF donor) in both.
  CHEVROLET_BOLT_ACC_PEDAL = GMPlatformConfig(
    [
      GMCarDocs("Chevrolet Bolt EUV 2022-23 - ACC w Pedal"),
      GMCarDocs("Chevrolet Bolt EV 2022-23 - ACC w Pedal"),
    ],
    CHEVROLET_BOLT_EUV.specs,
  )
  CHEVROLET_EQUINOX_CC = GMPlatformConfig(
    [GMCarDocs("Chevrolet Equinox NO ACC 2019-22")],
    CHEVROLET_EQUINOX.specs,
  )
  CHEVROLET_SUBURBAN = GMPlatformConfig(
    [GMCarDocs("Chevrolet Suburban Premier 2016-20")],
    CarSpecs(mass=2731, wheelbase=3.302, steerRatio=17.3, centerToFrontRatio=0.49),
  )
  CHEVROLET_SUBURBAN_CC = GMPlatformConfig(
    [GMCarDocs("Chevrolet Suburban 2016-20")],
    CHEVROLET_SUBURBAN.specs,
  )
  GMC_YUKON_CC = GMPlatformConfig(
    [GMCarDocs("GMC Yukon No ACC")],
    CarSpecs(mass=2541, wheelbase=2.95, steerRatio=16.3, centerToFrontRatio=0.4),
  )
  CADILLAC_CT6_CC = GMPlatformConfig(
    [GMCarDocs("Cadillac CT6 No ACC")],
    CarSpecs(mass=2358, wheelbase=3.11, steerRatio=17.7, centerToFrontRatio=0.4),
  )
  CHEVROLET_TRAILBLAZER_CC = GMPlatformConfig(
    [GMCarDocs("Chevrolet Trailblazer NO ACC 2021-22")],
    CHEVROLET_TRAILBLAZER.specs,
  )
  CHEVROLET_MALIBU_CC = GMPlatformConfig(
    [GMCarDocs("Chevrolet Malibu No ACC")],
    CarSpecs(mass=1450, wheelbase=2.8, steerRatio=15.8, centerToFrontRatio=0.4),
  )
  CADILLAC_XT5_CC = GMPlatformConfig(
    [GMCarDocs("Cadillac XT5 No ACC")],
    CarSpecs(mass=1810, wheelbase=2.86, steerRatio=16.34, centerToFrontRatio=0.5),
  )
  BUICK_BABYENCLAVE = GMPlatformConfig(
    [GMCarDocs("Buick Baby Enclave 2020-23", "Driver Assist Package")],
    CarSpecs(mass=2050, wheelbase=2.86, steerRatio=16.0, centerToFrontRatio=0.5),
  )
  CHEVROLET_TRAX = GMPlatformConfig(
    [GMCarDocs("Chevrolet TRAX 2024")],
    CarSpecs(mass=1365, wheelbase=2.7, steerRatio=16.1, centerToFrontRatio=0.7),
  )
  CADILLAC_CT6_ACC = GMPlatformConfig(
    [GMCarDocs("CT6-2019 Advanced ACC", "Adaptive Cruise Control (ACC)")],
    GMCarSpecs(mass=1736, wheelbase=3.11, steerRatio=17.7, centerToFrontRatio=0.4),
  )
  GMC_YUKON = GMPlatformConfig(
    [GMCarDocs("GMC Yukon 2019-20", "Adaptive Cruise Control (ACC) & LKAS")],
    GMCarSpecs(mass=2490, wheelbase=2.94, steerRatio=17.3, centerToFrontRatio=0.5, tireStiffnessFactor=1.0),
  )


class CruiseButtons:
  INIT = 0
  UNPRESS = 1
  RES_ACCEL = 2
  DECEL_SET = 3
  MAIN = 5
  CANCEL = 6
  GAP_DIST = 7

class AccState:
  OFF = 0
  ACTIVE = 1
  STANDBY = 2
  FAULTED = 3
  STANDSTILL = 4

class CanBus:
  POWERTRAIN = 0
  OBSTACLE = 1
  CAMERA = 2
  CHASSIS = 2
  LOOPBACK = 128
  DROPPED = 192

class GMFlags(IntFlag):
  PEDAL_LONG = 1
  CC_LONG = 2
  NO_CAMERA = 4
  NO_ACCELERATOR_POS_MSG = 8
  SPEED_RELATED_MSG = 16
  # Carrot Web ChevroletSteerFF choice these CarParams were built with (opendbc.car.interfaces): at most one is set.
  # Kept in CarParams so an interface rebuilt from saved CarParams (controlsd) always matches their lateral tuning.
  STEER_FF_TRAILBLAZER_NNFF = 256
  STEER_FF_VOLT_NNFF = 512
  STEER_FF_VOLT_LINEAR = 1024


# In a Data Module, an identifier is a string used to recognize an object,
# either by itself or together with the identifiers of parent objects.
# Each returns a 4 byte hex representation of the decimal part number. `b"\x02\x8c\xf0'"` -> 42790951
GM_BOOT_SOFTWARE_PART_NUMER_REQUEST = b'\x1a\xc0'  # likely does not contain anything useful
GM_SOFTWARE_MODULE_1_REQUEST = b'\x1a\xc1'
GM_SOFTWARE_MODULE_2_REQUEST = b'\x1a\xc2'
GM_SOFTWARE_MODULE_3_REQUEST = b'\x1a\xc3'

# Part number of XML data file that is used to configure ECU
GM_XML_DATA_FILE_PART_NUMBER = b'\x1a\x9c'
GM_XML_CONFIG_COMPAT_ID = b'\x1a\x9b'  # used to know if XML file is compatible with the ECU software/hardware

# This DID is for identifying the part number that reflects the mix of hardware,
# software, and calibrations in the ECU when it first arrives at the vehicle assembly plant.
# If there's an Alpha Code, it's associated with this part number and stored in the DID $DB.
GM_END_MODEL_PART_NUMBER_REQUEST = b'\x1a\xcb'
GM_END_MODEL_PART_NUMBER_ALPHA_CODE_REQUEST = b'\x1a\xdb'
GM_BASE_MODEL_PART_NUMBER_REQUEST = b'\x1a\xcc'
GM_BASE_MODEL_PART_NUMBER_ALPHA_CODE_REQUEST = b'\x1a\xdc'
GM_FW_RESPONSE = b'\x5a'

GM_FW_REQUESTS = [
  GM_BOOT_SOFTWARE_PART_NUMER_REQUEST,
  GM_SOFTWARE_MODULE_1_REQUEST,
  GM_SOFTWARE_MODULE_2_REQUEST,
  GM_SOFTWARE_MODULE_3_REQUEST,
  GM_XML_DATA_FILE_PART_NUMBER,
  GM_XML_CONFIG_COMPAT_ID,
  GM_END_MODEL_PART_NUMBER_REQUEST,
  GM_END_MODEL_PART_NUMBER_ALPHA_CODE_REQUEST,
  GM_BASE_MODEL_PART_NUMBER_REQUEST,
  GM_BASE_MODEL_PART_NUMBER_ALPHA_CODE_REQUEST,
]

GM_RX_OFFSET = 0x400

FW_QUERY_CONFIG = FwQueryConfig(
  requests=[request for req in GM_FW_REQUESTS for request in [
    Request(
      [StdQueries.SHORT_TESTER_PRESENT_REQUEST, req],
      [StdQueries.SHORT_TESTER_PRESENT_RESPONSE, GM_FW_RESPONSE + bytes([req[-1]])],
      rx_offset=GM_RX_OFFSET,
      bus=0,
      logging=True,
    ),
  ]],
  extra_ecus=[(Ecu.fwdCamera, 0x24b, None)],
)

EV_CAR = {CAR.CHEVROLET_VOLT, CAR.CHEVROLET_VOLT_2019, CAR.CHEVROLET_BOLT_EUV, CAR.CHEVROLET_VOLT_CC, CAR.CHEVROLET_BOLT_CC,
          CAR.CHEVROLET_BOLT_ACC_PEDAL}
CC_ONLY_CAR = {CAR.CHEVROLET_VOLT_CC, CAR.CHEVROLET_BOLT_CC, CAR.CHEVROLET_EQUINOX_CC, CAR.CHEVROLET_SUBURBAN_CC, CAR.GMC_YUKON_CC, CAR.CADILLAC_CT6_CC, CAR.CHEVROLET_TRAILBLAZER_CC, CAR.CADILLAC_XT5_CC, CAR.CHEVROLET_MALIBU_CC}
CC_REGEN_PADDLE_CAR = {CAR.CHEVROLET_BOLT_CC}
# BoltPedalLongMode values for CHEVROLET_BOLT_ACC_PEDAL with a pedal (2 behaves as a CC-only pedal-long car)
BOLT_PEDAL_LONG_MODE_HYBRID = 1
BOLT_PEDAL_LONG_MODE_STARPILOT = 2

def is_bolt_acc_pedal_starpilot(CP) -> bool:
  """The Bolt ACC with pedal in StarPilot mode (BoltPedalLongMode 2): the interface sets PEDAL_LONG only then."""
  return CP.carFingerprint == CAR.CHEVROLET_BOLT_ACC_PEDAL and bool(getattr(CP, "flags", 0) & GMFlags.PEDAL_LONG)


def is_bolt_camera_long(CP) -> bool:
  """The Bolt EUV/EV 2022-23 with the factory camera ACC on openpilot camera long (0x2CB/0x315/0x370 through its ECM):
  the Bolt ACC with pedal in BoltPedalLongMode 1, and the same car without a pedal (CHEVROLET_BOLT_EUV, or
  CHEVROLET_BOLT_ACC_PEDAL with no pedal). Not the pedal-long setups (PEDAL_LONG: mode 2, or the Bolt EUV with a pedal),
  which cancel the ECM's ACC. They share one ECM, so its stop, resume and engage rules apply to all of them."""
  return (str(getattr(CP, "carFingerprint", "")) in (CAR.CHEVROLET_BOLT_EUV, CAR.CHEVROLET_BOLT_ACC_PEDAL) and
          bool(getattr(CP, "openpilotLongitudinalControl", False)) and
          not int(getattr(CP, "flags", 0)) & GMFlags.PEDAL_LONG.value)


def is_cc_only(CP) -> bool:
  """CC_ONLY_CAR, plus the Bolt ACC with pedal in StarPilot mode, which runs as a CC-only pedal-long car."""
  return CP.carFingerprint in CC_ONLY_CAR or is_bolt_acc_pedal_starpilot(CP)


# LKA command timing taken from FrogPilot GM (~33 Hz, +10/-15). Measured on the Traverse; the Trax and
# Trailblazer use the same timing at the user's request. The Bolt No-ACC joins them (user request 2026-11-17): in a
# 75 km/h Bolt EUV No-ACC log the +5 ramp (0.42/s) left the sent torque at 0.05 while the controller asked for 1.0 and
# the car drifted over the outer line of a curve; a replay with the fitted steering response cut the tracking lag
# from 0.60 to 0.28-0.30 s. Not yet checked in the car; the fingerprint also covers the Bolt EV 2017-23 No-ACC.
# The Bolt ACC with pedal shares the Bolt No-ACC's steering, so it takes the same timing; so does the Bolt EUV with the
# factory ACC and no pedal (CHEVROLET_BOLT_EUV, user request 2026-11-28: the same car as the Bolt ACC with pedal).
# The Malibu ASCM harness steers like the Traverse (user request 2026-11-19); StarPilot also runs its GM cars at this timing.
# The other Chevrolets that borrow the Traverse's Trailblazer NNFF model (Malibu, Malibu CC, Volt 2019, Volt CC) take the
# same timing so their steering matches the Traverse too (user request 2026-11-19). Not checked in these cars.
FROGPILOT_STEER_TIMING_CAR = {CAR.CHEVROLET_TRAVERSE, CAR.CHEVROLET_TRAILBLAZER, CAR.CHEVROLET_TRAILBLAZER_CC, CAR.CHEVROLET_TRAX,
                              CAR.CHEVROLET_BOLT_CC, CAR.CHEVROLET_BOLT_ACC_PEDAL, CAR.CHEVROLET_BOLT_EUV, CAR.CHEVROLET_MALIBU_ASCM,
                              CAR.CHEVROLET_MALIBU, CAR.CHEVROLET_MALIBU_CC, CAR.CHEVROLET_VOLT_2019, CAR.CHEVROLET_VOLT_CC}
# We're integrated at the Safety Data Gateway Module on these cars
SDGM_CAR = {CAR.CADILLAC_XT4, CAR.CHEVROLET_TRAVERSE, CAR.BUICK_BABYENCLAVE, CAR.CHEVROLET_VOLT_2019}

# We're integrated at the camera with VOACC on these cars (instead of ASCM w/ OBD-II harness)
CAMERA_ACC_CAR = {CAR.CHEVROLET_BOLT_EUV, CAR.CHEVROLET_BOLT_ACC_PEDAL, CAR.CHEVROLET_SILVERADO, CAR.CHEVROLET_EQUINOX, CAR.CHEVROLET_TRAILBLAZER, CAR.CHEVROLET_TRAX}
CAMERA_ACC_CAR.update({CAR.CHEVROLET_VOLT_CC, CAR.CHEVROLET_BOLT_CC, CAR.CHEVROLET_EQUINOX_CC, CAR.GMC_YUKON_CC, CAR.CADILLAC_CT6_CC, CAR.CHEVROLET_TRAILBLAZER_CC, CAR.CADILLAC_XT5_CC, CAR.CHEVROLET_MALIBU_CC})
# CAMERA_ACC_CAR.update(CC_ONLY_CAR)
# We're integrated between the ASCM and the powertrain bus on these cars (StarPilot ASCM_INT): the ASCM keeps the stock ACC
# and radar, the panda forwards both ways except the ASCM's LKAS command, which openpilot replaces.
ASCM_INT_CAR = {CAR.CHEVROLET_MALIBU_ASCM}
# Speeds shown on the comma follow the Malibu cluster, as the Traverse's do (cluster_speed.MALIBU_CLUSTER_RATIO)
MALIBU_CLUSTER_CAR = {CAR.CHEVROLET_MALIBU, CAR.CHEVROLET_MALIBU_CC, CAR.CHEVROLET_MALIBU_ASCM}
# Alt ASCMActiveCruiseControlStatus
ALT_ACCS = {CAR.GMC_YUKON}

STEER_THRESHOLD = 1.0

DBC = CAR.create_dbc_map()

if __name__ == "__main__":
  cars = []
  for platform in CAR:
    for doc in platform.config.car_docs:
      cars.append(doc.name)
  cars.sort()
  for c in cars:
    print(c)
