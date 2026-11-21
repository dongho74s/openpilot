import math
import time

import numpy as np

from openpilot.cereal import log
from openpilot.common.constants import CV
from openpilot.common.filter_simple import MyMovingAverage
from openpilot.common.realtime import DT_MDL
from openpilot.common.params import Params
from openpilot.selfdrive.carrot.bluetooth.model import CommandReader, RUNTIME, read_json
from openpilot.selfdrive.carrot.bluetooth.physical import HOLD_PATH
from openpilot.selfdrive.controls.lib.desire_lib.boundary_indicator import BoundaryIndicator
from openpilot.selfdrive.controls.lib.desire_lib.boundary_lane import MANUAL_INDICATOR_MIN_SPEED
from openpilot.selfdrive.controls.lib.desire_lib.indicator_observer import ModelIndicator
from openpilot.selfdrive.controls.lib.desire_lib.assistant_lane import AssistantLane
from openpilot.selfdrive.controls.lib.desire_lib.tesla_turn import TeslaTurnJourney

from openpilot.selfdrive.controls.lib.desire_lib.constants import (
  LaneChangeState, LaneChangeDirection, TurnDirection,
  LANE_CHANGE_SPEED_MIN, LANE_CHANGE_TIME_MAX,
  BLINKER_NONE, BLINKER_LEFT, BLINKER_RIGHT,
  DESIRES, TURN_DESIRES
)
from openpilot.selfdrive.controls.lib.desire_lib.side_state import SideState
from openpilot.selfdrive.controls.lib.desire_lib.maneuver_classifier import classify_maneuver_type
from openpilot.selfdrive.controls.lib.desire_lib.lever import LEVER_REQUEST_WAIT_S, LaneCrossing, LeverForcedTurn, LeverRequestWait, lever_maneuver

# Without a model-detected crossing, a lever press this early in laneChangeFinishing is still the driver
# re-lighting the lamp of the change that just crossed, not a request for the next one.
STALK_QUEUE_GUARD_S = 0.3
# An accepted App action may wait for interlocks, but must not start unexpectedly much later.
AUTO_APP_WAIT_S = 10.0
# An ATC-only lane-change attempt that died on a transient condition (speed
# dipped below the minimum, the command flickered) may re-enter only after
# the navigation command has stayed live under continuous lateral control
# for this long. Deliberately longer than a resume transient, so a stale
# command never starts a maneuver the moment control returns.
ATC_RETRY_WAIT_S = 2.0

# ── Auto overtake (automatic left-lane overtake) ──────────────────────────
# When the lead car is this much slower than the cruise set speed, start an
# automatic overtake to the left lane: announce with the existing lane-change
# notification (left blinker + HUD), wait OVERTAKE_ANNOUNCE_S, then start the
# lane change through the normal auto-lane-change path. A solid centre line
# (left lane_change_available false) blocks the overtake entirely — it is
# never even announced.
OVERTAKE_SPEED_DIFF_KPH = 20.0
OVERTAKE_ANNOUNCE_S = 2.0
OVERTAKE_ARMED_TIMEOUT_S = 3.0
OVERTAKE_MIN_SPEED_KPH = 60.0
OVERTAKE_MAX_LEAD_DIST_M = 150.0
# Signal-stop guard: when the model plans to stop, a planned stop point this
# close to the lead car counts as stopping FOR the lead (lead following),
# not for a signal. The same 2 m boundary the signal-stop logic already uses
# (lead_in_stop_range) to tell a lead stop from a signal stop.
OVERTAKE_SIGNAL_STOP_LEAD_GAP_M = 2.0

# ── Auto overtake return (return to the original lane after an overtake) ───
# After an automatic left-lane overtake, return to the original (right) lane
# once the overtaken car is confirmed passed. The pass is an ESTIMATE, not a
# measurement: this car has no rear-side sensor — no stock BSM 0x142 on the
# high-speed bus, no radar side leads, and the model only sees forward — so
# "passed" is judged by three overlapping signals (A: no slow lead ahead,
# B: the model right zone has been clear, C: dead-reckoned relative advance
# past the latched lead position) and the return is conservative by default:
# any doubt keeps the car in the left lane.
#
# Rear-side check (user request: keep the code in place even though the
# signal is empty today): the return checks carstate.rightBlindspot, the
# right SideState BSD hold and side-object flag every frame. Today all three
# are always clear on this car, so they only bite when the model/radar sees
# something on the right. When the low-speed CAN sniffer feeds 0x142 into
# carstate.rightBlindspot, the same code starts working with no change here
# (carstate is the single funnel: opendbc/car/gm/carstate.py reads
# BCMBlindSpotMonitor into left/rightBlindspot).
#
# Estimation speed (user request: return at a reasonable time, not too
# late): the dead reckoning uses the measured relative speed, but with a
# floor of OVERTAKE_RETURN_MIN_VREL_MS, and the clearance is one car length
# plus a 3.5 m margin. The trigger already guarantees the lead is at least
# 20 km/h (~5.6 m/s) slower than the set speed and ego drives near the set
# speed, so the true relative speed during a pass is normally above the
# floor — the floor only fills in when the model under-reports the speed
# difference. If the measured relative speed is <= 0 (the lead is not being
# closed on at all) nothing is accumulated, so a lead that speeds up to
# match ego stalls the pass instead of faking it.
OVERTAKE_RETURN_ANNOUNCE_S = 2.0
OVERTAKE_RETURN_ARMED_TIMEOUT_S = 3.0
OVERTAKE_RETURN_MIN_LEFT_LANE_S = 3.0
OVERTAKE_RETURN_TIMEOUT_S = 30.0
OVERTAKE_RETURN_COOLDOWN_S = 15.0
OVERTAKE_RETURN_ZONE_CLEAR_S = 1.0
OVERTAKE_RETURN_ZONE_MAX_M = 50.0
OVERTAKE_RETURN_ZONE_MIN_PROB = 0.5
OVERTAKE_RETURN_CLEARANCE_M = 8.0
OVERTAKE_RETURN_MIN_VREL_MS = 5.0
OVERTAKE_RETURN_LEAD_FAST_MARGIN_MS = 2.0


class DesireHelper:
  def __init__(self, tesla_auto_signals=False, boundary_indicator_supported=False, *, fingerprint='', native_voice_signal=False,
               kick_supported=False):
    self.params = Params()
    self.external_indicator_supported = boundary_indicator_supported
    self.boundary_indicator_supported = boundary_indicator_supported or kick_supported
    self.boundary_indicator_enabled = False
    self.manual_turn_indicator_enabled = False
    journal = read_json(RUNTIME/'boundary-indicator.json', {}) if fingerprint or self.boundary_indicator_supported else {}
    journal = journal if isinstance(journal, dict) else {}
    self.boundary_indicator = BoundaryIndicator(
      journal if self.boundary_indicator_supported or journal.get('voice_fingerprint') == fingerprint else None,
      minimum_speed=MANUAL_INDICATOR_MIN_SPEED)
    self.model_indicator = ModelIndicator()
    self.model_desire_indicator_enabled = False
    self.model_path_indicator_enabled = False
    self.indicator_journal_time = 0.
    self.bluetooth_commands = CommandReader('lane')
    self.assistant_lane = AssistantLane(fingerprint=fingerprint, native_signal=native_voice_signal)
    # Only a car whose carcontroller may kick the lamp (CanfdBlinkerKick: HyundaiSafetyFlags.CANFD_BLINKER_KICK, set by
    # modeld from CarParams) takes lamp ownership for it; every other car keeps the previous lamp interpretation.
    self.kick_supported = kick_supported
    self.manual_indicator_minimum = 0.
    self.frame = 0

    # FSM core
    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none
    self.lane_change_timer = 0.0
    self.lane_change_ll_prob = 1.0
    self.lane_change_delay = 0.0
    self.maneuver_type = "none"  # "none" / "turn" / "lane_change"

    self.desire = log.Desire.none
    self.turn_direction = TurnDirection.none
    self.enable_turn_desires = True
    self.turn_desire_state = False
    self.desire_disable_count = 0
    self.turn_disable_count = 0

    # per-side states
    self.left = SideState("left")
    self.right = SideState("right")

    # blinker/ATC state (원본 변수들 유지)
    self.blinker_ignore = False
    self.driver_blinker_state = BLINKER_NONE
    self.carrot_blinker_state = BLINKER_NONE
    self.carrot_lane_change_count = 0
    self.carrot_cmd_index_last = 0
    self.atc_type = ""
    self.atc_active = 0  # 0: 없음, 1: ATC 동작, 2: 충돌

    # auto lane change
    self.auto_lane_change_enable = False
    self.next_lane_change = False

    # auto overtake (left-lane overtake when the lead is much slower than the
    # cruise set speed). States: "off" -> "announcing" (left blinker on, the
    # existing lane-change notification) -> "armed" (auto-lane-change path
    # armed, waiting for the FSM to start the change) -> "off" once the lane
    # change starts or the arming times out. A solid centre line blocks the
    # overtake before it is even announced. Disabled entirely when the
    # AutoOvertake setting is off (default on).
    self._overtake_state = "off"
    self._overtake_timer = 0.0
    self.auto_overtake_enabled = True
    # Overtaken lead latched at the overtake trigger (the anchor for the
    # return pass estimate; re-latched at the lane-change handoff when the
    # lead is clearly the same car).
    self._overtake_lead_d0 = 0.0
    self._overtake_lead_v0 = 0.0
    # Signal-stop guard for the auto overtake: moving averages matching
    # CarrotPlanner's stop detection (vFilter window 10, xStop median 3).
    self._signal_v_filter = MyMovingAverage(10)
    self._signal_stop_x_filter = MyMovingAverage(3)

    # auto overtake return (return to the original lane after an overtake).
    # States: "off" -> "overtaking" (left lane, watching the pass) -> "delay"
    # (wait LaneChangeDelay after the pass is confirmed) -> "announcing_right"
    # (right blinker, 2 s) -> "armed_right" (auto-lane-change path armed, up
    # to 3 s) -> "off" once the FSM starts the right lane change. Same
    # AutoOvertake toggle as the overtake; no separate setting. One return
    # per overtake; after a return the overtake is paused for
    # OVERTAKE_RETURN_COOLDOWN_S so it does not immediately re-overtake the
    # same slow lead.
    self._return_state = "off"
    self._return_timer = 0.0
    self._return_pending = False
    self._return_change_started = False
    self._return_rel_advance = 0.0
    self._return_zone_clear_timer = 0.0
    self._return_left_lane_timer = 0.0
    self._return_lead_d0 = 0.0
    self._return_lead_v0 = 0.0
    self._return_delay_timer = 0.0
    self._return_cooldown_timer = 0.0

    # turn-signal lever presses (carState.*BlinkerStalkCount): an explicit request for the next lane change
    self.stalk_counts = None
    self.queued_lane_change = BLINKER_NONE
    self.finishing_timer = 0.0
    self.queued_wait = 0.0
    self.lever_wait = LeverRequestWait()
    self.lever_forced_turn = LeverForcedTurn()
    self.crossing = LaneCrossing()
    self.auto_turn_signal = LaneChangeDirection.none
    self.auto_turn_signal_lane_change = False
    self.blinker_hold_direction = LaneChangeDirection.none  # modelV2.meta.laneChangeBlinkerHold
    self.lever_turn = False  # modelV2.meta.leverTurn: the turn desire comes from a latched lever
    self.tesla_auto_signals = tesla_auto_signals
    self.auto_signal_direction = LaneChangeDirection.none
    self.auto_signal_request_id = 0
    self._auto_signal_counter = time.monotonic_ns()
    self._auto_lane_completed = False
    self._auto_lane_direction = LaneChangeDirection.none
    self._auto_app_direction = BLINKER_NONE
    self._auto_app_wait = 0.0
    self._auto_app_deadline = 0.0
    self._auto_atc_type = ""
    self._driver_signal_counts = None
    self._manual_signal_direction = BLINKER_NONE
    self._manual_signal_grace = 0.0
    self.auto_turn_journey = TeslaTurnJourney()

    # keep pulse
    self.keep_pulse_timer = 0.0

    # params
    self.laneChangeNeedTorque = 0
    self.laneChangeBsd = 0
    self.laneLineCheck = 0
    self.laneChangeDelay = 0.0
    self.blinkerLatchedTurn = 0.0  # m/s, 0 = off
    self.laneChangeLeverWait = False

    # misc
    self.prev_desire_enabled = False
    self.desireLog = ""
    # ATC-only retry clock (seconds spent in off with a live ATC-only
    # request under continuous lateral control).
    self._atc_retry_wait = 0.0

    # externally readable flags
    self.lane_change_available_left = False
    self.lane_change_available_right = False

  # ─────────────────────────────────────────────
  # params/model
  # ─────────────────────────────────────────────
  def _update_params_periodic(self):
    if self.kick_supported or self.boundary_indicator_supported and (self.frame == 1 or self.frame % 20 == 0):
      mode = self.params.get_int("AutoBlinkerControl")
      kick = self.params.get_bool("CanfdBlinkerKick")
      self.manual_indicator_minimum = 2. if self.kick_supported and kick else 0.
      supported = self.kick_supported if kick else self.external_indicator_supported
      lamp_mode = supported and mode in (1, 2) and not self.params.get_bool("CanfdBlinkerControl")
      self.boundary_indicator_enabled = lamp_mode and self.params.get_bool("ManualLaneChangeIndicator")
      self.manual_turn_indicator_enabled = lamp_mode and self.params.get_bool("ManualTurnIndicator")
      self.model_desire_indicator_enabled = lamp_mode and self.params.get_bool("ModelDesireIndicator")
      self.model_path_indicator_enabled = lamp_mode and self.params.get_bool("ModelPathIndicator")
      # Low-speed TBT source waits for this ownership acknowledgement BEFORE
      # sending a lamp-only command. Also restores masks after process restart.
      journal = read_json(RUNTIME/'boundary-indicator.json', {})
      stamp, direction = journal.get('time'), journal.get('mask', journal.get('direction'))
      now = time.monotonic()
      if (type(stamp) in (int, float) and math.isfinite(stamp) and self.indicator_journal_time < stamp <= now and
          now-stamp < 20 and type(direction) is int and direction in (1, 2, 3)):
        self.boundary_indicator.own(now, direction)
        self.indicator_journal_time = stamp
    if self.frame % 100 == 0:
      self.laneChangeNeedTorque = self.params.get_int("LaneChangeNeedTorque")
      self.laneChangeBsd = self.params.get_int("LaneChangeBsd")
      self.laneLineCheck = self.params.get_int("LaneLineCheck")
      self.laneChangeDelay = self.params.get_float("LaneChangeDelay") * 0.1
      self.blinkerLatchedTurn = self.params.get_int("BlinkerLatchedTurn") * CV.KPH_TO_MS
      self.laneChangeLeverWait = self.params.get_bool("LaneChangeLeverWait")
      self.auto_overtake_enabled = self.params.get_bool("AutoOvertake")
      # kick_model removed: upstream's common indicator policy deleted its only
      # use (lane-change start lamp kick); CanfdBlinkerKickModelMode is gone.

  def _check_desire_state(self, modeldata, carstate, maneuver_type):
    desire_state = modeldata.meta.desireState
    orientation_rate = abs(modeldata.orientationRate.z[5])
    orientation_rate_future = abs(modeldata.orientationRate.z[15])

    self.turn_desire_state = (desire_state[1] + desire_state[2]) > 0.1

    if maneuver_type == "turn" and abs(carstate.steeringAngleDeg) > 80 and orientation_rate_future < orientation_rate:
      self.turn_disable_count = int(10.0 / DT_MDL)
    else:
      self.turn_disable_count = max(0, self.turn_disable_count - 1)

  # ─────────────────────────────────────────────
  # blinkers/ATC (원본 로직 유지, side 계산은 별개)
  # ─────────────────────────────────────────────
  def _update_driver_blinker(self, carstate, *, valid=True):
    lamps = carstate.leftBlinker * 1 + carstate.rightBlinker * 2
    st = self.boundary_indicator.filter_lamp(lamps)
    if self.tesla_auto_signals or self.kick_supported and self.boundary_indicator.blocked:
      driver_lamps = lamps if self.kick_supported else st
      driver = int(getattr(carstate, "turnSignalDriverDirection", 0))
      driver_valid = valid and bool(getattr(carstate, "turnSignalDriverValid", False))
      counts = (getattr(carstate, "leftBlinkerStalkCount", 0), getattr(carstate, "rightBlinkerStalkCount", 0))
      last, self._driver_signal_counts = self._driver_signal_counts, counts
      left_press, right_press = (counts[0] != last[0], counts[1] != last[1]) if last else (False, False)
      press = (BLINKER_LEFT if left_press else BLINKER_RIGHT) if left_press != right_press else BLINKER_NONE
      if driver_lamps == 3:
        self._manual_signal_direction = BLINKER_NONE
        self._manual_signal_grace = 0.0
      elif not driver_valid:
        self._manual_signal_direction = BLINKER_NONE
        self._manual_signal_grace = 0.0
        st = BLINKER_NONE
      elif driver in (1, 2) or press:
        self._manual_signal_direction = driver or press
        self._manual_signal_grace = 1.0
        st = self._manual_signal_direction
      elif driver_lamps == self._manual_signal_direction and driver_lamps:
        st = self._manual_signal_direction
        self._manual_signal_grace = 0.5
      elif st != 3 and self._manual_signal_grace > DT_MDL:
        self._manual_signal_grace -= DT_MDL
        st = BLINKER_NONE
      else:
        self._manual_signal_direction = BLINKER_NONE
        self._manual_signal_grace = 0.0
        st = BLINKER_NONE
    elif self.kick_supported:
      self._manual_signal_direction = BLINKER_NONE
      self._manual_signal_grace = 0.0
      self._driver_signal_counts = (getattr(carstate, 'leftBlinkerStalkCount', 0),
                                    getattr(carstate, 'rightBlinkerStalkCount', 0))
    changed = st != self.driver_blinker_state
    self.driver_blinker_state = st

    enabled = st in (BLINKER_LEFT, BLINKER_RIGHT)
    if self.laneChangeNeedTorque < 0:
      enabled = False
    return st, changed, enabled

  def _update_stalk(self, carstate, *, valid=True) -> int:
    """Side of a turn-signal lever press since the last update (BLINKER_LEFT/RIGHT), else BLINKER_NONE.

    The lamp-based blinker cannot show a press while the lamp is already flashing (one press flashes
    for ~4 s); the lever counters can, and a counter never misses a press between 20 Hz updates.
    """
    counts = (getattr(carstate, "leftBlinkerStalkCount", None), getattr(carstate, "rightBlinkerStalkCount", None))
    if counts[0] is None or counts[1] is None:
      return BLINKER_NONE
    last, self.stalk_counts = self.stalk_counts, counts
    if last is None:
      return BLINKER_NONE
    if self.tesla_auto_signals or self.kick_supported and self.boundary_indicator.blocked:
      left_press, right_press = counts[0] != last[0], counts[1] != last[1]
      if (carstate.leftBlinker and carstate.rightBlinker) or \
         not valid or not getattr(carstate, "turnSignalDriverValid", False) or left_press == right_press:
        return BLINKER_NONE
      direction = BLINKER_LEFT if left_press else BLINKER_RIGHT
      driver = getattr(carstate, "turnSignalDriverDirection", 0)
      return direction if driver in (0, direction) else BLINKER_NONE
    if counts[0] != last[0]:
      return BLINKER_LEFT
    if counts[1] != last[1]:
      return BLINKER_RIGHT
    return BLINKER_NONE

  def _update_atc_blinker(self, carrotMan, driver_blinker_state, remote=None):
    atc_type = carrotMan.atcType
    atc_blinker_state = BLINKER_NONE

    # 유지 카운트는 DesireHelper에서 관리
    if self.tesla_auto_signals and self._auto_app_direction:
      atc_blinker_state = self._auto_app_direction
    elif self.carrot_lane_change_count > 0:
      atc_blinker_state = self.carrot_blinker_state
    elif not self.tesla_auto_signals and remote in ('laneLeft', 'laneRight'):
      self.carrot_lane_change_count = int(0.2 / DT_MDL)
      self.carrot_blinker_state = BLINKER_LEFT if remote == 'laneLeft' else BLINKER_RIGHT
      atc_blinker_state = self.carrot_blinker_state
    elif not self.tesla_auto_signals and carrotMan.carrotCmdIndex != self.carrot_cmd_index_last and carrotMan.carrotCmd == "LANECHANGE":
      self.carrot_cmd_index_last = carrotMan.carrotCmdIndex
      self.carrot_lane_change_count = int(0.2 / DT_MDL)
      self.carrot_blinker_state = BLINKER_LEFT if carrotMan.carrotArg == "LEFT" else BLINKER_RIGHT
      atc_blinker_state = self.carrot_blinker_state
    elif atc_type in ("turn left", "turn right"):
      if self.atc_active != 2:
        atc_blinker_state = BLINKER_LEFT if atc_type == "turn left" else BLINKER_RIGHT
        self.atc_active = 1
        self.blinker_ignore = False
    elif atc_type in ("fork left", "fork right", "atc left", "atc right"):
      if self.atc_active != 2:
        atc_blinker_state = BLINKER_LEFT if atc_type in ("fork left", "atc left") else BLINKER_RIGHT
        self.atc_active = 1
    else:
      self.atc_active = 0

    # 충돌 시 ATC 무효
    if driver_blinker_state != BLINKER_NONE and atc_blinker_state != BLINKER_NONE and driver_blinker_state != atc_blinker_state:
      atc_blinker_state = BLINKER_NONE
      self.atc_active = 2

    atc_desire_enabled = atc_blinker_state in (BLINKER_LEFT, BLINKER_RIGHT)
    if self.tesla_auto_signals and self._auto_app_direction and self.lane_change_state not in (
      LaneChangeState.off, LaneChangeState.preLaneChange
    ):
      # The FSM now owns execution. Keep the side for torque cancellation, not a second request.
      atc_desire_enabled = False

    # blinker_ignore
    if driver_blinker_state == BLINKER_NONE:
      self.blinker_ignore = False
    if self.blinker_ignore:
      atc_blinker_state = BLINKER_NONE
      atc_desire_enabled = False

    # 타입 변경 1프레임 무시
    if self.atc_type != atc_type and not (self.tesla_auto_signals and self._auto_app_direction):
      atc_desire_enabled = False
    self.atc_type = atc_type

    return atc_blinker_state, atc_desire_enabled

  def _update_auto_app_request(self, carstate, lateral_active, carrot_man, remote, driver_st, stalk_press):
    new_command = carrot_man.carrotCmdIndex != self.carrot_cmd_index_last and carrot_man.carrotCmd == "LANECHANGE"
    if new_command:
      # Consume even rejected commands: recovery must never execute an old App action.
      self.carrot_cmd_index_last = carrot_man.carrotCmdIndex
    requested = (BLINKER_LEFT if remote == "laneLeft" else BLINKER_RIGHT if remote == "laneRight" else
                 BLINKER_LEFT if new_command and carrot_man.carrotArg == "LEFT" else
                 BLINKER_RIGHT if new_command and carrot_man.carrotArg == "RIGHT" else BLINKER_NONE)
    waiting = self.lane_change_state in (LaneChangeState.off, LaneChangeState.preLaneChange)
    now = time.clock_gettime(getattr(time, "CLOCK_BOOTTIME", time.CLOCK_MONOTONIC))
    permitted = (lateral_active and carstate.canValid and not carstate.trailerConnected and
                 not getattr(carstate, "vehicleSensorsInvalid", False) and
                 str(getattr(carstate, "gearShifter", "drive")) == "drive" and
                 carstate.turnSignalDriverValid and not driver_st and not stalk_press and
                 not self._manual_signal_direction and not (carstate.leftBlinker and carstate.rightBlinker) and
                 (not waiting or carstate.vEgo >= LANE_CHANGE_SPEED_MIN) and self.desire_disable_count == 0)
    self._auto_app_wait += DT_MDL if self._auto_app_direction and waiting else 0.0
    expired = waiting and self._auto_app_direction and (
      self._auto_app_wait >= AUTO_APP_WAIT_S or now >= self._auto_app_deadline)
    if not permitted or expired:
      if self._auto_app_direction and not driver_st:
        self.lane_change_state = LaneChangeState.off
        self.lane_change_direction = LaneChangeDirection.none
      self._auto_app_direction = BLINKER_NONE
    if (requested and permitted and not self._auto_app_direction and
        self.lane_change_state == LaneChangeState.off and not self.auto_signal_request_id):
      self._auto_app_direction, self._auto_app_wait = requested, 0.0
      self._auto_app_deadline = now + AUTO_APP_WAIT_S

  # ─────────────────────────────────────────────
  # per-side processing (핵심: 좌/우 모두 매 프레임 계산)
  # ─────────────────────────────────────────────
  def _process_sides(self, carstate, modeldata, radarState):
    # geometry (좌/우)
    # left: outer laneLines[0], current laneLines[1], edge[0], cur_prob laneLineProbs[1]
    self.left.update_lane_geometry(
      modeldata.laneLines[0], modeldata.laneLineProbs[0],
      modeldata.laneLines[1],
      modeldata.roadEdges[0],
      cur_prob=modeldata.laneLineProbs[1],
    )
    # right: outer laneLines[3], current laneLines[2], edge[1], cur_prob laneLineProbs[2]
    self.right.update_lane_geometry(
      modeldata.laneLines[3], modeldata.laneLineProbs[3],
      modeldata.laneLines[2],
      modeldata.roadEdges[1],
      cur_prob=modeldata.laneLineProbs[2],
    )

    # lane line info (HUD용 raw는 기존대로 leftLaneLine/rightLaneLine)
    self.left.update_lane_line_info(carstate.leftLaneLine)
    self.right.update_lane_line_info(carstate.rightLaneLine)

    # BSD 설정
    ignore_bsd = (self.laneChangeBsd < 0)

    # obstacles
    v_ego = carstate.vEgo
    self.left.update_obstacles(v_ego, radarState.leadLeft, carstate.leftBlindspot, ignore_bsd,
                               bsd_hold_sec=2.0, radar_objects=radarState.leadsLeft)
    self.right.update_obstacles(v_ego, radarState.leadRight, carstate.rightBlindspot, ignore_bsd,
                                bsd_hold_sec=2.0, radar_objects=radarState.leadsRight)

    # compute available (include BSD+object)
    if self.laneLineCheck >= 1:
      left_line_ok = self.left.lane_line_info_mod in (0, 5)
      right_line_ok = self.right.lane_line_info_mod in (0, 5)
    else:
      # Hyundai lane color: 0 none, 1 white, 2 yellow, 3 blue. Only yellow blocks a lane change.
      left_line_ok = self.left.lane_line_info_raw // 10 != 2
      right_line_ok = self.right.lane_line_info_raw // 10 != 2
    self.left.compute_lane_change_available(lane_line_info_lt_20=left_line_ok, ignore_bsd=ignore_bsd)
    self.right.compute_lane_change_available(lane_line_info_lt_20=right_line_ok, ignore_bsd=ignore_bsd)

    self.left.update_triggers()
    self.right.update_triggers()

    # externally readable
    self.lane_change_available_left = self.left.lane_change_available
    self.lane_change_available_right = self.right.lane_change_available

  def _get_selected_side(self, blinker_state: int) -> SideState:
    return self.left if blinker_state == BLINKER_LEFT else self.right

  @staticmethod
  def _is_last_lane(side: SideState) -> bool:
    return side.lane_exist_count.counter <= 0 and not side.lane_change_available_geom

  def _model_signal_stop(self, modeldata, carstate, radarState):
    """True when the driving model plans to stop for a signal (red or solid
    yellow light, stop line) ahead, as opposed to stopping behind the lead
    car. Port of CarrotPlanner.check_model_stopping + lead_in_stop_range
    (openpilot/selfdrive/carrot/carrot_functions.py); keep the thresholds in
    sync with that code. Two deliberate differences from the port source:
    the firm-decel suppression is left out (the guard needs the stop intent
    during the whole approach, not only once the car is nearly stopped), and
    a missing or malformed plan returns False, so a bad model frame never
    blocks an overtake. The camera signal pipeline has no yellow class
    (red/green only), so a solid yellow light is covered here through the
    model's stop plan, the same way the car stops at yellow today.
    """
    try:
      x = modeldata.position.x
      y = modeldata.position.y
      v = modeldata.velocity.x
      if len(x) < 32 or len(v) < 1 or len(y) < 1:
        return False
      v_ego_kph = float(carstate.vEgo) * CV.MS_TO_KPH
      v0 = float(v[0])
      model_v = self._signal_v_filter.process(float(v[-1]))
      model_x = float(x[-1])
      y_end = float(y[-1])
    except (AttributeError, TypeError, ValueError, IndexError):
      return False
    lead = getattr(radarState, "leadOne", None)
    lead_ok = bool(lead is not None and getattr(lead, "status", False))
    lead_dist = float(getattr(lead, "dRel", 0.0) or 0.0) if lead_ok else 1000.0
    # Planned stop point (same index CarrotPlanner uses for xStop).
    stop_x = self._signal_stop_x_filter.process(float(x[31]), median=True)
    # The planned stop is for the lead car, not for a signal: the stop point
    # sits within 2 m of the lead. A stopped lead on the open road must
    # still be overtaken; only a stop well before the lead is a signal stop.
    if (lead_ok and lead_dist > 0 and math.isfinite(lead_dist)
            and math.isfinite(stop_x)
            and (lead_dist - stop_x) < OVERTAKE_SIGNAL_STOP_LEAD_GAP_M):
      return False
    # Does the model plan to stop? Same speed bands as check_model_stopping.
    if v_ego_kph < 1.0:
      stop_sign = model_x < 20.0 and model_v < 10.0
    elif v_ego_kph < 82.0:
      stop_sign = (
        model_x < lead_dist - 3.0
        and model_x < np.interp(v0 * CV.MS_TO_KPH, [60.0, 80.0], [120.0, 150.0])
        and ((model_v < 3.0) or (model_v < v0 * 0.7))
        and abs(y_end) < 5.0
      )
    else:
      stop_sign = False
    return bool(stop_sign)

  # ─────────────────────────────────────────────
  # auto overtake
  # ─────────────────────────────────────────────
  def _update_auto_overtake(self, carstate, radarState, lateral_active, signal_stop=False):
    """Automatic left-lane overtake.

    Trigger: engaged, ego speed >= OVERTAKE_MIN_SPEED_KPH, a lead car is
    present and at least OVERTAKE_SPEED_DIFF_KPH slower than the cruise set
    speed, and the left lane change is available (this already encodes the
    solid-line check via LaneLineCheck and the left-side obstacle/BSD check).
    A model-planned signal stop (red or solid yellow light, stop line)
    blocks the trigger: a stopped car at a signal is never overtaken, while
    a stopped car on the open road still is. On trigger, announce with the
    existing lane-change notification (left blinker) for
    OVERTAKE_ANNOUNCE_S, then arm the normal auto-lane-change path so the
    FSM starts the change. Any loss of the trigger condition, a solid line
    appearing, the left lane becoming blocked, or driver intervention
    cancels the overtake before the lane change starts. After arming, the
    state is "armed" (not "off") until the lane change actually starts, so
    a still-slow lead cannot immediately re-trigger a new announcement; if
    the lane change has not started within OVERTAKE_ARMED_TIMEOUT_S the
    arming is withdrawn.
    """
    # Auto overtake off (AutoOvertake setting): never trigger, and drop any
    # announcement/arming already in progress.
    if not self.auto_overtake_enabled:
      if self._overtake_state != "off":
        self._overtake_state = "off"
        self._overtake_timer = 0.0
        self.carrot_lane_change_count = 0
      return
    # Cancel whenever we are not in a state to overtake.
    lane_change_busy = self.lane_change_state not in (LaneChangeState.off, LaneChangeState.preLaneChange)
    driver_blinker = carstate.leftBlinker or carstate.rightBlinker
    if (not lateral_active) or lane_change_busy or driver_blinker:
      # If the FSM just took over the overtake's own left lane change, arm
      # the return monitor (and latch the overtaken lead for the pass
      # estimate). A driver's own change (physical blinker on) never arms it.
      if lane_change_busy and lateral_active and not driver_blinker:
        self._overtake_latch_return_start(radarState)
      self._overtake_state = "off"
      self._overtake_timer = 0.0
      return

    # Lead speed (km/h) vs cruise set speed (km/h).
    lead = getattr(radarState, "leadOne", None)
    lead_ok = bool(lead is not None and getattr(lead, "status", False))
    v_cruise_kph = float(getattr(carstate, "vCruise", 0.0) or 0.0)
    v_ego_kph = float(carstate.vEgo) * CV.MS_TO_KPH
    if lead_ok:
      lead_kph = float(getattr(lead, "vLead", 0.0) or 0.0) * CV.MS_TO_KPH
      lead_dist = float(getattr(lead, "dRel", 0.0) or 0.0)
    else:
      lead_kph = 0.0
      lead_dist = 0.0

    trigger = (
      lead_ok and
      not signal_stop and
      v_ego_kph >= OVERTAKE_MIN_SPEED_KPH and
      v_cruise_kph > 0.0 and
      (v_cruise_kph - lead_kph) >= OVERTAKE_SPEED_DIFF_KPH and
      0.0 < lead_dist <= OVERTAKE_MAX_LEAD_DIST_M and
      # Left lane change available: solid centre line blocks this (via
      # LaneLineCheck) and so does a blocked/occupied left lane.
      self.left.lane_change_available
    )

    if self._overtake_state == "off":
      # No new overtake while a return is being watched/announced/armed, or
      # during the cooldown after a return: otherwise the car would
      # immediately re-overtake the same slow lead it just returned behind.
      if trigger and self._return_state == "off" and self._return_cooldown_timer <= 0.0:
        self._overtake_state = "announcing"
        self._overtake_timer = 0.0
        # Latch the overtaken lead at the trigger: leadOne IS the slow car
        # here (the trigger guarantees it), so its distance/speed are the
        # anchor for the return pass estimate.
        self._overtake_lead_d0 = lead_dist
        self._overtake_lead_v0 = float(getattr(lead, "vLead", 0.0) or 0.0)
        self._return_rel_advance = 0.0
        # Announce with the existing lane-change notification: left blinker.
        # Refresh the count every frame while announcing so it stays on.
        self.carrot_blinker_state = BLINKER_LEFT
        self.carrot_lane_change_count = int(OVERTAKE_ANNOUNCE_S / DT_MDL) + 2
    elif self._overtake_state == "announcing":
      if not trigger:
        # Condition lost (lead sped up, solid line appeared, left blocked,
        # or speed dropped): cancel before any lane change starts.
        self._overtake_state = "off"
        self._overtake_timer = 0.0
        self.carrot_lane_change_count = 0
        return
      # Keep the left blinker (the existing notification) on.
      self.carrot_blinker_state = BLINKER_LEFT
      self.carrot_lane_change_count = int(OVERTAKE_ANNOUNCE_S / DT_MDL) + 2
      self._overtake_timer += DT_MDL
      if self._overtake_timer >= OVERTAKE_ANNOUNCE_S:
        # Arm the normal auto-lane-change path; its trigger and the
        # lane_change_available gate (solid line + obstacles) still apply,
        # so a line that turned solid in the meantime still blocks the start.
        # Stay in "armed" (not "off") until the FSM actually starts the
        # lane change, so the still-slow lead cannot re-trigger a fresh
        # announcement in the next frame.
        self.auto_lane_change_enable = True
        self._overtake_state = "armed"
        self._overtake_timer = 0.0
        self.desireLog = "OVERTAKE:announced->armed left"
    elif self._overtake_state == "armed":
      if lane_change_busy:
        # The FSM has taken over the lane change.
        self._overtake_state = "off"
        self._overtake_timer = 0.0
        return
      if not trigger:
        # Condition lost before the lane change started: withdraw the arming.
        self.auto_lane_change_enable = False
        self._overtake_state = "off"
        self._overtake_timer = 0.0
        self.carrot_lane_change_count = 0
        return
      # Keep the left blinker on while waiting for the lane change to start.
      self.carrot_blinker_state = BLINKER_LEFT
      self.carrot_lane_change_count = int(OVERTAKE_ANNOUNCE_S / DT_MDL) + 2
      self._overtake_timer += DT_MDL
      if self._overtake_timer >= OVERTAKE_ARMED_TIMEOUT_S:
        # The lane change never started (e.g. the target lane was never
        # detected): give up instead of announcing forever.
        self.auto_lane_change_enable = False
        self._overtake_state = "off"
        self._overtake_timer = 0.0
        self.carrot_lane_change_count = 0

  # ─────────────────────────────────────────────
  # auto overtake return
  # ─────────────────────────────────────────────
  def _overtake_latch_return_start(self, radarState):
    """Arm the return monitor when the FSM takes over the overtake's own left lane change.

    Called from _update_auto_overtake when the overtake is dropped because
    the FSM started a lane change. Only arms when that change is the
    overtake's own left change (direction left + the overtake's left
    announcement still on the notification). A driver's own change (physical
    blinker on) never reaches here, and an ATC navigation change is excluded
    so a return is never armed behind a route maneuver.

    The pass anchor: prefer the lead at the handoff when it is clearly the
    same car the overtake triggered on (speed within
    OVERTAKE_RETURN_LEAD_FAST_MARGIN_MS and closer than at the trigger) —
    the anchor is then nearer and the return comes sooner. Otherwise keep
    the trigger anchor (and the advance accumulated since the trigger).
    """
    if self._overtake_state not in ("announcing", "armed"):
      return
    if self.lane_change_direction != LaneChangeDirection.left:
      return
    if self.carrot_blinker_state != BLINKER_LEFT:
      return
    # An ATC lane-change request owns this change, not the overtake: never
    # arm a return behind a navigation maneuver.
    if self.atc_type in ("fork left", "fork right", "atc left", "atc right"):
      return
    lead = getattr(radarState, "leadOne", None)
    if (lead is not None and getattr(lead, "status", False) and
        abs(float(getattr(lead, "vLead", 0.0) or 0.0) - self._overtake_lead_v0) <=
        OVERTAKE_RETURN_LEAD_FAST_MARGIN_MS and
        float(getattr(lead, "dRel", 0.0) or 0.0) <= self._overtake_lead_d0):
      self._return_lead_d0 = float(getattr(lead, "dRel", 0.0) or 0.0)
      self._return_lead_v0 = float(getattr(lead, "vLead", 0.0) or 0.0)
      # Anchor moved to the handoff: restart the advance count there too.
      self._return_rel_advance = 0.0
    else:
      self._return_lead_d0 = self._overtake_lead_d0
      self._return_lead_v0 = self._overtake_lead_v0
    self._return_pending = True
    self._return_change_started = True
    self._return_zone_clear_timer = 0.0
    self._return_left_lane_timer = 0.0
    self._return_delay_timer = 0.0
    self.desireLog = "OVERTAKE_RETURN:armed"

  def _overtake_return_handover(self):
    """The FSM took over the return's right lane change: hand over and start
    the re-overtake cooldown so the car does not immediately re-overtake the
    same slow lead it just returned behind."""
    self._return_state = "off"
    self._return_timer = 0.0
    self._return_pending = False
    self._return_change_started = False
    self._return_cooldown_timer = OVERTAKE_RETURN_COOLDOWN_S
    self.desireLog = "OVERTAKE_RETURN:started"

  def _overtake_return_reset(self):
    """Drop the return state machine (no return for this overtake)."""
    was_announcing = self._return_state in ("announcing_right", "armed_right")
    self._return_state = "off"
    self._return_timer = 0.0
    self._return_pending = False
    self._return_change_started = False
    self._return_rel_advance = 0.0
    self._return_zone_clear_timer = 0.0
    self._return_left_lane_timer = 0.0
    self._return_lead_d0 = 0.0
    self._return_lead_v0 = 0.0
    self._return_delay_timer = 0.0
    if was_announcing and self.carrot_blinker_state == BLINKER_RIGHT:
      # The right-blinker notification was ours: take it down.
      self.carrot_lane_change_count = 0

  def _return_vrel_eff(self, carstate, radarState):
    """Relative closing speed (m/s) used for the pass dead reckoning.

    Measured from the current lead, floored at OVERTAKE_RETURN_MIN_VREL_MS
    while actually closing (the model often under-reports the speed gap), and
    zero when not closing at all (a lead that caught up stalls the pass
    instead of faking it). With no lead the road ahead is clear and the
    floor lets the estimate finish once the zone check is also clear.
    """
    lead = getattr(radarState, "leadOne", None)
    if lead is None or not getattr(lead, "status", False):
      return OVERTAKE_RETURN_MIN_VREL_MS
    v_rel = float(carstate.vEgo) - float(getattr(lead, "vLead", 0.0) or 0.0)
    if v_rel <= 0.0:
      return 0.0
    return max(v_rel, OVERTAKE_RETURN_MIN_VREL_MS)

  def _return_right_zone_edges(self, modeldata):
    """Lateral bounds (model y, +left) of the right lane at the ego position."""
    lines = getattr(modeldata, "laneLines", None) or ()
    if len(lines) >= 4:
      try:
        y_cur = float(lines[2].y[0])
        y_out = float(lines[3].y[0])
        if y_out < y_cur < 0.0:
          return y_cur, y_out
      except (TypeError, ValueError, IndexError, AttributeError):
        pass
    # Fallback when the lane lines are missing: one lane width to the right.
    return -1.6, -5.6

  def _return_right_zone_occupied(self, modeldata):
    """True if the model sees a car in the right (original) lane ahead.

    Same observation as the cluster's model BSD zone (cluster_route_replay
    _update_model_bsd_zone), but a live decision here rather than a display.
    """
    leads = getattr(modeldata, "leadsV3", None) or ()
    y_cur, y_out = self._return_right_zone_edges(modeldata)
    for lead in leads:
      try:
        prob = float(getattr(lead, "prob", 0.0) or 0.0)
        xs = getattr(lead, "x", None)
        ys = getattr(lead, "y", None)
        if not xs or not ys:
          continue
        x = float(xs[0])
        y = float(ys[0])
      except (TypeError, ValueError, IndexError, AttributeError):
        continue
      if prob < OVERTAKE_RETURN_ZONE_MIN_PROB:
        continue
      if not 0.2 < x < OVERTAKE_RETURN_ZONE_MAX_M:
        continue
      if y_out - 0.5 <= y <= y_cur + 0.5:
        return True
    return False

  def _return_right_rear_clear(self, carstate):
    """Rear-side (blind-spot) check for the return. All three inputs must be clear.

    Today on this car they are always clear (no stock BSM 0x142 on the
    high-speed bus, no radar side leads), so they only bite when the
    model/radar sees something on the right — but the check is live: when
    the low-speed CAN sniffer feeds 0x142 into carstate.rightBlindspot, the
    same code starts working with no change here. carstate is the single
    funnel (opendbc/car/gm/carstate.py reads BCMBlindSpotMonitor into
    left/rightBlindspot; SideState folds the same bit into
    bsd_hold_counter / side_object_detected via _process_sides). The raw
    carstate bit is checked directly, even when LaneChangeBsd is set to
    ignore BSD for manual lane changes: the return is a new automatic
    maneuver, so it stays conservative.
    """
    return (not bool(getattr(carstate, "rightBlindspot", False)) and
            self.right.bsd_hold_counter == 0 and
            not self.right.side_object_detected)

  def _return_pass_confirmed(self, carstate, radarState):
    """Three overlapping signals that the overtaken car has been passed.

    A. The lead ahead is no longer the overtaken car (it is gone, or a
       faster lead than the latched one — it outran the overtaken car).
    B. The model right zone has been clear for OVERTAKE_RETURN_ZONE_CLEAR_S.
    C. Dead-reckoned relative advance past the latched lead position by
       OVERTAKE_RETURN_CLEARANCE_M (one car length + margin).
    D. At least OVERTAKE_RETURN_MIN_LEFT_LANE_S spent in the left lane.
    All four must hold; any doubt keeps the car in the left lane.
    """
    lead = getattr(radarState, "leadOne", None)
    lead_ok = bool(lead is not None and getattr(lead, "status", False))
    if lead_ok:
      v_lead = float(getattr(lead, "vLead", 0.0) or 0.0)
      a_ok = v_lead >= self._return_lead_v0 + OVERTAKE_RETURN_LEAD_FAST_MARGIN_MS
    else:
      a_ok = True
    b_ok = self._return_zone_clear_timer >= OVERTAKE_RETURN_ZONE_CLEAR_S
    c_ok = self._return_rel_advance >= self._return_lead_d0 + OVERTAKE_RETURN_CLEARANCE_M
    d_ok = self._return_left_lane_timer >= OVERTAKE_RETURN_MIN_LEFT_LANE_S
    return a_ok and b_ok and c_ok and d_ok

  def _update_overtake_return(self, carstate, modeldata, radarState, lateral_active, carrotMan):
    """Return to the original (right) lane after an automatic overtake.

    States: "off" -> "overtaking" (left lane, watching the pass) -> "delay"
    (wait LaneChangeDelay after the pass is confirmed) -> "announcing_right"
    (right blinker, 2 s) -> "armed_right" (auto-lane-change path armed, up
    to 3 s) -> "off" once the FSM starts the right lane change. The right
    lane change itself runs through the existing FSM gates — no bypass.

    Same AutoOvertake toggle as the overtake; no separate setting. One
    return per overtake; after a return the overtake is paused for
    OVERTAKE_RETURN_COOLDOWN_S. Called unconditionally from update() so it
    can cancel itself (trailer, disengage, AutoOvertake off) instead of
    freezing.
    """
    # Cooldown after a return always ticks down.
    if self._return_cooldown_timer > 0.0:
      self._return_cooldown_timer = max(0.0, self._return_cooldown_timer - DT_MDL)

    # The return is part of the overtake: same AutoOvertake toggle, no
    # separate setting. Off cancels any return in progress.
    if not self.auto_overtake_enabled:
      if self._return_state != "off" or self._return_pending:
        self._overtake_return_reset()
      return

    # Dead reckoning: relative advance since the overtake triggered (or since
    # the lane change started, when the anchor was re-latched at handoff).
    if self._overtake_state in ("announcing", "armed") or self._return_pending or \
        self._return_state in ("overtaking", "delay"):
      self._return_rel_advance += self._return_vrel_eff(carstate, radarState) * DT_MDL

    # Model right-zone clear time (B) and left-lane time (D).
    if self._return_state in ("overtaking", "delay", "announcing_right", "armed_right"):
      if self._return_right_zone_occupied(modeldata):
        self._return_zone_clear_timer = 0.0
      else:
        self._return_zone_clear_timer += DT_MDL
      self._return_left_lane_timer += DT_MDL

    if self._return_state == "off":
      if self._return_pending:
        if self.lane_change_state == LaneChangeState.off:
          # The change ended without a detected crossing (no lane lines, or
          # it was cancelled): drop the pending return.
          self._overtake_return_reset()
        elif self.lane_change_state in (LaneChangeState.laneChangeStarting,
                                        LaneChangeState.laneChangeFinishing) and \
            self.lane_change_direction == LaneChangeDirection.left and \
            self.crossing.since >= 0.0:
          # Centre crossed into the left lane: start watching the pass.
          self._return_pending = False
          self._return_state = "overtaking"
          self._return_timer = 0.0
          self._return_left_lane_timer = 0.0
          self.desireLog = "OVERTAKE_RETURN:overtaking"
      return

    # --- active states: common cancels --------------------------------------
    lane_change_busy = self.lane_change_state in (LaneChangeState.laneChangeStarting,
                                                  LaneChangeState.laneChangeFinishing)
    driver_blinker = bool(carstate.leftBlinker or carstate.rightBlinker)
    # Steering against the return (right): torque > 0 is leftward.
    driver_oppose = bool(carstate.steeringPressed and carstate.steeringTorque > 0)
    atc_live = getattr(carrotMan, "atcType", "") not in ("", "none", "none prepare", "straight", "straight prepare")
    if (not lateral_active) or driver_blinker or driver_oppose or atc_live or \
        carstate.vEgo < LANE_CHANGE_SPEED_MIN or carstate.trailerConnected:
      self._overtake_return_reset()
      return

    # The FSM took over the return's own right lane change. It can start
    # during "announcing_right": the announcement refreshes
    # carrot_lane_change_count (which forces the FSM trigger) and the FSM's
    # own lane_change_delay gate opens first, so the 2 s announcement does
    # not gate the start — exactly like the overtake's left change, which
    # can also start during its announcement. Hand over (do not cancel:
    # cancelling here would skip the re-overtake cooldown and the car would
    # immediately re-overtake the same slow lead).
    if self._return_state in ("announcing_right", "armed_right") and lane_change_busy and \
        self.lane_change_direction == LaneChangeDirection.right:
      self._overtake_return_handover()
      return

    # A new lane change that is not the entry change (and not the return's own
    # change, handled above) cancels the return.
    if self._return_state in ("overtaking", "delay", "announcing_right"):
      if lane_change_busy and not self._return_change_started:
        self._overtake_return_reset()
        return
      if self.lane_change_state in (LaneChangeState.off, LaneChangeState.preLaneChange):
        self._return_change_started = False

    if self._return_state == "overtaking":
      # Give up if the pass cannot be confirmed within the timeout: stay left.
      if self._return_left_lane_timer >= OVERTAKE_RETURN_TIMEOUT_S:
        self._overtake_return_reset()
        return
      if self._return_pass_confirmed(carstate, radarState):
        # Pass confirmed: wait LaneChangeDelay before announcing the return
        # (user request: do not go right right after the lead check).
        self._return_state = "delay"
        self._return_delay_timer = self.laneChangeDelay
        self._return_timer = 0.0
        self.desireLog = "OVERTAKE_RETURN:passed->delay"
      return

    if self._return_state == "delay":
      if self._return_left_lane_timer >= OVERTAKE_RETURN_TIMEOUT_S:
        self._overtake_return_reset()
        return
      if not self._return_pass_confirmed(carstate, radarState):
        # Conditions broke while waiting: back to watching the pass.
        self._return_state = "overtaking"
        self._return_timer = 0.0
        return
      self._return_delay_timer -= DT_MDL
      if self._return_delay_timer <= 0.0:
        self._return_state = "announcing_right"
        self._return_timer = 0.0
        self.carrot_blinker_state = BLINKER_RIGHT
        self.carrot_lane_change_count = int(OVERTAKE_RETURN_ANNOUNCE_S / DT_MDL) + 2
        self.desireLog = "OVERTAKE_RETURN:delay->announcing_right"
      return

    if self._return_state == "announcing_right":
      # Right lane must stay available (solid line / obstacle / BSD blocks).
      if not self.right.lane_change_available or not self._return_right_rear_clear(carstate):
        self._overtake_return_reset()
        return
      if not self._return_pass_confirmed(carstate, radarState):
        # Pass no longer confirmed (e.g. a car reappeared on the right):
        # back to watching instead of announcing.
        self._return_state = "overtaking"
        self._return_timer = 0.0
        self.carrot_lane_change_count = 0
        return
      self.carrot_blinker_state = BLINKER_RIGHT
      self.carrot_lane_change_count = int(OVERTAKE_RETURN_ANNOUNCE_S / DT_MDL) + 2
      self._return_timer += DT_MDL
      if self._return_timer >= OVERTAKE_RETURN_ANNOUNCE_S:
        self.auto_lane_change_enable = True
        self._return_state = "armed_right"
        self._return_timer = 0.0
        self.desireLog = "OVERTAKE_RETURN:announcing_right->armed_right"
      return

    if self._return_state == "armed_right":
      if not self.right.lane_change_available or not self._return_right_rear_clear(carstate):
        self.auto_lane_change_enable = False
        self._overtake_return_reset()
        return
      if not self._return_pass_confirmed(carstate, radarState):
        self.auto_lane_change_enable = False
        self._return_state = "overtaking"
        self._return_timer = 0.0
        self.carrot_lane_change_count = 0
        return
      self.carrot_blinker_state = BLINKER_RIGHT
      self.carrot_lane_change_count = int(OVERTAKE_RETURN_ANNOUNCE_S / DT_MDL) + 2
      self._return_timer += DT_MDL
      if self._return_timer >= OVERTAKE_RETURN_ARMED_TIMEOUT_S:
        self.auto_lane_change_enable = False
        self._overtake_return_reset()
      return

  # ─────────────────────────────────────────────
  # main update
  # ─────────────────────────────────────────────
  def update(self, carstate, modeldata, lateral_active, lane_change_prob, carrotMan, radarState, *,
             indicator_valid=False, indicator_curvature=None, assistant_valid=False, turn_hold_allowed=False, turn_pose=None,
             indicator_yaw_rate=None):
    self.frame += 1
    self._update_params_periodic()
    if self.boundary_indicator_supported:
      indicator_now = time.monotonic()
      # Reset parking maneuver evidence while CANCEL is latched. Keep updating
      # lamp isolation so an OEM one-touch tail cannot become a steering request.
      lamp_busy = getattr(carstate, 'cruiseCancelState', False) or self.lane_change_state in (
        LaneChangeState.laneChangeStarting, LaneChangeState.laneChangeFinishing)
      self.boundary_indicator.update(indicator_now, carstate, modeldata,
        enabled=self.boundary_indicator_enabled, valid=indicator_valid,
        # Navigation preparation is not driver intent. Reserve only an executing
        # lane-change maneuver; model desire probabilities also predict manual moves.
        busy=lamp_busy)
      if self.model_indicator.active and self.model_indicator.active.source == 'manual' and not self.boundary_indicator_enabled:
        self.model_indicator.active = None
      self.model_indicator.update(indicator_now, carstate, modeldata,
        valid=indicator_valid, lateral_active=lateral_active,
        desire_enabled=self.model_desire_indicator_enabled, path_enabled=self.model_path_indicator_enabled,
        busy=lamp_busy,
        isolation=self.boundary_indicator, manual_edge=self.boundary_indicator.request_direction, command_curvature=indicator_curvature,
        feedback=read_json(HOLD_PATH, {}) if self.model_indicator.active else None,
        manual_turn_enabled=self.manual_turn_indicator_enabled, yaw_rate=indicator_yaw_rate,
        manual_minimum=self.manual_indicator_minimum)

    elif self.boundary_indicator.blocked or not self.boundary_indicator.initialized:
      # Expire a voice-owned lamp mask without enabling PE-only geometry observation.
      self.boundary_indicator.update(time.monotonic(), carstate, modeldata, enabled=False, valid=assistant_valid, busy=False)

    # counts
    self.carrot_lane_change_count = max(0, self.carrot_lane_change_count - 1)
    self.lane_change_delay = max(0.0, self.lane_change_delay - DT_MDL)

    v_ego = carstate.vEgo
    below_lane_change_speed = v_ego < LANE_CHANGE_SPEED_MIN
    trailer_maneuver_blocked = carstate.trailerConnected

    prev_lane_change_state = self.lane_change_state
    lever_decided_turn = False

    # per-side compute (좌/우 모두)
    self._process_sides(carstate, modeldata, radarState)
    if trailer_maneuver_blocked:
      self.left.lane_change_available = False
      self.right.lane_change_available = False
      self.lane_change_available_left = False
      self.lane_change_available_right = False
      self.auto_lane_change_enable = False
      self.next_lane_change = False
      self.desireLog = "TRAILER:MANEUVER_BLOCKED"

    # auto overtake (left-lane overtake when the lead is much slower than the
    # cruise set speed). Runs after _process_sides so the left lane's solid-line
    # and obstacle state is known, and before the blinker update so the
    # announcement (left blinker) is picked up in this same frame. A
    # model-planned signal stop (red / solid yellow light, stop line) blocks
    # the overtake; a stopped lead on the open road still triggers it.
    if not trailer_maneuver_blocked:
      signal_stop = self._model_signal_stop(modeldata, carstate, radarState)
      self._update_auto_overtake(carstate, radarState, lateral_active, signal_stop=signal_stop)
    # The return is called unconditionally: it must cancel itself when the
    # trailer connects (or lateral control drops) instead of freezing.
    self._update_overtake_return(carstate, modeldata, radarState, lateral_active, carrotMan)

    # desire state from model
    self._check_desire_state(modeldata, carstate, self.maneuver_type)

    # blinkers
    driver_input_valid = not self.kick_supported or indicator_valid and carstate.canValid
    driver_st, driver_changed, driver_enabled = self._update_driver_blinker(carstate, valid=driver_input_valid)
    physical_stalk_press = self._update_stalk(carstate, valid=driver_input_valid)
    stalk_press = (physical_stalk_press if self.kick_supported and getattr(carstate, 'turnSignalDriverValid', False)
                   else self.boundary_indicator.filter_lamp(physical_stalk_press))
    lever = getattr(carstate, "blinkerLever", 0)
    forced_turn = self.lever_forced_turn.update(lever, getattr(carstate, "blinkerLamp", 0))
    if self.lever_wait.update(self.laneChangeLeverWait, stalk_press, lever, driver_st, self.lane_change_state,
                              self.laneChangeDelay):
      driver_enabled = driver_changed = False  # one-touch request timed out: ignore the lamp until it goes dark
    remote = self.bluetooth_commands.read(allowed=(lateral_active and carstate.canValid and
      not below_lane_change_speed and not trailer_maneuver_blocked))
    # Separate authenticated voice intent; never synthesize physical remote events,
    # steering torque or turn-signal acknowledgement. All normal FSM gates remain.
    assistant_direction = None
    try:
      voice_side = self.assistant_lane.active.get('direction') if self.assistant_lane.active else None
      opposite_torque = carstate.steeringPressed and ((voice_side == 'left' and carstate.steeringTorque < 0) or
                                                     (voice_side == 'right' and carstate.steeringTorque > 0))
      owned_signal = self.assistant_lane.signal_id
      raw_lamps = int(carstate.leftBlinker) | (int(carstate.rightBlinker) << 1)
      expected_lamps = (BLINKER_LEFT if voice_side == 'left' else BLINKER_RIGHT) if owned_signal else BLINKER_NONE
      if owned_signal and raw_lamps and raw_lamps != expected_lamps and not (lever or physical_stalk_press):
        # Unexplained opposite/hazard feedback must not re-enter the FSM as a driver request.
        self.boundary_indicator.own(time.monotonic(), raw_lamps)
        driver_st = BLINKER_NONE
        driver_changed = driver_enabled = False
      signal_driver_busy = not self.tesla_auto_signals and (
        (raw_lamps and raw_lamps != expected_lamps) or
        ((lever or physical_stalk_press) and not self.assistant_lane.own_tap(carstate)))
      assistant_state = {
        'valid': assistant_valid and carstate.canValid and not getattr(carstate, 'vehicleSensorsInvalid', True),
        'lateral_active': lateral_active, 'drive': str(getattr(carstate, 'gearShifter', '')) == 'drive',
        'trailer': trailer_maneuver_blocked, 'speed': v_ego,
        'phase': {LaneChangeState.off: 'off', LaneChangeState.preLaneChange: 'preLaneChange',
                  LaneChangeState.laneChangeStarting: 'laneChangeStarting', LaneChangeState.laneChangeFinishing: 'laneChangeFinishing'
                  }.get(self.lane_change_state, 'unknown'),
        'driver_busy': bool(driver_enabled or stalk_press or opposite_torque),
        'indicator_driver_busy': signal_driver_busy,
        'driver_override': bool(stalk_press or opposite_torque or (owned_signal and signal_driver_busy)),
        'other_request': bool(remote or self.desire_disable_count or self.turn_desire_state or self.turn_direction != TurnDirection.none or
                              carrotMan.atcType not in ('', 'none', 'none prepare', 'straight', 'straight prepare') or
                              (carrotMan.carrotCmd == 'LANECHANGE' and carrotMan.carrotCmdIndex != self.carrot_cmd_index_last)),
      }
      for side_name, side_value in (('left', self.left), ('right', self.right)):
        assistant_state[side_name] = {'available': side_value.lane_change_available and side_value.lane_change_available_geom,
                                     'blindspot': bool(getattr(carstate, side_name + 'Blindspot', True) or side_value.bsd_hold_counter or
                                                      side_value.side_object_detected)}
      assistant_direction, cancel_voice = self.assistant_lane.before(assistant_state)
      if cancel_voice:
        self.carrot_lane_change_count = 0
        self._auto_app_direction = BLINKER_NONE
        if self.lane_change_state == LaneChangeState.preLaneChange:
          self.lane_change_state = LaneChangeState.off
          self.lane_change_direction = LaneChangeDirection.none
      if assistant_direction:
        remote = 'laneLeft' if assistant_direction == 'left' else 'laneRight'
    except (OSError, ValueError, TypeError, KeyError):
      # A failed intent/status channel cannot take down the model loop.
      assistant_direction = None
    if self.tesla_auto_signals:
      self._update_auto_app_request(carstate, lateral_active, carrotMan, remote, driver_st, stalk_press)
    atc_st, atc_enabled = self._update_atc_blinker(carrotMan, driver_st, remote)

    desire_enabled = driver_enabled or atc_enabled
    blinker_state = driver_st if driver_enabled else atc_st

    # 선택된 side (FSM은 이 side만 참고)
    side = self._get_selected_side(blinker_state) if blinker_state in (BLINKER_LEFT, BLINKER_RIGHT) else None
    atc_lane_change_requested = (
      atc_enabled and not self._auto_app_direction and
      self.atc_type in ("fork left", "fork right", "atc left", "atc right")
    )
    atc_lane_change_manual_only = (
      atc_lane_change_requested and
      not driver_enabled and
      self.atc_type in ("fork left", "atc left")
    )
    atc_lane_change_only = atc_lane_change_requested and not driver_enabled
    # Do not treat a blocked->available retry as permission to cross solid/unknown lines.
    # Geometry-based ATC at the last lane still works when the lane/road edge opens up.
    atc_lane_change_retry_line_blocked = (
      atc_lane_change_only and
      side is not None and
      side.lane_line_info_mod not in (0, 5)
    )

    # ATC-only retry clock. While the FSM sits in off with a live ATC-only
    # request and lateral control continuously active, count up; any break
    # (command off, lateral inactive, driver request, state left off)
    # resets the clock. It stays quiet while lateral is inactive, so
    # nothing re-arms on the resume frame itself.
    if self.lane_change_state == LaneChangeState.off and lateral_active and atc_lane_change_only:
      self._atc_retry_wait += DT_MDL
    else:
      self._atc_retry_wait = 0.0

    # auto lane change trigger (기존 로직 유지하되 side 기반)
    auto_lane_change_trigger = False
    if desire_enabled and side is not None and not trailer_maneuver_blocked:
      # carrot_lane_change_count>0이면 강제 허용
      if self.carrot_lane_change_count > 0 or (self.tesla_auto_signals and self._auto_app_direction):
        auto_lane_change_trigger = side.lane_change_available
      else:
        # 기존 조건: edge_available + (trigger or appeared) + not side_object_detected
        auto_lane_change_trigger = (
          self.auto_lane_change_enable and
          (not atc_lane_change_manual_only) and
          side.edge_available and
          (side.lane_available_trigger or side.lane_appeared) and
          (not side.side_object_detected) and
          (side.bsd_hold_counter == 0)
        )
      self.desireLog = (
        f"{side.name}:ALC={self.auto_lane_change_enable}, "
        #f"L={side.lane_available},E={side.edge_available}, "
        #f"T={side.lane_available_trigger},A={side.lane_appeared}, "
        #f"OBJ={side.side_object_detected},BSD={side.bsd_hold_counter>0}"
      )
    else:
      self.auto_lane_change_enable = False
      self.next_lane_change = False

    # ───────────────────────── FSM ─────────────────────────
    if not lateral_active or self.lane_change_timer > LANE_CHANGE_TIME_MAX or trailer_maneuver_blocked:
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
      self.turn_direction = TurnDirection.none
      self.maneuver_type = "none"

    elif self.desire_disable_count > 0:
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
      self.turn_direction = TurnDirection.none
      self.maneuver_type = "none"

    else:
      # classify maneuver type using selected side
      if desire_enabled and side is not None:
        new_type = classify_maneuver_type(
          blinker_state=blinker_state,
          carstate=carstate,
          side=side,
          turn_desire_state=self.turn_desire_state,
          atc_type=self.atc_type,
          old_type=self.maneuver_type,
        )
      else:
        new_type = "none"
      if self.tesla_auto_signals and self._auto_app_direction:
        new_type = "lane_change"
      if self.assistant_lane.active and not self.assistant_lane.active['began'] and new_type != 'lane_change':
        try:
          self.assistant_lane.publish('rejected', 'not_a_lane_change')
        except OSError:
          pass
        self.carrot_lane_change_count = 0
        self._auto_app_direction = BLINKER_NONE
        desire_enabled = False
        new_type = 'none'

      # BlinkerLatchedTurn: the driver's lever replaces the speed/lane-line guess (see lever_maneuver).
      lever_type, lever_undecided = (None, False)
      if driver_enabled and side is not None:
        lever_type, lever_undecided = lever_maneuver(lever, v_ego, self.blinkerLatchedTurn, forced_turn)
      if lever_type is not None:
        new_type = lever_type
      lever_decided_turn = lever_type == "turn"

      if trailer_maneuver_blocked and new_type in ("lane_change", "turn"):
        new_type = "none"

      # switching rules
      if self.maneuver_type == "lane_change" and new_type == "turn" and self.lane_change_state not in (
        LaneChangeState.preLaneChange, LaneChangeState.laneChangeStarting
      ):
        self.maneuver_type = "turn"
        self.lane_change_state = LaneChangeState.off
      elif self.lane_change_state in (LaneChangeState.off, LaneChangeState.preLaneChange):
        self.maneuver_type = new_type

      # ─ TURN mode ─
      if desire_enabled and self.maneuver_type == "turn" and self.enable_turn_desires:
        self.lane_change_state = LaneChangeState.off
        if self.turn_disable_count > 0:
          self.turn_direction = TurnDirection.none
          self.lane_change_direction = LaneChangeDirection.none
        else:
          self.turn_direction = TurnDirection.turnLeft if blinker_state == BLINKER_LEFT else TurnDirection.turnRight
          self.lane_change_direction = self.turn_direction

      # ─ Lane change FSM ─
      else:
        self.turn_direction = TurnDirection.none

        if self.lane_change_state == LaneChangeState.off:
          # A lever press while the lamp is still flashing has no lamp edge but is still a fresh request.
          driver_desire_started = driver_enabled and (driver_changed or stalk_press == blinker_state)
          # ATC-only re-entry: a live navigation command whose attempt died
          # on a transient condition may try again once the retry clock has
          # run out. A fresh driver request still re-arms at once; the
          # rising edge for a fresh ATC command is unchanged.
          atc_retry_ready = atc_lane_change_only and self._atc_retry_wait >= ATC_RETRY_WAIT_S
          if desire_enabled and (not self.prev_desire_enabled or driver_desire_started or atc_retry_ready) and \
             not below_lane_change_speed and side is not None:
            self.lane_change_state = LaneChangeState.preLaneChange
            self.lane_change_ll_prob = 1.0
            self.lane_change_delay = self.laneChangeDelay

            # 맨 끝 차선이 아니면, ATC 자동 차선변경 비활성
            # (원본 유지: 차선 존재하거나 geom 가능하면 auto off, 아니면 on)
            self.auto_lane_change_enable = self._is_last_lane(side)
            self.next_lane_change = False

        elif self.lane_change_state == LaneChangeState.preLaneChange:
          if side is None:
            self.lane_change_state = LaneChangeState.off
            self.lane_change_direction = LaneChangeDirection.none
          else:
            self.lane_change_direction = LaneChangeDirection.left if blinker_state == BLINKER_LEFT else LaneChangeDirection.right
            # An explicit lever press asks for this change: no steering-torque confirmation needed.
            if self.next_lane_change and driver_enabled and stalk_press == blinker_state:
              self.next_lane_change = False

            # torque direction cond
            torque_cond = (carstate.steeringTorque > 0) if blinker_state == BLINKER_LEFT else (carstate.steeringTorque < 0)
            torque_applied = carstate.steeringPressed and torque_cond

            # BSD config
            ignore_bsd = (self.laneChangeBsd < 0)
            block_lanechange_bsd = (self.laneChangeBsd == 1)
            bsd_active = (side.bsd_hold_counter > 0) and (not ignore_bsd)
            side_clear_without_line = (side.lane_available or side.edge_available) and \
                                      (not side.side_object_detected) and (not bsd_active)
            atc_driver_confirm = atc_lane_change_requested and driver_enabled
            atc_geometry_release = atc_lane_change_only and auto_lane_change_trigger
            atc_line_release = (atc_driver_confirm or atc_geometry_release) and side_clear_without_line

            # Arm automatic ATC only after this side has actually become the last lane.
            # Keep it latched so a newly appearing lane can start the maneuver later.
            if atc_lane_change_only and self._is_last_lane(side):
              self.auto_lane_change_enable = True

            if not desire_enabled or below_lane_change_speed:
              self.lane_change_state = LaneChangeState.off
              self.lane_change_direction = LaneChangeDirection.none
            else:
              # 차선변경 시작 조건:
              # - side.lane_change_available는 BSD+object 포함(요구사항)
              # - 하지만 BSD 중에도 torque override 허용해야 하므로, BSD 분기를 별도로 둠(원본 동작 유지)
              # LaneLineCheck=2: 실선에서도 토크 override 허용
              solid_line_blocked = (self.laneLineCheck >= 2) and (not side.lane_change_available_geom) and \
                                   (side.lane_available or side.edge_available)
              block_released = side.lane_change_available_released
              # A BSD/radar release must not start pure ATC unless ATC was armed at a last lane.
              # Driver blinkers retain the existing retry behavior.
              block_released_auto = block_released and (driver_enabled or self.auto_lane_change_enable) and \
                                    not atc_lane_change_retry_line_blocked
              start_gate = (side.lane_change_available_geom and self.lane_change_delay == 0) or \
                           side.lane_line_info_edge_detect or solid_line_blocked or block_released_auto or atc_line_release
              # Only automatic Tesla requests wait for their own echoed, physically confirmed lamp session.
              signal_ready = (not self.tesla_auto_signals or driver_enabled or
                              (self.auto_signal_request_id != 0 and
                               getattr(carstate, "autoTurnSignalReadyRequestId", 0) == self.auto_signal_request_id))
              can_start = False
              if start_gate and not lever_undecided and signal_ready:
                if solid_line_blocked:
                  if atc_line_release or (torque_applied and not (bsd_active and block_lanechange_bsd)):
                    can_start = True
                elif bsd_active:
                  if torque_applied and (not block_lanechange_bsd):
                    can_start = True
                elif self.laneChangeNeedTorque > 0 or self.next_lane_change:
                  if torque_applied:
                    can_start = True
                elif driver_enabled:
                  # driver blinker면 바로 시작(원본 유지)
                  # 단, object/bzd 막힘은 side.lane_change_available에서 걸림
                  if side.lane_change_available or atc_line_release:
                    can_start = True
                else:
                  if torque_applied or ((not atc_lane_change_manual_only) and (
                    auto_lane_change_trigger or side.lane_line_info_edge_detect or block_released_auto
                  )):
                    # 여기서는 시작 직전 안전성 체크
                    if side.lane_change_available or atc_line_release:
                      can_start = True
              if can_start and assistant_direction and not self.tesla_auto_signals and self.assistant_lane.signal_route != 'ordinary':
                # Reserve our lamp before publishing its intent. A delayed blink must never become driver input.
                self.boundary_indicator.own(time.monotonic(), blinker_state)
                can_start = self.assistant_lane.request_signal(assistant_direction, carstate)
              elif self.assistant_lane.signal_id:
                # Once armed, losing admission cancels the attempt rather than rearming a delayed maneuver.
                can_start = False
                self.assistant_lane.publish('rejected', 'lane_conditions_changed')
                self.carrot_lane_change_count = 0
                self.lane_change_state = LaneChangeState.off
                self.lane_change_direction = LaneChangeDirection.none
              if can_start:
                self.lane_change_state = LaneChangeState.laneChangeStarting

        elif self.lane_change_state == LaneChangeState.laneChangeStarting:
          # A press before the car centre crossed the line re-lights this change's lamp and is ignored (never a second
          # lane at once); one after it asks for the next change. The model keeps the lane-change probability up
          # ~1.5 s past the crossing, so waiting for laneChangeFinishing missed them (2026-10-05 drive: 4/4 next-lane
          # presses came 0.1-1.1 s after the crossing, all still in laneChangeStarting).
          since_crossing = self.crossing.update(modeldata, self.lane_change_direction == LaneChangeDirection.left)
          if stalk_press != BLINKER_NONE and since_crossing >= 0.0:
            self.queued_lane_change = stalk_press
          self.lane_change_ll_prob = max(self.lane_change_ll_prob - 2 * DT_MDL, 0.0)
          if lane_change_prob < 0.02 and self.lane_change_ll_prob < 0.01:
            self.lane_change_state = LaneChangeState.laneChangeFinishing
            self.finishing_timer = 0.0

        elif self.lane_change_state == LaneChangeState.laneChangeFinishing:
          self.finishing_timer += DT_MDL
          crossed = self.crossing.since >= 0.0  # the crossing was seen during laneChangeStarting
          if stalk_press != BLINKER_NONE and (crossed or self.finishing_timer >= STALK_QUEUE_GUARD_S):
            self.queued_lane_change = stalk_press
          self.lane_change_ll_prob = min(self.lane_change_ll_prob + DT_MDL, 1.0)
          if self.lane_change_ll_prob > 0.99:
            self.lane_change_direction = LaneChangeDirection.none
            if desire_enabled:
              self.lane_change_state = LaneChangeState.preLaneChange
              # Holding the blinker through the change still needs torque for the next one; a lever press
              # made during finishing (same side as the lamp) is an explicit request and does not.
              self.next_lane_change = not (driver_enabled and self.queued_lane_change == blinker_state)
              self.queued_wait = 0.0
            else:
              self.lane_change_state = LaneChangeState.off

    # timer
    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.preLaneChange):
      self.lane_change_timer = 0.0
    else:
      self.lane_change_timer += DT_MDL

    # commit last per-side
    self.left.commit_last()
    self.right.commit_last()

    self.prev_desire_enabled = desire_enabled

    # 반대 방향 토크로 cancel (기존 유지)
    steering_pressed_cancel = carstate.steeringPressed and (
      (carstate.steeringTorque < 0 and blinker_state == BLINKER_LEFT) or
      (carstate.steeringTorque > 0 and blinker_state == BLINKER_RIGHT)
    )
    if steering_pressed_cancel and self.lane_change_state != LaneChangeState.off:
      self.lane_change_direction = LaneChangeDirection.none
      self.lane_change_state = LaneChangeState.off
      self.blinker_ignore = True

    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.preLaneChange):
      self.crossing = LaneCrossing()
    # A queued request lives until the next change starts; waiting in preLaneChange (blocked) it gets the same
    # 5 flashes as a one-touch press, and it ends if the lamp side changes or everything goes off.
    if self.lane_change_state == LaneChangeState.preLaneChange and self.queued_lane_change != BLINKER_NONE:
      self.queued_wait += DT_MDL
    if self.lane_change_state == LaneChangeState.off or        (self.lane_change_state == LaneChangeState.laneChangeStarting and
        prev_lane_change_state == LaneChangeState.preLaneChange) or        (self.lane_change_state == LaneChangeState.preLaneChange and
        (self.queued_wait >= LEVER_REQUEST_WAIT_S or self.queued_lane_change != blinker_state)):
      self.queued_lane_change = BLINKER_NONE

    hold = self.lever_wait.update_hold(self.lane_change_state, blinker_state)
    if self.queued_lane_change != BLINKER_NONE:
      # The lamp of a press made during a change may stop before that change ends (2026-10-05: ~2.5 s after the
      # press), which would drop the request just as it becomes due: keep it lit until the queued change starts.
      hold = self.queued_lane_change
    self.blinker_hold_direction = {BLINKER_LEFT: LaneChangeDirection.left,
                                   BLINKER_RIGHT: LaneChangeDirection.right}.get(hold, LaneChangeDirection.none)

    # final desire
    if self.turn_direction != TurnDirection.none:
      self.desire = TURN_DESIRES[self.turn_direction]
      self.lane_change_direction = self.turn_direction
    else:
      self.desire = DESIRES[self.lane_change_direction][self.lane_change_state]
    self.lever_turn = lever_decided_turn and self.turn_direction != TurnDirection.none
    if self.tesla_auto_signals:
      self._update_auto_signal(carstate, carrotMan, lateral_active, atc_enabled, atc_st, driver_st, prev_lane_change_state,
                               turn_hold_allowed, turn_pose)
    if self.tesla_auto_signals and (self.lane_change_state == LaneChangeState.off or self._auto_lane_completed):
      self._auto_app_direction = BLINKER_NONE

    try:
      phase = {LaneChangeState.off: 'off', LaneChangeState.preLaneChange: 'preLaneChange',
               LaneChangeState.laneChangeStarting: 'laneChangeStarting', LaneChangeState.laneChangeFinishing: 'laneChangeFinishing'
               }.get(self.lane_change_state, 'unknown')
      direction = {LaneChangeDirection.left: 'left', LaneChangeDirection.right: 'right'}.get(self.lane_change_direction, 'none')
      self.assistant_lane.after(phase, direction, crossed=self.crossing.since >= 0)
    except (OSError, ValueError, TypeError, KeyError):
      pass  # Missing feedback is not completion; the caller must report unknown.

    # keep pulse
    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.laneChangeStarting):
      self.keep_pulse_timer = 0.0
    elif self.lane_change_state == LaneChangeState.preLaneChange:
      self.keep_pulse_timer += DT_MDL
      if self.keep_pulse_timer > 1.0:
        self.keep_pulse_timer = 0.0
      elif self.desire in (log.Desire.keepLeft, log.Desire.keepRight):
        self.desire = log.Desire.none

    # Origin is latched before injected lamps can look like driver input.
    # preLaneChange is deliberately excluded: indicate only actual lane changes.
    turning = self.desire in (log.Desire.turnLeft, log.Desire.turnRight)
    changing = self.lane_change_state in (LaneChangeState.laneChangeStarting, LaneChangeState.laneChangeFinishing)
    continuing = changing if self.auto_turn_signal_lane_change else turning
    if not continuing or self.lane_change_direction != self.auto_turn_signal:
      self.auto_turn_signal = LaneChangeDirection.none
    if self.auto_turn_signal == LaneChangeDirection.none and atc_enabled and not driver_enabled and self.atc_active == 1:
      if turning and self.atc_type in ("turn left", "turn right"):
        self.auto_turn_signal = self.lane_change_direction
        self.auto_turn_signal_lane_change = False
      elif self.lane_change_state == LaneChangeState.laneChangeStarting and atc_lane_change_only:
        self.auto_turn_signal = self.lane_change_direction
        self.auto_turn_signal_lane_change = True
    return self.desire

  def _update_auto_signal(self, carstate, carrot_man, lateral_active, atc_enabled, atc_st, driver_st, previous,
                          turn_hold_allowed, turn_pose):
    direction = LaneChangeDirection.none
    if self.atc_type != self._auto_atc_type:
      self._auto_lane_completed = False
    self._auto_atc_type = self.atc_type
    if previous == LaneChangeState.laneChangeFinishing and self.lane_change_state != previous:
      self._auto_lane_completed = True
    if self.lane_change_state == LaneChangeState.laneChangeStarting and previous != self.lane_change_state:
      self._auto_lane_completed = False
    if ((self.carrot_lane_change_count > 0 or self._auto_app_direction) and previous == LaneChangeState.off and
        self.lane_change_state == LaneChangeState.preLaneChange):
      self._auto_lane_completed = False
    common = (self.tesla_auto_signals and not getattr(carstate, "cruiseCancelState", False) and carstate.canValid and not carstate.trailerConnected and
                 not getattr(carstate, "vehicleSensorsInvalid", False) and
                 str(getattr(carstate, "gearShifter", "drive")) == "drive" and carstate.turnSignalDriverValid and
                 not driver_st and not self._manual_signal_direction and
                 not (carstate.leftBlinker and carstate.rightBlinker) and self.desire_disable_count == 0)
    stopped_hold = (self.auto_turn_journey.direction and turn_hold_allowed and getattr(carstate, "standstill", False) and
                    math.isfinite(carstate.vEgo) and abs(carstate.vEgo) < .1 and self.auto_signal_request_id and
                    getattr(carstate, "autoTurnSignalReadyRequestId", 0) == self.auto_signal_request_id)
    turn_desire = (1 if self.desire == log.Desire.turnLeft else 2 if self.desire == log.Desire.turnRight else 0) if atc_enabled else 0
    direction = self.auto_turn_journey.update(time.monotonic(), carstate, turn_pose, self.atc_type,
      getattr(carrot_man, "xDistToTurn", None), turn_desire, common and (lateral_active or stopped_hold), lateral_active)
    permitted = common and lateral_active
    if not permitted or self.lane_change_state == LaneChangeState.off or self._auto_lane_completed:
      if not permitted and self._auto_lane_direction != LaneChangeDirection.none:
        self._auto_lane_completed = True
      self._auto_lane_direction = LaneChangeDirection.none
    elif (atc_enabled and self._auto_lane_direction == LaneChangeDirection.none and
          (self.lane_change_state == LaneChangeState.preLaneChange or previous == LaneChangeState.preLaneChange)):
      # The existing FSM owns the maneuver, including a bounded App preparation wait.
      self._auto_lane_direction = LaneChangeDirection.left if atc_st == BLINKER_LEFT else LaneChangeDirection.right
    if permitted and not direction and not self.auto_turn_journey.consumed:
      if self._auto_lane_direction != LaneChangeDirection.none:
        direction = self._auto_lane_direction
      elif (not self._auto_lane_completed and self.atc_active != 2 and not self.blinker_ignore and
            self.atc_type in ("atc left", "atc right") and getattr(carrot_man, "xTurnInfo", 0) in (1, 2) and
            0 < getattr(carrot_man, "xDistToTurn", 0) <= min(250, max(40, carstate.vEgo * 8 + 20))):
        direction = LaneChangeDirection.left if carrot_man.xTurnInfo == 1 else LaneChangeDirection.right
    if direction != LaneChangeDirection.none and direction != self.auto_signal_direction:
      self._auto_signal_counter += 1
    self.auto_signal_direction = direction
    self.auto_signal_request_id = self._auto_signal_counter if direction != LaneChangeDirection.none else 0
