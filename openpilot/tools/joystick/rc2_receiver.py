#!/usr/bin/env python3
"""Drive joystick debug mode from a DJI RC 2 on the same network.

The RC 2 app (rc2-openpilot-joystick) broadcasts on its network to find the comma, then sends
authenticated stick packets over UDP (see rc2_protocol). This process turns them into testJoystick
for joystickd and sends the car's state back for the RC 2 screen.

manager runs it while the Carrot web switch Rc2RemoteControl is on. Whether a drive runs in RC 2
joystick mode is decided once at ignition (process_config), so flipping the switch mid-drive never
swaps controlsd and joystickd under a moving car.

Nothing here relaxes openpilot's own checks: selfdrived still needs the driver to engage with the
cruise buttons and still disengages on brake, seatbelt, door and driver monitoring, and the panda
keeps its torque and accel limits. On top of that this process brakes gently and centres the wheel
whenever the RC 2 link is lost, the dead-man button is released or the stick data goes stale. Drops of the
dead-man or freshness flag shorter than glitch_hold keep the last command, and after a stop longer than
recenter_after it waits for both sticks to return to centre before passing them through again.
"""
import argparse
import os
import socket
import time
from dataclasses import dataclass

from openpilot.tools.joystick import rc2_protocol as proto

JOYSTICKD_ACCEL_GAIN = 4.0  # joystickd: actuators.accel = 4.0 * axes[0]
CONTROL_HZ = 100
STATUS_HZ = 5
HANDSHAKE_IDLE_S = 1.0  # a new app session may replace the current one only after this much silence
PEER_IDLE_S = 3.0       # stop sending STATUS to an app that went quiet
PARAM = "Rc2RemoteControl"


@dataclass(frozen=True)
class Limits:
  # Full stick asks for what openpilot lets the car do; the car controller and the panda still clip it
  # (Traverse: +2.0 / -4.0 m/s^2 and its EPS torque limit). No speed cap unless one is set.
  accel_max: float = 2.0         # m/s^2 at full forward stick (openpilot / GM ACCEL_MAX)
  brake_max: float = 4.0         # m/s^2 at full back stick (joystickd's full range, GM ACCEL_MIN)
  failsafe_decel: float = 1.5    # m/s^2 while the link is lost, disarmed or stale
  speed_cap: float | None = None  # m/s; when set, forward stick adds no accel above it
  steer_scale: float = 1.0       # fraction of the car's max steer torque at full stick
  link_timeout: float = 0.5      # s without a valid CONTROL packet (Wi-Fi stalls of a few hundred ms happen)
  stick_timeout_ms: int = 500    # age of the DJI stick sample the app last read (the DJI stream is ~10 Hz)
  glitch_hold: float = 0.25      # s: dead-man/fresh flags dropping this briefly keep the last command
  recenter_after: float = 1.0    # s: only a stop longer than this needs both sticks back at centre
  deadzone: float = 0.05
  steer_expo: float = 0.0        # linear: half stick is half torque
  center_tol: float = 0.05


def shape(value: float, deadzone: float, expo: float = 0.0) -> float:
  value = max(-1.0, min(1.0, value))
  if abs(value) <= deadzone:
    return 0.0
  scaled = (abs(value) - deadzone) / (1.0 - deadzone)
  scaled = expo * scaled ** 3 + (1.0 - expo) * scaled
  return scaled if value > 0 else -scaled


class Rc2Link:
  """Handshake, replay protection and safety shaping. Free of cereal so tests can drive it on a Mac."""

  def __init__(self, key: bytes, limits: Limits = Limits(), rng=os.urandom):
    self.key = key
    self.limits = limits
    self.rng = rng
    self.session: int | None = None
    self.nonce: bytes | None = None
    self.peer = None
    self.last_seq = -1
    self.last_rx = -1e9
    self.last_heard = -1e9
    self.ctrl: proto.Control | None = None
    self.need_center = True
    self.rejected = 0
    self.status_seq = 0
    self.last_good: tuple[float, float] | None = None  # last command passed through
    self.last_good_t = -1e9
    self.bad_since: float | None = None  # when the current stop began
    self.in_glitch = False
    self.glitches = 0  # flag drops bridged by glitch_hold

  def handle(self, data: bytes, now: float, addr=None) -> bytes | None:
    """Feed one datagram; returns a reply to send back to its sender, if any."""
    kind = proto.packet_type(data)
    if kind == proto.HELLO:
      session = proto.decode_hello(self.key, data)
      if session is None:
        self.rejected += 1
        return None
      if session == self.session and self.nonce is not None:
        self.peer, self.last_heard = addr, now
        return proto.encode_challenge(self.key, session, self.nonce)  # periodic re-hello, or a lost challenge
      if now - self.last_rx < HANDSHAKE_IDLE_S:
        return None  # keep the live session; a replayed HELLO must not knock it off
      self.session, self.nonce, self.peer, self.last_heard = session, self.rng(proto.NONCE_LEN), addr, now
      self.last_seq, self.ctrl, self.need_center = -1, None, True
      self.last_good, self.bad_since = None, None
      return proto.encode_challenge(self.key, session, self.nonce)

    if kind == proto.CONTROL and self.nonce is not None:
      ctrl = proto.decode_control(self.key, self.nonce, data)
      if ctrl is None or ctrl.session != self.session or ctrl.seq <= self.last_seq:
        self.rejected += 1
        return None
      self.last_seq, self.last_rx, self.last_heard, self.ctrl = ctrl.seq, now, now, ctrl
      if addr is not None:
        self.peer = addr
      return None

    self.rejected += 1
    return None

  def _blocked(self, now: float) -> str | None:
    lim, ctrl = self.limits, self.ctrl
    if ctrl is None:
      return "no_link"
    if now - self.last_rx > lim.link_timeout:
      return "link_lost"
    if not ctrl.armed:
      return "disarmed"
    if not ctrl.fresh or ctrl.stick_age_ms > lim.stick_timeout_ms:
      return "stale_sticks"
    return None

  def axes(self, now: float, v_ego: float) -> tuple[float, float, str]:
    """testJoystick axes [gas/brake, steer] for joystickd, and the link code (rc2_protocol.LINK_CODES)."""
    lim, ctrl = self.limits, self.ctrl
    reason = self._blocked(now)
    if reason is not None:
      # a missed DJI frame or a flicker of the dead-man bit must not jab the brakes: keep the last command briefly
      if reason in ("disarmed", "stale_sticks") and self.last_good is not None and now - self.last_good_t <= lim.glitch_hold:
        if not self.in_glitch:
          self.glitches += 1
          self.in_glitch = True
        return self.last_good[0], self.last_good[1], "ok"
      if self.bad_since is None:
        self.bad_since = now
      self.last_good = None
      return -lim.failsafe_decel / JOYSTICKD_ACCEL_GAIN, 0.0, reason

    self.in_glitch = False
    if self.bad_since is not None:
      # short stops resume where they left off; after a real pause both sticks must come back to centre first
      if now - self.bad_since > lim.recenter_after:
        self.need_center = True
      self.bad_since = None

    if self.need_center:
      if abs(ctrl.accel) > lim.center_tol or abs(ctrl.steer) > lim.center_tol:
        return -lim.failsafe_decel / JOYSTICKD_ACCEL_GAIN, 0.0, "center_sticks"
      self.need_center = False

    stick_accel = shape(ctrl.accel, lim.deadzone)
    accel = stick_accel * (lim.accel_max if stick_accel > 0 else lim.brake_max)
    if lim.speed_cap is not None:
      if v_ego > lim.speed_cap:
        accel = min(accel, 0.0)
      if v_ego > lim.speed_cap + 1.0:
        accel = min(accel, -0.5)

    # openpilot steers left for positive torque; the app sends stick right as positive
    steer = -shape(ctrl.steer, lim.deadzone, lim.steer_expo) * lim.steer_scale
    out = accel / JOYSTICKD_ACCEL_GAIN, max(-1.0, min(1.0, steer))
    self.last_good, self.last_good_t = out, now
    return out[0], out[1], "ok"

  def status_packet(self, now: float, **fields) -> tuple[bytes, object] | None:
    """A STATUS datagram and where to send it, while an app is connected."""
    if self.nonce is None or self.peer is None or now - self.last_heard > PEER_IDLE_S:
      return None
    self.status_seq += 1
    st = proto.Status(session=self.session, seq=self.status_seq, **fields)
    return proto.encode_status(self.key, self.nonce, st), self.peer


def car_status(sm, params, remote_drive: bool) -> dict:
  cs, ss, ds = sm['carState'], sm['selfdriveState'], sm['deviceState']
  state = 0
  for on, bit in ((ds.started, proto.ST_ONROAD), (remote_drive, proto.ST_REMOTE_DRIVE), (ss.enabled, proto.ST_ENABLED),
                  (ss.active, proto.ST_ACTIVE), (cs.brakePressed, proto.ST_BRAKE), (cs.gasPressed, proto.ST_GAS),
                  (cs.doorOpen, proto.ST_DOOR_OPEN), (cs.seatbeltUnlatched, proto.ST_SEATBELT_OFF),
                  (cs.standstill, proto.ST_STANDSTILL), (cs.cruiseState.available, proto.ST_CRUISE_AVAILABLE),
                  (params.get_bool(PARAM), proto.ST_TOGGLE_ON),
                  (cs.steerFaultTemporary or cs.steerFaultPermanent, proto.ST_STEER_FAULT)):
    if on:
      state |= bit
  text = " / ".join(t for t in (ss.alertText1, ss.alertText2) if t)
  return {"v_ego": cs.vEgo, "steering_angle": cs.steeringAngleDeg, "gear": int(cs.gearShifter.raw), "state": state,
          "max_temp_c": int(ds.maxTempC), "text": text}


def run(args, limits: Limits) -> None:
  if not os.path.isfile(args.key):
    proto.make_key(args.key)
    print(f"created a new key at {args.key}; pair the RC 2 app with it")
  key = proto.load_key(args.key)
  sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
  sock.bind((args.bind, args.port))
  sock.setblocking(False)
  link = Rc2Link(key, limits)

  pm = sm = params = cloudlog = messaging = None
  if not args.dry_run:
    from openpilot.cereal import messaging
    from openpilot.common.params import Params
    from openpilot.common.swaglog import cloudlog
    pm = messaging.PubMaster(['testJoystick'])
    sm = messaging.SubMaster(['carState', 'selfdriveState', 'controlsState', 'deviceState'])
    params = Params()
  print(f"rc2_receiver listening on udp {args.bind}:{args.port}" + (" (dry run)" if args.dry_run else ""), flush=True)

  period = 1.0 / CONTROL_HZ
  next_t = time.monotonic()
  last_status, last_print, last_tx = None, 0.0, 0.0
  while True:
    now = time.monotonic()
    for _ in range(64):
      try:
        data, addr = sock.recvfrom(512)
      except OSError:  # nothing waiting (BlockingIOError) or a transient network error
        break
      reply = link.handle(data, now, addr)
      if reply is not None:
        try:
          sock.sendto(reply, addr)
        except OSError:
          pass

    v_ego = 0.0
    if sm is not None:
      sm.update(0)
      v_ego = sm['carState'].vEgo
    gb, steer, status = link.axes(now, v_ego)

    if pm is not None:
      msg = messaging.new_message('testJoystick')
      msg.valid = True
      msg.testJoystick.axes = [gb, steer]
      pm.send('testJoystick', msg)

    if now - last_tx >= 1.0 / STATUS_HZ:
      last_tx = now
      if sm is not None:
        remote_drive = sm['controlsState'].lateralControlState.which() == 'debugState'
        fields = car_status(sm, params, remote_drive)
      else:
        fields = {"text": "컴퓨터 시험 모드 · 차 없음"}  # shown on the RC 2 screen
      out = link.status_packet(now, link=status, accel_cmd=gb * JOYSTICKD_ACCEL_GAIN, steer_cmd=steer,
                               speed_cap_kph=round(limits.speed_cap * 3.6) if limits.speed_cap else 0, **fields)
      if out is not None:
        try:
          sock.sendto(*out)
        except OSError:
          pass  # the app's network went away; it will say hello again

    # under manager only status changes are printed; a dry run also prints once a second
    if status != last_status or (args.dry_run and now - last_print > 1.0):
      ctrl = link.ctrl
      stick = f"({ctrl.accel:+.2f},{ctrl.steer:+.2f}) age={ctrl.stick_age_ms}ms" if ctrl else "(none)"
      where = f"peer={link.peer} rejected={link.rejected}"
      print(f"{status:13s} gb={gb:+.3f} steer={steer:+.3f} v={v_ego * 3.6:5.1f}km/h stick={stick} {where}", flush=True)
      if cloudlog is not None and status != last_status:
        cloudlog.event("rc2_joystick_status", status=status, prev=last_status, glitches=link.glitches,
                       flags=ctrl.flags if ctrl else -1, stick_age_ms=ctrl.stick_age_ms if ctrl else -1,
                       rx_age_ms=int((now - link.last_rx) * 1000))
      last_status, last_print = status, now

    next_t += period
    time.sleep(max(0.0, next_t - time.monotonic()))
    if time.monotonic() - next_t > 0.5:
      next_t = time.monotonic()


def main() -> None:
  d = Limits()
  parser = argparse.ArgumentParser(description="DJI RC 2 -> testJoystick bridge for joystick debug mode")
  parser.add_argument("--key", default=os.environ.get("RC2_JOYSTICK_KEY", proto.DEFAULT_KEY_PATH))
  parser.add_argument("--make-key", action="store_true", help="create a new key file (refuses to overwrite) and exit")
  parser.add_argument("--bind", default="0.0.0.0")
  parser.add_argument("--port", type=int, default=proto.DEFAULT_PORT)
  parser.add_argument("--dry-run", action="store_true", help="print the axes instead of publishing testJoystick")
  parser.add_argument("--accel-max", type=float, default=d.accel_max, help="m/s^2 at full forward stick")
  parser.add_argument("--brake-max", type=float, default=d.brake_max, help="m/s^2 at full back stick")
  parser.add_argument("--speed-cap-kph", type=float, default=0.0, help="0 = no cap")
  parser.add_argument("--steer-scale", type=float, default=d.steer_scale)
  args, _ = parser.parse_known_args()  # manager starts us with its own argv

  if args.make_key:
    proto.make_key(args.key)
    print(f"wrote a new key to {args.key}")
    return

  # joystickd can only express +-4 m/s^2 (4.0 * axis)
  limits = Limits(accel_max=min(max(args.accel_max, 0.0), JOYSTICKD_ACCEL_GAIN),
                  brake_max=min(max(args.brake_max, 0.5), JOYSTICKD_ACCEL_GAIN),
                  speed_cap=args.speed_cap_kph / 3.6 if args.speed_cap_kph > 0 else None,
                  steer_scale=min(max(args.steer_scale, 0.0), 1.0))
  run(args, limits)


if __name__ == "__main__":
  main()
