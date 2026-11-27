import pytest

from openpilot.tools.joystick import rc2_protocol as proto
from openpilot.tools.joystick.rc2_receiver import JOYSTICKD_ACCEL_GAIN, PEER_IDLE_S, Limits, Rc2Link, shape

KEY = bytes(range(32))
NONCE = bytes(range(16, 32))
FAILSAFE = -Limits().failsafe_decel / JOYSTICKD_ACCEL_GAIN

# Shared with the RC 2 app's ProtocolTest.java so both ends agree on the bytes.
VECTOR_CONTROL = proto.Control(session=0x01020304, seq=7, accel=0.5, steer=-0.25, stick_age_ms=40, buttons=0x40,
                               flags=proto.FLAG_ARMED | proto.FLAG_FRESH)
VECTOR_HEX = {
  "hello": "5243324a01010000040302014e11fe1f88d7bd5a3a0a92d4ae0c297f",
  "challenge": "5243324a0102000004030201101112131415161718191a1b1c1d1e1f41f0eb4b58f9da1d30585d343832b24b",
  "control": "5243324a010303000403020107000000f40106ff28004000051c2c647f1583952b6784919782ca55",
  "status": "5243324a010400000403020109000000a10185ff020507006afffa000f3ded95b8eb93a4ec9d8420ec9ea1ec9cbcec84b8ec9a94"
            + "00" * 74 + "3cc5a759e48843da9b39ecb7cf3ebe00",
}
VECTOR_STATUS = proto.Status(session=0x01020304, seq=9, v_ego=4.17, steering_angle=-12.3, gear=2, link="center_sticks",
                             state=proto.ST_ONROAD | proto.ST_REMOTE_DRIVE | proto.ST_ENABLED, accel_cmd=-1.5,
                             steer_cmd=0.25, speed_cap_kph=15, max_temp_c=61, text="핸들을 잡으세요")


def fixed_rng(n):
  return NONCE[:n]


class Session:
  """An RC 2 app talking to an Rc2Link through the real wire format."""

  def __init__(self, link, key=KEY, session=0xABCD):
    self.link, self.key, self.session, self.seq, self.nonce = link, key, session, 0, None

  def hello(self, now):
    reply = self.link.handle(proto.encode_hello(self.key, self.session), now)
    if reply is not None:
      decoded = proto.decode_challenge(self.key, reply)
      assert decoded is not None and decoded[0] == self.session
      self.nonce = decoded[1]
    return reply

  def control(self, now, accel=0.0, steer=0.0, armed=True, fresh=True, age=40, seq=None):
    self.seq = self.seq + 1 if seq is None else seq
    flags = (proto.FLAG_ARMED if armed else 0) | (proto.FLAG_FRESH if fresh else 0)
    ctrl = proto.Control(self.session, self.seq, accel, steer, age, 0, flags)
    return proto.encode_control(self.key, self.nonce, ctrl)

  def send(self, now, **kw):
    self.link.handle(self.control(now, **kw), now)


def connected(limits=None):
  link = Rc2Link(KEY, limits or Limits(), rng=fixed_rng)
  app = Session(link)
  assert app.hello(0.0) is not None
  app.send(0.0)  # centred sticks clear the start-up centre check
  assert link.axes(0.0, 0.0)[2] == "ok"
  return link, app


class TestProtocol:
  def test_round_trip(self):
    assert proto.decode_hello(KEY, proto.encode_hello(KEY, 42)) == 42
    assert proto.decode_challenge(KEY, proto.encode_challenge(KEY, 42, NONCE)) == (42, NONCE)
    assert proto.decode_control(KEY, NONCE, proto.encode_control(KEY, NONCE, VECTOR_CONTROL)) == VECTOR_CONTROL

  def test_vectors_match_app(self):
    assert proto.encode_hello(KEY, VECTOR_CONTROL.session).hex() == VECTOR_HEX["hello"]
    assert proto.encode_challenge(KEY, VECTOR_CONTROL.session, NONCE).hex() == VECTOR_HEX["challenge"]
    assert proto.encode_control(KEY, NONCE, VECTOR_CONTROL).hex() == VECTOR_HEX["control"]
    assert proto.encode_status(KEY, NONCE, VECTOR_STATUS).hex() == VECTOR_HEX["status"]

  def test_status_round_trip(self):
    data = proto.encode_status(KEY, NONCE, VECTOR_STATUS)
    assert len(data) == proto.STATUS_LEN
    assert proto.decode_status(KEY, NONCE, data) == VECTOR_STATUS
    assert proto.decode_status(KEY, bytes(16), data) is None

  def test_status_text_is_cut_on_a_character_boundary(self):
    st = proto.Status(1, 1, text="가" * 40)  # 120 bytes of UTF-8
    out = proto.decode_status(KEY, NONCE, proto.encode_status(KEY, NONCE, st))
    assert out.text == "가" * 32

  @pytest.mark.parametrize("index", range(proto.CONTROL_LEN))
  def test_any_flipped_byte_is_rejected(self, index):
    data = bytearray(proto.encode_control(KEY, NONCE, VECTOR_CONTROL))
    data[index] ^= 0x01
    assert proto.decode_control(KEY, NONCE, bytes(data)) is None

  def test_wrong_key_or_nonce_is_rejected(self):
    data = proto.encode_control(KEY, NONCE, VECTOR_CONTROL)
    assert proto.decode_control(bytes(32), NONCE, data) is None
    assert proto.decode_control(KEY, bytes(16), data) is None
    assert proto.decode_hello(bytes(32), proto.encode_hello(KEY, 1)) is None

  def test_axes_are_clamped(self):
    ctrl = proto.Control(1, 1, 3.0, -3.0, 70000, 0x1FFFF, proto.FLAG_ARMED)
    out = proto.decode_control(KEY, NONCE, proto.encode_control(KEY, NONCE, ctrl))
    assert (out.accel, out.steer, out.stick_age_ms, out.buttons) == (1.0, -1.0, 0xFFFF, 0xFFFF)

  def test_make_key_never_overwrites(self, tmp_path):
    path = str(tmp_path / "sub" / "key")
    proto.make_key(path)
    key = proto.load_key(path)
    assert len(key) == proto.KEY_LEN
    assert (tmp_path / "sub" / "key").stat().st_mode & 0o077 == 0
    with pytest.raises(FileExistsError):
      proto.make_key(path)
    assert proto.load_key(path) == key


class TestLink:
  def test_failsafe_until_connected(self):
    link = Rc2Link(KEY, rng=fixed_rng)
    assert link.axes(0.0, 0.0) == (FAILSAFE, 0.0, "no_link")

  def test_control_before_handshake_is_ignored(self):
    link = Rc2Link(KEY, rng=fixed_rng)
    app = Session(link)
    app.nonce = NONCE  # guessing the nonce without a challenge still gets nowhere
    app.send(0.0)
    assert link.ctrl is None

  def test_drive_and_steer(self):
    link, app = connected()
    lim = Limits()
    app.send(0.1, accel=1.0, steer=0.0)
    assert link.axes(0.1, 0.0) == pytest.approx((lim.accel_max / JOYSTICKD_ACCEL_GAIN, 0.0, "ok"))
    app.send(0.2, accel=-1.0, steer=1.0)
    gb, steer, status = link.axes(0.2, 0.0)
    assert gb == pytest.approx(-lim.brake_max / JOYSTICKD_ACCEL_GAIN)
    assert steer == pytest.approx(-1.0)  # stick right -> openpilot negative torque (right)
    assert status == "ok"

  def test_deadzone(self):
    link, app = connected()
    app.send(0.1, accel=0.04, steer=-0.04)
    assert link.axes(0.1, 0.0) == (0.0, 0.0, "ok")

  def test_link_timeout_brakes(self):
    link, app = connected()
    app.send(0.1, accel=0.8)
    assert link.axes(0.55, 0.0)[2] == "ok"  # within the 0.5 s link timeout
    assert link.axes(0.65, 0.0) == (FAILSAFE, 0.0, "link_lost")

  def test_short_stop_resumes_without_recentre(self):
    link, app = connected()
    app.send(0.1, accel=0.8)
    link.axes(0.1, 0.0)
    assert link.axes(0.7, 0.0)[2] == "link_lost"
    app.send(1.2, accel=0.8)  # back within a second, stick still held forward
    assert link.axes(1.2, 0.0)[0] > 0

  def test_long_stop_needs_recentre(self):
    link, app = connected()
    app.send(0.1, accel=0.8)
    link.axes(0.1, 0.0)
    assert link.axes(0.7, 0.0)[2] == "link_lost"
    app.send(2.0, accel=0.8)
    assert link.axes(2.0, 0.0) == (FAILSAFE, 0.0, "center_sticks")
    app.send(2.05)
    assert link.axes(2.05, 0.0)[2] == "ok"
    app.send(2.1, accel=0.8)
    assert link.axes(2.1, 0.0)[0] > 0

  def test_held_stick_at_start_waits_for_centre(self):
    link = Rc2Link(KEY, rng=fixed_rng)
    app = Session(link)
    app.hello(0.0)
    app.send(0.0, steer=0.9)
    assert link.axes(0.0, 0.0) == (FAILSAFE, 0.0, "center_sticks")

  @pytest.mark.parametrize("kw", [{"armed": False}, {"fresh": False}, {"age": 600}])
  def test_flag_flicker_keeps_last_command(self, kw):
    link, app = connected()
    app.send(0.1, accel=0.8, steer=0.5)
    good = link.axes(0.1, 0.0)
    app.send(0.2, accel=0.8, steer=0.5, **kw)  # one packet with the dead-man bit missing or a late DJI frame
    assert link.axes(0.2, 0.0) == good
    app.send(0.25, accel=0.8, steer=0.5)
    assert link.axes(0.25, 0.0) == good
    assert link.glitches == 1

  def test_released_dead_man_brakes_after_the_hold(self):
    link, app = connected()
    app.send(0.1, accel=0.8)
    good = link.axes(0.1, 0.0)
    app.send(0.3, accel=0.8, armed=False)
    assert link.axes(0.3, 0.0) == good  # 0.2 s after the last command: still held
    app.send(0.4, accel=0.8, armed=False)
    assert link.axes(0.4, 0.0) == (FAILSAFE, 0.0, "disarmed")

  @pytest.mark.parametrize("kw,status", [
    ({"armed": False}, "disarmed"),
    ({"fresh": False}, "stale_sticks"),
    ({"age": 600}, "stale_sticks"),
  ])
  def test_unsafe_packets_brake(self, kw, status):
    link, app = connected()
    app.send(0.5, accel=0.5, steer=0.5, **kw)
    assert link.axes(0.5, 0.0) == (FAILSAFE, 0.0, status)
    app.send(0.6, accel=0.5, steer=0.5)
    assert link.axes(0.6, 0.0)[2] == "ok"  # a short stop picks up where it left off

  def test_replayed_or_old_packets_are_rejected(self):
    link, app = connected()
    old = app.control(0.1, accel=1.0)
    link.handle(old, 0.1)
    app.send(0.2, accel=0.0)
    link.handle(old, 0.25)  # same seq again
    assert link.ctrl.accel == 0.0
    link.handle(app.control(0.3, accel=1.0, seq=1), 0.3)  # lower seq
    assert link.ctrl.accel == 0.0
    assert link.rejected == 2

  def test_old_session_cannot_be_replayed_after_new_handshake(self):
    nonces = iter([NONCE, bytes(reversed(NONCE))])
    link = Rc2Link(KEY, rng=lambda n: next(nonces))
    app = Session(link, session=1)
    app.hello(0.0)
    recorded = [app.control(0.0), app.control(0.0, accel=1.0)]
    for pkt in recorded:
      link.handle(pkt, 0.0)
    # the app goes away; later a new session gets a different nonce
    app2 = Session(link, session=2)
    assert app2.hello(5.0) is not None
    for pkt in recorded:
      link.handle(pkt, 5.1)
    assert link.ctrl is None

  def test_replayed_hello_does_not_take_over_live_session(self):
    link, app = connected()
    intruder_hello = proto.encode_hello(KEY, 0x9999)
    assert link.handle(intruder_hello, 0.2) is None
    assert link.session == app.session
    app.send(0.25, accel=0.5)
    assert link.axes(0.25, 0.0)[2] == "ok"

  def test_lost_challenge_is_resent(self):
    link = Rc2Link(KEY, rng=fixed_rng)
    first = link.handle(proto.encode_hello(KEY, 7), 0.0)
    again = link.handle(proto.encode_hello(KEY, 7), 0.1)
    assert first == again

  def test_no_speed_cap_by_default(self):
    link, app = connected()
    app.send(0.1, accel=1.0)
    assert link.axes(0.1, 40.0)[0] == pytest.approx(Limits().accel_max / JOYSTICKD_ACCEL_GAIN)

  def test_full_stick_reaches_the_car_limits(self):
    link, app = connected()
    app.send(0.1, accel=1.0, steer=-1.0)
    gb, steer, _ = link.axes(0.1, 0.0)
    assert gb * JOYSTICKD_ACCEL_GAIN == pytest.approx(2.0)
    assert steer == pytest.approx(1.0)  # full left torque
    app.send(0.2, accel=-1.0, steer=0.5)
    gb, steer, _ = link.axes(0.2, 0.0)
    assert gb * JOYSTICKD_ACCEL_GAIN == pytest.approx(-4.0)
    assert steer == pytest.approx(-(0.5 - 0.05) / 0.95)  # linear past the deadzone

  def test_optional_speed_cap(self):
    lim = Limits(speed_cap=15 / 3.6)
    link, app = connected(lim)
    app.send(0.1, accel=1.0)
    assert link.axes(0.1, lim.speed_cap - 0.1)[0] > 0
    assert link.axes(0.1, lim.speed_cap + 0.1)[0] == 0.0
    assert link.axes(0.1, lim.speed_cap + 1.1)[0] == pytest.approx(-0.5 / JOYSTICKD_ACCEL_GAIN)
    app.send(0.15, accel=-1.0)  # braking still works above the cap
    assert link.axes(0.15, lim.speed_cap + 2.0)[0] == pytest.approx(-lim.brake_max / JOYSTICKD_ACCEL_GAIN)

  def test_garbage_is_counted(self):
    link = Rc2Link(KEY, rng=fixed_rng)
    for data in (b"", b"RC2J", b"x" * 40, proto.encode_hello(bytes(32), 1)):
      assert link.handle(data, 0.0) is None
    assert link.rejected == 4


class TestStatus:
  def test_nothing_to_send_before_an_app_says_hello(self):
    link = Rc2Link(KEY, rng=fixed_rng)
    assert link.status_packet(0.0, link="no_link") is None

  def test_sent_to_the_app_and_stops_when_it_goes_quiet(self):
    link = Rc2Link(KEY, rng=fixed_rng)
    addr = ("192.168.43.20", 50000)
    link.handle(proto.encode_hello(KEY, 5), 0.0, addr)
    data, to = link.status_packet(0.1, link="no_link", v_ego=3.0)
    assert to == addr
    st = proto.decode_status(KEY, NONCE, data)
    assert (st.session, st.seq, st.v_ego, st.link) == (5, 1, 3.0, "no_link")
    assert link.status_packet(0.2, link="no_link")[0] != data  # seq moves on
    assert link.status_packet(PEER_IDLE_S + 0.1, link="no_link") is None

  def test_follows_the_app_to_its_latest_address(self):
    link = Rc2Link(KEY, rng=fixed_rng)
    app = Session(link, session=5)
    link.handle(proto.encode_hello(KEY, 5), 0.0, ("10.0.0.2", 1))
    app.nonce = NONCE
    link.handle(app.control(0.1), 0.1, ("10.0.0.3", 2))
    assert link.status_packet(0.1, link="ok")[1] == ("10.0.0.3", 2)


def test_shape():
  assert shape(0.05, 0.05) == 0.0
  assert shape(1.0, 0.05) == pytest.approx(1.0)
  assert shape(-1.0, 0.05, 0.4) == pytest.approx(-1.0)
  assert 0 < shape(0.5, 0.05, 0.4) < shape(0.5, 0.05)  # expo softens the middle
  assert shape(5.0, 0.05) == pytest.approx(1.0)
