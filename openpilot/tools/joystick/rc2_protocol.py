"""Wire format between the DJI RC 2 app (rc2-openpilot-joystick) and rc2_receiver.

UDP, little-endian. Every packet starts with an 8-byte header (b"RC2J", version, type, flags, 0)
and ends with the first 16 bytes of HMAC-SHA256 under a shared 32-byte key, so nothing on the
network can steer the car without the key.

  HELLO      app -> comma   header | session u32                                  | mac(key, packet)
  CHALLENGE  comma -> app   header | session u32 | nonce[16]                      | mac(key, packet)
  CONTROL    app -> comma   header | session u32 | seq u32 | accel i16 | steer i16
                                   | stick_age_ms u16 | buttons u16               | mac(key, nonce + packet)
  STATUS     comma -> app   header | session u32 | seq u32 | car and comma state | text[96]
                                                                                  | mac(key, nonce + packet)

The app finds the comma by broadcasting HELLO on its network; only a comma holding the same key
answers. CONTROL is bound to a nonce the receiver picks for each session and its seq must strictly
increase, so a recording of an earlier drive can't be replayed. accel and steer are the raw stick
positions in thousandths (-1000..1000): accel > 0 is stick forward, steer > 0 is stick right.
STATUS carries what the RC 2 screen shows about the car and the comma.
"""
import hashlib
import hmac
import os
import struct
from dataclasses import dataclass

MAGIC = b"RC2J"
VERSION = 1
HELLO, CHALLENGE, CONTROL, STATUS = 1, 2, 3, 4
FLAG_ARMED = 0x01  # arm toggle on and dead-man button held on the RC 2
FLAG_FRESH = 0x02  # the stick sample came from the DJI stream within the app's freshness window

# STATUS.state bits
ST_ONROAD = 1 << 0
ST_REMOTE_DRIVE = 1 << 1    # joystickd runs now, driven by the RC 2
ST_ENABLED = 1 << 2
ST_ACTIVE = 1 << 3
ST_BRAKE = 1 << 4
ST_GAS = 1 << 5
ST_DOOR_OPEN = 1 << 6
ST_SEATBELT_OFF = 1 << 7
ST_STANDSTILL = 1 << 8
ST_CRUISE_AVAILABLE = 1 << 9
ST_TOGGLE_ON = 1 << 10      # the Carrot web switch is on (onroad it applies once openpilot is disengaged)
ST_STEER_FAULT = 1 << 11

# STATUS.link codes, the reason rc2_receiver passes the sticks through or brakes
LINK_CODES = ("ok", "no_link", "link_lost", "disarmed", "stale_sticks", "center_sticks")
STATUS_TEXT_LEN = 96

MAC_LEN = 16
KEY_LEN = 32
NONCE_LEN = 16
AXIS_SCALE = 1000
DEFAULT_PORT = 7720
DEFAULT_KEY_PATH = "/data/rc2_joystick/key"

_HEADER = struct.Struct("<4sBBBB")
_HELLO = struct.Struct("<I")
_CHALLENGE = struct.Struct("<I16s")
_CONTROL = struct.Struct("<IIhhHH")
# session, seq, v_ego cm/s, steering angle 0.1 deg, gear, link code, state bits,
# commanded accel cm/s^2, commanded steer thousandths, speed cap km/h, max temp C, text
_STATUS = struct.Struct(f"<IIHhBBHhhBB{STATUS_TEXT_LEN}s")

HELLO_LEN = _HEADER.size + _HELLO.size + MAC_LEN
CHALLENGE_LEN = _HEADER.size + _CHALLENGE.size + MAC_LEN
CONTROL_LEN = _HEADER.size + _CONTROL.size + MAC_LEN
STATUS_LEN = _HEADER.size + _STATUS.size + MAC_LEN


@dataclass(frozen=True)
class Control:
  session: int
  seq: int
  accel: float  # -1..1, stick forward is positive
  steer: float  # -1..1, stick right is positive
  stick_age_ms: int
  buttons: int
  flags: int

  @property
  def armed(self) -> bool:
    return bool(self.flags & FLAG_ARMED)

  @property
  def fresh(self) -> bool:
    return bool(self.flags & FLAG_FRESH)


@dataclass(frozen=True)
class Status:
  session: int
  seq: int
  v_ego: float = 0.0          # m/s
  steering_angle: float = 0.0  # deg, left positive
  gear: int = 0               # car.CarState.GearShifter raw value
  link: str = "no_link"
  state: int = 0
  accel_cmd: float = 0.0      # m/s^2 sent to joystickd
  steer_cmd: float = 0.0      # -1..1 sent to joystickd, left positive
  speed_cap_kph: int = 0
  max_temp_c: int = 0
  text: str = ""              # current alert, already joined into one line


def _utf8_prefix(text: str, limit: int) -> bytes:
  data = text.encode("utf-8")[:limit]
  return data.decode("utf-8", "ignore").encode("utf-8")


def _mac(key: bytes, data: bytes) -> bytes:
  return hmac.new(key, data, hashlib.sha256).digest()[:MAC_LEN]


def _header(kind: int, flags: int = 0) -> bytes:
  return _HEADER.pack(MAGIC, VERSION, kind, flags & 0xFF, 0)


def _axis(value: float) -> int:
  return max(-AXIS_SCALE, min(AXIS_SCALE, round(value * AXIS_SCALE)))


def packet_type(data: bytes) -> int | None:
  if len(data) < _HEADER.size + MAC_LEN:
    return None
  magic, version, kind, _, _ = _HEADER.unpack_from(data)
  if magic != MAGIC or version != VERSION:
    return None
  return kind


def encode_hello(key: bytes, session: int) -> bytes:
  body = _header(HELLO) + _HELLO.pack(session)
  return body + _mac(key, body)


def encode_challenge(key: bytes, session: int, nonce: bytes) -> bytes:
  body = _header(CHALLENGE) + _CHALLENGE.pack(session, nonce)
  return body + _mac(key, body)


def encode_control(key: bytes, nonce: bytes, ctrl: Control) -> bytes:
  body = _header(CONTROL, ctrl.flags) + _CONTROL.pack(ctrl.session, ctrl.seq, _axis(ctrl.accel), _axis(ctrl.steer),
                                                      max(0, min(0xFFFF, ctrl.stick_age_ms)), ctrl.buttons & 0xFFFF)
  return body + _mac(key, nonce + body)


def _clamp(value: float, lo: int, hi: int) -> int:
  return max(lo, min(hi, round(value)))


def encode_status(key: bytes, nonce: bytes, st: Status) -> bytes:
  link = LINK_CODES.index(st.link) if st.link in LINK_CODES else 0xFF
  body = _header(STATUS) + _STATUS.pack(
    st.session, st.seq, _clamp(st.v_ego * 100, 0, 0xFFFF), _clamp(st.steering_angle * 10, -32768, 32767),
    st.gear & 0xFF, link, st.state & 0xFFFF, _clamp(st.accel_cmd * 100, -32768, 32767),
    _clamp(st.steer_cmd * AXIS_SCALE, -AXIS_SCALE, AXIS_SCALE), _clamp(st.speed_cap_kph, 0, 255),
    _clamp(st.max_temp_c, 0, 255), _utf8_prefix(st.text, STATUS_TEXT_LEN))
  return body + _mac(key, nonce + body)


def decode_status(key: bytes, nonce: bytes, data: bytes) -> Status | None:
  body = _verified_body(data, STATUS, STATUS_LEN, key, nonce)
  if body is None:
    return None
  session, seq, v, angle, gear, link, state, accel, steer, cap, temp, text = _STATUS.unpack_from(body, _HEADER.size)
  return Status(session, seq, v / 100, angle / 10, gear, LINK_CODES[link] if link < len(LINK_CODES) else "unknown", state,
                accel / 100, steer / AXIS_SCALE, cap, temp, text.rstrip(b"\0").decode("utf-8", "ignore"))


def _verified_body(data: bytes, kind: int, length: int, key: bytes, prefix: bytes = b"") -> bytes | None:
  if len(data) != length or packet_type(data) != kind:
    return None
  body, tag = data[:-MAC_LEN], data[-MAC_LEN:]
  if not hmac.compare_digest(tag, _mac(key, prefix + body)):
    return None
  return body


def decode_hello(key: bytes, data: bytes) -> int | None:
  body = _verified_body(data, HELLO, HELLO_LEN, key)
  return None if body is None else _HELLO.unpack_from(body, _HEADER.size)[0]


def decode_challenge(key: bytes, data: bytes) -> tuple[int, bytes] | None:
  body = _verified_body(data, CHALLENGE, CHALLENGE_LEN, key)
  return None if body is None else _CHALLENGE.unpack_from(body, _HEADER.size)


def decode_control(key: bytes, nonce: bytes, data: bytes) -> Control | None:
  body = _verified_body(data, CONTROL, CONTROL_LEN, key, nonce)
  if body is None:
    return None
  flags = body[6]
  session, seq, accel, steer, age, buttons = _CONTROL.unpack_from(body, _HEADER.size)
  return Control(session, seq, accel / AXIS_SCALE, steer / AXIS_SCALE, age, buttons, flags)


def load_key(path: str) -> bytes:
  with open(path) as f:
    key = bytes.fromhex(f.read().strip())
  if len(key) != KEY_LEN:
    raise ValueError(f"{path} must hold {KEY_LEN} bytes as hex")
  return key


def make_key(path: str) -> None:
  """Write a new random key readable only by its owner. Never overwrites an existing key."""
  os.makedirs(os.path.dirname(path) or ".", mode=0o700, exist_ok=True)
  fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
  with os.fdopen(fd, "w") as f:
    f.write(os.urandom(KEY_LEN).hex() + "\n")
