"""Opt-in SSH relay. Ephemeral keys and a separate loopback-only supervised sshd.

Never changes GithubSshKeys, SshEnabled or the system SSH service. The watchdog
unit owns all shell children and expires when this process/guard stops renewing.
"""
import base64
import binascii
import json
import os
import re
import subprocess
import sys
import time

from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params
from openpilot.system.hylink import runtime
from openpilot.system.hylink.relay import Relay, run_relay

SSH_REQUEST = runtime.RUNTIME_ROOT / "ssh_request.json"
PREFIX = "wayon-ssh-authorize-v1."
UNIT = "hylink-parking-ssh"


def decode_authorization(command):
  if not command.startswith(PREFIX) or len(command) > 6500:
    return None
  try:
    encoded = command[len(PREFIX):]
    payload = json.loads(base64.b64decode(encoded + "=" * (-len(encoded) % 4), altchars=b"-_", validate=True))
    if not isinstance(payload, dict):
      return None
    key = payload.get("publicKey", "")
    match = re.fullmatch(r"(ssh-rsa|ssh-ed25519) ([A-Za-z0-9+/]{40,4096}={0,2})(?: [^\r\n]{1,128})?", key)
    if not match or not re.fullmatch(r"[0-9a-f]{32}", payload.get("authorizationId", "")):
      return None
    ttl = payload.get("ttlSeconds")
    if type(ttl) is not int or not 60 <= ttl <= 120:
      return None
    base64.b64decode(match[2], validate=True)
    return {"key": f"{match[1]} {match[2]}", "until": time.monotonic() + ttl}
  except (ValueError, TypeError, binascii.Error):
    return None


def service_command():
  return ["sudo", "-n", "systemd-run", "--quiet", "--collect", "--unit=" + UNIT,
          "--property=RuntimeMaxSec=300", "--property=KillMode=control-group",
          "--property=TimeoutStopSec=1", "--working-directory=" + BASEDIR,
          "--setenv=PYTHONPATH=" + BASEDIR, sys.executable, "-m", "openpilot.system.hylink.ssh_service"]


class SshRelay(Relay):
  def __init__(self, allowed):
    super().__init__(allowed, target=("127.0.0.1", 12222))
    self.keys = []

  def tick(self):
    self.keys = [k for k in self.keys if k["until"] > time.monotonic()]
    runtime.write_json(SSH_REQUEST, {"pid": os.getpid(), "at": time.monotonic(), "keys": self.keys})

  def command(self, command):
    if command.startswith(PREFIX):
      key = decode_authorization(command)
      if key is None or not self.allowed():
        raise ValueError("Invalid SSH authorization")
      self.keys = [key]  # One authenticated phone/session at a time.
      self.tick()

  def open_local(self, ws):
    if not self.allowed() or not any(k["until"] > time.monotonic() for k in self.keys):
      return
    self.tick()
    # A running unit returns nonzero; do not restart or drop its current session.
    subprocess.run(service_command(), timeout=3, check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    deadline = time.monotonic() + 5
    while self.allowed() and time.monotonic() < deadline:
      self.tick()
      try:
        return super().open_local(ws)
      except OSError:
        time.sleep(0.1)
    raise OSError("Parking SSH unavailable")


def main():
  params = Params()
  relay = SshRelay(lambda: runtime.remote_ready(False, params))
  try:
    run_relay(relay, params, "ssh")
  finally:
    SSH_REQUEST.unlink(missing_ok=True)


if __name__ == "__main__":
  main()
