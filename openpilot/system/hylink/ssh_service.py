"""Runs in a transient systemd cgroup, never in the driving manager's cgroup."""
import os
import pwd
import subprocess
import time
from pathlib import Path

from openpilot.common.params import Params
from openpilot.system.hylink import runtime
from openpilot.system.hylink.remote import SSH_REQUEST

KEYS_PATH = runtime.RUNTIME_ROOT / "ssh_authorized_keys"
AGNOS_HOST_KEY = Path("/data/etc/ssh/ssh_host_ed25519_key")
PRIVATE_HOST_KEY = runtime.CONFIG_PATH.with_name("ssh_host_ed25519_key")
AGNOS_RSA_HOST_KEY = Path("/data/etc/ssh/ssh_host_rsa_key")
PRIVATE_RSA_HOST_KEY = runtime.CONFIG_PATH.with_name("ssh_host_rsa_key")


def ssh_command(host_key=AGNOS_HOST_KEY, rsa_host_key=AGNOS_RSA_HOST_KEY):
  return ["/usr/sbin/sshd", "-D", "-e", "-f", "/dev/null", "-p", "12222",
          "-o", "ListenAddress=127.0.0.1", "-o", "AllowUsers=comma",
          "-o", "HostKey=" + str(host_key), "-o", "PasswordAuthentication=no",
          # Android's JSch may lack an Ed25519 provider. Offer RSA with the
          # server's default SHA-2 signatures too; do not enable legacy SHA-1.
          "-o", "HostKey=" + str(rsa_host_key),
          "-o", "UsePAM=no", "-o", "StrictModes=no", "-o", "PidFile=" + str(runtime.RUNTIME_ROOT / "sshd.pid"),
          "-o", "KbdInteractiveAuthentication=no", "-o", "AuthenticationMethods=publickey",
          "-o", "AuthorizedKeysFile=" + str(KEYS_PATH), "-o", "AllowTcpForwarding=no",
          "-o", "AllowAgentForwarding=no", "-o", "X11Forwarding=no", "-o", "PermitTunnel=no",
          "-o", "GatewayPorts=no", "-o", "MaxSessions=1", "-o", "MaxAuthTries=3",
          "-o", "LoginGraceTime=15", "-o", "ClientAliveInterval=10", "-o", "ClientAliveCountMax=2"]


def ensure_host_key(key_type="ed25519"):
  # AGNOS stores its keys under /data/etc/ssh, not /etc/ssh. If system SSH
  # has never run, create a separate Hylink host key without enabling it.
  assert key_type in ("ed25519", "rsa")
  existing = AGNOS_HOST_KEY if key_type == "ed25519" else AGNOS_RSA_HOST_KEY
  private = PRIVATE_HOST_KEY if key_type == "ed25519" else PRIVATE_RSA_HOST_KEY
  if existing.is_file():
    return existing
  if not private.is_file():
    private.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    size = ["-b", "3072"] if key_type == "rsa" else []
    subprocess.run(["/usr/bin/ssh-keygen", "-q", "-t", key_type, *size, "-N", "", "-f", str(private)],
                   check=True, timeout=5, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
  return private


def main():
  params = Params()
  if not runtime.remote_ready(False, params):
    return
  host_key = ensure_host_key()
  rsa_host_key = ensure_host_key("rsa")
  Path("/run/sshd").mkdir(mode=0o755, exist_ok=True)
  process = None
  last_keys = None
  try:
    while runtime.remote_ready(False, params):
      record = runtime.fresh_record(SSH_REQUEST)
      if not record:
        break
      keys = [k["key"] for k in record.get("keys", []) if k.get("until", 0) > time.monotonic()]
      if keys != last_keys:
        # Empty file immediately expires authorization; active session is bounded
        # by the cgroup's five-minute deadline and offroad/relay heartbeats.
        KEYS_PATH.write_text("\n".join("no-port-forwarding,no-agent-forwarding,no-X11-forwarding " + k for k in keys) + "\n")
        owner = pwd.getpwnam("comma")
        os.chown(KEYS_PATH, owner.pw_uid, owner.pw_gid)
        os.chmod(KEYS_PATH, 0o600)
        last_keys = keys
      if process is None:
        if not keys:
          break
        # No PAM login scope: all children stay in the transient unit's cgroup.
        # StrictModes is disabled only for this private, 0600 tmpfs key file.
        process = subprocess.Popen(ssh_command(host_key, rsa_host_key), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
      if process.poll() is not None:
        break
      time.sleep(0.25)
  finally:
    KEYS_PATH.unlink(missing_ok=True)
    if process and process.poll() is None:
      process.terminate()
      try:
        process.wait(timeout=1)
      except subprocess.TimeoutExpired:
        process.kill()
    # Exiting the main unit process makes systemd kill every remaining shell child.


if __name__ == "__main__":
  main()
