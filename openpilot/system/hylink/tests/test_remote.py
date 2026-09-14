import base64
import json
import os
import time

import pytest

from openpilot.system.hylink import remote, ssh_service, runtime


def command(**changes):
  payload = {"publicKey": "ssh-ed25519 " + base64.b64encode(b"test" * 12).decode(),
             "authorizationId": "a" * 32, "ttlSeconds": 90, **changes}
  return remote.PREFIX + base64.urlsafe_b64encode(json.dumps(payload).encode()).decode().rstrip("=")


def test_cloud_authorization_is_short_lived_and_validated():
  key = remote.decode_authorization(command())
  assert time.monotonic() < key["until"] <= time.monotonic() + 90
  for bad in (command(ttlSeconds=999), command(ttlSeconds=True), command(publicKey='ssh-ed25519 abc\ncommand="sh"'),
              command(authorizationId="../attack"), command(publicKey=None), remote.PREFIX + "!", "ambient-command"):
    assert remote.decode_authorization(bad) is None


def test_remote_offroad_gate_runs_before_service_start(monkeypatch):
  monkeypatch.setattr(remote.subprocess, "run", lambda *a, **k: pytest.fail("No SSH process may start"))
  relay = remote.SshRelay(lambda: False)
  relay.open_local(None)
  with pytest.raises(ValueError):
    relay.command(command())


def test_ssh_does_not_change_persistent_keys_or_expose_network_port(tmp_path, monkeypatch):
  path = tmp_path / "ssh_request.json"
  monkeypatch.setattr(remote, "SSH_REQUEST", path)
  relay = remote.SshRelay(lambda: True)
  relay.command(command())
  assert len(runtime.read_json(path)["keys"]) == 1
  relay.keys[0]["until"] = time.monotonic() - 1
  relay.tick()
  assert runtime.read_json(path)["keys"] == []
  args = ssh_service.ssh_command()
  assert "ListenAddress=127.0.0.1" in args
  assert "PasswordAuthentication=no" in args and "AllowTcpForwarding=no" in args
  assert "HostKey=/data/etc/ssh/ssh_host_ed25519_key" in args
  assert "HostKey=/data/etc/ssh/ssh_host_rsa_key" in args
  assert not any("+ssh-rsa" in a for a in args)  # No SHA-1 algorithm downgrade.
  assert "UsePAM=no" in args  # Keep sessions in the cgroup that is stopped offroad->onroad.
  assert not any("GithubSshKeys" in a for a in args)
  args = remote.service_command()
  assert "--property=KillMode=control-group" in args and "--property=RuntimeMaxSec=300" in args


def test_ssh_unit_uses_checkout_dependencies_without_inheriting_arbitrary_pythonpath(monkeypatch):
  monkeypatch.setattr(remote, "BASEDIR", "/data/openpilot")
  monkeypatch.setenv("PYTHONPATH", "/untrusted/path")
  args = remote.service_command()
  paths = [a for a in args if a.startswith("--setenv=PYTHONPATH=")]
  assert paths == ["--setenv=PYTHONPATH=" + os.pathsep.join(("/data/openpilot/pydeps", "/data/openpilot"))]
  assert not any("/untrusted/path" in a for a in args)


def test_host_key_uses_agnos_key_without_starting_system_ssh(tmp_path, monkeypatch):
  existing = tmp_path / "host-key"
  existing.touch()
  monkeypatch.setattr(ssh_service, "AGNOS_HOST_KEY", existing)
  monkeypatch.setattr(ssh_service.subprocess, "run", lambda *a, **k: pytest.fail("Do not regenerate an existing key"))
  assert ssh_service.ensure_host_key() == existing


def test_android_rsa_host_key_uses_existing_key_or_private_3072_bit_fallback(tmp_path, monkeypatch):
  existing, private = tmp_path / "agnos-rsa", tmp_path / "hylink" / "rsa"
  monkeypatch.setattr(ssh_service, "AGNOS_RSA_HOST_KEY", existing)
  monkeypatch.setattr(ssh_service, "PRIVATE_RSA_HOST_KEY", private)
  calls = []
  monkeypatch.setattr(ssh_service.subprocess, "run", lambda args, **kwargs: calls.append(args))
  existing.touch()
  assert ssh_service.ensure_host_key("rsa") == existing
  assert calls == []
  existing.unlink()
  assert ssh_service.ensure_host_key("rsa") == private
  assert calls == [["/usr/bin/ssh-keygen", "-q", "-t", "rsa", "-b", "3072", "-N", "", "-f", str(private)]]
