from pytest_mock import MockerFixture

import openpilot.system.ui.lib.wifi_manager as wifi_manager_mod
from openpilot.system.ui.lib.wifi_manager import WifiManager


def _make_wm(mocker: MockerFixture, *, sharing: bool, ipv4_forward: bool):
  wm = WifiManager.__new__(WifiManager)
  wm._exit = True  # no threads running; keeps __del__ -> stop() a no-op
  wm._ipv4_forward = ipv4_forward
  params = mocker.MagicMock()
  params.get_bool.return_value = sharing
  mocker.patch.object(wifi_manager_mod, "Params", return_value=params)
  # _apply_tethering_networking sleeps 5 s after the connection is up.
  mocker.patch.object(wifi_manager_mod.time, "sleep")
  return wm


def _mock_subprocess(mocker: MockerFixture, *, rule_present: bool):
  subprocess = mocker.MagicMock()
  # iptables -C returns 0 when the MASQUERADE rule already exists.
  subprocess.run.return_value = mocker.MagicMock(returncode=0 if rule_present else 1)
  mocker.patch.object(wifi_manager_mod, "subprocess", subprocess)
  return subprocess


def _run_calls(subprocess):
  return [call.args[0] for call in subprocess.run.call_args_list]


def test_sharing_enables_nat_when_rule_absent(mocker: MockerFixture):
  wm = _make_wm(mocker, sharing=True, ipv4_forward=False)
  subprocess = _mock_subprocess(mocker, rule_present=False)

  wm._apply_tethering_networking()

  calls = _run_calls(subprocess)
  assert ["sudo", "sysctl", "net.ipv4.ip_forward=1"] in calls
  assert ["sudo", "iptables-legacy", "-t", "nat", "-C", "POSTROUTING", "-s", "192.168.43.0/24", "-j", "MASQUERADE"] in calls
  assert ["sudo", "iptables-legacy", "-t", "nat", "-A", "POSTROUTING", "-s", "192.168.43.0/24", "-j", "MASQUERADE"] in calls
  assert ["sudo", "sysctl", "net.ipv4.ip_forward=0"] not in calls


def test_sharing_skips_duplicate_rule(mocker: MockerFixture):
  wm = _make_wm(mocker, sharing=True, ipv4_forward=False)
  subprocess = _mock_subprocess(mocker, rule_present=True)

  wm._apply_tethering_networking()

  calls = _run_calls(subprocess)
  assert ["sudo", "sysctl", "net.ipv4.ip_forward=1"] in calls
  assert ["sudo", "iptables-legacy", "-t", "nat", "-A", "POSTROUTING", "-s", "192.168.43.0/24", "-j", "MASQUERADE"] not in calls


def test_no_sharing_keeps_forwarding_off(mocker: MockerFixture):
  wm = _make_wm(mocker, sharing=False, ipv4_forward=False)
  subprocess = _mock_subprocess(mocker, rule_present=False)

  wm._apply_tethering_networking()

  calls = _run_calls(subprocess)
  assert ["sudo", "sysctl", "net.ipv4.ip_forward=0"] in calls
  assert not any("iptables-legacy" in call for call in calls)


def test_no_sharing_with_ui_forward_request_touches_nothing(mocker: MockerFixture):
  wm = _make_wm(mocker, sharing=False, ipv4_forward=True)
  subprocess = _mock_subprocess(mocker, rule_present=False)

  wm._apply_tethering_networking()

  subprocess.run.assert_not_called()


def test_sharing_disabled_when_params_missing(mocker: MockerFixture):
  wm = WifiManager.__new__(WifiManager)
  wm._exit = True
  mocker.patch.object(wifi_manager_mod, "Params", None)

  assert wm._is_internet_sharing_enabled() is False
