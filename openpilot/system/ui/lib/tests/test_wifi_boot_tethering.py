from pytest_mock import MockerFixture

import openpilot.system.ui.lib.wifi_manager as wifi_manager_mod
from openpilot.system.ui.lib.wifi_manager import ConnectStatus, WifiManager, WifiState


def _make_wm(mocker: MockerFixture, *, tethering_active: bool):
  wm = WifiManager.__new__(WifiManager)
  wm._exit = True  # no threads running; keeps __del__ -> stop() a no-op
  wm._tethering_ssid = "weedle-49da"
  wm._wifi_state = WifiState(
    ssid="weedle-49da" if tethering_active else "U+Net1F78",
    status=ConnectStatus.CONNECTED,
  )
  wm.set_tethering_active = mocker.MagicMock()
  return wm


def _set_param(mocker: MockerFixture, hotspot_on_boot: bool):
  params = mocker.MagicMock()
  params.get_bool.return_value = hotspot_on_boot
  mocker.patch.object(wifi_manager_mod, "Params", return_value=params)
  mocker.patch.object(wifi_manager_mod, "_boot_tethering_done", False)


def test_enables_tethering_when_param_set(mocker: MockerFixture):
  wm = _make_wm(mocker, tethering_active=False)
  _set_param(mocker, True)

  wm._maybe_enable_tethering_on_boot()

  wm.set_tethering_active.assert_called_once_with(True)


def test_does_nothing_when_param_unset(mocker: MockerFixture):
  wm = _make_wm(mocker, tethering_active=False)
  _set_param(mocker, False)

  wm._maybe_enable_tethering_on_boot()

  wm.set_tethering_active.assert_not_called()


def test_does_nothing_when_already_active(mocker: MockerFixture):
  wm = _make_wm(mocker, tethering_active=True)
  _set_param(mocker, True)

  wm._maybe_enable_tethering_on_boot()

  wm.set_tethering_active.assert_not_called()


def test_enables_only_once_per_process(mocker: MockerFixture):
  wm = _make_wm(mocker, tethering_active=False)
  _set_param(mocker, True)

  # First WifiManager to init enables tethering; a later one (e.g. opening
  # settings after a manual tether-off) must not re-enable it.
  wm._maybe_enable_tethering_on_boot()
  wm._maybe_enable_tethering_on_boot()

  wm.set_tethering_active.assert_called_once_with(True)
