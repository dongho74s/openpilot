"""Local-network key page and explicit sharing choices (no extra pairing code)."""
from pathlib import Path

from openpilot.system.hylink import runtime, setup

def pairing_allowed(params):
  return (params.get_bool("IsOffroad") and not params.get_bool("IsOnroad")
          and runtime.fresh_record(runtime.STATE_PATH).get("offroad") is True)


def status(params):
  if not pairing_allowed(params):
    raise ValueError("시동을 끄고 콤마의 전원·온도·차량 연결 상태를 확인해 주세요.")
  config = runtime.read_json(runtime.CONFIG_PATH)
  if config.get("device_id") != runtime.param_text(params, "DongleId") or config.get("registered") is not True:
    return {"ready": False}
  return {"ready": True, "key": config["token"], "enabled": config.get("enabled") is True,
          "deviceId": config["device_id"], "media": config.get("media_enabled") is True,
          "impact": config.get("impact_enabled") is True, "remote": config.get("remote_enabled") is True}


def connect(params, body, enroll=None):
  # Network authorization and CSRF protection belong to connect_server.Handler.
  if not pairing_allowed(params):
    raise ValueError("시동을 끄고 콤마의 전원·온도·차량 연결 상태를 확인해 주세요.")
  if body.get("consent") is not True:
    raise ValueError("위치·차량 상태·주행 기록 전송 동의가 필요해요.")
  if not runtime.CONFIG_PATH.exists() and Path("/data/wayon_cloud/config.json").exists():
    setup.import_legacy(params)
  config = (enroll or setup.enroll)(params, activate=False)
  # Registration can block on network; never activate after an ignition transition.
  if not pairing_allowed(params):
    raise ValueError("차량 상태가 바뀌었어요. 주차 후 다시 연결해 주세요.")
  config.update(enabled=True, media_enabled=body.get("media") is True,
                impact_enabled=body.get("impact") is True,
                remote_enabled=body.get("remote") is True)
  runtime.write_json(runtime.CONFIG_PATH, config)
  return {"ok": True, **status(params)}
