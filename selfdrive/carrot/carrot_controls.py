from openpilot.common.realtime import DT_CTRL
from openpilot.common.params import Params

class CarrotControls:
  def __init__(self, CP):
    self.CP = CP
    self.params = Params()
    self.lat_suspend_active = False
    self.lat_suspend_enter_t = 0.0
    self.lat_suspend_hold_t = 0.0
    self.lat_suspend_release_t = 0.0

  def lat_suspend_control(self, CS, latActive):
    # 수동조향 '세기'(운전자 조향토크)로 자동조향 일시해제/복귀를 판정한다.
    #  종전엔 조향각(steeringAngleDeg) 기준이었는데, 커브에서는 도로를 따라가느라
    #  휠각 자체가 커져 (1) 약한 개입에도 해제되고 (2) 커브가 끝나 각이 15° 이내로
    #  돌아올 때까지 복귀를 못 했다("코너 복귀 늦음"). 게다가 이 차량의 커브 휠각은
    #  ~15°에 불과해(주행로그 00000147--20), 각도 임계(최소 45°)로는 어떤 설정에서도
    #  발동 자체가 불가능했다. 운전자 토크는 커브 곡률과 무관하므로 두 문제를 함께 해소.
    #  - 해제: '강한' 수동조향(토크 임계 초과)이 잠깐 지속될 때만.
    #  - 복귀: 손을 떼거나 약하게 잡으면(토크 임계 미만) 커브 도중이라도 곧 복귀.
    enter_torque = float(self.params.get_int("LatSuspendAngleDeg"))  # 재해석: 해제 운전자토크 임계
    exit_torque  = enter_torque * 0.4   # 이력(hysteresis): 이 미만이면 '약한 조향/손 뗌'으로 간주
    enter_sec    = 0.3   # 강한 토크가 이만큼 지속돼야 해제(순간 노이즈·움찔 무시)
    hold_min_sec = 0.3   # 해제 후 최소 유지시간(즉시 복귀 채터링 방지)
    release_sec  = 0.4   # 약한 토크가 이만큼 지속되면 복귀(휠각 무관)

    drv_torque = abs(CS.steeringTorque)

    # 1) 해제(suspend) 진입: 강한 수동조향이 enter_sec 지속
    if not self.lat_suspend_active:
      if CS.steeringPressed and drv_torque >= enter_torque:
        self.lat_suspend_enter_t += DT_CTRL
        if self.lat_suspend_enter_t >= enter_sec:
          self.lat_suspend_active = True
          self.lat_suspend_hold_t = 0.0
          self.lat_suspend_release_t = 0.0
      else:
        self.lat_suspend_enter_t = 0.0

    # 2) 해제 중: 최소 유지시간 + 약한 토크(손 뗌) 지속 시 복귀(각도 조건 없음)
    else:
      self.lat_suspend_hold_t += DT_CTRL
      if (not CS.steeringPressed) or drv_torque < exit_torque:
        self.lat_suspend_release_t += DT_CTRL
      else:
        self.lat_suspend_release_t = 0.0
      if self.lat_suspend_hold_t >= hold_min_sec and self.lat_suspend_release_t >= release_sec:
        self.lat_suspend_active = False
        self.lat_suspend_enter_t = 0.0

    if self.lat_suspend_active:
      latActive = False
    return latActive
