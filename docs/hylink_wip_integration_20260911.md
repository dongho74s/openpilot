# Hylink ↔ carrotpilot/wip integration — 2026-09-11

## 결론 / 배포 상태

Hylink Android 앱의 화면을 콤마 UI로 복사한 작업이 아니다.
기존 앱이 사용하는 Wayon Cloud 규격에 맞춰 **차량 쪽 데이터 송신기**를 추가했다.
앱 소스, My Traverse New, Wayon/Sunnypilot, 운영 Cloud 서버, 실제 차량은 변경하지 않았다.

- 기준 브랜치: `leehyuk1108/carrotpilot:wip`
- 작업 전 기준: `ba70f690945fba2f167ce37eccde5bd7144586ab`
- 참고한 Wayon 코드: `7faab119a1c079d74ddf1c59e9def4c9b4b4f29c`
- **실차 접근 불가. 실제 부팅, 주행, 무선망, 카메라/IMU 하드웨어 검증은 수행하지 못했다.**
- 기본값은 전체 연동 OFF. 명시적으로 등록한 차량만 데이터를 전송한다.
- 기본 등록 후에는 읽기 전용 상태·GPS·주행 기록만 사용한다.
- 주차 사진·라이브·충격 감지는 별도 OFF. 실차 검증 전에는 활성화하지 않는 것이 이 배포의 전제다.
- 따라서 이 커밋은 **오프라인 검증을 마친 선택적 연동 코드**이지, 모든 하드웨어 기능의 무오류 인증이나 전체 앱 기능 완성 선언이 아니다.

## 범위

| 기능 | 구현 | 기본 등록 후 |
|---|---|---|
| 현재 차량 위치, 속도, ACC/조향 경고, 기기 온도·전원·연결 | 읽기 전용 cereal 구독 → `/api/telemetry` | 사용 |
| 종료된 최신 주행 경로·정차 품질·활성 시간 | qlog 요약 → `/api/trips` | 조건이 맞는 비주행 때 사용 |
| 홈 지도 차량만 / 전체 지도 내 위치+차량 | 기존 Android 앱 동작 그대로 | 휴대폰 위치 권한은 앱에서 별도 |
| 주차 사진, 광각+실내 라이브, 사진/10·30초 클립 저장 | 기존 앱의 Wayon 미디어 규격 | 별도 OFF, 하드웨어 미검증 |
| 주차 IMU 충격 이벤트 | 센서 관찰 → `/api/impact` | 별도 OFF, 오탐률 미검증 |
| 충격 직후 사진을 해당 이벤트에 연결 | 이번 버전에는 없음 | 충격 이벤트와 정기 사진은 별개 |
| Hylink 원격 SSH·브랜치 설치 | 이번 포트에서 제외 | 앱의 해당 버튼은 사용할 수 없음 |
| 개인 차량 도어 제어·Firebase 조회·앰비언트/CAN 명령 | 제외 | 사용 안 함 |
| BSM/TPMS/연료·충전 데이터 추정 | 하지 않음 | 없는 데이터를 정상 수치로 만들어 보내지 않음 |

**원격 SSH를 제외한 이유:** 기존 구현은 SSH 서비스 자동 시작, 임시 공개키 삽입,
Athena와의 PID 공유를 포함했다. 실차 검증 없는 이번 변경에서는 이 권한 확장을 가져오지 않았다.
기존 comma Athena/SSH 설정은 보존한다. 향후 추가하려면 별도 접근 제어·키 회수 검증이 필요하다.

## 기존 주행 코드 보호

다음 생산 코드는 기준 커밋과 동일하다.

- `opendbc_repo/opendbc/car`: 트레일블레이저 카운터·체크섬·버튼·가감속/조향 수정 포함
- `opendbc_repo/opendbc/safety`, `panda`: Panda 안전 정책 및 펌웨어
- `openpilot/selfdrive`: 제어·모델·UI
- `openpilot/system/camerad`, `loggerd`, `sensord`: 기존 하드웨어 구현
- 부트/AGNOS/모델 설치 스크립트 및 의존성 목록

기존 생산 코드 변경은 `system/manager/process_config.py`의 연동 프로세스 추가와
카메라·센서의 **명시적 비주행 사용 조건 추가**뿐이다.
등록하지 않으면 기존 조건과 동일하다. 주행 때 기존 camerad/sensord 조건도 그대로 참이다.
이것은 하드웨어 경쟁이 전혀 없다는 증명이 아니다. 미검증 미디어/IMU 기능을 OFF로 둔 이유다.

기존 manager 테스트의 `Params.put("정수키", "문자열")` 7곳도 현재 typed Params API에 맞는
`put_int`로 바꿨다. 생산 설정값이나 테스트 기대 결과는 바꾸지 않았다.

## 충돌 방지 구조

`openpilot/system/hylink/` 안에 격리했다.

| 프로세스 | 역할 / 제한 |
|---|---|
| hylink_guard | 네트워크 없이 0.5초마다 비주행 허용 상태 게시 |
| hylink_telemetry | 가벼운 상태 송신. 사진/로그 읽기로 막히지 않음 |
| hylink_worker | 낮은 CPU 우선순위의 비주행 qlog/사진/충격 이벤트 업로드 |
| hylink_live / hylink_relay | 인증된 Cloud ↔ localhost 영상 연결, 비주행만 |
| hylink_encoderd | 기존 `encoderd --stream` 사용. onroad에서 차단 |
| hylink_impact | 비주행 IMU 관찰만, CAN 읽기/송신 없음 |

### 시동을 켜면

1. fresh `pandaStates`에서 ignitionLine/ignitionCan을 확인하거나 manager가 onroad가 되면 차단한다.
2. 비주행 worker/live/relay/impact/stream encoder는 manager에서 SIGKILL로 중지한다.
   막힌 네트워크 호출이나 업로드 스레드가 정상 종료를 지연시키지 않도록 했다.
3. camerad는 Hylink가 직접 시작·종료하지 않는다. 기존 manager가 계속 소유한다.
4. `IsTakingSnapshot`, `IsOnroad`, `AthenadPid`, 운전 설정 Params를 Hylink가 조작하지 않는다.

차단은 소프트웨어 주기와 OS 스케줄링에 따른다. 하드 실시간이나 0ms 전환을 보장하지 않는다.
카메라의 offroad→onroad 연속 사용 및 encoder 자원 회수 타이밍은 실차 미검증 항목이다.

### 상태를 모르면

- deviceState/pandaStates가 없거나 invalid, 2.5초 초과, Panda unknown/heartbeat lost/fault이면 비주행 기능 차단.
- 점화와 Params가 모순되거나 배터리 12V 미만, green 이외 온도 상태, maxTempC 80도 이상이면 차단.
- 카메라 요청 파일은 살아 있는 PID와 2.5초 heartbeat가 모두 필요하다.
- 사진/라이브는 flock으로 하나만 카메라 요청을 소유한다. 프로세스가 죽으면 OS가 lock을 회수한다.
- 운전자 카메라 보기/기존 snapshot 요청, 광각 비활성화, C3X Lite에서는 미디어를 허용하지 않는다.
- 오래된 carState를 현재 속도 0/정상 CAN으로 표시하지 않는다.
- 오래된 GPS는 `fresh:false`, 마지막 위치임을 명시한다. 위도/경도 0 자체는 유효한 값이다.
- wip에 없는 Panda 좌/우 제어 허용 필드를 억지로 읽지 않는다.
- Navdy가 없는 차량을 Navdy 연결 고장으로 판정하지 않는다.

### 부하 / Cloud 예산

- 상태 heartbeat: 주행 30초, 비주행 300초. 주요 상태 변화는 각각 최소 5초/15초 간격.
- 실패 재시도: 채널별 30→60→120→240→최대 300초, jitter. 네트워크 오류가 운전 프로세스로 전파되지 않는다.
- 주행 로그: 최근 24시간의 최신 route만, 종료 후 45초 유예, 최대 4시간/240 segments.
  qlog만 읽으며 압축 파일당 32MiB, 해제 후 64MiB, 처리 45초 제한.
  누락 segment/오류/취소는 완료 업로드로 기록하지 않는다. 점 수 최대 720.
  제한에 걸린 route는 기록에 안 나타날 수 있으며, 모든 과거 주행 backfill은 제공하지 않는다.
- 사진: 별도 활성화 시 성공 간격 1시간. 광각·실내 촬영 및 Cloud 전송을 포함한다.
- 라이브: localhost 8765만 listen, Cloud 인증 relay만 연결. 세션 최대 5분, 앱 heartbeat 12초.
- 클립: 버퍼 카메라별 최대 16MiB/800 frames, 세션을 넘어 업로드 대기 최대 3건,
  압축 아카이브 32MiB 제한.
- 주차 충격: 30초 안정화 뒤 감지, 큐 최대 64건. CAN 기반 도어 잠금/충격 원인 추정 없음.
- 기기 절전·저전압 종료를 해제하거나 강제 깨우지 않는다. 꺼진 기기를 원격으로 켜는 기능은 아니다.

## 등록 / 비활성화

차량 소유자가 위치·차량 데이터 전송에 동의한 경우에만, **차량의 wip 저장소 안에서**
해당 차량 Python 런타임으로 실행한다. 일반 설치 경로는 `/data/openpilot`이다.

```sh
cd /data/openpilot
python3 -m openpilot.system.hylink.setup enable
python3 -m openpilot.system.hylink.setup status
python3 -m openpilot.system.hylink.setup show-key
```

`show-key` 결과는 해당 소유자의 Hylink 앱에만 입력한다. Git/공개 로그/메신저 방에 올리지 않는다.
앱 데이터·로그인을 바꾸거나 서버의 다른 차량 키를 덮어쓰지 않는다.

저장 위치는 `/data/hylink/config.json`, 권한 0600.
자동 등록이나 인증 없는 1108번 LAN 키 표시 페이지는 없다.
등록 중 네트워크가 끊겨도 같은 pending 키로 재시도한다.

### 같은 콤마가 예전에 Wayon에 등록되어 있던 경우

서버가 HTTP 409를 반환하면 새 키로 덮어쓰지 않는다.
로컬 `/data/wayon_cloud/config.json`에 **동일한 DongleId의 기존 키**가 있다면:

```sh
python3 -m openpilot.system.hylink.setup import-legacy
python3 -m openpilot.system.hylink.setup enable
```

기존 파일은 보존하고 ID/키만 복사한다. 개인 차량 명령·설정은 복사하지 않는다.
다른 콤마의 config, 등록 완료된 Hylink 키의 덮어쓰기는 거절한다.
기존 키가 없으면 소유자의 키 복구/재등록 절차가 먼저 필요하다.

### 즉시 되돌리기

```sh
python3 -m openpilot.system.hylink.setup disable
```

다음 manager 점검 때 Hylink 프로세스가 중단된다. 키와 서버 기록은 삭제하지 않는다.
기존 주행 코드/설정 복구나 하드 리셋이 필요한 구조가 아니다.

`media-on/off`, `impact-on/off` 명령도 있지만 **이번 배포에서는 OFF 유지**를 권장한다.
실차 카메라/IMU/전환/주차 전력 검증 없이 이 명령을 실행해서 모든 기능이 검증됐다고 간주하지 않는다.

## 실행한 검증

실행 환경: macOS arm64, 이 wip의 Python 3.12/cereal/Params, 실제 차량 연결 없이 수행.
테스트의 HTTP 요청은 차단했고 서버 검증은 가짜 D1/KV와 실제 Worker 코드를 사용했다.

| 검증 | 결과 |
|---|---|
| Hylink 단위·통합 테스트 | 80개 통과 |
| 기존 manager/camera 조건 테스트 중 비기동 테스트 | 9개 통과 |
| 트레일블레이저 롱컨 회귀 테스트 | 42개 통과 |
| 미등록 상태의 기존 manager와 변경 후 조건 비교 | 1,440건 일치 |
| 현재 cereal → 새 송신기 → 실제 Wayon Worker → 앱 `/api/state` | 오프라인 계약 검증 통과 |
| 새 모듈 import / compileall / Ruff / git diff --check | 통과 |
| 전체 GM car 테스트 | 64 통과, 아래 기존 fingerprint 8 실패 |
| 전체 GM C safety 테스트 | 아래 C/Python 테스트 ABI 불일치로 완료 못 함 |

### 숨기지 않은 기존 테스트 문제

1. 기준 커밋의 원본 values/fingerprints/test 클래스를 별도 Python 프로세스에서 직접 읽어
   실행해도 다음 8 fingerprint 테스트가 동일하게 실패했다:
   VOLT_CC, BOLT_CC, YUKON_CC, CT6_CC, TRAILBLAZER_CC, MALIBU_CC, XT5_CC, TRAX.
   빈 목록 또는 카메라 진단 주소 0x24B가 없는 목록을 기존 테스트가 거절한다.
   이 작업에서 실제 차량 식별 테이블을 추정으로 바꾸지 않았다.
2. 추가 GM C safety 실행은 로컬 시험 중 segfault로 중단됐다.
   현재 `opendbc/safety/safety.h`의 `safety_fwd_hook(CANPacket_t *)`와
   기존 `tests/libsafety/libsafety_py.py`의 `safety_fwd_hook(int, int)` 선언이 다르다.
   이 시험 하네스 문제를 차량 firmware 오류라고 단정하지 않으며, C safety 전체 통과를 주장하지 않는다.
   해당 생산 코드/기존 시험 바이너리는 이 작업에서 수정하지 않았다.
3. 기존 manager 시험 3건은 typed Params에 문자열로 정수를 쓰는 시험 코드 때문에 실패했다.
   `put_int`로 고친 뒤 선택한 9건은 모두 통과했다.
4. manager 전체 prepare/실제 프로세스 전체 기동/실차 부팅은 실행하지 않았다.
   카메라·센서 실출력, 장시간 부하·배터리 소모, 실제 운전 중 경고 없음은 미확인이다.

### 재현 명령

저장소 런타임에서:

```sh
python3 -m pytest -n 0 -q openpilot/system/hylink/tests
python3 -m pytest -n 0 -q openpilot/system/manager/test/test_camera_config.py openpilot/system/manager/test/test_manager.py -k 'not manager_prepare and not set_params_with_default_value and not clean_exit and not startup_time'
python3 -m pytest -o addopts='' -q opendbc_repo/opendbc/car/gm/tests/test_gm.py::TestTrailblazerLongitudinalIntegrity
ruff check openpilot/system/hylink openpilot/system/manager/test/test_manager.py
python3 -m compileall -q openpilot/system/hylink openpilot/system/manager/process_config.py
git diff ba70f690945fba2f167ce37eccde5bd7144586ab -- opendbc_repo panda openpilot/selfdrive openpilot/system/loggerd openpilot/system/camerad openpilot/system/sensord
```

`tests/verify_cloud_contract.mjs`는 stdin으로 시험 telemetry JSON을 받고 첫 인자로
실제 Wayon `worker.js` 경로를 받는다. 외부 fetch는 차단하며 운영 서버나 실제 차량 데이터를 수정하지 않는다.

## 실차 접근이 가능해질 때 남은 확인

1. 기본 OFF에서 기존처럼 부팅·ACC/조향이 동작하는지.
2. 데이터 연동만 켜고 네트워크 단절/복구, 오래된 GPS 표시, 주행 후 trip 업로드.
3. 그 뒤에만 주차 미디어를 별도로 검증: 사진/라이브/클립, 카메라 사용 중 시동 켜기,
   앱 강제 종료, 통신 단절, 광각 OFF/드라이버 뷰 충돌.
4. 마지막으로 주차 IMU/오탐/소모 전력과 다음 주행의 sensord 상태 확인.

실차 확인 불가라는 제약을 코드 검증으로 없앤 것처럼 보고하지 않는다.
