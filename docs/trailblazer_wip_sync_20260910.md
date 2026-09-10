# Trailblazer carrot-wip 동기화 — 2026-09-10

## 최종 사용 브랜치: `wip`

사용자 지정에 따라 최종 사용 대상은 `leehyuk1108/carrotpilot:wip`로 정했다.
이미 있던 `wip` (`bdcbff6e525dc203f0744032ec9829f6f7d2ccc2`)의 이력을 보존하고,
아래에서 검증한 최신 WIP + Trailblazer 패치 (`e8e3c245`)를 병합했다.
`trailblazer_long_fix_carrot_wip_pr` 등 다른 원격 브랜치는 이 단계에서 변경하지 않았다.

기존 `wip`에만 있던 `bdcbff6e`의 테스트 보강 11줄도 유지했다.
이는 `TRAILBLAZER_CC`와 gateway 구성에 전용 수정이 잘못 적용되지 않는지 검사한다.
차량 제어 코드와 DBC는 아래의 검증 버전과 동일하며, 새 튜닝을 추가하지 않았다.
추가 테스트를 포함해 같은 회귀 묶음을 다시 실행한 결과는
**808 passed, 3 skipped, 3 subtests passed**다.
아래의 기존 전체 검사 제한과 실기기/실차 미검증 범위는 그대로 적용된다.

## 반영 기준

- 배포 대상: `leehyuk1108/carrotpilot:trailblazer_long_fix`
- 이전 배포: `11871064ddf97367f24693fafcb3753b9a8d78fd`
- 병합한 upstream: `ajouatom/openpilot:carrot-wip`, `fdfa5680a569eade8f36602810c4d5125a3a8d2b`
- 병합 커밋: `6701961e5e261091c6a6a8d6a3a7158ed2039da7`
- 이번 작업은 기존 패치를 최신 WIP에 유지하는 업데이트다. 260904 조향 로그에 대한 추가 튜닝은 포함하지 않았다.

## 유지한 차량별 수정

이전 배포와 비교해 GM 차량 코드 전체, GM DBC 수정 두 파일, GM safety 코드에는 차이가 없다.
upstream과의 차이도 기존 Trailblazer 패치 범위를 유지한다.

- 순정 `0x2CB` 카운터에 맞춘 종방향 명령 그룹 송신
- Trailblazer 전용 24비트 가스/리젠 체크섬과 다른 GM의 기존 계산 보존
- 순정 종방향 활성 상태 및 `0x370` 상태에 따른 명령 제한
- 초기 순정 기준 메시지 수신 대기 및 동기화 참고 메시지의 alive 검사 처리
- 가속페달 개입 시 가스/브레이크 명령 중립화
- 합성 cancel의 press/release 완결 및 이후 +/- 버튼 정상 처리
- 기존 저속 인게이지 제한

기존 상세 원인 분석은 [이전 종합 보고서](trailblazer_longitudinal_fault_analysis.md)를 참고한다.
이번 병합 자체가 그 보고서의 실차 검증 범위를 넓히거나, 미확정 조향 원인을 확정하는 것은 아니다.

## 새로 가져온 공통 변경

레이더/선행차 판단, 가감속 계획, 런타임 의존성 설치와 부팅 처리, 업데이트 처리,
eGPU 및 비전 기능 등 upstream 변경을 함께 가져왔다.
`launch_env.sh`의 AGNOS 기준은 `19.6.3-carrot`이다.

충돌은 한국어/영어 `cruise-gap.md` 두 파일에서만 발생했다.
기존 GM 설명과 upstream의 선행차 이탈 예측 설명을 모두 보존했다.

추가 수정은 부팅 회귀 테스트 한 줄이다.
`test_offline_python_bootstrap.py`가 이미 `start_manager` 함수 호출로 바뀐 실행부에서
예전 문자열 `./manager.py`를 찾던 것을 실제 호출 이름으로 맞췄다.
실행 코드나 안전 제한을 테스트 통과 목적으로 변경하지 않았다.
`test_manager_affinity.py`에서 해당 함수의 실행 및 종료 코드 전달도 별도로 검사한다.

## 검증 결과

검증 환경은 macOS arm64, Python 3.12.13이다. 기기에 설치하거나 CAN을 송신하지 않았다.

| 검사 | 결과 |
|---|---|
| 최종 선택 회귀 테스트 묶음 | **806 passed, 3 skipped, 3 subtests passed** |
| Trailblazer 집중 테스트 | 위 묶음에 포함, 40개 통과 |
| 크루즈 버튼 상태 머신 | 위 묶음에 포함, 11개 통과 |
| GM 가속페달 개입 시 중립 명령 허용 | 위 묶음에 포함, 1개 통과 |
| 부팅·AGNOS·eGPU·업데이트 검사 묶음 | 위 묶음에 포함, 104개 통과 / 3개 건너뜀 |
| 레이더·비전·추종거리·정차·가감속 계획 | 위 묶음에 포함 |
| 핵심 native 모듈 및 주행/운전자 감시 모델의 Mac 빌드 | 통과 |
| 부팅 shell 구문 검사 | 통과 |
| 실기기 AGNOS 부팅, eGPU 연결, 실제 주행 | **이번 작업에서 미검증** |

Safety 검사는 Mac 호스트용으로 Clang으로 빌드한 libsafety를 사용했다.
Panda 펌웨어가 제공하는 Hyundai forwarding 전역 변수의 호스트 링크 정의만 추가했으며,
GM safety 로직은 수정하지 않았다. 이 호스트 보조 파일과 빌드 산출물은 배포에 포함하지 않는다.

핵심 빌드 명령은 다음과 같다. 전체 기기 빌드나 실차 검증과 혼동하지 않는다.

```sh
python3 -m SCons --minimal -j8 \
  openpilot/common openpilot/cereal msgq_repo \
  openpilot/selfdrive/controls/lib/lateral_mpc_lib \
  openpilot/selfdrive/controls/lib/longitudinal_mpc_lib \
  openpilot/selfdrive/modeld openpilot/selfdrive/locationd \
  openpilot/selfdrive/pandad openpilot/selfdrive/ui \
  openpilot/selfdrive/carrot/realtime openpilot/system/loggerd
bash -n launch_chffrplus.sh launch_env.sh restart.sh
```

### 전체 검사에서 남아 있는 사항

1. 전체 GM 테스트의 fingerprint 실패 8개: 빈 fingerprint 또는 카메라 진단 주소 누락.
   이전 보고서에도 같은 8개가 기록되어 있고, 이번 병합에서 GM fingerprint/values/tests는 변경되지 않았다.
2. 기존 `test_longcontrol.py` 실패 3개: 테스트 호출에 `a_ego`, `stopping_accel`, `radarState` 인수가 없다.
   해당 함수는 이전 배포 `11871064`에서도 이미 그 인수를 요구했고 테스트도 이번 병합 전후 동일하다.
   이 실패를 숨기려고 런타임 함수나 테스트를 변경하지 않았다.
3. Mac 전체 `SCons --minimal` 빌드: PC용 `jotpluggler`의 스키마 생성에서
   `Import failed: /include/c++.capnp` 오류가 발생했다.
   해당 생성 스크립트는 이전 배포와 동일하다. `SConstruct`에서 이 도구는
   `arch != "larch64"`일 때만 빌드되므로 AGNOS 차량 기기 빌드 대상은 아니다.
   전체 Mac 빌드 성공으로 보고하지 않으며, 위의 핵심 타깃 빌드 성공과 구분한다.

초기 검증 중 로컬 Python 실행 경로와 의존성, 미다운로드 LFS 모델 때문에 발생한 준비 오류는
테스트 환경을 구성하고 원본 LFS 파일을 받은 뒤 해소했다.
최종 테스트 수치는 그 준비가 끝난 뒤 한 번에 실행한 결과다.

## 적용 후 확인할 사항

안전하게 정차한 상태에서 업데이트/재부팅을 마치고 설치 커밋 및 차량 인식을 확인한다.
부팅 후 CAN 상태, 인게이지와 해제, +/- 버튼, 가속페달 개입,
정차/재출발, 후진 후 D 복귀 상태는 실차에서 별도로 확인해야 한다.
경고가 재발하면 운전자가 즉시 제어를 맡고 해당 시점 rlog와 실제 설치 커밋을 함께 확보한다.

되돌릴 기준 커밋은 위의 이전 배포 `11871064`다. 자동 롤백이나 기기 원격 설치는 수행하지 않았다.
