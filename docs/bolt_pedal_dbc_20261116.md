# 볼트 EUV + 콤마 페달 CAN 오류 (2026-11-16)

## 증상

볼트 EUV에 콤마 페달을 단 사용자가 WIP-FR(`8edded84`)에서 "CAN 오류: 연결을 확인하세요"를 받았다.
받은 rlog(`00000003--88b21ebc6e--2`, comma 3X, 3분)를 보면:

- `carState`·`carParams`가 하나도 없다.
- `card`가 차량 인식("fingerprinted") 뒤 약 0.45초 만에 `card is dead with 1`로 죽고 다시 뜬다. 3분 동안 38번 반복된다.
- 판다는 끝까지 `elm327` 안전 모드다. GM 모드로 못 넘어가 하네스 릴레이가 안 열려서 버스 0과 2가 똑같이 보인다. 결과일 뿐 배선 문제는 아니다. 버스 오류(bus-off)도 없다.
- 페달의 `GAS_SENSOR` STATE는 5(FAULT_TIMEOUT)다. 아무도 페달 명령을 안 보냈기 때문이다.

## 원인

1. CAN에 페달 메시지(0x201)가 보이면 GM `interface.py`가 `enableGasInterceptorDEPRECATED`를 켠다.
2. 그러면 `carstate.py`가 `pt_cp.vl["GAS_SENSOR"]`를 읽는다.
3. GM 공용 DBC `gm_global_a_powertrain_volt.dbc`에는 페달 메시지 정의가 없었다. `CM_ "IMPORT _community.dbc"` 한 줄만 있었는데, IMPORT는 `dbc/generator/` 아래 파일을 생성할 때만 처리된다. `dbc/` 바로 아래 파일에서는 그냥 주석이다.
4. CAN 파서 `_add_message`의 `assert msg is not None`에서 card가 죽는다.

당근 원본 carrot-wip 최신(`90430cd1`)도 구조가 같다. 원본부터 있던 결함이고, 페달을 단 GM 차 전부가 해당된다. 페달이 없는 차(트래버스 등)는 이 메시지를 읽지 않아서 영향이 없다.

## 수정

- `gm_global_a_powertrain_volt.dbc`에서 동작하지 않던 IMPORT 줄을 빼고, `generator/gm/_community.dbc`의 `GAS_COMMAND`(0x200)·`GAS_SENSOR`(0x201) 정의와 STATE 값 표를 그대로 넣었다. `BU_`에는 `INTERCEPTOR`를 추가했다.
- `opendbc/car/gm/tests/test_pedal_interceptor.py`를 추가했다.
  - 모든 GM 차종 DBC로 페달 명령을 만들고, 그 로그의 실제 GAS_SENSOR 프레임을 해석한다.
  - BOLT_CC·BOLT_EUV에 페달 지문을 주고 CarState를 갱신해도 죽지 않는지, gas 값이 맞는지 본다.
- 조향·제동 로직, 판다 안전 코드, Params는 바꾸지 않았다. 판다 `safety_gm.h`는 원래 페달(0x200 송신, 0x201 수신)을 지원한다.

## 검증

- 맥(가짜 Params·msgq 심): 새 테스트 33개가 통과한다. 수정 전 DBC로 돌리면 33개 모두 실패한다.
- GM 테스트 전체: 수정 전 930개 통과·8개 실패, 수정 후 963개 통과·8개 실패. 실패 8개는 수정 전후 목록이 같다(`test_can_fingerprints`의 카메라 진단 주소 문제로, 이번 수정과 무관).
- 로그 재생: 수정본 코드에 그 rlog의 설정값과 CAN 5997프레임을 넣었다. BOLT_CC·BOLT_EUV 둘 다 끝까지 돌고, CAN이 유효하며 페달 명령 0x200이 나간다. 수정 전 코드는 둘 다 첫 갱신에서 같은 AssertionError로 죽는다.
- 못 한 것: 볼트 실차·실기기 확인. 그 기기의 실제 오류 출력은 rlog에 남지 않아 보지 못했다.

## 참고

그 rlog의 경로 시작 시점 `CarSelected3`는 "Chevrolet Bolt EUV 2022-23"(ACC 모델, `CHEVROLET_BOLT_EUV`)이다. 직전 경로는 `CHEVROLET_BOLT_CC`(No-ACC, 수동 선택)로 돌았다.
ACC가 없는 차에 페달을 단 경우라면 "Chevrolet Bolt EUV 2022-23 - No-ACC"를 골라야 한다.
