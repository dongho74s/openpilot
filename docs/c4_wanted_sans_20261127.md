# C4 글꼴: Pretendard → Wanted Sans (2026-11-27)

요청: "콤마 폰트도 wanted sans로 바꿔주고 깨지는 글자 없는지 실기기에서 테스트까지".
당근웹은 같은 날 Wanted Sans 가변 웹폰트로 바꿨다(ef058434). 이번엔 C4 화면(raylib, `BIG_UI=False`) 차례다.

## 바꾼 것

- C4의 모든 글자 역할(NORMAL·MEDIUM·BOLD·SEMI_BOLD·DISPLAY·DISPLAY_REGULAR·ROMAN, 스피너의 PRETENDARD 역할)이
  `WantedSans-SemiBold.fnt`(200px)를, 한글은 `WantedSans-KR-SemiBold.fnt`(48px)를 쓴다. 굵기는 예전과 같은 SemiBold(600).
  크기·위치·FONT_SCALE은 그대로다. C3(BIG_UI=True)는 기존 Inter·KaiGen·Pretendard를 유지한다.
- 원본: 공식 저장소 wanteddev/wanted-sans `packages/wanted-sans/fonts/ttf/WantedSans-SemiBold.ttf`
  (sha256 `1378a491…de89d70`, 2,352,124바이트, OFL 1.1, 라이선스는 같은 폴더 `OFL.txt`).
- 원본은 `fonts/wanted/`에 둔다. `fonts/*.ttf`에 두면 기존 생성기(process.py)와 그 SCons 명령이 새 원본을 보고
  번들된 아틀라스 전체를 다시 만들어, 기기의 추적 파일(Pretendard-*.fnt/png)을 더럽힌다.
- `fonts/process_wanted.py`가 두 아틀라스를 만든다. SCons가 부팅 빌드 때 실행하고, 결과는 git에 넣지 않는다.
  옛 `process_korean.py`(Pretendard-KR)와 그 SCons 명령은 쓰는 곳이 없어져 지웠다. 기기에 남은 옛 생성 파일은 무해하다.
- 아틀라스가 아직 없을 때(빌드 전 스피너)는 `_resolve_font_path`가 `fonts/wanted/WantedSans-SemiBold.ttf`를 직접 읽는다.
- 시동 실패 복구 화면(startup_recovery.py)도 같은 TTF를 쓴다. 파일이 없으면 예전 Pretendard-Medium으로 돌아간다.

## 빠진 글자 처리

Wanted Sans에는 UI가 쓰는 글자 몇 개가 없다. 특히 한국어 번역의 "Wi‑Fi"에 들어간 U+2011(줄바꿈 없는 하이픈)이 없어서,
그대로 두면 "Wi‑Fi 네트워크 검색 중..." 같은 문장 전체가 unifont(픽셀 글꼴)로 바뀐다.
그 밖에 ↳ ⇧ ⌫ ␣ ◀ ▶, 키릴 문자(우크라이나어 번역), 일부 CJK 문장부호가 없다.

Pretendard SemiBold는 UPM 2048, hhea 1950/-494, 굵기 600이 Wanted Sans와 같다. 그래서 생성기는
Wanted Sans에 없는 코드포인트만 같은 크기로 Pretendard에서 래스터해 한 아틀라스에 합친다
(라틴 75자 중 68자, 한글 아틀라스 1194자 중 180자). 둘 다 없는 글자(⚙ ✔ ✕, 한자 등)는 예전처럼 실행 중 unifont로 간다.

raylib 6(기기 6.0.0.1, 맥 6.2)의 7인자 `LoadFontData`는 글꼴에 없는 코드포인트를 결과에서 뺀다.
생성기는 이 결과로 빠진 글자를 찾는다. 옛 빌드처럼 빈 항목을 남기는 경우도 걸러 낸다.

## 확인한 것

- 글자 범위: 기존 기기 아틀라스(Pretendard)와 비교해 실제 글자가 사라진 코드포인트가 없다. 라틴 쪽에서 빠진 7개
  (`\n` ⚙ ✔ ✕ U+FE0F 🌧 🔥)는 옛 아틀라스에서도 폭 0·크기 0인 빈 항목이었다.
- 기기 CPU 생성(임시 폴더, 시동 감시): 13초, 최대 메모리 약 190MB. 라틴 227자는 맥 결과와 같다.
  한글 아틀라스는 맥 11,561자 / 기기 11,560자이고, 차이는 보이지 않는 한글 채움 문자 U+3164 하나다.
- 글자 폭: Wanted/Pretendard 비율 숫자 1.003, 대문자 1.024, 소문자 1.019, 한글 1.003, 타이머 1.012.
- 맥 시험: `system/ui/lib/tests` 전체, 알림 배치(test_mici_alert_presentation), 복구 화면 시험 114개 통과.
  실패 1건(`mici/layouts/welcome.py:21`의 `GuiTextAlignment`)은 이번 변경 전부터 있던 것이다.
  `test_handle_state_change.py`(jeepney 없음), `test_wifi_scanner.py`(pytest_mock 없음)는 맥에서 수집되지 않아 뺐다.

실기기 확인(배포·재부팅 뒤 아틀라스 빌드, 기기 GPU 화면 렌더)은 아래에 이어서 적는다.
