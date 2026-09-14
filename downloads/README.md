# Hylink Dev 1.6.5 — 2026-09-14

[1.6.5 APK 다운로드](https://github.com/leehyuk1108/carrotpilot/raw/refs/heads/hylink-app/downloads/hylink-dev-1.6.5.apk)

차량 탭 맨 아래 앱 업데이트 버튼을 추가했습니다. 자동 버전 확인, 명시적 다운로드,
파일·서명 검증, Android 설치 확인 화면을 지원합니다. 1.6.4 사용자는 이번 APK를
한 번 수동으로 덮어 설치하면 이후 앱 안에서 업데이트할 수 있습니다.

- 패키지 `app.hylink.mobile.debug`, versionCode `13`, versionName `1.6.5-app-updates-debug`
- 14,008,928 bytes, Android 7.0 이상
- SHA-256: `8e6d5680c4ac8d08d618069363d49f35687619b9f6f5a90efa6586724299d8bf`
- 원래 개발 인증서 SHA-256: `55a14240fb656db17c66c695a61ae5679982a5ae24e233563a43211c5125fa94`
- JVM 테스트 16개(기존 API 5 + 업데이트 정책 11), UI 상태/12개 레이아웃 검사 통과.
- [동작·검증 범위·다음 버전 게시 방법](../docs/APP_UPDATES.md)

콤마용 브랜치는 `wip`입니다. `hylink-app`은 Android 앱 소스이므로 콤마에 설치하지 마세요.
차량 주행 코드나 기존 라이브 간헐 끊김·전원 종료 문제를 수정하는 APK는 아닙니다.

## 이전 버전: Hylink Dev 1.6.4

[APK 다운로드](https://github.com/leehyuk1108/carrotpilot/raw/refs/heads/hylink-app/downloads/hylink-dev-1.6.4.apk)

실제 휴대폰에 설치해 확인한 APK 원본입니다. GitHub Releases 첨부가 아니라
`leehyuk1108/carrotpilot` 저장소의 Hylink 앱 전용 `hylink-app` 브랜치에서 다운로드됩니다.
차량에 설치하는 브랜치는 같은 저장소의 `wip`이며, `hylink-app`을 콤마에 설치하면 안 됩니다.

## 버전과 설치

- 앱 이름: Hylink Dev
- 패키지: `app.hylink.mobile.debug`
- versionName: `1.6.4-live-view-only-debug`
- versionCode: `12`
- Android 7.0 이상, targetSdk 36
- APK 크기: 13,939,087 bytes
- 앱 기능 소스: `d4eda04da492e4402d9cd4699c087de9cb7b37b8`
- 이후 `ad993a8d4`는 로컬 진단 스크립트만 추가하며 APK 기능 소스는 같습니다.
- SHA-256: `5f00d0358cfd4ed713ea19481c5938da2ab92c711e644403fa0dd405fd8a334f`
- 서명 인증서 SHA-256: `55a14240fb656db17c66c695a61ae5679982a5ae24e233563a43211c5125fa94`

기존 Hylink Dev와 같은 서명으로 만든 개발용 APK입니다. 기존 앱을 지우지 말고
덮어 설치하면 됩니다. 다른 서명으로 설치한 앱에는 덮어 설치할 수 없습니다.
My Traverse New는 별도 앱이며 이 APK로 바뀌지 않습니다. Wayon Cloud 키와
SSH 개인키는 APK에 포함하지 않습니다. 이 개발용 APK는 최종 배포용 보안 인증이나
Play Store용 릴리스 서명을 마친 빌드라는 뜻이 아닙니다.

## 변경과 연결

- 라이브의 수동 사진 촬영·10/30초 영상 저장 버튼과 전송 코드를 제거했습니다.
- 라이브 보기, 자동 주차/충격 사진, 기존 사진·영상 조회와 재생은 유지합니다.
- 영상이 오래 갱신되지 않으면 수신 지연을 표시하고, 다시 수신되면 복구합니다.
- 라이브 하트비트를 Wayon Cloud가 전달하는 바이너리 규격에 맞췄습니다.
- 차량 `leehyuk1108/carrotpilot:wip`과 연동합니다. 시동을 끄고 같은 개인 네트워크에서
  `http://콤마IP:1108`에 접속해 키를 복사한 뒤 앱의 **차량 → Wayon Cloud 키**에 넣습니다.
- 키는 차량 위치·카메라·원격 터미널 접근 권한이므로 공유하거나 공개하지 마세요.

## 확인된 범위와 남은 문제

실제 Android 휴대폰과 comma 4에서 키 연결, 상태 조회, 자동 사진 조회,
라이브 영상 수신, 기존 영상 재생, 원격 터미널의 읽기 명령 실행 및
라이브 종료/앱 백그라운드 전환 후 카메라 자원 회수를 확인했습니다.
휴대폰에 설치한 파일과 이 파일의 버전·서명·해시가 일치합니다.

장시간 라이브에서 간헐적인 전송 끊김이 관찰됐고 원인은 확정되지 않았습니다.
차량 `03c4a8ec`의 빠른 재연결 보완은 자동 회귀 검사를 통과했지만,
해당 보완을 설치한 실기기에서의 장시간/강제 단절 시험은 아직 하지 않았습니다.
차량 전원 종료 원인도 확정되지 않았으며 APK 업데이트로 해결됐다고 볼 수 없습니다.
실제 주행 중 전환과 물리적 충격 촬영은 별도 검증이 필요합니다.
앱이 닫혀 있을 때 Firebase 푸시 알림은 구현되지 않았습니다.

Hylink 주차 기능의 11.5V 기준은 기기 전체의 전원 유지 설정이 아닙니다.
콤마 자체의 저전압/오프로드 시간 종료 조건은 유지되며, 전원이 꺼진 콤마를
이 앱으로 다시 켤 수는 없습니다. 모든 기능의 무오류 보증 버전이 아닙니다.
