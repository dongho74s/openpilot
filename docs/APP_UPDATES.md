# Hylink Dev 앱 업데이트 — 1.6.5

## 사용 방법

차량 탭 맨 아래 **앱 업데이트**에서 현재 버전과 확인 결과를 볼 수 있습니다.
차량 연결 키 없이도 업데이트 기능을 사용할 수 있습니다.

1. 앱 실행/전경 복귀 시 업데이트를 확인합니다. 자동 확인은 6시간 간격으로 제한합니다.
   **업데이트 확인** 버튼은 이 대기 시간을 건너뜁니다.
2. 새 버전이 있으면 차량 탭에 작은 표시가 생기고 **업데이트 다운로드** 버튼이 표시됩니다.
   자동으로 다운로드하거나 설치하지 않습니다.
3. 다운로드 진행률과 취소 버튼을 표시합니다. 앱을 백그라운드로 보내면 진행 중 작업을
   중지합니다. 다시 다운로드할 때 처음부터 받습니다. 별도 상주 서비스는 없습니다.
4. 파일 검증이 끝나면 **업데이트 설치**를 누릅니다. 처음에는 Android의
   **이 출처의 앱 설치 허용** 화면으로 안내할 수 있습니다. 허용 후 앱으로 돌아와
   설치를 다시 누르면 Android 시스템 설치 확인 화면이 열립니다.
5. 설치를 취소/실패하면 기존 앱은 유지됩니다. 성공 시 기존 연결 키와 데이터를
   그대로 사용하는 앱 업데이트이며 앱 삭제나 데이터 초기화를 하지 않습니다.

라이브/터미널 세션을 사용하는 동안에는 업데이트 확인·다운로드·설치를 새로 시작하지 않습니다.
앱 설치는 Android의 정상 사용자 확인을 거칩니다. `UPDATE_PACKAGES_WITHOUT_USER_ACTION`,
특권 설치 권한, 루트 권한, 앱 설치 허용의 강제 변경을 사용하지 않습니다.
기기 정책/자동 차단 기능이 외부 APK 설치를 막으면 사용자가 정책을 확인해야 합니다.

1.6.4에는 업데이트 코드가 없으므로 **1.6.5를 한 번 수동 설치**해야 합니다.
이후 GitHub에 새 APK와 업데이트 정보를 함께 게시하면 앱에서 발견할 수 있습니다.
차량용 `wip` 코드/업데이트와는 별개이며 이 기능은 콤마에 브랜치를 설치하지 않습니다.

## 다운로드와 검증

- 공개 메타데이터: `https://raw.githubusercontent.com/leehyuk1108/carrotpilot/refs/heads/hylink-app/downloads/update.json`
- APK: 같은 저장소의 **40자리 커밋 SHA에 고정된** `downloads/hylink-dev-x.y.z.apk`만 허용합니다.
  다른 호스트/저장소/HTTP/가변 브랜치/쿼리/fragment/redirect를 받지 않습니다.
- manifest 최대 32 KiB, APK 최대 80 MiB. 연결/읽기/전체 요청 시간도 제한합니다.
- 메타데이터의 채널·패키지·정수 버전·최소 Android·크기·SHA-256 형식을 검사합니다.
- 다운로드 바이트 수와 SHA-256, APK 내부 패키지/버전명/버전 코드/최소 Android,
  설치된 앱과의 **현재 서명 인증서 집합 일치**를 검사합니다. 같거나 낮은 버전은 설치하지 않습니다.
- 설치 직전 파일과 서명을 다시 검사하고 Android 설치기의 최종 서명 검증도 거칩니다.
- APK는 전용 private cache에만 저장하고 FileProvider는 그 디렉터리만 노출합니다.
  검증된 단일 APK에만 일시적 읽기 권한을 줍니다. 부분 다운로드는 실패/취소 때 삭제합니다.
- GitHub 요청에 Wayon Cloud 키, SSH 키, 차량 위치, 계정 정보를 포함하지 않습니다.
  업데이트 네트워크 작업은 Cloud 갱신 실행기와 분리합니다.
- 공개 manifest 자체는 별도 오프라인 서명 문서가 아닙니다. HTTPS, 고정 저장소/커밋,
  파일 해시와 설치된 앱의 인증서를 함께 확인합니다. 기존 앱 서명키를 안전하게 보관해야 합니다.

## 다음 버전 게시 절차

대상 저장소는 **leehyuk1108/carrotpilot**, 앱 브랜치는 **hylink-app**입니다.
Wayon/Sunnypilot이나 차량용 wip에 앱 소스/APK를 덮어쓰지 않습니다.

1. `versionCode`를 증가시키고 `versionName`을 변경합니다. 기존 개발 앱을 업데이트하려면
   같은 패키지 `app.hylink.mobile.debug`와 원래 서명키를 유지합니다.
2. 단위 테스트·UI/보안 검사·빌드를 실행하고 APK를 원래 인증서로 서명합니다.
   기본 Mac debug.keystore는 원래 기기 인증서와 다를 수 있으므로 반드시 비교합니다.
3. 버전별 APK를 `downloads/`에 추가하고 소스/APK를 커밋합니다. 기존 APK를 덮어쓰지 않습니다.
4. 그 커밋 SHA의 raw URL, 실제 APK 해시·바이트 수·패키지/버전·최소 SDK로
   `downloads/update.json`을 갱신합니다. `schemaVersion: 1`, `channel: hylink-dev`를 유지합니다.
5. `node scripts/check-update-publication.mjs`로 로컬 APK와 지목한 Git blob의 일치를 확인합니다.
6. 메타데이터를 커밋하고 명시적 `HEAD:refs/heads/hylink-app`으로 푸시합니다.
   내려받은 manifest/APK와 설치 결과를 다시 확인한 후 완료로 보고합니다.

기존 버전은 `hylink-dev-v1.6.4` 태그에 남아 있습니다. 서명키 변경/정식 패키지로의
전환은 별도의 마이그레이션 계획이 필요하며 업데이트 검사를 느슨하게 해서 우회하지 않습니다.

## 검증 및 UI 원칙

- JVM: 허용 저장소/커밋, 타입·크기 제한, 해시, 부분 파일, 취소, 잘못된 인증서,
  다른 패키지/버전, 다운그레이드 등을 검사합니다.
- 브라우저: 실제 앱 asset에 합성 native callback을 넣어 상태별 액션과 12개
  화면 폭/글자 크기/테마 조합을 검사합니다. 실제 설치 성공으로 간주하지 않습니다.
- Apple Design 원칙의 **Layout / Accessibility / Feedback / Loading**을 적용했습니다.
  기존 차량 카드와 같은 색·여백, 48px 이상 터치 영역, 큰 글씨 재배치, 밝고 어두운
  테마에서 4.5:1 이상 상태 문구 대비, 상태 읽기·진행률·취소·재시도 표시를 사용합니다.
  불필요한 자동 팝업으로 홈 화면이나 라이브를 가리지 않습니다.
- 실기기 검증은 별도로 기록합니다. 앱 업데이트 기능이 차량 주행 안전이나
  기존 라이브 간헐 끊김·전원 종료 문제를 해결했다는 뜻은 아닙니다.

API 근거: [Android 설치 권한](https://developer.android.com/reference/android/content/pm/PackageManager#canRequestPackageInstalls()),
[FileProvider](https://developer.android.com/reference/androidx/core/content/FileProvider),
[앱 서명과 업데이트](https://developer.android.com/studio/publish/app-signing#considerations).
