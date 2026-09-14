# Hylink

This `carrotpilot:hylink-app` branch contains the Android app, not openpilot.
Do not install this branch on a comma. Use `carrotpilot:wip` for the vehicle.

Hylink is an Android companion for a vehicle connected to Wayon Cloud.
It is a separate application from My Traverse and uses a distinct package ID:
`app.hylink.mobile`.

## Download: Hylink Dev 1.6.6

**Validation warning:** The 1.6.5 → 1.6.6 in-app installation succeeded and Google
Play Protect reported the APK safe, but Samsung subsequently flagged it as a
potentially harmful app installed after a phishing attempt. The Samsung trigger
is unresolved; this is not a security-cleared production release. See the
[device test record](docs/APP_UPDATES.md#166-재시험--설치-성공-삼성-경고-미해결).

[Download the APK](https://github.com/leehyuk1108/carrotpilot/raw/refs/heads/hylink-app/downloads/hylink-dev-1.6.6.apk)

This is the existing Hylink Dev package (`app.hylink.mobile.debug`, versionCode 14),
signed with the original development certificate for in-place updates. It is not
My Traverse New or a production-signed Play Store build. Keep the existing app
installed when updating. See [changes, checksum and validation limits](downloads/README.md).

Version 1.6.5 added **Vehicle → App update** at the bottom of the vehicle tab.
It checks GitHub automatically and downloads only on request. Android asks for
installation approval. Version 1.6.6 changes only the version number to exercise
the published 1.6.5 → 1.6.6 update path. Users on 1.6.5 can use the in-app button;
users on 1.6.4 or earlier must install this APK manually once.
See [updater behavior and release procedure](docs/APP_UPDATES.md).

## wip connection (1.6.1)

Open `http://<comma IP>:1108` on the same private network while offroad, then paste
the displayed key into **Vehicle → Wayon Cloud key**. Driving telemetry remains
connected; live/media/impact/SSH are enabled by default but remain offroad-only.
The key page has no setup checkboxes: opening it connects all features automatically.
See [setup, changes and verification boundaries](docs/WIP_CONNECT_20260911.md).

## Approved dashboard (1.5.0)

The production entry point is `app/src/main/assets/main.html`, using
`hylink-app.js`, `hylink-model.js` and the three `hylink-*.css` dashboard styles.
The older `hylink.js`/refinement assets are not loaded. The standalone
`design/cloud-overview/` remains a synthetic design reference, not app data.
See [implementation and verification notes](docs/APP_UI_20260910.md).

## Implemented features

- Current vehicle location and Wayon telemetry
- Onroad/offroad, ignition, speed, bearing, GPS quality, voltage, current, and power
- openpilot state, availability, engageability, personality, mode, and current alert
- Cloud trip history with distance/time/speed insights and a saved route map
- Parking and impact snapshots with camera, size, capture, and sensor metadata
- Viewing existing saved photos and 10/30-second clips
- Offroad 360-degree Live viewing (manual photo/video saving removed in 1.6.4)
- Impact force and jerk, with detected time and severity
- Device CPU/GPU/memory/storage usage and detailed thermal sensors
- Network quality, screen state, electrical flow, and estimated offroad energy
- Panda connection, harness, safety model, counters, fault health, and uptime
- Offroad-only remote SSH terminal through the per-device Wayon relay

Vehicle features use only Wayon Cloud data, camera, and remote-session endpoints.
The app updater separately reads public files from `leehyuk1108/carrotpilot` on
GitHub; no vehicle key or telemetry is sent to GitHub. Account
vehicle status and lock state bundled by the Cloud are removed before rendering. Hylink
contains no vehicle commands, account integration, diagnostic clearing, remote
start, door lock, climate, window, widget command, or Wear OS command code.

## Configuration

The Wayon Cloud key is entered in the app and stored in its private Android
preferences. No key is committed or embedded in the APK.

The first terminal connection creates a dedicated 3072-bit RSA key inside the
app-private, non-backed-up files directory. Its public key must be added to the
comma device's SSH key list once. The private key never leaves the phone. Wayon
still restricts the relay to the Dongle ID bound to the supplied Cloud key, and
the device-side relay stops on Onroad transition.

The default Cloud address is:

```text
https://wayon-cloud.hyuklee.workers.dev
```

Override it for a local build with `wayon.cloudUrl` in `local.properties` or
the `WAYON_CLOUD_URL` environment variable.

## Build

Use JDK 17 or newer:

```bash
./gradlew :app:testDebugUnitTest :app:assembleDebug
```
