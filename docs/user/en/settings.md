# Understanding Settings

[한국어](../ko/settings.md)

> [!NOTE]
> This is the canonical English user guide maintained with the `carrot-wip` code. When user-visible behavior changes, update this document together with the related code and tests.

Use **Carrot Web** to view and change all carrotpilot-specific settings. The device settings screen remains useful for Wi-Fi, device information, standard openpilot toggles, and software updates. Parameters defined by `carrot_settings.json` belong in the **Settings** screen in Carrot Web.

An accelerometer-detected suspected horizontal impact of at least 1.5g displays a warning and a ten-second cancellation notice. Without a touch, it saves `OpenpilotEnabledToggle` OFF and reboots into Dashcam mode, preventing control until manually enabled again. See [detection, cancellation, recovery, and recording interruption](dashcam-log-sharing.md#automatic-dashcam-mode-after-a-suspected-impact).

> [!IMPORTANT]
> **Current support status**
>
> - Supported settings interface: **Carrot Web**
> - CarrotMan: not supported as a user app or external integration
> - CarrotLink: not supported
> - Older CarrotMan or CarrotLink connection instructions no longer describe the current workflow
>
> A `carrotMan` name may still appear inside code or messages. It does not mean that the former user app is supported.

The driving model (the internal model running on the comma's own GPU) is changed in **Tools → Driving model card**, not on the settings page. It can only be changed while parked with the ignition off and is used from the next drive. See the [Carrot Web guide](carrot-web.md#changing-the-driving-model-internal-model).

## Connecting to Carrot Web

**Speed & Decel > Speed Camera > Navdy Connection Mode** selects the existing
configuration, wired USB, or wireless Bluetooth. Wired mode uses direct Comma USB
or a paired Jetson's USB relay, without Bluetooth fallback. Wireless mode leaves
USB available for diagnostics. Phone reconnection, ambient BLE and Navigation
Input are unchanged. Change modes while parked because navigation briefly stops
during handover. The first update needs a native build and fresh manager; later
changes are checked every five seconds plus any in-progress connection attempt.

1. Connect the comma device and your phone or computer to the same network.
2. Open `http://DEVICE-IP:7000` in a browser.
3. Select **Settings** from the top menu.

For example, if the device IP is `192.168.0.25`, open:

    http://192.168.0.25:7000

See [Carrot Web](https://github.com/ajouatom/openpilot/wiki/Guide-Carrot-Web) for connection troubleshooting and an overview of the other screens.

### Web layout defaults

On a fresh install, `Tools > Web Settings > Layout` starts with **Area 1 full screen** in both orientations: Area 1 is **Carrot Vision** and Area 2 is **Carrot Navi**. **Default** restores both orientations to Area 1 full screen with Carrot Vision in Area 1 and Carrot Navi in Area 2. See [Carrot Web layout](carrot-web.md#layout) for the per-screen details.

## Using the Settings screen

### C4 display selection: new ui

**Display → On-screen Info → new ui** (`NewUi`) takes 0 to 3 and defaults to 0. 0 is the Carrot UI, 1 is the Wayon/sunnypilot-style layout described below (called ON below), 2 is the clock face and 3 is the robot face. A previously enabled ON value stays as 1. On C3/C3X only 1 applies, showing the v10-nui screen described in "C3/C3X display" below.

In new ui, lane lines and road edges are continuous white outlines without lane-type, engagement or steering-torque color changes. Confidence-based opacity and lane geometry remain; the central driving path and lead-bar colors are unchanged. Navdy still receives the original lane types and solid/dashed markings. OFF restores the legacy C4 presentation.

ON uses the full 536×240 display. The right-hand confidence ball and traffic-light indicator are hidden and their reserved 60px column is removed. Camera projection, lanes, front-vehicle markers, BSM and the HUD share the full width. OFF restores the original 476px camera area and right-hand status column. This is a display change only; signal detection, alerts and control behavior remain unchanged.

**Display → On-screen Info → new ui** (`NewUi`) defaults to OFF. ON selects the Wayon/sunnypilot-style C4 onroad layout: a bottom steering wheel and torque bar, plus MAX set speed for about 2.5 seconds after a change. The persistent large speed/gear/debug panel and VISION status card are hidden. Camera, lanes, surrounding vehicles, radar/vision lead markers, blind-spot warnings, parking/hold timers and primary alerts remain. DM moves to the top left and yields to MAX, timers and alerts. Lateral-only engagement retains a green steering wheel. Cruise gap, driving mode, NAVI connection text, slowdown target speed and the old reason panel are hidden; select OFF when those details are needed.

The front-lead rectangle becomes a rounded 3px bar at the projected vehicle base: green `#20D86A` for a radar-backed lead, bright blue `#50BEFF` for vision-only. Width follows distance within 24–140px. It fades in over 0.18s and out over 0.20s; a missing or stale lead is never held indefinitely. A new session, layout change or offroad transition clears the old marker. Colors indicate detection source, not control activation or braking. `ShowRadarInfo` does not control this main-lead marker. OFF retains the original rectangle; Navdy transmission and detection/control inputs are unchanged.

The selected slowdown reason appears as a compact filled badge to the right of the wheel, above the torque bar (`TURN`, `CAM`, `BUMP`, `SECTION`, etc.), with subtly rounded corners. It is shown only with longitudinal control active and the corresponding cap applied in a fresh plan; an unapplied candidate is hidden. While applied, it flashes green/white once per second and keeps its dark label visible. This indicates an applied speed cap, not physical brake actuation. Ending the cap or a pedal override hides the badge immediately; reactivation starts green. Only the selected reason is shown, not all candidates. Stale navigation, alerts and parking/hold timers hide the badge.

BSM uses the original Wayon orange vehicle artwork (108×128, 20px from each content edge, y=100), with its 0.15s first-order fade filter. This replaces the Carrot edge strips and projected BSM walls only in new ui. OEM and vision detections still use the existing merged carState flags; their source colors are not distinguished in this layout. The ordinary wheel yields while BSM is visible, including lateral-only engagement; the torque indicator remains. Left BSM takes priority over transient MAX, and the slowdown badge has its own space at x=136. Critical steering warnings and parking/hold timers take priority over the BSM artwork. Invalid, stale or previous-session carState clears BSM immediately without fading an old warning. Detection and control logic are unchanged.

DM uses the original Wayon person silhouette and direction cone instead of the animated eyes and status label. Its 60×60 area is 16px from the left and 10px from the top. Existing monitoring state drives green/orange coloring and the original animated phone icon. Previous-session, stale/frozen data (0.5s), camera failure or face loss cannot retain the green cone or phone. NewUi OFF keeps the existing DM presentation (C3/C3X option 1 is described below). Monitoring decisions, alerts, sounds and engagement conditions are unchanged.

BSM/DM attribution: This project uses software from Haibin Wen and SUNNYPILOT LLC and is licensed under a custom license requiring permission for use. See the [retained upstream license](../../third_party/sunnypilot-bsm-license.md).

**2 (clock)**: while driving, the C4 screen turns off the camera, lanes, speed panel and DM display and shows only a wide analog clock in Korea time: minute and hour marks along the outer edge, 12/3/6/9 numerals, hour and minute hands and a sweeping second hand. The left side shows a steering-wheel icon and the current speed, the right side the weekday (in English) and date. The wheel icon is green while openpilot steers and gray otherwise. After dark the whole face turns red and the second hand is hidden; the wheel icon is then bright red while steering and dark red otherwise. Night uses the same sun rule as the Navdy ambient light: it starts once the sun is 4° below the horizon (computed for Seoul, about 20 minutes after sunset) and ends when it rises above -3° at dawn, fading over about a second. Camera exposure is not used because at night it reads like overcast daytime driving, so daytime tunnels and garages do not switch it. Right after starting it shows the mode for the current time at once; until the comma clock is set it stays in day mode. The parking and hold timers are not shown, nor are the low-priority notices they would have covered while stopped (parking brake, brake hold, resume required, low-speed steering). The stop is still timed in the background, so switching to 1 shows the real elapsed time. Other alerts and critical alerts still appear over the clock. Only the display changes; alert decisions, sounds and vehicle control are unchanged.

**3 (robot face)**: while driving, the C4 screen turns off the camera, lanes, speed panel and DM display, and two glowing rounded-square eyes change expression with the car state. Their colour tells the state: orange at rest, green while engaged, amber when worried or straining, red for danger. The eyes keep breathing, blink every 2-6 seconds, and when nothing happens they pick small habits such as a head tilt, eye contact, a wink, looking around, sunglasses or heart eyes. On engagement they hop with star eyes, then a steering wheel rises from the bottom edge with two robot hands on it, turning the way the car's wheel turns: small angles such as lane keeping are drawn about three times bigger so they show, easing to a stop at about 9 degrees, where the outer hand would leave the screen. The mouth is always drawn above the wheel, never over it. When the driver turns the wheel, or the car drops below the speed it can steer at (10 km/h on this car), the hands let go and rise; they take the wheel again when steering returns. It shows no icons by the screen edges and no driver-monitoring mark: it reacts with its face and motion only. A drive starting gets eyes opening with a hop and a sparkle. A lane change leans it that way, and it nods twice when the change is done. A car in the blind spot makes it flinch back and glare that way with a "!", and the car ahead pulling away makes its eyes go wide with a "!". A cut-in makes it flinch back with a quick double blink, and pressing the gas gets a curious glance at the driver with a question mark. Being too slow to steer gets a little bow toward the driver, and a fast bend swings it outward with wind streaks past it. Hard braking or a forward collision warning turns the eyes red with a "!", and clearing a driver-monitoring alert gets a "phew" puff. Other events: the seatbelt off while driving (a worried look toward the driver, every 30 seconds while it stays off); reverse gear (looking down and beeping); park (a happy goodbye and a sigh); hazard lights (a worried look with slightly red eyes); and the driver steering while it drives (hands off for a moment with a look at the driver, at most every 15 seconds, not during lane changes). It also wears or holds props: headphones while the phone plays a song, a red beacon turning and flashing on its head while the hazard lights are on, a whistle it blows in short bursts during a driver-monitoring alert, and the seatbelt buckle held up and shaken in its driver-side hand when the belt is off (that hand leaves the wheel meanwhile). It does not react to navigation data (speed cameras and limits, turn guidance, speed bumps, arriving at the destination), only to the car and the driver. The cruise set speed shows big on engagement (after the star-eyed hop) and whenever it changes while engaged: the eyes first shut to a line and the number takes their place. On engagement its digits drop in one by one; on a change only the changed digits roll over like an odometer (up: rolling up in green; down: rolling down in amber; chevrons above the number). After 2.4 seconds (counted from the last change) the number shrinks away and the eyes open again. The eyes and the number are never drawn at the same time. A reaction that needs the eyes (hard braking, a driver-monitoring alert, the blind spot, the seatbelt, disengaging) makes the number give way at once. After more than 45 seconds stopped it dozes off in a nightcap and wakes when the car pulls away; the car ahead moving off or a song also wakes it. The harder it steers (from 50% of the car's torque limit, all-out at 95%), the harder it strains: squinting, gritting its teeth, trembling with its hands on the wheel, and screwing its eyes shut (>_<) near the limit. While the phone is actually playing a song (Navdy HUD v186 or later tells the comma only the phone player's playback state, never the title or artist; a paused or stopped track does not count even while its info is shown, playback has to hold for 1.5 seconds before it starts, and it stops at once), it bobs along with floating notes, following the beat the microphone finds or 120 BPM when it finds none, and does not doze while it plays. A reaction that comes up while a more important one is playing (driver-monitoring alerts and hard braking come first) waits 1 to 5 seconds and then plays, or is dropped if its turn does not come. The same reaction does not repeat within a few to tens of seconds. Parking and hold timers and normal or prompt alerts do not cover the face (the robot reacts instead and alert sounds still play); only red critical alerts appear over it. Only the display changes; alert decisions, sounds and vehicle control are unchanged.

The selection refreshes about every 0.5 seconds without a reboot; 0 restores the existing Carrot UI. C3/C3X change only with option 1 (below); it does not change the external cluster, offroad greetings/summary, steering, braking or driver-monitoring decisions. The initial software update must complete its normal build to register the new parameter.

### C3/C3X display: new ui 1 (v10-nui)

On C3/C3X only `NewUi` 1 changes the screen: it shows the onroad screen from the carrotpilot `v10-nui` branch (the FrogPilot testing-v1 design). Its artwork is prepared offroad; switching to 1 while driving keeps the existing screen while it is prepared (usually under a second). 0, 2 and 3 keep the Carrot screen there (2 clock and 3 robot face are C4 only).

- The camera fills the screen and the four screen corners are rounded. With the sidebar open, the outer corners of the whole screen (sidebar included) are rounded while the edge where the sidebar meets the camera stays square; full-screen alerts are rounded too. The status border, Carrot speed panel, navigation guidance, debug text and experimental-mode button are hidden. The top is shaded slightly and the bottom progressively so the HUD stays readable. On a C3X, whose road frame (1344×760) is smaller than the screen, the video is zoomed to fill it, and lanes, path and lead markers use the same zoom so they sit on the road (0, 2 and 3 keep the existing screen).
- Current speed and unit (KM/H or MPH) sit at the bottom centre; the following-gap icon (personality, 1-4 bars) and `SET` speed are to the right, and a steering wheel sits in the bottom-right corner. The wheel turns with the steering angle; while steering it shifts from white towards yellow and red as the steering output grows, and turns blue when the driver steers over it. The steer-limit warning (`steerSaturated`) is shown as an enlarged wheel with a red exclamation mark instead of an alert box.
- The LFA icon right of the speed is green when engaged, mint for lateral-only assist, blue while the accelerator overrides, white when engageable and grey otherwise. With Carrot navigation linked, an `APN` badge appears above the gap icon (blue when connected, green while a camera or section alert is active).
- Turn signals blink green arrows beside the speed. A vehicle in the blind spot shows an orange vehicle icon at that screen edge, blinking fast while that turn signal is on.
- After a minute stopped, the stop duration (m:ss) replaces the current speed.
- Lane lines are white (opacity follows confidence) and road edges red. The path is green (a 0.5 s fade when the throttle allowance changes), acceleration-coloured in experimental mode, mint for lateral-only assist and white in lane mode, where the ego lane lines are thick and green. During a lane change the line being crossed is hidden and the outer lines of the current and target lanes are highlighted in mint.
- When neither openpilot nor lateral assist is active, the camera turns greyscale at 62% brightness, lane lines are hidden and the path is grey.
- With `ShowPathEnd` at 1 or more, leads get a rounded box (red for the radar SCC lead, orange for other radar leads, blue for vision only, ochre for the second lead), with radar (red/orange) and vision (blue) distances below it.
- The bottom-left DM badge is a person silhouette with a gaze cone: green when engaged (including lateral-only), white when engageable and grey otherwise. It dims and centres the cone when camera monitoring is not in charge or the monitoring data is older than 0.5 s, and it hides under alerts.
- Alerts: small alerts use a one-line box at the bottom, mid alerts a two-line box and full alerts the whole screen. An alert without a second line uses the one-line box. FCW/AEB and LDW shade the screen and blink a large icon near the top. Standstill resume (`resumeRequired`) shades the screen and shows an auto-hold icon, the elapsed time and the prompt. When the lead pulls away while stopped (1-30 m ahead, opening by more than 0.8 m/s), "Lead vehicle departed" fills the screen for three seconds; stalled or invalid radar data neither starts nor extends it. This notice is visual only.
- While driving the sidebar starts hidden and a tap opens it. It closes by itself after 10 seconds without a touch on it (touching the sidebar restarts the 10 seconds), and another tap opens it again; tapping the camera view while it is open closes it at once. A sidebar left open under 0, 2 or 3 starts its 10 seconds when the value changes to 1. With 0, 2 and 3 it stays open until tapped, as before. Leaving 1, or opening settings, and coming back restarts the stop and auto-hold timers; a new drive never continues the previous auto-hold time. With an external HUD connected and `ShowCameraWithCluster=0`, the camera area stays black and lanes/path are not drawn.

The v10-nui branch also changed alert texts, sizes and sounds (selfdrived events) and screen brightness handling. This setting changes the display only, so those parts were not brought over: alert decisions, sounds and texts, driving control, driver-monitoring decisions, brightness, the offroad screens and the C4 option 1 screen are unchanged.

Carrot Web provides:

- **Category navigation** through category, group, and section
- **Search** by display title or parameter name
- **Favorites** by pressing and holding an item
- **Profiles** to save and restore named groups of values
- **Compare** to inspect a profile or backup before applying it
- **Reset** to restore the catalog defaults
- **File and QR backups** from the tools screen

A full reset or a backup from another vehicle can change many values at once. Save the current settings as both a profile and a file first.

### Per-setting detailed guides

Select the body of a setting, outside its value control, to open its detail screen. The information box below the existing setting box always shows **Description**, **Popular**, and **History** tabs, with **Description** selected by default. A tab remains available and shows an empty state when it has no data.

- **Description** first uses the setting guide for the current language from the validated GitHub published content.
- Content is shown only when both the published index and guide-page hashes match.
- If the request or validation fails, Carrot Web uses the last validated browser copy.
- If neither a validated remote version nor a validated browser copy exists, it shows an empty state instead of using an on-device document.
- **Popular** shows the collected value distribution as a reference, not as a recommendation.
- **History** shows the latest three changes first and offers the full history when more records exist.
- Network or documentation processing failures do not disable viewing or changing the setting.
- Korean and English Carrot Web sessions use their matching guide language.
- Until a Chinese guide exists, Chinese titles and short descriptions remain Chinese while the detailed panel explicitly falls back to English.
- In the GitHub Wiki sidebar, `User Guide > Understanding Settings > All Settings` follows
  Carrot Web's Korean menu hierarchy and order. The central settings catalog remains the
  multilingual Korean, English, and Chinese directory.

The information box stays vertically below the existing setting box in both portrait and landscape layouts. Multiplier and default actions keep their existing position and behavior inside the setting box.

Validated published guides are generated against the current setting code's defaults, ranges, and options. If a short catalog description and the detailed guide differ, diagnose the issue together with the current branch's setting code.

## Reading values and defaults

Each entry in `carrot_settings.json` contains:

| Field | Meaning |
|---|---|
| `name` | Unique name used by the code and Params |
| `title` | Title shown in Carrot Web |
| `descr` | Direction of adjustment, mode numbers, and cautions |
| `min` / `max` | Allowed range |
| `unit` | Step used by the `+` and `-` controls |
| `default` | Initial catalog value |

### The current value comes first

For a 2026 Staria EV, use `Hyundai Staria EV 2026` in vehicle selection. Existing Staria selections are not migrated automatically; check HDA2 and camera SCC settings separately against the vehicle wiring. See [vehicle selection](carrot-web.md#tools-page).

Persistent values stored on the device may remain after an update. Record the value currently shown by your device before relying on the JSON `default`, a number in a title, or another user's value.

A catalog default is not a recommended tune for every vehicle. Vehicle tuning, existing Params, and the branch from which the device was updated can all affect the starting value.

### Check the scale and unit

| Display example | Stored value | Effective value |
|---|---:|---:|
| `TFollowGap1 x0.01s` | `110` | 1.10 seconds |
| `SteerActuatorDelay` | `30` | 0.30 seconds |
| `LateralTorqueAccelFactor x0.001` | `2500` | 2.500 |
| `StopDistanceCarrot` | `600` | 6.00 m (600 cm) base value |
| Percentage `%` | `105` | 105% |

Ignoring `x0.01`, `x0.001`, `cm`, `km/h`, or `%` can make a value appear one hundred or one thousand times larger than it really is.

## Settings map

The current `carrot_settings.json` contains **199 parameters**. The driver-monitoring exception and `NNFF` are search only; the remaining entries appear in these menus:

| Category | Count | Groups |
|---|---:|---|
| Driving control | 123 | Startup and auto, buttons and presets, steering, speed and deceleration, cruise and following gap |
| Vehicle and hardware | 18 | Hyundai/Kia, CAN FD/HDA, radar, driver monitoring, vehicle assistance, device hardware |
| Display | 34 | Information, path, brightness/on-road view, external HUD |
| System | 12 | Recording/power, network/map, sound, software |

## Driving control

These 123 settings can affect vehicle motion. Change one item at a time.

<a id="start-auto"></a>
### Startup and auto — 9 settings

| Section | Parameters | Purpose |
|---|---|---|
| Startup | `AlwaysLateral`, `AutoEngage`, `DisableMinSteerSpeed` | Always-on lateral control, automatic engagement, and low-speed steering limits |
| Auto cruise | `AutoCruiseControl`, `AutoGasTokSpeed`, `AutoGasCancelSpeed`, `AutoGasSyncSpeed`, `CruiseOnDist` | Automatic cruise activation and accelerator-pedal behavior |

- `AlwaysLateral` permits lateral control even when cruise is not engaged. On supported Tesla vehicles it can also steer at true standstill in a forward-driving gear; moving below the minimum steering speed remains restricted. See [Tesla engagement](tesla.md#engagement-and-standstill).
- Always Lateral waits for the first confirmation of valid model, vehicle-parameter, pose, and other required inputs after system startup. Once confirmed, this startup check ends; later input interruptions or disengagement/reengagement do not rearm it. Existing runtime safety handling remains unchanged. Expiry of the startup waiting period alone does not pass this check.
- `AutoEngage`: `0` off, `1` lateral on, `2` lateral on with cruise ready.
- `AutoCruiseControl` covers Hyundai/Kia auto-cruise and soft-hold behavior.
- In the shared `WIP-FR` cruise logic, `AutoCruiseControl=0` also prevents automatic-activation speed selection from changing the set speed when a pedal is released. A gas tap while already engaged still increases the set speed to the next configured step; it does not enable automatic engagement. SET/RES buttons, the separate `AutoGasSyncSpeed` setting while holding the accelerator, paddles, and road/model speed features retain their existing behavior.
- `DisableMinSteerSpeed` is vehicle-specific and relates to low-speed steering restrictions on SMDPS-equipped cars.

### Buttons and presets — 16 settings

Select a section title for the code-based state machine, units, and application conditions.

| Section | Parameters | Purpose |
|---|---|---|
| [Button modes](buttons-presets.md#button-modes) | `CruiseButtonMode`, `NavigationSetSpeed`, `CancelButtonMode`, `LfaButtonMode`, `PaddleMode` | Cruise, SET engagement speed, cancel, LFA, and paddle-button behavior |
| [Speed units](buttons-presets.md#speed-units) | `CruiseSpeedUnit`, `CruiseSpeedUnitBasic`, `CruiseButtonLongDelay` | Short/long-press increments and long-press timing |
| [Button-message tests](buttons-presets.md#button-spam) | `CruiseButtonTest1`, `CruiseButtonTest2`, `CruiseButtonTest3` | Values used to synchronize the stock SCC set speed |
| [Speed presets](buttons-presets.md#speed-presets) | `CruiseSpeed1` through `CruiseSpeed5` | Speed table used by custom mode 3 |

The result depends heavily on whether the car uses stock SCC and which button message the vehicle accepts. Diagnose unexpected behavior with the normal `CruiseButtonMode=0` behavior first.

Volkswagen's separate `SET` button sets current speed and `RES` restores the previous set speed, while `+`/`-` follow the button mode, speed units, and long-press setting. Manual engagement with openpilot longitudinal control remains tied to the physical `SET`/`RES` buttons.

With `NavigationSetSpeed` enabled, `WIP-FR` SET engagement uses the greater of current speed and a valid navigation road limit plus 10 km/h (plus 5 for a 30 km/h limit); see [navigation-based initial set speed](buttons-presets.md#button-modes). The toggle defaults off and updates within about 0.1 seconds for the next SET engagement. Missing/stale navigation, stock ACC, and buttons while already engaged retain their existing behavior.

<a id="vehicle-steering"></a>
### Vehicle steering — 44 top-level + 6 ONNX detail + 1 search-only setting

| Section | Parameters | Purpose |
|---|---|---|
| ONNX Lane and BSD | `ShareData`, `OnnxBsdEnabled`, `OnnxLaneThreshold`, `OnnxLaneIntervalMs`, `OnnxBsdThreshold`, `OnnxBsdSmoothingMs`, `OnnxBsdIntervalMs` | Lane-type detection plus separately controlled camera BSD and tuning |
| Centering | `PathOffset`, `CameraYawTrimDeg`, `ClampEnable`, `ClampMinDistLane`, `ClampMinDistEdge`, `ClampSteerGain`, `ClampSteerAlert` | Lane-mode path position, camera yaw trim, the lane-line and road-edge minimum-distance clamp (lane mode and laneless) |
| Steering feel | `SteerActuatorDelay`, `LatSmoothSec`, `LatSuspendAngleDeg`, `CustomSR`, `SteerRatioRate`, `LateralAccelLimit`, `LatJerkMax` | Timing, smoothing, suspension angle, steering ratio, lateral acceleration and jerk limits |
| [Lane change](lane-change.md) and automatic turn | `LaneChangeNeedTorque`, `LaneChangeDelay`, `LaneChangeBsd`, `LaneLineCheck`, `BlinkerLatchedTurn`, `LaneChangeLeverWait`, `AutoTurnControl`, `AutoTurnControlSpeedTurn`, `AutoTurnControlTurnEnd`, `AutoTurnMapChange` | Lane-change entry conditions and ATC behavior |
| Lane mode | `LatMpcPathCost`, `LatMpcMotionCost`, `LatMpcAccelCost`, `LatMpcJerkCost`, `LatMpcSteeringRateCost`, `LatMpcInputOffset`, `UseLaneLineSpeed`, `UseLaneLineCurveSpeed`, `AdjustLaneOffset` | Lane-mode MPC weights and lane-line conditions |
| Advanced torque | `LateralTorqueCustom`, `LateralTorqueAccelFactor`, `LateralTorqueFriction`, `LateralTorqueKpV`, `LateralTorqueKiV`, `LateralTorqueKf`, `LateralTorqueKd` | Custom torque-control gains |
| Advanced torque, search only | `NNFF` | Neural-network feedforward for torque steering |
| Steering limits | `CustomSteerMax`, `CustomSteerDeltaUp`, `CustomSteerDeltaDown`, `CustomSteerDeltaUpLC`, `CustomSteerDeltaDownLC` | Maximum torque and torque-rate limits |

`ONNX Lane and BSD Detection` (`ShareData`) runs lane-type classification on the device. Camera BSD is separately controlled by `OnnxBsdEnabled` in the detail screen and defaults to off. When enabled, BSD alternates across configured sides whose model-estimated lane width is at least 2 m; speed and lane-change direction do not gate evaluation. The three-class model distinguishes a blind-spot threat from no nearby vehicle and a distant/rear vehicle. When camera BSD is off, its ONNX network is unloaded while lane classification continues; the visual service and camera subscription remain for lane detection and snapshots. Vehicle-provided BSD is unaffected. The detail screen keeps the feature toggle at the top and shows the BSD detection-area editor in a separate card directly below it, with only side selection, image refresh, point undo/reset, and area save in the primary view. The basic rectangular point picker at the image's upper left uses `1(L)` and `1(R)` labels; tapping empty space adds a point whenever none is selected. Zoom, viewport panning, and whole-area dragging are omitted so only the selected point moves. Two equal-width buttons directly below the four editing actions open the road- and wide-camera images in pop-up dialogs. Runtime state, confidence, performance, diagnostics, and six detail settings open from the lower-right **Expand/Collapse advanced settings** text and are collapsed by default. The editor retains one last camera frame and shows a dim default road example when no frame is available. An area can be saved only after receiving a real camera image in the current session. BSD defaults to off, and saved settings survive a service restart. The update includes OpenCV, prepared automatically during normal startup. When `ShareData` is enabled, the service starts after onroad initialization and model/CAN/Panda readiness have settled for 0.5 seconds. Engaging cruise is not required. It stops offroad and waits again at the next onroad start. See [conditions and detailed values](lane-change.md#sharedata--onnx-lane-and-bsd-detection).

Even when lane mode is enabled through `UseLaneLineSpeed`, control temporarily switches to laneless if the model speed trajectory starts below 70% of measured vehicle speed or ends below 70% of its starting speed. Lane mode can resume after approximately one continuous second of acceptable speeds, provided the existing lane and speed conditions also pass.

Laneless driving steers with the model's own curvature, so `PathOffset` (lane-mode path) and `AdjustLaneOffset` (lane mode) do not move the car in laneless. Instead the **minimum-distance clamp** keeps the car from getting too close to a lane line or the road edge. On 2026-11-18 it replaced the old laneless view-shift offsets (`LanelessCenterOffset`, `LanelessEdgeOffset`, `LanelessLineGap`); it is ported from the same feature on the aabc88/openpilot `ej` branch.

- `ClampEnable` (default on): the car's centre never comes closer than `ClampMinDistLane` (cm, 60–200, default 135) to a detected lane line, about 35 cm between the side and the line for a 2 m wide car.
  - A line counts once its confidence is above 0.5 and stops counting only below 0.3.
  - On a side where no line is seen it keeps `ClampMinDistEdge` (cm, 40–180, default 115) from the road edge (curb), used only when the model is sure of the edge (std under 0.5 m).
  - If the lane is narrower than both distances together, it goes to the lane centre.
- In lane mode it moves the path it follows (`PathOffset` included) only where it breaks the bound, and only by as much; otherwise the path is untouched.
- In laneless, when the model's planned path one second ahead (5–30 m) breaks the bound by more than 20 cm, a small curvature that brings it back is added to the model's curvature.
  - It turns off once the violation is under 10 cm.
  - The correction never exceeds 1.5 m/s² of lateral acceleration or a curvature of 0.02, and eases from 0 to full over about half a second.
  - `ClampSteerGain` (×0.1, 0–20, default 10 = 1.0×) sets how strong it is; 0 turns off only the laneless correction (the lane-mode path clamp stays).
- In both lane mode and laneless it rests during a lane change, while a turn signal is on and for 2 s after a lane change. It is off below 5 km/h and fades in up to 10 km/h.
- A lane change or turn signal, switching it off, low speed, or invalid or stale model data stops the laneless correction at once, not gradually. Speeding up never takes the correction past that speed's limit (1.5 m/s² of lateral acceleration).
- `ClampSteerAlert` (default on): when the laneless correction kicks in, a small `클램프 조향 보정 작동` alert shows with a ding; it stays quiet for 5 s after one ends. Off silences the alert only.
- Settings apply within about a second after saving. The control loop (100 Hz) reads the clamp and jerk settings once a second; the lane planner reads them with its other settings twice a second.
- The model's view is no longer shifted, so the drawn lane lines and path and the model outputs are in the original camera frame.

A larger `SteerActuatorDelay` compensates by commanding earlier. A larger `LatSmoothSec` is smoother but may respond more slowly. Changing both together makes diagnosis difficult.

The default `SteerRatioRate` of `100%` applies the learned steering ratio without scaling. It is used when `CustomSR=0`; a stored rate outside the allowed range (`30–200%`) safely falls back to `100%`.

`LateralAccelLimit` (lateral acceleration limit, ×0.1 m/s², `30–45`, default `30`) is the lateral acceleration limit that clips the steering curvature the model requests. The default `30` (3.0 m/s²) is openpilot's stock limit (ISO 11270) and behaves as before.
- Chevrolet with NNFF (the FrogPilot-parity steering path) uses this value at every speed.
- Other cars keep the 4.5 m/s² allowance below 80 km/h and reduce it to this value between 80 and 120 km/h, so for them it mainly changes highway speeds.
- Road roll compensation is still added, and the steering-rate (jerk) limit of 5.0 m/s³ is unchanged.
- A change while driving is read within about a second and applied gradually, 0.5 m/s² per second.
- The car still cannot steer beyond its maximum steering torque and its own steering limits. Cars that reach their torque limit first, such as the Traverse, may show little difference.
- Above 3.0 the car may steer harder than openpilot's stock safety limit; the driver must be ready to take over at any time. A stored value that is outside the range or unreadable is treated as `30`.

`LatJerkMax` (lateral jerk limit, ×0.1 m/s³, `20–100`, default `50`) is the upper limit on how fast the requested steering curvature may change. The default `50` (5.0) matches stock openpilot. Higher turns the wheel faster into a curve but can feel abrupt; lower is smoother but follows later. It is separate from `LateralAccelLimit` and applies within about a second after saving.

Manual steering ratio and learned-ratio scaling also apply to VW MEB, including ID.4. `CustomSR=159` selects a ratio of `15.9` and takes precedence over learned-ratio scaling. `CustomSR=0` with `SteerRatioRate=100%` uses the learned ratio unchanged; the control loop rereads settings while running. Selecting a manual ratio does not stop learning; it selects the value used for control.

Previously ignored ID.4 values now take effect. For example, with `CustomSR=0`, a saved rate of `30%` applies `0.3 times` the learned ratio. Set the rate to `100%` to use the learned value unchanged.

`LateralTorqueCustom` and `CustomSteer*` are advanced settings that can affect the vehicle tune and safety limits. Do not alter them without a vehicle-specific validated baseline and a recovery path.

On GM vehicles `CustomSteerMax` is capped at 300. In Traverse logs the torque the EPS reported (`LKATorqueDelivered`) stayed at about 3.00 while commands were above roughly 300; commands between 300 and 400 only delayed torque release.

The Chevrolet Traverse, Trailblazer, Trax, Bolt No-ACC (CHEVROLET_BOLT_CC) Bolt ACC w Pedal (CHEVROLET_BOLT_ACC_PEDAL), Malibu ASCM harness (CHEVROLET_MALIBU_ASCM), Malibu, Malibu CC, Volt 2019 (CHEVROLET_VOLT_2019) and Volt CC (CHEVROLET_VOLT_CC) send LKA torque commands at about 33 Hz (every 3 frames) with default steps of +10 up and -15 down, matching FrogPilot (the Trailblazer, Trax and Bolt No-ACC use the Traverse values and have not been checked in the car; the Malibu ASCM harness matches the Traverse, which is also StarPilot's GM timing, and has not been checked in the car on this branch; the Malibu, Malibu CC, Volt 2019 and Volt CC borrow the Traverse's Trailblazer NNFF model and so take its steering as well, at the user's request on 2026-11-19, not yet checked in the car), so 0 to 300 takes about 0.9 s. The Volt CC used to have no lateral tuning, so with `NNFF` off its lateral computation stopped with an error once engaged; it now uses torque control with NNFF off too, as the Traverse does. The Bolt No-ACC was moved after a 2026-11-17 Bolt EUV No-ACC log (a 75 km/h curve) where, with the old timing (25 Hz, +5, 2.4 s from 0 to 300), the controller asked for full torque while the torque actually sent rose from 0.05 at only 0.4 per second and the car crossed the outer line of the curve. The Bolt EV 2017-23 No-ACC, which shares the fingerprint, and the Bolt EUV with ACC (CHEVROLET_BOLT_EUV, unchanged) were not checked. A `CustomSteerDeltaUp`/`CustomSteerDeltaDown` above 0 overrides these steps. The values validated on the Traverse are exactly 10/15. On GM a custom value is clipped to the panda safety limits: at most 25 down, and at most 12 up on the 33 Hz cars and 16 up on the 25 Hz cars. The panda limits the change within 250 ms to 128; after the Bolt No-ACC moved to 33 Hz, running the panda safety code showed a rise of 15 breaking that limit (one command of timing slack included). So that a control loop running late and then catching up cannot break the limit either, GM also clips each rise against the commands sent in about the last 0.29 s (and, right after start, the panda's initial reference of 0). At the regular command rate this step never changes the torque, and it never stops unwinding toward zero. With lower values such as 6/8, logs show torque lagging the request in large curves and the wheel winding and unwinding.

With `NNFF` on, the Chevrolets without an NNFF model of their own (Traverse, Malibu, Malibu CC, Malibu ASCM harness (CHEVROLET_MALIBU_ASCM), Volt 2019 (CHEVROLET_VOLT_2019), Volt CC (CHEVROLET_VOLT_CC), Trailblazer CC, Trax) use the Trailblazer NNFF model and the Bolt No-ACC (CHEVROLET_BOLT_CC) and Bolt ACC w Pedal (CHEVROLET_BOLT_ACC_PEDAL) the Bolt EUV NNFF model as a substitute. A Chevrolet running NNFF uses FrogPilot's NNFF lateral computation: the NN low-speed table, the steering-rate jerk input, a jerk window based on the learned lateral delay, no integrator cut at the torque limit, no friction term, the pitch input and a 3.0 m/s² lateral acceleration limit. With a native or substitute model for each, this applies to every Chevrolet. The Bolt EUV and Volt 2017-18 have comma's own NN but, as in FrogPilot, use NNFF first (the Bolt EUV model is FrogPilot's); the Bolt No-ACC (CHEVROLET_BOLT_CC) also has comma's NN but with NNFF on uses NNFF first through the Bolt EUV model. In steady curves of two 2026-11-17 Bolt EUV No-ACC log segments the torque actually needed was 1.50-1.64x comma's NN feedforward and 1.05-1.12x the Bolt EUV NNFF model's (an offline comparison; the effect on the road is not yet checked). Whether the model suits the Bolt EV 2017-23 No-ACC, which shares the fingerprint, is unverified. Whether the Trailblazer model suits the cars other than the Traverse that borrow it (user request 2026-11-19: the Malibu ASCM harness, then every Chevrolet without an NNFF model) is unverified on the road. The Volt 2019 and Volt CC use the Trailblazer model as requested, although the Volt 2017-18 model exists for the same Volt family. With NNFF off these cars are unchanged. The GMC Sierra 1500 2020-21 shares the `CHEVROLET_SILVERADO` fingerprint and is also included when running NNFF. Other non-Chevrolet vehicles are unchanged.

The Trailblazer (CHEVROLET_TRAILBLAZER) differs in one point when running NNFF. Above 1 m/s² of desired lateral acceleration NNFF blends in a stronger "error response"; on the Trailblazer that blend weight is cut to 0.3x up to 55 km/h and returns to normal at 65 km/h. Trailblazer logs from 2026-11-15 showed the wheel winding and unwinding about once a second in every 33-47 km/h curve. The Traverse, running the same controller, did not do this. That the blend more than doubles the feedback in slow, sharp curves is computed; that this combines with the Trailblazer's low-speed steering response into an oscillation is a hypothesis. The `LateralTorqueKpV` and `LateralTorqueKiV` settings are unchanged, and the computation is the same as before below 1 m/s² of desired lateral acceleration (straights, gentle curves), at 65 km/h and above, and on any other car (including the Traverse, the Trailblazer CC and the other cars borrowing the Trailblazer model). An integrator built up in a preceding slow curve does not decay by itself; it stays until later errors move it. The 0.3x scale and the 55/65 km/h points are provisional values from the logs, and the effect has not been checked on the road yet.


The car choices `Chevrolet Bolt EUV 2022-23 - ACC w Pedal` and `Chevrolet Bolt EV 2022-23 - ACC w Pedal` (CHEVROLET_BOLT_ACC_PEDAL) are for a Bolt with the factory camera ACC and a comma pedal. The `BoltPedalLongMode` setting picks the longitudinal mode (reboot after changing it): `1` (default) drives through the car's ACC and uses the pedal only to start from a stop; `2` is StarPilot's mode. The items below describe `1` unless stated.
- Normal acceleration and braking use the Bolt EUV's camera longitudinal control: openpilot sends the ECM the gas/regen request (0x2CB), the friction brake (0x315) and the cluster ACC display (0x370) in place of the stock camera. Regen is requested from the ECM directly, so it should behave the same with One Pedal Driving on or off.
- The longitudinal controller is carrot's, as on the other GM cars, so the Carrot Web longitudinal settings (`LongTuningKpV`, `LongTuningKiV`, `LongTuningKf`, `StoppingAccel`, `LongActuatorDelay`, `CruiseCoastingPercent`, `LongPitch` and so on) apply as they are. Engaging, auto cruise and soft hold work as on the other carrot GM cars with camera longitudinal control.
- Starts and stops follow how StarPilot (`2`) drives this car. The start accel is 0.55 m/s², and when starting behind a lead with a planned accel of 0.25 m/s² or less, the plan is used, but not below 0.15 m/s² (starting already releases the stop hold). While stopping, the target only ramps to -0.25 m/s²; a weaker `StoppingAccel` is still used. Changed after a 2026-11-20 log where the start held gas 673 (half the maximum) for 0.7 s and the car then jumped at 1.4 m/s², and where the brake rose from 25-33 to 40-51 just before the stop (0.6-1.4 km/h). Not yet checked in the car.
- The comma pedal is used only to start from a stop: when the ECM's ACC is in standstill and openpilot wants to go, the pedal is pressed lightly (about 7%, 18/255) to wake the ACC. Carrot's RES button repeats while stopping and starting are kept. While driving no pedal command is sent, so the driver's own accelerator passes through. The pedal is pressed only while the longitudinal controller is starting; once the car rolls it stops even if the ECM still reports standstill.
- While stopped it holds the car like the stock camera ACC, with the full-stop flag (0x2CB FullStop) and the friction brake hold mode (0x315 0xD), and while engaged the ACC active bit stays on through a stop and a resume. The hold is released, and the pedal tapped, only when the longitudinal controller starts.
- This fixes the cruise fault (ACC faulted) in a 2026-11-19 drive log: right after the ECM entered standstill, controlsd's resume flag came on briefly while the controller was still stopping, the existing GM code cleared the ACC active bit, the ECM turned its ACC off, and when the bit came back the ECM faulted 0.5 s later. In the same log the stock camera kept the active bit on and sent FullStop with 0xD.
- Below 5 km/h, come to a full stop, hold the brake firmly (value 20 or more), press SET, and release the brake only after 'Release Brake to Engage' appears (the stock ACC's rule). Without the brake, or while still rolling below 5 km/h, only a 'speed too low' notice appears and it does not engage, because the Bolt's ECM treats such an activation as a fault. A 2026-11-19 log showed two cruise faults 0.5 s after engaging without the brake while held in one-pedal mode behind a lead. Coming to a stop while engaged is fine (seen in the same log).
- Carrot clears this standstill state on the other GM cars; on this car (with a pedal) it is kept, and the longitudinal controller does not let it hold off starting.
- No stock-ACC cancel button is sent even with a pedal. With a pedal on the Bolt EUV, the existing code marks the car as pedal-only longitudinal control and keeps cancelling the ECM's ACC; this car drives through that ACC, so it is left out.
- Accelerator press detection uses the panda's rule (the raw average of the two pedal tracks above 550).
- Steering is the same as the Bolt No-ACC (33 Hz torque commands; with NNFF on, the Bolt EUV model).
- The panda firmware does not change. Camera longitudinal control is allowed only on a panda built on the device (debug build). Without a pedal the car runs the Bolt EUV's camera path.
- `2` (StarPilot) ports StarPilot's same car (CHEVROLET_BOLT_ACC_2022_2023_PEDAL, commit 2589caad of 2026-11-14): all acceleration through the pedal, braking through the regen paddle and gear position (PRNDL2) signals that the panda sends right after each stock frame plus the friction brake (up to 400), and the stock ACC cancelled toward the camera. Its longitudinal controller is StarPilot's own for this car (acceleration-error PID, feedforward 0.2, 0.6 s actuator delay, pitch compensation always on), so the Carrot Web longitudinal settings do not apply. It needs One Pedal Driving on to brake properly, and since its panda requires the driver's SET/RES it engages only from those buttons (auto cruise, soft hold re-engage and remote engage wait for the button). Accelerator press detection is the raw average above 595. It ran this way from 2026-11-17 to 18 and can be chosen again with the setting since 2026-11-19.
- Not checked in a real car yet (2026-11-18), in particular whether camera longitudinal control is smooth on this car and whether the pedal tap restarts the ACC from a stop (with One Pedal Driving on and off).

On the Chevrolet camera ACC and SDGM cars under openpilot longitudinal control (Traverse, Bolt EUV, Bolt ACC w Pedal in mode 1, Trailblazer, Trax, Equinox, Silverado and so on), gas (0x2CB) and braking (maximum regen plus the friction brake) switch at an acceleration of exactly 0. While holding a speed, a command moving around 0 therefore alternated gas and brake (a 2026-11-20 Bolt ACC w Pedal log: 53 times a minute on a roughly 4-degree downhill). The gas side now only lifts off and coasts down to -0.2 m/s², and the brake side keeps regen only up to +0.1 m/s². After a switch the side is kept for about 0.5 s, except that braking stronger than -0.4 m/s² or acceleration stronger than +0.3 m/s² switches at once. It is not used while stopping or while longitudinal control is off; EVs with `EVTable` on (which already have a regen band) and the other GM cars are unchanged. Not yet checked in the car; in a model fitted to the logs the switching dropped sharply on flat roads and 3-4x on the Bolt downhill. On the Bolt the gas side (about -0.26 m/s² at zero gas) and the brake side (about -0.88 m/s² with regen only) are far apart, so on a long downhill needing a deceleration in between, the switching is reduced but not removed.

The car choice `Chevrolet Malibu ASCM Harness 2017-19` (CHEVROLET_MALIBU_ASCM) is for a Malibu whose harness sits in the ASCM (adaptive cruise module) connector, putting the comma between the ASCM and the car. Cars with a SASCM added pick it too. It ports StarPilot's same car (CHEVROLET_MALIBU_ASCM, StarPilotHQ/StarPilot stable 8589b73f).
- It is not detected automatically. Its CAN is the same as `Chevrolet Malibu Premier 2017`, so choose it in the CarrotPilot web car list and reboot. The panda safety code changed, so the panda firmware is reflashed on the reboot after the update.
- The ASCM stays alive and keeps the stock ACC (speed control and radar); openpilot only steers. The panda forwards both sides like a camera harness, blocking only the ASCM's steering command (0x180), which openpilot replaces. openpilot engages with the stock ACC (from 5 km/h) and steers from about 11 km/h (7 mph).
- openpilot longitudinal control (StarPilot's optional speed control through a SASCM) is not ported yet. With longitudinal control turned on in the web settings this car still runs the stock ACC.
- The brake is read as on the ASCM path: the brake value in the accelerator position message (0xBE) at 10 or more, the same threshold the panda uses. A car without 0xBE uses the brake switch in the engine status message (0xC9) as StarPilot does, and the panda then checks the brake pedal position message (0xF1) instead of 0xBE.
- A pedal message (0x201) on the bus does not switch it to pedal longitudinal control. The panda lets this car send only the steering command, PSCMStatus toward the ASCM and the cancel button toward the ASCM.
- Steering is the Traverse's: about 33 Hz with steps of +10 up and -15 down, and `CustomSteerMax`, `CustomSteerDeltaUp` and `CustomSteerDeltaDown` apply within the same ranges as on the Traverse. The panda's steering checks are also those of the other GM cars (StarPilot's panda blocks steering limit violations; this branch's GM panda does not). The default steering torque values are the existing Malibu's (the Volt 2017-18 values as a substitute). With `NNFF` on it uses the Trailblazer NNFF model as a substitute, as the Traverse does, along with the same FrogPilot NNFF lateral computation as the Traverse. Whether that model suits the Malibu's steering has not been checked on the road.
- The radar is read only when the whole stock radar stream is on bus 1.
- Speeds on the comma follow the cluster's numbers, as on the Traverse (2026-11-20, all three Malibu choices). The current speed is the wheel speed times 1.046: on a 2026-11-19 Malibu ASCM log the ECM vehicle speed (0x3E9) read 4.5-4.8% above the wheel speed across 15-70 km/h. A Malibu with the cluster speed message (0x260) keeps using it. The set speed is the stock ACC set speed (0x370) through the same ratio, and the Malibu ASCM always shows the stock set speed whatever `SpeedFromPCM` is: its panda lets openpilot send the stock cruise only the cancel button, so a separate Carrot set speed could never change the stock one. On a Malibu with openpilot longitudinal control, a set speed taken in cluster numbers goes back to the real target speed through the same ratio (as on the Trailblazer and Trax), and the set speed sent to the cluster is scaled to match. The wheel speed used for speed and acceleration control does not change. That the cluster shows the set speed through the same ratio as the current speed follows the Traverse measurement; it has not been checked against a Malibu cluster.
- `Chevrolet Malibu Premier 2017` (the ASCM removed and replaced by the comma) and `Chevrolet Malibu No ACC` are unchanged. Choosing either on a car that keeps its ASCM makes the panda cut the ASCM off the car or leaves the stock ACC unusable, which shows up as a CAN error or no engagement.
- Not checked in a real car yet (2026-11-19); only tested on a Mac with the real vehicle and panda code.

#### NNFF torque control

`NNFF` is a neural-network feedforward option that defaults to off. Enabling selects torque-controller tuning on vehicles without angle control and uses NNFF when a matching model is available. Vehicles supported by comma's own NNFF retain that path's precedence, except the Chevrolet Bolt EUV and Volt 2017-18, where NNFF comes first as in FrogPilot, and the Bolt No-ACC, where NNFF comes first with the Bolt EUV model; angle-control vehicles are unaffected.

Disabling turns off this NNFF option without changing the separate `NNFFLite` value. Changes apply at the next onroad start or after a device reboot, rather than immediately during a drive. An absent startup notice alone does not establish whether NNFF is active.

#### Steering recovery after driver intervention

Hyundai/Kia/Genesis angle-control vehicles use the former combined mode 3 as standard. The mode selector has been removed, and previously saved mode values are ignored. Torque-control vehicles are unaffected.

- Steering-error and driver-force levels and trends permit limited recovery as they converge. Unclear convergence pauses the increase or gently reduces it.
- A rapid force decline after sustained intervention takes priority. Once low force is confirmed, smaller error permits faster torque-ceiling recovery and larger error slows it. The two recovery increases are never added together.
- Strong renewed intervention yields quickly. Target steering angles and existing angle limits remain unchanged.

Force-sensor trends do not establish loss of hand contact or driver consent to handover. Existing `steeringPressed` and driver monitoring remain unchanged. Making this behavior standard does not establish the same steering feel on every vehicle.

### Speed and deceleration — 23 settings

| Section | Parameters | Purpose |
|---|---|---|
| [Speed cameras](speed-deceleration.md#speed-camera) | `AutoNaviSpeedCtrlMode`, `AutoNaviSpeedCtrlEnd`, `AutoNaviRearCameraHoldDistance`, `AutoNaviSpeedDecelRate`, `AutoNaviSpeedSafetyFactor`, `AutoNaviCountDownMode`, `VehicleNaviCanControl`, `VehicleNaviSchoolZoneControl`, `VehicleNaviDecelCancel`(`VehicleNaviDecelCancelBox`, `VehicleNaviDecelCancelMobileZone`, `VehicleNaviDecelCancelBump`, `VehicleNaviDecelCancelUnknown`, `VehicleNaviDecelCancelEarlyWarning`), `VehicleNaviSectionAvgControl`, `VehicleSpeedCameraControlMode`, `VehicleSpeedCameraDistanceTime` | Event types, rear-camera post-pass hold, stock camera distance matching and virtual distance, PV5 warning-confirmed camera retention/cancellation and section speed caps, deceleration start, and target speed |
| [Road speed limit](speed-deceleration.md#road-speed-limit) | `AutoRoadSpeedLimitOffset`, `AutoRoadSpeedAdjust`, `AutoSpeedUptoRoadSpeedLimit` | Desired-speed adjustment from the road limit |
| [Speed bumps](speed-deceleration.md#speed-bump) | `AutoNaviSpeedBumpTime`, `AutoNaviSpeedBumpSpeed`, `AutoNaviSpeedBumpEndDistance` | Completion time, crossing speed, and early-release distance |
| [Curves and turns](speed-deceleration.md#curve-turn) | `AutoCurveSpeedFactor`, `AutoCurveSpeedLowerLimit`, `TurnSpeedControlMode`, `MapTurnSpeedFactor`, `ApplyModelSpeed` | Curve slowing from curvature and distance, prompt recovery after confirmed easing, and route-turn speed |
| [Traffic lights](speed-deceleration.md#traffic-light) | `TrafficLightDetectMode`, `TrafficStopDistanceAdjust` | Stop/go detection, stop-position adjustment, and automatic stopped-vehicle alignment |

`AutoNaviSpeedCtrlMode` is `0` off, `1` fixed speed cameras, `2` cameras plus speed bumps, or `3` those events plus mobile-camera events.

While external navigation is connected, deceleration, countdowns, and navigation speed displays use it exclusively. Stock navigation remains excluded even without an external guidance item and resumes according to its settings after disconnection or receive timeout is detected.

`VehicleSpeedCameraControlMode=2` treats a new accelerator press after vehicle-received camera deceleration has actually begun as a request to ignore the current event. It keeps the highest speed reached while accelerating as the floor until the event ends; an accelerator held from before deceleration began does not start the override.

`AutoNaviSpeedDecelRate` defaults to `120` (1.20 m/s²); a lower value begins slowing farther away. Updating the software does not change an existing saved value. `AutoNaviSpeedSafetyFactor` applies a percentage of the event limit as the target. Before tuning either value, confirm that the event type, limit, and remaining distance are being received correctly.

`TrafficLightDetectMode` is `0` off, `1` stop detection, or `2` stop and go detection. This is model-based assistance; the driver must always verify the signal.

### Cruise and following gap — 29 settings overall, 26 on Hyundai/Kia/Genesis

| Section | Parameters | Purpose |
|---|---|---|
| [Driving mode](cruise-gap.md#driving-mode) | `MyDrivingMode`, `MyDrivingModeAuto` | Eco, safe, normal, high-speed modes and automatic selection |
| [Speed-based acceleration](cruise-gap.md#acceleration-table) | `CruiseMaxVals0` through `CruiseMaxVals6` | Maximum acceleration tendency by speed band |
| [Stopping and restarting](cruise-gap.md#stop-resume) | `StopDistanceCarrot`, `StoppingAccel`, `VEgoStopping`, `AChangeCostStarting` | Stop position, stop entry, and restart behavior |
| [Longitudinal tuning](cruise-gap.md#longitudinal-tuning) | `LongTuningKpV`, `LongTuningKiV`, `LongTuningKf`, `LongActuatorDelay` | Hyundai/Kia/Genesis fix/hide `100/0/100`; Toyota/Lexus fix/hide `0/0/100`; VW MEB, including ID.4, applies saved gains |
| [Following gap](cruise-gap.md#following-gap) | `TFollowGap1` through `TFollowGap4`, `DynamicTFollowLC`, `SpeedTFFactor`, `TFollowDecelBoost` | Gap times, lane-change relief using selected leads, and deceleration margin (default 0%) |
| [Following responsiveness](cruise-gap.md#lead-response) | `LeadAccelResponse`, `LeadAccelResponseTF1`–`LeadAccelResponseTF4` | Lead-start, acceleration and approach response at every following-distance level |
| [Carrot cruise](cruise-gap.md#carrot-cruise) | `CruiseEcoControl`, `CruiseCoastingPercent`, `CarrotCruiseDecel`, `CarrotCruiseAtcDecel` | Economy control, coasting margin with a fixed entry reference (default 0%: existing control), and cruise deceleration limits |

`MyDrivingMode` is `1` eco, `2` safe, `3` normal, or `4` high speed. High-speed mode ignores traffic-light control and increases acceleration tendency, so read its behavior before selecting it.

Eco caps lead response at 2 and Safe at 3; Normal and High retain the selected value. Caps follow common/gap-specific selection and never raise lower choices or 0. Eco ×1.1 and Safe ×1.2 TF multipliers remain, with gradual release of mode allowance. Automatic selection uses Safe for stopping approaches and sustained slow following. Outside a stopping approach, lead acceleration above 1.0 m/s² for about 0.5 seconds restores Normal/Eco; otherwise, it requires three seconds of flow recovery or four seconds with no lead at 15 km/h or above.

`CruiseGapLevels` (Gap cycle levels) limits button cycling to 2 through the vehicle-supported maximum, which is the default. 2 uses TF1 and TF2; 3 uses TF1 through TF3. It applies on the next gap-button press and preserves unused TF and following responsiveness values. Applies with openpilot longitudinal control.

`TFollowGap1` through `TFollowGap4` are stored in hundredths of a second. Lower values reduce the time gap. Use `LeadAccelResponse` for acceleration response: levels 1–3 are gradual, 4 is quick, and 5 retains maximum response. Braking-onset distance is not separately remembered to enlarge the gap.

`LeadAccelResponse`: Adjusts how the car follows a lead vehicle as it starts or speeds up. Lower levels close the gap more gradually; higher levels follow more quickly. Level 0 turns off the acceleration boost, and level 5 is the most responsive test setting. See [Following responsiveness](cruise-gap.md#lead-response) for details. Extra-headroom hold conditions apply up to 1.2 times TF distance (speed × TF + stopping distance), ease progressively from 1.2 to 1.5 times, and no longer pause recovery at or above 1.5 times regardless of lead speed.

`SpeedTFFactor` applies a linear speed multiplier to the selected base TF: 10 is unchanged; 20 doubles it at 100 km/h. `LeadAccelResponseTF1`–`TF4` use the common response at -1 and a gap-specific response at 0–5. Levels 4–5 retain speed TF. The driving-screen bar shows the dynamically adjusted following target in metres.


Deceleration preview operates independently of the response level. During active control, remaining correction releases progressively when relative acceleration eases or the lead switches between radar and vision or disappears. Accelerator or brake intervention and longitudinal control exit clear it immediately.

`LongTuning*` and `LongActuatorDelay` are advanced settings that directly affect vehicles using openpilot longitudinal control. Hyundai, Kia, and Genesis fix `LongTuningKpV`, `LongTuningKiV`, and `LongTuningKf` at the safe `100/0/100` values and hide them from settings. Toyota/Lexus fix and hide common gains at `0/0/100` to avoid duplicate correction before their dedicated controller. Some parameters have no effect when stock ACC remains responsible for acceleration and braking.

`StoppingAccel` is adjustable: default `-50`, range `-100 to -1`, step `1` (the weakest value moved from -0.50 to -0.01 m/s² on 2026-11-20). The stored value is multiplied by 0.01, with the same bounds enforced by control. Original stop-entry and braking behavior and vehicle-specific soft hold are restored; changes apply within about one second. See [Stopping and restarting](cruise-gap.md#stop-resume).

`VEgoStopping` has a range of `10–100` and a default of `50`; `10` means `0.10 m/s`. Previously stored values below `10` are automatically raised to `10`, and control enforces the same minimum during driving.

For Hyundai/Kia CANFD openpilot longitudinal control, aReqRaw and aReqValue become zero in the first StopReq frame, so this setting does not change transmitted acceleration while StopReq remains active. Automatic release/retry is removed.

On supported Tesla vehicles with the additional vehicle bus detected, the device's **alpha longitudinal** (`AlphaLongitudinalEnabled`) toggle also enables [automatic cruise set-speed adjustment](tesla.md#automatic-cruise-speed) to the vehicle-reported limit. Navigation/Carrot speed control takes precedence and pauses DAS-based adjustment until it ends and the limit is stable again; any manual pause remains in effect. Turning the right speed wheel pauses it; an opposite-direction wheel gesture within one second or disengaging and re-engaging resumes it. There is no separate Carrot Web setting for this feature.

<a id="vehicle-hardware"></a>
## Vehicle and hardware

These settings describe the car, harness, and device hardware configuration. Do not enable them merely as a display experiment.

| Group | Parameters | Purpose |
|---|---|---|
| Hyundai/Kia | `HyundaiCameraSCC`, `IsLdwsCar`, `HapticFeedbackWhenSpeedCamera` | SCC connection, LDWS behavior, and speed-event haptics |
| CAN FD/HDA | `CanfdHDA2`, `HyundaiCanfdClusterDirectTx`, `CanfdDebug`, `HDPuse` | HDA2 selection, cluster direct send, CAN FD diagnostics, and HDP |
| Radar | `EnableRadarTracks`, `RadarTrackFlip`, `EnableCornerRadar`, `CarrotRadarMode`, `CarrotRadarCutInSensitivity` | SCC radar, front-track orientation, corner radar, and Carrot Radar processing and cut-in sensitivity |
| Driver monitoring | `DriverMonitoringEnabled` (search only), `DriverMonitoringMode`, `GmClusterTakeSteering`, `CarrotVisionEnabled`, `MuteDoor`, `MuteSeatbelt` | Driver monitoring, the GM cluster steering alert and selected vehicle alerts |
| Vehicle assistance | `MaxAngleFrames`, `SpeedFromPCM`, `AutoStopOff`, `BoltPedalLongMode` | Steering-angle frames, stock-SCC speed control, Traverse auto stop off, and the Bolt ACC w Pedal longitudinal mode |
| Device hardware | `HardwareC3xLite` | Speakerless C3X Lite audio and process configuration |

> [!CAUTION]
> Incorrect `HyundaiCameraSCC`, `CanfdHDA2`, `EnableRadarTracks`, `CarrotRadarMode`, `CarrotRadarCutInSensitivity`, or `SpeedFromPCM` values can change vehicle identification, SCC, radar, or longitudinal behavior. Confirm the vehicle, model year, HDA generation, harness location, and whether stock ACC is retained.

`DriverMonitoringEnabled` defaults to ON and is available only through Carrot Web setting search. Search for the parameter name to find it. The OFF option is intended for an absent or failed DM camera; leaving the default ON unchanged is recommended. Turning it off stops normal-driving driver-monitoring alerts, monitoring-triggered force deceleration, and lockout, and may violate applicable laws or driving requirements depending on where and how the vehicle is used. Driver View keeps face preview with neutral enforcement. A Web OFF persists across drives and restarts until you manually turn the setting back ON. File and QR backups, restores and setting profiles exclude this value, so a backup from another device cannot turn monitoring off.

Regardless of gear or speed, including at standstill, three distinct physical CANCEL presses, each separated by a release, within three seconds turn monitoring off for the current ignition session without changing `DriverMonitoringEnabled`. Gear and speed changes do not reset the count. Automatic-control CANCEL echoes and BT CANCEL do not count. Any received non-CANCEL vehicle-button event, whether a press or release, resets the sequence. Invalid or stale state, an input-stream gap, or timeout also resets it; an indistinguishable stock-ACC speed-button echo may reset progress too. The next ignition-on or manager/device restart clears only the temporary off state, so monitoring resumes only if the saved setting is ON. `DriverMonitoringMode` and `CarrotVisionEnabled` remain unchanged.

`DriverMonitoringMode` applies while driver monitoring is on. It defaults to 0: stock comma camera monitoring criteria, or 15/30/45-second interaction monitoring when the camera is absent or failed. Mode 1 is for controlled experiments, with empty-road timing extensions and an interaction grace before camera warnings. A shared exception resets the usage restriction after one continuous second of valid Park, standstill and disengaged status. Mode changes apply live at roughly half-second intervals without rebooting. Switching preserves accumulated monitoring time, warning counts and lockout, and ends the previous interaction grace and forward-attention streak. See [driver monitoring and experimental-use conditions](driver-monitoring.md). `CarrotVisionEnabled` controls web road video independently. `DisableDM` remains migration-only; only its old value 2 video function is migrated once to `CarrotVisionEnabled`. Validated original `STEER_TOUCH_2AF` input on Hyundai/Kia/Genesis CAN-FD is supported without a vehicle-name whitelist, with held-contact and new-contact behavior depending on camera availability and mode.

**GM cluster Take Steering alert (experimental)** (`GmClusterTakeSteering`) works only on a Traverse with the SDGM camera harness and openpilot longitudinal control. On other cars or with stock longitudinal control the saved value changes nothing. It defaults to `0` (off). openpilot normally tells the stock camera that your hands are on the wheel, which hides the stock "Take Steering" alert. `1` reports hands off from the audible driver-monitoring alert (level 2 and above), and `2` from the first monitoring alert (level 1), only while that alert is showing, so the stock camera can put its steering alert on the instrument cluster. Steering commands, driver-monitoring decisions and the real steering torque signal do not change. GM's owner's manual says Lane Keep Assist may become temporarily unavailable after repeated Take Steering alerts. The camera's delay, its chime, whether stock Lane Keep Assist must be on, any other cluster messages and recovery after repeated alerts have not been verified in the car. It applies within about 0.5 seconds without a reboot.

See [Radar tracks and corner radar](radar.md) before changing radar modes.

**Cluster CAN Direct Send** (`HyundaiCanfdClusterDirectTx`) compares the original direct-send method on Hyundai/Kia/Genesis CAN-FD CameraSCC vehicles with intermittent cluster warnings. It defaults to OFF for every vehicle; enable it manually on an affected vehicle. Reboot the device after changing it. Updated Panda firmware supporting this setting is required. Resolution of vehicle warnings has not been validated.

With `HyundaiCameraSCC=0` and no camera-SCC configuration already applied to the vehicle, receiving `SCC_CONTROL` (CAN-FD) or `SCC12` (classic CAN) on the camera bus during the current onroad session adds **Enable CameraSCC** to the CAN error alert. This is bus2, or the corresponding camera bus with multiple Pandas. The hint uses reception history and does not establish the cause of every CAN error. It never changes the setting automatically; existing disengagement and engagement blocking remain active. Select the mode appropriate for the vehicle and wiring while stopped, then verify it in the next onroad session.

`SpeedFromPCM` defaults to `2` (curve/camera deceleration) and affects PCM speed selection and button spamming. With mode `3`, Toyota/Lexus use stock set speed without changing longitudinal-control mode. For `TOYOTA_RAV4_TSS2_2022` using PCM with direct longitudinal control, mode `0` also applies valid stock set-speed changes and cruise resume while preserving lower deceleration targets. See [button transmission details](buttons-presets.md#button-spam).

`AutoStopOff` defaults to `0` (off). With `1`, engine auto stop is turned off on every ignition. It is an experimental feature that works only on the Traverse with openpilot longitudinal control.
- 5 seconds after start, while stopped with cruise and openpilot off, it sends the auto-stop-off switch press (`0x1E1`), then checks that the auto-stop-off state (`0x1F4`) becomes 1.
- It does not press, and stops an attempt in progress, while a steering-wheel button is held, the car moves, CAN is unhealthy, or another button frame was just sent.
- It never presses when auto stop is already off. It tries at most 3 times per drive and stops after a success, so auto stop that the driver turns back on stays on.
- The press is added between the real button frames, so whether the vehicle actually turns auto stop off, and whether it shows a warning, is not verified yet. Toggle changes apply within about 0.1 second.

`BoltPedalLongMode` defaults to `1`. It is used only when the car is `ACC w Pedal` with a comma pedal and changes nothing on other cars. `1` is the hybrid mode (the car's stock ACC commands and carrot's longitudinal control drive and brake; the pedal only starts from a stop); `2` is StarPilot's mode (all acceleration through the pedal, braking through the regen paddle signal and friction brake, StarPilot's controller, One Pedal Driving needed, SET/RES-only engage). It is read when the car settings start, so reboot after changing it. See the `ACC w Pedal` paragraph under vehicle steering.

For dPath RadarD, `EnableRadarTracks=-2` is the vision-only experiment; `-1` always uses SCC without vision matching; `0` also always uses SCC; `1` matches front radar without SCC; `2` matches front radar plus low-speed SCC; and `3` uses SCC unconditionally after front-radar/vision matching fails. Matching modes use central vision at probability `0.40` or higher when matching fails. Modes `-1`, `0`, and `3` use vision only when SCC is absent. All SCC association ignores lateral position. SCC-only modes `-1` and `0` apply no additional dPath gate to vision fallback. Legacy Mando radar variants with 32 or 64 slots are handled automatically. A new stationary front lead requires vision or a matching corner detection; continuous front-radar observation alone cannot authorize it. When a separate measured moving target agrees with the visual position and speed, that vision cannot authorize or retain a different stationary reflection. Corner corroboration must match the selected stationary object itself. For a front candidate without corresponding corner corroboration, a vision-support interruption beyond the permitted brief hold resets both the pending object and its confirmation time before confirmation starts again. An already selected moving front can remain L1 within a bounded vision-uncertainty range while the same measured track stays physically continuous; a fixed 8 m difference alone no longer discards it. A new nearer match can replace it immediately, and gaps or physical jumps reset this allowance. A distant stopped front can qualify on a gentle curve through continuous measured radar history and repeated visual position agreement. Continuous corner position/speed evidence participates in that decision. Confirmed stationary fronts can bridge visual-range noise within distance and time limits while publishing radar distance and speed.

`RadarTrackFlip` offers `0: Normal` (default) and `1: Invert left/right`. It inverts both lateral position and lateral velocity of front radar tracks for lead selection and display, preserving SCC, corner radar and vision. There is no automatic inversion by vehicle model; use it only after confirming a left/right mismatch. Restart the vehicle or reboot the device to apply it at the next OnRoad start. See [radar left/right inversion](radar.md#radar-track-flip).

In versions supporting Bosch MRRevo14F reception, detecting the Bosch message group on a Bosch MRRevo14F candidate platform (vehicle-validated only on the Elantra AD) with `EnableRadarTracks=1`–`3` enables passive reception of tracks already being transmitted, skipping startup radar-activation UDS writes and SCC ECU communication disabling. `EnableRadarTracksResult` is not updated; check actual reception, CAN validity and radar errors. Passive reception does not mean all vehicle control is disabled. See the [Bosch passive reception exception](radar.md#bosch-passive-reception).

In modes `EnableRadarTracks=1`–`3`, a confirmed departing front lead can receive bounded future headway relief in ACC when outward radar motion and a visual switch to a farther vehicle agree. See [predicted CUT-OUT behavior](radar.md).

`CarrotRadarMode` continuously tracks vehicles with the front and corner radars to detect cut-ins, then matches camera and radar information in a new way to select the vehicle ahead. On vehicles with neither corner-radar nor radar-track support, it behaves the same as the existing mode. It can change acceleration and braking, so enable it only on the same vehicle after completing validation. The value is latched when the next OnRoad session starts, so end the current drive and restart the vehicle or reboot the device after changing it. The previous `RadarMotionMode` value is migrated to the new name once on the first startup after updating.

`CarrotRadarCutInSensitivity` controls only Carrot Radar Mode CUT-IN detection: `0` disables it, `1` is insensitive, `3` is normal (default), and `5` is very sensitive; `2` and `4` are the intermediate levels. Levels `1` through `5` require `0.50`, `0.40`, `0.35`, `0.25`, and `0.20 s` of continuing measured motion evidence, while the physical future prediction remains fixed at 5.0 seconds. A front-radar track with at least 0.50 m of strongly one-way progress in its recent measured history may receive at most one 20 Hz radar-frame credit so timestamp quantization does not discard a completed dwell; small adjacent drift does not. It does not affect conventional radar mode or `EnableCornerRadar`. The value is read at the next OnRoad start, so restart the vehicle or reboot the device after changing it.

`HardwareC3xLite` must remain off on standard C3 and C3X hardware. Enable it only on a C3X Lite, then reboot the device. The setting skips the unavailable amplifier so startup is not delayed by I2C retries, uses the GPIO buzzer for alerts, disables `micd`, `soundd`, and `loggerd`, and turns off `RecordAudio`. Normal route logging is unavailable while this hardware mode is enabled.

<a id="display"></a>
## Display

Display contains 34 settings. External-HUD settings control the layout and output of separate display hardware.

`ClusterHud` (External HUD Display) switches a **direct USB HUD or a registered ESP32-S3 Wi-Fi HUD** on or off. A HUD connected to Jetson starts automatically even when this value is `0`, and normally turns its display off with ignition. Use the existing always-on `ClusterHudDebug` modes to keep it visible offroad. Brightness and layout settings also apply to the Jetson HUD.

S3 Wi-Fi output requires a TURZX 9.2-inch display, a powered hub, and C4/S3 on the same 2.4 GHz network. Follow the [initial registration instructions](../../../tools/esp32s3_display/README.md) to register once over USB or import the registration file from the phone setup page. A registered S3 reconnects after reboot or an IP change. This path defaults to JPEG quality 50 and a 15 FPS target regardless of `ClusterHudEncoder`, allowing up to 10 FPS while the eGPU is active (direct USB keeps its 5 FPS cap). Actual speed depends on image content and wireless conditions; C4 vehicle measurements remain separate. Remove the registration file and restart HUD to restore direct USB output.

The [NAS installer and setup guide](https://upload.shind0.synology.me/models/carrot-s3/index.html) provides the N16R8 board checklist, wiring, browser firmware installation, phone/C4 registration and firmware ZIP. S3 1.1.0 confirms and retries shutdown; updating preserves existing registration.


The Jetson HUD shows the highest internal sensor temperature below `jetSON`, or in the upper-right corner in full-screen navigation/graph modes. If temperature data has not refreshed for three seconds, it shows `--°C`. A top-strip temperature warning starts 5°C below each sensor's configured thermal limit; reaching that limit shows an overheating warning. For example, a device configured to throttle at 99°C warns from 94°C. Check the fan and ventilation when warned. Vehicle driving alerts take precedence, and Jetson temperature remains separate from device temperature/memory statistics. This feature requires updated Jetson HUD software.

With `CarrotVisionEnabled` on, the external HUD and web camera view can be used together. Simultaneous video use may increase device load. See the [Carrot Web guide](carrot-web.md).

`NavdyAmbientDarkOff` (Navdy ambient off on unlit dark roads, default 0) turns the ambient light connected to the Navdy off while driving on dark roads without street or city lights. Darkness comes from the car's auto-headlamp light sensor and the lights check from the auto high beam camera; both are currently read only on the Chevrolet Traverse. It also turns the light off only after sunset: the sun must be more than 4° below the horizon, about 20 minutes after sunset (computed for Seoul, so a few minutes off elsewhere in Korea). An overcast or rainy afternoon that reads dark to the light sensor never turns it off. A light that is off returns to normal lighting once the sun rises above 3° below the horizon at dawn. Until the comma's clock is set, this check is skipped and the rules below apply alone. The light turns off after 5 seconds of darkness with 10 seconds of no lights from the camera, and returns to its normal lighting after 15 seconds of city lights, 10 seconds of brightness, or 15 seconds without a camera reading. The camera does not judge lights in daylight and always reports no lights then, so that reading only counts after the camera has reported lights or a high beam advice once since it got dark. Daytime tunnels therefore never turn it off, and at night it stays on until the camera has judged once (for example right after starting). More than a minute of brightness or turning the car off clears that record. Tunnels are an exception: while the front radar reports a tunnel it never turns the light off, even when the camera reports no lights, and a light that was already off returns to normal lighting after 2 seconds in the tunnel (some lit tunnels read as no lights to the camera). On a dark road after the tunnel it turns off again after the usual 10 seconds. Without a radar reading the camera rules above apply unchanged. Shifting to Park or opening a door returns it to normal lighting right away, so exit and parked lighting work as before once the car is off. The overspeed warning blink comes from the Navdy app and can still light it while it is off. Changing the Wayon lighting profile while it is off passes the new profile through once and then turns the light off again. If the Navdy link is down from shifting to Park until the car is off, that exit lighting can be skipped. A manual ambient light set from the Wayon app takes priority. The Navdy app is unchanged; the comma uses its existing manual lighting command, so the light may stay on if the Navdy clock is far off. Applies within about 2 seconds without rebooting.

| Group | Parameters | Purpose |
|---|---|---|
| Information | `ShowDebugUI`, `ShowTpms`, `ShowDateTime`, `ShowPathEnd`, `ShowDeviceState`, `ShowLaneInfo`, `ShowRadarInfo`, `ShowRouteInfo`, `ShowPlotMode` | Debug, tire, time, lane, radar, and route information |
| Path | `ShowPathMode`, `ShowPathColor`, `ShowPathColorCruiseOff`, `ShowPathModeLane`, `ShowPathColorLane` | Path shape and color by driving state |
| Brightness/on-road view | `ShowCustomBrightness`, `ShowModelView`, `ShowCameraWithCluster` | Brightness, camera/model composition, and the on-device camera while the external HUD is connected |
| External HUD | `ClusterHud`, `ClusterHudBrightness`, `ClusterHudOrientation`, and related `ClusterHud*` settings, `NavdyAmbientDarkOff` | Supported TURZX HUD layout, live brightness, screen rotation, camera, radar, and encoder options; Navdy ambient off on unlit dark roads |

For RAV4 2022 with the ARS display stream, `ShowRadarInfo=3` on the standard device
driving screen also shows adjacent-lane objects separately from control leads. Modes 1/2
and other receivers retain their behavior. A marker does not mean braking-target selection
or Toyota corner-radar support. See the [display conditions](radar.md).

USB external HUD output is fixed at **10 FPS**; while the eGPU is active, rendering, encoding, and USB output switch to **5 FPS**. Changes to eGPU activity apply automatically; changing the H.264 encoder rate restarts the HUD. `ClusterNaviMapFps` remains a separate setting for Android map input.

Onroad, the main UI uses CPU 6 and the external HUD uses CPU 7 with low-priority normal scheduling (nice 19). Offroad, both return to CPUs 0–3 so CPUs 4–7 can enter power save; always-on debug output follows the same rule. FPS selection, CPU selection, and realtime priority settings have been removed, and previously stored values are ignored. Actual refresh rates may fall below the output cap under load.

`ShowPlotMode` selects an on-road diagnostic graph; `0` turns it off. Modes `4` and `5` both use the primary lead vehicle (`radarState.leadOne`), and the mici device display also updates these graphs when the lead message values change.

| Color | `4`: Lead acceleration/relative speed | `5`: Lead acceleration/jerk |
|---|---|---|
| Yellow | First planned ego acceleration `longitudinalPlan.accels[0]` (m/s²) | Actual ego acceleration `carState.aEgo` (m/s²) |
| Green | Estimated lead acceleration `leadOne.aLeadK` (m/s²) | Lead acceleration `leadOne.aLead` (m/s²) |
| Orange | Relative lead speed `leadOne.vRel` (m/s) | Rate of change of lead acceleration `leadOne.jLead` (m/s³) |

Negative relative speed means the gap is closing; positive relative speed means it is opening. Lead values display as zero when no lead is detected, and this setting changes only the diagnostic display, with no effect on vehicle control.

An APN label remaining in the `ShowRouteInfo` description refers to route-input state. It is not an indication that CarrotMan or CarrotLink is supported.

`ShowCameraWithCluster=0` keeps the existing default: while the external HUD is connected, the on-device camera is hidden. Set it to `1` to show the on-device camera video.

`ShowModelView` (Driving Screen Layout) selects the C4 view: `0` camera + lanes/path, `1` camera only, `2` lanes/path on a black background, or `3` hide both. Lane/path graphics include lead-vehicle markers. It applies at every brightness setting, including automatic and 100%, with no additional wait after entering driving mode. Changes while driving appear within about five seconds without rebooting. Speed, alerts, and driver status stay visible; camera capture, driving decisions, and vehicle control are unchanged. This setting does not apply to C3/C3X. While an external HUD is connected, `ShowCameraWithCluster=0` hides both the on-device camera and lane/path graphics regardless of this choice.

This migration branch's Ambient HUD commands **20% in dark theme and 60% in light theme**. Stored `ClusterHudBrightness` and camera exposure are ignored; manual web changes are rejected. Automatic `ClusterHudTheme=0` switching remains available. These are device commands, not measured light-output percentages. `ClusterHudOrientation` supports only `0` (0 degrees) and `2` (180 degrees). The TURZX process checks theme brightness and orientation every 100 ms; a managed H.264 orientation change restarts the HUD.

`ClusterHudPanelLayout=0` places the driving view selected by `ClusterHudCameraViewMode` on the left and the information panel selected by the screen, debug, and navigation state on the right. `1` swaps them, placing information on the left and the driving view on the right. The running HUD applies the setting within about one second without a restart. Modes without two side regions, such as the full-screen graph and full-screen navigation, are unchanged. `ClusterHudDebug` forces always-on output and optional debug UI or navigation input; any resulting debug or navigation information panel follows the selected layout.

While the external HUD is connected over USB, `ShowCameraWithCluster=0` switches the on-device driving view on both regular C3/C3X hardware and mici to a black background and skips camera-video and model-path rendering. Setting it to `1` removes that connection-specific suppression and uses the normal on-device camera and on-road rendering path. A live change takes effect within about five seconds. Speed, speed limit, driver state, alerts, and the driving-state border remain visible with either value, and the option has no display effect after the external HUD disconnects. With value `0`, `camerad` and model input continue running; only duplicate rendering on the device display is reduced.

The final `ClusterHudScreenMode` layout is:

- `-1` uses the full width only in 3D camera views `0` and `1`, with no information panel or world shift. Left-side HUD items retain their margins, right-side gauges and TPMS align to the physical right edge, and the clock, world, and turn signals use the full-display center axis. In road-camera view `2`, it behaves exactly like mode `0`, including automatic navigation/report selection and `ClusterHudPanelLayout`.
- `0` is the default screen. It shows navigation while live navigation is received and automatically shows the driving report otherwise.
- `1` is the general live-debug screen, grouping delay, torque, steering, and lateral-plan state into four 2×2 cards. In 3D camera views, the world and driving HUD use the same 1124 px driving region as mode `0`, while the four cards fill the opposite 792 px information region. The acceleration, steering, fuel, and DEF gauges plus TPMS remain inside the driving region's right edge.
- `2` is the fixed system-debug screen. Its detail card shows the network address, output resolution/frame rate, camera state, memory capacity, and per-core CPU use. Its system-health card adds 2×2 circular gauges for CPU load, temperature, memory, and disk use, plus the device pitch/yaw target and numeric angles. In 3D camera views, the world and driving HUD use the same 1124 px driving region as mode `0`, while both cards use the opposite 792 px information region. It remains fixed regardless of connected, live, disconnected, or debug navigation state and never switches automatically to navigation, `NAVI DISCONNECTED`, the route overlay, or the driving report.
- `3` disables the driving scene and shows the `ShowPlotMode` graph at large size.
- `4` fits the 3D world and driving HUD into the same 1124 px driving region as mode `0` and uses the complete opposite 792 px information region for the graph. The acceleration, steering, fuel, and DEF gauges plus TPMS remain inside the driving region's right edge. Swapping the panel layout exchanges the graph and driving regions together.
- `5` always shows the driving report.

`ClusterHudScreenMode=5` shows a live driving report in the information panel. In default screen mode (`0`), the same report is shown automatically while no live navigation is being received, and the navigation panel returns when reception starts. The report background, cards, outlines, primary text, secondary text, and unavailable-value colors follow the active `ClusterHudTheme`, including Auto, Dark, and Light modes. Its large card summarizes driving time, distance, average and maximum speed, the automated-driving ratio, maximum acceleration/deceleration, and hard acceleration/braking/corner counts. The small card presents CPU load, temperature, memory, and disk use as a 2×2 set of circular gauges. Its lower target plots the stored device pitch (P) vertically and yaw (Y) horizontally relative to the calibrated center while retaining the numeric angles. The driving area retains the branch, network address, and frame-rate status; the core-usage text is omitted when it would overlap the report. In road-camera view, detected vehicles are enclosed by transparent rounded frames whose border retains the existing detection color; ungrouped radar detections use smaller transparent rounded markers in their source color. Vehicle frames use one lightweight outline, and frames that would be partially projected at the screen edge or stretched by a noisy radar heading are omitted.

The external HUD follows the device `LanguageSetting` and updates driving-report, driving-mode, and navigation status labels live in Korean (`ko`) or English (`en`). Other language values, including Chinese, fall back to English. With `IsMetric` enabled, vehicle/cruise/limit speeds, navigation, radar labels, and the driving report use `km/h`, `m`, and `km`; with it disabled they are converted to `mph`, `ft`, and `mi`. Acceleration and temperature remain `m/s²` and `°C`. Both settings are polled about once per second and do not require a HUD restart.

The enlarged driving-mode text beside the speed display has no background or border and remains above the final speed digit. Its text is green for Efficiency, orange for Safety, white for Normal, and red for High Speed. The gear badge also has a transparent center, retaining only its letter and outline. The upper-left steering/LFA, Wi-Fi, and clock row keeps extra outer and inter-icon spacing, and the speed-limit sign is offset slightly left.

`ClusterHudTheme=1` (Dark) renders the normal HUD's empty background in the same pure black used behind maps and while navigation is disconnected. Auto (`0`) uses the same dark palette from 18:00 to 06:00. The road, gauges, and regular information panels retain distinct dark shades for separation and readability.

On Hyundai/Kia CAN-FD hybrids, the external HUD's green `EV` indicator is enabled only when ECAN `0xFA` and `0x230` are both present with DLC32. It decodes the four-bit hybrid power-flow mode in `0x230` and shows `EV` for the observed motor/regen modes 1, 2, and 6. The normal HUD shows it between vehicle speed and cruise-set speed; full navigation intentionally omits it. The indicator remains hidden for other mode values and when the capability or sample is missing, invalid, or stale.

The normal external HUD also shows the current driving mode beside the traffic-state dot above vehicle speed. `MyDrivingMode` value `1` is a green Eco badge, `2` an orange Safe badge, `3` a white Normal badge, and `4` a red High badge. The badge is hidden for an unavailable, invalid, or stale `longitudinalPlan` and for values outside that range; full navigation omits it. The adjacent red or green dot is an independent model traffic-state indicator, not a driving-mode state.

The normal and road camera HUDs use the same fixed TPMS position below the acceleration, steering, fuel, and DEF gauges. The pressure font size is unchanged, with each value placed inside one of the enlarged tires of a simple toy-car diagram. The whole display is hidden only when all four pressure values are unavailable; an individually missing value shows `--`. Pressures below 31 psi are red, and no surrounding card or outline is drawn. When external navigation is active or its dashboard is connected, a green `NAV` appears below the Wi-Fi icon instead of the former lower-right `NAVI` label. The center clock, EV indicator, and fuel/DEF gauges remain unchanged.

### Carrot Vision AR and replay navigation events

Carrot Vision provides separate **Show AR** and **AR debug** controls outside the `carrot_settings.json` catalog. **Show AR** overlays driving guidance on the Vision video and requests the additional real-time data only while it is enabled. **AR debug** adds a troubleshooting panel with sign, anchor, and draw counts, the current blocking reason, and copyable history.

The Replay event timeline also identifies Carrot Navi connection and route-session changes, current and next maneuvers, lane guidance, road-safety alerts, average-speed zones, traffic signals, and intersection guidance. These entries are labels for reviewing transitions recorded in the replay.

<a id="system"></a>
## System

The 12 system settings cover recording, power, network, maps, sound, and software menus.

| Group | Parameters | Purpose |
|---|---|---|
| Recording and power | `RecordRoadCam`, `MaxTimeOffroadMin` | Road-camera storage and delayed shutdown |
| Network and map | `HotspotOnBoot`, `MapboxStyle` | Boot hotspot and map background style |
| Sound | `SoundLanguageSetting`, `SoundVolumeAdjust`, `SoundVolumeAdjustEngage` | Prompt language and volume |
| Software | `SoftwareMenu` | Carrot Web software-menu availability |

Check storage use for recording and network use, heat, and privacy before enabling live streaming.

In this migration branch, `MaxTimeOffroadMin` supports up to `4320` minutes (three days); existing saved values are unchanged. Automatic shutdown happens only while the harness is detected as connected, and never while ignition is on: USB power such as a phone charger or laptop, detected as no harness, never powers off by itself (unless the harness is misdetected as connected). The offroad timer requests shutdown only after the longer of the setting and five minutes offroad, and only once the device has gone onroad (ignition ON) since hardwared started or an hour after booting (so even a setting of 0 does not power off right after an install or power-up, 2026-11-16). The low-voltage shutdown has no such wait. Low-voltage shutdown follows after both raw and filtered battery voltage stay below `11.0V` (changed from `11.3V` on 2026-11-16) for ten seconds with fresh measurements. Ignition ON, voltage recovery or missing measurements reset that confirmation. The manager checks fresh ignition, offroad and harness state again immediately before requesting poweroff; stale automatic requests cannot latch a manual shutdown. The estimated battery-capacity cutoff is not used. Manual shutdown/reboot behavior is unchanged. Long operation and a low voltage threshold increase depletion and no-start risk.

`CarrotNaviSource`, under Speed and deceleration, selects `0` network guidance or `1` Navdy. Restart after saving. The default transport policy tries direct USB, then the configured Bluetooth peer and paired Jetson relays; USB reattachment takes priority. An explicitly configured `bluetooth_only` policy keeps telemetry on Bluetooth even with USB attached for diagnostics. Bluetooth requires the compatible Navdy receiver and prior pairing; a Git update alone does not install the APK or enroll a peer. See [Bluetooth setup](../../../tools/navdy/BLUETOOTH.md). Navdy mode does not fall back to network guidance; music and clock synchronization remain available. See [input and expiry rules](speed-deceleration.md#navdy-input).

`SoundVolumeAdjust = -1` displays **Automatic** and adjusts general sound volume using microphone ambient-noise measurements. `0` mutes general sounds; `1-200` remains a percentage multiplier of automatic volume, not a fixed absolute volume. `-1` and the existing `100` use the same volume curve. Changes apply at the next noise measurement while no alert is playing. The reboot chime has no ambient sample and uses its default `0.5` gain in automatic mode. Engage volume and browser playback volume retain their separate settings. DM's first audible warning has a 70% minimum output gain; its final warning always uses 100%, regardless of volume settings or ambient-noise attenuation. The preceding visual-only notice remains silent. See [driver monitoring](driver-monitoring.md) for details.

Experimental DM switches to standard monitoring for 20 seconds only when moving traffic first appears after an absence. Additional vehicles do not extend it; experimental monitoring then resumes, with no empty-road bonus while traffic remains.

## Signal Assistance Mode (Trial)

`SignalAssistMode` defaults to `0`. Mode `0` disables auxiliary signals; `1` enables recognition, display, alerts and the stopped-red departure veto; `2` also enables moving-red preparation.

Mode `2` gradually requests acceleration toward approximately −0.5m/s² after qualifying red evidence, with no lead, pedal input or turn and a speed above 5km/h and below 82km/h. At night, at least two persistent red lamps must have compatible placement and road-direction support; inferred housing size is not used as a stopping distance.

Preparation releases when red evidence disappears or conflicts with green. The existing driving model still controls stopping toward the stop line; this trial does not guarantee deceleration or stopping at every red signal.

## Safe adjustment order

1. Record the current value in Carrot Web.
2. Save both a baseline profile and a file backup.
3. Write down the parameter, previous value, and reason for the change.
4. Change only one item by one `unit` step.
5. If a restart is required, restart only while parked and off-road.
6. Repeat the same test in a permitted, controlled environment.
7. Restore the previous value or baseline profile immediately if the result is worse or unclear.

See the Wiki [Tuning introduction](https://github.com/ajouatom/openpilot/wiki/Guide-Tuning) for the recommended steering and longitudinal adjustment order.
