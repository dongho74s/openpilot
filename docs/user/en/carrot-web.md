# Carrot Web User Guide

[한국어](../ko/carrot-web.md)

Carrot Web is a local web interface for viewing and managing carrotpilot from a phone, tablet, or PC connected to the same network as the device. It provides driving status, settings, logs, updates, and diagnostic tools.

> [!WARNING]
> Do not operate Carrot Web while driving. Change settings, select logs, enter terminal commands, and run tools only after parking safely.

## Connecting

1. Connect the carrotpilot device and your browser device to the same Wi-Fi network or hotspot.
2. Find the carrotpilot device IP address.
3. Open `http://device-IP:7000` in the browser.

Example: `http://192.168.0.25:7000`

Select the white carrot icon on the device to show a large QR code for its current address. On C3, use the bottom-left button; on C4, use the bottom status-icon row. Scan it with a phone on the same network to connect. If the device IP changes while the QR screen is open, both the QR code and displayed address update automatically. Long addresses scale to fit instead of being shortened. The last QR refresh time appears below the address as numeric `HH:MM:SS`, with the 30-second auto-close countdown on the right. Tap the QR screen to close it, or leave it open and it closes when the countdown reaches zero.

Carrot Web is a local device-management interface. Do not expose it directly to the internet or give its address, a remote-support link, or terminal access to an untrusted person.

## Pages at a glance

A wide display (a PC, a tablet, an unfolded Fold, a phone held sideways) shows the left rail, and Settings shows the groups on the left and the items on the right. A portrait phone shows the floating bottom bar and one column. Below 1000 px (an unfolded Fold) the rail is narrower, the Tools notifications pull down from the top and Logs stacks each route card. On very wide screens the settings items and the Tools cards keep a readable width.

| Page | Main purpose |
|---|---|
| Drive | Live road video, HUD and driving data, screen-recording control, and recorded-log replay |
| Settings | carrotpilot and device settings, search, favorites, profiles, and change history |
| Tools | Vehicle and language selection, web layout, Git management, backup/restore, and system actions |
| Logs | Dashcam routes, playback, analysis and upload, plus screen-recording playback and download |
| Terminal | Device terminal and time-limited remote-support sessions |

Some controls or content may be absent depending on device type, driving state, branch, and browser capabilities.

Settings, Tools and Logs follow the phone's (browser's) light or dark setting by default; `Tools → Theme` can pin them to light or dark. Drive and Terminal are always dark. The bottom navigation is a rounded bar floating over the page; the current page is shown in bold.

## Drive page

The Drive page displays live video and driving state. Its usual layout contains road video and an auxiliary information area. Use `Tools > Web Settings` to adjust split direction, visible areas, and content.

### Live video and HUD

With `CarrotVisionEnabled` on, you can start the web camera view while the external HUD is enabled. Simultaneous video use may increase device load; if video stutters, stop the web camera view and compare.

- Select `Start Drive Vision` to begin connecting to the camera stream.
- The HUD on the video looks like the C3/C3X `new ui` 1 screen: current speed (KM/H) at the bottom centre, the round steering-wheel button at the bottom right with the `SET` speed and following-gap icon to its left, the LFA icon next to the speed (green when engaged, mint when only steering) with the APN badge, turn signals, blind-spot icons, and alert boxes along the bottom.
- When neither engaged nor steering, the video turns grey and dim, lane lines are hidden and the path is drawn grey. During a lane change the line being crossed disappears and the lane edges turn mint. In lane mode your lane edges are green.
- Values the web does not receive are not shown: the driver-monitoring icon, the standstill timer after a minute stopped, the lead-departed notice, and the gas-override (blue) look.
- Enable or disable AR in `Tools > Web Settings`.
- If another browser owns the video stream, the page may say that it is in use elsewhere. Select `Use here` only when you intend to move the stream to the current browser.
- A parked device or unavailable camera/driving data produces a waiting state instead of live video.

The web video and HUD are reference displays. Always prioritize the vehicle's actual warnings and the road ahead.

With `ShowPathEnd` on, the lead car gets a rounded box and distance badges (radar and vision) like the C3 screen; they are hidden while neither engaged nor steering. The old web following-distance bar and its 25 m label are not part of the C3 `new ui` 1 screen and are no longer drawn. See [following-gap settings](cruise-gap.md#following-gap) for the gap itself.

### AR display and diagnostics

- Enabling **Show AR** in `Tools > Web Settings` overlays Carrot Navi driving guidance on the Carrot Vision video and requests additional real-time pose and position data only while it is enabled.
- **AR debug** shows sign, anchor, and draw counts plus the current blocking reason, and can copy or save its diagnostic history.
- The replay event timeline distinguishes recorded Carrot Navi connection changes, current and next maneuvers, lane guidance, road-safety alerts, average-speed zones, traffic signals, and intersection guidance.

### Layout

`Tools > Web Settings > Layout` controls:

- horizontal or vertical split;
- split view, Area 1 full screen, or Area 2 full screen;
- road video or another supported content type in each area; and
- swapping the two areas.

A fresh install starts with **Area 1 full screen** in both orientations: Area 1 is **Carrot Vision** and Area 2 is **Carrot Navi**, so the Drive page shows Carrot Vision full screen. Switching to Split divides landscape 70% / 30% and portrait 50% / 50%.

**Default** restores both orientations to **Area 1 full screen** with Area 1 Carrot Vision and Area 2 Carrot Navi.

Unavailable content reports waiting, recovering, unavailable, or unsupported. Changing the web layout does not change vehicle-control behavior.

### Starting and stopping a screen recording

The `Record` button on the Drive page toggles the device `ScreenRecord` state.

1. Select `Record` to start.
2. Confirm that a `REC` badge appears on the Drive navigation button.
3. Select `Record` again after capturing the required behavior.
4. Open `Logs > Screen Record` to play or download the saved file.

A screen recording helps explain what was visible, but it does not replace the raw driving log required for vehicle-control analysis. Use [Sending Dashcam Logs for Analysis](dashcam-log-sharing.md) for abnormal driving behavior.

### Replaying a recorded drive

From `Logs > Dashcam`, open a segment menu and select `Replay` to display the saved road video with recorded driving data on the Drive page. Previous/next segment navigation, playback speed, HUD, events, graphs, sensor views, and raw-log analysis depend on the data stored in that segment and browser support.

Recorded replay may stop automatically when real driving begins.

## Settings page

The large words `carrotpilot` and `Device` at the top of Settings are its two tabs; the bold, dark one is the current tab.

- The first screen has the search field, the `Changed settings` card, Favorites, and the group tiles, in that order.
  - Each group tile shows how many settings it holds and how many differ from their defaults (`38 settings · 17 changed`).
  - The large number on `Changed settings` counts every setting that differs from its default. Tap the card to keep only the groups with changes, and only the changed rows inside them; tap again for everything. A reload goes back to everything.
  - Favorites show their current values as a row of cards; tap to open the Favorites list.
- A group's list shows each item's name and current value only. A value that differs from its default is carrot-coloured, with the default in small type beneath it (`Default 160`). A bracket at the end of a title that only repeats the default, such as `(160)`, is left out of the list.
- On/off items switch right in the list.
- Tap any other row to open the value sheet from the bottom.
  - Change the value with the `−`/`+` beside the large number (hold to repeat), or tap the number to type one.
  - You can also drag the ruler under the number. The centre needle is the current value and the small dot is the default; the value is saved when you let go.
  - `Step` is how much one `−`/`+` press changes; each tap cycles it.
  - The description, the setting key, `Back to default N` and `History and guide` are in the same sheet. Choice items list their options.
  - Tap outside or press `Esc` to close.
- `All / Changed` above the list shows only the items that differ from their defaults.
- On a wide screen the groups sit on the left and the items on the right; at 1100 px or more the items sit side by side in two columns.
- Long-press a setting to add or remove it from Favorites.
- Profiles save a group of values and allow selected values to be applied later.
- A setting detail page (`History and guide`) can show popular values, recent change history, and a verified Wiki guide.
- Risky settings carry a caution mark, and some settings take effect only after reboot.
- Resetting defaults changes many values; make a backup first.

See [Understanding Settings](settings.md) for ranges, units, and a safe tuning order.


### My Traverse key

`Settings → Device → My Traverse key` shows this comma's My Traverse (Wayon) device ID and key. The comma uses the key to upload drives and photos to the My Traverse cloud.

- At first only the first and last four characters of the key are shown.
- `키 복사` (Copy key), after confirmation, copies the whole key to the clipboard. Carrot Web runs over plain http, so the browser may block automatic copying; the key is then shown in a field below for you to copy by long-press. It is cleared after 60 seconds, or when you switch to another app or another Carrot Web page. If you leave while the key is still being fetched, the fetched key is dropped and never shown.
- A copied key stays on the clipboard. After pasting it, copy something else over it.
- The page only opens when you reach the comma's own address, from the same Wi-Fi or your own Tailscale at `http://<comma address>:7000`. Requests carrying proxy headers (`Forwarded`, `X-Forwarded-*`, `Via`, `X-Real-IP`) are refused, and other sites cannot show Carrot Web inside their own page.
- These checks only narrow who can reach the key page; they do not prove you are the owner. Anyone who can open Carrot Web from the same Wi-Fi or Tailscale can also read the same file through its terminal and tools. Do not put the comma on a Wi-Fi you do not trust, or share it with others on Tailscale.
- Anyone holding the key can reach the cloud as this comma. Do not send it to others or paste it anywhere public.
- If no key is reported, the comma has no `/data/wayon_cloud/config.json`. That file is not part of the git branch; a new comma gets its own key on this page as below.

#### Getting this comma's own key (another comma)

A comma without a key can get **its own key** on this page and register to the My Traverse cloud as a separate device; its drives, photos and locations stay apart from other commas.

1. Open `Settings → Device → My Traverse key` in the new comma's Carrot Web. Without a key it shows `이 기기 키 발급받기` (Get this device's key).
2. Press it and confirm with `발급` (Get). The comma makes a key, proves to the cloud that it is this comma by signing with its comma device key, and registers.
3. Copy the key with `키 복사` (Copy key) into that car's My Traverse app. The comma starts connecting by itself shortly after.

- Only a comma registered with comma's servers (one with a device ID) can get a key, and only for its own ID.
- To protect the cloud's usage (the D1 free tier), at most five devices register this way in any 24 hours and five in total; beyond that the page says to contact the administrator.
- If the device ID is already registered with another key, the page says the key must be reissued.
- If the reply is lost or writing the key file fails, press again on the same page; it finishes with the key it made first.
- If an earlier attempt for another cloud or device ID is left (`/data/wayon_cloud/register.pending.json`), getting a key stops. Delete that file only if you are sure that attempt failed.
- If a key file already exists (even an unreadable file or a link), the button is not shown and the file is never overwritten.
- The new comma's key file holds only its own cloud address, key and device ID, with the original car's external vehicle-status feed switched off. Phone-proximity auto lock/unlock, GM remote start and door locks, and Traverse-only vehicle signals (fuel, odometer, door state) do not come with it.
- There is no page yet to remove a registered comma from the cloud.

## Tools page

Tools contains management actions that can immediately change device or repository state. From the top: the driving model, the last drive's evaluation, Update, Shortcuts, Device, Settings backup, and the actions that can't be undone. Tap the status line under the title to open the notification (activity) panel.
Manage `ONNX Lane and BSD Detection` in **Settings → Driving → Steering → ONNX Lane & BSD**, not through a Tools shortcut.

### Changing the driving model (internal model)

The **Tools → Driving model card** selects the driving model that runs on the comma's own GPU.
The default is `CD210` (comma's official model). The list contains only models whose inputs and outputs
this comma can run: the stock layout (with a 24- or 32-frame history) and the layout upstream uses since
September 2026 (the history kept inside the model).

- Available now
  - `CD210 = WMI V12` (default): comma's official model. Its weights are identical to sunnypilot's `WMI V12`
    (upstream exported the same weights again).
  - `RDF` (recommended, experimental): an unmerged model comma tested in August 2026.
  - `WMI V11` (experimental): sunnypilot's `WMI V11`, which reverted V10, so its weights equal `WMI V9`. Its vision
    is the stock one; only the policy (path and speed planning) differs. In sunnypilot V9 and V11 differ only in
    lateral smoothing (0.1 s → 0 s); carrot uses the `LatSmoothSec` setting for it instead.
  - `CH Driving` (experimental): an unmerged test model comma pushed only to a branch on 5 November 2026
    (commaai/openpilot `875a6b7e`). It is newly trained, not a retrain of an earlier model, and keeps its
    32-frame history inside the model (the newer layout). It downloads from comma's Hugging Face storage.
  - `Polaris Vega`, `Polaris Vega 2` (experimental): the StarPilot community's "Polaris Series" (MIT license),
    comma's RDFv4 model with its action and desire (lane change) stage retrained. Its authors say it learns from
    the best comma models (distillation) and fills the cases they all miss with human driving data. 32-frame history.
    `Vega` came out on 7 November and `Vega 2` on 22 November. The files download from StarPilot's public Hugging
    Face storage; their checksums are pinned, so if the source changes the comma does not use it and the card says why.
- To change it, park with the ignition off, tap **Change** on the card, pick a model, then press the **Apply** button that appears. The card shows the model in use in large type with its state beneath it (`Stock model`, `Ready`, ...). With the default settings it cannot be
  changed while driving.
- **Switching while driving (test, off by default):** with `Switch Driving Model While Driving (test)`
  (`InternalModelLiveSwitch`, Settings → System → Driving Model) on, a model can be picked while driving, and a model
  that is already compiled (and the stock model) switches **without restarting the car or rebooting the comma**: as
  soon as openpilot is disengaged and neither steering nor speed control is active. If you apply it while engaged, it
  switches the moment you disengage. It is still a test feature: try it parked (P) first.
  - Engaging is blocked first, and the model loads only once that block is confirmed. If you engage meanwhile, the
    switch waits and happens when you disengage again.
  - While the model loads, the path on the driving screen pauses for a moment (one to a few seconds).
  - The new model starts with an empty history, so engaging stays blocked until the new model itself has run a full
    history (about 5 s for a 24-frame model, about 6.5 s for a 32-frame one, plus 1 s; longer if frames are missed).
    Trying to engage meanwhile shows "주행 모델 바꾸는 중" (driving model switching), and steering and speed control
    (AlwaysLateral included) all pause meanwhile.
  - While a Jetson or eGPU model drives, the switch waits for the next drive instead.
  - If modeld restarts during a switch, the block lasts until the model it loads again has a full history.
  - Turning the setting off while a switch waits cancels it (the model is used from the next drive); once loading has
    started, the block stays until the history is full.
  - A model that is not compiled yet is not compiled while driving (it would share the GPU with the running model).
    It is downloaded and compiled once parked and used from the next drive; after that it switches right away.
- While parked the comma downloads the model (about 100 MB), checks it against the pinned checksum and the expected
  inputs/outputs, then compiles it on the comma (about 10-20 minutes). Turning the ignition on pauses this until you park again.
  - Upstream published `WMI V11` only as a separate policy file, so the comma downloads that file (about 14 MB) and
    builds the model by replacing the policy weights in the stock model file. The stock file, the policy file and the
    result are all checked against pinned checksums. If an update changed the stock model file, it is not built and
    the card says why.
- When it finishes preparing while parked, the model is used **from the next drive**. `In use` on the card means it actually ran.
- The stock model is never removed. The comma drives with it automatically, and the card says why, when:
  - compilation has not finished, or a software update requires compiling again (this runs automatically while parked)
  - the model could not be loaded
  - the model failed while driving (an error or invalid output such as NaN; press **Apply** again to retry)
  - the model did not start properly twice in a row (press **Apply** again to retry once)
- To go back, select `CD210` and press **Apply**.
- Each model drives differently (position in the lane, curves, stopping and starting). Check an experimental model carefully at first.

### Tuning from the last drive

The **Tools → Tuning from the last drive card** shows a steering-tuning suggestion from the last drive's logs and how well the car followed the model's intent.

- **When it runs:** it picks the latest drive with more than 3 minutes of steering above 8 km/h (short moves such as parking are skipped), always at the lowest CPU and disk priority.
  - **Permanent power:** a minute after the ignition goes off the comma analyses the drive on its own (usually 2–3 minutes).
  - **The comma powers off with the car:** a minute after the next ignition it analyses the previous drive (never the one being recorded). Open Carrot web during that drive to see the result.
  - Switching the ignition on does not stop it. **In P at a standstill** it runs at full pace; once you drive off it carries on, resting as long as it worked after every step (usually 5–6 minutes in all) so the drive is not affected.
  - With the ignition on it waits while free memory is under 600 MB (about 860 MB is typical while driving; the low-memory alert comes at about 360 MB) or the CPU/GPU is at 75 °C or more, and puts the analysis off to the next ignition if it has to wait more than 10 minutes.
  - While it runs, the card shows the percentage and a progress bar, e.g. `37% done · reading the logs (8/22 segments)` (updated every 3 seconds). The steps are: finding the last drive → reading the logs → replaying the same curves.
  - **Analyse again** on the card reruns it at any time.
- **Model intent vs actual:** how closely the two lines of debug plot 6 (green = the lateral accel the controller requested, yellow = the measured one) agreed.
  - Average match = 1 − Σ|actual − requested| / Σ max(|actual|, |requested|): the average share of the requested turn the car achieved.
  - Curve and straight match = the share of time the difference stayed within the tolerance (0.3 m/s² or 20 % of the request, whichever is larger) in curves (request above 1.0 m/s²) and on straights (below 0.5 m/s²).
  - Time with the driver on the wheel and below 8 km/h is left out.
- **Reading the card:** the one line at the top is what to do: `Keep your current settings`, or `Change one value: Lateral accel limit 35 → 40 (3.5 → 4.0 m/s²)`.
  - Below it, **Settings checked** lists each setting on one row: its name, its key (e.g. `LateralTorqueKiV`), the value the drive used, **Keep/Change** and a short reason (`Changing it makes no difference`, …). Values are the numbers you type in the settings page; the brackets give their meaning (e.g. `70 (0.70)`).
  - Tap a row to jump to that setting in the settings page. With a suggested change, **Change it in settings** takes you there, or press **Apply**.
- **Suggestion:** the comma replays the drive's own curves (simulation) with one value changed by one step at a time: the lateral accel limit (`LateralAccelLimit` ±0.5), the lateral jerk limit (`LatJerkMax` ±1.0), steering I (`LateralTorqueKiV` ±0.20), steering Kf (`LateralTorqueKf` ±0.10), the torque-limit P filter (off ↔ 0.3 s) and the GM torque rise and fall rates (`CustomSteerDeltaUp` ±2, `CustomSteerDeltaDown` ±3). The torque rates start from 0 (the car's own: 10 up, 15 down on the Traverse) and stay inside what the panda allows (12 up, 25 down on the Traverse). Steering I, P, Kf and Kd are only looked at with `Custom torque tuning` (`LateralTorqueCustom`) on.
  - The car's response is a model fitted to that drive. The same setting replayed with that response changed by 10 % is the noise; only differences larger than that count.
  - A change is suggested only if it helps in most curves (60 % or more) and hardly any get worse, cuts the error by more than 8 % or the drift to the outside by more than 3 cm, and adds no jitter, straight-line error, torque let-go at the limit or time at the limit. Otherwise it says **Keep the current values**.
  - Steering P is shown for reference only (the simulation once judged P differently from the real car, so P is decided from real drives). Steering Kd too: Kd reacts to sensor noise, and the replay has none.
  - **Steer ratio and steering delay** cannot be judged by the replay (it runs in lateral-accel space, where the steer ratio cancels out), so they are compared with what the comma learned while driving.
    - Steer ratio: the median learned ratio (liveParameters) against the one in use (`CustomSR`, or the learned one × `SteerRatioRate` when 0). More than 8% apart shows **Check** with the value that would match, unless the difference is intended. It is never the automatic suggestion.
    - Steering delay: the learned delay (liveDelay) beside `SteerActuatorDelay`, as **Info**. Cars on FrogPilot-style NNFF such as the Traverse steer with the learned delay; the setting only shifts the lane-mode path (0 uses the learned one there too).
  - Friction (`LateralTorqueFriction`) and accel factor (`LateralTorqueAccelFactor`) are not used with NNFF and are not checked.
  - A tuning value changed during the drive: only the part after the change is analysed, with that value. The GM torque limits (`CustomSteerMax`, `CustomSteerDeltaUp/Down`) set back to 0 mid-drive keep their last value in the car controller for the rest of the drive, so the analysis does the same; they are read every 0.5 s, so one set back within a second may never have been used and only the part before the change is analysed.
  - One suggestion at a time. **Apply** changes that one setting right away and records it in the settings history with source `tuning_advice`. It works only with the ignition off; the comma checks again right before writing, so switching the ignition on while the confirm dialog is open changes nothing. While a newer drive is not analysed yet or the latest analysis failed, an older suggestion cannot be applied and the card says so.
  - The simulation uses the drive's steer-ratio settings (`CustomSR`, `SteerRatioRate`) and the GM custom torque limits (`CustomSteerMax`, `CustomSteerDeltaUp/Down`). Stretches with steering requests it does not reproduce (lane mode, the minimum-distance clamp) are left out of the comparison.
- **Last change:** when this drive and the drive analysed before it differ in exactly one tuning value (switching custom torque tuning on or off counts as one), the card puts their match, takeovers, torque let-go and tracking error side by side and says better / worse / unclear. Different roads give different figures, so treat it as a hint.
- **Not enough data:** under 3 minutes of steering, fewer than 3 curves steered without the driver, or only slow city streets (no steady steering to fit the car's response): no suggestion, with the reason shown. The simulation covers GM torque steering only; other cars get the match and the drive figures.
- The simulation has no driver and no model reaction in the loop. A suggestion is "worth trying on the next drive"; a real drive confirms it.

### Jetson connection, IP and temperature

After Jetson integration, the **Tools → eGPU status card** in `carrot-wip` also
shows a connected Jetson's IP, temperature and current connection/error state,
even without a previous eGPU connection. Open Carrot Web using the comma IP;
the Jetson IP displayed inside the card is its management address. Expired
temperature/address readings are not presented as current values.

A healthy idle installation using the internal model without Jetson is distinct
from active inference losing its connection, a real error or excessive heat.
Do not suppress real faults to clear a warning. The eGPU model compilation button
is unavailable for a Jetson-only installation.

Use the [short Korean installation guide](https://github.com/ajouatom/carrot-jetson/blob/main/docs/INSTALL-WINDOWS-KO.md):
extract the installer ZIP and run 01, then 02. Image preparation and the USB-C fix are automatic. Wi-Fi settings come from the connected
comma, so changing its SSID/password does not require removing the SD card.
The existing P-gear summary display remains intentional. See the guide's preview
notice for physical validation limits of each installation path.

### Shortcuts

- `Car Select`: choose the vehicle maker and model.
- For the 2026 electric Staria, select `Hyundai Staria EV 2026`. Its accelerator and gear decoding differs from `Hyundai Staria 2023`; an existing selection is not migrated automatically. Reboot after selecting it, then check gear/pedal indications and CAN errors while parked.
- `Language`: change the Carrot Web language.
- `Web Settings`: configure the Drive layout, video, HUD, AR, and other web-only display options.
- `Info`: view and copy the device type, branch, commit, identifiers, and network information.
- `Bluetooth remote`, `Map search`
- `Theme`: `Automatic (follow the phone)`, `Light` or `Dark`. The choice is saved in that browser only, so a phone and a PC can differ. Drive and Terminal always stay dark.

### Update

- `Get update` (git pull): fetch and integrate new commits for the current branch. The right side shows how many commits are waiting (`New 3`), or `✓` when up to date.
- `Sync` (git sync): synchronize with the remote branch; local work may be cleaned up.
- `Change log` (git log): inspect commits and choose a commit or branch.
- `Change branch`: choose a local or remote branch.

### Device

- `Screen recording`: a switch that starts and stops screen recording; the elapsed time is shown while recording.
- `Reboot`: reboot the device (asks first).
- `Save tmux log`: capture the current tmux log and download it through the browser.
- `Send tmux log`: request a server-log upload.

CAN diagnostics are uploaded automatically only when a currently received `carState` or `radarState` from the present onroad session reports a real CAN error. A timeout left over from the previous drive is not used, and capture is delayed for five seconds after detection so the log includes the immediate aftermath. In an automatic diagnostic log, `CarrotException can_error queued` means the upload was queued; the preceding `current onroad CAN error detected from ...` line identifies the source used for the decision.

### Settings backup and restore

`Backup`/`Restore` (file), `QR Backup`/`QR Restore`, `Copy` (setting values) and `View` (all values). Compare current and backup values before restoring. A backup from another vehicle or an older branch can have different meaning or recommended ranges even when parameter names match.

### Can't be undone

The red items sit apart at the bottom. Each one asks again before it runs.

- `Discard local changes (git reset)`: move code back to a selected reference (hard/mixed/soft).
- `Reset calibration`: clear camera calibration and start calibration again; the device reboots.
- `Rebuild everything`: rebuild and restart the software.
- `Delete all videos`: delete screen recordings.
- `Delete all logs`: delete stored driving logs.
- `Reset repository` (reset repo): forcibly reset repository remotes and branch state.

> [!CAUTION]
> `Sync`, `Discard local changes`, and `Reset repository` can discard local code changes. If you do not understand the action, do not run it. For a support request, read the current branch and commit from `Info` instead.

After deleting logs or videos, Carrot Web cannot replay or upload them for analysis. Complete any support upload first.

### System command field

The `Command` field (just above the actions that can't be undone) accepts only server-allowlisted Git management or inspection commands and diagnostics such as `df`, `free`, and `uptime`. Type a command and press `Run`; the output appears in the notification panel (tap the status line under the title to open it). Prefer the prepared buttons for routine device operations.

## Logs page

Logs has `Dashcam` and `Screen Record` tabs.

### Dashcam

- View a card and time range for each drive.
- Expand a card to see segments of about one minute each.
- Select a segment to play its road video.
- The segment menu offers recorded-drive replay, log upload, and `qcamera`, `rlog`, or `qlog` download.
- `Drive Report` summarizes assisted/manual driving time, distance, speed, hard acceleration/deceleration, interventions, and warnings.
- Check segments and use `Upload selected`, Select all, or range selection for a multi-segment upload.
- The top-right Logs menu provides sorting and quick upload of the newest 2, 5, or 10 completed logs.

See [Sending Dashcam Logs for Analysis](dashcam-log-sharing.md) for the complete problem-selection and upload procedure.

### Screen Record

The newest files appear first with name, saved time, and size. Select a row to play it or use the download button on the right to save the original. This tab displays existing recordings; start and stop recording with `Record` on the Drive page.

## Terminal page

Terminal connects to the device's `carrot-terminal` tmux session. It provides reconnect, clear display, `Ctrl+C`, arrows, Tab, and other mobile-friendly helper keys.

Terminal commands run immediately on the device and can have a much wider impact than prepared Tools actions. Do not enter a command you do not understand. If disconnected, try `Reconnect` first. `Clear` only clears the visible terminal display; it does not delete device files.

### Remote support

`Support` creates a time-limited terminal session for an external helper.

1. Enter a short symptom note.
2. Choose only the required duration—normally 15 minutes, 30 minutes, or one hour.
3. Prefer `Approve each` for command permission.
4. Send the support link and PIN only to the designated helper.
5. Read every command shown on the device and approve only what is required.
6. Select `Stop` as soon as support is complete.

Use `Allow all` or an unlimited session only if you understand the risk and will monitor the entire session. Never post the link or PIN in a public channel.

## Connection and display troubleshooting

1. Confirm that the browser and device are on the same network.
2. Check whether the device IP address changed.
3. Confirm the `http://device-IP:7000` address format.
4. Check whether a public Wi-Fi network blocks traffic between clients.
5. Refresh the page. If needed, reboot the device and wait for its services to start.
6. Include device type, vehicle, branch, commit, occurrence time, and exact error text in a support request.

A device with a SIM (cellular data) parked at the edge of a saved Wi-Fi keeps joining and dropping that Wi-Fi, which breaks its Tailscale connection. So while cellular data works and the saved Wi-Fi's signal is below 30, the device rests Wi-Fi autoconnect (the saved networks and passwords stay). It reconnects automatically once the signal holds at 45 or more for 10 s, or once no saved Wi-Fi has been in sight for 30 s. After 3 failed attempts within 5 minutes, even at a usable signal, it rests 5 minutes and then tries again. When cellular data is not working (nothing received over it for 2 minutes) it never rests, and a Wi-Fi you pick on the device screen still connects. A reboot turns autoconnect back on.

If only video is missing, check whether another browser owns the stream, whether the device is ready to provide camera/driving data, and whether the browser supports the required video codec and WebRTC.

## Related guides

- [Sending Dashcam Logs for Analysis](dashcam-log-sharing.md)
- [Understanding Settings](settings.md)
- [Buttons and Presets](buttons-presets.md)
- [Speed and Deceleration](speed-deceleration.md)
- [Cruise and Following Gap](cruise-gap.md)
- [Radar Tracks and Corner Radar](radar.md)
