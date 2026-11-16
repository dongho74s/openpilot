# Navdy surrounding-vehicle blinking (2026-11-16)

## Report

The user saw surrounding-vehicle icons on the Navdy appear and disappear,
"often when a car is close, or at random". The HUD draws every 5 Hz payload as
it arrives and has no hold or fade of its own for vehicles, so a vehicle missing
from one payload blinks for 0.2 s.

## How it was measured

`vreplay.py` (Claude scratchpad, not in the repo) replays a segment's rlog on the
comma through the same order as `bridge.live_payload`: `payload_from_messages`,
`VehicleOverlay.update`, `annotate_camera`, `settle_overlaps`, at every fourth
`modelV2` (5 Hz). The bridge's tick clock is never behind the messages it reads,
so the replay uses `max(model time, liveTracks time)` as `now`; using the model
event time alone made out-of-order log events look like stale radar data and
invented blinks. A blink is an icon with no icon near it (within
max(3.5 m, 15%) and 1.3 m) on the next send that comes back near the same place
within 1 s. For each blink the replay records which stage dropped the object.

Drives: 0000021f (2026-11-15 evening, 45 segments) and 00000220 (2026-11-16,
5 segments), 14,052 consecutive sends with the road drawn (about 47 minutes).

## Causes found (deployed 8edded84)

336 blinks, 234 of them one send long, and 137 jumps of more than 2 m where the
icon changed source.

1. radard alternates a far lead between its radar point and the model's own
   range several times a second. Example 0000021f--10: leadOne switches between
   radar track 25 at 56 m and a camera-only lead at 47 m every 0.2-0.4 s, while
   the stock camera object agrees with the radar at 55 m. The icon jumped 9 m or
   vanished, and the camera object or the radar point drawn behind it came and
   went (`settle_overlaps` hid it whenever the 47 m lead was in front).
2. The lane check (`vehicle_lane`, `_lead_lane`) flickers for a steady object
   on a lane line, at low lane confidence, or while stopped when the plan is too
   short to reach the object. The object is then dropped for a send.
3. A confirmed radar point missing from one snapshot was removed at once and had
   to be confirmed again over two 5 Hz sends (0.2-0.4 s off).

## Changes (display only)

- `VehicleOverlay`: a confirmed point missing from a fresh snapshot is drawn
  until `TRACK_HOLD` (0.35 s, Wayon's `NAVDY_RADAR_STALE_SEC`) after its last
  sighting, by radar time and by the tick clock, coasting at its relative speed.
  It is remembered for `TRACK_MEMORY` (0.6 s) so that a point back on the next
  5 Hz tick keeps its confirmation instead of blinking on the return tick. The
  hold never extends itself; unconfirmed points still go at once; stale data,
  radar errors, inactive display and the other existing clears are unchanged.
- A camera-only published lead is drawn on a radar point only when radard
  reported that same lead slot on that point within `LEAD_TRACK_MEMORY` (1.5 s;
  65 of 67 such stretches on 11/15 returned within 1.6 s), the point has been
  tracked without a break since, no radar-backed lead is on it, it passes the
  existing de-duplication gate, and the stock camera does not show the lead and
  the point as different objects (nearest camera object to each differs, or an
  object at the lead and none at the point). The lead keeps its colour role and
  longitudinal flag. The bridge takes the camera snapshot before
  `VehicleOverlay.update` and passes the same objects to both steps.
- A point whose lane check fails keeps its last lane for `LANE_HOLD` (0.35 s);
  a track that restarts under a reused id loses it. Camera-only objects get the
  same memory in `annotate_camera`, by camera object id and only near where the
  lane was given (the bridge passes `VehicleOverlay.camera_lanes` and its tick
  time).
- Tried and removed: holding a published lead that radard drops for a tick. It
  duplicated a vehicle that changed lead slot and lane, and revived leads from
  an invalid or stale `radarState` (Codex review N2/N4).

## Review

Codex reviewed four rounds (R1-R4, N1-N4 and a close cut-in follow-up, each
reproduced against the real display code); the last round found nothing more.

## Results

Same 14,052 sends, same replay, final code: 336 blinks became 220 (-35%).
Under 10 m 40 became 18, 10-25 m 73 became 47, 25-50 m 107 became 73, over
50 m 116 became 82. The 0000021f--10 lead that alternated between 56 m and
47 m now stays at the radar's 56 m on every send.

What is left, by the stage that dropped the object:
- camera-only leads with no radar point that radard reports off for a tick or
  whose lane check fails (about 48);
- camera-only objects the stock camera itself drops for a tick (about 49);
- vehicles hidden for a tick by `settle_overlaps` behind another icon (38);
- lane checks failing for longer than 0.35 s (about 33).

Mac Navdy suite: 184 failed / 1544 passed; the 184 are the same list as the
8edded84 base (Mac environment), plus the new tests.

## Not verified

- On-road: the HUD during a real drive after the change.
- Objects that really leave stay up to 0.35 s longer than before.
- Coasting assumes a constant relative speed and a fixed lateral position for
  at most 0.35 s.
