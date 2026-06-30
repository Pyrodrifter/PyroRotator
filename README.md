# PyroRotator

ESP32 Az/El antenna-rotator firmware — PJ4 · PyroLabs.

Open-loop NEMA 17 steppers via TMC2209 (standalone, StealthChop2 + 1/256 MicroPlyer
for silent, smooth tracking), limit-switch homing on elevation, compass-set azimuth
zero. Designed as the hardware end of [SkyPhreak](https://github.com/Pyrodrifter/SkyPhreak)'s
continuous-motion **SuperRot** protocol, but also speaks rotctld and EasyComm.

## Control surfaces
- **Web app** — `http://rotator.local/` (manual control, status, protocol toggle).
- **TCP `:4533`** — `rotctld` (Hamlib net-rotctl) **or** **SuperRot** continuous motion
  (`A/V/P/S/K/?`), selectable from the web app. See [SUPERROT.md](SUPERROT.md).
- **USB serial @ 9600** — EasyComm II (e.g. SkyRoof via `rotctld -m 202`).

## Build
- Board: **ESP32 Dev Module** (Arduino IDE / arduino-cli).
- Libraries: **AccelStepper** (everything else is ESP32 core).
- Edit the CONFIG block at the top of `PyroRotator.ino`: WiFi creds, gear ratios,
  microstepping (match the CNC-shield MS jumpers), pins, travel limits, homing.

## Files
- `PyroRotator.ino` — firmware.
- `index_html.h` — embedded web UI (kept out of the `.ino` so Arduino's preprocessor
  doesn't choke on the raw-string HTML).
- `SUPERROT.md` — the SuperRot continuous-motion protocol spec.

## Motion notes
- Azimuth travel `0–450°` (90° overlap past a full turn). The host streams **continuous**
  (unwrapped) azimuth and owns trajectory continuity; the firmware positions
  absolutely and does **not** wrap. Unwind the cable manually when wound near the limit.
- Two-stage homing: fast seek → back off → slow gentle re-approach, so contact is soft
  and repeatable (no hard press / skipped step). Homing aborts (no false zero) if the
  switch isn't found.
