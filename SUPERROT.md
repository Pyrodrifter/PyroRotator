# SuperRot Protocol

**Version 1.0** · A continuous-motion control protocol for az/el antenna rotators.

SuperRot is a small ASCII line protocol for driving a two-axis (azimuth/elevation)
rotator. It exists because the usual rotator protocol — Hamlib's `rotctld` — only
carries *position*: every update means "go to this point and **stop**." During
satellite or astrophotography tracking the host sends a fresh position roughly once
per second, so the rotator lurches: accelerate, arrive, stop, repeat. That start-stop
sawtooth is visible in the image and audible in the motors.

SuperRot instead carries **position *and* angular velocity** in a single setpoint. The
host streams these continuously and the firmware always keeps the axes moving toward a
*moving* target, so the motion is smooth and never fully stops between updates. It also
streams telemetry back so the host can close its tracking loop.

SuperRot is a **superset** of the position-only model: it still accepts a plain
go-to-and-stop command (`P`), so it degrades gracefully for clients that don't do
velocity planning.

---

## 1. Transport

SuperRot is transport-agnostic. The same commands work over either:

| Transport | Typical use | Notes |
|-----------|-------------|-------|
| **TCP** | ESP32 / networked controller | Listen on port **4533** by convention (shared with rotctld). One client at a time is sufficient. |
| **USB serial** | Directly-attached MCU | **115200** baud, 8N1. |

The framing and command set are identical on both. A controller may offer both
simultaneously.

> **Port 4533 note:** rotctld also uses 4533. A controller that speaks both should pick
> the active protocol explicitly (e.g. a config toggle), because the two share command
> letters (`P`, `S`) with different reply formats.

---

## 2. Framing

- Every command and every reply is a single line terminated by `\n` (LF).
  `\r` (CR) is tolerated and ignored — accept `\n`, `\r`, or `\r\n`.
- Fields within a line are separated by one or more spaces.
- All values are decimal ASCII. The leading character of a line is the **command
  letter**; parameters follow after a space.
- Angles are **degrees**. Rates are **degrees per second**. Azimuth is `0..360`
  (controllers may allow overlap past 360 for cable unwind); elevation is `0..90` or
  `0..180` depending on the build.
- Recommended numeric precision on the wire: positions to 2 decimals, rates to 3.
  Parsers must accept any reasonable float.

---

## 3. Commands (host → rotator)

| Command | Form | Meaning |
|---------|------|---------|
| **Track** | `A <az> <el> <azRate> <elRate>` | Primary command. Setpoint = target position **plus** feedforward angular velocity. The axis drives toward `<az>/<el>` at a speed derived from `<azRate>/<elRate>`. Stream these continuously (typically 1–10 Hz) for smooth motion. |
| **Velocity** | `V <azRate> <elRate>` | Pure velocity mode (°/s). No target position; the axis just slews at the commanded rate until the next command or a soft limit. Useful for manual nudging / joystick control. |
| **Goto** | `P <az> <el>` | rotctld-compatible: move to a position and stop on arrival. For one-shot moves or non-tracking clients. |
| **Stop** | `S` | Decelerate to a stop and hold. |
| **Park** | `K` | Move to the configured park position. |
| **Query** | `?` | Request one telemetry line immediately (see §5). |

### Notes on the `A` command (the important one)

The feedforward rate is what makes motion smooth. The firmware uses it to set the
axis's speed cap so the axis *arrives in step with the host's prediction* rather than
racing to the point and stopping. Implementations should:

- Keep a small **minimum speed floor** so that when the rate is ~0 the axis can still
  trim small position errors instead of stalling.
- Clamp the rate to a firmware **maximum angular velocity** for safety.
- Treat each `A` as the current setpoint — there is no queue. A new `A` supersedes the
  previous one immediately.

If the host stops sending `A` commands, a position-mode firmware will simply arrive at
the last target and stop, which is safe.

---

## 4. Replies (rotator → host)

Command acknowledgements are single lines:

| Reply | Meaning |
|-------|---------|
| `OK` | Command accepted. |
| `ERR <message>` | Command rejected; `<message>` is human-readable (e.g. `ERR unknown`). |

`A`, `V`, `P`, `S`, `K` each return `OK` (or `ERR …`). `?` returns no ack — it triggers
a telemetry line instead.

> A minimal firmware may reply `OK` to commands it doesn't fully implement, as long as
> it doesn't move unsafely. Hosts should not depend on rich error text.

---

## 5. Telemetry (rotator → host)

The rotator streams its live state so the host can close the tracking loop:

```
T <az> <el> <azRate> <elRate>
```

- `<az> <el>`   — current measured/estimated position, degrees
- `<azRate> <elRate>` — current angular velocity, °/s (signed; sign = direction)

**Cadence:** stream continuously at **~10 Hz** (every ~100 ms) while a client is
connected. Also send one `T` line in response to a `?` query.

Telemetry and acknowledgements share the same line channel; a host parser distinguishes
them by the leading letter (`T` = telemetry, anything else = reply). A host should
ignore lines it does not recognise.

---

## 6. Worked example

Host tracks a pass, streaming setpoints at 5 Hz, while reading back telemetry:

```
→ A 142.50 31.20 1.850 0.640      (go toward 142.5°/31.2°, moving +1.85°/s az, +0.64°/s el)
← OK
← T 141.10 30.85 1.840 0.633      (telemetry, ~10 Hz, independent of commands)
← T 141.47 30.97 1.845 0.638
→ A 142.90 31.33 1.850 0.640
← OK
← T 141.85 31.06 1.847 0.641
...
→ S                                (end of pass)
← OK
```

Manual slew with the velocity command:

```
→ V 3.0 0.0      (slew azimuth +3°/s, elevation hold)
← OK
→ V 0.0 0.0      (stop slewing)
← OK
```

---

## 7. Implementing a rotator (firmware side)

You need: a line reader, a command dispatcher, a motion layer, and a 10 Hz telemetry
timer. Pseudocode for the dispatcher:

```
on_line(line):
    cmd = line[0]
    a, b, c, d = parse_floats(line[1:])   # missing fields default to 0
    switch cmd:
        'A': track(a, b, c, d); send("OK")
        'V': velocity(a, b);    send("OK")
        'P': goto(a, b);        send("OK")
        'S': stop();            send("OK")
        'K': park();            send("OK")
        '?': /* telemetry timer handles it */
        else: send("ERR unknown")

every 100 ms:
    send("T %.2f %.2f %.3f %.3f" % (az, el, azRate, elRate))
```

### Mapping `track()` onto a stepper library

The trajectory planning lives on the **host**; firmware just does smooth, ramped step
generation. Two common approaches:

- **Hardware-timed (e.g. FastAccelStepper + TMC2209):** set the axis speed from the
  feedforward rate, then `moveTo(target)`. Because the host streams continuously, the
  target keeps moving and the axis never decelerates to a halt. TMC2209 `intpol` +
  StealthChop give near-silent, micro-smooth motion.

- **Software-stepped (e.g. AccelStepper):** identical idea — `setMaxSpeed(|rate| ×
  steps_per_deg)` (with a small floor and a hard cap), then `moveTo(target)`, and call
  `run()` in the main loop. Slightly less smooth than hardware timing but the protocol
  is identical.

`velocity()` is the same trick with the target projected toward the soft limit in the
rate's direction. `stop()` decelerates and holds. Always clamp targets to your soft
limits.

---

## 8. Implementing a host (controller side)

1. Open the transport (TCP to `<ip>:4533` or serial @ 115200).
2. Read lines; split on whitespace. Leading `T` → parse telemetry
   `{az, el, azRate, elRate}`; anything else → treat as a reply (`OK` / `ERR …`).
3. From your orbit/trajectory model, compute position **and** its time-derivative
   (velocity) and stream `A az el azRate elRate` at 1–10 Hz.
4. Use `P` for slews to a fixed point, `V` for manual joystick control, `S` to stop,
   `K` to park.

A reference client (`SuperRotClient`, Node.js, TCP + serial) ships with PyroSatTrack in
`electron/superrot.cjs`. It exposes `track()`, `velocity()`, `goto()`, `stopMotion()`,
and `park()`, and emits parsed telemetry events.

---

## 9. Quick reference card

```
Host → Rotator                         Rotator → Host
  A az el azRate elRate   track          OK
  V azRate elRate         velocity       ERR <msg>
  P az el                 goto/stop      T az el azRate elRate   (~10 Hz)
  S                       stop
  K                       park
  ?                       query telemetry

Transport : TCP :4533  or  serial 115200 8N1
Framing   : one line, '\n' terminated, space-separated
Units     : degrees, degrees/second
```
