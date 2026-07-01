# Code Review — `master..Development`

Review of the Development branch relative to `master` (11 commits: UM982/ByNav
GPS support, tool XTE steering, board label). Findings ranked most-severe first.
Status: **documented only, not yet fixed.**

---

## 1. Actuator end-stop protection ignores the invert flags (safety)
**File:** `AutoSteerTeensy/Toolsteer.ino` — `motorDrive()`

```c
if (toolSettings.invertActuator) pwmDrive *= -1;      // sign flipped here
if (fabsf(actuatorPositionPercent) > toolSettings.maxActuatorLimit) {
    if (actuatorPositionPercent > 0 && pwmDrive > 0) pwmDrive = 0;
    if (actuatorPositionPercent < 0 && pwmDrive < 0) pwmDrive = 0;
}
```

The limit check assumes `+pwmDrive` drives the actuator toward
`+actuatorPositionPercent`. But `pwmDrive` was just inverted by `invertActuator`,
and the position sign is independently inverted by `invertAPOS`. If the two
conventions disagree, the block zeros the *retracting* command and lets the
command that drives *further past the limit* through — the hard stop protects the
wrong direction.

**Failure:** with `invertActuator=1`, actuator at +65% (limit 60), the command
that pushes further out is `pwmDrive<0`, which is not zeroed → actuator drives
past its mechanical limit.

---

## 2. Bang-bang valve timing tied to PGN-233 arrival rate, not loop time
**Files:** `AutoSteerTeensy/Toolsteer.ino` — `BangBangDrive()`;
`AutoSteerTeensy/ToolComm.ino` — case 233

`ValveCounter` is incremented only in the PGN-233 handler (`ValveCounter++`), yet
`BangBangDrive()` compares it against `valveOnTime`/`valveOffTime` as if they were
25 ms loop ticks (comment: "5 × 25ms = 125ms"). PGN 233 arrives at the AgIO/tool
send rate (~10 Hz), not the 40 Hz loop rate, so the real pulse period is ~2–3×
longer than documented and shifts if the message rate changes.

**Failure:** operator configures on/off expecting 125 ms / 375 ms; actual pulse is
~500 ms / 1000 ms → sluggish valve response that doesn't match the set values.

---

## 3. Bang-bang leaves `pwmDrive` stale on the reset tick
**File:** `AutoSteerTeensy/Toolsteer.ino` — `BangBangDrive()`

```c
if (ValveCounter < valveOnTime)       pwmDrive = 255 * dir;
else if (ValveCounter < valveOffTime) pwmDrive = 0;
else                                  ValveCounter = 0;   // pwmDrive not assigned
```

On the tick where `ValveCounter >= valveOffTime`, no branch assigns `pwmDrive`, so
it carries the previous loop's value. Benign with defaults (previous value is 0),
but if `valveOffTime <= valveOnTime` the reset tick re-emits full `255*dir` and the
pulse never de-energizes.

**Failure:** misordered on/off settings → valve stays fully energized.

---

## 4. `fixQuality` passed through raw from KSXT
**File:** `AutoSteerTeensy/ByNav.ino` — `ParseKSXT()`

```c
fixQuality[0] = f[9][0]; fixQuality[1] = '\0';   // may need remap for RTK display
```

KSXT position-status codes are not the same enumeration as NMEA GGA fix-quality
(where 4 = RTK fixed, 5 = RTK float). The KSXT digit is copied straight into the
`$PANDA` fix-quality field (the code comment already flags this).

**Failure:** AgIO shows the wrong fix status / doesn't recognize RTK-fixed, which
can degrade guidance status even with a fixed solution.

---

## 5. `ProportionalDrive()` divides by operator-supplied `lowHighDistance`
**File:** `AutoSteerTeensy/Toolsteer.ino` — `ProportionalDrive()`

```c
float lowHighPerCM = (float)(toolSettings.highPWM - toolSettings.lowPWM)
                     / toolSettings.lowHighDistance;
```

`lowHighDistance` comes straight from PGN 232 `Data[11]` with no floor. At `0` this
is a divide-by-zero (→ `inf`). Today the result is only consumed inside
`if (errorAbs < lowHighDistance)` (false when 0), so it doesn't propagate — but it
is a latent trap if that guard is refactored, and in `BangBangDrive()` a `0` here
silently disables all pulsing (always "large error → full drive").

**Failure:** operator sets low/high distance to 0 → degenerate control; latent
divide-by-zero. Suggested: clamp to `max(1, …)` on ingest in PGN 232.

---

## Notes (not findings)
- `Panda.ino`'s diff is ~90% whitespace (tab/space) churn plus the mechanical
  `IMU_* → ATT_*` rename; the only behavioral change is the added `KSXT_Feed(c)`.
- PGNs 232/233 don't validate CRC, unlike config PGN 251 — consistent with the
  existing AgIO steer/hello PGNs, so not flagged, but it is a trust boundary.
- `ByNavConfig()` blocks ~10 s at startup (`SAVECONFIG` + `REBOOT` delays) but only
  on first-time config; acceptable for setup.

**Priority:** #1 (safety inversion) and #2 (valve timing vs. documented values)
before merging to `master`.
