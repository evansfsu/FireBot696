# Wiring reference

**Runtime pin map** for motors, stepper, and solenoid is defined in
[firebot_mega.ino](../arduino/firebot_mega/firebot_mega.ino) and matches
[arduino/tested_arduino](../arduino/tested_arduino) (see **Bench-tested
harness** below). The KiCad netlist remains the electrical drawing
reference.

KiCad: [Fire_Bot/Fire_Bot.kicad_sch](../Fire_Bot/Fire_Bot.kicad_sch); netlist
[Fire_Bot.net](../Fire_Bot.net); SVG [Fire_Bot_V2.svg](../Fire_Bot_V2.svg).

## Bench-tested harness (what `firebot_mega.ino` uses)

**Use this table for the current firmware**, copied from
[arduino/tested_arduino/front_reverse_spin](arduino/tested_arduino/front_reverse_spin/front_reverse_spin.ino)
and
[arduino/tested_arduino/solenoid_stepper_combined](arduino/tested_arduino/solenoid_stepper_combined/solenoid_stepper_combined.ino).
If you move wires on the breadboard or PCB, update the `#define` /
`const uint8_t` block at the top of
[firebot_mega.ino](../arduino/firebot_mega/firebot_mega.ino) to match.

| Function | Pins | Notes |
|---|---|---|
| Front-right PWM / INA / INB | D12 (`FP1`), D51 (`FA1`), D49 (`FB1`) | |
| Front-left PWM / INA / INB | D13 (`FP2`), D52 (`FA2`), D50 (`FB2`) | |
| Rear-left PWM / INA / INB | D10 (`RP1`), D25 (`RA1`), D27 (`RB1`) | |
| Rear-right PWM / INA / INB | D11 (`RP2`), D24 (`RA2`), D26 (`RB2`) | |
| A4988 STEP | D8 | |
| A4988 DIR | D9 | advance = LOW, retract = HIGH (matches tested sketch) |
| Solenoid (pin pull) | D4 | |
| Encoder Sensor A | D30, D32, D34, D36 | unchanged from netlist -- aggregate |
| Encoder Sensor B | D31, D33, D35, D37 | |
| HC-SR04 TRIG | D48 | |
| HC-SR04 ECHO | D47 | |
| MAX9814 + KY-032 /OUT | A0, D46 | unchanged |
| Pi USB serial | D0 / D1 | |

Skid-steer mixing (`left = vx + wz`, `right = vx - wz`) still applies; the
firmware sets **per-corner** direction bits using the same patterns as the
tested `moveForward` / `moveReverse` / `spinInPlace` routines, then applies
`abs(left)` to `FP2`+`RP1` and `abs(right)` to `FP1`+`RP2`.

---

## Schematic netlist (legacy PCB reference)

The following table reflects **Fire_Bot.net** and older revisions where each
logical signal was routed to **two** Mega pins (shorted pairs). The
production `firebot_mega.ino` does **not** use this map for motors,
stepper, or solenoid — keep it for KiCad / bring-up comparison only.

Source of truth for the drawing: [Fire_Bot.net](../Fire_Bot.net). KiCad at
[Fire_Bot/Fire_Bot.kicad_sch](../Fire_Bot/Fire_Bot.kicad_sch).

| Function | Pins | Notes |
|---|---|---|
| Left PWM (PWM1) | D10, D12 | shorted on PCB -- drive both with same value |
| Left INA | D38, D42 | shorted |
| Left INB | D39, D43 | shorted |
| Right PWM (PWM2) | D11, D13 | shorted |
| Right INA | D40, D44 | shorted |
| Right INB | D41, D45 | shorted |
| Encoder Sensor A | D30, D32, D34, D36 | one shared net -- aggregate only |
| Encoder Sensor B | D31, D33, D35, D37 | one shared net -- aggregate only |
| A4988 STEP (netlist) | D3 | **not** used by current bench firmware (see D8 above) |
| A4988 DIR (netlist) | D4 | **Conflicts** with harness solenoid pin -- schematic vs bench |
| Solenoid gate (netlist) | D7 | **not** used by current bench firmware (see D4 above) |
| HC-SR04 TRIG | D48 | |
| HC-SR04 ECHO | D47 | |
| MAX9814 + KY-032 /OUT | A0, D46 | one shared net -- see below |
| Pi USB serial | D0 / D1 | via the Mega's USB-serial bridge |

Unused per the netlist: D0-D2 (serial), D5, D14-D21, D50-D53, A1-A15,
AREF.

## Consequences (legacy schematic)

1. **Skid-steer, not mecanum.** Both DFR0601 channel-1 motors are
   ganged (left pair) and both channel-2 motors are ganged (right
   pair). Firmware mixes:
   - `left  = vx + wz`
   - `right = vx - wz`
   - `vy` is accepted on `/cmd/drive` for API parity and ignored.
2. **Legacy PCB:** Drive signals were physically shorted across pin pairs.
   **Bench firmware** instead drives four independent motor channels on
   D10–D13 / D24–D27 / D49–D52 as tabulated above.
3. **Aggregate encoders.** All four Sensor A outputs on one net, all
   four Sensor B outputs on another. Firmware reports `encA_total` /
   `encB_total`; no per-wheel PID. D30-D37 are PORTC, which doesn't
   support pin-change interrupts, so encoder edges are sampled in the
   main loop. Fine for "wheels moving" sanity checks.

## Shared MAX9814 / KY-032 line

The `/OUT` net ties MAX9814 analog envelope and KY-032 open-collector
digital output, branching to both A0 and D46. Firmware reads both each
status tick:

- `digitalRead(D46) == LOW` -> KY-032 detected object (priority).
- `analogRead(A0)` -> mic envelope 0..1023 (only meaningful when IR is
  HIGH).

Brain treats `/sensors/ir` as priority on that line. If this coupling
ever gets in the way, the fix is a jumper on the board, not a firmware
change.

## Known gaps

- No buzzer pin. `/cmd/warning` (`W,<mode>`) is accepted and logged by
  the Mega but produces no physical output.
- KY-032 `/EN` and MAX9814 `/GAIN`, `/AR` are all floating (default
  device behavior).
