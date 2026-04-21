# Wiring reference

Source of truth: [Fire_Bot.net](../Fire_Bot.net). Every pin below was
read from that netlist. KiCad project at
[Fire_Bot/Fire_Bot.kicad_sch](../Fire_Bot/Fire_Bot.kicad_sch); SVG
export at [Fire_Bot_V2.svg](../Fire_Bot_V2.svg).

## Mega 2560 pin map

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
| A4988 STEP | D3 | driven as GPIO square wave |
| A4988 DIR | D4 | |
| Solenoid gate | D7 | through R1 to Q1 (NMOS); D1 is the flyback |
| HC-SR04 TRIG | D48 | |
| HC-SR04 ECHO | D47 | |
| MAX9814 + KY-032 /OUT | A0, D46 | one shared net -- see below |
| Pi USB serial | D0 / D1 | via the Mega's USB-serial bridge |

Unused per the netlist: D0-D2 (serial), D5, D14-D21, D50-D53, A1-A15,
AREF.

## Consequences

1. **Skid-steer, not mecanum.** Both DFR0601 channel-1 motors are
   ganged (left pair) and both channel-2 motors are ganged (right
   pair). Firmware mixes:
   - `left  = vx + wz`
   - `right = vx - wz`
   - `vy` is accepted on `/cmd/drive` for API parity and ignored.
2. **Drive signals are physically shorted across pin pairs.** Firmware
   drives both pins of each pair simultaneously via `setRedundantPair`
   / `setDirPair` in
   [firebot_mega.ino](../arduino/firebot_mega/firebot_mega.ino) so
   they never contend.
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
