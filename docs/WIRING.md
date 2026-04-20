# FireBot696 wiring reference

Authoritative source: [Fire_Bot.net](../Fire_Bot.net). Every pin below was
read from that netlist. KiCad project at [Fire_Bot/Fire_Bot.kicad_sch](../Fire_Bot/Fire_Bot.kicad_sch);
SVG export at [Fire_Bot_V2.svg](../Fire_Bot_V2.svg).

## Arduino Mega 2560 pin map

| Function | Mega pins | Notes |
|---|---|---|
| Left PWM (PWM1) | D10, D12 | shorted on PCB -- drive both with the same value |
| Left INA (INA1) | D38, D42 | shorted on PCB |
| Left INB (INB1) | D39, D43 | shorted on PCB |
| Right PWM (PWM2) | D11, D13 | shorted on PCB |
| Right INA (INA2) | D40, D44 | shorted on PCB |
| Right INB (INB2) | D41, D45 | shorted on PCB |
| Encoder Sensor A | D30, D32, D34, D36 | all four on one net -- aggregate only |
| Encoder Sensor B | D31, D33, D35, D37 | all four on one net -- aggregate only |
| A4988 STEP | D3 | INT1 capable; we drive it as a GPIO square wave |
| A4988 DIR | D4 | |
| Solenoid gate | D7 | through R1 to Q1 (NMOS); D1 is the flyback |
| HC-SR04 TRIG | D48 | |
| HC-SR04 ECHO | D47 | |
| MAX9814 + KY-032 /OUT | A0 and D46 | ONE shared net -- see below |
| Pi USB serial | D0 / D1 | native USB-serial bridge on the Mega |

Unused Mega pins per the netlist: `D0-D2` (reserved for serial), `D5`,
`D14-D21`, `D50-D53`, `A1-A15`, `AREF`.

## Three structural consequences

1. **Skid-steer, not full mecanum.** The PCB ties both DFR0601 channel-1
   motors together and both channel-2 motors together. Firmware mixes:
   - `left = vx + wz`
   - `right = vx - wz`
   - `vy` is accepted on `/cmd/drive` for API parity but ignored.
2. **Drive signals are physically shorted across pairs of Mega pins.** The
   firmware drives both pins of each pair simultaneously via
   `setRedundantPair` / `setDirPair` helpers in
   [firebot_mega.ino](../arduino/firebot_mega/firebot_mega.ino) so the two
   pins never contend.
3. **Aggregate encoders only.** All four FIT0186 Sensor A outputs are on one
   net and all four Sensor B outputs are on another, so the firmware reports
   `encA_total` and `encB_total` only, not four wheel counts. Per-wheel PID
   is out of scope. Note: D30-D37 are PORTC on the Mega2560, which does not
   support pin-change interrupts, so encoder edges are sampled in the main
   loop instead of via ISR. This is adequate for "wheels moving" sanity
   checks.

## Shared MAX9814 / KY-032 line

The netlist has one net (`/OUT`) tying MAX9814 analog envelope and KY-032
open-collector digital output together, with branches to both `A0` and `D46`
on the Mega. Firmware reads both every status tick:

- `digitalRead(D46) == LOW` -> KY-032 object detected (priority).
- `analogRead(A0)` -> mic envelope 0..1023 (valid only when IR is HIGH).

The brain node is expected to treat `/sensors/ir` as the priority signal on
that line. If this coupling becomes a problem, the fix is a jumper on the
physical board, not a firmware change.

## Known gaps

- No dedicated buzzer pin. `/cmd/warning` (`W,<mode>`) is accepted by the
  Mega and logged, but produces no physical output until a buzzer is added.
- `/EN` on KY-032 and `/GAIN`, `/AR` on MAX9814 are all floating (default
  device behavior).
