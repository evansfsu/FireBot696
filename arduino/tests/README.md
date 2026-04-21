# Arduino test sketches

Self-contained single-subsystem sketches. All use the `Fire_Bot.net`
pin map, so they work on the robot as wired.

## Focused tests

| Sketch | Tests |
|---|---|
| [drive_test/drive_test.ino](drive_test/drive_test.ino) | 4 motors via 2 DFR0601 drivers, skid-steer spin |
| [extinguisher_test/extinguisher_test.ino](extinguisher_test/extinguisher_test.ino) | Solenoid + A4988 lead-screw |

Serial Monitor at 115200 / Newline. Type `?` for the menu.

### drive_test

```text
f 120        forward
b 120        backward
l 100        spin left
r 100        spin right
s            stop
spin         2.5 s left, pause, 2.5 s right, stop
demo         short fwd/back + full L/R spin
speed 80     change default PWM
```

### extinguisher_test

```text
pin on       solenoid energize
pin off      release
pulse 1500   pin on for 1500 ms then off
advance      lead-screw forward 6 s (auto-stops)
retract      lead-screw reverse 6 s (auto-stops)
stop         halt the stepper now
seq          full discharge: pin -> advance -> retract
abort        cancel seq mid-run
```

## Legacy sketches

Original bring-up demos. Kept for reference. **They use ad-hoc pin
numbers, not the wired schematic -- don't flash them without checking
pins first.**

- `A4988_Motor_Driver_Test/`
- `FIT0186_DFR0601_Motor_Driver_Test/`
- `Remote_Control_Stepper_Motor/`
- `Solenoid/`
- `motor_test/`

## Full system

For the real robot (ROS2 + Pi + Mega), flash
[`arduino/firebot_mega/firebot_mega.ino`](../firebot_mega/firebot_mega.ino).
It merges every behavior above with the Serial Monitor command set
described in [`docs/ARDUINO_TESTING.md`](../../docs/ARDUINO_TESTING.md).
