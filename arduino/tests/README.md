# Arduino test sketches

Self-contained sketches for bench-testing one subsystem at a time. They
all use the exact `Fire_Bot.net` pin map, so they work with the robot
wired as-is.

## Recommended focused tests

| Sketch | What it tests | Typical use |
|---|---|---|
| [drive_test/drive_test.ino](drive_test/drive_test.ino) | 4 motors via 2 DFR0601 drivers, skid-steer spin | "Do the wheels turn the right way?" |
| [extinguisher_test/extinguisher_test.ino](extinguisher_test/extinguisher_test.ino) | Solenoid pin-puller + A4988 lead-screw stepper | "Does the extinguisher mechanism discharge?" |

Both expose a short menu on the Serial Monitor (115200 baud, line
ending = Newline). Type `?` after flashing to see every command.

### drive_test quick reference

```text
f 120        # forward at PWM 120
b 120        # backward
l 100        # spin LEFT in place
r 100        # spin RIGHT in place
s            # stop
spin         # 2.5 s left, pause, 2.5 s right, stop
demo         # short forward/back/spin left/spin right cycle
speed 80     # change default pwm
```

### extinguisher_test quick reference

```text
pin on       # solenoid energize (pin pulled)
pin off      # release
pulse 1500   # pin on for 1500 ms then off
advance      # lead-screw forward 6 s (auto-stops)
retract      # lead-screw reverse 6 s (auto-stops)
stop         # halt the stepper now
seq          # full discharge: pin -> advance -> retract
abort        # cancel a running sequence
```

## Legacy reference sketches

These were the original bring-up demos. They are kept for reference but
use ad-hoc pin numbers, not the wired schematic -- **do not flash them
onto the robot without double-checking pins first.**

- `A4988_Motor_Driver_Test/`
- `FIT0186_DFR0601_Motor_Driver_Test/`
- `Remote_Control_Stepper_Motor/`
- `Solenoid/`
- `motor_test/`

## For the full system

For the actual robot (ROS2 + Pi + Mega), flash
[`arduino/firebot_mega/firebot_mega.ino`](../firebot_mega/firebot_mega.ino).
That sketch merges all of the above behaviors with the human-command
Serial Monitor interface described in
[`docs/ARDUINO_TESTING.md`](../../docs/ARDUINO_TESTING.md).
