# Arduino Serial Monitor reference

For the big picture of how the Mega and Pi interact, see
[INTEGRATION.md](INTEGRATION.md). This page is just the command list for
`arduino/firebot_mega/firebot_mega.ino` when you're driving it by hand.

For single-subsystem sketches with a smaller command set, see
`arduino/tests/drive_test/` and `arduino/tests/extinguisher_test/`
(details in [../arduino/tests/README.md](../arduino/tests/README.md)).

Serial Monitor: **115200 baud, line ending = Newline**. On boot the
Mega prints:

```
L,firebot_mega_ready
L,type 'help' for the Serial Monitor command menu
```

Human commands are latched until you cancel them (type `stop`). The
Pi-side protocol (`M,E,W,S,R,C`) *is* watchdogged at 1 s so a crashed
bridge can't leave the motors running.

## Subsystem tests

Non-blocking. `stop` or `test stop` cancels.

```text
test motors          fwd 1s -> stop -> back 1s -> stop -> spin L -> spin R -> stop
test motors 200      same, PWM 200 (default 150)
test solenoid        on 0.8s -> off 0.8s -> on 0.8s -> off
test stepper         advance 3s -> stop -> retract 3s -> stop
test all             motors, stepper, solenoid in sequence
test stop            cancel
```

Each step prints a line like `L,test_step motors[0] forward 1000ms`.
Missing hardware is fine -- the pins toggle either way, useful for
sanity-checking the firmware side before parts are mounted.

## Motion

Case-insensitive, space-separated. Numbers are optional (default to
sane speeds).

```text
forward 120
back 120
left 100
right 100
drive 80 40          vx=80, wz=40 (forward + turning right)
stop
```

## Extinguisher

```text
pin on               solenoid energize
pin off              release
advance              A4988 forward 6 s (auto-stops)
retract              A4988 reverse 6 s
extstop              halt the stepper
```

## Sensors

Off at boot so unwired pins don't latch noise.

```text
sensor us on         HC-SR04
sensor ir on         KY-032
sensor mic on        MAX9814
us                   one-shot read
ir
mic
sensor us off
```

Disabled sensors report `off` in `status` and `-1` in the `D` line to
the Pi.

### Live sensor readout

```text
sensor us on
watch us             one line per second: 'W,<ms> us=37cm'
watch us 250         every 250 ms (min 50)
watch all            one combined line per tick for every enabled sensor
watch off            stop
```

Sample lines:

```text
W,12345 us=37cm
W,13345 us=35cm
W,14345 us=no_echo
W,15345 us=off
```

`watch` keeps printing while other commands run, so you can
`watch us` then `forward 80` and see the distance drop.

## State machine mirror

Same seven states as the Pi brain, on the Mega alone:

```text
state searching      rotate in place
state awaiting       stop and wait (simulated operator checkpoint)
state approaching    short-pulse forward drive
state warning        5 s countdown, then auto-extinguish,
                     then auto-complete, then auto-idle
```

Step through manually:

```text
state idle
state next           -> searching
state next           -> awaiting_confirm
state next           -> approaching
state next           -> warning (auto-runs to the end)
```

`stop` exits the state mirror along with any motion.

## Verbose / status

```text
status               one-shot status dump
verbose on           pretty status every 500 ms
verbose off
```

## E-stop

```text
estop                all outputs LOW, firmware -> FW_ESTOP
```

Any subsequent motion command clears it.

## Notes

- Protocol commands (`M,80,0,0`, `E,1`, `S`) and human commands work in
  the same session.
- The Mega is silent unless it's booting, responding to you, or
  `verbose on` is active.
- Close the Pi side before reflashing -- only one process can own the
  serial port.
- Unknown commands print `L,err unknown command: <text>` with a hint.
