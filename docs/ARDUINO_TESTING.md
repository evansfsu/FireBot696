# Arduino IDE Serial Monitor quick-test guide

This walks through testing the robot without the Raspberry Pi or any ROS
setup -- just the Arduino Mega, a USB cable, and the Arduino IDE. Any
hardware that isn't connected yet is skipped gracefully; you'll see `off`
for disabled sensors and the robot simply won't move for whatever isn't
wired. **The Mega can stay on the robot for all of this.**

Want an even smaller sketch that only tests one thing? There are two
focused single-purpose sketches next to the unified firmware:

- [`arduino/tests/drive_test/drive_test.ino`](../arduino/tests/drive_test/drive_test.ino)
  -- just the 4 drive motors and in-place spinning. Commands: `f`, `b`,
  `l`, `r`, `s`, `spin`, `demo`.
- [`arduino/tests/extinguisher_test/extinguisher_test.ino`](../arduino/tests/extinguisher_test/extinguisher_test.ino)
  -- just the solenoid + A4988 lead-screw. Commands: `pin on|off`,
  `pulse`, `advance`, `retract`, `stop`, `seq`.

Flash one of those when you only want to exercise the drivetrain or the
extinguisher without dealing with the state machine. Both use the same
`Fire_Bot.net` pin map as the unified firmware. See
[`arduino/tests/README.md`](../arduino/tests/README.md) for the full
reference.

For the "how does the Mega talk to the Pi, and what changes when I plug
them together" big picture, read
[INTEGRATION.md](INTEGRATION.md) first. The short version: the same
firmware works in both modes. Standalone mode is for bench/bring-up
testing; when the Pi opens `/dev/ttyACM0`, it just starts sending the
machine-protocol `M/E/W/S/R/C` commands and a 1 s drive watchdog kicks
in so a crashed Pi can't leave the motors running. Human commands typed
in the Serial Monitor are **not** watchdogged -- they stay latched
until you type `stop`.

## One-time setup

1. Open `arduino/firebot_mega/firebot_mega.ino` in the Arduino IDE.
2. Tools -> Board -> **Arduino Mega or Mega 2560**.
3. Tools -> Port -> whichever COM / tty the Mega enumerated on.
4. Click **Upload**.
5. Open **Serial Monitor** (magnifier icon, top-right).
6. In the Serial Monitor:
   - baud rate: **115200**
   - line ending: **Newline** (or "Both NL & CR")
7. You should see:
   ```
   L,firebot_mega_ready
   L,type 'help' for the Serial Monitor command menu
   ```

Type `help` and press Enter to see every command.

## One-word subsystem tests

If you just want to check "does this part work?" without typing a whole
sequence, use the canned tests. Each runs non-blocking; typing `stop` or
`test stop` cancels immediately.

```text
test motors          # forward 1s -> stop -> back 1s -> stop -> spin left -> stop -> spin right -> stop
test motors 200      # same, at PWM 200 (default is 150)
test solenoid        # on 0.8s -> off 0.8s -> on 0.8s -> off
test stepper         # advance 3s -> stop -> retract 3s -> stop
test all             # motors, then stepper, then solenoid
test stop            # cancel whatever's running
```

Each step prints a one-liner like `L,test_step motors[0] forward 1000ms` so
you can see what the Mega is doing, and `L,test_done=motors` when the
sequence finishes.

If a subsystem isn't wired up yet, the command still runs -- the outputs
just toggle into the void and the log lines still appear. That's useful for
sanity-checking the Mega side before the motors or solenoid are mounted.

## Typical bring-up session

Commands are case-insensitive and use spaces. Numbers are optional on the
motion shortcuts (they default to sane speeds).

```text
help               # print the full menu
status             # dump everything the Mega currently sees
forward 120        # drive forward at PWM 120
stop               # motors off
left 100           # spin left
right 100          # spin right
back 120           # reverse
drive 80 40        # vx=80, wz=40 (forward while turning right)
stop
```

## Extinguisher mechanism (solenoid + lead-screw)

```text
pin on             # energize the solenoid (pin-puller)
pin off            # release
advance            # run the A4988 stepper forward for 6 s (auto-stops)
retract            # reverse for 6 s
extstop            # halt the stepper immediately if needed
```

## Sensors (skip any that aren't wired)

Sensors are off at boot so a disconnected wire never latches false
readings. Enable only what you have:

```text
sensor us on       # HC-SR04 ultrasonic
sensor ir on       # KY-032 IR beam-break
sensor mic on      # MAX9814 microphone envelope
us                 # one-shot distance read
ir                 # one-shot IR read
mic                # one-shot mic envelope read
sensor us off      # turn it off again
```

Disabled sensors report `off` in `status` (and `-1` in the machine protocol
`D` line that the Pi eventually consumes).

### Live sensor readings in the Serial Monitor

For a rolling readout of a single sensor (e.g. walk up to a wall and
watch the distance drop), use `watch`:

```text
sensor us on
watch us            # prints one line per second: 'W,<ms> us=37cm'
watch us 250        # same, every 250 ms (minimum 50 ms)
watch all           # one combined line for every enabled sensor
watch off           # stop the periodic prints
```

Lines look like:

```text
W,12345 us=37cm
W,13345 us=35cm
W,14345 us=no_echo
W,15345 us=off            # sensor got disabled mid-watch
```

`watch` keeps running in the background while you send other commands,
so you can `watch us` and then do `forward 80` to see the distance drop
as the robot moves. `watch` and `verbose` are independent -- use
whichever is less noisy for what you're doing.

## Running the full brain state sequence on the Mega

The `state` commands mirror the Pi brain's seven-state machine on the Mega
so you can see every physical behavior without the Pi:

```text
state searching     # rotate in place
state awaiting      # stop and wait (simulated operator checkpoint)
state approaching   # drive forward
state warning       # 5 s countdown prints every second
                    # then AUTO-runs extinguishing (pin -> advance -> retract)
                    # then AUTO-enters 'complete' for 3 s
                    # then AUTO-returns to 'idle'
```

To step through it by hand instead of waiting for the auto-advances:

```text
state idle
state next          # -> searching
state next          # -> awaiting_confirm
state next          # -> approaching
state next          # -> warning (this one auto-runs to the end)
```

To cancel whatever state you're in:

```text
stop                # ends motion AND the state mirror
```

## Continuous status printing

If you want a live readout while you poke at things:

```text
verbose on
# ... drive / toggle / etc ...
verbose off
```

This prints the pretty `status` block every 500 ms.

## Emergency stop

```text
estop              # all outputs LOW, stepper disabled, firmware -> ESTOP
```

Any subsequent motion command clears the estop state.

## Useful notes for teammates

- You can mix human commands with the raw protocol in the same session. The
  Pi uses the compact form (`M,80,0,0`, `E,1`, `S`). Either works.
- The Mega never initiates messages on its own except on boot and in response
  to your commands, so the Serial Monitor should stay quiet unless `verbose
  on` is active.
- If you flash firmware while the Pi is running, close the Pi side first --
  both can't own the serial port at the same time.
- Typing a command the firmware doesn't recognize prints
  `L,err unknown command: <what you typed>` followed by a hint. Typing
  nothing is fine.
