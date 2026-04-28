# Pi <-> Mega serial protocol

USB-CDC on the Mega's native serial (D0/D1), 115200 baud, ASCII, `\n`
terminated. The bridge sends commands as they arrive from ROS topics
plus an `S` status request on a timer. The Mega only initiates
messages for `L,<log>` lines (boot, estop, errors).

**Command map:** see [COMMANDS.md](COMMANDS.md) for machine vs human vs Pi script.

Machine lines start with a single **uppercase** letter followed by nothing or a comma (see parser in `firebot_mega.ino`). Anything else (e.g. `go`, `help`, `forward 75`) is handled as a **human Serial Monitor** command — useful for bench tests from `scripts/rpi_test_arduino_serial.py` without adding a new protocol letter.

## Pi -> Mega

| Command | Args | Meaning |
|---|---|---|
| `M,vx,vy,wz` | ints -255..255 | drive. `vy` ignored (skid-steer). Mega computes `left = vx + wz`, `right = vx - wz`. |
| `E,phase` | 0..4 | extinguisher: `0` off, `1` pin-pull, `2` advance, `3` retract, `4` stop stepper. Phases `2` and `3` run the lead screw for **~5.3 s** each (`EXT_STEPPER_RUN_MS`), with the same ramp as the bench sketch, then the firmware stops that motion. |
| `W,mode` | 0..2 | warning: `0` off, `1` steady, `2` countdown. No physical output yet (no buzzer). |
| `S` | -- | request status reply. |
| `R` | -- | e-stop. All outputs LOW, firmware -> `ESTOP`. |
| `C,<US\|IR\|MIC>,<0\|1>` | sensor + enable | turn an optional sensor on or off. Disabled sensors report `-1`. |

## Mega -> Pi

| Line | Fields | Meaning |
|---|---|---|
| `D,encA,encB,us,ir,mic,state` | ints; sensors `-1` when off; `state` is firmware FSM (0=IDLE, 1=DRIVING, 2=EXTINGUISHING, 3=ESTOP) | reply to `S`. |
| `L,<message>` | free text | firmware log (boot banner, estop ack, `W` ack, etc.). |

## Example session

```
Pi -> Mega: C,US,1
Pi -> Mega: C,IR,0
Pi -> Mega: C,MIC,0
Pi -> Mega: S
Mega -> Pi: D,0,0,42,-1,-1,0
Pi -> Mega: M,80,0,0
Pi -> Mega: S
Mega -> Pi: D,134,127,41,-1,-1,1
Pi -> Mega: E,1
Pi -> Mega: E,2
Pi -> Mega: S
Mega -> Pi: D,134,127,41,-1,-1,2
Pi -> Mega: R
Mega -> Pi: L,estop
```

## Reliability

- Bridge keeps a single `serial.Serial` open for the life of the
  process, 50 ms read timeout. A missed line doesn't back up state --
  the next `S` tick replaces whatever was stale.
- No sequence numbers. The brain re-publishes `/cmd/drive` every
  control cycle at 10 Hz, so a dropped command self-heals in one
  tick.
- Opening the serial port triggers a Mega auto-reset, so the bridge
  sleeps 2 s after `Serial()` before sending config commands.

## Drive watchdog (protocol `M` only)

If the last **non-zero** `M,vx,vy,wz` came from the protocol path (`g_protocol_drive` armed), and no new `M` arrives for **~1 s** (`PROTOCOL_DRIVE_WATCHDOG_MS`), the Mega zeros the motors. Custom scripts that hold a pose longer than that must **repeat `M` at least every second** (see `scripts/rpi_test_arduino_serial.py` `motor_hold()`). Human Serial Monitor commands (`forward`, `left`, …) do **not** arm this watchdog.

## Non-protocol lines: `go` and friends

The firmware accepts the same plain-text commands as the Serial Monitor on the same port. Example: send ASCII `go\n` for the full bench solenoid + lead-screw sequence (timed like `arduino/tested_arduino/solenoid_stepper_combined/`). For a minimal Pi-side harness without ROS, use [scripts/rpi_test_arduino_serial.py](../scripts/rpi_test_arduino_serial.py).
