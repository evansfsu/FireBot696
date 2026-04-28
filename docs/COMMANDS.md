# Command guide (Mega + Pi)

This page is the map. Details live in the linked docs.

| You are using… | Doc / tool |
|---|---|
| **Pi or PC → machine protocol** (`M`, `E`, `S`, `R`, `C`) | [PROTOCOL.md](PROTOCOL.md) |
| **Arduino IDE Serial Monitor** (plain English) | [ARDUINO_TESTING.md](ARDUINO_TESTING.md) |
| **Pi without ROS** (USB test script) | [scripts/rpi_test_arduino_serial.py](../scripts/rpi_test_arduino_serial.py) + section below |
| **Pi with ROS / Docker** | [RPI5_COMMANDS.md](RPI5_COMMANDS.md), [INTEGRATION.md](INTEGRATION.md) |

Serial on the Mega is always **115200 baud**, lines end with **`\n`**. Opening the port from the Pi resets the Mega (DTR); wait **~2 s** before sending commands.

---

## 1. Machine protocol (Pi / script / any serial terminal)

Uppercase letter + comma forms are treated as protocol. Summary:

| Send | Role |
|---|---|
| `M,vx,vy,wz` | Skid steer, integers −255…255 (`vy` ignored). |
| `E,phase` | `0` off, `1` pin-pull solenoid, `2` lead-screw advance, `3` retract, `4` stop stepper. |
| `S` | Request status → Mega replies `D,...`. |
| `R` | E-stop: all outputs low. |
| `W,mode` | Warning echo only (no buzzer yet). |
| `C,...` | Sensor on/off: `C,US,1` / `C,IR,0` / `C,MIC,0` (see PROTOCOL.md). |

**Drive watchdog:** If the Mega last saw motion from a **protocol** `M,...` with non-zero command, it stops the motors if no new `M` arrives for about **1 s**. Long moves from a script must **re-send `M` every &lt;1 s** (the repo script does this).

**Lead screw:** `E,2` and `E,3` each run **about 5.3 s** in firmware (`EXT_STEPPER_RUN_MS`), then the Mega can move to a stop phase. You can still send `E,0` afterward for a clean idle.

**Human-style lines:** Anything that does **not** look like protocol (e.g. lowercase `go`, `help`) is parsed as [Serial Monitor commands](ARDUINO_TESTING.md). So the Pi can send `go\n` for the full bench solenoid + stepper sequence.

Full grammar and examples: [PROTOCOL.md](PROTOCOL.md).

---

## 2. Serial Monitor (human) commands

Typed in the Arduino IDE with **line ending: Newline**. Not watchdogged the same way as the Pi; motion stays until `stop` (unless a scripted `go` or extinguisher phase is running).

Minimum set:

```text
help              full menu
forward 75        default speed 75 if you omit the number
left 75 / right 75
stop
go                full bench sequence (solenoid + stepper timing)
advance / retract ~5.3 s lead-screw run each
test motors       canned cycle (default test PWM 75)
estop
```

Full list: [ARDUINO_TESTING.md](ARDUINO_TESTING.md).

---

## 3. Pi: `scripts/rpi_test_arduino_serial.py`

Requires `pip install pyserial` (or `python3-serial` from apt). Robot **on blocks** for motor tests.

```bash
cd ~/FireBot696

# Optional: see lines without USB
python3 scripts/rpi_test_arduino_serial.py smoke --dry-run

# Default USB port /dev/ttyACM0 — override with --port
python3 scripts/rpi_test_arduino_serial.py smoke              # short spin + estop
python3 scripts/rpi_test_arduino_serial.py spin --ms 4000 --wz 40
python3 scripts/rpi_test_arduino_serial.py drive --vx 60 --wz 0 --ms 2000
python3 scripts/rpi_test_arduino_serial.py advance             # E,2 ~5.3 s, then E,0
python3 scripts/rpi_test_arduino_serial.py retract
python3 scripts/rpi_test_arduino_serial.py go --wait-s 30      # human 'go' line
python3 scripts/rpi_test_arduino_serial.py status
python3 scripts/rpi_test_arduino_serial.py estop
```

Common flags: `--port /dev/ttyACM0`, `--baud 115200`, `--settle 2.0` (seconds after open).

---

## 4. ROS bridge (when the stack is running)

`arduino_bridge_node` maps topics to the same `M` / `E` / … lines; the brain publishes drive at control rate so the watchdog stays fed. Topic-level overview: [INTEGRATION.md](INTEGRATION.md), [ARCHITECTURE.md](ARCHITECTURE.md).
