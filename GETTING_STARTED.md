# Getting Started

Everything below assumes `firebot_mega.ino` pin assignments match your
bench wiring (see **Bench-tested harness** in [docs/WIRING.md](docs/WIRING.md)
and `arduino/tested_arduino/`). Legacy KiCad tables are for reference only.

## Three sketches

| Sketch | Scope |
|---|---|
| `arduino/tests/drive_test/drive_test.ino` | Drive motors only |
| `arduino/tests/extinguisher_test/extinguisher_test.ino` | Solenoid + lead-screw only |
| `arduino/firebot_mega/firebot_mega.ino` | Everything + state-machine demo + the Pi-facing protocol |

Flash one, open Serial Monitor at **115200 / Newline**, type `?` (tests)
or `help` (unified firmware).

## Drive test (`drive_test.ino`)

Put the robot on blocks for the first pass.

```text
f 100       forward
b 100       backward
l 100       spin left in place
r 100       spin right in place
s           stop
spin        canned 2.5s left, pause, 2.5s right
demo        short fwd/back + full left/right spin
speed 80    change the default PWM
```

If a side spins backwards, swap its motor leads or flip the direction
pins in the sketch.

## Extinguisher test (`extinguisher_test.ino`)

```text
pin on       solenoid energize (pin pulled)
pin off      release
pulse 1500   pin on for 1.5 s then off
advance      lead-screw forward ~5.3 s (auto-stops)
retract      lead-screw reverse ~5.3 s (auto-stops)
stop         halt the stepper now
seq          full discharge: pin -> advance -> retract
abort        cancel seq mid-run
```

## Unified firmware (`firebot_mega.ino`)

Same hardware tests as above plus a full brain-state mirror and the
machine protocol the Pi speaks. Useful commands:

```text
help                       full menu
status                     current state, drive values, sensors
forward 100 / back 100     drive (default speed 75 if omitted)
left 75 / right 75         spin in place
go                         full bench solenoid + lead-screw sequence
pin on / pin off           solenoid
advance / retract          lead-screw ~5.3 s each (auto-stop)
sensor us on | off         enable/disable HC-SR04 (same for ir, mic)
watch us                   live distance line every second
state searching            spin in place until 'stop'
state approaching          short-pulse forward drive
state warning              5 s countdown, then auto-extinguish
state next                 step one phase forward in the FSM
test motors | solenoid | stepper | all
estop                      all outputs LOW, firmware -> ESTOP
```

Full reference: [docs/ARDUINO_TESTING.md](docs/ARDUINO_TESTING.md). Command map (Pi vs Monitor vs script): [docs/COMMANDS.md](docs/COMMANDS.md).

## Indoor safety

- Default drive PWM is ~100/255. Push higher only on blocks.
- `state approaching` pulses drive 400 ms, rests 600 ms, re-checks
  detection. It does not drive continuously.
- `test motors` uses 500 ms forward/back bursts.
- Stop paths: `stop`, `estop`, or unplug USB (1 s watchdog zeros the
  motors when the Pi is the one driving).

For wall avoidance:

```text
sensor us on
watch us           # cm readout every second
```

Then on the Pi side, set `approach_strategy: yolo_ultrasonic` in
`firebot_ws/src/firebot/config/firebot_params.yaml` so the brain
respects `safety_stop_cm`.

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| Garbled text in Serial Monitor | Not 115200 baud. |
| Commands do nothing | Line ending isn't set to Newline. |
| Upload fails, "port busy" | Another Serial Monitor has the port open, or a power-only USB cable. |
| Motors buzz, don't turn | Battery off / driver VM not powered. USB alone doesn't power motors. |
| Only one side drives | Loose wire on one motor channel (see WIRING.md harness table). |
| `advance`/`retract` goes the wrong way | Swap A4988 coils or the DIR logic. |
| `us` always returns -1 | HC-SR04 unwired or out of range. Try `watch us` while moving your hand in front of it. |
| `pin on` does nothing | Solenoid rail off, or Q1 gate/flyback missing. Check the schematic. |

## Adding the Pi

Command list for Pi 5 (git, Docker, scripts): [docs/RPI5_COMMANDS.md](docs/RPI5_COMMANDS.md).

Flash `firebot_mega.ino`, plug the Mega into the Pi over USB, then on
the Pi:

```bash
docker compose -f docker/docker-compose.yml up firebot
```

```bash
docker exec -it firebot-firebot-1 firebot alarm     # start the sequence
docker exec -it firebot-firebot-1 firebot confirm   # operator OK
docker exec -it firebot-firebot-1 firebot estop
```

Serial Monitor commands still work while the Pi is driving, but the
brain will overwrite them on the next tick. Full flow in
[docs/INTEGRATION.md](docs/INTEGRATION.md).

### Vision weights

Default Docker compose mounts `models/` to `/models`. The sample
[`best_small.pt`](https://github.com/sayedgamal99/Real-Time-Smoke-Fire-Detection-YOLO11)
is tracked in-repo; to use your own `.pt`, place it under `models/` and set
`vision_node.model_path` in `firebot_ws/src/firebot/config/firebot_params.yaml`.

### Standalone Pi tests (camera / YOLO / Mega USB)

From the repo root on the Pi (with dependencies installed):

```bash
python3 scripts/rpi_test_camera.py --preview
python3 scripts/rpi_test_yolo_fire.py --video-mode
python3 scripts/rpi_test_yolo_fire.py --headless
python3 scripts/rpi_test_arduino_serial.py --port /dev/ttyACM0
```

YOLO script defaults to `models/best_small.pt` (see [Flare Guard YOLO11](https://github.com/sayedgamal99/Real-Time-Smoke-Fire-Detection-YOLO11)). Use `--model path/to/fire_yolo11_small.pt` if you keep weights under another name. With the full stack running, `python3 scripts/rpi_test_yolo_fire.py --ros` forwards **a/z** (alarm), **c/x** (confirm/deny), **e** (estop) to ROS while showing the camera (desktop or VNC).

Arduino serial test runs a **short spin** then **estop** — use blocks and `--spin-speed`/`--spin-ms` if needed. `--dry-run` prints commands only.
