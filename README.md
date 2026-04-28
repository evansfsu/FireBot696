# FireBot696

Fire-fighting robot. Raspberry Pi 5 + Arduino Mega 2560, ROS2 Humble in
Docker, YOLO on an ov5647 camera, skid-steer drive with two DFR0601
drivers, A4988 lead-screw clamp, solenoid pin-puller.

## Where to look

| What you want | File |
|---|---|
| Fast path for bench-testing with Arduino | [GETTING_STARTED.md](GETTING_STARTED.md) |
| Command map (Pi, Serial Monitor, test script) | [docs/COMMANDS.md](docs/COMMANDS.md) |
| Full Serial Monitor command reference | [docs/ARDUINO_TESTING.md](docs/ARDUINO_TESTING.md) |
| How the Pi and Mega behave together | [docs/INTEGRATION.md](docs/INTEGRATION.md) |
| Node + topic diagram | [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) |
| Pin map (authoritative) | [docs/WIRING.md](docs/WIRING.md) |
| Serial protocol | [docs/PROTOCOL.md](docs/PROTOCOL.md) |

## Layout

```
arduino/
  firebot_mega/firebot_mega.ino     # unified firmware (flash this for the real robot)
  tests/drive_test/                 # motors only
  tests/extinguisher_test/          # solenoid + lead-screw only
  tests/...                         # legacy single-peripheral sketches
Fire_Bot/                           # KiCad project
Fire_Bot.net                        # netlist -- source of truth for pin map
firebot_ws/src/
  firebot/                          # vision, brain, bridge, CLI
  firebot_interfaces/               # FireDetection.msg
docker/                             # Dockerfile + compose + entrypoint
docs/
```

## Running on the Pi

```bash
docker compose -f docker/docker-compose.yml build
docker compose -f docker/docker-compose.yml up firebot
```

From another shell:

```bash
docker exec -it firebot-firebot-1 firebot status
docker exec -it firebot-firebot-1 firebot alarm      # simulate alarm
docker exec -it firebot-firebot-1 firebot confirm    # operator OK
docker exec -it firebot-firebot-1 firebot estop
```

State machine:
`IDLE -> SEARCHING -> AWAITING_CONFIRM -> APPROACHING -> WARNING -> EXTINGUISHING -> COMPLETE -> IDLE`.

## Running the Arduino on its own

`firebot_mega.ino` takes plain-English commands at 115200 baud when the
Pi isn't plugged in. Type `help` in the Serial Monitor. Missing hardware
is tolerated -- disabled sensors show `off`, unwired outputs just toggle
into the void.

## Wiring gotchas

- Skid-steer, not mecanum. The PCB ganges both DFR0601 channel-1 inputs
  (left pair) and both channel-2 inputs (right pair). `vy` is accepted
  by the protocol but ignored.
- Encoders are aggregate only -- all four Sensor A outputs share one
  net, all four Sensor B outputs share another.
- MAX9814 and KY-032 share one sense line. KY-032 pulls it LOW when
  triggered, which also pulls the mic envelope near zero. IR is the
  priority signal on that line.

## Dev without hardware

```bash
docker compose -f docker/docker-compose.yml --profile dev up firebot-dev
```

Mounts the source tree. No camera or Mega required; nodes no-op.
