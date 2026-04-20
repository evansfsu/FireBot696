# FireBot696

A deliberately minimal fire-fighting robot stack: Raspberry Pi 5 + Arduino
Mega 2560, ROS2 Humble in Docker, custom YOLO vision on an ov5647 camera,
skid-steer drive via two DFR0601 motor drivers, an A4988 lead-screw clamp,
and a solenoid pin-puller.

> **New teammate? Start with [GETTING_STARTED.md](GETTING_STARTED.md).**
> It walks through cloning the repo, flashing the Arduino, and running
> the robot from the Serial Monitor without needing the Raspberry Pi.

## Docs at a glance

| Audience | Where to start |
|---|---|
| First-time contributor | [GETTING_STARTED.md](GETTING_STARTED.md) |
| Arduino-only bench tester | [docs/ARDUINO_TESTING.md](docs/ARDUINO_TESTING.md) |
| Wants the big picture | [docs/INTEGRATION.md](docs/INTEGRATION.md) |
| Editing firmware / nodes | [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md), [docs/WIRING.md](docs/WIRING.md), [docs/PROTOCOL.md](docs/PROTOCOL.md) |

## Repo layout

```
FireBot696/
├── arduino/
│   ├── firebot_mega/firebot_mega.ino   # unified firmware (FLASH THIS)
│   └── tests/                          # original bench-test sketches, reference only
├── Fire_Bot/                           # KiCad project (schematic, PCB, etc.)
├── Fire_Bot.net                        # netlist -- source of truth for pin map
├── Fire_Bot_V2.svg                     # rendered schematic
├── firebot_ws/src/
│   ├── firebot/                        # ament_python package: 3 nodes + CLI
│   └── firebot_interfaces/             # FireDetection.msg
├── docker/                             # Dockerfile, compose, entrypoint
├── models/                             # drop best.pt here before building
└── docs/                               # architecture, wiring, protocol
```

## Quick start on the Pi

```bash
cd ~/FireBot696
docker compose -f docker/docker-compose.yml build
docker compose -f docker/docker-compose.yml up firebot
```

From another Pi shell, once the stack is up:

```bash
docker exec -it firebot-firebot-1 bash -lc 'firebot status'
docker exec -it firebot-firebot-1 bash -lc 'firebot alarm'     # simulate alarm
docker exec -it firebot-firebot-1 bash -lc 'firebot confirm'   # operator OK
docker exec -it firebot-firebot-1 bash -lc 'firebot estop'     # hard stop
```

## State machine

`IDLE -> SEARCHING -> AWAITING_CONFIRM -> APPROACHING -> WARNING -> EXTINGUISHING -> COMPLETE -> IDLE`

- [docs/INTEGRATION.md](docs/INTEGRATION.md) -- how the Pi and Mega work
  together, standalone vs. tethered, and the indoor-safety guardrails.
  **Start here if you're trying to understand the full system.**
- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) -- topic / node diagrams.
- [docs/PROTOCOL.md](docs/PROTOCOL.md) -- Pi <-> Mega serial format.

## Testing the Arduino by itself (no Pi required)

The same `firebot_mega.ino` sketch exposes a human-friendly command set in
the Arduino IDE Serial Monitor. Flash it, open Serial Monitor at **115200**
with line ending **Newline**, type `help`, and you can exercise motors,
solenoid, stepper, sensors, and every brain state with plain commands like
`forward 150`, `pin on`, `sensor us on`, `state searching`. Missing
hardware is ignored gracefully. See [docs/ARDUINO_TESTING.md](docs/ARDUINO_TESTING.md)
for the teammate-facing walkthrough.

## Hardware constraints (read before editing firmware)

- Drive is **skid-steer**, not full mecanum. The PCB ties both DFR0601 channel-1
  inputs together (left pair) and both channel-2 inputs together (right pair),
  so `vy` is accepted by the protocol but ignored by the Mega.
- Encoders are **aggregate only**. All four FIT0186 Sensor A outputs share one
  net and all four Sensor B outputs share another, so the firmware reports two
  tick counters, not eight.
- MAX9814 and KY-032 share one sense line (`/OUT`). When KY-032 fires, it pulls
  the wire LOW and the mic envelope reads ~0. Brain logic treats IR as the
  priority signal on that line.

Full details in [docs/WIRING.md](docs/WIRING.md).

## Development (no robot)

```bash
docker compose -f docker/docker-compose.yml --profile dev up firebot-dev
```

`firebot-dev` mounts the source tree so you can iterate on Python without
rebuilding the image. No camera / Mega is required -- the nodes fall back to
"no device" behavior cleanly.
