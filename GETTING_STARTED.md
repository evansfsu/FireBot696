# Getting Started (Teammate Guide)

Welcome. This guide is written for someone who has:

- Never seen this codebase before.
- Used the Arduino IDE a few times.
- Not necessarily used ROS2, Docker, or Linux.

By the end, you'll be able to drive the motors, fire the extinguisher,
and run the full brain state machine on the robot -- all from the
Arduino IDE Serial Monitor. No Raspberry Pi required.

> TL;DR: clone the repo, flash one `.ino` file, open the Serial
> Monitor, type `help`. That's it.

---

## Contents

1. [What you need](#1-what-you-need)
2. [Clone the repo](#2-clone-the-repo)
3. [Pick a sketch](#3-pick-a-sketch)
4. [Flash your first test (motors)](#4-flash-your-first-test-motors)
5. [Test the extinguisher](#5-test-the-extinguisher)
6. [The big unified firmware (same flow but everything at once)](#6-the-big-unified-firmware)
7. [Indoor-safety cheat sheet](#7-indoor-safety-cheat-sheet)
8. [What to do if something looks wrong](#8-troubleshooting)
9. [When you're ready to add the Raspberry Pi](#9-when-youre-ready-to-add-the-raspberry-pi)

---

## 1. What you need

Hardware:
- The FireBot696 robot, wired per [`Fire_Bot.net`](Fire_Bot.net) (battery
  connected, drivers + solenoid + stepper hooked up).
- An Arduino Mega 2560 on the robot. Can be powered through its USB-B
  cable from your laptop -- you do not need the Pi for any of this.
- A USB-A-to-USB-B cable.

Software (install once):
- [Arduino IDE](https://www.arduino.cc/en/software) 2.x (any recent version works).
- [Git](https://git-scm.com/downloads) so you can clone this repo.

Optional but helpful:
- VS Code, Cursor, or any text editor for reading the code.

## 2. Clone the repo

Pick a folder on your machine and run:

```bash
git clone https://github.com/evansfsu/FireBot696.git
cd FireBot696
```

(Or click **Code -> Open with GitHub Desktop** on the repo page if you
prefer a GUI.)

Every path in this document is relative to that top-level
`FireBot696/` folder.

## 3. Pick a sketch

There are three Arduino sketches, arranged from smallest to biggest:

| Sketch | Tests | Good for |
|---|---|---|
| [`arduino/tests/drive_test/drive_test.ino`](arduino/tests/drive_test/drive_test.ino) | Just the 4 drive motors | "Do the wheels spin?" |
| [`arduino/tests/extinguisher_test/extinguisher_test.ino`](arduino/tests/extinguisher_test/extinguisher_test.ino) | Just solenoid + lead-screw | "Does the mech discharge?" |
| [`arduino/firebot_mega/firebot_mega.ino`](arduino/firebot_mega/firebot_mega.ino) | Everything -- motors, extinguisher, sensors, a full state-machine demo | Integration testing, or the real deal with the Pi |

**Start with `drive_test`.** It's the smallest, and it lets you confirm
the motor wiring is correct before you try anything more complicated.

## 4. Flash your first test (motors)

1. Open the Arduino IDE.
2. **File -> Open...** and pick
   `arduino/tests/drive_test/drive_test.ino`.
3. **Tools -> Board -> Arduino AVR Boards -> Arduino Mega or Mega 2560**.
4. Plug the Mega into your laptop over USB. Wait a couple of seconds.
5. **Tools -> Port** -> pick whichever COM port just appeared (on
   Windows it'll be something like `COM5`; on macOS/Linux it'll be
   `/dev/tty.usbmodemXXXX` or `/dev/ttyACM0`).
6. Click the **Upload** arrow (top-left). Wait for "Done uploading".
7. Open the **Serial Monitor** (magnifier icon, top-right, or
   Tools -> Serial Monitor).
8. In the Serial Monitor window set:
   - Baud rate: **115200**
   - Line ending: **Newline**
9. You should see a menu like:

   ```text
   drive_test.ino -- motor + in-place spin test
   Open Serial Monitor at 115200 baud, line ending = Newline.
   Commands (case-insensitive, space-separated):
     ? | h            this menu
     f [pwm]          forward
     ...
   ready
   ```

### Your first commands

**Put the robot on a stable surface, or better yet, up on blocks with
the wheels in the air.** Then type each of these in the Serial Monitor
and press Enter:

```text
f 100          (robot should roll forward)
s              (all wheels stop)
b 100          (rolls backward)
s
l 100          (spins left in place)
s
r 100          (spins right in place)
s
spin           (canned 2.5 s left, pause, 2.5 s right, stop)
demo           (small forward/back + full left/right spin cycle)
```

If all four wheels spin the expected direction for each command,
**your motor wiring is good**. If one side spins backwards, open the
code and flip the direction pins for that side (or just re-wire the
motor leads).

## 5. Test the extinguisher

Same flow, different sketch:

1. **File -> Open -> `arduino/tests/extinguisher_test/extinguisher_test.ino`**.
2. Upload to the Mega (Board / Port already set from step 4).
3. Open Serial Monitor at 115200, Newline.
4. Try these:

   ```text
   pin on       (solenoid energizes -- you should hear it click)
   pin off      (releases)
   pulse 1500   (on for 1.5 s then off -- same length the robot uses)
   advance      (lead-screw runs forward for 6 s, then auto-stops)
   retract      (runs back for 6 s, then auto-stops)
   stop         (halt the stepper immediately if needed)
   seq          (full discharge: pin on -> off -> advance -> retract)
   abort        (cancel seq mid-run)
   ```

If `advance` and `retract` move the lead-screw the direction you
expect, and if `pin on` actually clicks the solenoid, **your
extinguisher wiring is good**.

## 6. The big unified firmware

Once individual parts work, flash the all-in-one sketch:

- **File -> Open -> `arduino/firebot_mega/firebot_mega.ino`**.
- Upload.
- Open Serial Monitor at 115200, Newline.
- Type `help`.

This is the *same* firmware the Raspberry Pi will eventually talk to,
but it exposes plain-English commands so you can drive everything by
hand. Highlights:

```text
help                      show every command
status                    print current state, sensors, drive values
forward 100 / back 100    drive (same as drive_test)
left 100 / right 100      spin in place
pin on / pin off          solenoid
advance / retract         lead-screw
sensor us on              enable HC-SR04 ultrasonic
watch us                  live distance readout in the Serial Monitor
state searching           spin in place until you say 'stop'
state approaching         short-pulse forward drive (indoor-safe)
state warning             5 s countdown, then auto-extinguish sequence
state next                step one state forward through the FSM
estop                     slam everything to zero
test motors               canned motor subsystem test
test solenoid             canned solenoid subsystem test
test stepper              canned lead-screw subsystem test
test all                  all three in a row
```

Full reference: [`docs/ARDUINO_TESTING.md`](docs/ARDUINO_TESTING.md).

## 7. Indoor-safety cheat sheet

The robot lives in a room. Defaults are deliberately conservative:

- Motor PWM defaults are around **100 / 255** (not 255). Only push
  higher if the robot is on blocks.
- `state approaching` does **not** drive continuously. It pulses:
  drive 0.4 s, coast 0.6 s, re-check detection, repeat. That gives
  you time to hit `stop` between pulses if anything looks wrong.
- `test motors` forward/back bursts are only **0.5 s** each.
- **Any of these stops everything immediately:**
  - Type `stop` in the Serial Monitor.
  - Type `estop`.
  - Yank the USB cable (if the Pi is connected, a 1 s watchdog zeros
    the motors automatically).

If you want a true wall-avoidance stop, turn on the ultrasonic:

```text
sensor us on
watch us          # now you see the distance in cm every second
```

Once that's proven out, the Pi brain can be told to respect it by
setting `approach_strategy: yolo_ultrasonic` in
[`firebot_ws/src/firebot/config/firebot_params.yaml`](firebot_ws/src/firebot/config/firebot_params.yaml).

## 8. Troubleshooting

| Symptom | Likely cause |
|---|---|
| Serial Monitor shows garbled characters | Wrong baud rate. It must be 115200. |
| Commands seem to do nothing | Line ending isn't set to Newline in the Serial Monitor. |
| Nothing uploads, "port busy" error | Close any other Serial Monitor or VS Code terminal that has the port open. Try a different USB cable (some cables are power-only). |
| Motors buzz but don't turn | Check battery voltage / driver power. USB alone does NOT power the motors. |
| Only one side drives | Likely a loose wire on one of the two DFR0601 drivers -- both channels on each driver are ganged per the schematic. |
| `advance` / `retract` moves the wrong way | Swap the DIR pin logic in the sketch, or swap the coils on the A4988. |
| `sensor us on` then `us` always returns `-1` | HC-SR04 isn't wired or is out of range (> ~4 m). Try `watch us` while waving your hand in front of it. |
| `pin on` does nothing | Solenoid power rail may be off, or the NMOS gate resistor is missing. Check Q1 on the schematic. |

If a command confuses you, `help` prints the full menu again.

## 9. When you're ready to add the Raspberry Pi

Flash `firebot_mega.ino` onto the Mega, connect the Mega to the Pi
over USB, and on the Pi run:

```bash
cd ~/FireBot696
docker compose -f docker/docker-compose.yml build
docker compose -f docker/docker-compose.yml up firebot
```

From the Pi you can now run:

```bash
docker exec -it firebot-firebot-1 bash -lc 'firebot status'
docker exec -it firebot-firebot-1 bash -lc 'firebot alarm'    # kick off the state machine
docker exec -it firebot-firebot-1 bash -lc 'firebot confirm'  # operator OK
docker exec -it firebot-firebot-1 bash -lc 'firebot estop'
```

The Mega firmware behaves identically whether you are driving it from
the Pi or from the Serial Monitor. For a step-by-step walkthrough of
a full fire-fight with the Pi in the loop, read
[`docs/INTEGRATION.md`](docs/INTEGRATION.md).

---

Questions? Open an issue on the GitHub repo or ping the person who
sent you this guide.
