# Pi <-> Mega serial protocol

- Transport: USB-CDC on the Mega's native serial pins (D0/D1), 115200 baud,
  ASCII, `\n` terminated.
- Flow: bridge node sends commands as they arrive from ROS topics, plus an
  `S` status request on a periodic timer. The Mega never initiates messages
  except for `L,<log>` boot/error lines.

## Pi -> Mega

| Command | Arguments | Meaning |
|---|---|---|
| `M,vx,vy,wz` | three ints, each -255..255 | drive. `vy` is ignored (PCB is skid-steer). Mega does `left = vx + wz`, `right = vx - wz`. |
| `E,phase` | 0..4 | extinguisher phase. `0` off, `1` pin-pull (solenoid), `2` advance lead-screw, `3` retract lead-screw, `4` stop stepper. |
| `W,mode` | 0..2 | warning mode. `0` off, `1` steady, `2` countdown-beep. No physical output yet (no buzzer wired). |
| `S` | (none) | request status reply. |
| `R` | (none) | e-stop: all outputs LOW, stepper disabled, firmware enters `ESTOP`. |
| `C,<US\|IR\|MIC>,<0\|1>` | sensor + enable | enable/disable an optional sensor. Disabled sensors report `-1` in the status reply. |

## Mega -> Pi

| Line | Fields | Meaning |
|---|---|---|
| `D,encA,encB,us,ir,mic,state` | all ints; sensors are `-1` when disabled; `state` is the firmware FSM enum (0=IDLE, 1=DRIVING, 2=EXTINGUISHING, 3=ESTOP) | status reply to `S`. |
| `L,<message>` | free-text ASCII | firmware log line (boot banner, estop ack, `W` acknowledgement, etc.). |

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

## Reliability notes

- The bridge keeps a single `serial.Serial` open for the life of the process
  and a 50 ms read timeout. A missed line does not back up state: the next
  `S` tick simply replaces the stale data.
- Both sides are stateless with respect to sequence numbers. If a command is
  lost, the brain re-publishes the same `/cmd/drive` Twist every control
  cycle at 10 Hz, so the robot self-heals within one tick.
- On the Pi side, opening the serial port triggers a Mega auto-reset, so the
  bridge sleeps for 2 seconds after `Serial()` before sending the first
  configuration commands.
