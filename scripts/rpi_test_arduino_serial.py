#!/usr/bin/env python3
"""USB serial test: Raspberry Pi -> Arduino Mega (firebot_mega.ino).

Uses the machine protocol in docs/PROTOCOL.md (115200 baud, newline-terminated).
No ROS and no vision — just pyserial.

Examples:
  python3 scripts/rpi_test_arduino_serial.py smoke
  python3 scripts/rpi_test_arduino_serial.py smoke --dry-run
  python3 scripts/rpi_test_arduino_serial.py spin --ms 3000 --wz 40
  python3 scripts/rpi_test_arduino_serial.py advance
  python3 scripts/rpi_test_arduino_serial.py go
  python3 scripts/rpi_test_arduino_serial.py --port /dev/ttyACM0 estop

Stepper advance/retract: firmware runs ~5.3 s each (E,2 / E,3) then stops itself.
Motor commands from the Pi: resend M,.. every <1 s (watchdog) for long moves.

Requires: pip install pyserial
"""
from __future__ import annotations

import argparse
import sys
import time

# Mega PROTOCOL_DRIVE_WATCHDOG_MS — must refresh M,.. faster than this.
MOTOR_KEEPALIVE_MS = 300


def configure_sensors(send) -> None:
    send("C,US,0")
    send("C,IR,0")
    send("C,MIC,0")


def motor_hold(
    send, vx: int, vy: int, wz: int, duration_s: float, *, simulate: bool = False
) -> None:
    """Hold skid-steer command for duration_s, refreshing M before 1 s watchdog."""
    send(f"M,{vx},{vy},{wz}")
    if simulate:
        print(
            f"  (dry-run: would re-send M every {MOTOR_KEEPALIVE_MS} ms for {duration_s:.2f} s)"
        )
        send("M,0,0,0")
        return
    deadline = time.time() + max(duration_s, 0.0)
    while time.time() < deadline:
        time.sleep(MOTOR_KEEPALIVE_MS / 1000.0)
        if time.time() >= deadline:
            break
        send(f"M,{vx},{vy},{wz}")
    send("M,0,0,0")


def open_serial(port: str, baud: int, settle: float):
    import serial

    ser = serial.Serial(port, baud, timeout=0.2)
    print(f"Opened {port} @ {baud}, waiting {settle}s for Mega boot…")
    time.sleep(settle)
    t_drain = time.time()
    while ser.in_waiting and time.time() - t_drain < 1.0:
        line = ser.readline().decode("ascii", errors="replace").strip()
        if line:
            print(f"  < {line}")
    return ser


def cmd_smoke(args, send, read_for) -> None:
    configure_sensors(send)
    read_for(200)
    send("S")
    read_for(500)
    motor_hold(
        send, 0, 0, args.spin_speed, args.spin_ms / 1000.0, simulate=args.dry_run
    )
    read_for(100)
    send("S")
    read_for(500)
    send("R")
    read_for(500)
    print("Done (smoke: spin + estop).")


def cmd_spin(args, send, read_for) -> None:
    configure_sensors(send)
    read_for(200)
    motor_hold(send, 0, 0, args.wz, args.ms / 1000.0, simulate=args.dry_run)
    read_for(200)
    print("Done (spin).")


def cmd_drive(args, send, read_for) -> None:
    configure_sensors(send)
    read_for(200)
    motor_hold(send, args.vx, 0, args.wz, args.ms / 1000.0, simulate=args.dry_run)
    read_for(200)
    print("Done (drive).")


def cmd_advance(args, send, read_for) -> None:
    configure_sensors(send)
    read_for(200)
    send("E,2")
    print("Advance: Mega runs lead-screw ~5.3 s (see EXT_STEPPER_RUN_MS). Waiting…")
    if not args.dry_run:
        time.sleep(args.wait_s)
    else:
        print(f"  (dry-run: would sleep {args.wait_s} s)")
    send("E,0")
    read_for(500)
    print("Done (advance).")


def cmd_retract(args, send, read_for) -> None:
    configure_sensors(send)
    read_for(200)
    send("E,3")
    print("Retract: Mega runs ~5.3 s. Waiting…")
    if not args.dry_run:
        time.sleep(args.wait_s)
    else:
        print(f"  (dry-run: would sleep {args.wait_s} s)")
    send("E,0")
    read_for(500)
    print("Done (retract).")


def cmd_go(args, send, read_for) -> None:
    """Full bench sequence (human `go` command — not in PROTOCOL.md table)."""
    configure_sensors(send)
    read_for(200)
    send("go")
    print("GO: long timed sequence on Mega; watch L,* log lines. Waiting…")
    if not args.dry_run:
        time.sleep(args.wait_s)
    else:
        print(f"  (dry-run: would sleep {args.wait_s} s)")
    read_for(500)
    print("Done (go sent — does not wait for full sequence unless --wait-s is large).")


def cmd_estop(args, send, read_for) -> None:
    configure_sensors(send)
    read_for(100)
    send("R")
    read_for(500)
    print("Done (estop).")


def cmd_status(args, send, read_for) -> None:
    configure_sensors(send)
    read_for(100)
    send("S")
    read_for(800)
    print("Done (status).")


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", default="/dev/ttyACM0")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--settle", type=float, default=2.0, help="seconds after open (Mega reset)")
    p.add_argument("--dry-run", action="store_true", help="print actions only, no USB/serial")

    sub = p.add_subparsers(dest="command")

    s = sub.add_parser("smoke", help="short spin + estop (original default test)")
    s.add_argument("--spin-speed", type=int, default=35, help="wz for M,0,0,wz")
    s.add_argument("--spin-ms", type=int, default=400, help="spin duration (ms)")
    s.set_defaults(func=cmd_smoke)

    s = sub.add_parser("spin", help="in-place spin with watchdog keepalive")
    s.add_argument("--wz", type=int, default=40)
    s.add_argument("--ms", type=int, default=2000, help="total spin time (ms)")
    s.set_defaults(func=cmd_spin)

    s = sub.add_parser("drive", help="skid-steer: vx forward/back, wz yaw")
    s.add_argument("--vx", type=int, default=0)
    s.add_argument("--wz", type=int, default=0)
    s.add_argument("--ms", type=int, default=2000)
    s.set_defaults(func=cmd_drive)

    s = sub.add_parser("advance", help="E,2 lead-screw forward (~5.3 s in firmware)")
    s.add_argument("--wait-s", type=float, default=6.0, help="sleep after E,2 before E,0")
    s.set_defaults(func=cmd_advance)

    s = sub.add_parser("retract", help="E,3 lead-screw reverse (~5.3 s)")
    s.add_argument("--wait-s", type=float, default=6.0)
    s.set_defaults(func=cmd_retract)

    s = sub.add_parser("go", help="human 'go' = full bench solenoid+stepper sequence")
    s.add_argument("--wait-s", type=float, default=25.0, help="how long to drain serial after")
    s.set_defaults(func=cmd_go)

    s = sub.add_parser("estop", help="send R")
    s.set_defaults(func=cmd_estop)

    s = sub.add_parser("status", help="send S and print reply window")
    s.set_defaults(func=cmd_status)

    s = sub.add_parser("dry-commands", help="print example lines only (no serial)")
    s.add_argument("name", nargs="?", default="smoke")
    s.set_defaults(func=None)

    args = p.parse_args()
    if not args.command:
        args.command = "smoke"
        args.func = cmd_smoke
        if not hasattr(args, "spin_speed"):
            args.spin_speed = 35
        if not hasattr(args, "spin_ms"):
            args.spin_ms = 400

    if args.command == "dry-commands":
        print("C,US,0\nC,IR,0\nC,MIC,0")
        print("M,0,0,35  # repeat every <1s for long moves")
        print("E,2  # advance ~5.3s   E,3  # retract ~5.3s   E,0  # off")
        print("go  # human line, full sequence")
        print("S\nR")
        return 0

    if args.func is None:
        p.error("choose a subcommand (e.g. smoke, spin, advance) or use dry-commands")

    if args.dry_run:

        def send(line: str) -> None:
            print(f"> {line}")

        def read_for(ms: float) -> None:
            del ms  # no serial to drain

        args.func(args, send, read_for)
        return 0

    try:
        import serial  # noqa: F401
    except ImportError:
        print("pyserial missing: pip install pyserial", file=sys.stderr)
        return 1

    try:
        ser = open_serial(args.port, args.baud, args.settle)
    except Exception as e:
        print(f"open {args.port} failed: {e}", file=sys.stderr)
        return 1

    def send(line: str) -> None:
        ser.write((line + "\n").encode("ascii"))
        ser.flush()
        print(f"> {line}")

    def read_for(ms: float) -> None:
        deadline = time.time() + ms / 1000.0
        while time.time() < deadline:
            raw = ser.readline().decode("ascii", errors="replace").strip()
            if raw:
                print(f"  < {raw}")

    try:
        args.func(args, send, read_for)
    finally:
        ser.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
