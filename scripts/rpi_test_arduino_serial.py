#!/usr/bin/env python3
"""USB serial test: Raspberry Pi -> Arduino Mega (firebot_mega.ino).

Uses the machine protocol in docs/PROTOCOL.md (115200 baud, newline-terminated).
No ROS and no vision — just pyserial.

Examples (motors only — no stepper/solenoid/E commands):
  cd ~/Desktop/696/696   # example repo root on Pi
  python3 scripts/rpi_test_arduino_serial.py --version
  python3 scripts/rpi_test_arduino_serial.py           # default = motors
  python3 scripts/rpi_test_arduino_serial.py probe    # Serial only: do you see D,... ?
  python3 scripts/rpi_test_arduino_serial.py ports
  python3 scripts/rpi_test_arduino_serial.py drive --vx 75 --wz 0 --ms 1500
  python3 scripts/rpi_test_arduino_serial.py spin --ms 3000 --wz 75

Other:
  python3 scripts/rpi_test_arduino_serial.py smoke   # legacy: spin + R estop
  python3 scripts/rpi_test_arduino_serial.py advance

Stepper advance/retract: firmware runs ~5.3 s each (E,2 / E,3) then stops itself.
Motor commands from the Pi: resend M,.. every <1 s (watchdog) for long moves.
Motor PWM does not spin wheels if the **motor battery / driver supply is off** (USB only powers the Mega).
If nothing moves but you see `> M,...` and `D,...` replies, check VMOT on the drivers.

Requires: pip install pyserial
"""
from __future__ import annotations

import argparse
import sys
import time

__version__ = "5"  # Pi: git pull this repo so you get updates

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


def open_serial(
    port: str,
    baud: int,
    settle: float,
    *,
    wait_ready: bool = True,
    ready_timeout: float = 5.0,
) -> "serial.Serial":
    import serial

    ser = serial.Serial(port, baud, timeout=0.2)
    # Mega prints boot lines right after USB open (DTR reset). If we sleep without
    # reading, the USB CDC RX buffer can overflow and we lose L,firebot_mega_ready.
    print(
        f"Opened {port} @ {baud}; reading serial for {settle}s while Mega boots "
        "(unread boot text can overflow USB buffer and hide L,firebot_mega_ready)…"
    )
    saw_ready = False
    settle_end = time.time() + settle
    while time.time() < settle_end:
        raw = ser.readline().decode("ascii", errors="replace").strip()
        if raw:
            print(f"  < {raw}")
            if "firebot_mega_ready" in raw:
                saw_ready = True
        else:
            time.sleep(0.02)

    # Catch a late banner if prints just after the window
    t_late = time.time()
    while not saw_ready and time.time() - t_late < 0.5:
        raw = ser.readline().decode("ascii", errors="replace").strip()
        if raw:
            print(f"  < {raw}")
            if "firebot_mega_ready" in raw:
                saw_ready = True
                break
        else:
            time.sleep(0.05)

    if wait_ready and not saw_ready:
        deadline = time.time() + max(ready_timeout, 0.5)
        print(f"Still no ready banner; reading up to {ready_timeout:.1f}s more …")
        while time.time() < deadline:
            raw = ser.readline().decode("ascii", errors="replace").strip()
            if raw:
                print(f"  < {raw}")
                if "firebot_mega_ready" in raw:
                    saw_ready = True
                    break
            else:
                time.sleep(0.05)
        if not saw_ready:
            print(
                "[warn] No firebot_mega_ready — wrong port, wrong baud, or not firebot_mega.ino. "
                "Run: python3 scripts/rpi_test_arduino_serial.py probe",
                file=sys.stderr,
            )
    elif wait_ready and saw_ready:
        print("(OK: Mega boot banner received.)")

    return ser


def cmd_probe(args, send, read_for) -> None:
    """Send C + S after open; confirms Mega speaks protocol."""
    configure_sensors(send)
    read_for(300)
    send("S")
    read_for(1000)
    print("Done (probe). If you never saw D,... the Mega is not parsing lines (port/firmware).")


def cmd_motors(args, send, read_for) -> None:
    """Forward pulse, then spin — only M,.. commands (no E, go, or R unless --with-estop)."""
    print(
        "[check] Motor drivers need VMOT/battery; USB alone will not turn wheels.\n"
        "[check] Nothing else should use the same serial port (Docker bridge, Serial Monitor).",
        flush=True,
    )
    if not args.minimal_protocol:
        configure_sensors(send)
        read_for(200)
        send("S")
        read_for(800)
    motor_hold(
        send, args.vx, 0, 0, args.forward_ms / 1000.0, simulate=args.dry_run
    )
    read_for(150)
    motor_hold(
        send, 0, 0, args.wz, args.spin_ms / 1000.0, simulate=args.dry_run
    )
    read_for(200)
    if args.with_estop:
        send("R")
        read_for(300)
        print("Done (motors + estop).")
    else:
        print("Done (motors only — wheels stopped with M,0,0,0; no extinguisher/stepper).")


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
    print("Done (smoke: spin + estop). [Prefer: motors — no R; defaults match front_reverse_spin PWM 75]")


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
    p.add_argument("--version", action="store_true", help="print script revision and exit")
    p.add_argument("--port", default="/dev/ttyACM0")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--settle", type=float, default=2.5, help="seconds after open (Mega reset)")
    p.add_argument(
        "--no-wait-ready",
        action="store_true",
        help="do not wait for L,firebot_mega_ready after open",
    )
    p.add_argument(
        "--ready-timeout",
        type=float,
        default=5.0,
        help="max seconds to wait for boot banner",
    )
    p.add_argument("--dry-run", action="store_true", help="print actions only, no USB/serial")

    sub = p.add_subparsers(dest="command")

    s = sub.add_parser(
        "motors",
        help="drive test only: forward then spin (M commands); safe if stepper/solenoid unwired",
    )
    s.add_argument("--vx", type=int, default=75, help="forward PWM (bench sketch uses 75)")
    s.add_argument("--forward-ms", type=int, default=1000, help="forward pulse length (ms)")
    s.add_argument("--wz", type=int, default=75, help="spin rate for second pulse (in-place)")
    s.add_argument("--spin-ms", type=int, default=2000, help="spin duration (ms)")
    s.add_argument(
        "--minimal-protocol",
        action="store_true",
        help="omit C,.. and S (only M lines — use if sensor config causes issues)",
    )
    s.add_argument(
        "--with-estop",
        action="store_true",
        help="send R at end (normally unnecessary; wheels already stopped)",
    )
    s.set_defaults(func=cmd_motors)

    s = sub.add_parser(
        "smoke",
        help="legacy: short spin then R estop (use `motors` for a normal drive check)",
    )
    s.add_argument("--spin-speed", type=int, default=75, help="wz for M,0,0,wz")
    s.add_argument("--spin-ms", type=int, default=1500, help="spin duration (ms)")
    s.set_defaults(func=cmd_smoke)

    s = sub.add_parser("spin", help="in-place spin with watchdog keepalive")
    s.add_argument("--wz", type=int, default=75)
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

    s = sub.add_parser(
        "probe",
        help="open serial, wait for boot banner, send S — use this if motors do nothing",
    )
    s.set_defaults(func=cmd_probe)

    s = sub.add_parser("ports", help="list USB serial devices (pyserial)")
    s.set_defaults(func=None)

    s = sub.add_parser("dry-commands", help="print example lines only (no serial)")
    s.add_argument("name", nargs="?", default="motors")
    s.set_defaults(func=None)

    args = p.parse_args()
    if args.version:
        print(f"rpi_test_arduino_serial.py revision {__version__}")
        print("Default subcommand with no args: motors (not smoke).")
        return 0

    if not args.command:
        args.command = "motors"
        args.func = cmd_motors
        for key, val in (
            ("vx", 75),
            ("forward_ms", 1000),
            ("wz", 75),
            ("spin_ms", 2000),
            ("minimal_protocol", False),
            ("with_estop", False),
        ):
            if not hasattr(args, key):
                setattr(args, key, val)

    if args.command == "dry-commands":
        print("# Motors only (no E/go/R); default PWM 75 like front_reverse_spin:")
        print("M,75,0,0  # refresh every ~300ms for multi-second moves; then M,0,0,0")
        print("# Subcommands: motors | drive | spin   (~ stepper: advance | retract | go)")
        print("C,US,0\nC,IR,0\nC,MIC,0")
        print("E,2  # advance ~5.3s   E,3  # retract ~5.3s   E,0  # off")
        print("go  # human line, full sequence")
        print("S\nR")
        return 0

    if args.command == "ports":
        try:
            from serial.tools import list_ports
        except ImportError:
            print("pyserial missing: pip install pyserial", file=sys.stderr)
            return 1
        found = False
        for pobj in list_ports.comports():
            found = True
            print(f"{pobj.device}\t{pobj.description}")
        if not found:
            print("No serial ports found.")
        return 0

    if args.func is None:
        p.error("choose a subcommand (e.g. motors, spin, drive, probe) or use dry-commands")

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
        ser = open_serial(
            args.port,
            args.baud,
            args.settle,
            wait_ready=not args.no_wait_ready,
            ready_timeout=args.ready_timeout,
        )
    except Exception as e:
        print(f"open {args.port} failed: {e}", file=sys.stderr)
        print("Try: python3 scripts/rpi_test_arduino_serial.py ports", file=sys.stderr)
        return 1

    print(
        f"--- Subcommand: {args.command!r} (script rev {__version__}) | "
        "firebot_mega applyDrive() matches tested_arduino/front_reverse_spin ---"
    )

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
