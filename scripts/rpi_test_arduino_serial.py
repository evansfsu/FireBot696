#!/usr/bin/env python3
"""USB serial test: Arduino Mega (firebot_mega.ino) <-> Raspberry Pi.

Sends protocol lines from docs/PROTOCOL.md (115200 baud, newline-terminated).

Usage:
  python3 scripts/rpi_test_arduino_serial.py
  python3 scripts/rpi_test_arduino_serial.py --port /dev/ttyACM0 --dry-run

Safety: sends a short low-speed spin then stop and estop. Put robot on blocks.

Requires: pyserial (pip install pyserial).
"""
from __future__ import annotations

import argparse
import sys
import time


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", default="/dev/ttyACM0")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--settle", type=float, default=2.0, help="seconds after open (Mega reset)")
    p.add_argument("--spin-speed", type=int, default=35, help="wz for brief M,0,0,wz")
    p.add_argument("--spin-ms", type=int, default=400, help="duration of spin test")
    p.add_argument("--dry-run", action="store_true", help="print commands only")
    args = p.parse_args()

    if args.dry_run:
        print("C,US,0\nC,IR,0\nC,MIC,0\nS")
        print(f"M,0,0,{args.spin_speed}\n  (sleep {args.spin_ms} ms)\nM,0,0,0\nS\nR")
        return 0

    try:
        import serial
    except ImportError:
        print("pyserial missing: pip install pyserial", file=sys.stderr)
        return 1

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.2)
    except serial.SerialException as e:
        print(f"open {args.port} failed: {e}", file=sys.stderr)
        return 1

    print(f"Opened {args.port} @ {args.baud}, waiting {args.settle}s for Mega boot…")
    time.sleep(args.settle)
    t_drain = time.time()
    while ser.in_waiting and time.time() - t_drain < 1.0:
        line = ser.readline().decode("ascii", errors="replace").strip()
        if line:
            print(f"  < {line}")

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

    send("C,US,0")
    read_for(200)
    send("C,IR,0")
    read_for(200)
    send("C,MIC,0")
    read_for(200)
    send("S")
    read_for(500)
    send(f"M,0,0,{args.spin_speed}")
    time.sleep(max(args.spin_ms, 50) / 1000.0)
    read_for(100)
    send("M,0,0,0")
    read_for(300)
    send("S")
    read_for(500)
    send("R")
    read_for(500)

    ser.close()
    print("Done (estop sent).")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
