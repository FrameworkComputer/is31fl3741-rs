#!/usr/bin/env python3
"""Validate E observations on the Framework matrix. Does not flash firmware.
Requires Linux serial access; records raw reports and rejects missing event evidence.
Short-event signaling is checked, but physical short diagnosis is not validated.
"""
import argparse
import json
import os
from pathlib import Path
import re
import select
import termios
import time


def decode(line):
    if not line.startswith("OBS:") or not line.endswith("; state restored"):
        raise ValueError("Incomplete observation or failed restoration: " + line)
    fields = dict(re.findall(r"(\w+)=([^\s;]+)", line))
    bitmap = bytes.fromhex(fields["bitmap"])
    if len(bitmap) != 45:
        raise ValueError("Expected 45 fault bytes")
    faults = {(sw + 1, cs + 1) for sw in range(9) for cs in range(39) if bitmap[sw * 5 + cs // 8] & (1 << (cs % 8))}
    if len(faults) != int(fields["faults"]):
        raise ValueError("Fault count does not match raw bitmap")
    return fields, faults


def validate(lines):
    expected_open = {(sw, 39) for sw in range(3, 9)}
    # Expected pin/status transitions for masked, latched, auto-clear, short event.
    expected = [("0", "0", "0", "1/0"), ("1", "1", "1", "1/0"), ("1", "0", "0", "0/0"), ("1", "1", "1", "2/0")]
    for index, (line, values) in enumerate(zip(lines, expected)):
        fields, faults = decode(line)
        actual = tuple(fields[name] for name in ["low1", "low12", "low100", "status"])
        if actual != values or fields["low_after_read"] != "0":
            raise ValueError(f"Unexpected INTB/status transition: {line}")
        if fields["scaling"] != "255" or fields["pwm"] != "255":
            raise ValueError("Unexpected drive configuration")
        if index < 3 and faults != expected_open:
            raise ValueError("Open bitmap does not match six unmapped active channels")
        # A short status event was generated, but the physical meaning of its
        # bitmap on this healthy board remains unresolved. Do not assert it is valid.


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--runs", type=int, default=3)
    args = parser.parse_args()
    if args.runs < 1 or args.output.exists():
        parser.error("Use a positive run count and a new output filename")
    fd = os.open(args.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    original = termios.tcgetattr(fd)
    attributes = termios.tcgetattr(fd)
    attributes[0] = attributes[1] = attributes[3] = 0
    attributes[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    attributes[4] = attributes[5] = termios.B115200
    records = []
    try:
        termios.tcsetattr(fd, termios.TCSANOW, attributes)
        for index in range(args.runs):
            os.write(fd, b"E")
            data = b""
            deadline = time.monotonic() + 10
            while data.count(b"\n") < 4 and time.monotonic() < deadline:
                if select.select([fd], [], [], 0.1)[0]:
                    data += os.read(fd, 4096)
            lines = data.decode().splitlines()
            records.append(lines)
            args.output.write_text(json.dumps({"reports": records, "short_bitmap_physical_meaning": "unvalidated"}, indent=2) + "\n")
            if len(lines) != 4:
                raise RuntimeError("Expected four observation lines")
            validate(lines)
            print(f"Run {index + 1}: PASS: open bitmap, mask, latch/status read, auto-clear, short-event signaling; short diagnosis unvalidated", flush=True)
    finally:
        termios.tcsetattr(fd, termios.TCSANOW, original)
        os.close(fd)


if __name__ == "__main__":
    main()
