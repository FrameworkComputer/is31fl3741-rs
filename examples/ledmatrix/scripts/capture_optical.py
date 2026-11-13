#!/usr/bin/env python3
"""Linux camera regression capture; requires optical hardware_check firmware, ffmpeg,
v4l2-ctl, and Pillow. Does not flash firmware. Restores camera and matrix state.
"""
import argparse
import json
import os
from pathlib import Path
import select
import subprocess
import tempfile
import termios
import time

from PIL import Image, ImageDraw, ImageStat


def run(*args):
    return subprocess.check_output(args, text=True, stderr=subprocess.STDOUT)


def request(fd, command):
    os.write(fd, command.encode("ascii"))
    response = b""
    deadline = time.monotonic() + 8
    while b"\n" not in response and time.monotonic() < deadline:
        if select.select([fd], [], [], 0.1)[0]:
            response += os.read(fd, 4096)
    text = response.decode().strip()
    if not text.startswith("PASS:") or b"\n" not in response:
        raise RuntimeError(f"{command}: {text!r}")
    print(f"{command}: {text}", flush=True)
    return text


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="Matrix application serial device")
    parser.add_argument("--camera", default="/dev/video0")
    parser.add_argument("--output", type=Path, required=True, help="New, empty evidence directory")
    parser.add_argument("--crop", type=int, nargs=4, default=[175, 185, 490, 280], metavar=("LEFT", "TOP", "RIGHT", "BOTTOM"))
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    controls = ["auto_exposure", "exposure_time_absolute", "exposure_dynamic_framerate", "white_balance_automatic", "white_balance_temperature"]
    original = {}
    for control in controls:
        original[control] = int(run("v4l2-ctl", "-d", args.camera, "--get-ctrl", control).split(":", 1)[1].split()[0])
    def set_control(name, value):
        run("v4l2-ctl", "-d", args.camera, "--set-ctrl", f"{name}={value}")
    fd = os.open(args.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    serial_original = termios.tcgetattr(fd)
    attributes = termios.tcgetattr(fd)
    attributes[0] = attributes[1] = attributes[3] = 0
    attributes[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    attributes[4] = attributes[5] = termios.B115200
    termios.tcsetattr(fd, termios.TCSANOW, attributes)
    records = []
    try:
        for name, value in [("auto_exposure", 1), ("exposure_time_absolute", 50), ("exposure_dynamic_framerate", 0), ("white_balance_automatic", 0), ("white_balance_temperature", 4600)]:
            set_control(name, value)
        with tempfile.TemporaryDirectory(prefix="is31-optical-") as temporary:
            for index, command in enumerate("0123456540"):
                report = request(fd, command)
                time.sleep(0.5)
                pattern = str(Path(temporary) / "frame-%02d.png")
                run("ffmpeg", "-hide_banner", "-loglevel", "error", "-f", "v4l2", "-i", args.camera, "-vf", r"select=gte(n\,15)", "-frames:v", "3", "-fps_mode", "vfr", "-y", pattern)
                values = []
                for frame in range(1, 4):
                    with Image.open(Path(temporary) / f"frame-{frame:02}.png") as source:
                        if not (0 <= args.crop[0] < args.crop[2] <= source.width and 0 <= args.crop[1] < args.crop[3] <= source.height):
                            raise ValueError("Crop must be inside the camera frame")
                        crop = source.convert("RGB").crop(tuple(args.crop))
                        mean = sum(ImageStat.Stat(crop).mean) / 3
                        histogram = crop.histogram()
                        clipped = sum(histogram[channel * 256 + 255] for channel in range(3))
                        values.append({"mean_rgb": mean, "saturated_channel_samples": clipped})
                        if frame == 1:
                            crop.save(args.output / f"{index:02}-mode-{command}.png")
                records.append({"mode": command, "report": report, "frames": values})
                (args.output / "measurements.json").write_text(json.dumps({"original_controls": original, "capture_controls": {"auto_exposure": 1, "exposure_time_absolute": 50, "exposure_dynamic_framerate": 0, "white_balance_automatic": 0, "white_balance_temperature": 4600}, "crop_xyxy": args.crop, "sequence": records}, indent=2) + "\n")
        width, height = args.crop[2] - args.crop[0], args.crop[3] - args.crop[1]
        sheet = Image.new("RGB", (width, (height + 24) * len(records)))
        draw = ImageDraw.Draw(sheet)
        for index, record in enumerate(records):
            draw.text((4, index * (height + 24) + 4), f"Mode {record['mode']}", fill="white")
            with Image.open(args.output / f"{index:02}-mode-{record['mode']}.png") as crop:
                sheet.paste(crop, (0, index * (height + 24) + 24))
        sheet.save(args.output / "comparison.png")
    finally:
        errors = []
        try:
            request(fd, "Z")
        except Exception as error:
            errors.append(str(error))
        # Manual mode first permits restoring absolute exposure and white balance.
        for name, value in [("auto_exposure", 1), ("white_balance_automatic", 0), ("exposure_time_absolute", original["exposure_time_absolute"]), ("white_balance_temperature", original["white_balance_temperature"]), ("exposure_dynamic_framerate", original["exposure_dynamic_framerate"]), ("white_balance_automatic", original["white_balance_automatic"]), ("auto_exposure", original["auto_exposure"])]:
            try:
                set_control(name, value)
            except Exception as error:
                errors.append(str(error))
        termios.tcsetattr(fd, termios.TCSANOW, serial_original)
        os.close(fd)
        if errors:
            raise RuntimeError("Restoration incomplete: " + "; ".join(errors))


if __name__ == "__main__":
    main()
