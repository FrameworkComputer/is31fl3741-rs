#!/usr/bin/env python3
"""Build Cortex-M0+ mapping probes and retain assembly/object size evidence."""
import argparse
import json
from pathlib import Path
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parent.parent
    args.output.mkdir(parents=True, exist_ok=True)
    result = subprocess.run(["cargo", "build", "--offline", "--release", "--target", "thumbv6m-none-eabi", "--all-features", "--message-format=json"], cwd=root, check=True, capture_output=True, text=True)
    artifacts = {}
    for line in result.stdout.splitlines():
        item = json.loads(line)
        if item.get("reason") == "compiler-artifact":
            for filename in item["filenames"]:
                if filename.endswith(".rlib"):
                    artifacts[item["target"]["name"]] = filename
    base = ["rustc", "--edition=2021", "--crate-type", "lib", "--target", "thumbv6m-none-eabi", "-C", "opt-level=3", "-L", "dependency=" + str(root / "target/thumbv6m-none-eabi/release/deps")]
    for name in ["is31fl3741", "embedded_hal"]:
        base += ["--extern", f"{name}={artifacts[name]}"]
    for kind, suffix in [("asm", "s"), ("obj", "o")]:
        subprocess.run(base + ["--emit=" + kind, str(root / "measurements/mapping_probe.rs"), "-o", str(args.output / ("mapping." + suffix))], check=True)
    for tool in ["llvm-size", "llvm-nm"]:
        options = ["-S"] if tool == "llvm-nm" else []
        text = subprocess.check_output([tool] + options + [str(args.output / "mapping.o")], text=True)
        (args.output / (tool + ".txt")).write_text(text)
        print(text)
    print("Static table contents are external to the probe object: compare a linked firmware image too.")


if __name__ == "__main__":
    main()
