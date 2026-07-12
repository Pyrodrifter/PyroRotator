#!/usr/bin/env python3
"""Package Arduino-ESP32 output for the SkyPhreak flasher."""

import argparse
import hashlib
import json
import shutil
import subprocess
from pathlib import Path


def one(build: Path, pattern: str) -> Path:
    matches = sorted(build.glob(pattern))
    if len(matches) != 1:
        raise SystemExit(f"Expected one {pattern!r} in {build}, found {len(matches)}")
    return matches[0]


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--build-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--version", required=True)
    args = parser.parse_args()

    build = args.build_dir.resolve()
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)

    sources = [
        (0x1000, one(build, "*.ino.bootloader.bin"), "bootloader.bin"),
        (0x8000, one(build, "*.ino.partitions.bin"), "partitions.bin"),
        (0x10000, one(build, "*.ino.bin"), "superrot-firmware.bin"),
    ]

    # boot_app0 is required by Arduino's default OTA partition layout. Its exact
    # core package location is recorded in the compile output, so Arduino copies
    # it into the build directory on supported core releases.
    boot_app = sorted(build.glob("**/boot_app0.bin"))
    if boot_app:
        sources.insert(2, (0xE000, boot_app[0], "boot_app0.bin"))

    images = []
    for offset, source, name in sources:
        destination = output / name
        shutil.copyfile(source, destination)
        images.append({"offset": offset, "file": name, "sha256": digest(destination)})

    manifest = {
        "schemaVersion": 1,
        "firmwareVersion": args.version,
        "chip": "esp32",
        "baud": 460800,
        "images": images,
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")

    # esptool is bundled with the ESP32 Arduino core and available on PATH while
    # packaging in CI. The merged image is useful for factory/recovery flashing;
    # SkyPhreak uses the individually checksummed images above.
    command = ["esptool", "--chip", "esp32", "merge-bin", "-o", str(output / "superrot-merged.bin")]
    for offset, _, name in sources:
        command.extend([hex(offset), str(output / name)])
    try:
        subprocess.run(command, check=True)
    except FileNotFoundError:
        print("esptool executable not on PATH; skipping optional merged image")


if __name__ == "__main__":
    main()
