"""Linux left Teensy upload; button mode or explicit isolated-board soft reboot."""
import argparse
import hashlib
from pathlib import Path
import subprocess
import sys
import time

LEFT_SERIAL = "5879620"


def usb_inventory(root=Path("/sys/bus/usb/devices")):
    devices = []
    for path in root.iterdir():
        try:
            devices.append(dict(path=path.name, vendor=(path / "idVendor").read_text().strip(),
                                product=(path / "idProduct").read_text().strip(),
                                serial=(path / "serial").read_text().strip() if (path / "serial").exists() else None))
        except (OSError, ValueError):
            continue
    return devices


def target_path(devices):
    matches = [d for d in devices if d["serial"] == LEFT_SERIAL and d["vendor"] == "16c0" and d["product"] == "0483"]
    if len(matches) != 1:
        raise RuntimeError("Expected exactly one normal USB-serial left encoder 5879620; do not press Program before this check")
    if any(d["vendor"] == "16c0" and d["product"] == "0478" for d in devices):
        raise RuntimeError("A Teensy is already in programming mode; inspect it before uploading")
    return matches[0]["path"]


def bootloader_ready(devices, expected_path):
    bootloaders = [d for d in devices if d["vendor"] == "16c0" and d["product"] == "0478"]
    if not bootloaders:
        return False
    if len(bootloaders) != 1 or bootloaders[0]["path"] != expected_path:
        raise RuntimeError("Programming-mode device is not exclusively at the verified left encoder USB location")
    return True


def require_only_left_teensy(devices):
    path = target_path(devices)
    pjrc = [d for d in devices if d["vendor"] == "16c0"]
    if len(pjrc) != 1 or pjrc[0]["serial"] != LEFT_SERIAL:
        raise RuntimeError("Automatic reboot requires the left encoder to be the ONLY connected Teensy; keep the control 4.1 disconnected")
    return path


def loader_command(uploader, firmware, soft_reboot=False):
    flags = ["--mcu=mk20dx256", "-v"]
    if soft_reboot:
        flags.extend(["-s", "-w"])
    return [str(uploader), *flags, str(firmware)]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--uploader", type=Path, required=True)
    parser.add_argument("--port", type=Path, required=True)
    parser.add_argument("--firmware", type=Path, required=True)
    parser.add_argument("--soft-reboot", action="store_true",
                        help="No-button upload only when the left encoder is the sole connected Teensy")
    args = parser.parse_args(argv)
    if not sys.platform.startswith("linux"):
        parser.error("Build on any supported host; guarded upload runs on Tractor01 Linux only")
    if not args.uploader.is_file() or not args.firmware.is_file():
        parser.error("Uploader or compiled firmware is missing")
    from serial.tools import list_ports
    ports = [p for p in list_ports.comports() if Path(p.device).resolve() == args.port.resolve()]
    if len(ports) != 1 or ports[0].serial_number != LEFT_SERIAL:
        parser.error(f"Configured upload_port {args.port} is not left encoder {LEFT_SERIAL}; inspect current USB ports")
    # Refuse while the position producer owns the encoder.
    state = subprocess.run(["systemctl", "is-active", "calc-publish-position.service"], capture_output=True, text=True)
    if state.stdout.strip() in ("active", "activating", "reloading"):
        parser.error("Stop calc-publish-position.service before flashing the encoder")
    path = target_path(usb_inventory())
    print("Verified left encoder", LEFT_SERIAL, "at USB location", path, flush=True)
    print("Firmware SHA256:", hashlib.sha256(args.firmware.read_bytes()).hexdigest(), flush=True)
    if args.soft_reboot:
        require_only_left_teensy(usb_inventory())
        print("Automatic reboot test: verified ONLY left encoder connected. Do NOT press Program.", flush=True)
        return subprocess.run(loader_command(args.uploader, args.firmware, soft_reboot=True), timeout=30).returncode
    print("Keep tractor stopped, engine off, phone Pause. Press ONLY the LEFT ENCODER Teensy Program button now (within 90 seconds).", flush=True)
    deadline = time.monotonic() + 90
    while not bootloader_ready(usb_inventory(), path):
        if time.monotonic() >= deadline:
            raise TimeoutError("No matching left-encoder programming device; nothing uploaded")
        time.sleep(.2)
    # No -s, teensy_reboot, generic port discovery or automatic controller reset.
    command = loader_command(args.uploader, args.firmware)
    return subprocess.run(command, timeout=30).returncode


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError, TimeoutError) as exc:
        print("LEFT ENCODER UPLOAD STOPPED:", exc, file=sys.stderr)
        raise SystemExit(1)
