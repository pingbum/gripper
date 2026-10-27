"""Install the reviewed CAN0 TDC overlay on this Orin NX; default is dry-run."""
import argparse
from datetime import datetime
import difflib
import hashlib
import os
from pathlib import Path
import re
import stat
import subprocess
import tempfile


BOOT_CONFIG = Path("/boot/extlinux/extlinux.conf")
OVERLAY_DEST = Path("/boot/dtb/ma732-orin-can0-tdc.dtbo")
BASE_DTB = Path("/boot/dtb/kernel_tegra234-p3768-0000+p3767-0000-nv.dtb")
BASE_SHA256 = "5e7178320a5cf5812a44f58f176db1aa6201cfcb63b618ac968767ec1e855ed9"
ORIGINAL_SHA256 = "d242154d0e7e3f34c5843fa427aa5e1a9f5ba0c9bfc1c0f6a104f4612c530993"
OVERLAY_ONLY_SHA256 = "9e0be00fc137e143ec5c0e380a4ce1cb8a35b44582a08cff13e5fab31c309c5b"
SOURCE_SHA256 = "b37bf3eb9b6dfabb9be613d96bb4948a2b55191ec7cb3a5ee6c083c8646b02bf"


def digest(data):
    return hashlib.sha256(data).hexdigest()


def candidate_config(original):
    if digest(original) == OVERLAY_ONLY_SHA256:
        # Upgrade only our exact previous candidate; keep primary untouched.
        prefix, entry = original.decode().split("\nLABEL can0-tdc\n", 1)
        return (prefix + "\nLABEL can0-tdc\n" + add_base_dtb(entry)).encode()
    if digest(original) != ORIGINAL_SHA256:
        raise ValueError("Boot configuration differs from the reviewed original; refusing to replace it")
    text = original.decode()
    primary = re.search(r"^LABEL primary\s*\n((?:[ \t]+[^#\n].*\n)+)", text, re.M)
    if primary is None:
        raise ValueError("Missing primary boot entry")
    entry = re.sub(r"^\s*MENU LABEL .*$", "      MENU LABEL CAN0 TDC test (5Mbps)",
                   primary[1], flags=re.M)
    entry += f"      OVERLAYS {OVERLAY_DEST}\n"
    text = re.sub(r"^DEFAULT primary$", "DEFAULT can0-tdc", text, flags=re.M)
    text = re.sub(r"^TIMEOUT 30$", "TIMEOUT 100", text, flags=re.M)
    return (text + "\nLABEL can0-tdc\n" + add_base_dtb(entry)).encode()


def add_base_dtb(entry):
    # R36 Jetson-IO uses both FDT and OVERLAYS; OVERLAYS alone did not load here.
    entry = entry.replace("      LINUX /boot/Image\n", f"      LINUX /boot/Image\n      FDT {BASE_DTB}\n")
    return re.sub(r"^(\s*APPEND .*)$", r"\1 ma732.can0_tdc=1", entry, flags=re.M)


def atomic_write(path, data):
    fd, name = tempfile.mkstemp(prefix=".can0-tdc-", dir=path.parent)
    try:
        with os.fdopen(fd, "wb") as stream:
            os.fchmod(stream.fileno(), 0o644)
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(name, path)
        directory = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
    finally:
        Path(name).unlink(missing_ok=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--install", action="store_true")
    parser.add_argument("--reboot", action="store_true", help="Reboot only after verified installation")
    args = parser.parse_args()
    if args.reboot and not args.install:
        parser.error("--reboot requires --install")
    if args.install and os.geteuid() != 0:
        raise ValueError("Installation requires sudo")
    compatible = Path("/proc/device-tree/compatible").read_bytes().split(b"\0")
    if b"nvidia,p3768-0000+p3767-0000" not in compatible:
        raise ValueError("This installer is limited to the inspected Orin NX board")
    if "REVISION: 4.7," not in Path("/etc/nv_tegra_release").read_text():
        raise ValueError("Revalidate the overlay for this L4T release before installation")
    if BOOT_CONFIG.is_symlink() or not stat.S_ISREG(BOOT_CONFIG.stat().st_mode):
        raise ValueError("Unexpected boot configuration file type")
    original = BOOT_CONFIG.read_bytes()
    candidate = candidate_config(original)
    source = Path(__file__).with_name("orin_can0_tdc_overlay.dts")
    source_data = source.read_bytes()
    if digest(source_data) != SOURCE_SHA256:
        raise ValueError("Overlay source differs from the reviewed source")
    base_data = BASE_DTB.read_bytes()
    if digest(base_data) != BASE_SHA256:
        raise ValueError("Base DTB differs from the verified board DTB")

    with tempfile.TemporaryDirectory(prefix="ma732-tdc-install-") as work:
        work = Path(work)
        (work / "overlay.dts").write_bytes(source_data)
        overlay, live, merged = (work / name for name in ("overlay.dtbo", "live.dtb", "merged.dtb"))
        base = work / "base.dtb"
        base.write_bytes(base_data)
        subprocess.run(["dtc", "-@", "-I", "dts", "-O", "dtb", "-o", str(overlay), str(work / "overlay.dts")], check=True)
        subprocess.run(["dtc", "-q", "-I", "fs", "-O", "dtb", "-o", str(live), "/proc/device-tree"], check=True)
        subprocess.run(["fdtoverlay", "-i", str(live), "-o", str(merged), str(overlay)], check=True)
        base_merged = work / "boot-merged.dtb"
        subprocess.run(["fdtoverlay", "-i", str(base), "-o", str(base_merged), str(overlay)], check=True)
        for setting, expected in (("2m_1m", "0 48 7f00 0"), ("5m", "0 48 7f00 600"), ("8m", "0 48 7f00 400")):
            node = f"/bus@0/mttcan@c310000/prod-settings/prod_c_can_{setting}/board"
            for tree in (merged, base_merged):
                actual = subprocess.check_output(["fdtget", "-t", "x", str(tree), node, "prod"], text=True).strip()
                if actual != expected:
                    raise ValueError(f"Merged overlay verification failed: {tree.name}: {setting}: {actual}")
        blob = overlay.read_bytes()

    print("".join(difflib.unified_diff(original.decode().splitlines(True), candidate.decode().splitlines(True),
                                    fromfile=str(BOOT_CONFIG), tofile="reviewed candidate")))
    print("Overlay compile and merge checks passed.", flush=True)
    if not args.install:
        print("Dry-run only: no boot files changed.")
        return
    if BOOT_CONFIG.read_bytes() != original:
        raise ValueError("Boot configuration changed during validation")
    if BASE_DTB.read_bytes() != base_data:
        raise ValueError("Base DTB changed during validation")
    if OVERLAY_DEST.is_symlink() or (OVERLAY_DEST.exists() and OVERLAY_DEST.read_bytes() != blob):
        raise ValueError("A different overlay already exists at the target path")
    backup = BOOT_CONFIG.with_name("extlinux.conf.pre-can0-tdc-" + datetime.now().strftime("%Y%m%d-%H%M%S"))
    with backup.open("xb") as stream:
        stream.write(original)
        stream.flush()
        os.fsync(stream.fileno())
    os.chmod(backup, 0o644)
    print(f"Backup: {backup}", flush=True)
    atomic_write(OVERLAY_DEST, blob)
    if OVERLAY_DEST.read_bytes() != blob:
        raise ValueError("Installed overlay verification failed; boot configuration was not changed")
    try:
        atomic_write(BOOT_CONFIG, candidate)
        if BOOT_CONFIG.read_bytes() != candidate:
            raise ValueError("Installed boot configuration verification failed")
    except Exception:
        atomic_write(BOOT_CONFIG, original)
        raise
    os.sync()
    print("INSTALLED: default=can0-tdc; original primary entry retained; boot menu timeout=10 seconds.", flush=True)
    print("Recovery: select 'primary kernel' in the boot menu, or restore the backup above.", flush=True)
    print("After reboot: close app.py, then run 'bash jetson/check_can0_5m.sh' from tools.", flush=True)
    if args.reboot:
        print("Installation verified. Rebooting now.", flush=True)
        subprocess.run(["systemctl", "reboot"], check=True)
    else:
        print("Reboot required before hardware validation.")


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, subprocess.CalledProcessError) as exc:
        raise SystemExit(f"STOP: {exc}")
