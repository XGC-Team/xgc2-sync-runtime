#!/usr/bin/env python3
"""Check target ELF identity only; not a dependency-closure or dlopen verdict."""
import argparse
from pathlib import Path
import struct
import subprocess
import sys

MACHINES = {"focal-noetic-amd64": 62, "focal-noetic-arm64": 183}


def verify(path, platform):
    path = Path(path)
    if path.is_symlink() and path.resolve().parent != path.parent.resolve():
        raise ValueError("library link escapes its directory: {}".format(path))
    with path.open("rb") as stream:
        header = stream.read(64)
    if len(header) != 64 or header[:7] != b"\x7fELF\x02\x01\x01":
        raise ValueError("not a 64-bit little-endian ELF: {}".format(path))
    kind, machine, version = struct.unpack_from("<HHI", header, 16)
    if machine != MACHINES[platform] or kind not in (2, 3) or version != 1:
        raise ValueError("wrong ELF target/type: {} (machine={}, type={}, expected={})".format(path, machine, kind, MACHINES[platform]))
    check = subprocess.run(["readelf", "--file-header", str(path)], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    if check.returncode:
        raise ValueError("readelf rejected {}: {}".format(path, check.stderr.strip()))
    return "ELF64 little-endian machine={} type={} {}".format(machine, kind, path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--platform", choices=sorted(MACHINES), required=True)
    parser.add_argument("paths", nargs="+")
    args = parser.parse_args()
    try:
        for path in args.paths:
            print(verify(path, args.platform))
    except (OSError, ValueError, RuntimeError) as error:
        print("verify-onboard-elf: {}".format(error), file=sys.stderr)
        return 4
    return 0


if __name__ == "__main__":
    sys.exit(main())
