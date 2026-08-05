#!/usr/bin/env python3
"""Generate Python (and optionally C) protobuf bindings for the CartPole protocol.

Works in three modes:
1. PlatformIO Nanopb already installed under `.pio/libdeps/esp32dev/Nanopb`
2. Vendored/downloaded Nanopb under `.deps/nanopb`
3. System `protoc` + downloaded `nanopb.proto` (Python bindings only)
"""

from __future__ import annotations

import argparse
import io
import os
import shutil
import subprocess
import sys
import tarfile
import urllib.request
import zipfile
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
PROTO_FILE = REPO_ROOT / "protocol.proto"
C_OUTPUT_DIR = REPO_ROOT / "firmware" / "src"
PY_OUTPUT_DIR = REPO_ROOT / "cartpole" / "device"
DEPS_DIR = REPO_ROOT / ".deps"
NANOPB_DIR = DEPS_DIR / "nanopb"
PIO_NANOPB = REPO_ROOT / ".pio" / "libdeps" / "esp32dev" / "Nanopb"
NANOPB_RELEASE = "0.4.8"
NANOPB_URL = (
    f"https://github.com/nanopb/nanopb/archive/refs/tags/{NANOPB_RELEASE}.tar.gz"
)


def _run(cmd: list[str], cwd: Path | None = None) -> None:
    print("+", " ".join(cmd))
    proc = subprocess.run(
        cmd,
        cwd=cwd or REPO_ROOT,
        text=True,
        capture_output=True,
    )
    if proc.returncode != 0:
        raise RuntimeError(
            f"Command failed ({proc.returncode}): {' '.join(cmd)}\n"
            f"stdout:\n{proc.stdout}\nstderr:\n{proc.stderr}"
        )


def _fix_import(path: Path) -> None:
    text = path.read_text(encoding="utf-8")
    fixed = text.replace("import nanopb_pb2", "from . import nanopb_pb2")
    if fixed != text:
        path.write_text(fixed, encoding="utf-8")


def ensure_nanopb() -> Path:
    if PIO_NANOPB.exists():
        return PIO_NANOPB
    if (NANOPB_DIR / "generator" / "proto" / "nanopb.proto").exists():
        return NANOPB_DIR

    DEPS_DIR.mkdir(parents=True, exist_ok=True)
    print(f"[update_protobuf] Downloading Nanopb {NANOPB_RELEASE}...")
    with urllib.request.urlopen(NANOPB_URL) as response:
        payload = response.read()
    with tarfile.open(fileobj=io.BytesIO(payload), mode="r:gz") as archive:
        try:
            archive.extractall(DEPS_DIR, filter="data")
        except TypeError:
            archive.extractall(DEPS_DIR)
    extracted = DEPS_DIR / f"nanopb-{NANOPB_RELEASE}"
    if NANOPB_DIR.exists():
        shutil.rmtree(NANOPB_DIR)
    extracted.rename(NANOPB_DIR)
    return NANOPB_DIR


def find_protoc() -> str | None:
    return shutil.which("protoc")


def generate_with_nanopb_protoc(nanopb_root: Path, with_c: bool) -> None:
    nanopb_protoc = nanopb_root / "generator" / "protoc"
    if not nanopb_protoc.exists():
        # Some releases ship generator/protoc as a python script without +x
        nanopb_protoc = nanopb_root / "generator" / "protoc"
    proto_include = nanopb_root / "generator" / "proto"
    PY_OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    if with_c:
        C_OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    cmd = [
        sys.executable,
        str(nanopb_protoc),
        f"-I{proto_include}",
        f"-I{PROTO_FILE.parent}",
        f"--python_out={PY_OUTPUT_DIR}",
    ]
    if with_c:
        cmd.append(f"--nanopb_out={C_OUTPUT_DIR}")
    cmd.extend(
        [
            str(proto_include / "nanopb.proto"),
            str(PROTO_FILE),
        ]
    )
    _run(cmd)
    _fix_import(PY_OUTPUT_DIR / "protocol_pb2.py")


def generate_with_system_protoc(nanopb_root: Path) -> None:
    protoc = find_protoc()
    if not protoc:
        raise RuntimeError(
            "Neither Nanopb generator/protoc nor system protoc is available."
        )
    proto_include = nanopb_root / "generator" / "proto"
    PY_OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    cmd = [
        protoc,
        f"-I{proto_include}",
        f"-I{PROTO_FILE.parent}",
        f"--python_out={PY_OUTPUT_DIR}",
        str(proto_include / "nanopb.proto"),
        str(PROTO_FILE),
    ]
    _run(cmd)
    _fix_import(PY_OUTPUT_DIR / "protocol_pb2.py")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--python-only",
        action="store_true",
        help="Skip C/firmware bindings (default for Raspberry Pi Docker).",
    )
    parser.add_argument(
        "--with-c",
        action="store_true",
        help="Also generate firmware C bindings via Nanopb.",
    )
    args = parser.parse_args()
    with_c = args.with_c and not args.python_only

    if not PROTO_FILE.exists():
        raise FileNotFoundError(f"Missing protocol file: {PROTO_FILE}")

    nanopb_root = ensure_nanopb()
    nanopb_protoc = nanopb_root / "generator" / "protoc"
    try:
        if nanopb_protoc.exists():
            generate_with_nanopb_protoc(nanopb_root, with_c=with_c)
        else:
            generate_with_system_protoc(nanopb_root)
    except Exception:
        # Final fallback: system protoc for Python bindings.
        generate_with_system_protoc(nanopb_root)

    protocol_py = PY_OUTPUT_DIR / "protocol_pb2.py"
    nanopb_py = PY_OUTPUT_DIR / "nanopb_pb2.py"
    if not protocol_py.exists() or not nanopb_py.exists():
        raise RuntimeError(
            f"Protobuf generation incomplete: {protocol_py.exists()=} {nanopb_py.exists()=}"
        )
    print("[update_protobuf.py] Protobuf bindings generated")
    print(f"  - {protocol_py}")
    print(f"  - {nanopb_py}")


if __name__ == "__main__":
    main()
