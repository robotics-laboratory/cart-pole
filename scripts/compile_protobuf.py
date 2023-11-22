from pathlib import Path
import subprocess as sp
import logging
import sys
import os

PROJECT_ROOT = Path(__file__).parent.parent
NANOPB_ROOT = PROJECT_ROOT / Path(".pio/libdeps/main/Nanopb")
PROTO_FILE = PROJECT_ROOT / Path("protocol.proto")
C_OUTPUT_DIR = PROJECT_ROOT / Path("firmware/src/proto")
PY_OUTPUT_DIR = PROJECT_ROOT / Path("cartpole/device/proto")

LOGGING_LEVEL = os.environ.get("LOGGING_LEVEL", "INFO").upper()
LOGGING_FORMAT = "%(levelname)s | %(filename)s:%(lineno)d :: %(message)s"


def run_protoc(args):
    args = [f"{sys.executable}", "-m", "grpc_tools.protoc", *args]
    logging.debug(f"Running cmd: {args}")
    proc = sp.Popen(args, text=True, stdout=sp.PIPE, stderr=sp.PIPE)
    if proc.wait() != 0:
        logging.error(f"Protobuf compilation failed:\n{proc.stderr.read()}")
        exit(1)


def main():
    logging.basicConfig(stream=sys.stdout, level=LOGGING_LEVEL, format=LOGGING_FORMAT)
    logging.debug(f"Project root: {PROJECT_ROOT}")
    logging.debug(f"Nanopb root: {NANOPB_ROOT}")
    logging.debug(f"Proto file: {PROTO_FILE}")
    logging.debug(f"C output path: {C_OUTPUT_DIR}")
    logging.debug(f"PY output path: {PY_OUTPUT_DIR}")

    if not NANOPB_ROOT.exists():
        logging.error(f"Failed to locate nanopb library at {NANOPB_ROOT}")
        logging.error("To initialize platformio, run 'pio pkg install'")
        exit(1)

    C_OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    PY_OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    nanopb_args = [
        f"--plugin=protoc-gen-nanopb={NANOPB_ROOT / 'generator' / 'protoc-gen-nanopb'}",
        f"--nanopb_out={C_OUTPUT_DIR}",
        f"-I{NANOPB_ROOT / 'generator' / 'proto'}",
        f"-I{PROTO_FILE.parent}",
        f"{PROTO_FILE}",
    ]
    run_protoc(nanopb_args)

    betterproto_args = [
        "--python_betterproto_opt=pydantic_dataclasses",
        f"--python_betterproto_out={PY_OUTPUT_DIR}",
        f"-I{NANOPB_ROOT / 'generator' / 'proto'}",
        f"-I{PROTO_FILE.parent}",
        f"{PROTO_FILE}",
    ]
    run_protoc(betterproto_args)

    logging.info("Protobuf bindings generated")


if __name__ == "__main__":
    main()
