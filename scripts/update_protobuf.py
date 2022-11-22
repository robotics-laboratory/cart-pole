from pathlib import Path
import subprocess as sp
import sys

NANOPB_ROOT = Path(".pio/libdeps/esp32dev/Nanopb")
PROTO_FILE = Path("protocol.proto")
C_OUTPUT_DIR = Path("firmware/src")
PY_OUTPUT_DIR = Path("cartpole/device")

cmd = [
    f"{sys.executable}",
    f"{NANOPB_ROOT / 'generator' / 'protoc'}",
    f"-I {NANOPB_ROOT / 'generator' / 'proto'}",
    f"-I {PROTO_FILE.parent}",
    f"--nanopb_out {C_OUTPUT_DIR}",
    f"--python_out {PY_OUTPUT_DIR}",
    f"{NANOPB_ROOT / 'generator' / 'proto' / 'nanopb.proto'}",
    f"{PROTO_FILE}",
    "&&",
    # https://github.com/protocolbuffers/protobuf/issues/1491
    "sed -i 's/import nanopb_pb2/from . import nanopb_pb2/'",
    f"{PY_OUTPUT_DIR / 'protocol_pb2.py'}",
]

proc = sp.Popen(" ".join(cmd), shell=True, text=True, stdout=sp.PIPE, stderr=sp.PIPE)
assert proc.wait() == 0, f"Protobuf compilation failed:\n{proc.stderr.read()}"
print("[update_protobuf.py] Protobuf bindings generated")
