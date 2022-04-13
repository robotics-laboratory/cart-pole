from pathlib import Path
import subprocess
import sys

Import("env")

NANOPB_ROOT = Path(".pio/libdeps/esp32dev/Nanopb")
PROTO_FILE = Path("brain_controller.proto")
C_OUTPUT_DIR = "src"
PY_OUTPUT_DIR = "cartpole/device"

print("KEK" * 20)
subprocess.check_call([
    sys.executable, 
    NANOPB_ROOT / "generator" / "protoc",  
    "-I", NANOPB_ROOT / "generator" / "proto",
    "-I", PROTO_FILE.parent,
    # "-I.", 
    "--nanopb_out", C_OUTPUT_DIR, 
    "--python_out", PY_OUTPUT_DIR, 
    PROTO_FILE
])
