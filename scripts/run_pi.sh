#!/usr/bin/env bash
# One-command launcher for Raspberry Pi CartPole DDPG runtime.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

export PYTHONPATH="${PYTHONPATH:-$ROOT}"
export SERIAL_PORT="${SERIAL_PORT:-/dev/ttyUSB0}"
export SERIAL_SPEED="${SERIAL_SPEED:-500000}"
CHECKPOINT="${CHECKPOINT:-outputs/ddpg_simba_small_push_seed42/ddpg_model.pt}"
THETA_OFFSET="${THETA_OFFSET:-3.141592653589793}"
OUT_DIR="${OUT_DIR:-outputs/ddpg_phys_eval}"

ensure_protobuf() {
  if [[ ! -f cartpole/device/protocol_pb2.py || ! -f cartpole/device/nanopb_pb2.py ]]; then
    echo "[run_pi] Generating protobuf Python bindings..."
    python scripts/update_protobuf.py --python-only
  fi
}

# Always ensure bindings exist when the repo is bind-mounted over the image.
if [[ "${1:-}" != "help" && "${1:-}" != "-h" && "${1:-}" != "--help" ]]; then
  ensure_protobuf
fi

usage() {
  cat <<'EOF'
CartPole DDPG Raspberry Pi launcher

Usage:
  ./scripts/run_pi.sh help
  ./scripts/run_pi.sh doctor
  ./scripts/run_pi.sh dry-run
  ./scripts/run_pi.sh hardware [extra args...]
  ./scripts/run_pi.sh train [extra args...]
  ./scripts/run_pi.sh shell

Environment:
  SERIAL_PORT   USB serial device (default /dev/ttyUSB0)
  SERIAL_SPEED  Baud rate (default 500000)
  CHECKPOINT    Path to .pt checkpoint
  THETA_OFFSET  Angle offset (default pi)
  OUT_DIR       Hardware eval output directory
EOF
}

cmd_doctor() {
  echo "== CartPole environment doctor =="
  python - <<'PY'
import importlib
import platform
import sys

mods = [
    "torch",
    "numpy",
    "yaml",
    "gymnasium",
    "pydantic",
    "serial",
    "google.protobuf",
    "mcap",
    "cartpole.device.protocol_pb2",
    "cartpole.simulator",
    "cartpole.log",
]
print(f"python={sys.version.split()[0]} machine={platform.machine()}")
failed = []
for name in mods:
    try:
        mod = importlib.import_module(name)
        version = getattr(mod, "__version__", "ok")
        print(f"OK  {name} ({version})")
    except Exception as exc:  # noqa: BLE001
        failed.append((name, exc))
        print(f"FAIL {name}: {exc}")
if failed:
    raise SystemExit(1)
print("doctor: all required imports succeeded")
PY
  if [[ -e "$SERIAL_PORT" ]]; then
    echo "OK  serial device exists: $SERIAL_PORT"
  else
    echo "WARN serial device not found: $SERIAL_PORT (needed for hardware)"
  fi
  if [[ -f "$CHECKPOINT" ]]; then
    echo "OK  checkpoint: $CHECKPOINT"
  else
    echo "WARN checkpoint missing: $CHECKPOINT"
  fi
}

cmd_dry_run() {
  if [[ "${1:-}" == "--" ]]; then shift; fi
  python scripts/ddpg_phys_eval/eval_one_episode.py \
    --checkpoint "$CHECKPOINT" \
    --theta-offset "$THETA_OFFSET" \
    --out-dir "$OUT_DIR" \
    --dry-run \
    "$@"
}

cmd_hardware() {
  if [[ "${1:-}" == "--" ]]; then shift; fi
  if [[ ! -e "$SERIAL_PORT" ]]; then
    echo "ERROR: SERIAL_PORT does not exist: $SERIAL_PORT" >&2
    echo "Connect the ESP32 USB-UART adapter and update .env" >&2
    exit 1
  fi
  echo "Starting hardware episode on $SERIAL_PORT @ $SERIAL_SPEED"
  echo "Checkpoint: $CHECKPOINT"
  echo "Press Ctrl+C to abort. Homing may take up to ~30s."
  python scripts/ddpg_phys_eval/eval_one_episode.py \
    --checkpoint "$CHECKPOINT" \
    --serial-port "$SERIAL_PORT" \
    --serial-speed "$SERIAL_SPEED" \
    --theta-offset "$THETA_OFFSET" \
    --out-dir "$OUT_DIR" \
    "$@"
}

cmd_train() {
  if [[ "${1:-}" == "--" ]]; then shift; fi
  python scripts/ddpg/train.py \
    --config scripts/ddpg/config_simba_small_push.yaml \
    --device cpu \
    "$@"
}

cmd="${1:-help}"
shift || true

case "$cmd" in
  help|-h|--help)
    usage
    ;;
  doctor)
    cmd_doctor "$@"
    ;;
  dry-run)
    cmd_dry_run "$@"
    ;;
  hardware)
    cmd_hardware "$@"
    ;;
  train)
    cmd_train "$@"
    ;;
  shell)
    exec bash "$@"
    ;;
  *)
    echo "Unknown command: $cmd" >&2
    usage
    exit 1
    ;;
esac
