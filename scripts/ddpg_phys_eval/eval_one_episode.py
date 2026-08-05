import argparse
import csv
import json
import math
import os
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np
import torch
from pydantic import BaseModel

REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPTS_DIR = REPO_ROOT / "scripts"
DDPG_DIR = SCRIPTS_DIR / "ddpg"

sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(SCRIPTS_DIR))
sys.path.insert(0, str(DDPG_DIR))

from cartpole import log  # noqa: E402
from cartpole.common.rl_types import State as CommonState  # noqa: E402
from cartpole.device import CartPoleDevice  # noqa: E402
from cartpole.device.wire_interface import DeviceConfig  # noqa: E402
from train import (  # noqa: E402
    angle_error_to_upright,
    append_history,
    create_agent_and_buffer,
    empty_history,
    get_device,
    load_config,
    make_env,
    set_seed,
)


DEFAULT_CHECKPOINT = (
    REPO_ROOT / "outputs" / "ddpg_simba_small_push_seed42" / "ddpg_model.pt"
)
DEFAULT_CONFIG = (
    REPO_ROOT / "outputs" / "ddpg_simba_small_push_seed42" / "config_used.yaml"
)
DEFAULT_OUT_DIR = REPO_ROOT / "outputs" / "ddpg_phys_eval"


class DDPGPhysEvalStep(BaseModel):
    episode: int
    step: int
    return_so_far: float
    reward: float
    action: float
    acceleration: float
    upright_error: float
    is_upright: bool
    terminated: bool
    truncated: bool
    stop_reason: str
    theta_raw: float
    theta: float
    theta_offset: float
    accelerometer_value: float
    motor_angle: float
    motor_velocity: float


def hardware_checkpoint_warning(checkpoint_path: Path) -> str:
    if checkpoint_path.name == "best_ddpg_model.pt":
        return ""
    return (
        f"{checkpoint_path} is not eligibility-selected. "
        "Running this checkpoint on hardware is allowed for diagnostics, "
        "but its behavior may be unsafe or unsuccessful."
    )


def load_checkpoint_and_config(args: argparse.Namespace) -> tuple[dict[str, Any], dict[str, Any]]:
    checkpoint_path = Path(args.checkpoint)
    if not checkpoint_path.exists():
        raise FileNotFoundError(f"Checkpoint not found: {checkpoint_path}")

    try:
        checkpoint = torch.load(checkpoint_path, map_location="cpu", weights_only=False)
    except TypeError:
        checkpoint = torch.load(checkpoint_path, map_location="cpu")
    if args.config:
        config = load_config(args.config)
    elif checkpoint.get("config") is not None:
        config = checkpoint["config"]
    elif DEFAULT_CONFIG.exists():
        config = load_config(DEFAULT_CONFIG)
    else:
        raise FileNotFoundError(
            "No config in checkpoint and default config_used.yaml is missing. "
            "Pass --config explicitly."
        )
    # Raspberry Pi / Docker runtime is CPU-only.
    config.setdefault("device", {})
    config["device"]["use_cuda"] = False
    return checkpoint, config


def load_agent(checkpoint: dict[str, Any], config: dict[str, Any], device: torch.device):
    env = make_env(config)
    try:
        agent, _replay_buffer, history_shape = create_agent_and_buffer(config, env, device)
    finally:
        env.close()

    agent.load_state_dict(checkpoint["model_state_dict"])
    agent.eval()
    return agent, history_shape


def make_reference_env(config: dict[str, Any]):
    env = make_env(config)
    return env


def state_to_obs(state, theta_offset: float) -> tuple[np.ndarray, float]:
    theta = float(state.pole_angle) + theta_offset
    obs = np.array(
        [
            float(state.cart_position),
            float(state.cart_velocity),
            math.sin(theta),
            math.cos(theta),
            float(state.pole_angular_velocity),
        ],
        dtype=np.float32,
    )
    return obs, theta


def device_state_to_common_state(state, *, theta: float, stamp: float) -> CommonState:
    return CommonState(
        cart_position=float(state.cart_position),
        cart_velocity=float(state.cart_velocity),
        cart_acceleration=float(getattr(state, "cart_acceleration", 0.0)),
        pole_angle=float(theta),
        pole_angular_velocity=float(state.pole_angular_velocity),
        stamp=float(stamp),
        error=int(state.error),
    )


def reward_for_state(
    *,
    reference_env,
    x: float,
    x_dot: float,
    theta: float,
    theta_dot: float,
    acceleration: float,
    previous_acceleration: float,
    error: int,
) -> float:
    state = CommonState(
        cart_position=float(x),
        cart_velocity=float(x_dot),
        cart_acceleration=float(acceleration),
        pole_angle=float(theta),
        pole_angular_velocity=float(theta_dot),
        error=int(error),
    )
    return reference_env._reward(
        state,
        float(acceleration),
        previous_acceleration=float(previous_acceleration),
    )


def should_stop(state, *, x_limit: float, velocity_limit: float) -> tuple[bool, str]:
    if int(state.error):
        return True, f"device_error={int(state.error)}"
    if abs(float(state.cart_position)) > x_limit:
        return True, f"x_limit={x_limit}"
    if abs(float(state.cart_velocity)) > velocity_limit:
        return True, f"velocity_limit={velocity_limit}"
    return False, ""


def write_rows(csv_path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        return
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def make_hardware_eval_paths(
    out_dir: str | Path,
    checkpoint_path: str | Path,
    *,
    timestamp: datetime | None = None,
) -> tuple[Path, Path]:
    out_dir = Path(out_dir)
    checkpoint_stem = Path(checkpoint_path).stem
    timestamp = timestamp or datetime.now(timezone.utc)
    timestamp_token = timestamp.astimezone(timezone.utc).strftime(
        "%Y%m%dT%H%M%S_%fZ"
    )
    stem = f"eval_{checkpoint_stem}_{timestamp_token}"
    return out_dir / f"{stem}.csv", out_dir / f"{stem}.json"


def write_eval_metadata(path: str | Path, payload: dict[str, Any]) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as file:
        json.dump(payload, file, indent=2, sort_keys=True)


def run_episode(args: argparse.Namespace) -> Path:
    checkpoint_warning = hardware_checkpoint_warning(
        Path(args.checkpoint)
    )
    if checkpoint_warning:
        print(f"WARNING: {checkpoint_warning}")
    checkpoint, config = load_checkpoint_and_config(args)
    set_seed(int(config["seed"]))

    torch_device = get_device(config)
    agent, history_shape = load_agent(checkpoint, config, torch_device)

    ref_env = make_reference_env(config)
    max_acceleration = float(args.max_acceleration or ref_env.max_acceleration)
    x_limit = float(args.x_limit or ref_env.config.control_limit.cart_position)
    velocity_limit = float(args.velocity_limit or ref_env.config.control_limit.cart_velocity)
    dt = float(args.dt or ref_env.dt)
    max_episode_time = float(args.max_episode_time or ref_env.max_episode_time)

    reward_for_state(
        reference_env=ref_env,
        x=0.0,
        x_dot=0.0,
        theta=math.pi,
        theta_dot=0.0,
        acceleration=0.0,
        previous_acceleration=0.0,
        error=0,
    )

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path, metadata_path = make_hardware_eval_paths(
        out_dir,
        args.checkpoint,
    )
    metadata = {
        "checkpoint": str(Path(args.checkpoint).resolve()),
        "config_source": (
            str(Path(args.config).resolve())
            if args.config
            else (
                "checkpoint"
                if checkpoint.get("config") is not None
                else str(DEFAULT_CONFIG.resolve())
            )
        ),
        "rollout": str(csv_path.resolve()),
        "theta_offset": float(args.theta_offset),
        "dt": dt,
        "max_episode_time": max_episode_time,
        "max_acceleration": max_acceleration,
        "x_limit": x_limit,
        "velocity_limit": velocity_limit,
    }
    write_eval_metadata(metadata_path, metadata)

    device_config = DeviceConfig(
        max_position=float(args.device_max_position),
        max_velocity=float(args.device_max_velocity),
        max_acceleration=max_acceleration,
        hard_max_position=float(args.device_hard_max_position),
        hard_max_velocity=float(args.device_hard_max_velocity),
        hard_max_acceleration=float(args.device_hard_max_acceleration),
        clamp_position=bool(args.clamp_position),
        clamp_velocity=bool(args.clamp_velocity),
        clamp_acceleration=bool(args.clamp_acceleration),
    )

    print(f"checkpoint={Path(args.checkpoint).resolve()}")
    serial_port = args.serial_port or os.environ.get("SERIAL_PORT", "")
    serial_speed = str(args.serial_speed or os.environ.get("SERIAL_SPEED", "500000"))
    print(f"serial={serial_port or '<unset>'} speed={serial_speed}")
    if args.mcap:
        print(f"mcap={Path(args.mcap).resolve()}")
    print(
        "limits: "
        f"dt={dt:.3f}s duration={max_episode_time:.2f}s "
        f"max_acceleration={max_acceleration:.2f} x_limit={x_limit:.3f} "
        f"velocity_limit={velocity_limit:.3f}"
    )

    if args.dry_run:
        ref_env.close()
        print("dry-run: model loaded; hardware episode was not started")
        return csv_path

    if args.serial_port:
        os.environ["SERIAL_PORT"] = args.serial_port
    if args.serial_speed:
        os.environ["SERIAL_SPEED"] = str(args.serial_speed)
    if not os.environ.get("SERIAL_PORT"):
        ref_env.close()
        raise RuntimeError("Set SERIAL_PORT or pass --serial-port.")
    if not os.environ.get("SERIAL_SPEED"):
        os.environ["SERIAL_SPEED"] = "500000"

    try:
        device = CartPoleDevice()
    except Exception:
        ref_env.close()
        raise
    history = empty_history(history_shape)
    rows: list[dict[str, Any]] = []
    episode_return = 0.0
    stop_reason = ""
    truncated = False
    terminated = False
    use_mcap = bool(args.mcap)
    final_stamp = 0.0
    previous_acceleration = 0.0

    try:
        if use_mcap:
            log.setup(log_path=args.mcap, level=log.Level.INFO)
            log.log("DDPG physical eval started", stamp=0.0, level=log.Level.INFO)

        time.sleep(float(args.startup_delay))
        device.reset(device_config)
        time.sleep(float(args.post_reset_delay))

        start = time.perf_counter()
        next_tick = start
        max_steps = int(math.ceil(max_episode_time / dt))

        for step_idx in range(max_steps):
            now = time.perf_counter()
            if now < next_tick:
                time.sleep(next_tick - now)

            t = time.perf_counter() - start
            final_stamp = t
            state = device.get_state()
            obs, theta = state_to_obs(state, float(args.theta_offset))
            action = agent.act(obs, history=history, explore=False)
            action_value = float(np.clip(np.asarray(action).reshape(-1)[0], -1.0, 1.0))
            acceleration = action_value * max_acceleration

            reward = reward_for_state(
                reference_env=ref_env,
                x=float(state.cart_position),
                x_dot=float(state.cart_velocity),
                theta=theta,
                theta_dot=float(state.pole_angular_velocity),
                acceleration=acceleration,
                previous_acceleration=previous_acceleration,
                error=int(state.error),
            )
            device.set_target(acceleration)
            previous_acceleration = acceleration
            episode_return += reward

            terminated, stop_reason = should_stop(
                state,
                x_limit=x_limit,
                velocity_limit=velocity_limit,
            )
            truncated = t >= max_episode_time or step_idx == max_steps - 1
            if truncated and not stop_reason:
                stop_reason = "timeout"
            angle_error = angle_error_to_upright(theta)
            is_upright = angle_error < float(config["eval"]["upright_error_threshold"])

            rows.append(
                {
                    "episode": 0,
                    "step": step_idx,
                    "t": t,
                    "return_so_far": episode_return,
                    "reward": reward,
                    "terminated": int(terminated),
                    "truncated": int(truncated),
                    "stop_reason": stop_reason,
                    "action": action_value,
                    "acceleration": acceleration,
                    "upright_error": angle_error,
                    "is_upright": int(is_upright),
                    "x": float(state.cart_position),
                    "x_dot": float(state.cart_velocity),
                    "x_ddot": float(getattr(state, "cart_acceleration", 0.0)),
                    "theta_raw": float(state.pole_angle),
                    "theta": theta,
                    "theta_dot": float(state.pole_angular_velocity),
                    "error": int(state.error),
                    "accelerometer_value": float(getattr(state, "accelerometer_value", 0.0) or 0.0),
                    "motor_angle": float(getattr(state, "motor_angle", 0.0) or 0.0),
                    "motor_velocity": float(getattr(state, "motor_velocity", 0.0) or 0.0),
                }
            )

            if use_mcap:
                log.publish(
                    "/cartpole/state",
                    device_state_to_common_state(state, theta=theta, stamp=t),
                    t,
                )
                log.publish(
                    "/ddpg/phys_eval_step",
                    DDPGPhysEvalStep(
                        episode=0,
                        step=step_idx,
                        return_so_far=episode_return,
                        reward=reward,
                        action=action_value,
                        acceleration=acceleration,
                        upright_error=angle_error,
                        is_upright=is_upright,
                        terminated=terminated,
                        truncated=truncated,
                        stop_reason=stop_reason,
                        theta_raw=float(state.pole_angle),
                        theta=theta,
                        theta_offset=float(args.theta_offset),
                        accelerometer_value=float(getattr(state, "accelerometer_value", 0.0) or 0.0),
                        motor_angle=float(getattr(state, "motor_angle", 0.0) or 0.0),
                        motor_velocity=float(getattr(state, "motor_velocity", 0.0) or 0.0),
                    ),
                    t,
                )

            if step_idx % max(int(args.print_every), 1) == 0:
                print(
                    f"step={step_idx:04d} t={t:5.2f} x={state.cart_position:+.3f} "
                    f"theta={theta:+.3f} action={action_value:+.3f} "
                    f"acc={acceleration:+.3f} return={episode_return:+.2f}"
                )

            history = append_history(history, obs, np.array([action_value], dtype=np.float32))

            if terminated or truncated:
                break

            next_tick += dt
    finally:
        try:
            device.set_target(0.0)
            time.sleep(float(args.stop_settle))
        finally:
            device.close()
            ref_env.close()
            if use_mcap:
                log.log("DDPG physical eval stopped", stamp=final_stamp, level=log.Level.INFO)
                log.close()
        write_rows(csv_path, rows)
        metadata.update(
            {
                "samples": len(rows),
                "terminated": bool(terminated),
                "truncated": bool(truncated),
                "stop_reason": stop_reason,
            }
        )
        write_eval_metadata(metadata_path, metadata)

    length = len(rows)
    upright_fraction = (
        float(np.mean([row["is_upright"] for row in rows])) if rows else 0.0
    )
    print(
        f"episode=0 return={episode_return:.2f} len={length} "
        f"upright_fraction={upright_fraction:.2f} "
        f"terminated={int(terminated)} truncated={int(truncated)}"
    )
    if stop_reason:
        print(f"stop_reason={stop_reason}")
    print(f"Saved rollout to {csv_path.resolve()}")
    print(f"Saved metadata to {metadata_path.resolve()}")
    return csv_path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", default=str(DEFAULT_CHECKPOINT))
    parser.add_argument("--config", default="")
    parser.add_argument("--out-dir", default=str(DEFAULT_OUT_DIR))
    parser.add_argument("--mcap", default="")
    parser.add_argument("--serial-port", default="")
    parser.add_argument("--serial-speed", type=int, default=500000)
    parser.add_argument("--dt", type=float, default=None)
    parser.add_argument("--max-episode-time", type=float, default=None)
    parser.add_argument("--max-acceleration", type=float, default=None)
    parser.add_argument("--x-limit", type=float, default=None)
    parser.add_argument("--velocity-limit", type=float, default=None)
    parser.add_argument("--theta-offset", type=float, default=math.pi)
    parser.add_argument("--startup-delay", type=float, default=3.0)
    parser.add_argument("--post-reset-delay", type=float, default=0.2)
    parser.add_argument("--stop-settle", type=float, default=0.2)
    parser.add_argument("--print-every", type=int, default=10)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--clamp-position", action="store_true")
    parser.add_argument("--clamp-velocity", action="store_true")
    parser.add_argument("--clamp-acceleration", action="store_true")
    parser.add_argument("--device-max-position", type=float, default=0.25)
    parser.add_argument("--device-max-velocity", type=float, default=2.0)
    parser.add_argument("--device-hard-max-position", type=float, default=0.27)
    parser.add_argument("--device-hard-max-velocity", type=float, default=2.5)
    parser.add_argument("--device-hard-max-acceleration", type=float, default=5.0)
    return parser.parse_args()


def main() -> None:
    run_episode(parse_args())


if __name__ == "__main__":
    main()
