import argparse
import copy
import csv
import math
import sys
from pathlib import Path
from typing import Any

import numpy as np
import torch
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from cartpole_rl_env import CartPoleRLEnv

from model import DDPG, RAM_ReplayBuffer, convert_step_output_to_experience


CONFIG_PATH = Path(__file__).with_name("config.yaml")


DEFAULT_CONFIG: dict[str, Any] = {
    "run_name": "ddpg_transformer_swingup",
    "seed": 42,
    "device": {
        "use_cuda": False,
    },
    "env": {
        "dt": 0.02,
        "max_episode_time": 10.0,
        "random_initial_angle": True,
        "initial_angle": 0.0,
        "initial_angle_center": math.pi,
        "initial_angle_range": 0.10,
        "random_initial_cart": True,
        "initial_cart_position_range": 0.03,
        "initial_cart_velocity_range": 0.05,
        "initial_pole_angular_velocity_range": 0.0,
        "reward_mode": "swingup_soft",
        "terminate_on_control_limit": True,
        "noisy_physics_enabled": False,
        "physics_noise_low": 0.8,
        "physics_noise_high": 1.2,
        "action_delay_min_steps": 0,
        "action_delay_max_steps": 0,
        "theta_dot_penalty": 0.0,
        "center_penalty": 0.0,
        "action_delta_penalty": 0.0,
    },
    "curriculum": {
        "enabled": True,
        "mode": "linear",
        "start_angle_range": 0.10,
        "max_angle_range": math.pi,
        "linear_increment_per_episode": 0.002,
        "success_threshold": 0.75,
        "success_increment": 0.15,
        "initial_state": {
            "enabled": False,
        },
        "physics_noise": {
            "enabled": False,
        },
        "action_latency": {
            "enabled": False,
        },
    },
    "model": {
        "model_type": "transformer",
        "hidden_dims": [256, 256],
        "gamma": 0.995,
        "target_exponential_averaging": 0.995,
        "exploration_std": 0.35,
        "min_exploration_std": 0.05,
        "exploration_decay": 0.995,
        "simba": {
            "actor_hidden_dim": 128,
            "critic_hidden_dim": 256,
            "actor_num_blocks": 1,
            "critic_num_blocks": 2,
            "history_len": 1,
            "include_action_history": True,
            "rsnorm_eps": 0.00001,
        },
    },
    "history": {
        "enabled": True,
        "length": 8,
        "transformer": {
            "d_model": 32,
            "n_heads": 2,
            "n_layers": 1,
            "dropout": 0.0,
        },
    },
    "train": {
        "n_episodes": 5000,
        "batch_size": 256,
        "max_buffer_size": 200000,
        "random_warmup_steps": 15000,
        "preheat_steps": 0,
        "updates_per_step": 1,
        "actor_update_frequency": 2,
        "actor_lr": 0.0003,
        "critic_lr": 0.0003,
        "weight_decay": 0.0,
        "max_grad_norm": 10.0,
        "log_every": 10,
        "eval_every": 100,
        "save_every": 100,
        "out_dir": "outputs/ddpg",
    },
    "eval": {
        "n_episodes": 10,
        "push_episodes": 0,
        "upright_error_threshold": 0.25,
        "stable_x_threshold": 0.20,
        "required_upright_time": 2.0,
        "max_post_acquisition_rotations": 0.75,
        "out_dir": "outputs/ddpg_eval",
    },
    "logging": {
        "wandb": False,
        "wandb_project": "Cartpole-DDPG",
    },
}


def deep_update(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    output = dict(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(output.get(key), dict):
            output[key] = deep_update(output[key], value)
        else:
            output[key] = value
    return output


def load_config(path: str | Path = CONFIG_PATH) -> dict[str, Any]:
    path = Path(path)
    with path.open("r", encoding="utf-8") as f:
        user_config = yaml.safe_load(f) or {}
    return deep_update(DEFAULT_CONFIG, user_config)


def save_config(config: dict[str, Any], out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    with (out_dir / "config_used.yaml").open("w", encoding="utf-8") as f:
        yaml.safe_dump(config, f, sort_keys=False, allow_unicode=True)


def set_seed(seed: int) -> None:
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)


def get_device(config: dict[str, Any]) -> torch.device:
    use_cuda = bool(config["device"].get("use_cuda", False))
    return torch.device("cuda:0" if torch.cuda.is_available() and use_cuda else "cpu")


def make_env(config: dict[str, Any]) -> CartPoleRLEnv:
    env_config = config["env"]
    return CartPoleRLEnv(
        dt=float(env_config["dt"]),
        max_episode_time=float(env_config["max_episode_time"]),
        random_initial_angle=bool(env_config["random_initial_angle"]),
        initial_angle=float(env_config["initial_angle"]),
        initial_angle_center=float(env_config["initial_angle_center"]),
        initial_angle_range=float(env_config["initial_angle_range"]),
        random_initial_cart=bool(env_config["random_initial_cart"]),
        initial_cart_position_range=float(env_config["initial_cart_position_range"]),
        initial_cart_velocity_range=float(env_config["initial_cart_velocity_range"]),
        initial_pole_angular_velocity_range=float(
            env_config.get("initial_pole_angular_velocity_range", 0.0)
        ),
        reward_mode=str(env_config["reward_mode"]),
        terminate_on_control_limit=bool(env_config["terminate_on_control_limit"]),
        noisy_physics_enabled=bool(env_config.get("noisy_physics_enabled", False)),
        physics_noise_low=float(env_config.get("physics_noise_low", 0.8)),
        physics_noise_high=float(env_config.get("physics_noise_high", 1.2)),
        physics_randomization=env_config.get("physics_randomization"),
        observation_noise_std=env_config.get("observation_noise_std"),
        action_delay_min_steps=int(env_config.get("action_delay_min_steps", 0)),
        action_delay_max_steps=int(env_config.get("action_delay_max_steps", 0)),
        initial_state_mix=env_config.get("initial_state_mix"),
        disturbance=env_config.get("disturbance"),
        theta_dot_penalty=float(env_config.get("theta_dot_penalty", 0.0)),
        center_penalty=float(env_config.get("center_penalty", 0.0)),
        action_delta_penalty=float(env_config.get("action_delta_penalty", 0.0)),
        legacy_theta_dot_penalty=float(
            env_config.get("legacy_theta_dot_penalty", 0.01)
        ),
        legacy_action_penalty=float(
            env_config.get("legacy_action_penalty", 0.002)
        ),
        legacy_action_delta_penalty=float(
            env_config.get("legacy_action_delta_penalty", 0.0)
        ),
        legacy_action_saturation_threshold=float(
            env_config.get("legacy_action_saturation_threshold", 1.0)
        ),
        legacy_action_saturation_penalty=float(
            env_config.get("legacy_action_saturation_penalty", 0.0)
        ),
    )


def get_curriculum_angle_range(config: dict[str, Any], episode: int) -> float:
    curriculum_config = config.get("curriculum", {})
    env_config = config["env"]
    if not curriculum_config.get("enabled", False):
        return float(env_config["initial_angle_range"])

    start_range = float(curriculum_config["start_angle_range"])
    max_range = float(curriculum_config["max_angle_range"])
    mode = str(curriculum_config.get("mode", "linear"))

    if mode == "linear":
        increment = float(curriculum_config["linear_increment_per_episode"])
        return min(max_range, start_range + increment * max(episode - 1, 0))

    if mode == "success":
        return float(env_config["initial_angle_range"])

    raise ValueError(f"Unknown curriculum mode: {mode}")


def _linear_schedule(
    *,
    episode: int,
    start_episode: int,
    end_episode: int,
    start_value: float,
    end_value: float,
) -> float:
    if end_episode <= start_episode:
        return float(end_value)
    progress = (episode - start_episode) / (end_episode - start_episode)
    progress = min(max(progress, 0.0), 1.0)
    return float(start_value + progress * (end_value - start_value))


def _scheduled_float(section: dict[str, Any], key: str, default: float, episode: int) -> float:
    if not section.get("enabled", False):
        return float(default)
    start = float(section.get(f"{key}_start", default))
    end = float(section.get(f"{key}_end", start))
    return _linear_schedule(
        episode=episode,
        start_episode=int(section.get("start_episode", 1)),
        end_episode=int(section.get("end_episode", 1)),
        start_value=start,
        end_value=end,
    )


def _scheduled_int(section: dict[str, Any], key: str, default: int, episode: int) -> int:
    return int(round(_scheduled_float(section, key, float(default), episode)))


def generate_state_curriculum(config: dict[str, Any], episode: int) -> dict[str, Any]:
    env_config = config["env"]
    curriculum_config = config.get("curriculum", {})
    curriculum_enabled = bool(curriculum_config.get("enabled", False))
    initial_state_config = curriculum_config.get("initial_state", {})
    physics_config = curriculum_config.get("physics_noise", {})
    latency_config = curriculum_config.get("action_latency", {})

    params = {
        "initial_angle_range": get_curriculum_angle_range(config, episode),
        "initial_cart_position_range": float(env_config["initial_cart_position_range"]),
        "initial_cart_velocity_range": float(env_config["initial_cart_velocity_range"]),
        "initial_pole_angular_velocity_range": float(
            env_config.get("initial_pole_angular_velocity_range", 0.0)
        ),
        "noisy_physics_enabled": bool(env_config.get("noisy_physics_enabled", False)),
        "physics_noise_low": float(env_config.get("physics_noise_low", 0.8)),
        "physics_noise_high": float(env_config.get("physics_noise_high", 1.2)),
        "action_delay_min_steps": int(env_config.get("action_delay_min_steps", 0)),
        "action_delay_max_steps": int(env_config.get("action_delay_max_steps", 0)),
        "initial_state_mix_progress": 0.0,
    }

    if curriculum_enabled and initial_state_config.get("enabled", False):
        params["initial_state_mix_progress"] = _linear_schedule(
            episode=episode,
            start_episode=int(initial_state_config.get("start_episode", 1)),
            end_episode=int(initial_state_config.get("end_episode", 1)),
            start_value=0.0,
            end_value=1.0,
        )
        params["initial_cart_position_range"] = _scheduled_float(
            initial_state_config,
            "cart_position_range",
            params["initial_cart_position_range"],
            episode,
        )
        params["initial_cart_velocity_range"] = _scheduled_float(
            initial_state_config,
            "cart_velocity_range",
            params["initial_cart_velocity_range"],
            episode,
        )
        params["initial_pole_angular_velocity_range"] = _scheduled_float(
            initial_state_config,
            "theta_dot_range",
            params["initial_pole_angular_velocity_range"],
            episode,
        )

    if curriculum_enabled and physics_config.get("enabled", False):
        params["noisy_physics_enabled"] = True
        params["physics_noise_low"] = _scheduled_float(
            physics_config,
            "low",
            params["physics_noise_low"],
            episode,
        )
        params["physics_noise_high"] = _scheduled_float(
            physics_config,
            "high",
            params["physics_noise_high"],
            episode,
        )

    if curriculum_enabled and latency_config.get("enabled", False):
        params["action_delay_min_steps"] = _scheduled_int(
            latency_config,
            "min_delay_steps",
            params["action_delay_min_steps"],
            episode,
        )
        params["action_delay_max_steps"] = _scheduled_int(
            latency_config,
            "max_delay_steps",
            params["action_delay_max_steps"],
            episode,
        )
        params["action_delay_min_steps"] = max(0, params["action_delay_min_steps"])
        params["action_delay_max_steps"] = max(
            params["action_delay_min_steps"],
            params["action_delay_max_steps"],
        )

    return params


def get_hard_eval_params(config: dict[str, Any]) -> dict[str, Any]:
    curriculum = config.get("curriculum", {})
    final_episode = max(
        [int(config["train"]["n_episodes"])]
        + [
            int(section.get("end_episode", 1))
            for section in curriculum.values()
            if isinstance(section, dict)
        ]
    )
    params = generate_state_curriculum(config, episode=final_episode)
    if curriculum.get("enabled", False):
        params["initial_angle_range"] = float(curriculum["max_angle_range"])
        params["initial_state_mix_progress"] = 1.0
    return params


def apply_initial_angle_range(env: CartPoleRLEnv, angle_range: float) -> None:
    env.initial_angle_center = math.pi
    env.initial_angle_range = float(min(max(angle_range, 0.0), math.pi))
    env.random_initial_angle = True


def prepare_push_eval_episode(env: CartPoleRLEnv) -> np.ndarray:
    state = env.sim.get_state().model_copy(
        update={
            "cart_position": 0.0,
            "cart_velocity": 0.0,
            "cart_acceleration": 0.0,
            "pole_angle": math.pi,
            "pole_angular_velocity": 0.0,
        }
    )
    env.sim.reset(state)
    env._elapsed_time = 0.0
    env._last_acceleration = 0.0
    env._reset_disturbance_progress()
    return np.asarray(env._make_obs(env.sim.get_state()), dtype=np.float32)


def apply_episode_curriculum(env: CartPoleRLEnv, params: dict[str, Any]) -> None:
    apply_initial_angle_range(env, float(params["initial_angle_range"]))
    env.set_initial_state_ranges(
        cart_position_range=float(params["initial_cart_position_range"]),
        cart_velocity_range=float(params["initial_cart_velocity_range"]),
        pole_angular_velocity_range=float(params["initial_pole_angular_velocity_range"]),
    )
    env.set_physics_noise(
        enabled=bool(params["noisy_physics_enabled"]),
        low=float(params["physics_noise_low"]),
        high=float(params["physics_noise_high"]),
    )
    env.set_action_delay_range(
        min_steps=int(params["action_delay_min_steps"]),
        max_steps=int(params["action_delay_max_steps"]),
    )
    env.set_initial_state_mix_progress(float(params.get("initial_state_mix_progress", 0.0)))


def angle_error_to_upright(theta: float) -> float:
    return float(abs(math.atan2(math.sin(theta - math.pi), math.cos(theta - math.pi))))


def summarize_eval_episode(
    samples,
    *,
    dt: float,
    upright_error_threshold: float,
    stable_x_threshold: float,
    required_upright_time: float,
    max_post_acquisition_rotations: float,
    terminated: bool,
) -> dict[str, float]:
    upright_errors = [
        angle_error_to_upright(float(sample["theta"]))
        for sample in samples
    ]
    upright = [
        error < upright_error_threshold for error in upright_errors
    ]
    stable = [
        is_upright and abs(float(sample["x"])) < stable_x_threshold
        for sample, is_upright in zip(samples, upright)
    ]

    longest_run = current_run = 0
    for is_stable in stable:
        current_run = current_run + 1 if is_stable else 0
        longest_run = max(longest_run, current_run)

    acquisition_step = next((i for i, value in enumerate(stable) if value), None)
    rotation_count = 0.0
    if acquisition_step is not None:
        theta = [float(sample["theta"]) for sample in samples[acquisition_step:]]
        rotation_count = sum(abs(b - a) for a, b in zip(theta, theta[1:])) / (2.0 * math.pi)

    required_steps = max(1, int(math.ceil(required_upright_time / dt)))
    finishes_stable = len(stable) >= required_steps and all(stable[-required_steps:])
    disturbance_step = next(
        (i for i, sample in enumerate(samples) if sample.get("disturbance_applied", False)),
        None,
    )
    post_disturbance_rotations = 0.0
    maximum_unwrapped_excursion = 0.0
    recovery_step = None
    if disturbance_step is not None:
        disturbance_sample = samples[disturbance_step]
        theta = [float(sample["theta"]) for sample in samples[disturbance_step:]]
        post_disturbance_rotations = (
            sum(abs(b - a) for a, b in zip(theta, theta[1:])) / (2.0 * math.pi)
        )
        reference_theta = disturbance_sample.get("disturbance_reference_theta")
        excursion_theta = (
            [float(reference_theta), *theta]
            if reference_theta is not None
            else theta
        )
        unwrapped_theta = np.unwrap(np.asarray(excursion_theta, dtype=float))
        maximum_unwrapped_excursion = float(
            np.max(np.abs(unwrapped_theta - unwrapped_theta[0]))
        )
        for end in range(disturbance_step + required_steps, len(stable)):
            if all(stable[end - required_steps + 1:end + 1]):
                recovery_step = end
                break

    push_catch = (
        not terminated
        and disturbance_step is not None
        and recovery_step is not None
    )
    no_turn_recovery = (
        push_catch and maximum_unwrapped_excursion < 2.0 * math.pi
    )
    push_recovery_success = (
        not terminated
        and disturbance_step is not None
        and recovery_step is not None
        and post_disturbance_rotations <= max_post_acquisition_rotations
    )
    success = (
        not terminated
        and finishes_stable
        and rotation_count <= max_post_acquisition_rotations
    )
    upright_error_values = [
        error for error, is_upright in zip(upright_errors, upright) if is_upright
    ]
    upright_action_deltas = [
        float(current["action"]) - float(previous["action"])
        for previous, current, previous_upright, current_upright in zip(
            samples,
            samples[1:],
            upright,
            upright[1:],
        )
        if previous_upright
        and current_upright
        and "action" in previous
        and "action" in current
    ]
    theta_dots = [
        float(sample["theta_dot"]) for sample in samples if "theta_dot" in sample
    ]
    positions = [float(sample["x"]) for sample in samples]
    actions = [float(sample["action"]) for sample in samples if "action" in sample]
    action_deltas = [current - previous for previous, current in zip(actions, actions[1:])]
    saturated_actions = sum(abs(action) >= 0.999 for action in actions)
    return {
        "success": float(success),
        "termination": float(terminated),
        "upright_fraction": float(np.mean(upright)) if upright else 0.0,
        "longest_upright_time": longest_run * dt,
        "post_acquisition_rotation_count": rotation_count,
        "disturbance_applied": float(disturbance_step is not None),
        "push_recovery_success": float(push_recovery_success),
        "push_catch": float(push_catch),
        "no_turn_recovery": float(no_turn_recovery),
        "recovery_time": (
            (recovery_step - disturbance_step) * dt
            if recovery_step is not None and disturbance_step is not None
            else float("nan")
        ),
        "post_disturbance_rotation_count": post_disturbance_rotations,
        "maximum_unwrapped_angular_excursion": maximum_unwrapped_excursion,
        "upright_theta_rms": (
            float(np.sqrt(np.mean(np.square(upright_errors))))
            if upright_errors
            else math.pi
        ),
        "upright_theta_p95": (
            float(np.percentile(upright_errors, 95))
            if upright_errors
            else math.pi
        ),
        "theta_dot_rms": (
            float(np.sqrt(np.mean(np.square(theta_dots)))) if theta_dots else 0.0
        ),
        "mean_cart_position": float(np.mean(positions)) if positions else 0.0,
        "action_rms": (
            float(np.sqrt(np.mean(np.square(actions)))) if actions else 0.0
        ),
        "action_delta_rms": (
            float(np.sqrt(np.mean(np.square(action_deltas))))
            if action_deltas
            else 0.0
        ),
        "action_saturation_rate": (
            float(saturated_actions / len(actions)) if actions else 0.0
        ),
        "action_saturation_count": float(saturated_actions),
        "upright_error_rms": (
            float(np.sqrt(np.mean(np.square(upright_error_values))))
            if upright_error_values
            else math.pi
        ),
        "upright_action_delta_rms": (
            float(np.sqrt(np.mean(np.square(upright_action_deltas))))
            if upright_action_deltas
            else 2.0
        ),
    }


def build_named_eval_scenarios(
    config: dict[str, Any],
    *,
    base_seed: int,
    episodes_per_case: int = 20,
) -> list[dict[str, Any]]:
    final_params = get_hard_eval_params(config)
    episode_time = float(config["env"]["max_episode_time"])
    noisy_low = float(final_params["physics_noise_low"])
    noisy_high = float(final_params["physics_noise_high"])
    eval_config = config.get("eval", {})
    scenarios: list[dict[str, Any]] = []

    def add(
        name: str,
        *,
        initial_state: str,
        physics_noise_enabled: bool,
        max_episode_time: float,
        count: int,
        sign: int = 0,
        impulse: float | None = None,
        report_only: bool = False,
        physics_noise_low: float = noisy_low,
        physics_noise_high: float = noisy_high,
        action_delay_steps: int = 0,
    ) -> None:
        for _ in range(count):
            scenarios.append(
                {
                    "name": name,
                    "sign": sign,
                    "seed": base_seed + len(scenarios),
                    "initial_state": initial_state,
                    "max_episode_time": max_episode_time,
                    "physics_noise_enabled": physics_noise_enabled,
                    "physics_noise_low": physics_noise_low,
                    "physics_noise_high": physics_noise_high,
                    "fixed_delta_theta_dot": impulse,
                    "disturbance_upright_time": 1.0,
                    "disturbance_delay": 0.0,
                    "report_only": report_only,
                    "action_delay_steps": action_delay_steps,
                }
            )

    add(
        "swingup",
        initial_state="swingup",
        physics_noise_enabled=bool(final_params["noisy_physics_enabled"]),
        max_episode_time=episode_time,
        count=episodes_per_case,
    )
    add(
        "balance",
        initial_state="upright",
        physics_noise_enabled=False,
        max_episode_time=15.0,
        count=episodes_per_case,
    )
    delayed_push_magnitude = float(
        eval_config.get("delayed_push_magnitude", 1.0)
    )
    delayed_push_token = f"{delayed_push_magnitude:.1f}".replace(".", "p")
    for delay_steps in eval_config.get("delayed_action_delay_steps", []):
        delay_steps = int(delay_steps)
        add(
            f"balance_delay{delay_steps}",
            initial_state="upright",
            physics_noise_enabled=False,
            max_episode_time=15.0,
            count=episodes_per_case,
            action_delay_steps=delay_steps,
        )
        for sign in (-1, 1):
            add(
                f"push_delay{delay_steps}_{delayed_push_token}",
                initial_state="upright",
                physics_noise_enabled=True,
                physics_noise_low=0.8,
                physics_noise_high=1.2,
                max_episode_time=episode_time,
                count=episodes_per_case,
                sign=sign,
                impulse=sign * delayed_push_magnitude,
                action_delay_steps=delay_steps,
            )
    for name, magnitude, noisy, report_only in (
        ("push_nominal_0p5", 0.5, False, False),
        ("push_rank_1p0", 1.0, True, False),
        ("push_hard_1p5", 1.5, True, True),
        ("push_hard_2p0", 2.0, True, True),
        ("push_hard_2p5", 2.5, True, True),
    ):
        for sign in (-1, 1):
            add(
                name,
                initial_state="upright",
                physics_noise_enabled=noisy,
                physics_noise_low=0.8 if noisy else noisy_low,
                physics_noise_high=1.2 if noisy else noisy_high,
                max_episode_time=episode_time,
                count=episodes_per_case,
                sign=sign,
                impulse=sign * magnitude,
                report_only=report_only,
            )
    return scenarios


def named_checkpoint_is_eligible(
    metrics: dict[str, float],
    eval_config: dict[str, Any] | None = None,
) -> bool:
    eval_config = eval_config or {}
    reference_success_rate = eval_config.get(
        "reference_swingup_success_rate"
    )
    if reference_success_rate is None:
        min_swingup_success_rate = 0.90
    else:
        min_swingup_success_rate = max(
            0.0,
            float(reference_success_rate)
            - float(eval_config.get("swingup_success_tolerance", 0.0)),
        )
    eligible = (
        metrics["eval_balance_termination_rate"] == 0.0
        and metrics["eval_swingup_success_rate"] + 1e-12
        >= min_swingup_success_rate
        and metrics.get("eval_swingup_termination_rate", 0.0)
        <= float(eval_config.get("max_swingup_termination_rate", 1.0))
        and metrics.get("eval_swingup_theta_dot_rms", 0.0)
        <= float(eval_config.get("max_swingup_theta_dot_rms", math.inf))
        and metrics.get("eval_swingup_action_saturation_rate", 0.0)
        <= float(
            eval_config.get("max_swingup_action_saturation_rate", 1.0)
        )
        and metrics.get("eval_swingup_action_delta_rms", 0.0)
        <= float(eval_config.get("max_swingup_action_delta_rms", math.inf))
        and metrics["eval_balance_theta_rms"] <= math.radians(0.7)
        and metrics["eval_balance_theta_p95"] <= math.radians(1.5)
        and abs(metrics["eval_balance_mean_x"]) <= 0.05
        and metrics["eval_balance_action_rms"] <= 0.10
        and metrics["eval_balance_action_delta_rms"] <= 0.10
        and metrics["eval_balance_action_saturation_count"] == 0.0
    )
    if not eligible or not eval_config.get(
        "delayed_checkpoint_eligibility_enabled", False
    ):
        return eligible

    delayed_push_magnitude = float(
        eval_config.get("delayed_push_magnitude", 1.0)
    )
    delayed_push_token = f"{delayed_push_magnitude:.1f}".replace(".", "p")
    for delay_steps in eval_config.get("delayed_action_delay_steps", []):
        delay_steps = int(delay_steps)
        balance_prefix = f"eval_balance_delay{delay_steps}"
        push_prefix = (
            f"eval_push_delay{delay_steps}_{delayed_push_token}"
        )
        if (
            metrics[f"{balance_prefix}_termination_rate"] != 0.0
            or metrics[f"{push_prefix}_catch_rate"]
            < float(eval_config["delayed_min_catch_rate"])
            or metrics[f"{push_prefix}_no_turn_recovery_rate"]
            < float(eval_config["delayed_min_no_turn_recovery_rate"])
            or metrics[f"{push_prefix}_termination_rate"]
            > float(eval_config["delayed_max_termination_rate"])
        ):
            return False
    return True


def named_checkpoint_rank(
    metrics: dict[str, float],
    eval_config: dict[str, Any] | None = None,
) -> tuple[float, ...]:
    eval_config = eval_config or {}
    delayed_rank: list[float] = []
    if eval_config.get("delayed_checkpoint_eligibility_enabled", False):
        delayed_push_magnitude = float(
            eval_config.get("delayed_push_magnitude", 1.0)
        )
        delayed_push_token = f"{delayed_push_magnitude:.1f}".replace(".", "p")
        for delay_steps in eval_config.get(
            "delayed_action_delay_steps", []
        ):
            prefix = (
                f"eval_push_delay{int(delay_steps)}_{delayed_push_token}"
            )
            delayed_rank.extend(
                (
                    metrics[f"{prefix}_catch_rate"],
                    metrics[f"{prefix}_no_turn_recovery_rate"],
                    -metrics[f"{prefix}_termination_rate"],
                    -metrics[f"{prefix}_recovery_time_mean"],
                )
            )
    return (
        metrics["eval_swingup_success_rate"],
        -metrics.get("eval_swingup_termination_rate", 0.0),
        metrics["eval_push_rank_1p0_no_turn_recovery_rate"],
        -metrics.get("eval_push_rank_1p0_termination_rate", 0.0),
        metrics["eval_push_rank_1p0_catch_rate"],
        -metrics.get("eval_swingup_theta_dot_rms", 0.0),
        -metrics.get("eval_swingup_action_saturation_rate", 0.0),
        -metrics["eval_push_rank_1p0_recovery_time_mean"],
        -metrics["eval_balance_theta_rms"],
        -metrics["eval_balance_theta_p95"],
        -abs(metrics["eval_balance_mean_x"]),
        -metrics["eval_balance_action_rms"],
        -metrics["eval_balance_action_delta_rms"],
    ) + tuple(delayed_rank)


def consider_named_checkpoint(
    metrics: dict[str, float],
    best_rank: tuple[float, ...] | None,
    eval_config: dict[str, Any] | None = None,
) -> tuple[bool, tuple[float, ...] | None]:
    if not named_checkpoint_is_eligible(metrics, eval_config):
        return False, best_rank
    rank = named_checkpoint_rank(metrics, eval_config)
    if best_rank is None or rank > best_rank:
        return True, rank
    return False, best_rank


def consider_named_checkpoint_candidates(
    metrics: dict[str, float],
    *,
    best_candidate_rank: tuple[float, ...] | None,
    best_eligible_rank: tuple[float, ...] | None,
    eval_config: dict[str, Any] | None = None,
) -> tuple[
    bool,
    tuple[float, ...],
    bool,
    tuple[float, ...] | None,
]:
    eval_config = eval_config or {}
    rank = named_checkpoint_rank(metrics, eval_config)
    eligible = named_checkpoint_is_eligible(metrics, eval_config)
    candidate_rank = (float(eligible),) + rank
    save_candidate = (
        best_candidate_rank is None or candidate_rank > best_candidate_rank
    )
    next_candidate_rank = (
        candidate_rank if save_candidate else best_candidate_rank
    )
    save_eligible = eligible and (
        best_eligible_rank is None or rank > best_eligible_rank
    )
    next_eligible_rank = rank if save_eligible else best_eligible_rank
    return (
        save_candidate,
        next_candidate_rank,
        save_eligible,
        next_eligible_rank,
    )


def named_evaluation_enabled(config: dict[str, Any]) -> bool:
    eval_config = config.get("eval", {})
    return bool(
        eval_config.get("scenario_evaluation_enabled", False)
        and eval_config.get("checkpoint_eligibility_enabled", False)
    )


def update_named_eval_fail_fast(
    config: dict[str, Any],
    *,
    episode: int,
    metrics: dict[str, float],
    consecutive_failures: int,
) -> tuple[int, bool, str]:
    fail_fast = config.get("train", {}).get(
        "named_eval_fail_fast", {}
    )
    if (
        not fail_fast.get("enabled", False)
        or episode < int(fail_fast.get("start_episode", 0))
    ):
        return consecutive_failures, False, ""

    required_metrics = (
        "eval_swingup_success_rate",
        "eval_swingup_termination_rate",
    )
    if any(metric not in metrics for metric in required_metrics):
        return consecutive_failures, False, ""

    success_rate = float(metrics["eval_swingup_success_rate"])
    termination_rate = float(metrics["eval_swingup_termination_rate"])
    is_failed_eval = (
        success_rate
        <= float(fail_fast.get("max_swingup_success_rate", 0.0))
        and termination_rate
        >= float(fail_fast.get("min_swingup_termination_rate", 1.0))
    )
    if not is_failed_eval:
        return 0, False, ""

    consecutive_failures += 1
    patience = int(fail_fast.get("patience", 1))
    reason = (
        f"swingup_success_rate={success_rate:.3f}, "
        f"swingup_termination_rate={termination_rate:.3f}, "
        f"consecutive_bad_evals={consecutive_failures}/{patience}"
    )
    return consecutive_failures, consecutive_failures >= patience, reason


def consider_periodic_checkpoint(
    config: dict[str, Any],
    metrics: dict[str, float],
    best_score: tuple[float, ...] | None,
) -> tuple[bool, tuple[float, ...] | None]:
    if named_evaluation_enabled(config):
        return consider_named_checkpoint(
            metrics,
            best_score,
            config.get("eval", {}),
        )
    score = eval_checkpoint_score(metrics)
    return best_score is None or score > best_score, max(
        (score, best_score) if best_score is not None else (score,)
    )


def eval_checkpoint_score(metrics: dict[str, float]) -> tuple[float, ...]:
    if "eval_push_recovery_rate" in metrics:
        return (
            metrics["eval_push_recovery_rate"],
            metrics["eval_success_rate"],
            -metrics["eval_post_disturbance_rotation_mean"],
            -metrics.get("eval_recovery_time_mean", float("inf")),
            -metrics["eval_rotation_count_mean"],
            -metrics.get("eval_upright_error_rms", math.pi),
            -metrics.get("eval_upright_action_delta_rms", 2.0),
            metrics["eval_upright_fraction_mean"],
            metrics["eval_return_mean"],
        )
    return (
        metrics["eval_success_rate"],
        -metrics["eval_rotation_count_mean"],
        metrics["eval_upright_fraction_mean"],
        metrics["eval_return_mean"],
    )


def get_model_type(config: dict[str, Any]) -> str:
    return str(config.get("model", {}).get("model_type", "transformer")).lower()


def build_history_shape(config: dict[str, Any], obs_dim: int, action_dim: int):
    model_type = get_model_type(config)
    if model_type == "simba":
        simba_config = config["model"].get("simba", {})
        history_len = int(simba_config.get("history_len", 1))
        if history_len <= 1:
            return None
        return history_len - 1, obs_dim + action_dim

    history_config = config["history"]
    if not history_config.get("enabled", False):
        return None
    history_len = int(history_config.get("length", 0))
    if history_len <= 0:
        return None
    return history_len, obs_dim + action_dim


def empty_history(history_shape):
    if history_shape is None:
        return None
    return np.zeros(history_shape, dtype=np.float32)


def append_history(history, obs, action):
    if history is None:
        return None
    next_history = np.empty_like(history)
    next_history[:-1] = history[1:]
    next_history[-1] = np.concatenate(
        [
            np.asarray(obs, dtype=np.float32).reshape(-1),
            np.asarray(action, dtype=np.float32).reshape(-1),
        ],
        axis=0,
    )
    return next_history


def rollout_eval_episode(
    agent,
    env: CartPoleRLEnv,
    obs: np.ndarray,
    history_shape,
    on_step=None,
) -> dict[str, Any]:
    history = empty_history(history_shape)
    episode_return = 0.0
    samples: list[dict[str, Any]] = []
    terminated = truncated = False
    max_steps = int(math.ceil(env.max_episode_time / env.dt)) + 1

    for step_idx in range(max_steps):
        action = agent.act(obs, history=history, explore=False)
        next_obs, reward, terminated, truncated, info = env.step(action)
        next_history = append_history(history, obs, action)
        action_value = float(np.asarray(action).reshape(-1)[0])
        samples.append(
            {
                "theta": float(info["theta"]),
                "theta_dot": float(info.get("theta_dot", 0.0)),
                "x": float(info.get("x", 0.0)),
                "action": action_value,
                "disturbance_applied": bool(
                    info.get("disturbance_applied", False)
                ),
                "disturbance_reference_theta": info.get(
                    "disturbance_reference_theta"
                ),
            }
        )
        episode_return += float(reward)
        if on_step is not None:
            on_step(
                step_idx,
                action_value,
                float(reward),
                episode_return,
                bool(terminated),
                bool(truncated),
                info,
            )
        obs = next_obs
        history = next_history
        if terminated or truncated:
            break

    return {
        "return": episode_return,
        "length": len(samples),
        "samples": samples,
        "terminated": bool(terminated),
        "truncated": bool(truncated),
    }


def validate_experience_shapes(experience, obs_shape, action_shape, history_shape) -> None:
    expected_shapes = {
        "state": obs_shape,
        "action": action_shape,
        "reward": (1,),
        "terminated": (1,),
        "truncated": (1,),
        "next_state": obs_shape,
    }
    if history_shape is not None:
        expected_shapes["history"] = history_shape
        expected_shapes["next_history"] = history_shape

    for key, expected_shape in expected_shapes.items():
        value = experience[key]
        if value.shape != expected_shape:
            raise ValueError(
                f"Bad shape for '{key}': expected {expected_shape}, got {value.shape}."
            )


def collect_random_steps(
    env: CartPoleRLEnv,
    replay_buffer: RAM_ReplayBuffer,
    history_shape,
    n_steps: int,
    seed: int,
    config: dict[str, Any],
) -> int:
    if n_steps <= 0:
        return 0

    apply_episode_curriculum(env, generate_state_curriculum(config, episode=1))

    obs, _info = env.reset(seed=seed)
    history = empty_history(history_shape)
    collected_steps = 0

    while collected_steps < n_steps:
        action = env.action_space.sample().astype(np.float32)
        next_obs, reward, terminated, truncated, _info = env.step(action)
        next_history = append_history(history, obs, action)

        experience = convert_step_output_to_experience(
            obs=obs,
            action=action,
            reward=reward,
            terminated=terminated,
            truncated=truncated,
            next_obs=next_obs,
            history=history,
            next_history=next_history,
        )
        if collected_steps == 0:
            validate_experience_shapes(
                experience,
                env.observation_space.shape,
                env.action_space.shape,
                history_shape,
            )
        replay_buffer.add(experience)

        obs = next_obs
        history = next_history
        collected_steps += 1

        if terminated or truncated:
            obs, _info = env.reset()
            history = empty_history(history_shape)

    return collected_steps


def create_agent_and_buffer(config: dict[str, Any], env: CartPoleRLEnv, device: torch.device):
    obs_shape = env.observation_space.shape
    action_shape = env.action_space.shape
    if len(obs_shape) != 1 or len(action_shape) != 1:
        raise ValueError(f"DDPG expects flat Box spaces, got {obs_shape=} {action_shape=}.")

    model_type = get_model_type(config)
    history_shape = build_history_shape(config, obs_shape[0], action_shape[0])
    transformer_config = (
        config["history"].get("transformer", {})
        if model_type == "transformer" and history_shape is not None
        else None
    )

    agent = DDPG(
        obs_dim=obs_shape,
        min_action_values=env.action_space.low,
        max_action_values=env.action_space.high,
        hidden_dims=config["model"]["hidden_dims"],
        exploration_std=float(config["model"]["exploration_std"]),
        gamma=float(config["model"]["gamma"]),
        target_exponential_averaging=float(config["model"]["target_exponential_averaging"]),
        device=device,
        history_shape=history_shape,
        transformer_config=transformer_config,
        model_type=model_type,
        simba_config=config["model"].get("simba", {}),
    )

    keys = ["state", "action", "reward", "terminated", "truncated", "next_state"]
    if history_shape is not None:
        keys.extend(["history", "next_history"])

    replay_buffer = RAM_ReplayBuffer(
        batch_size=int(config["train"]["batch_size"]),
        max_buffer_size=int(config["train"]["max_buffer_size"]),
        keys=keys,
        device=device,
    )

    return agent, replay_buffer, history_shape


def make_optimizer(parameters, lr: float, weight_decay: float) -> torch.optim.Optimizer:
    if weight_decay > 0.0:
        return torch.optim.AdamW(parameters, lr=lr, weight_decay=weight_decay)
    return torch.optim.Adam(parameters, lr=lr)


def periodic_checkpoint_path(out_dir: Path, episode: int) -> Path:
    return out_dir / "checkpoints" / f"episode_{episode:04d}.pt"


def save_checkpoint(
    path: Path,
    agent: DDPG,
    config: dict[str, Any],
    obs_shape,
    action_shape,
    history_shape,
    episode: int,
    total_steps: int,
    selection_metadata: dict[str, Any] | None = None,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    torch.save(
        {
            "model_state_dict": agent.state_dict(),
            "config": config,
            "obs_shape": tuple(obs_shape),
            "action_shape": tuple(action_shape),
            "history_shape": tuple(history_shape) if history_shape is not None else None,
            "episode": int(episode),
            "total_steps": int(total_steps),
            "selection": selection_metadata,
        },
        path,
    )


def write_training_log(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    preferred_fields = [
        "episode",
        "total_steps",
        "return",
        "length",
        "upright_fraction",
        "terminated",
        "truncated",
        "buffer_size",
        "updates",
        "exploration_std",
        "critic_loss",
        "actor_loss",
        "curriculum_angle_range",
        "curriculum_cart_position_range",
        "curriculum_cart_velocity_range",
        "curriculum_theta_dot_range",
        "curriculum_physics_noise_low",
        "curriculum_physics_noise_high",
        "curriculum_action_delay_min_steps",
        "curriculum_action_delay_max_steps",
        "action_delay_steps",
        "physics_friction_coef",
        "physics_mass_coef",
        "physics_action_scale",
        "disturbance_episode_eligible",
        "disturbance_applied",
        "disturbance_delta_theta_dot",
        "disturbance_applied_rate",
        "disturbance_applied_given_eligible_rate",
        "fail_fast_bad_eval_count",
        "fail_fast_triggered",
        "fail_fast_reason",
        "eval_return_mean",
        "eval_return_std",
        "eval_length_mean",
        "eval_success_rate",
        "eval_upright_fraction_mean",
        "eval_longest_upright_time_mean",
        "eval_rotation_count_mean",
        "eval_disturbance_applied_rate",
        "eval_push_recovery_rate",
        "eval_recovery_time_mean",
        "eval_post_disturbance_rotation_mean",
        "eval_upright_error_rms",
        "eval_upright_action_delta_rms",
    ]
    all_fields = set()
    for row in rows:
        all_fields.update(row.keys())
    fieldnames = [field for field in preferred_fields if field in all_fields]
    fieldnames.extend(sorted(all_fields - set(fieldnames)))

    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def disturbance_episode_log_metrics(
    *,
    episode_eligible: bool,
    disturbance_applied: bool,
    disturbance_delta_theta_dot: float,
    episodes_seen: int,
    eligible_episodes_seen: int,
    applied_episodes_seen: int,
) -> dict[str, float | int]:
    return {
        "disturbance_episode_eligible": int(episode_eligible),
        "disturbance_applied": int(disturbance_applied),
        "disturbance_delta_theta_dot": float(
            disturbance_delta_theta_dot
        ),
        "disturbance_applied_rate": (
            float(applied_episodes_seen / episodes_seen)
            if episodes_seen > 0
            else 0.0
        ),
        "disturbance_applied_given_eligible_rate": (
            float(applied_episodes_seen / eligible_episodes_seen)
            if eligible_episodes_seen > 0
            else 0.0
        ),
    }


def evaluate(
    agent: DDPG,
    config: dict[str, Any],
    history_shape,
    seed_offset: int = 10000,
    angle_range: float | None = None,
    episode_params: dict[str, Any] | None = None,
):
    eval_config = config["eval"]
    n_episodes = int(eval_config["n_episodes"])
    push_episodes = min(int(eval_config.get("push_episodes", 0)), n_episodes)
    upright_threshold = float(eval_config["upright_error_threshold"])
    stable_x_threshold = float(eval_config["stable_x_threshold"])
    required_upright_time = float(eval_config.get("required_upright_time", 2.0))
    max_rotations = float(eval_config.get("max_post_acquisition_rotations", 0.75))

    was_training = agent.training
    agent.eval()

    episode_returns = []
    episode_lengths = []
    successes = []
    upright_fractions = []
    longest_upright_times = []
    rotation_counts = []
    disturbance_applied = []
    push_recoveries = []
    recovery_times = []
    post_disturbance_rotations = []
    upright_error_rms_values = []
    upright_action_delta_rms_values = []

    for episode_idx in range(n_episodes):
        env = make_env(config)
        if episode_params is not None:
            apply_episode_curriculum(env, episode_params)
        elif angle_range is not None:
            apply_initial_angle_range(env, angle_range)
        obs, info = env.reset(seed=seed_offset + episode_idx)
        is_push_episode = episode_idx < push_episodes
        if is_push_episode:
            obs = prepare_push_eval_episode(env)
        rollout = rollout_eval_episode(agent, env, obs, history_shape)

        env.close()
        episode_returns.append(rollout["return"])
        episode_lengths.append(rollout["length"])
        summary = summarize_eval_episode(
            rollout["samples"],
            dt=env.dt,
            upright_error_threshold=upright_threshold,
            stable_x_threshold=stable_x_threshold,
            required_upright_time=required_upright_time,
            max_post_acquisition_rotations=max_rotations,
            terminated=rollout["terminated"],
        )
        successes.append(summary["success"])
        upright_fractions.append(summary["upright_fraction"])
        longest_upright_times.append(summary["longest_upright_time"])
        rotation_counts.append(summary["post_acquisition_rotation_count"])
        track_push = is_push_episode or push_episodes == 0
        if track_push:
            was_pushed = bool(summary["disturbance_applied"])
            disturbance_applied.append(summary["disturbance_applied"])
            push_recoveries.append(summary["push_recovery_success"])
            recovery_times.append(
                summary["recovery_time"]
                if math.isfinite(summary["recovery_time"])
                else float(config["env"]["max_episode_time"])
            )
            post_disturbance_rotations.append(
                summary["post_disturbance_rotation_count"]
                if was_pushed
                else max_rotations + 1.0
            )
        upright_error_rms_values.append(summary["upright_error_rms"])
        upright_action_delta_rms_values.append(
            summary["upright_action_delta_rms"]
        )

    if was_training:
        agent.train()

    return {
        "eval_return_mean": float(np.mean(episode_returns)),
        "eval_return_std": float(np.std(episode_returns)),
        "eval_length_mean": float(np.mean(episode_lengths)),
        "eval_success_rate": float(np.mean(successes)),
        "eval_upright_fraction_mean": float(np.mean(upright_fractions)),
        "eval_longest_upright_time_mean": float(np.mean(longest_upright_times)),
        "eval_rotation_count_mean": float(np.mean(rotation_counts)),
        "eval_disturbance_applied_rate": float(np.mean(disturbance_applied)),
        "eval_push_recovery_rate": float(np.mean(push_recoveries)),
        "eval_recovery_time_mean": (
            float(np.mean(recovery_times))
            if recovery_times
            else float(config["env"]["max_episode_time"])
        ),
        "eval_post_disturbance_rotation_mean": float(
            np.mean(post_disturbance_rotations)
        ) if post_disturbance_rotations else 0.0,
        "eval_upright_error_rms": float(np.mean(upright_error_rms_values)),
        "eval_upright_action_delta_rms": float(
            np.mean(upright_action_delta_rms_values)
        ),
    }


def evaluate_named_scenarios(
    agent: DDPG,
    config: dict[str, Any],
    history_shape,
    *,
    base_seed: int = 10000,
    episodes_per_case: int | None = None,
) -> dict[str, float]:
    eval_config = config["eval"]
    episodes_per_case = (
        int(eval_config.get("scenario_episodes_per_case", 20))
        if episodes_per_case is None
        else int(episodes_per_case)
    )
    scenarios = build_named_eval_scenarios(
        config,
        base_seed=base_seed,
        episodes_per_case=episodes_per_case,
    )
    final_params = get_hard_eval_params(config)
    grouped: dict[str, list[tuple[dict[str, float], float, list[dict[str, Any]]]]] = {}
    was_training = bool(getattr(agent, "training", False))
    agent.eval()

    try:
        for scenario in scenarios:
            env = make_env(config)
            env.max_episode_time = float(scenario["max_episode_time"])
            env.initial_state_mix["enabled"] = False
            action_delay_steps = int(
                scenario.get("action_delay_steps", 0)
            )
            env.set_action_delay_range(
                min_steps=action_delay_steps,
                max_steps=action_delay_steps,
            )
            env.set_physics_noise(
                enabled=bool(scenario["physics_noise_enabled"]),
                low=float(scenario["physics_noise_low"]),
                high=float(scenario["physics_noise_high"]),
            )
            env.disturbance = dict(env.disturbance)
            impulse = scenario["fixed_delta_theta_dot"]
            if impulse is None:
                env.disturbance["enabled"] = False
            else:
                env.disturbance.update(
                    enabled=True,
                    episode_probability=1.0,
                    upright_time=float(scenario["disturbance_upright_time"]),
                    delay_min=float(scenario["disturbance_delay"]),
                    delay_max=float(scenario["disturbance_delay"]),
                    fixed_delta_theta_dot=float(impulse),
                )

            if scenario["initial_state"] == "swingup":
                apply_initial_angle_range(env, math.pi)
                env.set_initial_state_ranges(
                    cart_position_range=float(
                        final_params["initial_cart_position_range"]
                    ),
                    cart_velocity_range=float(
                        final_params["initial_cart_velocity_range"]
                    ),
                    pole_angular_velocity_range=float(
                        final_params["initial_pole_angular_velocity_range"]
                    ),
                )
            else:
                env.set_initial_state_ranges(
                    cart_position_range=0.0,
                    cart_velocity_range=0.0,
                    pole_angular_velocity_range=0.0,
                )

            obs, _ = env.reset(seed=int(scenario["seed"]))
            if scenario["initial_state"] == "upright":
                obs = prepare_push_eval_episode(env)
            rollout = rollout_eval_episode(agent, env, obs, history_shape)
            summary = summarize_eval_episode(
                rollout["samples"],
                dt=env.dt,
                upright_error_threshold=float(
                    eval_config["upright_error_threshold"]
                ),
                stable_x_threshold=float(eval_config["stable_x_threshold"]),
                required_upright_time=float(
                    eval_config.get("required_upright_time", 2.0)
                ),
                max_post_acquisition_rotations=float(
                    eval_config.get("max_post_acquisition_rotations", 0.75)
                ),
                terminated=rollout["terminated"],
            )
            grouped.setdefault(str(scenario["name"]), []).append(
                (summary, float(rollout["return"]), rollout["samples"])
            )
            env.close()
    finally:
        if was_training:
            agent.train()

    metrics: dict[str, float] = {}
    mean_fields = {
        "termination": "termination_rate",
        "success": "success_rate",
        "disturbance_applied": "disturbance_applied_rate",
        "push_catch": "catch_rate",
        "no_turn_recovery": "no_turn_recovery_rate",
        "maximum_unwrapped_angular_excursion": "maximum_unwrapped_excursion_mean",
    }
    for name, episodes in grouped.items():
        prefix = f"eval_{name}"
        summaries = [summary for summary, _, _ in episodes]
        sample_groups = [samples for _, _, samples in episodes]
        pooled_samples = [sample for samples in sample_groups for sample in samples]
        theta_errors = [
            angle_error_to_upright(float(sample["theta"])) for sample in pooled_samples
        ]
        theta_dots = [
            float(sample["theta_dot"])
            for sample in pooled_samples
            if "theta_dot" in sample
        ]
        positions = [float(sample["x"]) for sample in pooled_samples]
        action_groups = [
            [float(sample["action"]) for sample in samples if "action" in sample]
            for samples in sample_groups
        ]
        actions = [action for group in action_groups for action in group]
        action_deltas = [
            current - previous
            for group in action_groups
            for previous, current in zip(group, group[1:])
        ]
        metrics[f"{prefix}_episode_count"] = float(len(episodes))
        metrics[f"{prefix}_return_mean"] = float(
            np.mean([episode_return for _, episode_return, _ in episodes])
        )
        for source, suffix in mean_fields.items():
            metrics[f"{prefix}_{suffix}"] = float(
                np.mean([summary[source] for summary in summaries])
            )
        metrics[f"{prefix}_action_saturation_count"] = float(
            sum(summary["action_saturation_count"] for summary in summaries)
        )
        metrics[f"{prefix}_theta_rms"] = (
            float(np.sqrt(np.mean(np.square(theta_errors))))
            if theta_errors
            else math.pi
        )
        metrics[f"{prefix}_theta_p95"] = (
            float(np.percentile(theta_errors, 95)) if theta_errors else math.pi
        )
        metrics[f"{prefix}_theta_dot_rms"] = (
            float(np.sqrt(np.mean(np.square(theta_dots)))) if theta_dots else 0.0
        )
        metrics[f"{prefix}_mean_x"] = (
            float(np.mean(positions)) if positions else 0.0
        )
        metrics[f"{prefix}_action_rms"] = (
            float(np.sqrt(np.mean(np.square(actions)))) if actions else 0.0
        )
        metrics[f"{prefix}_action_delta_rms"] = (
            float(np.sqrt(np.mean(np.square(action_deltas))))
            if action_deltas
            else 0.0
        )
        metrics[f"{prefix}_action_saturation_rate"] = (
            metrics[f"{prefix}_action_saturation_count"] / len(actions)
            if actions
            else 0.0
        )
        recovery_times = [
            summary["recovery_time"]
            if math.isfinite(summary["recovery_time"])
            else float(
                next(
                    scenario["max_episode_time"]
                    for scenario in scenarios
                    if scenario["name"] == name
                )
            )
            for summary in summaries
        ]
        metrics[f"{prefix}_recovery_time_mean"] = float(np.mean(recovery_times))
    return metrics


def evaluate_named_scenarios_clean_and_noisy(
    agent: DDPG,
    config: dict[str, Any],
    history_shape,
    *,
    base_seed: int = 10000,
) -> dict[str, float]:
    clean_config = copy.deepcopy(config)
    clean_config["env"]["observation_noise_std"] = [0.0] * 5
    metrics = evaluate_named_scenarios(
        agent,
        clean_config,
        history_shape,
        base_seed=base_seed,
    )

    noisy_config = config["eval"].get("noisy_robustness", {})
    if not noisy_config.get("enabled", False):
        return metrics

    robustness_config = copy.deepcopy(config)
    robustness_config["env"]["observation_noise_std"] = list(
        noisy_config["observation_noise_std"]
    )
    noisy_metrics = evaluate_named_scenarios(
        agent,
        robustness_config,
        history_shape,
        base_seed=base_seed + 100000,
    )
    for key, value in noisy_metrics.items():
        if not key.startswith("eval_"):
            raise ValueError(
                "Named evaluation metrics must use the 'eval_' prefix, "
                f"got {key!r}"
            )
        metrics[f"eval_noisy_{key.removeprefix('eval_')}"] = value
    return metrics


def run_periodic_evaluation(
    *,
    agent: DDPG,
    config: dict[str, Any],
    history_shape,
    episode_params: dict[str, Any],
) -> dict[str, float]:
    if named_evaluation_enabled(config):
        return evaluate_named_scenarios_clean_and_noisy(
            agent,
            config,
            history_shape,
            base_seed=int(config["seed"]) + 10000,
        )
    return evaluate(
        agent,
        config,
        history_shape,
        episode_params=episode_params,
    )


def train(
    name: str,
    agent: DDPG,
    env: CartPoleRLEnv,
    actor_optimizer: torch.optim.Optimizer,
    critic_optimizer: torch.optim.Optimizer,
    replay_buffer: RAM_ReplayBuffer,
    config: dict[str, Any],
    out_dir: Path,
    history_shape=None,
    wandb_enabled: bool = False,
) -> None:
    train_config = config["train"]
    eval_config = config["eval"]
    seed = int(config["seed"])
    n_episodes = int(train_config["n_episodes"])
    preheat_steps = int(train_config["preheat_steps"])
    random_warmup_steps = int(train_config.get("random_warmup_steps", preheat_steps))
    updates_per_step = int(train_config["updates_per_step"])
    actor_update_frequency = int(train_config["actor_update_frequency"])
    max_grad_norm = float(train_config["max_grad_norm"])
    log_every = int(train_config["log_every"])
    eval_every = int(train_config["eval_every"])
    save_every = int(train_config["save_every"])
    min_exploration_std = float(config["model"]["min_exploration_std"])
    exploration_decay = float(config["model"]["exploration_decay"])
    upright_threshold = float(eval_config["upright_error_threshold"])

    wandb_run = None
    if wandb_enabled:
        import wandb

        wandb_run = wandb.init(
            project=config["logging"]["wandb_project"],
            name=name,
            config=config,
        )

    rows: list[dict[str, Any]] = []
    total_steps = collect_random_steps(
        env=env,
        replay_buffer=replay_buffer,
        history_shape=history_shape,
        n_steps=random_warmup_steps,
        seed=seed,
        config=config,
    )
    if total_steps > 0:
        print(
            f"random_warmup_steps={total_steps} "
            f"buffer={len(replay_buffer)}"
        )
    update_steps = 0
    disturbance_eligible_episodes = 0
    disturbance_applied_episodes = 0
    fail_fast_consecutive_evals = 0
    fail_fast_triggered = False
    fail_fast_reason = ""
    last_episode = 0
    best_eval_score = None
    best_candidate_rank = None
    best_eligible_rank = None
    hard_eval_params = get_hard_eval_params(config)
    current_curriculum_range = get_curriculum_angle_range(config, episode=1)

    checkpoint_path = out_dir / "ddpg_model.pt"
    best_checkpoint_path = out_dir / "best_ddpg_model.pt"
    best_eligible_checkpoint_path = out_dir / "best_eligible_ddpg_model.pt"
    log_path = out_dir / "training_log.csv"

    for episode in range(1, n_episodes + 1):
        last_episode = episode
        if config.get("curriculum", {}).get("enabled", False):
            if str(config["curriculum"].get("mode", "linear")) == "linear":
                current_curriculum_range = get_curriculum_angle_range(config, episode)
        episode_params = generate_state_curriculum(config, episode)
        if str(config.get("curriculum", {}).get("mode", "linear")) == "success":
            episode_params["initial_angle_range"] = current_curriculum_range
        apply_episode_curriculum(env, episode_params)

        obs, info = env.reset(seed=seed + episode)
        episode_info = info
        history = empty_history(history_shape)
        episode_return = 0.0
        episode_len = 0
        upright_steps = 0
        disturbance_episode_eligible = bool(
            episode_info.get("disturbance_episode_eligible", False)
        )
        episode_disturbance_applied = False
        episode_disturbance_delta_theta_dot = 0.0
        critic_losses = []
        actor_losses = []
        terminated = False
        truncated = False

        max_steps = int(math.ceil(env.max_episode_time / env.dt)) + 1
        for _ in range(max_steps):
            if total_steps < preheat_steps:
                action = env.action_space.sample().astype(np.float32)
            else:
                agent.train()
                action = agent.act(obs, history=history, explore=True)

            next_obs, reward, terminated, truncated, info = env.step(action)
            next_history = append_history(history, obs, action)

            experience = convert_step_output_to_experience(
                obs=obs,
                action=action,
                reward=reward,
                terminated=terminated,
                truncated=truncated,
                next_obs=next_obs,
                history=history,
                next_history=next_history,
            )
            if total_steps == 0:
                validate_experience_shapes(
                    experience,
                    env.observation_space.shape,
                    env.action_space.shape,
                    history_shape,
                )
            replay_buffer.add(experience)

            if len(replay_buffer) >= replay_buffer.batch_size and total_steps >= preheat_steps:
                for _ in range(updates_per_step):
                    batch = replay_buffer.sample_batch()

                    critic_optimizer.zero_grad(set_to_none=True)
                    critic_loss = agent.get_critic_loss(batch)
                    critic_loss.backward()
                    torch.nn.utils.clip_grad_norm_(agent.critic_parameters(), max_grad_norm)
                    critic_optimizer.step()
                    critic_losses.append(float(critic_loss.detach().cpu()))

                    if update_steps % actor_update_frequency == 0:
                        for parameter in agent.critic_parameters():
                            parameter.requires_grad_(False)
                        try:
                            actor_optimizer.zero_grad(set_to_none=True)
                            actor_loss = agent.get_actor_loss(batch)
                            actor_loss.backward()
                            torch.nn.utils.clip_grad_norm_(agent.actor_parameters(), max_grad_norm)
                            actor_optimizer.step()
                            actor_losses.append(float(actor_loss.detach().cpu()))
                        finally:
                            for parameter in agent.critic_parameters():
                                parameter.requires_grad_(True)

                    agent.update_target_networks()
                    update_steps += 1

            upright_error = angle_error_to_upright(float(info["theta"]))
            if upright_error < upright_threshold:
                upright_steps += 1
            if info.get("disturbance_applied", False):
                episode_disturbance_applied = True
                episode_disturbance_delta_theta_dot = float(
                    info["disturbance_delta_theta_dot"]
                )

            episode_return += float(reward)
            episode_len += 1
            obs = next_obs
            history = next_history
            total_steps += 1

            if terminated or truncated:
                break

        agent.exploration_std = max(min_exploration_std, agent.exploration_std * exploration_decay)
        disturbance_eligible_episodes += int(
            disturbance_episode_eligible
        )
        disturbance_applied_episodes += int(
            episode_disturbance_applied
        )

        row = {
            "episode": episode,
            "total_steps": total_steps,
            "return": episode_return,
            "length": episode_len,
            "upright_fraction": upright_steps / max(episode_len, 1),
            "terminated": int(terminated),
            "truncated": int(truncated),
            "buffer_size": len(replay_buffer),
            "updates": update_steps,
            "exploration_std": float(agent.exploration_std),
            "critic_loss": float(np.mean(critic_losses)) if critic_losses else float("nan"),
            "actor_loss": float(np.mean(actor_losses)) if actor_losses else float("nan"),
            "curriculum_angle_range": float(current_curriculum_range),
            "curriculum_cart_position_range": float(episode_params["initial_cart_position_range"]),
            "curriculum_cart_velocity_range": float(episode_params["initial_cart_velocity_range"]),
            "curriculum_theta_dot_range": float(episode_params["initial_pole_angular_velocity_range"]),
            "curriculum_physics_noise_low": float(episode_params["physics_noise_low"]),
            "curriculum_physics_noise_high": float(episode_params["physics_noise_high"]),
            "curriculum_action_delay_min_steps": int(episode_params["action_delay_min_steps"]),
            "curriculum_action_delay_max_steps": int(episode_params["action_delay_max_steps"]),
            "action_delay_steps": int(episode_info["action_delay_steps"]),
            "physics_friction_coef": float(episode_info["physics_friction_coef"]),
            "physics_mass_coef": float(episode_info["physics_mass_coef"]),
            "physics_action_scale": float(
                episode_info.get("physics_action_scale", 1.0)
            ),
        }
        row.update(
            disturbance_episode_log_metrics(
                episode_eligible=disturbance_episode_eligible,
                disturbance_applied=episode_disturbance_applied,
                disturbance_delta_theta_dot=(
                    episode_disturbance_delta_theta_dot
                ),
                episodes_seen=episode,
                eligible_episodes_seen=disturbance_eligible_episodes,
                applied_episodes_seen=disturbance_applied_episodes,
            )
        )

        if eval_every > 0 and episode % eval_every == 0:
            eval_metrics = run_periodic_evaluation(
                agent=agent,
                config=config,
                history_shape=history_shape,
                episode_params=hard_eval_params,
            )
            row.update(eval_metrics)
            curriculum_config = config.get("curriculum", {})
            success_rate = eval_metrics.get(
                "eval_swingup_success_rate",
                eval_metrics.get("eval_success_rate", 0.0),
            )
            if (
                curriculum_config.get("enabled", False)
                and str(curriculum_config.get("mode", "linear")) == "success"
                and success_rate >= float(curriculum_config["success_threshold"])
            ):
                current_curriculum_range = min(
                    float(curriculum_config["max_angle_range"]),
                    current_curriculum_range + float(curriculum_config["success_increment"]),
                )
                config["env"]["initial_angle_range"] = current_curriculum_range

            if named_evaluation_enabled(config):
                eval_config = config.get("eval", {})
                (
                    save_candidate,
                    best_candidate_rank,
                    save_eligible,
                    best_eligible_rank,
                ) = consider_named_checkpoint_candidates(
                    eval_metrics,
                    best_candidate_rank=best_candidate_rank,
                    best_eligible_rank=best_eligible_rank,
                    eval_config=eval_config,
                )
                current_rank = named_checkpoint_rank(
                    eval_metrics,
                    eval_config,
                )
                current_eligible = named_checkpoint_is_eligible(
                    eval_metrics,
                    eval_config,
                )
                if save_candidate:
                    save_checkpoint(
                        best_checkpoint_path,
                        agent,
                        config,
                        env.observation_space.shape,
                        env.action_space.shape,
                        history_shape,
                        episode,
                        total_steps,
                        selection_metadata={
                            "kind": "best_candidate",
                            "eligible": current_eligible,
                            "rank": current_rank,
                        },
                    )
                if save_eligible:
                    save_checkpoint(
                        best_eligible_checkpoint_path,
                        agent,
                        config,
                        env.observation_space.shape,
                        env.action_space.shape,
                        history_shape,
                        episode,
                        total_steps,
                        selection_metadata={
                            "kind": "best_eligible",
                            "eligible": True,
                            "rank": current_rank,
                        },
                    )
            else:
                should_save, best_eval_score = consider_periodic_checkpoint(
                    config,
                    eval_metrics,
                    best_eval_score,
                )
                if should_save:
                    save_checkpoint(
                        best_checkpoint_path,
                        agent,
                        config,
                        env.observation_space.shape,
                        env.action_space.shape,
                        history_shape,
                        episode,
                        total_steps,
                    )
            (
                fail_fast_consecutive_evals,
                fail_fast_triggered,
                fail_fast_reason,
            ) = update_named_eval_fail_fast(
                config,
                episode=episode,
                metrics=eval_metrics,
                consecutive_failures=fail_fast_consecutive_evals,
            )
            row.update(
                fail_fast_bad_eval_count=fail_fast_consecutive_evals,
                fail_fast_triggered=int(fail_fast_triggered),
                fail_fast_reason=fail_fast_reason,
            )

        rows.append(row)

        if (log_every > 0 and episode % log_every == 0) or episode == 1:
            print(
                f"episode={episode} steps={total_steps} "
                f"return={episode_return:.2f} len={episode_len} "
                f"upright={row['upright_fraction']:.2f} "
                f"buffer={len(replay_buffer)} updates={update_steps} "
                f"critic_loss={row['critic_loss']:.4f} "
                f"actor_loss={row['actor_loss']:.4f}"
            )
            if "eval_return_mean" in row:
                print(
                    f"eval_return={row['eval_return_mean']:.2f} "
                    f"eval_success={row['eval_success_rate']:.2f}"
                )

        if wandb_run is not None:
            wandb_run.log(row, step=episode)

        if save_every > 0 and episode % save_every == 0:
            save_checkpoint(
                periodic_checkpoint_path(out_dir, episode),
                agent,
                config,
                env.observation_space.shape,
                env.action_space.shape,
                history_shape,
                episode,
                total_steps,
            )
            write_training_log(log_path, rows)

        if fail_fast_triggered:
            write_training_log(log_path, rows)
            print(
                f"Fail-fast stopped training at episode={episode}: "
                f"{fail_fast_reason}"
            )
            break

    save_checkpoint(
        checkpoint_path,
        agent,
        config,
        env.observation_space.shape,
        env.action_space.shape,
        history_shape,
        last_episode,
        total_steps,
    )
    write_training_log(log_path, rows)

    if wandb_run is not None:
        wandb_run.finish()

    print(f"Saved checkpoint to {checkpoint_path.resolve()}")
    print(f"Saved training log to {log_path.resolve()}")


def positive_int(value: str) -> int:
    parsed = int(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("must be a positive integer")
    return parsed


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=str, default=str(CONFIG_PATH))
    parser.add_argument("--n-episodes", type=int, default=None)
    parser.add_argument("--out-dir", type=str, default=None)
    parser.add_argument("--preheat-steps", type=int, default=None)
    parser.add_argument("--random-warmup-steps", type=int, default=None)
    parser.add_argument("--batch-size", type=int, default=None)
    parser.add_argument("--max-episode-time", type=float, default=None)
    parser.add_argument("--eval-every", type=int, default=None)
    parser.add_argument("--save-every", type=int, default=None)
    parser.add_argument("--scenario-episodes-per-case", type=positive_int, default=None)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--device", choices=["cpu", "cuda"], default=None)
    parser.add_argument("--model-type", "--model_type", choices=["transformer", "simba"], default=None)
    parser.add_argument("--simba-history-len", type=int, default=None)
    parser.add_argument("--weight-decay", type=float, default=None)
    parser.add_argument("--no-transformer", action="store_true")
    parser.add_argument("--wandb", action="store_true")
    return parser.parse_args()


def apply_cli_overrides(config: dict[str, Any], args: argparse.Namespace) -> dict[str, Any]:
    if getattr(args, "n_episodes", None) is not None:
        config["train"]["n_episodes"] = args.n_episodes
    if getattr(args, "out_dir", None) is not None:
        config["train"]["out_dir"] = args.out_dir
    if getattr(args, "preheat_steps", None) is not None:
        config["train"]["preheat_steps"] = args.preheat_steps
    if getattr(args, "random_warmup_steps", None) is not None:
        config["train"]["random_warmup_steps"] = args.random_warmup_steps
    if getattr(args, "batch_size", None) is not None:
        config["train"]["batch_size"] = args.batch_size
    if getattr(args, "max_episode_time", None) is not None:
        config["env"]["max_episode_time"] = args.max_episode_time
    if getattr(args, "eval_every", None) is not None:
        config["train"]["eval_every"] = args.eval_every
    if getattr(args, "save_every", None) is not None:
        config["train"]["save_every"] = args.save_every
    if getattr(args, "scenario_episodes_per_case", None) is not None:
        config["eval"]["scenario_episodes_per_case"] = args.scenario_episodes_per_case
    if getattr(args, "seed", None) is not None:
        config["seed"] = args.seed
    if getattr(args, "device", None) is not None:
        config["device"]["use_cuda"] = args.device == "cuda"
    if getattr(args, "model_type", None) is not None:
        config["model"]["model_type"] = args.model_type
    if getattr(args, "simba_history_len", None) is not None:
        config["model"].setdefault("simba", {})["history_len"] = args.simba_history_len
    if getattr(args, "weight_decay", None) is not None:
        config["train"]["weight_decay"] = args.weight_decay
    if getattr(args, "no_transformer", False):
        config["history"]["enabled"] = False
    if getattr(args, "wandb", False):
        config["logging"]["wandb"] = True
    return config


def main() -> None:
    args = parse_args()
    config = apply_cli_overrides(load_config(args.config), args)
    set_seed(int(config["seed"]))

    out_dir = Path(config["train"]["out_dir"])
    out_dir.mkdir(parents=True, exist_ok=True)
    save_config(config, out_dir)

    device = get_device(config)
    env = make_env(config)
    env.reset(seed=int(config["seed"]))

    agent, replay_buffer, history_shape = create_agent_and_buffer(config, env, device)
    weight_decay = float(config["train"].get("weight_decay", 0.0))
    actor_optimizer = make_optimizer(
        agent.actor_parameters(),
        lr=float(config["train"]["actor_lr"]),
        weight_decay=weight_decay,
    )
    critic_optimizer = make_optimizer(
        agent.critic_parameters(),
        lr=float(config["train"]["critic_lr"]),
        weight_decay=weight_decay,
    )

    print(
        f"DDPG model_type={get_model_type(config)} on {device}; obs_shape={env.observation_space.shape}, "
        f"action_shape={env.action_space.shape}, history_shape={history_shape}"
    )

    train(
        name=str(config["run_name"]),
        agent=agent,
        env=env,
        actor_optimizer=actor_optimizer,
        critic_optimizer=critic_optimizer,
        replay_buffer=replay_buffer,
        config=config,
        out_dir=out_dir,
        history_shape=history_shape,
        wandb_enabled=bool(config["logging"]["wandb"]),
    )

    env.close()


if __name__ == "__main__":
    main()
