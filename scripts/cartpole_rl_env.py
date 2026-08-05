import math
from typing import Any

import gymnasium as gym
import numpy as np
from gymnasium import spaces

from cartpole.common.rl_types import Config, Limits, Parameters, State, Target
from cartpole.simulator import Simulator


class CartPoleRLEnv(gym.Env):
    """
    Gymnasium wrapper над cartpole.simulator.Simulator.

    Observation:
        [x, x_dot, sin(theta), cos(theta), theta_dot]

    Action:
        one scalar in [-1, 1]

    Internal control:
        acceleration = action[0] * max_acceleration
        Target(acceleration=acceleration)
    """

    metadata = {"render_modes": []}

    def __init__(
        self,
        dt: float = 0.02,
        max_episode_time: float = 10.0,
        random_initial_angle: bool = True,
        initial_angle: float = 0.0,
        initial_angle_center: float = 0.0,
        initial_angle_range: float = 0.15,
        random_initial_cart: bool = False,
        initial_cart_position_range: float = 0.0,
        initial_cart_velocity_range: float = 0.0,
        initial_pole_angular_velocity_range: float = 0.0,
        reward_mode: str = "swingup",
        terminate_on_control_limit: bool = True,
        noisy_physics_enabled: bool = False,
        physics_noise_low: float = 0.8,
        physics_noise_high: float = 1.2,
        physics_randomization: dict[str, Any] | None = None,
        observation_noise_std: list[float] | tuple[float, ...] | None = None,
        action_delay_min_steps: int = 0,
        action_delay_max_steps: int = 0,
        initial_state_mix: dict[str, Any] | None = None,
        disturbance: dict[str, Any] | None = None,
        theta_dot_penalty: float = 0.0,
        center_penalty: float = 0.0,
        action_delta_penalty: float = 0.0,
        legacy_theta_dot_penalty: float = 0.01,
        legacy_action_penalty: float = 0.002,
        legacy_action_delta_penalty: float = 0.0,
        legacy_action_saturation_threshold: float = 1.0,
        legacy_action_saturation_penalty: float = 0.0,
    ):
        super().__init__()

        self.sim = Simulator()

        config = Config(
            hardware_limit=Limits(
                cart_position=0.2, # 0.5
                cart_velocity=3.0, # 2.0
                cart_acceleration=5.0,
            ),
            control_limit=Limits(
                cart_position=0.15, # 0.45
                cart_velocity=2.0, # 1.8
                cart_acceleration=3.0, # 3.5
            ),
            parameters=Parameters(
                gravity=9.81,
                friction_coef=0.3, # 0.0
                mass_coef=5, # 0.3
            ),
        )

        self.base_config = config
        self.sim.set_config(self.base_config)
        self.config = self.sim.get_config()

        self.random_initial_cart = random_initial_cart
        self.initial_cart_position_range = float(initial_cart_position_range)
        self.initial_cart_velocity_range = float(initial_cart_velocity_range)
        self.initial_pole_angular_velocity_range = float(initial_pole_angular_velocity_range)
        if self.initial_pole_angular_velocity_range < 0:
            raise ValueError("initial_pole_angular_velocity_range must be non-negative.")

        self.reward_mode = reward_mode
        self.theta_dot_penalty = float(theta_dot_penalty)
        self.center_penalty = float(center_penalty)
        self.action_delta_penalty = float(action_delta_penalty)
        self.legacy_theta_dot_penalty = float(legacy_theta_dot_penalty)
        self.legacy_action_penalty = float(legacy_action_penalty)
        self.legacy_action_delta_penalty = float(
            legacy_action_delta_penalty
        )
        self.legacy_action_saturation_threshold = float(
            legacy_action_saturation_threshold
        )
        self.legacy_action_saturation_penalty = float(
            legacy_action_saturation_penalty
        )
        self._validate_legacy_reward()
        self.terminate_on_control_limit = terminate_on_control_limit
        self.noisy_physics_enabled = bool(noisy_physics_enabled)
        self.physics_noise_low = float(physics_noise_low)
        self.physics_noise_high = float(physics_noise_high)
        self.physics_randomization = physics_randomization or {}
        self.observation_noise_std = np.asarray(
            observation_noise_std if observation_noise_std is not None else [0.0] * 5,
            dtype=np.float64,
        )
        self._physics_action_scale = 1.0
        self._validate_physics_noise()

        self.dt = float(dt)
        self.max_episode_time = float(max_episode_time)
        self.random_initial_angle = random_initial_angle
        self.initial_angle = float(initial_angle)
        self.initial_angle_center = float(initial_angle_center)
        self.initial_angle_range = float(initial_angle_range)

        self.max_acceleration = float(self.config.control_limit.cart_acceleration)
        if self.max_acceleration <= 0:
            raise ValueError(
                "control_limit.cart_acceleration must be positive. "
                f"Got {self.max_acceleration}."
            )

        self.action_space = spaces.Box(
            low=np.array([-1.0], dtype=np.float32),
            high=np.array([1.0], dtype=np.float32),
            dtype=np.float32,
        )

        self.observation_space = spaces.Box(
            low=np.array(
                [
                    -np.inf,  # x
                    -np.inf,  # x_dot
                    -1.0,     # sin(theta)
                    -1.0,     # cos(theta)
                    -np.inf,  # theta_dot
                ],
                dtype=np.float32,
            ),
            high=np.array(
                [
                    np.inf,
                    np.inf,
                    1.0,
                    1.0,
                    np.inf,
                ],
                dtype=np.float32,
            ),
            dtype=np.float32,
        )

        self._elapsed_time = 0.0
        self._last_acceleration = 0.0
        self.action_delay_min_steps = int(action_delay_min_steps)
        self.action_delay_max_steps = int(action_delay_max_steps)
        self.action_delay_steps = 0
        self._action_delay_queue: list[float] = []
        self._validate_action_delay()
        self.initial_state_mix = initial_state_mix or {}
        self.initial_state_mix_progress = 0.0
        self.disturbance = disturbance or {}
        self._validate_disturbance()
        self._initial_state_category = "default"
        self._disturbance_applied = False
        self._disturbance_reference_theta: float | None = None
        self._disturbance_due_step: int | None = None
        self._disturbance_episode_eligible = False
        self._upright_steps = 0
        self._step_index = 0

    def _validate_physics_noise(self) -> None:
        if self.physics_noise_low <= 0 or self.physics_noise_high <= 0:
            raise ValueError("physics noise multipliers must be positive.")
        if self.physics_noise_low > self.physics_noise_high:
            raise ValueError(
                "physics_noise_low must be less than or equal to physics_noise_high."
            )
        if self.observation_noise_std.shape != (5,):
            raise ValueError("observation_noise_std must contain exactly 5 values.")
        if np.any(self.observation_noise_std < 0):
            raise ValueError("observation_noise_std values must be non-negative.")
        for name in ("friction_coef", "mass_coef", "action_scale"):
            if name not in self.physics_randomization:
                continue
            bounds = self.physics_randomization[name]
            if len(bounds) != 2:
                raise ValueError(f"physics_randomization.{name} must be [low, high].")
            low, high = (float(bounds[0]), float(bounds[1]))
            if low > high:
                raise ValueError(
                    f"physics_randomization.{name} low must not exceed high."
                )
            if name in {"mass_coef", "action_scale"} and low <= 0:
                raise ValueError(f"physics_randomization.{name} must be positive.")
            if name == "friction_coef" and low < 0:
                raise ValueError("physics_randomization.friction_coef cannot be negative.")

    def _validate_action_delay(self) -> None:
        if self.action_delay_min_steps < 0 or self.action_delay_max_steps < 0:
            raise ValueError("action delay steps must be non-negative.")
        if self.action_delay_min_steps > self.action_delay_max_steps:
            raise ValueError(
                "action_delay_min_steps must be less than or equal to action_delay_max_steps."
            )

    def _validate_disturbance(self) -> None:
        probability = float(self.disturbance.get("episode_probability", 1.0))
        if not 0.0 <= probability <= 1.0:
            raise ValueError("disturbance episode_probability must be in [0, 1].")

    def _validate_legacy_reward(self) -> None:
        penalties = {
            "legacy_theta_dot_penalty": self.legacy_theta_dot_penalty,
            "legacy_action_penalty": self.legacy_action_penalty,
            "legacy_action_delta_penalty": self.legacy_action_delta_penalty,
            "legacy_action_saturation_penalty": (
                self.legacy_action_saturation_penalty
            ),
        }
        for name, value in penalties.items():
            if value < 0:
                raise ValueError(f"{name} must be non-negative.")
        if not 0.0 <= self.legacy_action_saturation_threshold <= 1.0:
            raise ValueError(
                "legacy_action_saturation_threshold must be in [0, 1]."
            )

    def _reset_disturbance_progress(self) -> None:
        self._disturbance_applied = False
        self._disturbance_reference_theta = None
        self._disturbance_due_step = None
        self._upright_steps = 0
        self._step_index = 0

    def _reset_disturbance_state(self) -> None:
        self._validate_disturbance()
        probability = float(self.disturbance.get("episode_probability", 1.0))
        if not self.disturbance.get("enabled", False) or probability == 0.0:
            self._disturbance_episode_eligible = False
        elif probability == 1.0:
            self._disturbance_episode_eligible = True
        else:
            self._disturbance_episode_eligible = bool(
                self.np_random.random() < probability
            )
        self._reset_disturbance_progress()

    def set_initial_state_ranges(
        self,
        *,
        cart_position_range: float,
        cart_velocity_range: float,
        pole_angular_velocity_range: float,
    ) -> None:
        self.initial_cart_position_range = float(cart_position_range)
        self.initial_cart_velocity_range = float(cart_velocity_range)
        self.initial_pole_angular_velocity_range = float(pole_angular_velocity_range)
        if self.initial_cart_position_range < 0 or self.initial_cart_velocity_range < 0:
            raise ValueError("initial cart ranges must be non-negative.")
        if self.initial_pole_angular_velocity_range < 0:
            raise ValueError("initial_pole_angular_velocity_range must be non-negative.")
        self.random_initial_cart = (
            self.initial_cart_position_range > 0.0
            or self.initial_cart_velocity_range > 0.0
        )

    def set_initial_state_mix_progress(self, progress: float) -> None:
        self.initial_state_mix_progress = float(np.clip(progress, 0.0, 1.0))

    def set_physics_noise(self, *, enabled: bool, low: float, high: float) -> None:
        self.noisy_physics_enabled = bool(enabled)
        self.physics_noise_low = float(low)
        self.physics_noise_high = float(high)
        self._validate_physics_noise()

    def set_action_delay_range(self, *, min_steps: int, max_steps: int) -> None:
        self.action_delay_min_steps = int(min_steps)
        self.action_delay_max_steps = int(max_steps)
        self._validate_action_delay()

    def _reset_action_delay_queue(self) -> None:
        self._validate_action_delay()
        if self.action_delay_min_steps == self.action_delay_max_steps:
            delay_steps = self.action_delay_min_steps
        else:
            delay_steps = int(
                self.np_random.integers(
                    self.action_delay_min_steps,
                    self.action_delay_max_steps + 1,
                )
            )
        self.action_delay_steps = delay_steps
        self._action_delay_queue = [0.0 for _ in range(delay_steps)]

    def _set_noisy_config(self):
        base_params = self.base_config.parameters
        friction_range = self.physics_randomization.get("friction_coef")
        mass_range = self.physics_randomization.get("mass_coef")
        action_scale_range = self.physics_randomization.get("action_scale")
        friction_coef = (
            self.np_random.uniform(*map(float, friction_range))
            if friction_range is not None
            else base_params.friction_coef
            * self.np_random.uniform(self.physics_noise_low, self.physics_noise_high)
        )
        mass_coef = (
            self.np_random.uniform(*map(float, mass_range))
            if mass_range is not None
            else base_params.mass_coef
            * self.np_random.uniform(self.physics_noise_low, self.physics_noise_high)
        )
        self._physics_action_scale = (
            float(self.np_random.uniform(*map(float, action_scale_range)))
            if action_scale_range is not None
            else 1.0
        )
        noisy_config = self.base_config.model_copy(deep=True)
        noisy_config.parameters = Parameters(
            gravity=base_params.gravity,
            friction_coef=float(friction_coef),
            mass_coef=float(mass_coef),
        )
        self.sim.set_config(noisy_config)
        self.config = self.sim.get_config()


    def reset(
        self,
        *,
        seed: int | None = None,
        options: dict[str, Any] | None = None,
    ):
        super().reset(seed=seed)

        if self.noisy_physics_enabled:
            self._set_noisy_config()
        else:
            self.sim.set_config(self.base_config)
            self.config = self.sim.get_config()
            self._physics_action_scale = 1.0

        category = "default"
        cart_position_range = self.initial_cart_position_range
        cart_velocity_range = self.initial_cart_velocity_range
        if self.initial_state_mix.get("enabled", False):
            start = np.asarray(self.initial_state_mix["start_probabilities"], dtype=float)
            end = np.asarray(self.initial_state_mix["end_probabilities"], dtype=float)
            probabilities = start + self.initial_state_mix_progress * (end - start)
            probabilities = probabilities / probabilities.sum()
            category = str(self.np_random.choice(["down", "upright", "uniform"], p=probabilities))
            if category == "down":
                angle_range = float(self.initial_state_mix["down_angle_range"])
                theta = self.np_random.uniform(-angle_range, angle_range)
            elif category == "upright":
                angle_range = float(self.initial_state_mix["upright_angle_range"])
                theta = math.pi + self.np_random.uniform(-angle_range, angle_range)
                cart_position_range = max(
                    cart_position_range,
                    self.initial_state_mix_progress
                    * float(self.initial_state_mix["recovery_cart_position_range"]),
                )
                cart_velocity_range = max(
                    cart_velocity_range,
                    self.initial_state_mix_progress
                    * float(self.initial_state_mix["recovery_cart_velocity_range"]),
                )
            else:
                theta = self.np_random.uniform(0.0, 2.0 * math.pi)
        elif self.random_initial_angle:
            theta = self.initial_angle_center + self.np_random.uniform(
                low=-self.initial_angle_range,
                high=self.initial_angle_range,
            )
        else:
            theta = self.initial_angle

        if self.random_initial_cart:
            cart_position = self.np_random.uniform(
                low=-cart_position_range,
                high=cart_position_range,
            )
            cart_velocity = self.np_random.uniform(
                low=-cart_velocity_range,
                high=cart_velocity_range,
            )
        else:
            cart_position = 0.0
            cart_velocity = 0.0

        if self.initial_pole_angular_velocity_range > 0.0:
            pole_angular_velocity = self.np_random.uniform(
                low=-self.initial_pole_angular_velocity_range,
                high=self.initial_pole_angular_velocity_range,
            )
        else:
            pole_angular_velocity = 0.0

        self._reset_action_delay_queue()

        state = State(
            cart_position=float(cart_position),
            cart_velocity=float(cart_velocity),
            cart_acceleration=0.0,
            pole_angle=float(theta),
            pole_angular_velocity=float(pole_angular_velocity),
        )

        self.sim.reset(state)
        self._elapsed_time = 0.0
        self._last_acceleration = 0.0
        self._initial_state_category = category
        self._reset_disturbance_state()

        obs = self._make_obs(self.sim.get_state())
        info = self._make_info()

        return obs, info

    def step(self, action):
        disturbance_applied, disturbance_delta = self._maybe_apply_disturbance()
        action = np.asarray(action, dtype=np.float32)
        requested_action_value = float(action.reshape(-1)[0])
        requested_action_value = float(np.clip(requested_action_value, -1.0, 1.0))

        if self._action_delay_queue:
            action_value = self._action_delay_queue.pop(0)
            self._action_delay_queue.append(requested_action_value)
        else:
            action_value = requested_action_value

        acceleration = (
            action_value * self.max_acceleration * self._physics_action_scale
        )
        previous_acceleration = self._last_acceleration

        self.sim.set_target(Target(acceleration=acceleration))
        self.sim.advance(self.dt)

        self._elapsed_time += self.dt

        state = self.sim.get_state()

        obs = self._make_obs(state)
        reward = self._reward(
            state,
            acceleration,
            previous_acceleration=previous_acceleration,
        )
        self._last_acceleration = acceleration
        terminated = self._terminated(state)
        truncated = self._elapsed_time >= self.max_episode_time
        info = self._make_info()

        info["raw_action"] = requested_action_value
        info["applied_action"] = action_value
        info["acceleration"] = acceleration
        info["acceleration_delta"] = acceleration - previous_acceleration
        info["action_delay_steps"] = self.action_delay_steps
        info["disturbance_applied"] = disturbance_applied
        info["disturbance_delta_theta_dot"] = disturbance_delta
        info["disturbance_reference_theta"] = (
            self._disturbance_reference_theta if disturbance_applied else None
        )
        self._step_index += 1

        return obs, reward, terminated, truncated, info

    def _maybe_apply_disturbance(self) -> tuple[bool, float]:
        if (
            not self.disturbance.get("enabled", False)
            or not self._disturbance_episode_eligible
            or self._disturbance_applied
        ):
            return False, 0.0

        state = self.sim.get_state()
        threshold = float(self.disturbance.get("upright_error_threshold", 0.25))
        if self._disturbance_due_step is None:
            if self._angle_error_to_upright(state.pole_angle) < threshold:
                self._upright_steps += 1
            else:
                self._upright_steps = 0
            required = int(math.ceil(float(self.disturbance["upright_time"]) / self.dt))
            if self._upright_steps >= required:
                delay = self.np_random.uniform(
                    float(self.disturbance["delay_min"]),
                    float(self.disturbance["delay_max"]),
                )
                self._disturbance_due_step = self._step_index + int(round(delay / self.dt))

        if self._disturbance_due_step is None or self._step_index < self._disturbance_due_step:
            return False, 0.0
        if self._angle_error_to_upright(state.pole_angle) >= threshold:
            self._disturbance_due_step = None
            self._upright_steps = 0
            return False, 0.0

        if "fixed_delta_theta_dot" in self.disturbance:
            delta = float(self.disturbance["fixed_delta_theta_dot"])
        else:
            use_small = (
                "small_probability" in self.disturbance
                and self.np_random.random()
                < float(self.disturbance["small_probability"])
            )
            prefix = "small_" if use_small else ""
            magnitude = self.np_random.uniform(
                float(self.disturbance[f"{prefix}theta_dot_delta_min"]),
                float(self.disturbance[f"{prefix}theta_dot_delta_max"]),
            )
            delta = float(magnitude * self.np_random.choice([-1.0, 1.0]))
        self.sim.reset(
            state.model_copy(
                update={"pole_angular_velocity": state.pole_angular_velocity + delta}
            )
        )
        self._disturbance_applied = True
        self._disturbance_reference_theta = float(state.pole_angle)
        return True, delta

    def close(self) -> None:
        self.sim.close()

    def _make_obs(self, state: State) -> np.ndarray:
        theta = state.pole_angle

        obs = np.array(
            [
                state.cart_position,
                state.cart_velocity,
                math.sin(theta),
                math.cos(theta),
                state.pole_angular_velocity,
            ],
            dtype=np.float32,
        )
        if np.any(self.observation_noise_std):
            obs += self.np_random.normal(
                loc=0.0,
                scale=self.observation_noise_std,
            ).astype(np.float32)
        return obs

    def _angle_error_to_upright(self, theta: float) -> float:
        return float(abs(math.atan2(math.sin(theta - math.pi), math.cos(theta - math.pi))))

    def _reward(
        self,
        state: State,
        acceleration: float,
        previous_acceleration: float | None = None,
    ) -> float:
        x = state.cart_position
        x_dot = state.cart_velocity
        theta = state.pole_angle
        theta_dot = state.pole_angular_velocity
        if previous_acceleration is None:
            previous_acceleration = acceleration

        angle_error = self._angle_error_to_upright(theta)

        if self.reward_mode == "balance":
            # Если x > 0 и acceleration > 0, мы толкаем тележку еще дальше вправо.
            # Если x < 0 и acceleration < 0, толкаем еще дальше влево.
            # Это надо отдельно штрафовать, иначе policy может держать угол ценой drift.
            push_away_from_center = max(0.0, x * acceleration)

            reward = (
                8.0
                - 12.0 * angle_error**2
                - 60.0 * x**2
                - 8.0 * x_dot**2
                - 1.0 * theta_dot**2
                - 0.01 * acceleration**2
                - 4.0 * push_away_from_center
            )

            if (
                angle_error < 0.12
                and abs(x) < 0.10
                and abs(x_dot) < 0.25
                and abs(theta_dot) < 0.40
            ):
                reward += 8.0

            if abs(x) > 0.20:
                reward -= 20.0 * (abs(x) - 0.20) / 0.25

            if abs(x) > 0.35:
                reward -= 40.0

        elif self.reward_mode == "swingup":
            upright = math.cos(theta - math.pi)

            reward = (
                2.0 * upright
                - 1.0 * x**2
                - 0.05 * x_dot**2
                - 0.01 * theta_dot**2
                - 0.001 * acceleration**2
            )

        elif self.reward_mode == "swingup_soft":
            upright = math.cos(theta - math.pi)
            upright_error = self._angle_error_to_upright(theta)
            upright_peak = math.exp(-0.5 * (upright_error / 0.25) ** 2)
            reward = (
                upright
                + 0.75 * upright_peak
                - 0.05 * x**2
                - self.theta_dot_penalty * upright_peak * theta_dot**2
                - self.center_penalty
                * upright_peak
                * (x / self.config.control_limit.cart_position) ** 2
                - 0.002 * acceleration**2
                - self.action_delta_penalty
                * upright_peak
                * (acceleration - previous_acceleration) ** 2
            )

        elif self.reward_mode == "swingup_soft_legacy":
            action_delta = acceleration - previous_acceleration
            normalized_action = abs(acceleration) / self.max_acceleration
            saturation_excess = max(
                0.0,
                normalized_action
                - self.legacy_action_saturation_threshold,
            )
            reward = (
                math.cos(theta - math.pi)
                - 0.05 * x**2
                - self.legacy_theta_dot_penalty * theta_dot**2
                - self.legacy_action_penalty * acceleration**2
                - self.legacy_action_delta_penalty * action_delta**2
                - self.legacy_action_saturation_penalty
                * saturation_excess**2
                + (1.0 if angle_error < 0.25 else 0.0)
            )

        else:
            raise ValueError(f"Unknown reward_mode: {self.reward_mode}")

        if state.error:
            reward -= 100.0

        return float(reward)

    def _terminated(self, state: State) -> bool:
        if state.error:
            return True

        if self.terminate_on_control_limit:
            if abs(state.cart_position) > self.config.control_limit.cart_position:
                return True

            if abs(state.cart_velocity) > self.config.control_limit.cart_velocity:
                return True

        else:
            if abs(state.cart_position) > self.config.hardware_limit.cart_position:
                return True

            if abs(state.cart_velocity) > self.config.hardware_limit.cart_velocity:
                return True

        return False

    def _make_info(self) -> dict[str, Any]:
        state = self.sim.get_state()
        sim_info = self.sim.get_info()

        return {
            "t": state.stamp,
            "x": state.cart_position,
            "x_dot": state.cart_velocity,
            "x_ddot": state.cart_acceleration,
            "theta": state.pole_angle,
            "theta_dot": state.pole_angular_velocity,
            "error": int(state.error),
            "step_count": sim_info.step_count,
            "integration_count": sim_info.integration_count,
            "physics_friction_coef": self.config.parameters.friction_coef,
            "physics_mass_coef": self.config.parameters.mass_coef,
            "physics_action_scale": self._physics_action_scale,
            "initial_angle_range": self.initial_angle_range,
            "initial_cart_position_range": self.initial_cart_position_range,
            "initial_cart_velocity_range": self.initial_cart_velocity_range,
            "initial_pole_angular_velocity_range": self.initial_pole_angular_velocity_range,
            "action_delay_steps": self.action_delay_steps,
            "initial_state_category": self._initial_state_category,
            "disturbance_episode_eligible": self._disturbance_episode_eligible,
            "disturbance_applied": False,
            "disturbance_delta_theta_dot": 0.0,
            "disturbance_reference_theta": None,
        }
