import numpy as np
import torch

from torch import nn


class RAM_ReplayBuffer:
    def __init__(self, batch_size, max_buffer_size, keys, device):
        self.memory = []
        self.batch_size = batch_size
        self.max_buffer_size = max_buffer_size
        self.keys = keys
        self.device = device
        self.env_goals = []

    def __len__(self):
        return len(self.memory)

    def _add_experience(self, experience):
        self.memory.append(experience)
        if len(self.memory) > self.max_buffer_size:
            self.memory.pop(0)

    def add(self, experience):
        self._add_experience(experience)

    def add_trajectory(self, trajectory, env_goal = True):
        if env_goal and trajectory and 'goal' in trajectory[-1]:
            self.env_goals.append(trajectory[-1]['goal'])
        for experience in trajectory:
            self._add_experience(experience)

    def sample_batch(self, batch_size = None):
        batch_size = self.batch_size if batch_size is None else batch_size
        if len(self.memory) < batch_size:
            raise ValueError(
                f"Cannot sample batch_size={batch_size} from replay buffer "
                f"with {len(self.memory)} transitions."
            )
        batch_idxs = np.random.choice(len(self.memory), size = batch_size)
        output_dict = {}

        for i in batch_idxs:
            for key in self.keys:
                if key not in self.memory[i]:
                    raise KeyError(f"Replay transition does not contain key '{key}'.")
                if key not in output_dict.keys():
                    output_dict[key] = []
                sample = torch.as_tensor(
                    self.memory[i][key],
                    dtype=torch.float32,
                    device=self.device,
                )
                output_dict[key].append(sample)

        # Convert array of dicts into dict of arrays
        return {key: torch.stack(output_dict[key], axis=0) for key in output_dict.keys()}

class ConvertedSigmoid(nn.Module):
    # Convert segment from [0; 1] to [low_bound; high_bound]
    def __init__(self, low_bound, high_bound):
        super(ConvertedSigmoid, self).__init__()
        low_bound = torch.as_tensor(low_bound, dtype=torch.float32)
        high_bound = torch.as_tensor(high_bound, dtype=torch.float32)
        self.register_buffer("low_bound", low_bound)
        self.register_buffer("scale", high_bound - low_bound)

    def forward(self, inp):
        return torch.sigmoid(inp) * self.scale + self.low_bound


class TanhActionScaling(nn.Module):
    def __init__(self, low_bound, high_bound):
        super(TanhActionScaling, self).__init__()
        low_bound = torch.as_tensor(low_bound, dtype=torch.float32)
        high_bound = torch.as_tensor(high_bound, dtype=torch.float32)
        self.register_buffer("center", (high_bound + low_bound) * 0.5)
        self.register_buffer("half_range", (high_bound - low_bound) * 0.5)

    def forward(self, inp):
        return torch.tanh(inp) * self.half_range + self.center


class RunningStandardNorm(nn.Module):
    def __init__(self, input_dim, eps=1e-5):
        super(RunningStandardNorm, self).__init__()
        self.eps = float(eps)
        self.register_buffer("running_mean", torch.zeros(input_dim, dtype=torch.float32))
        self.register_buffer("running_var", torch.ones(input_dim, dtype=torch.float32))
        self.register_buffer("count", torch.zeros((), dtype=torch.float32))

    @torch.no_grad()
    def update(self, batch):
        if batch.ndim == 1:
            batch = batch.unsqueeze(0)
        batch = batch.detach()
        batch_count = int(batch.shape[0])
        if batch_count == 0:
            return

        batch_mean = batch.mean(dim=0)
        batch_var = batch.var(dim=0, unbiased=False)
        count = self.count.to(batch.device)

        if float(count.item()) == 0.0:
            new_mean = batch_mean
            new_var = batch_var
            new_count = torch.as_tensor(float(batch_count), dtype=batch.dtype, device=batch.device)
        else:
            total_count = count + batch_count
            delta = batch_mean - self.running_mean
            current_m2 = self.running_var * count
            batch_m2 = batch_var * batch_count
            merged_m2 = current_m2 + batch_m2 + delta.pow(2) * count * batch_count / total_count
            new_mean = self.running_mean + delta * batch_count / total_count
            new_var = merged_m2 / total_count
            new_count = total_count

        self.running_mean.copy_(new_mean)
        self.running_var.copy_(new_var.clamp_min(0.0))
        self.count.copy_(new_count)

    def forward(self, batch, update=False):
        if update and self.training:
            self.update(batch)
        return (batch - self.running_mean) / torch.sqrt(self.running_var + self.eps)


class SimBaBlock(nn.Module):
    def __init__(self, hidden_dim):
        super(SimBaBlock, self).__init__()
        self.layer_norm = nn.LayerNorm(hidden_dim)
        self.mlp = nn.Sequential(
            nn.Linear(hidden_dim, 4 * hidden_dim),
            nn.ReLU(),
            nn.Linear(4 * hidden_dim, hidden_dim),
        )

    def forward(self, x):
        return x + self.mlp(self.layer_norm(x))


class SimBaActorNetwork(nn.Module):
    def __init__(
        self,
        input_dim,
        action_dim,
        hidden_dim,
        num_blocks,
        min_action_values,
        max_action_values,
    ):
        super(SimBaActorNetwork, self).__init__()
        self.input_projection = nn.Linear(input_dim, hidden_dim)
        self.blocks = nn.Sequential(*[SimBaBlock(hidden_dim) for _ in range(num_blocks)])
        self.post_layer_norm = nn.LayerNorm(hidden_dim)
        self.output_head = nn.Linear(hidden_dim, action_dim)
        self.output_activation = TanhActionScaling(min_action_values, max_action_values)

    def forward(self, obs_features):
        x = self.input_projection(obs_features)
        x = self.blocks(x)
        x = self.post_layer_norm(x)
        return self.output_activation(self.output_head(x))


class SimBaCriticNetwork(nn.Module):
    def __init__(self, input_dim, action_dim, hidden_dim, num_blocks):
        super(SimBaCriticNetwork, self).__init__()
        self.input_projection = nn.Linear(input_dim + action_dim, hidden_dim)
        self.blocks = nn.Sequential(*[SimBaBlock(hidden_dim) for _ in range(num_blocks)])
        self.post_layer_norm = nn.LayerNorm(hidden_dim)
        self.output_head = nn.Linear(hidden_dim, 1)

    def forward(self, obs_features, action):
        x = torch.cat([obs_features, action], dim=-1)
        x = self.input_projection(x)
        x = self.blocks(x)
        x = self.post_layer_norm(x)
        return self.output_head(x)


class MLP(nn.Module):
    def __init__(self, input_dim, hidden_dims, output_dims,
                 activation = nn.ReLU, output_activation = None):
        super(MLP, self).__init__()
        if isinstance(hidden_dims, int):
            hidden_dims = [hidden_dims]
        if len(hidden_dims) == 0:
            raise ValueError("hidden_dims must contain at least one hidden layer.")

        if output_activation is None:
            output_activation = nn.Identity()
        elif isinstance(output_activation, type):
            output_activation = output_activation()

        input_lin = nn.Sequential(nn.Linear(input_dim, hidden_dims[0]), activation())
        intermediate_mlp = []

        if len(hidden_dims) > 1:
            intermediate_mlp = [nn.Sequential(nn.Linear(hidden_input_dim, hidden_output_dim), activation()) for \
                                hidden_input_dim, hidden_output_dim in zip(hidden_dims[:-1], hidden_dims[1:])]

        output_lin = nn.Sequential(nn.Linear(hidden_dims[-1], output_dims), output_activation)
        self.mlp = nn.Sequential(input_lin, *intermediate_mlp, output_lin)

    def forward(self, batch):
        return self.mlp(batch)

def uniform(low, high):
    return np.random.rand(*[i for i in low.shape]).astype(np.float32) * (high - low) + low

# Convert environment output into experience which can be saved by replay buffer
def convert_step_output_to_experience(
    obs,
    action,
    reward,
    terminated,
    truncated,
    next_obs,
    history=None,
    next_history=None,
):
    experience = {
        'state': np.asarray(obs, dtype=np.float32),
        'action': np.asarray(action, dtype=np.float32).reshape(-1),
        'reward': np.asarray([reward], dtype=np.float32),
        'terminated': np.asarray([float(terminated)], dtype=np.float32),
        'truncated': np.asarray([float(truncated)], dtype=np.float32),
        'next_state': np.asarray(next_obs, dtype=np.float32),
    }
    if history is not None:
        experience['history'] = np.asarray(history, dtype=np.float32)
    if next_history is not None:
        experience['next_history'] = np.asarray(next_history, dtype=np.float32)
    return experience


class HistoryTransformer(nn.Module):
    def __init__(
        self,
        input_dim,
        history_len,
        d_model=32,
        n_heads=2,
        n_layers=1,
        dropout=0.0,
    ):
        super(HistoryTransformer, self).__init__()
        if d_model % n_heads != 0:
            raise ValueError("d_model must be divisible by n_heads.")

        self.input_dim = int(input_dim)
        self.history_len = int(history_len)
        self.output_dim = int(d_model)
        self.input_projection = nn.Linear(self.input_dim, d_model)
        self.position_embedding = nn.Parameter(torch.zeros(1, self.history_len, d_model))
        encoder_layer = nn.TransformerEncoderLayer(
            d_model=d_model,
            nhead=n_heads,
            dim_feedforward=max(2 * d_model, 32),
            dropout=dropout,
            activation="gelu",
            batch_first=True,
            norm_first=True,
        )
        self.encoder = nn.TransformerEncoder(encoder_layer, num_layers=n_layers)

    def forward(self, history):
        if history.ndim == 2:
            history = history.unsqueeze(0)
        if history.shape[-2] != self.history_len:
            raise ValueError(
                f"Expected history length {self.history_len}, got {history.shape[-2]}."
            )

        x = self.input_projection(history)
        x = x + self.position_embedding[:, : x.shape[1], :]
        encoded = self.encoder(x)
        return encoded[:, -1, :]


class ActorNetwork(nn.Module):
    def __init__(
        self,
        obs_dim,
        action_dim,
        hidden_dims,
        min_action_values,
        max_action_values,
        history_shape=None,
        transformer_config=None,
    ):
        super(ActorNetwork, self).__init__()
        self.history_shape = history_shape
        self.history_encoder = None
        history_output_dim = 0

        if history_shape is not None:
            transformer_config = transformer_config or {}
            history_len, history_input_dim = history_shape
            self.history_encoder = HistoryTransformer(
                input_dim=history_input_dim,
                history_len=history_len,
                **transformer_config,
            )
            history_output_dim = self.history_encoder.output_dim

        self.actor = MLP(
            obs_dim + history_output_dim,
            hidden_dims,
            action_dim,
            output_activation=ConvertedSigmoid(min_action_values, max_action_values),
        )

    def _encode_history(self, state, history):
        if self.history_encoder is None:
            return None
        if history is None:
            batch_size = state.shape[0]
            history = torch.zeros(
                batch_size,
                self.history_encoder.history_len,
                self.history_encoder.input_dim,
                dtype=state.dtype,
                device=state.device,
            )
        return self.history_encoder(history)

    def forward(self, state, history=None):
        history_embedding = self._encode_history(state, history)
        if history_embedding is not None:
            state = torch.cat([state, history_embedding], dim=-1)
        return self.actor(state)


class CriticNetwork(nn.Module):
    def __init__(
        self,
        obs_dim,
        action_dim,
        hidden_dims,
        history_shape=None,
        transformer_config=None,
    ):
        super(CriticNetwork, self).__init__()
        self.history_shape = history_shape
        self.history_encoder = None
        history_output_dim = 0

        if history_shape is not None:
            transformer_config = transformer_config or {}
            history_len, history_input_dim = history_shape
            self.history_encoder = HistoryTransformer(
                input_dim=history_input_dim,
                history_len=history_len,
                **transformer_config,
            )
            history_output_dim = self.history_encoder.output_dim

        self.critic = MLP(obs_dim + action_dim + history_output_dim, hidden_dims, 1)

    def _encode_history(self, state, history):
        if self.history_encoder is None:
            return None
        if history is None:
            batch_size = state.shape[0]
            history = torch.zeros(
                batch_size,
                self.history_encoder.history_len,
                self.history_encoder.input_dim,
                dtype=state.dtype,
                device=state.device,
            )
        return self.history_encoder(history)

    def forward(self, state, action, history=None):
        inputs = [state, action]
        history_embedding = self._encode_history(state, history)
        if history_embedding is not None:
            inputs.append(history_embedding)
        return self.critic(torch.cat(inputs, dim=-1))


class DDPG(nn.Module):
    def __init__(self, obs_dim, min_action_values, max_action_values, hidden_dims,
                 exploration_std, gamma,
                 target_exponential_averaging, device,
                 history_shape=None, transformer_config=None,
                 model_type="transformer", simba_config=None):
        super(DDPG, self).__init__()
        self.device = device
        self.gamma = gamma
        self.target_exponential_averaging = target_exponential_averaging
        self.exploration_std = exploration_std
        min_action_values = np.asarray(min_action_values, dtype=np.float32)
        max_action_values = np.asarray(max_action_values, dtype=np.float32)
        action_dim = min_action_values.shape
        self.min_action_values = torch.as_tensor(min_action_values, dtype=torch.float32).to(self.device)
        self.max_action_values = torch.as_tensor(max_action_values, dtype=torch.float32).to(self.device)
        self.history_shape = history_shape

        assert len(obs_dim) == 1, 'Obs dim must be flat'
        assert len(action_dim) == 1, 'Action dim must be flat'
        obs_size = int(obs_dim[0])
        action_size = int(action_dim[0])
        self.obs_size = obs_size
        self.action_size = action_size
        self.model_type = str(model_type or "transformer").lower()
        if self.model_type not in {"transformer", "simba"}:
            raise ValueError(f"Unknown model_type: {model_type}")

        if self.model_type == "simba":
            simba_config = simba_config or {}
            self.simba_history_len = max(1, int(simba_config.get("history_len", 1)))
            self.simba_include_action_history = bool(
                simba_config.get("include_action_history", False)
            )
            simba_input_dim = obs_size * self.simba_history_len
            if self.simba_include_action_history:
                simba_input_dim += action_size * (self.simba_history_len - 1)
            self.obs_norm = RunningStandardNorm(
                simba_input_dim,
                eps=float(simba_config.get("rsnorm_eps", 1e-5)),
            ).to(device)
            actor_hidden_dim = int(simba_config.get("actor_hidden_dim", 128))
            critic_hidden_dim = int(simba_config.get("critic_hidden_dim", 256))
            actor_num_blocks = int(simba_config.get("actor_num_blocks", 1))
            critic_num_blocks = int(simba_config.get("critic_num_blocks", 2))

            self.critic, self._target_critic = [
                SimBaCriticNetwork(
                    simba_input_dim,
                    action_size,
                    critic_hidden_dim,
                    critic_num_blocks,
                ).to(device)
                for i in range(2)
            ]
            self.actor, self._target_actor = [
                SimBaActorNetwork(
                    simba_input_dim,
                    action_size,
                    actor_hidden_dim,
                    actor_num_blocks,
                    min_action_values,
                    max_action_values,
                ).to(device)
                for i in range(2)
            ]
        else:
            self.simba_history_len = 1
            self.simba_include_action_history = False
            self.obs_norm = None
            self.critic, self._target_critic = [
                CriticNetwork(
                    obs_size,
                    action_size,
                    hidden_dims,
                    history_shape=history_shape,
                    transformer_config=transformer_config,
                ).to(device)
                for i in range(2)
            ]
            self.actor, self._target_actor = [
                ActorNetwork(
                    obs_size,
                    action_size,
                    hidden_dims,
                    min_action_values,
                    max_action_values,
                    history_shape=history_shape,
                    transformer_config=transformer_config,
                ).to(device)
                for i in range(2)
            ]

        self._target_critic.load_state_dict(self.critic.state_dict())
        self._target_actor.load_state_dict(self.actor.state_dict())

    def _apply_exploration_noise(self, action):
        noise = torch.randn_like(action) * self.exploration_std
        noisy_action = action + noise
        clipped_noisy_action = torch.clamp(
            noisy_action,
            min=self.min_action_values,
            max=self.max_action_values
        )
        return clipped_noisy_action

    def _build_simba_obs_features(self, state, history=None):
        if state.ndim == 1:
            state = state.unsqueeze(0)

        if self.simba_history_len <= 1:
            return state

        previous_obs_len = self.simba_history_len - 1
        batch_size = state.shape[0]
        history_width = self.obs_size + (
            self.action_size if self.simba_include_action_history else 0
        )
        if history is None:
            previous_obs = torch.zeros(
                batch_size,
                previous_obs_len,
                history_width,
                dtype=state.dtype,
                device=state.device,
            )
        else:
            if history.ndim == 2:
                history = history.unsqueeze(0)
            if history.shape[-1] < history_width:
                raise ValueError(
                    f"History last dim must be at least {history_width}, got {history.shape[-1]}."
                )
            previous_obs = history[..., :history_width]
            if previous_obs.shape[1] < previous_obs_len:
                padding = torch.zeros(
                    batch_size,
                    previous_obs_len - previous_obs.shape[1],
                    history_width,
                    dtype=state.dtype,
                    device=state.device,
                )
                previous_obs = torch.cat([padding, previous_obs], dim=1)
            elif previous_obs.shape[1] > previous_obs_len:
                previous_obs = previous_obs[:, -previous_obs_len:, :]

        return torch.cat([previous_obs.reshape(batch_size, -1), state], dim=1)

    def _normalize_simba_obs(self, state, history=None, update_obs_stats=False):
        obs_features = self._build_simba_obs_features(state, history)
        return self.obs_norm(obs_features, update=update_obs_stats)

    def _actor_forward(self, state, history=None, target=False, update_obs_stats=False):
        actor = self._target_actor if target else self.actor
        if self.model_type == "simba":
            obs_features = self._normalize_simba_obs(
                state,
                history,
                update_obs_stats=update_obs_stats,
            )
            return actor(obs_features)
        return actor(state, history)

    def _critic_forward(self, state, action, history=None, target=False, update_obs_stats=False):
        critic = self._target_critic if target else self.critic
        if self.model_type == "simba":
            obs_features = self._normalize_simba_obs(
                state,
                history,
                update_obs_stats=update_obs_stats,
            )
            return critic(obs_features, action)
        return critic(state, action, history)

    def _split_batch_into_cur_and_next(self, batch):
        cur_sa = {
            'state': batch['state'],
            'action': batch['action'],
            'history': batch.get('history'),
        }

        next_sa = {
            'state': batch['next_state'],
            'action': None,
            'history': batch.get('next_history'),
        }
        return cur_sa, next_sa

    def _fetch_actor_input(self, batch):
        actor_input = (batch['state'], batch.get('history'))
        return actor_input

    def _fetch_critic_input(self, batch):
        critic_input = (batch['state'], batch['action'], batch.get('history'))
        return critic_input

    def act(self, obs, history=None, explore=None):
        obs = torch.as_tensor(obs, dtype=torch.float32, device=self.device)
        is_single_observation = obs.ndim == 1
        if is_single_observation:
            obs = obs.unsqueeze(0)

        history_tensor = None
        if history is not None:
            history_tensor = torch.as_tensor(history, dtype=torch.float32, device=self.device)
            if history_tensor.ndim == 2:
                history_tensor = history_tensor.unsqueeze(0)

        with torch.no_grad():
            if explore is None:
                explore = self.training

            action = self._actor_forward(
                obs,
                history_tensor,
                update_obs_stats=self.model_type == "simba",
            )
            if explore:
                action = self._apply_exploration_noise(action)

            action = action.cpu().numpy()
            if is_single_observation:
                return action[0]
            return action

    def get_critic_loss(self, batch):
        reward, terminated = batch['reward'], batch['terminated']
        cur_data, next_data = self._split_batch_into_cur_and_next(batch)

        cur_q = self._critic_forward(
            cur_data['state'],
            cur_data['action'],
            cur_data.get('history'),
            update_obs_stats=self.model_type == "simba",
        )
        with torch.no_grad():
            next_action = self._actor_forward(
                next_data['state'],
                next_data.get('history'),
                target=True,
            )
        next_data['action'] = next_action

        next_critic_input = self._fetch_critic_input(next_data)
        target = self._calculate_target(next_critic_input, reward, terminated)
        critic_loss = torch.mean((cur_q - target)**2)
        return critic_loss

    def get_actor_loss(self, batch):
        cur_data, _ = self._split_batch_into_cur_and_next(batch)
        action = self._actor_forward(
            cur_data['state'],
            cur_data.get('history'),
        )

        cur_data['action'] = action

        actor_loss = -torch.mean(
            self._critic_forward(
                cur_data['state'],
                cur_data['action'],
                cur_data.get('history'),
            )
        )
        return actor_loss

    def _calculate_target(self, target_input, reward, terminated):
        terminated = terminated.float()
        with torch.no_grad():
            if self.model_type == "simba":
                next_state, next_action, next_history = target_input
                target_q = self._critic_forward(
                    next_state,
                    next_action,
                    next_history,
                    target=True,
                )
            else:
                target_q = self._target_critic(*target_input)
            target = reward + self.gamma * target_q * (1 - terminated)
        return target

    def update_target_networks(self):
        for target_net, net in zip([self._target_critic, self._target_actor],
                                   [self.critic, self.actor]):
            for param_name, param in target_net.state_dict().items():
                param.data.copy_(param.data * self.target_exponential_averaging +
                                 net.state_dict()[param_name].data * (1-self.target_exponential_averaging))

    def critic_parameters(self):
        return list(self.critic.parameters())

    def actor_parameters(self):
        return list(self.actor.parameters())

    def parameters(self, recurse: bool = True):
        return self.critic_parameters() + self.actor_parameters()
