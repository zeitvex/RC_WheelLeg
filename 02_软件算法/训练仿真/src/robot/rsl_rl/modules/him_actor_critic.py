import torch
import torch.nn as nn
from torch.distributions import Normal
from ..modules.him_estimator import HIMEstimator


class RunningMeanStd:
    def __init__(self, shape, device):
        self.n = 1e-4
        self.uninitialized = True
        self.mean = torch.zeros(shape, device=device)
        self.var = torch.ones(shape, device=device)

    def update(self, x):
        count = self.n
        batch_count = x.size(0)
        tot_count = count + batch_count

        old_mean = self.mean.clone()
        delta = torch.mean(x, dim=0) - old_mean

        self.mean = old_mean + delta * batch_count / tot_count
        m_a = self.var * count
        m_b = x.var(dim=0) * batch_count
        M2 = m_a + m_b + torch.square(delta) * count * batch_count / tot_count
        self.var = M2 / tot_count
        self.n = tot_count


class Normalization:
    def __init__(self, shape, device='cuda:0'):
        self.running_ms = RunningMeanStd(shape=shape, device=device)

    def __call__(self, x, update=False):
        if update:
            self.running_ms.update(x)
        x = (x - self.running_ms.mean) / (torch.sqrt(self.running_ms.var) + 1e-4)
        return x


class HIMActorCritic(nn.Module):
    is_recurrent = False

    def __init__(self, num_actor_obs,
                 num_critic_obs,
                 num_one_step_obs,
                 num_actions,
                 actor_hidden_dims=[512, 256, 128],
                 critic_hidden_dims=[512, 256, 128],
                 activation='elu',
                 init_noise_std=1.0,
                 **kwargs):
        if kwargs:
            print("ActorCritic.__init__ got unexpected arguments, which will be ignored: "
                  + str([key for key in kwargs.keys()]))
        super(HIMActorCritic, self).__init__()

        activation = get_activation(activation)

        self.history_size = int(num_actor_obs / num_one_step_obs)
        self.num_actor_obs = num_actor_obs
        self.num_actions = num_actions
        self.num_one_step_obs = num_one_step_obs

        mlp_input_dim_a = num_one_step_obs + 3 + 16
        mlp_input_dim_c = num_critic_obs

        # Estimator
        self.estimator = HIMEstimator(
            temporal_steps=self.history_size,
            num_one_step_obs=num_one_step_obs,
        )

        # Policy
        actor_layers = []
        actor_layers.append(nn.Linear(mlp_input_dim_a, actor_hidden_dims[0]))
        actor_layers.append(activation)
        for l in range(len(actor_hidden_dims)):
            if l == len(actor_hidden_dims) - 1:
                actor_layers.append(nn.Linear(actor_hidden_dims[l], num_actions))
            else:
                actor_layers.append(nn.Linear(actor_hidden_dims[l], actor_hidden_dims[l + 1]))
                actor_layers.append(activation)
        self.actor = nn.Sequential(*actor_layers)

        # Value function
        critic_layers = []
        critic_layers.append(nn.Linear(mlp_input_dim_c, critic_hidden_dims[0]))
        critic_layers.append(activation)
        for l in range(len(critic_hidden_dims)):
            if l == len(critic_hidden_dims) - 1:
                critic_layers.append(nn.Linear(critic_hidden_dims[l], 1))
            else:
                critic_layers.append(nn.Linear(critic_hidden_dims[l], critic_hidden_dims[l + 1]))
                critic_layers.append(activation)
        self.critic = nn.Sequential(*critic_layers)

        print(f"Actor MLP: {self.actor}")
        print(f"Critic MLP: {self.critic}")
        print(f'Estimator: {self.estimator.encoder}')

        # Action noise
        self.std = nn.Parameter(init_noise_std * torch.ones(num_actions))
        self.distribution = None
        Normal.set_default_validate_args = False

    @staticmethod
    def init_weights(sequential, scales):
        [torch.nn.init.orthogonal_(module.weight, gain=scales[idx])
         for idx, module in
         enumerate(mod for mod in sequential if isinstance(mod, nn.Linear))]

    def reset(self, dones=None):
        pass

    def forward(self):
        raise NotImplementedError

    @property
    def action_mean(self):
        return self.distribution.mean

    @property
    def action_std(self):
        return self.distribution.stddev

    @property
    def entropy(self):
        return self.distribution.entropy().sum(dim=-1)

    def update_distribution(self, obs_history):
        with torch.no_grad():
            vel, latent = self.estimator(obs_history)
        actor_input = torch.cat(
            (obs_history[:, :self.num_one_step_obs], vel, latent), dim=-1
        )

        if torch.isnan(actor_input).any() or torch.isinf(actor_input).any():
            print(f"[ERROR] NaN/Inf detected in actor_input before normalization!")
            print(f"  - obs_history: min={obs_history.min():.4f}, max={obs_history.max():.4f}")
            print(f"  - vel: min={vel.min():.4f}, max={vel.max():.4f}")
            print(f"  - latent: min={latent.min():.4f}, max={latent.max():.4f}")
            raise ValueError("NaN/Inf in actor_input before normalization")

        mean = self.actor(actor_input)

        if torch.isnan(mean).any() or torch.isinf(mean).any():
            print(f"[ERROR] NaN/Inf in actor output (mean)!")
            for name, param in self.actor.named_parameters():
                if torch.isnan(param).any():
                    print(f"  - NaN in actor weight: {name}")
                if torch.isinf(param).any():
                    print(f"  - Inf in actor weight: {name}")
            raise ValueError("NaN or Inf detected in actor network output!")

        self.distribution = Normal(mean, mean * 0. + self.std)

    def act(self, obs_history=None, **kwargs):
        self.update_distribution(obs_history)
        return self.distribution.sample()

    def get_actions_log_prob(self, actions):
        return self.distribution.log_prob(actions).sum(dim=-1)

    def act_inference(self, obs_history, observations=None):
        vel, latent = self.estimator(obs_history)
        actions_mean = self.actor(
            torch.cat(
                (obs_history[:, :self.num_one_step_obs], vel, latent), dim=-1
            )
        )
        return actions_mean

    def test_inference(self, obs_history, observations=None):
        vel, latent = self.estimator(obs_history)
        actions_mean = self.actor(
            torch.cat(
                (obs_history[:, :self.num_one_step_obs], vel, latent), dim=-1
            )
        )
        estimator_output = torch.cat((vel, latent), dim=-1)
        return actions_mean, estimator_output

    def evaluate(self, critic_observations, **kwargs):
        value = self.critic(critic_observations)
        return value


def get_activation(act_name):
    if act_name == "elu":
        return nn.ELU()
    elif act_name == "selu":
        return nn.SELU()
    elif act_name == "relu":
        return nn.ReLU()
    elif act_name == "crelu":
        return nn.ReLU()
    elif act_name == "lrelu":
        return nn.LeakyReLU()
    elif act_name == "tanh":
        return nn.Tanh()
    elif act_name == "sigmoid":
        return nn.Sigmoid()
    else:
        print("invalid activation function!")
        return None