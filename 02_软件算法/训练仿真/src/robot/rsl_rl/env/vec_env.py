import torch


class VecEnv:
    """Abstract vectorized environment interface for HIMLoco RSL-RL."""

    num_envs: int
    num_obs: int | None = None
    num_one_step_obs: int | None = None
    num_privileged_obs: int | None = None
    num_one_step_privileged_obs: int | None = None
    num_actions: int
    max_episode_length: int
    device: str

    def get_observations(self) -> torch.Tensor:
        raise NotImplementedError

    def get_privileged_observations(self) -> torch.Tensor | None:
        raise NotImplementedError

    def step(self, actions: torch.Tensor) -> tuple:
        raise NotImplementedError

    def reset(self) -> tuple:
        raise NotImplementedError

    def seed(self, seed: int = -1) -> int:
        raise NotImplementedError

    def close(self) -> None:
        raise NotImplementedError

    @property
    def episode_length_buf(self) -> torch.Tensor:
        raise NotImplementedError

    @episode_length_buf.setter
    def episode_length_buf(self, value: torch.Tensor) -> None:
        raise NotImplementedError