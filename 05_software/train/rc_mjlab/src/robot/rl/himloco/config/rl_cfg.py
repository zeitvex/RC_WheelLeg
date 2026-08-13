from dataclasses import MISSING
from isaaclab.utils import configclass
from typing import Literal


@configclass
class HIMBaseRunnerCfg:
    """Base configuration of the runner."""

    seed: int = 1
    """The seed for the experiment. Default is 1."""

    device: str = "cuda:0"
    """The device for the rl-agent. Default is cuda:0."""

    num_steps_per_env: int = MISSING
    """The number of steps per environment per update."""

    max_iterations: int = MISSING
    """The maximum number of iterations."""

    empirical_normalization: bool | None = None
    """This parameter is deprecated and will be removed in the future.

    Use `actor_obs_normalization` and `critic_obs_normalization` instead.
    """

    # obs_groups: dict[str, list[str]] = MISSING
    # """A mapping from observation groups to observation sets.

    # The keys of the dictionary are predefined observation sets used by the underlying algorithm
    # and values are lists of observation groups provided by the environment.

    # For instance, if the environment provides a dictionary of observations with groups "policy", "images",
    # and "privileged", these can be mapped to algorithmic observation sets as follows:

    # .. code-block:: python

    #     obs_groups = {
    #         "policy": ["policy", "images"],
    #         "critic": ["policy", "privileged"],
    #     }

    # This way, the policy will receive the "policy" and "images" observations, and the critic will
    # receive the "policy" and "privileged" observations.

    # For more details, please check ``vec_env.py`` in the rsl_rl library.
    # """

    # clip_actions: float | None = None
    # """The clipping value for actions. If None, then no clipping is done. Defaults to None.

    # .. note::
    #     This clipping is performed inside the :class:`RslRlVecEnvWrapper` wrapper.
    # """

    save_interval: int = MISSING
    """The number of iterations between saves."""

    experiment_name: str = MISSING
    """The experiment name."""

    run_name: str = ""
    """The run name. Default is empty string.

    The name of the run directory is typically the time-stamp at execution. If the run name is not empty,
    then it is appended to the run directory's name, i.e. the logging directory's name will become
    ``{time-stamp}_{run_name}``.
    """

    logger: Literal["tensorboard", "neptune", "wandb"] = "tensorboard"
    """The logger to use. Default is tensorboard."""

    neptune_project: str = "isaaclab"
    """The neptune project name. Default is "isaaclab"."""

    wandb_project: str = "isaaclab"
    """The wandb project name. Default is "isaaclab"."""

    resume: bool = False
    """Whether to resume a previous training. Default is False.

    This flag will be ignored for distillation.
    """

    load_run: str = ".*"
    """The run directory to load. Default is ".*" (all).

    If regex expression, the latest (alphabetical order) matching run will be loaded.
    """

    load_checkpoint: str = "model_.*.pt"
    """The checkpoint file to load. Default is ``"model_.*.pt"`` (all).

    If regex expression, the latest (alphabetical order) matching file will be loaded.
    """
    
    
@configclass
class HIMPPOActorCriticCfg:
    """Configuration of the HIM PPO actor-critic."""

    actor_hidden_dims: list[int] = [512, 256, 128]
    """The hidden dimensions of the actor network."""

    critic_hidden_dims: list[int] = [512, 256, 128]
    """The hidden dimensions of the critic network."""

    activation: str = "elu"
    """The activation function to use. Default is 'elu'."""

    init_noise_std: float = 1.0
    """The initial noise standard deviation for the actor. Default is 1.0."""
    
    normalize_obs:  bool = False
    """Whether to normalize observations. Default is False."""

@configclass
class HIMPPPOAlgorithmCfg:
    """Configuration of the HIM PPO algorithm."""
    num_learning_epochs: int = 1
    """The number of learning epochs per update. Default is 1."""

    num_mini_batches: int = 1
    """The number of mini-batches per update. Default is 1."""

    clip_param: float = 0.2
    """The clipping parameter for PPO. Default is 0.2."""

    gamma: float = 0.998
    """The discount factor. Default is 0.998."""

    lam: float = 0.95
    """The GAE lambda parameter. Default is 0.95."""

    value_loss_coef: float = 1.0
    """The coefficient for the value loss. Default is 1.0."""

    entropy_coef: float = 0.0
    """The coefficient for the entropy bonus. Default is 0.0."""

    learning_rate: float = 1.0e-3
    """The learning rate. Default is 1.0e-3."""

    max_grad_norm: float = 1.0
    """The maximum gradient norm for clipping. Default is 1.0."""
    
    use_clipped_value_loss: bool = True
    """Whether to use clipped value loss. Default is True."""

    schedule: str = "fixed"
    """The learning rate schedule. Default is 'fixed'."""

    desired_kl: float = 0.01
    """The desired KL divergence for adaptive learning rate. Default is 0.01."""

@configclass
class HIMOnPolicyRunnerCfg(HIMBaseRunnerCfg):
    """Configuration of the runner for on-policy algorithms."""

    class_name: str = "HIMOnPolicyRunner"
    """The runner class name. Default is OnPolicyRunner."""
    
    policy_class_name: str = "HIMActorCritic"
    """The policy class name. Default is HIMActorCritic."""
    
    algorithm_class_name: str = "HIMPPO"
    """The algorithm class name. Default is HIMPPO."""

    policy: HIMPPOActorCriticCfg = MISSING
    """The policy configuration."""

    algorithm: HIMPPPOAlgorithmCfg = MISSING
    """The algorithm configuration."""
    
    history_length: int = 0
    """Number of historical time steps to stack with current observation (0 means current only). Default is 0."""
    
    privileged_history_length: int = 0
    """Number of historical time steps to stack with current privileged observation. Default is 0."""