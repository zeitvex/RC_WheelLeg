from mjlab.rl import (
    RslRlModelCfg,
    RslRlOnPolicyRunnerCfg,
    RslRlPpoAlgorithmCfg,
)


def flat_ppo_runner_cfg() -> RslRlOnPolicyRunnerCfg:
    cfg = rough_ppo_runner_cfg()
    cfg.experiment_name = "robot_flat"
    cfg.max_iterations = 10_000
    return cfg


def rough_ppo_runner_cfg() -> RslRlOnPolicyRunnerCfg:
    return RslRlOnPolicyRunnerCfg(
        actor=RslRlModelCfg(
            hidden_dims=(512, 256, 128),
            activation="elu",
            obs_normalization=False,
            distribution_cfg={
                "class_name": "GaussianDistribution",
                "init_std": 1.0,
                "std_type": "log",
            },
        ),
        critic=RslRlModelCfg(
            hidden_dims=(512, 256, 128),
            activation="elu",
            obs_normalization=False,
        ),
        algorithm=RslRlPpoAlgorithmCfg(
            value_loss_coef=1.0,
            use_clipped_value_loss=True,
            clip_param=0.2,
            entropy_coef=0.002,
            num_learning_epochs=5,
            num_mini_batches=4,
            learning_rate=2.0e-4,
            schedule="adaptive",
            gamma=0.99,
            lam=0.95,
            desired_kl=0.01,
            max_grad_norm=1.0,
        ),
        experiment_name="robot_rough",
        save_interval=50,
        num_steps_per_env=24,
        max_iterations=15_000,
    )


def crawl_ppo_runner_cfg() -> RslRlOnPolicyRunnerCfg:
    cfg = rough_ppo_runner_cfg()
    cfg.experiment_name = "robot_crawl"
    return cfg



