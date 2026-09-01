from mjlab.tasks.registry import register_mjlab_task
from .config.env_cfgs import flat_env_cfg, rough_env_cfg, crawl_env_cfg
from .config.rl_cfg import flat_ppo_runner_cfg, rough_ppo_runner_cfg, crawl_ppo_runner_cfg

register_mjlab_task(
    task_id="Robot-Flat-v0",
    env_cfg=flat_env_cfg(),
    play_env_cfg=flat_env_cfg(play=True),
    rl_cfg=flat_ppo_runner_cfg(),
)

register_mjlab_task(
    task_id="Robot-Rough-v0",
    env_cfg=rough_env_cfg(),
    play_env_cfg=rough_env_cfg(play=True),
    rl_cfg=rough_ppo_runner_cfg(),
)

register_mjlab_task(
    task_id="Robot-Crawl-v0",
    env_cfg=crawl_env_cfg(),
    play_env_cfg=crawl_env_cfg(play=True),
    rl_cfg=crawl_ppo_runner_cfg(),
)

