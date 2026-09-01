"""Discrete mode command term for multi-conditioned policy training.

Architecture is modeled after UniformVelocityCommand / UniformVelocityCommandCfg in
mjlab/tasks/velocity/mdp/velocity_command.py.

During training: mode_id in {0, 1, 2} is sampled randomly per episode based on mode_probs.
During deployment: ModeCommandCfg.fixed_mode is set to 0, 1, or 2, which can be modified externally.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import torch

from mjlab.managers.command_manager import CommandTerm, CommandTermCfg

if TYPE_CHECKING:
    from mjlab.envs.manager_based_rl_env import ManagerBasedRlEnv


# Locomotion Mode ID Constants
MODE_WALK = 0
MODE_CLIMB = 1
MODE_CROUCH = 2
NUM_MODES = 3


class ModeCommand(CommandTerm):
    """Discrete locomotion mode command term.

    The command tensor shape is [num_envs, 1], holding normalized mode values in [-1.0, 1.0].
    Mapping details: mode_id -> observation value
      0 (walk)   -> -1.0
      1 (climb)  ->  0.0
      2 (crouch) -> +1.0
    Using normalized scalar values keeps observation dimensions minimal (1D).
    """

    cfg: ModeCommandCfg

    def __init__(self, cfg: ModeCommandCfg, env: ManagerBasedRlEnv):
        super().__init__(cfg, env)
        # Stores current integer mode_id for each environment
        self._mode_id = torch.zeros(
            self.num_envs, dtype=torch.long, device=self.device
        )
        # Normalized command observations: shape [num_envs, 1]
        self._command = torch.zeros(self.num_envs, 1, device=self.device)

        # Pre-calculate cumulative sum of mode probabilities (normalized for multinomial sampling)
        probs = torch.tensor(cfg.mode_probs, dtype=torch.float32, device=self.device)
        self._mode_probs = probs / probs.sum()

    @property
    def command(self) -> torch.Tensor:
        """Normalized mode command observation tensor of shape [num_envs, 1]."""
        return self._command

    def _update_metrics(self) -> None:
        pass

    def _resample_command(self, env_ids: torch.Tensor) -> None:
        if self.cfg.fixed_mode is not None:
            # Deployment mode: fixed mode
            self._mode_id[env_ids] = self.cfg.fixed_mode
        else:
            # Training mode: map modes based on active terrain type if configured
            terrain = getattr(self._env.scene, "terrain", None)
            terrain_types = getattr(terrain, "terrain_types", None)
            if self.cfg.terrain_mode_mapping is not None and terrain_types is not None:
                mapping = torch.tensor(
                    self.cfg.terrain_mode_mapping, dtype=torch.long, device=self.device
                )
                types = terrain_types[env_ids]
                # Defensive clamp to prevent out of bounds indexing
                types_safe = torch.clamp(types, 0, len(mapping) - 1)
                self._mode_id[env_ids] = mapping[types_safe]
            else:
                # Fallback to random multinomial sampling
                sampled = torch.multinomial(
                    self._mode_probs.expand(len(env_ids), -1),
                    num_samples=1,
                    replacement=True,
                ).squeeze(-1)
                self._mode_id[env_ids] = sampled

        # Update normalized command observations
        self._update_obs_from_mode_id(env_ids)

    def _update_command(self) -> None:
        # Modes are persistent across episodes, no step-level command updates required
        pass

    def _update_obs_from_mode_id(self, env_ids: torch.Tensor) -> None:
        """Map integer mode_id to normalized observation: walk=0 -> -1.0, climb=1 -> 0.0, crouch=2 -> +1.0."""
        mode = self._mode_id[env_ids].float()
        # Linear projection: [0, NUM_MODES - 1] -> [-1.0, +1.0]
        normalized = 2.0 * mode / (NUM_MODES - 1) - 1.0
        self._command[env_ids, 0] = normalized

    def create_gui(
        self,
        name: str,
        server,  # ViserServer instance (late-bound to avoid top-level dependency)
        get_env_idx,
        on_change=None,
        request_action=None,
    ) -> None:
        """Construct interactive mode selection controls in the Viser visualizer GUI.

        When enabled is active, slider values override target environment mode IDs in real-time.
        Discrete modes: 0=walk, 1=climb, 2=crouch (step size = 1).
        """
        with server.gui.add_folder(f"{name.capitalize()} (0=walk 1=climb 2=crouch)"):
            enabled = server.gui.add_checkbox("Enable", initial_value=False)
            mode_slider = server.gui.add_slider(
                "mode_id",
                min=0,
                max=NUM_MODES - 1,
                step=1,
                initial_value=0,
            )

        # Store GUI handles for compute() execution
        self._gui_enabled = enabled
        self._gui_slider = mode_slider
        self._gui_get_env_idx = get_env_idx

    def compute(self, dt: float) -> None:
        """Perform step updates: override mode_id with interactive sliders if visualizer GUI is active."""
        super().compute(dt)
        if (
            getattr(self, "_gui_enabled", None) is not None
            and self._gui_enabled.value
            and self._gui_get_env_idx is not None
        ):
            idx = self._gui_get_env_idx()
            new_mode = int(round(self._gui_slider.value))
            env_ids = torch.tensor([idx], dtype=torch.long, device=self.device)
            self._mode_id[env_ids] = new_mode
            self._update_obs_from_mode_id(env_ids)

    def get_mode_id(self, env_ids: torch.Tensor | None = None) -> torch.Tensor:
        """Retrieve active environment mode IDs for reward/termination conditioning."""
        if env_ids is None:
            return self._mode_id
        return self._mode_id[env_ids]


@dataclass(kw_only=True)
class ModeCommandCfg(CommandTermCfg):
    """Discrete ModeCommand configuration class."""

    resampling_time_range: tuple[float, float] = (20.0, 20.0)
    """Discrete mode resampling duration (resampled at episode boundary, equal to episode length)."""

    mode_probs: tuple[float, ...] = (0.5, 0.3, 0.2)
    """Probabilities for discrete modes (walk, climb, crouch). Normalized automatically."""

    fixed_mode: int | None = None
    """None for active random sampling during training; 0/1/2 for deployment."""

    terrain_mode_mapping: tuple[int, ...] | None = None
    """Force maps discrete modes based on environment sub-terrain grid index.
    
    E.g., (0, 0, 0, 0, 1, 1, 2) corresponds to walk mode for first four sub-terrains, 
    climb mode for the next two sub-terrains, and crouch mode for the last sub-terrain.
    Overrides mode_probs if active.
    """

    def build(self, env: ManagerBasedRlEnv) -> ModeCommand:
        return ModeCommand(self, env)
