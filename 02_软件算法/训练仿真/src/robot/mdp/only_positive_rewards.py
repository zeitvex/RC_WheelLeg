"""HIMLoco-style only_positive_rewards: clip total step reward to >= 0.

Mechanics:
- PPO Advantage = reward + gamma * V(s') - V(s)
- During early policy learning, when penalties greatly exceed positive rewards,
  the total step reward can become highly negative, producing negative advantages.
- This often causes policies to learn to intentionally "fall down immediately" to terminate
  the episode and minimize long-term negative accumulation.
- Clamping the total step reward to >= 0 prevents the penalty from generating negative
  step feedback, ensuring penalty terms can only offset positive gains without directing the
  advantage signal incorrectly.
- Individual term episode sums are still recorded correctly for tracking and diagnostics.
"""

from __future__ import annotations

import torch
from mjlab.managers.reward_manager import RewardManager

_original_compute = RewardManager.compute


def _compute_with_clamp(self: RewardManager, dt: float) -> torch.Tensor:
    """Compute step rewards and clamp the final total reward to >= 0."""
    reward = _original_compute(self, dt)
    # In-place clamp on the same buffer reference for consistency with original API
    return torch.clamp(reward, min=0.0)


def enable_only_positive_rewards() -> None:
    """Enable HIMLoco-style positive-only step reward clamping globally.

    Idempotent: calling this multiple times is completely safe.
    """
    if RewardManager.compute is not _compute_with_clamp:
        RewardManager.compute = _compute_with_clamp


def disable_only_positive_rewards() -> None:
    """Disable positive-only step reward clamping, restoring default behavior."""
    if RewardManager.compute is not _original_compute:
        RewardManager.compute = _original_compute
