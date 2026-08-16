"""Competition obstacle terrains for locomotion tasks.

Implemented in accordance with the interface specifications in
mjlab/src/mjlab/terrains/primitive_terrains.py. Terrain difficulty (difficulty in [0, 1])
is automatically supplied via TerrainGeneratorCfg and curriculum.
"""

from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np

from mjlab.terrains.terrain_generator import SubTerrainCfg, TerrainGeometry, TerrainOutput
from mjlab.terrains.utils import make_plane
from mjlab.terrains import BoxPyramidStairsTerrainCfg
from mjlab.utils.color import brand_ramp

# Color constants matching primitive_terrains.py style
_COLOR_ORANGE = (0.95, 0.55, 0.10)
_COLOR_PURPLE = (0.60, 0.20, 0.80)


@dataclass(kw_only=True)
class RCWallTerrainCfg(SubTerrainCfg):
    """Triple transverse wall obstacle terrain representing repeated race high walls.

    The robot must sprint from the flat platform, vault over three walls, and proceed.
    As difficulty scales from 0 to 1, the wall height increases linearly from
    wall_height_range[0] to wall_height_range[1].
    
    The actual physical wall specs are 1.0m width, 0.3m height, and 0.05m thickness.
    Difficulty range goes up to 0.35m to slightly exceed actual competition difficulty.
    """

    wall_height_range: tuple[float, float] = (0.0, 0.35)
    """Wall height range (m), linearly interpolated with difficulty."""
    wall_thickness: float = 0.05
    """Wall thickness (m)."""
    wall_length_frac: float = 0.8
    """Wall length fraction of the terrain width (leaving gaps for visualization/debugging)."""
    platform_width: float = 1.5
    """Sprint platform width (m)."""
    wall_centers_x: tuple[float, float, float] = (2.9, 4.45, 6.0)
    """Wall center positions along x, spaced to keep a short sprint, two recovery gaps, and exit room."""

    def function(
        self,
        difficulty: float,
        spec: mujoco.MjSpec,
        rng: np.random.Generator,
    ) -> TerrainOutput:
        del rng
        body = spec.body("terrain")
        geometries: list[TerrainGeometry] = []

        wall_height = self.wall_height_range[0] + difficulty * (
            self.wall_height_range[1] - self.wall_height_range[0]
        )

        # -- Ground floor base plane --
        floor_boxes = make_plane(body, self.size, 0.0, center_zero=False)
        floor_color = (0.45, 0.45, 0.45, 1.0)
        for box in floor_boxes:
            geometries.append(TerrainGeometry(geom=box, color=floor_color))

        if wall_height < 1e-3:
            origin = np.array([self.size[0] / 2, self.size[1] / 2, 0.0])
            return TerrainOutput(origin=origin, geometries=geometries)

        # -- Wall geometry: three transverse walls oriented along y-axis --
        wall_length = self.wall_length_frac * self.size[1]
        cy = self.size[1] / 2

        wall_color = brand_ramp(_COLOR_ORANGE, difficulty)

        for cx in self.wall_centers_x:
            wall_geom = body.add_geom(
                type=mujoco.mjtGeom.mjGEOM_BOX,
                size=(
                    self.wall_thickness / 2,   # half-size x
                    wall_length / 2,           # half-size y
                    wall_height / 2,           # half-size z
                ),
                pos=(cx, cy, wall_height / 2),
            )
            geometries.append(TerrainGeometry(geom=wall_geom, color=wall_color))

        # Spawn origin is set to the left platform area to allow a short sprint
        # before the first wall and limited recovery space between subsequent walls.
        origin = np.array([1.5, cy, 0.0])
        return TerrainOutput(origin=origin, geometries=geometries)


@dataclass(kw_only=True)
class RCLowBarTerrainCfg(SubTerrainCfg):
    """Low clearance bar obstacle terrain.

    A horizontal bar is placed along the y-axis, requiring the robot to crouch and duck under.
    As difficulty scales from 0 to 1, the clearance height decreases linearly from
    clearance_range[1] to clearance_range[0] (harder is lower).
    
    The actual competition spec is 0.30m離地 and 1.0m width.
    Here the range is configured as 0.25m to 0.35m to cover and exceed the standard specs.
    """

    clearance_range: tuple[float, float] = (0.25, 0.35)
    """Bar ground clearance range (m). difficulty=0 -> highest/easiest, difficulty=1 -> lowest/hardest."""
    bar_radius: float = 0.025
    """Bar cross-section radius (m)."""
    bar_length_frac: float = 0.85
    """Bar length fraction of the terrain width (leaving gaps for visualization/debugging)."""

    def function(
        self,
        difficulty: float,
        spec: mujoco.MjSpec,
        rng: np.random.Generator,
    ) -> TerrainOutput:
        del rng
        body = spec.body("terrain")
        geometries: list[TerrainGeometry] = []

        clearance = self.clearance_range[1] - difficulty * (
            self.clearance_range[1] - self.clearance_range[0]
        )
        bar_z = clearance + self.bar_radius  # Cylinder center height

        # -- Ground floor --
        floor_boxes = make_plane(body, self.size, 0.0, center_zero=False)
        floor_color = (0.45, 0.45, 0.45, 1.0)
        for box in floor_boxes:
            geometries.append(TerrainGeometry(geom=box, color=floor_color))

        # -- Low bar cylinder (oriented along y-axis) --
        bar_length = self.bar_length_frac * self.size[1]
        cx = self.size[0] / 2
        cy = self.size[1] / 2

        bar_color = brand_ramp(_COLOR_PURPLE, difficulty)

        # MuJoCo cylinder defaults to z-axis orientation. Rotate 90 degrees around x-axis
        # to align with y-axis: quat [cos(pi/4), sin(pi/4), 0, 0] -> [0.7071, 0.7071, 0.0, 0.0]
        bar_geom = body.add_geom(
            type=mujoco.mjtGeom.mjGEOM_CYLINDER,
            size=(self.bar_radius, bar_length / 2),
            pos=(cx, cy, bar_z),
        )
        bar_geom.quat = np.array([
            np.cos(np.pi / 4), np.sin(np.pi / 4), 0.0, 0.0
        ])
        geometries.append(TerrainGeometry(geom=bar_geom, color=bar_color))

        # -- Side supporting posts --
        post_color = bar_color
        post_half_h = bar_z / 2
        for y_off in [-bar_length / 2, bar_length / 2]:
            post = body.add_geom(
                type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                size=(self.bar_radius, post_half_h),
                pos=(cx, cy + y_off, post_half_h),
            )
            geometries.append(TerrainGeometry(geom=post, color=post_color))

        # Spawn origin is set to the left platform area (e.g., 1.5m) to allow
        # the robot ample room to sprint/accelerate instead of spawning directly under the bar.
        origin = np.array([1.5, cy, 0.0])
        return TerrainOutput(origin=origin, geometries=geometries)


@dataclass(kw_only=True)
class RCPyramidStairsTerrainCfg(BoxPyramidStairsTerrainCfg):
    """Refactored pyramid stairs terrain with the spawn origin shifted to the left edge.

    This ensures the robot can sprint/accelerate on flat ground before ascending,
    substantially reducing early falls or flips on vertical steps.
    """

    def function(
        self,
        difficulty: float,
        spec: mujoco.MjSpec,
        rng: np.random.Generator,
    ) -> TerrainOutput:
        output = super().function(difficulty, spec, rng)
        cy = self.size[1] / 2
        # Relocate the spawn origin to the left platform zone
        output.origin = np.array([1.5, cy, 0.0])
        return output
