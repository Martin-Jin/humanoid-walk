"""Load the project's configuration from the YAML files in ``configs/``."""

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

from humanoid_walk.utils.paths import CONFIGS_DIR


# frozen dataclasses for each config section, built from ``load_yaml``.
@dataclass(frozen=True)
class WorldConfig:
    """Configuration for the simulation environment."""

    gravity_mps2: float
    sim_hz: int
    arena_width_m: float
    floor_y_m: float


@dataclass(frozen=True)
class JointConfig:
    """Configuration for a single joint of the humanoid agent."""

    name: str
    limits: tuple[float, float]
    damping: float
    max_torque: float
    friction: float


@dataclass(frozen=True)
class HumanoidConfig:
    """Configuration for the humanoid agent."""

    joint_configs: list[JointConfig]
    mass_kg: float
    height_m: float
    leg_length_m: float
    leg_width_m: float
    arm_length_m: float
    arm_width_m: float
    foot_length_m: float
    foot_width_m: float
    head_radius_m: float
    torso_length_m: float
    torso_width_m: float


@dataclass(frozen=True)
class RenderingConfig:
    """Configuration for rendering and display."""

    screen_width: int
    screen_height: int
    pixels_per_meter: float
    floor_y_px: int


@dataclass(frozen=True)
class ViewerConfig:
    """Configuration for the viewer application."""

    rendering: RenderingConfig


# Config builders
def build_viewer_config(raw: dict[str, Any]) -> ViewerConfig:
    """Build and return the viewer configuration."""
    return ViewerConfig(rendering=RenderingConfig(**raw.get("rendering", {})))


def build_world_config(raw: dict[str, Any]) -> WorldConfig:
    """Build and return the simulation configuration."""
    return WorldConfig(**raw)


def build_humanoid_config(raw: dict[str, Any]) -> HumanoidConfig:
    """Build and return the humanoid configuration."""
    return HumanoidConfig(
        **{
            **raw,
            "joint_configs": [JointConfig(**jc) for jc in raw.get("joint_configs", [])],
        }
    )


def build_game_config(world_raw: dict[str, Any], humanoid_raw: dict[str, Any]) -> GameConfig:
    """Build and return the game configuration."""
    return GameConfig(
        world=build_world_config(world_raw),
        humanoid=build_humanoid_config(humanoid_raw),
    )


@dataclass(frozen=True)
class GameConfig:
    """Configuration for the game."""

    world: WorldConfig
    humanoid: HumanoidConfig


@dataclass(frozen=True)
class Config:
    """Configuration for the humanoid walk project."""

    game: GameConfig
    viewer: ViewerConfig


def load_yaml(name: str, directory: Path = CONFIGS_DIR) -> dict[str, Any]:
    """Read ``directory/name`` and return its top-level mapping."""
    with open(directory / name, encoding="utf-8") as file:
        data = yaml.safe_load(file)
    if not isinstance(data, dict):
        raise ValueError(f"{name} must contain a YAML mapping at the top level.")
    return data


def load_config() -> Config:
    """Load and return the project's configuration."""
    sim_config = load_yaml("sim.yaml")
    humanoid_config = load_yaml("humanoid.yaml")
    viewer_config = load_yaml("viewer.yaml")
    # Validate that config exists.
    if "world" not in sim_config:
        raise ValueError("sim.yaml must contain a 'world' section.")
    if "humanoid" not in humanoid_config:
        raise ValueError("humanoid.yaml must contain a 'humanoid' section.")
    if "rendering" not in viewer_config:
        raise ValueError("viewer.yaml must contain a 'rendering' section.")

    return Config(
        game=build_game_config(sim_config["world"], humanoid_config["humanoid"]),
        viewer=build_viewer_config(viewer_config),
    )
