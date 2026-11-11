"""Load the project's configuration from the YAML files in ``configs/``."""

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Self

import yaml

from humanoid_walk.utils.paths import CONFIGS_DIR


@dataclass(frozen=True, slots=True)
class WorldConfig:
    """Configuration for the simulation environment."""

    gravity_mps2: float
    sim_hz: int
    arena_width_m: float
    floor_y_m: float

    @classmethod
    def from_dict(cls, raw: dict[str, Any]) -> Self:
        return cls(**raw)


@dataclass(frozen=True, slots=True)
class JointConfig:
    """Configuration for a single joint of the humanoid agent."""

    name: str
    type: str
    limits: tuple[float, float]
    damping: float
    max_torque: float
    friction: float

    @classmethod
    def from_dict(cls, raw: dict[str, Any]) -> Self:
        return cls(**{**raw, "limits": tuple(raw["limits"])})


@dataclass(frozen=True, slots=True)
class HumanoidConfig:
    """Configuration for the humanoid agent."""

    joint_configs: tuple[JointConfig, ...]
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

    @classmethod
    def from_dict(cls, raw: dict[str, Any]) -> Self:
        joints = tuple(JointConfig.from_dict(jc) for jc in raw["joint_configs"])
        return cls(**{**raw, "joint_configs": joints})


@dataclass(frozen=True, slots=True)
class RenderingConfig:
    """Configuration for rendering and display."""

    screen_width: int
    screen_height: int
    pixels_per_meter: float
    floor_y_px: int

    @classmethod
    def from_dict(cls, raw: dict[str, Any]) -> Self:
        return cls(**raw)


@dataclass(frozen=True, slots=True)
class ControllerConfig:
    """Configuration for a single controller type."""

    type: str
    gains: dict[str, float]

    @classmethod
    def from_dict(cls, raw: dict[str, Any]) -> Self:
        return cls(type=raw["type"], gains=dict(raw["gains"]))


@dataclass(frozen=True, slots=True)
class Config:
    """Configuration for the humanoid walk project."""

    world: WorldConfig
    humanoid: HumanoidConfig
    rendering: RenderingConfig
    controllers: dict[str, ControllerConfig]
    joint_controller_types: dict[str, str]  # keyed by JointName value, e.g. "hip_l"


def load_yaml(name: str, directory: Path = CONFIGS_DIR) -> dict[str, Any]:
    """Read ``directory/name`` and return its top-level mapping."""
    with open(directory / name, encoding="utf-8") as file:
        data = yaml.safe_load(file)
    if not isinstance(data, dict):
        raise ValueError(f"{name} must contain a YAML mapping at the top level.")
    return data


def load_config() -> Config:
    raw = load_yaml("controller.yaml")
    return Config(
        world=WorldConfig.from_dict(load_yaml("sim.yaml")["world"]),
        humanoid=HumanoidConfig.from_dict(load_yaml("humanoid.yaml")),
        rendering=RenderingConfig.from_dict(load_yaml("viewer.yaml")["rendering"]),
        controllers={k: ControllerConfig.from_dict(v) for k, v in raw["Controller"].items()},
        joint_controller_types={k: v["type"] for k, v in raw["JointController"].items()},
    )
