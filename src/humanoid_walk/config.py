"""Load the project's configuration from the YAML files in ``configs/``."""

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Self

import yaml

from humanoid_walk.utils.paths import CONFIGS_DIR
from humanoid_walk.controllers.joint_state import JointName


@dataclass(frozen=True, slots=True)
class WorldConfig:
    """Configuration for the simulation environment."""

    gravity_mps2: float
    sim_hz: int
    arena_width_m: float
    floor_y_m: float
    floor_friction: float

    @classmethod
    def from_dict(cls, raw: dict[str, Any]) -> Self:
        return cls(**raw)


@dataclass(frozen=True, slots=True)
class JointConfig:
    """Configuration for a single joint of the humanoid agent."""

    name: str
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
    part_masses_kg: dict[str, float]

    def __post_init__(self):
        total = (
            self.part_masses_kg["head"]
            + self.part_masses_kg["torso"]
            + 2 * self.part_masses_kg["arm"]
            + 2 * self.part_masses_kg["thigh"]
            + 2 * self.part_masses_kg["shin"]
            + 2 * self.part_masses_kg["foot"]
        )
        if abs(total - self.mass_kg) > 1e-6:
            raise ValueError(f"part_masses_kg sums to {total}, but mass_kg is {self.mass_kg}")

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
    """Configuration for a single controller type and the joints it drives."""

    type: str
    gains: dict[str, float]
    joints: tuple[str, ...]

    @classmethod
    def from_dict(cls, raw: dict[str, Any]) -> Self:
        return cls(type=raw["type"], gains=dict(raw["gains"]), joints=tuple(raw["joints"]))


@dataclass(frozen=True, slots=True)
class Config:
    """Configuration for the humanoid walk project."""

    world: WorldConfig
    humanoid: HumanoidConfig
    rendering: RenderingConfig
    controllers: dict[str, ControllerConfig]
    joint_controller_names: dict[
        str, str
    ]  # joint name -> key in `controllers`, e.g. "hip_l" -> "PD_HIP"


def load_yaml(name: str, directory: Path = CONFIGS_DIR) -> dict[str, Any]:
    """Read ``directory/name`` and return its top-level mapping."""
    with open(directory / name, encoding="utf-8") as file:
        data = yaml.safe_load(file)
    if not isinstance(data, dict):
        raise ValueError(f"{name} must contain a YAML mapping at the top level.")
    return data


def build_joint_controller_names(
    controllers: dict[str, ControllerConfig], joint_names: set[str]
) -> dict[str, str]:
    """Map each joint name to the key of the controller that drives it."""
    mapping: dict[str, str] = {}
    for name, controller in controllers.items():
        for joint in controller.joints:
            if joint in mapping:
                raise ValueError(f"Joint {joint!r} is in both {mapping[joint]!r} and {name!r}.")
            mapping[joint] = name

    missing, unknown = joint_names - mapping.keys(), mapping.keys() - joint_names
    if missing or unknown:
        raise ValueError(
            f"Controller joints mismatch: missing={sorted(missing)}, unknown={sorted(unknown)}"
        )
    return mapping


def load_config() -> Config:
    humanoid = HumanoidConfig.from_dict(load_yaml("humanoid.yaml"))
    controllers = {
        k: ControllerConfig.from_dict(v)
        for k, v in load_yaml("controller.yaml")["Controller"].items()
    }

    return Config(
        world=WorldConfig.from_dict(load_yaml("sim.yaml")["world"]),
        humanoid=humanoid,
        rendering=RenderingConfig.from_dict(load_yaml("viewer.yaml")["rendering"]),
        controllers=controllers,
        joint_controller_names=build_joint_controller_names(
            controllers, {name.value for name in JointName}
        ),
    )
