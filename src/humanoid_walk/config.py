"""Load the project's configuration from the YAML files in ``configs/``."""

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Self

import yaml

import math
from humanoid_walk.utils.paths import CONFIGS_DIR
from humanoid_walk.controllers.joint_state import JointName
from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class WorldConfig:
    """Configuration for the simulation environment."""

    gravity_mps2: float
    sim_hz: int
    arena_width_m: float
    floor_y_m: float
    floor_friction: float
    physics_substeps: int
    ground_thickness_m: float
    spawn_x_frac: float  # humanoid spawns at this fraction of the screen width
    max_body_speed_mps: float  # cap on any body's speed

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
    ankle_offset_m: float  # the ankle sits this far behind the foot centre
    head_radius_m: float
    torso_length_m: float
    torso_width_m: float
    part_masses_kg: dict[str, float]
    pin_torso: bool

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
class AnkleGaitConfig:
    """Foot angles relative to the ground in rad (+ toes down, - toes up)."""

    stance_angle: float
    swing_angle: float
    pitch_ref: float  # 0-1, how much of the torso pitch is subtracted from the ankle target


@dataclass(frozen=True, slots=True)
class BackwardOverrides:
    """Values that replace the defaults when walking backward."""

    step_height_m: float
    touchdown_offset_m: float


@dataclass(frozen=True, slots=True)
class GaitConfig:
    """Configuration for the foot-trajectory gait."""

    period_s: float
    duty_factor: float
    step_length_m: float
    step_height_m: float
    hip_height_m: float
    touchdown_offset_m: float
    max_leg_ratio: float  # commanded leg never longer than this x leg_length_m
    balance_blend: float  # fraction of stance over which balance terms fade in/out
    ankle: AnkleGaitConfig
    phase_offsets: dict[str, float]  # "left" / "right" -> fraction of the cycle
    backward: BackwardOverrides
    ankle_balance: dict[str, float]
    hip_balance: dict[str, float]
    stand_only: bool
    arm: dict[str, float]  # arm swing parameters
    balance_gate: dict[str, float]  # thresholds for pausing/resuming the gait clock
    start_up: dict[str, float]  # startup sequence durations in seconds
    push_off: dict[str, float]  # late-stance leg extension and ankle plantarflexion
    touchdown: dict[str, float]  # closed-loop touchdown (contact-seeking reach, clock hold)

    def __post_init__(self):
        if self.period_s <= 0:
            raise ValueError(f"period_s must be positive, got {self.period_s}")
        if not 0.0 < self.duty_factor < 1.0:
            raise ValueError(f"duty_factor must be between 0 and 1, got {self.duty_factor}")
        if set(self.phase_offsets) != {"left", "right"}:
            raise ValueError("gait legs must define exactly 'left' and 'right'")

    @classmethod
    def from_dict(cls, raw: dict[str, Any]) -> Self:
        plain = {k: v for k, v in raw.items() if k not in ("ankle", "legs", "backward")}
        return cls(
            **plain,
            ankle=AnkleGaitConfig(**raw["ankle"]),
            phase_offsets={leg: v["phase_offset"] for leg, v in raw["legs"].items()},
            backward=BackwardOverrides(**raw["backward"]),
        )


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
    gait: GaitConfig


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


def validate_gait(gait: GaitConfig, humanoid: HumanoidConfig) -> None:
    """Reject gaits whose foot path the leg cannot reach, and out-of-range gait settings."""
    leg = humanoid.leg_length_m
    limit = gait.max_leg_ratio * leg  # longest leg the controller will command
    td = gait.touchdown
    push = gait.push_off

    # Reach: worst-case foot position relative to the hip
    max_offset = max(abs(gait.touchdown_offset_m), abs(gait.backward.touchdown_offset_m))
    drop = gait.hip_height_m + push["extend_m"] + td["reach_max_m"]
    reach_x = gait.step_length_m / 2 + max_offset + push["back_m"]
    reach = math.hypot(reach_x, drop)

    if drop > limit:
        raise ValueError(
            f"hip_height_m + extend_m + reach_max_m = {drop:.3f} m exceeds "
            f"max_leg_ratio * leg_length_m = {limit:.3f} m."
        )
    if reach >= limit:
        raise ValueError(
            f"Gait needs {reach:.3f} m of reach but the limit is {limit:.3f} m "
            f"({gait.max_leg_ratio} x leg_length_m of {leg})."
        )

    # Range checks on the new parameters
    if not 0.0 < gait.max_leg_ratio < 1.0:
        raise ValueError("max_leg_ratio must be between 0 and 1.")
    if not 0.0 < gait.duty_factor < 1.0:
        raise ValueError("duty_factor must be between 0 and 1.")
    if not 0.0 <= gait.balance_blend <= 0.5:
        raise ValueError("balance_blend must be between 0 and 0.5.")
    if not 0.0 <= push["start"] < 1.0:
        raise ValueError("push_off.start must be in [0, 1).")
    if not 0.0 <= gait.start_up["start_phase"] < 1.0:
        raise ValueError("start_up.start_phase must be in [0, 1).")
    if not 0.0 < push["unwind"] <= 1.0:
        raise ValueError("push_off.unwind must be in (0, 1].")
    if not 0.0 <= gait.ankle.pitch_ref <= 1.0:
        raise ValueError("ankle.pitch_ref must be between 0 and 1.")
    if gait.hip_balance["max_correction"] <= 0 or gait.ankle_balance["max_correction"] <= 0:
        raise ValueError("balance max_correction values must be positive.")
    if not 0.0 <= td["landing_start"] < 1.0:
        raise ValueError("touchdown.landing_start must be in [0, 1).")
    if not td["reach_start"] < td["hold_from"] <= 1.0:
        raise ValueError("touchdown needs reach_start < hold_from <= 1.")
    if td["reach_max_m"] < 0 or td["reach_rate_mps"] < 0:
        raise ValueError("touchdown.reach_max_m and reach_rate_mps must be >= 0.")
    if td["reach_decay_s"] <= 0 or td["hold_max_s"] < 0:
        raise ValueError("touchdown.reach_decay_s must be > 0 and hold_max_s >= 0.")


def load_config() -> Config:
    humanoid = HumanoidConfig.from_dict(load_yaml("humanoid.yaml"))
    controllers = {
        k: ControllerConfig.from_dict(v)
        for k, v in load_yaml("controller.yaml")["Controller"].items()
    }
    gait = GaitConfig.from_dict(load_yaml("gait.yaml")["gait"])
    validate_gait(gait, humanoid)

    return Config(
        world=WorldConfig.from_dict(load_yaml("sim.yaml")["world"]),
        humanoid=humanoid,
        rendering=RenderingConfig.from_dict(load_yaml("viewer.yaml")["rendering"]),
        controllers=controllers,
        joint_controller_names=build_joint_controller_names(
            controllers, {name.value for name in JointName}
        ),
        gait=gait,
    )
