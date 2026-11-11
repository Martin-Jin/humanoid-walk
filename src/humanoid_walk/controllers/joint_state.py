"""Shared joint state."""

from dataclasses import dataclass
from enum import Enum

# Shared side colours (RGB): left limbs green, right limbs orange
LEFT_RGB = (40, 170, 90)
RIGHT_RGB = (230, 120, 30)


class JointName(str, Enum):
    """The only joints that can exist. Single source of truth for joint names."""

    NECK = "neck"
    SHOULDER_L = "shoulder_l"
    SHOULDER_R = "shoulder_r"
    HIP_L = "hip_l"
    HIP_R = "hip_r"
    KNEE_L = "knee_l"
    KNEE_R = "knee_r"
    ANKLE_L = "ankle_l"
    ANKLE_R = "ankle_r"


@dataclass
class JointState:
    """A dataclass to hold the state of the joint."""

    current: float
    velocity: float = 0.0
    dt: float = 0.01  # Timestep in seconds, used by the integral term


JointTargets = dict[JointName, float]
JointStates = dict[JointName, JointState]


@dataclass
class BodyState:
    """A dataclass to hold the state of the humanoid body."""

    position: tuple[float, float]  # Of COM
    joint_states: JointStates
    velocity: tuple[float, float] = (0.0, 0.0)  # Of COM
    angular_velocity: float = 0.0  # About COM
    acceleration: tuple[float, float] = (0.0, 0.0)  # Of COM
