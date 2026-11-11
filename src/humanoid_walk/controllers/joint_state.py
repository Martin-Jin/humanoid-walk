"""Shared joint state."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from humanoid_walk.controllers.joint_control import JointController


@dataclass
class JointState:
    """A dataclass to hold the state of the joint."""

    current: float
    velocity: float = 0.0
    dt: float = 0.01  # Timestep in seconds, used by the integral term


@dataclass
class BodyState:
    """A dataclass to hold the state of the humanoid body."""

    position: tuple[float, float]  # Of COM
    joint_states: dict[str, JointState]
    velocity: tuple[float, float] = (0.0, 0.0)  # Of COM
    angular_velocity: float = 0.0  # About COM
    acceleration: tuple[float, float] = (0.0, 0.0)  # Of COM


@dataclass
class JointCommand:
    """A dataclass to hold the command for a joint."""

    target: float
    state: JointState


@dataclass
class JointControllers:
    """Defines the names of the joints and their corresponding controllers for the humanoid robot."""

    neck: JointController
    shoulder_l: JointController
    shoulder_r: JointController
    hip_l: JointController
    hip_r: JointController
    knee_l: JointController
    knee_r: JointController
    ankle_l: JointController
    ankle_r: JointController


@dataclass
class JointCommands:
    """Defines the commands for each joint of the humanoid robot."""

    neck: JointCommand
    shoulder_l: JointCommand
    shoulder_r: JointCommand
    hip_l: JointCommand
    hip_r: JointCommand
    knee_l: JointCommand
    knee_r: JointCommand
    ankle_l: JointCommand
    ankle_r: JointCommand
