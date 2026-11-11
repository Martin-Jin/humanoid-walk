"""Shared joint state."""

from dataclasses import dataclass


@dataclass
class JointState:
    """A dataclass to hold the state of the joint."""

    current: float
    target: float
    velocity: float = 0.0
    dt: float = 0.01  # Timestep in seconds, used by the integral term
