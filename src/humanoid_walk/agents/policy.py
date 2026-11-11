"""Policies decide joint targets from the current state."""

from abc import ABC, abstractmethod

from humanoid_walk.controllers.joint_state import JointName, JointStates, JointTargets


class Policy(ABC):
    """Maps joint states to target joint angles (rad)."""

    @abstractmethod
    def act(self, states: JointStates) -> JointTargets:
        """Return a target angle for every joint."""

    def reset(self) -> None:
        """Reset any internal state at episode start."""


class HoldPolicy(Policy):
    """Hold every joint at a fixed angle (default 0 rad, the standing pose)."""

    def __init__(self, pose: JointTargets | None = None):
        self.pose = pose or {name: 0.0 for name in JointName}

    def act(self, states: JointStates) -> JointTargets:
        return dict(self.pose)
