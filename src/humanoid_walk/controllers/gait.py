"""Manual gait controller for humanoid walking."""

from attr import dataclass
from humanoid_walk.controllers.joint_state import JointName, JointState, JointStates, JointTargets


class GaitController:
    def __init__(self):
        self.gait: list[Gait] | None = None

    def act(self, joint_states: JointStates) -> JointTargets:
        """Generate joint targets based on the current gait and time."""
        if self.gait is None:
            raise ValueError("Gait not set. Please set a gait before generating targets.")

        joint_targets: JointTargets = {}
        for gait in self.gait:
            joint_targets.update(gait(joint_states).targets)

        return joint_targets

    def reset(self):
        """Reset the gait controller to its initial state."""
        self.gait = None


@dataclass
class Gait:
    """A dataclass to hold the gait pattern for walking."""

    joint_state: JointStates  # relevant joints such as hip, knee, ankle
    hip: JointName
    knee: JointName
    ankle: JointName
    phase: float = 0.0  # Current phase of the gait cycle

    @property
    def targets(self) -> JointTargets:
        """Compute the target angle for a the leg joints."""
        # TODO
        return self.targets
