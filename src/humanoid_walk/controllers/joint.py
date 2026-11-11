"""Joint controller for the humanoid robot."""

from humanoid_walk.config import Config
from humanoid_walk.controllers.controller import Controller
from humanoid_walk.controllers.joint_state import JointState


class JointController:
    """A controller for a single joint of the humanoid robot."""

    def __init__(self, config: Config):
        self.config = config
        self.name = config.name
        self.limits = config.limits
        self.damping = config.damping
        self.max_torque = config.max_torque
        self.friction = config.friction
        self.k = 1.0  # Proportional gain for the default controller
        self.controller: Controller | None = None

    def compute_torque(self, state: JointState) -> float:
        """Compute the torque to apply to the joint."""
        # Clamp the target to the joint limits
        state.target = self.clamp_angle(state.target)

        if self.controller:
            torque = self.controller.compute_action(state)
        else:
            error = state.target - state.current
            torque = self.k * error * self.max_torque - self.damping * state.velocity

        return self.clamp_torque(torque)

    def clamp_angle(self, angle: float) -> float:
        """Clamp the angle to the joint limits."""
        return max(min(angle, self.limits[1]), self.limits[0])

    def clamp_torque(self, torque: float) -> float:
        """Clamp the torque to the maximum allowed torque."""
        return max(min(torque, self.max_torque), -self.max_torque)
