"""Defines the joint control logic for the humanoid robot,
and provides a controller for the entire humanoid body."""

from dataclasses import replace

from humanoid_walk.config import Config
from humanoid_walk.controllers.controller import Controller
from humanoid_walk.controllers.joint_state import (
    BodyState,
    JointCommand,
    JointCommands,
    JointControllers,
)


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

    def compute_torque(self, command: JointCommand) -> float:
        """Compute the torque to apply to the joint."""
        # Clamp the target to the joint limits without mutating the caller's command
        command = replace(command, target=self.clamp_angle(command.target))

        if self.controller:
            torque = self.controller.compute_action(command)
        else:
            error = command.target - command.state.current
            torque = self.k * error * self.max_torque - self.damping * command.state.velocity

        return self.clamp_torque(torque)

    def clamp_angle(self, angle: float) -> float:
        """Clamp the angle to the joint limits."""
        return max(min(angle, self.limits[1]), self.limits[0])

    def clamp_torque(self, torque: float) -> float:
        """Clamp the torque to the maximum allowed torque."""
        return max(min(torque, self.max_torque), -self.max_torque)


class BodyController:
    """A controller for the entire humanoid robot, managing multiple joints."""

    def __init__(
        self,
        config: Config,
        joint_controllers: JointControllers,
        body_state: BodyState | None = None,
    ):
        self.config = config
        self.joint_controllers = joint_controllers
        self.body_state = body_state  # This will hold the current state of the humanoid body

    def compute_torques(self, commands: JointCommands) -> dict[str, float]:
        """Compute the torques for all joints based on their commands."""
        torques = {}
        for name, command in vars(commands).items():
            controller = getattr(self.joint_controllers, name, None)
            if controller is None:
                raise ValueError(f"Joint '{name}' not found in the controller.")
            torques[name] = controller.compute_torque(command)
        return torques
