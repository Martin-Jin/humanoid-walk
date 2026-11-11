"""Implements of several controllers for the humanoid walk environment."""

from abc import ABC, abstractmethod

from humanoid_walk.config import Config
from humanoid_walk.controllers.joint_state import JointCommand


class Controller(ABC):
    """A controller for the humanoid robot."""

    def __init__(self, config: Config):
        self.config = config
        self.gains = config.gains
        self.type = config.type

    @abstractmethod
    def compute_action(self, command: JointCommand) -> float:
        """Compute the action to take based on the current joint state."""

    def reset(self) -> None:
        """Reset any internal state."""


class PController(Controller):
    """A simple proportional controller."""

    def __init__(self, config: Config):
        super().__init__(config)
        self.kp = self.gains.get("Kp", 1.0)

    def compute_action(self, command: JointCommand) -> float:
        error = command.target - command.state.current
        return self.kp * error


class PDController(Controller):
    """A simple proportional-derivative controller."""

    def __init__(self, config: Config):
        super().__init__(config)
        self.kp = self.gains.get("Kp", 1.0)
        self.kd = self.gains.get("Kd", 0.1)

    def compute_action(self, command: JointCommand) -> float:
        error = command.target - command.state.current
        return self.kp * error - self.kd * command.state.velocity


class PIController(Controller):
    """A proportional-integral controller with anti-windup."""

    def __init__(self, config: Config):
        super().__init__(config)
        self.kp = self.gains.get("Kp", 1.0)
        self.ki = self.gains.get("Ki", 0.01)
        self.integral_limit = self.gains.get("integral_limit", 10.0)
        self.integral = 0.0

    def compute_action(self, command: JointCommand) -> float:
        error = command.target - command.state.current
        self.integral += error * command.state.dt
        self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)
        return self.kp * error + self.ki * self.integral

    def reset(self) -> None:
        self.integral = 0.0


class PIDController(Controller):
    """A proportional-integral-derivative controller with anti-windup."""

    def __init__(self, config: Config):
        super().__init__(config)
        self.kp = self.gains.get("Kp", 1.0)
        self.ki = self.gains.get("Ki", 0.01)
        self.kd = self.gains.get("Kd", 0.1)
        self.integral_limit = self.gains.get("integral_limit", 10.0)
        self.integral = 0.0

    def compute_action(self, command: JointCommand) -> float:
        error = command.target - command.state.current
        self.integral += error * command.state.dt
        self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)
        return self.kp * error + self.ki * self.integral - self.kd * command.state.velocity

    def reset(self) -> None:
        self.integral = 0.0
