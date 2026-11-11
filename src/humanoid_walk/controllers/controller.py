"""Implements of several controllers for the humanoid walk environment."""

from abc import ABC, abstractmethod

from humanoid_walk.config import Config
from humanoid_walk.controllers.joint_state import JointState


class Controller(ABC):
    """A controller for the humanoid robot."""

    def __init__(self, config: Config | None = None):
        self.config = config
        self.gains = config.gains if config else {}
        self.type = config.type if config else "P"

    @abstractmethod
    def compute_action(self, target: float, state: JointState) -> float:
        """Compute the action to take for a target and the current joint state."""

    def reset(self) -> None:
        """Reset any internal state."""


class PController(Controller):
    """A simple proportional controller."""

    def __init__(self, config: Config | None = None):
        super().__init__(config)
        self.kp = self.gains.get("Kp", 1.0)

    def compute_action(self, target: float, state: JointState) -> float:
        return self.kp * (target - state.current)


class PDController(Controller):
    """A simple proportional-derivative controller."""

    def __init__(self, config: Config | None = None):
        super().__init__(config)
        self.kp = self.gains.get("Kp", 1.0)
        self.kd = self.gains.get("Kd", 0.1)

    def compute_action(self, target: float, state: JointState) -> float:
        return self.kp * (target - state.current) - self.kd * state.velocity


class PIController(Controller):
    """A proportional-integral controller with anti-windup."""

    def __init__(self, config: Config | None = None):
        super().__init__(config)
        self.kp = self.gains.get("Kp", 1.0)
        self.ki = self.gains.get("Ki", 0.01)
        self.integral_limit = self.gains.get("integral_limit", 10.0)
        self.integral = 0.0

    def compute_action(self, target: float, state: JointState) -> float:
        error = target - state.current
        self.integral += error * state.dt
        self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)
        return self.kp * error + self.ki * self.integral

    def reset(self) -> None:
        self.integral = 0.0


class PIDController(Controller):
    """A proportional-integral-derivative controller with anti-windup."""

    def __init__(self, config: Config | None = None):
        super().__init__(config)
        self.kp = self.gains.get("Kp", 1.0)
        self.ki = self.gains.get("Ki", 0.01)
        self.kd = self.gains.get("Kd", 0.1)
        self.integral_limit = self.gains.get("integral_limit", 10.0)
        self.integral = 0.0

    def compute_action(self, target: float, state: JointState) -> float:
        error = target - state.current
        self.integral += error * state.dt
        self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)
        return self.kp * error + self.ki * self.integral - self.kd * state.velocity

    def reset(self) -> None:
        self.integral = 0.0


CONTROLLERS: dict[str, type[Controller]] = {
    "P": PController,
    "PD": PDController,
    "PI": PIController,
    "PID": PIDController,
}


def make_controller(config: Config) -> Controller:
    """Create a controller from its config, based on config.type."""
    controller_cls = CONTROLLERS.get(config.type)
    if controller_cls is None:
        raise ValueError(f"Unknown controller type '{config.type}'. Options: {list(CONTROLLERS)}")
    return controller_cls(config)
