"""Defines the joint control logic for the humanoid robot,
and provides a controller for the entire humanoid body."""

from humanoid_walk.config import Config
from humanoid_walk.controllers.controller import (
    Controller,
    PController,
    make_controller,
)
from humanoid_walk.controllers.joint_state import (
    BodyState,
    JointName,
    JointState,
    JointStates,
    JointTargets,
)


class JointController:
    """A controller for a single joint of the humanoid robot."""

    def __init__(self, config: Config, controller: Controller | None = None):
        self.config = config
        self.name = config.name
        self.limits = config.limits
        self.damping = config.damping
        self.max_torque = config.max_torque
        self.friction = config.friction
        self.controller = controller or PController()

    def compute_torque(self, target: float, state: JointState) -> float:
        """Compute the torque to apply to the joint."""
        target = self.clamp_angle(target)
        return self.clamp_torque(self.controller.compute_action(target, state))

    def clamp_angle(self, angle: float) -> float:
        """Clamp the angle to the joint limits."""
        return max(min(angle, self.limits[1]), self.limits[0])

    def clamp_torque(self, torque: float) -> float:
        """Clamp the torque to the maximum allowed torque."""
        return max(min(torque, self.max_torque), -self.max_torque)


JointControllers = dict[JointName, JointController]


def _check_joints(names: set[JointName] | set, what: str) -> None:
    """Raise if the given names don't exactly match JointName."""
    expected = set(JointName)
    if set(names) != expected:
        missing = expected - set(names)
        extra = set(names) - expected
        raise ValueError(f"{what}: missing={sorted(missing)}, unknown={sorted(extra)}")


def build_joint_controllers(
    joint_configs: dict[JointName, Config],
    controller_configs: dict[str, Config],
) -> JointControllers:
    """Build one JointController per joint, each with its own controller instance."""
    _check_joints(set(joint_configs), "joint_configs")
    return {
        name: JointController(cfg, make_controller(controller_configs[cfg.type]))
        for name, cfg in joint_configs.items()
    }


class BodyController:
    """A controller for the entire humanoid robot, managing multiple joints."""

    def __init__(
        self,
        config: Config,
        joint_controllers: JointControllers,
        body_state: BodyState | None = None,
    ):
        _check_joints(set(joint_controllers), "joint_controllers")
        self.config = config
        self.joint_controllers = joint_controllers
        self.body_state = body_state

    def compute_torques(self, targets: JointTargets, states: JointStates) -> dict[JointName, float]:
        """Compute the torques for all joints from their targets and states."""
        _check_joints(set(targets), "targets")
        _check_joints(set(states), "states")
        return {
            name: self.joint_controllers[name].compute_torque(targets[name], states[name])
            for name in JointName
        }
