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


def base_name(name: JointName) -> str:
    """'hip_l' -> 'hip', 'neck' -> 'neck'."""
    return name.value.removesuffix("_l").removesuffix("_r")


def build_joint_controllers(
    # Per-joint physical limits (name, limits, damping, max_torque, friction).
    # Comes from humanoid.yaml -> config.humanoid.joint_configs, expanded so
    # hip becomes hip_l and hip_r (see make_body_controller).
    joint_configs: dict[JointName, Config],
    # Which gain set each joint uses, e.g. {HIP_L: "PD_LEG"}.
    # Comes from the JointController: section of controller.yaml.
    controller_types: dict[JointName, str],
    # The named gain sets, e.g. {"PD_LEG": ControllerConfig(type="PD", gains={...})}.
    # Comes from the Controller: section of controller.yaml.
    controller_configs: dict[str, Config],
) -> JointControllers:
    """Build one JointController per joint, each with its own controller instance."""
    # Fail early if either YAML section is missing a joint or has an unknown one
    _check_joints(set(joint_configs), "joint_configs")
    _check_joints(set(controller_types), "controller_types")
    return {
        # Name refer to the joint, e.g. JointName.HIP_L.
        name: JointController(
            # Limits, max_torque etc. for this joint, from humanoid.yaml
            joint_configs[name],
            # Two lookups into controller.yaml:
            #   controller_types[name]    -> gain set name for this joint (JointController:)
            #   controller_configs[...]   -> that gain set's type + gains (Controller:)
            # make_controller then reads .type to pick PD/PID/etc. and .gains for Kp/Ki/Kd.
            # Called once per joint, so each joint gets its own instance
            # (matters for the PID integral, which is per-joint state).
            make_controller(controller_configs[controller_types[name]]),
        )
        for name in JointName
    }


def make_body_controller(
    # The top-level Config, stored on the BodyController
    config: Config,
    # humanoid.yaml joints keyed by base name: {"hip": ..., "knee": ..., "neck": ...}
    # Built in app.py from config.humanoid.joint_configs
    joint_cfg_by_base: dict[str, Config],
    # controller.yaml JointController: section, keyed by plain strings ("hip_l")
    # From config.joint_controller_types
    controller_types: dict[str, str],
    # controller.yaml Controller: section, the named gain sets
    # From config.controllers
    controller_cfgs: dict[str, Config],
) -> BodyController:
    """Expand per-joint configs (hip) into per-side ones (hip_l, hip_r) and build."""
    # humanoid.yaml has one entry per joint type (hip), but JointName has a left and
    # right of each. base_name(HIP_L) -> "hip", so both sides share the same limits.
    joint_configs = {name: joint_cfg_by_base[base_name(name)] for name in JointName}
    # YAML keys are plain strings. JointName("hip_l") converts them to the enum and
    # raises ValueError if the YAML names a joint that doesn't exist.
    types = {JointName(k): v for k, v in controller_types.items()}
    return BodyController(config, build_joint_controllers(joint_configs, types, controller_cfgs))
