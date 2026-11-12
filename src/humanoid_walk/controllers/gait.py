"""Manual gait controller for humanoid walking."""

from attr import dataclass
from humanoid_walk.controllers.joint_state import JointName, JointStates, JointTargets
import math


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


def foot_target(
    phase: float,
    step_length: float,
    step_height: float,
    hip_height: float,
    duty_factor: float,
    touchdown_offset: float = 0.0,
    direction: int = 1,
) -> tuple[float, float]:
    """Foot (x, y) relative to the hip, in m. x: + forward, y: + down."""
    p = phase % 1.0
    half = step_length / 2.0

    if p < duty_factor:  # stance: foot moves backward relative to the hip
        s = p / duty_factor  # 0 -> 1
        x = half - step_length * s
        y = hip_height
    else:  # swing: foot moves forward and lifts
        s = (p - duty_factor) / (1.0 - duty_factor)  # 0 -> 1
        x = -half + step_length * 0.5 * (1.0 - math.cos(math.pi * s))
        y = hip_height - step_height * math.sin(math.pi * s)

    return direction * x + touchdown_offset, y


def leg_ik(x: float, y: float, l1: float, l2: float) -> tuple[float, float]:
    """Return (hip, knee) in rad for a foot target (x forward, y down) from the hip."""
    eps = 1e-6
    d = math.hypot(x, y)
    d = min(max(d, abs(l1 - l2) + eps), l1 + l2 - eps)  # clamp to reachable range

    phi = math.atan2(x, y)  # direction to foot, from straight down, + forward
    alpha = math.acos((l1**2 + d**2 - l2**2) / (2 * l1 * d))
    inner = math.acos((l1**2 + l2**2 - d**2) / (2 * l1 * l2))

    hip = -(phi + alpha)  # config: + hip = leg swings backward
    knee = math.pi - inner  # config: + knee = flexion, 0 = straight
    return hip, knee


def ankle_target(
    hip: float,
    knee: float,
    in_stance: bool,
    s: float,
    stance_angle: float,
    swing_angle: float,
    torso_pitch: float = 0.0,
    limits: tuple[float, float] = (-1.0, 0.5),
) -> float:
    """Ankle joint target in rad (+ toes down, - toes up)."""
    foot = stance_angle if in_stance else swing_angle * math.sin(math.pi * s)
    ankle = foot - torso_pitch - hip - knee
    return min(max(ankle, limits[0]), limits[1])
