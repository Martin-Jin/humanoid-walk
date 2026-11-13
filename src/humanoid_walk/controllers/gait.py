"""Manual gait controller for humanoid walking."""

import math
from dataclasses import dataclass, replace

from humanoid_walk.controllers.joint_state import JointName, JointStates, JointTargets
from humanoid_walk.config import Config


@dataclass(frozen=True)
class Leg:
    """The three joints of one leg and its phase offset (fraction of the cycle)."""

    hip: JointName
    knee: JointName
    ankle: JointName
    phase_offset: float


class GaitController:
    """Turns a gait clock into hip, knee and ankle targets for both legs."""

    def __init__(self, config: Config, direction: int = 1):
        self.config = config
        self.stand_only = config.gait.stand_only
        self.gait = config.gait
        if self.stand_only:
            self.gait = replace(
                self.gait,
                step_length_m=0.0,
                step_height_m=0.0,
                ankle=replace(self.gait.ankle, swing_angle=0.0),
            )
        self.direction = direction
        self.time_s = 0.0
        # For torso balance control, torso pitch and rate are used to adjust the hip target
        self.torso_pitch = 0.0
        self.torso_rate = 0.0
        self.com_dx = {JointName.ANKLE_L: 0.0, JointName.ANKLE_R: 0.0}  # COM x minus ankle x (m)
        self.com_vx = 0.0  # COM x velocity (m/s)
        self.balance_kp = config.gait.hip_balance["kp"]  # rad of hip target per rad of torso lean
        self.balance_kd = config.gait.hip_balance[
            "kd"
        ]  # rad of hip target per rad/s of torso rotation
        self.hip_balance = (
            config.gait.hip_balance["enabled"] > 0
        )  # If true, hip targets are adjusted for torso balance.
        # For ankle control
        self.ankle_balance_kp = config.gait.ankle_balance[
            "kp"
        ]  # rad of ankle target per rad of torso lean
        self.ankle_balance_kd = config.gait.ankle_balance[
            "kd"
        ]  # rad of ankle target per rad/s of torso rotation
        self.ankle_balance = (
            config.gait.ankle_balance["enabled"] > 0
        )  # If true, ankle targets are adjusted for torso balance.

        thigh = config.humanoid.leg_length_m / 2
        self.l1 = thigh
        self.l2 = thigh

        ankle_cfg = {j.name: j for j in config.humanoid.joint_configs}["ankle"]
        self.ankle_limits = ankle_cfg.limits

        self.legs = [
            Leg(
                JointName.HIP_L,
                JointName.KNEE_L,
                JointName.ANKLE_L,
                self.gait.phase_offsets["left"],
            ),
            Leg(
                JointName.HIP_R,
                JointName.KNEE_R,
                JointName.ANKLE_R,
                self.gait.phase_offsets["right"],
            ),
        ]

    def act(self, joint_states: JointStates) -> JointTargets:
        """Joint targets for the current time."""
        targets: JointTargets = {
            JointName.NECK: 0.0,
            JointName.SHOULDER_L: 0.0,
            JointName.SHOULDER_R: 0.0,
        }
        for leg in self.legs:
            targets.update(self._leg_targets(leg))
        return targets

    def reset(self) -> None:
        """Restart the gait clock."""
        self.time_s = 0.0

    def step_clock(self, dt: float) -> None:
        """Advance the gait clock by dt seconds."""
        self.time_s += dt

    def set_torso(self, pitch: float, rate: float) -> None:
        """Latest torso angle (rad) and angular velocity (rad/s) from the sim."""
        self.torso_pitch = pitch
        self.torso_rate = rate
        # print(f"pitch={self.torso_pitch:.3f} rate={self.torso_rate:.3f}")

    def set_com(self, dx: dict[JointName, float], vx: float) -> None:
        """Latest COM position relative to each ankle (m, + forward) and COM x velocity (m/s)."""
        self.com_dx = dx
        self.com_vx = vx

    def _leg_targets(self, leg: Leg) -> JointTargets:
        """Hip, knee and ankle targets for one leg at the current time."""
        g = self.gait
        backward = self.direction < 0
        step_height = g.backward.step_height_m if backward else g.step_height_m
        offset = g.backward.touchdown_offset_m if backward else g.touchdown_offset_m

        # Compute normalized phase of the leg in the gait cycle, 0.0 -> 1.0, with phase offset applied.
        phase = (self.time_s / g.period_s + leg.phase_offset) % 1.0

        # 1. Compute foot target (x, y) relative to the hip.
        x, y = foot_target(
            phase,
            g.step_length_m,
            step_height,
            g.hip_height_m,
            g.duty_factor,
            offset,
            self.direction,
        )

        # 2. Hip and knee from IK.
        hip_ik, knee = leg_ik(x, y, self.l1, self.l2)
        hip = hip_ik

        # 3. Stance or swing, and progress through it.
        in_stance = phase < g.duty_factor or self.stand_only
        if in_stance:
            s = phase / g.duty_factor
        else:
            s = (phase - g.duty_factor) / (1.0 - g.duty_factor)

        # 4. Hip balance, stance only.
        if self.hip_balance and in_stance:
            hip += self.balance_kp * self.torso_pitch + self.balance_kd * self.torso_rate

        # 5. Ankle balance, stance only.
        balance = 0.0
        if in_stance and self.ankle_balance:
            balance = (
                self.ankle_balance_kp * self.com_dx[leg.ankle] + self.ankle_balance_kd * self.com_vx
            )

        ankle = ankle_target(
            hip_ik,  # Use decoupled hip for ankle target, so that ankle is not affected by hip balance.
            knee,
            in_stance,
            s,
            g.ankle.stance_angle,
            g.ankle.swing_angle,
            limits=self.ankle_limits,
            balance=balance,
        )
        return {leg.hip: hip, leg.knee: knee, leg.ankle: ankle}


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
    balance: float = 0.0,
) -> float:
    """Ankle joint target in rad (+ toes down, - toes up)."""
    foot = stance_angle if in_stance else swing_angle * math.sin(math.pi * s)
    ankle = foot - torso_pitch - hip - knee + balance
    return min(max(ankle, limits[0]), limits[1])
