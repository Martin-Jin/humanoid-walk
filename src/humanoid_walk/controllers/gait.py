"""Manual gait controller for humanoid walking. Produces joint targets for the
legs based on a gait clock and torso state, which is passed to a body controller to
produce the torques for each joint. The gait controller can be used as a policy in the viewer."""

import math
from dataclasses import dataclass, replace

from humanoid_walk.controllers.joint_state import JointName, JointStates, JointTargets
from humanoid_walk.config import Config


@dataclass(frozen=True)
class Arm:
    """One shoulder joint and its phase offset (fraction of the cycle)."""

    shoulder: JointName
    phase_offset: float


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
        # For arm swing
        self.arm_enabled = config.gait.arm["enabled"] > 0 and not self.stand_only
        self.arm_amplitude = config.gait.arm["amplitude"]  # rad, peak shoulder angle
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

        # Set up the legs and their lengths. The leg length is used for inverse kinematics.
        self.thigh = config.humanoid.leg_length_m / 2

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
        # Note arm should be opposite phase to the leg on the same side.
        self.arms = [
            Arm(JointName.SHOULDER_L, self.gait.phase_offsets["right"]),
            Arm(JointName.SHOULDER_R, self.gait.phase_offsets["left"]),
        ]

    def act(self, joint_states: JointStates) -> JointTargets:
        """Get Joint targets for the current time."""
        targets: JointTargets = {
            JointName.NECK: 0.0,
        }
        for leg in self.legs:
            targets.update(self._leg_targets(leg))
        for arm in self.arms:
            targets.update(self._arm_target(arm))
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
        """Calculate the hip, knee and ankle targets for one leg."""
        g = self.gait
        backward = self.direction < 0
        step_height = g.backward.step_height_m if backward else g.step_height_m
        offset = g.backward.touchdown_offset_m if backward else g.touchdown_offset_m

        phase = (self.time_s / g.period_s + leg.phase_offset) % 1.0
        in_stance = phase < g.duty_factor or self.stand_only
        if in_stance:
            s = phase / g.duty_factor
        else:
            s = (phase - g.duty_factor) / (1.0 - g.duty_factor)

        x, y = self._foot_target(in_stance, s, step_height, offset)
        hip_ik, knee = self._leg_ik(x, y)
        hip = hip_ik

        balance = 0.0
        if in_stance:
            if self.hip_balance:
                hip += self.balance_kp * self.torso_pitch + self.balance_kd * self.torso_rate
            if self.ankle_balance:
                balance = (
                    self.ankle_balance_kp * self.com_dx[leg.ankle]
                    + self.ankle_balance_kd * self.com_vx
                )

        # Decoupled hip (hip_ik) so the ankle isn't affected by hip balance.
        ankle = self._ankle_target(hip_ik, knee, in_stance, s, balance)
        return {leg.hip: hip, leg.knee: knee, leg.ankle: ankle}

    def _foot_target(
        self, in_stance: bool, s: float, step_height: float, touchdown_offset: float
    ) -> tuple[float, float]:
        """Foot (x, y) relative to the hip, in m. x: + forward, y: + down."""
        g = self.gait
        half = g.step_length_m / 2.0
        if in_stance:  # foot moves backward relative to the hip
            x = half - g.step_length_m * s
            y = g.hip_height_m
        else:  # foot moves forward and lifts
            x = -half * math.cos(math.pi * s)
            y = g.hip_height_m - step_height * math.sin(math.pi * s)
        return self.direction * x + touchdown_offset, y

    def _leg_ik(self, x: float, y: float) -> tuple[float, float]:
        """Return (hip, knee) in rad for a foot target (x forward, y down) from the hip."""
        eps = 1e-6
        d = min(max(math.hypot(x, y), eps), 2 * self.thigh - eps)  # clamp to reachable range
        alpha = math.acos(d / (2 * self.thigh))  # equal link lengths
        hip = -(math.atan2(x, y) + alpha)  # + hip = leg swings backward
        knee = 2 * alpha  # + knee = flexion, 0 = straight
        return hip, knee

    def _ankle_target(
        self, hip: float, knee: float, in_stance: bool, s: float, balance: float = 0.0
    ) -> float:
        """Ankle joint target in rad (+ toes down, - toes up)."""
        a = self.gait.ankle
        foot = a.stance_angle if in_stance else a.swing_angle * math.sin(math.pi * s)
        lo, hi = self.ankle_limits
        return min(max(foot - hip - knee + balance, lo), hi)

    def _arm_target(self, arm: Arm) -> JointTargets:
        """Shoulder target in rad. Swings opposite to the same-side leg. + = arm back."""
        if not self.arm_enabled:
            return {arm.shoulder: 0.0}
        phase = (self.time_s / self.gait.period_s + arm.phase_offset) % 1.0
        swing = -self.direction * self.arm_amplitude * math.cos(2 * math.pi * phase)
        return {arm.shoulder: swing}
