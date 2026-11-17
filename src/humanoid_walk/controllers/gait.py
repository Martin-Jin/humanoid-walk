"""Manual gait controller for humanoid walking. Produces joint targets for the
legs based on a gait clock and torso state, which is passed to a body controller to
produce the torques for each joint. The gait controller can be used as a policy in the viewer.

Per-frame inputs besides the clock: set_torso(), set_com() and (optional, closed-loop
touchdown) set_contacts(). Without set_contacts() the touchdown is open loop."""

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


def _clamp(v: float, lo: float, hi: float) -> float:
    return min(max(v, lo), hi)


def _ramp(t: float, start: float, duration: float) -> float:
    """Smooth 0 to 1 ramp beginning at `start` and lasting `duration`."""
    x = _clamp((t - start) / max(duration, 1e-6), 0.0, 1.0)
    return x * x * (3.0 - 2.0 * x)


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
        g = self.gait
        h = config.humanoid
        self.direction = direction

        # Startup: stand straight, crouch to walking height, then ramp up the walk
        self.stand_s = g.start_up["stand_s"]
        self.crouch_s = g.start_up["crouch_s"]
        self.ramp_s = g.start_up["ramp_s"]

        # Leg geometry (the sim builds equal thigh and shin lengths)
        self.leg_length = h.leg_length_m
        self.thigh = self.leg_length / 2
        self.shin = self.leg_length - self.thigh
        self.max_leg_ratio = g.max_leg_ratio

        # Arm swing parameters
        self.arm_enabled = g.arm["enabled"] > 0 and not self.stand_only
        self.arm_amplitude = g.arm["amplitude"]  # rad, peak shoulder angle

        # Balance gate
        self.gate_enabled = g.balance_gate["enabled"] > 0
        self.gate_blend_s = g.balance_gate["blend_s"]
        self.gate_stop = g.balance_gate["stop_margin"]
        self.gate_resume = g.balance_gate["resume_margin"]
        # Foot extent either side of the ankle
        self.heel = h.foot_length_m / 2 - h.ankle_offset_m
        self.toe = h.foot_length_m / 2 + h.ankle_offset_m
        self.gravity = config.world.gravity_mps2

        # Push-off: late stance, the leg lengthens and the ankle plantarflexes
        self.push_start = g.push_off["start"]
        self.push_extend = g.push_off["extend_m"]
        self.push_ankle = g.push_off["ankle_angle"]
        self.push_back = g.push_off["back_m"]
        self.push_unwind = g.push_off["unwind"]  # fraction of swing over which push-off unwinds

        # Balance gains
        self.balance_kp = g.hip_balance["kp"]  # rad of hip target per rad of torso lean
        self.balance_kd = g.hip_balance["kd"]  # rad of hip target per rad/s of torso rotation
        self.hip_balance = g.hip_balance["enabled"] > 0
        self.ankle_balance_kp = g.ankle_balance["kp"]  # rad of ankle target per m of COM offset
        self.ankle_balance_kd = g.ankle_balance["kd"]  # rad of ankle target per m/s of COM speed
        self.ankle_balance = g.ankle_balance["enabled"] > 0
        # Static shift of the planned COM from over the hip, m (+ = ahead of the hip)
        self.com_offset = g.ankle_balance["com_offset_m"]
        self.balance_blend = g.balance_blend

        self.max_hip_balance = g.hip_balance["max_correction"]  # rad, cap on the correction
        self.max_ankle_balance = g.ankle_balance["max_correction"]  # rad, cap on the correction

        # Foot placement: land further ahead when the COM is faster than planned
        self.placement_gain = g.foot_placement["gain"]
        self.placement_max = g.foot_placement["max_m"]
        self.placement_deadband = g.foot_placement["deadband_mps"]

        # Touchdown tuning
        td = g.touchdown
        self.landing_start = td["landing_start"]
        self.reach_start = td["reach_start"]
        self.reach_max = td["reach_max_m"]
        self.reach_rate = td["reach_rate_mps"]
        self.reach_decay_s = td["reach_decay_s"]
        self.hold_from = td["hold_from"]
        self.hold_max_s = td["hold_max_s"]

        ankle_cfg = {j.name: j for j in h.joint_configs}["ankle"]
        self.ankle_limits = ankle_cfg.limits

        self.legs = [
            Leg(JointName.HIP_L, JointName.KNEE_L, JointName.ANKLE_L, g.phase_offsets["left"]),
            Leg(JointName.HIP_R, JointName.KNEE_R, JointName.ANKLE_R, g.phase_offsets["right"]),
        ]
        # Arm is in opposite phase to the leg on the same side.
        self.arms = [
            Arm(JointName.SHOULDER_L, g.phase_offsets["right"]),
            Arm(JointName.SHOULDER_R, g.phase_offsets["left"]),
        ]

        self._reset_state()

    # ------------------------------------------------------------------ state

    def _reset_state(self) -> None:
        """All runtime state, shared by __init__ and reset()."""
        self.elapsed_s = 0.0  # time since start, always runs (the gait clock starts later)
        self.time_s = self.gait.start_up["start_phase"] * self.gait.period_s  # gait clock
        self.crouch = 0.0  # 0 = straight legs, 1 = walking hip height
        self.walk = 0.0  # 0 = no stepping, 1 = full gait
        # Torso and COM state for balance control
        self.torso_pitch = 0.0
        self.torso_rate = 0.0
        self.com_dx = {JointName.ANKLE_L: 0.0, JointName.ANKLE_R: 0.0}  # COM x - ankle x (m)
        self.com_vx = 0.0  # COM x velocity (m/s)
        self.com_height = self.gait.hip_height_m  # placeholder until set_com() reports
        # Balance gate
        self.gate_scale = 1.0  # 1 = full gait, 0 = feet pulled back under the hips
        self.paused = False
        # Touchdown
        self.contacts: dict[JointName, bool] | None = None  # None = open loop
        self.reach = {leg.ankle: 0.0 for leg in self.legs}
        self.wait_s = 0.0
        self.joint_states: JointStates = {}  # measured joints from the latest act()
        # Foot placement: landing shift (m, in the gait frame) of the swing in progress, and the
        # one the current stance started with (so the stance and the next lift-off stay continuous)
        self.swing_shift = {leg.ankle: 0.0 for leg in self.legs}
        self.stance_shift = {leg.ankle: 0.0 for leg in self.legs}

    def reset(self) -> None:
        """Restart the startup sequence, the gait clock and all measured state."""
        self._reset_state()

    # ---------------------------------------------------------------- inputs

    def set_torso(self, pitch: float, rate: float) -> None:
        """Latest torso angle (rad) and angular velocity (rad/s) from the sim."""
        self.torso_pitch = pitch
        self.torso_rate = rate

    def set_com(self, dx: dict[JointName, float], vx: float, height: float) -> None:
        """Latest COM x relative to each ankle (m, + forward), COM x velocity (m/s)
        and COM height above the ankle (m)."""
        self.com_dx = dx
        self.com_vx = vx
        self.com_height = height

    def set_contacts(self, left: bool, right: bool) -> None:
        """Latest foot contact flags, e.g. set_contacts(*sim.feet_on_ground())."""
        self.contacts = {JointName.ANKLE_L: left, JointName.ANKLE_R: right}

    # ----------------------------------------------------------------- clock

    def _in_startup(self) -> bool:
        return self.elapsed_s <= self.stand_s + self.crouch_s

    def _standing(self) -> bool:
        return self.stand_only or self._in_startup()

    def _startup(self) -> tuple[float, float]:
        """Blend factors (crouch, walk), each 0 to 1, from the time since start."""
        t = self.elapsed_s
        crouch = _ramp(t, self.stand_s, self.crouch_s)
        if self.stand_only:
            return crouch, 0.0
        walk = _ramp(t, self.stand_s + self.crouch_s, self.ramp_s) * self.gate_scale
        return crouch, walk

    def step_clock(self, dt: float) -> None:
        """Advance the startup timer, and the gait clock once walking has begun."""
        self.elapsed_s += dt
        if self._in_startup():
            for k in self.reach:
                self.reach[k] = 0.0
            return  # still standing and crouching, gait clock stays at 0
        self._update_reach(dt)
        if self.gate_enabled:
            self._update_gate()
            step = dt / max(self.gate_blend_s, 1e-6)
            goal = 0.0 if self.paused else 1.0
            self.gate_scale += _clamp(goal - self.gate_scale, -step, step)
            if self.paused and self._both_in_stance():
                return  # hold the phase, balance terms keep running
        if self._awaiting_touchdown():
            self.wait_s += dt
            if self.wait_s < self.hold_max_s:
                return  # swing finished but no contact yet: wait for the floor
        else:
            self.wait_s = 0.0
        self.time_s += dt

    def _phase(self, leg: Leg) -> float:
        return (self.time_s / self.gait.period_s + leg.phase_offset) % 1.0

    def _leg_state(self, leg: Leg) -> tuple[bool, float, bool]:
        """(in_stance, s, standing) where s is 0 to 1 progress through stance or swing."""
        if self._standing():
            return True, 0.0, True
        g = self.gait
        phase = self._phase(leg)
        if phase < g.duty_factor:
            return True, _clamp(phase / max(g.duty_factor, 1e-6), 0.0, 1.0), False
        s = (phase - g.duty_factor) / max(1.0 - g.duty_factor, 1e-6)
        return False, _clamp(s, 0.0, 1.0), False

    def _stance_legs(self) -> list[Leg]:
        """Legs currently in stance (by the clock)."""
        return [leg for leg in self.legs if self._phase(leg) < self.gait.duty_factor]

    def _both_in_stance(self) -> bool:
        return len(self._stance_legs()) == len(self.legs)

    # ------------------------------------------------------- closed-loop bits

    def _update_reach(self, dt: float) -> None:
        """Late in swing, keep lowering the foot until contact. Unwind after touchdown."""
        for leg in self.legs:
            in_stance, s, _ = self._leg_state(leg)
            r = self.reach[leg.ankle]
            if self.contacts is None:
                r = 0.0
            elif not in_stance and s >= self.reach_start and not self.contacts[leg.ankle]:
                r = min(r + self.reach_rate * dt, self.reach_max)
            elif in_stance or s < self.reach_start:
                r = max(r - self.reach_max / max(self.reach_decay_s, 1e-6) * dt, 0.0)
            # else: late swing and in contact, hold the reach
            self.reach[leg.ankle] = r

    def _awaiting_touchdown(self) -> bool:
        """True if a leg has finished its swing but its foot is not yet on the ground."""
        if self.contacts is None:
            return False
        for leg in self.legs:
            in_stance, s, _ = self._leg_state(leg)
            if not in_stance and s >= self.hold_from and not self.contacts[leg.ankle]:
                return True
        return False

    def _update_gate(self) -> None:
        """Pause the gait clock when the capture point nears the edge of the support area."""
        stance = self._stance_legs()
        if not stance:
            return
        omega = math.sqrt(self.gravity / max(self.com_height, 0.1))
        cp = self.com_vx / omega  # capture point relative to the COM (m)
        ankles = [-self.com_dx[leg.ankle] for leg in stance]  # stance ankle x relative to COM
        rear = min(ankles) - self.heel
        front = max(ankles) + self.toe
        margin = min(cp - rear, front - cp)  # distance to the nearest edge, negative if outside
        limit = self.gate_resume if self.paused else self.gate_stop
        self.paused = margin < limit

    # ---------------------------------------------------------------- targets

    def act(self, joint_states: JointStates) -> JointTargets:
        """Get joint targets for the current time. (Joint states are unused: the
        controller works from the clock and the set_* inputs.)"""
        self.joint_states = joint_states
        self.crouch, self.walk = self._startup()
        targets: JointTargets = {JointName.NECK: 0.0}
        for leg in self.legs:
            targets.update(self._leg_targets(leg))
        for arm in self.arms:
            targets.update(self._arm_target(arm))
        return targets

    def _hip_height(self) -> float:
        g = self.gait
        return self.leg_length - self.crouch * (self.leg_length - g.hip_height_m)

    def _y_limit(self, hip_h: float) -> float:
        """Longest commanded leg: max_leg_ratio of full length, unless hip_h is already above."""
        return max(self.max_leg_ratio * self.leg_length, hip_h)

    def _push_off(self, in_stance: bool, s: float) -> float:
        """0 to 1 push-off blend: ramps up over late stance, unwinds over early swing."""
        if in_stance:
            return _ramp(s, self.push_start, 1.0 - self.push_start) * self.walk
        return (1.0 - _ramp(s, 0.0, self.push_unwind)) * self.walk

    def _extension(self, in_stance: bool, s: float, hip_h: float) -> float:
        """How far (m) a leg in this state is commanded longer than nominal."""
        y = hip_h + self.push_extend * self._push_off(in_stance, s)
        return min(y, self._y_limit(hip_h)) - hip_h

    def _balance_weight(self, in_stance: bool, s: float, standing: bool) -> float:
        """Fades balance terms in after touchdown and out before lift-off."""
        if standing:
            return 1.0
        if not in_stance:
            return 0.0
        b = max(self.balance_blend, 1e-6)
        return _ramp(s, 0.0, b) * (1.0 - _ramp(s, 1.0 - b, b))

    def _support_coupling(self, leg: Leg, in_stance: bool, s: float, hip_h: float) -> float:
        """Extra drop (m) so this foot stays on the floor while the other leg is extended.

        The other leg's push-off raises the pelvis, so this foot has to reach lower by the
        same amount. Applied fully in stance, ramped in over late swing, and it unwinds
        with the other leg's extension, so it is continuous across touchdown."""
        other = next(l for l in self.legs if l is not leg)
        o_stance, o_s, o_standing = self._leg_state(other)
        if o_standing:
            return 0.0
        ext_other = self._extension(o_stance, o_s, hip_h)
        ext_self = self._extension(in_stance, s, hip_h)
        drop = max(0.0, ext_other - ext_self)
        if in_stance:
            return drop
        return drop * _ramp(s, self.landing_start, 1.0 - self.landing_start)

    def _leg_targets(self, leg: Leg) -> JointTargets:
        """Calculate the hip, knee and ankle targets for one leg."""
        g = self.gait
        backward = self.direction < 0
        step_height = (g.backward.step_height_m if backward else g.step_height_m) * self.walk
        offset = g.backward.touchdown_offset_m if backward else g.touchdown_offset_m

        in_stance, s, standing = self._leg_state(leg)
        x, y = self._foot_target(leg, in_stance, s, step_height, offset)

        # Nominal IK, used for the ankle so it is unaffected by hip balance.
        hip0, knee0 = self._leg_ik(x, y)

        w = self._balance_weight(in_stance, s, standing)
        delta = 0.0
        if self.hip_balance:
            delta = w * (self.balance_kp * self.torso_pitch + self.balance_kd * self.torso_rate)
            delta = _clamp(delta, -self.max_hip_balance, self.max_hip_balance)
        # Hip balance applied as a foot-target shift at constant height, before IK.
        hip, knee = hip0 + delta, knee0

        balance = 0.0
        if self.ankle_balance:
            # Planned COM is over the hip, so relative to this ankle it should sit at -x.
            dx_ref = -x + self.com_offset
            vx_ref = self.direction * g.step_length_m * self.walk / (g.duty_factor * g.period_s)
            balance = w * (
                self.ankle_balance_kp * (self.com_dx[leg.ankle] - dx_ref)
                + self.ankle_balance_kd * (self.com_vx - vx_ref)
            )
            balance = _clamp(balance, -self.max_ankle_balance, self.max_ankle_balance)
        measured = self._measured_chain(leg, hip0, knee0)
        ankle = self._ankle_target(hip0, knee0, in_stance, s, balance, measured)
        return {leg.hip: hip, leg.knee: knee, leg.ankle: ankle}

    def _foot_target(
        self, leg: Leg, in_stance: bool, s: float, step_height: float, touchdown_offset: float
    ) -> tuple[float, float]:
        """Foot (x, y) relative to the hip, in m. x: + forward, y: + down."""
        g = self.gait
        length = g.step_length_m * self.walk
        hip_h = self._hip_height()
        half = length / 2.0
        push = self._push_off(in_stance, s)
        couple = self._support_coupling(leg, in_stance, s, hip_h)
        if in_stance:
            self.stance_shift[leg.ankle] = self.swing_shift[leg.ankle]  # latched at touchdown
            x = half + self.stance_shift[leg.ankle] - length * s
            y = hip_h + self.push_extend * push
        else:
            lift_x = -half + self.stance_shift[leg.ankle]
            land_x = half + self._landing_shift(leg)
            x = lift_x + (land_x - lift_x) * (1.0 - math.cos(math.pi * s)) / 2.0
            y = hip_h - step_height * math.sin(math.pi * s) + self.push_extend * push
        y += couple + self.reach[leg.ankle]
        y = min(y, self._y_limit(hip_h))  # keep away from the straight-leg singularity
        x -= self.push_back * push  # lead the foot target backward in late stance
        return self.direction * x + touchdown_offset, y

    def _landing_shift(self, leg: Leg) -> float:
        """How far (m, gait frame) the swing foot should land ahead of the planned spot.

        When the COM is faster than planned the capture point (where the COM would come to
        rest on a point foot) is ahead of the plan, so the foot lands further ahead."""
        if self.placement_gain <= 0.0:
            return 0.0
        g = self.gait
        omega = math.sqrt(self.gravity / max(self.com_height, 0.1))
        vx_ref = self.direction * g.step_length_m * self.walk / (g.duty_factor * g.period_s)
        error = self.com_vx - vx_ref
        error = math.copysign(max(abs(error) - self.placement_deadband, 0.0), error)
        shift = self.placement_gain * error / omega * self.walk
        shift = _clamp(self.direction * shift, -self.placement_max, self.placement_max)
        self.swing_shift[leg.ankle] = shift
        return shift

    def _leg_ik(self, x: float, y: float) -> tuple[float, float]:
        """Return (hip, knee) in rad for a foot target (x forward, y down) from the hip."""
        t, sh = self.thigh, self.shin
        eps = 1e-6
        d = _clamp(math.hypot(x, y), abs(t - sh) + eps, t + sh - eps)  # reachable range
        beta = math.acos(_clamp((t * t + d * d - sh * sh) / (2 * t * d), -1.0, 1.0))
        gamma = math.acos(_clamp((t * t + sh * sh - d * d) / (2 * t * sh), -1.0, 1.0))
        hip = -(math.atan2(x, y) + beta)  # + hip = leg swings backward
        knee = math.pi - gamma  # + knee = flexion, 0 = straight
        return hip, knee

    def _measured_chain(self, leg: Leg, hip0: float, knee0: float) -> float:
        """Torso pitch + hip + knee from the measured joints, or from the plan if unknown."""
        state = self.joint_states
        if leg.hip not in state or leg.knee not in state:
            return self.torso_pitch + hip0 + knee0
        return self.torso_pitch + state[leg.hip].current + state[leg.knee].current

    def _ankle_target(
        self,
        hip: float,
        knee: float,
        in_stance: bool,
        s: float,
        balance: float = 0.0,
        measured_chain: float | None = None,
    ) -> float:
        """Ankle joint target in rad (+ toes down, - toes up)."""
        a = self.gait.ankle
        foot = a.stance_angle if in_stance else a.swing_angle * self.walk * math.sin(math.pi * s)
        foot += self.push_ankle * self._push_off(in_stance, s)
        lo, hi = self.ankle_limits
        # The foot angle in the world is torso pitch + hip + knee + ankle. The planned chain
        # trusts the leg to follow its targets, the measured one follows the joints as they are.
        if measured_chain is None or a.measured_ref == 0.0:
            foot -= a.pitch_ref * self.torso_pitch
            return _clamp(foot - hip - knee + balance, lo, hi)
        planned = a.pitch_ref * self.torso_pitch + hip + knee
        chain = (1.0 - a.measured_ref) * planned + a.measured_ref * measured_chain
        return _clamp(foot - chain + balance, lo, hi)

    def _arm_target(self, arm: Arm) -> JointTargets:
        """Shoulder target in rad. Swings opposite to the same-side leg. + = arm back."""
        if not self.arm_enabled:
            return {arm.shoulder: 0.0}
        phase = (self.time_s / self.gait.period_s + arm.phase_offset) % 1.0
        swing = -self.direction * self.arm_amplitude * self.walk * math.cos(2 * math.pi * phase)
        return {arm.shoulder: swing}
