"""Defines the physics for the game using pymunk.
This includes the simulation of the humanoid and environment."""

import pymunk

from humanoid_walk.config import Config
from humanoid_walk.controllers.joint_state import JointName
from humanoid_walk.controllers.joint_state import JointName, JointState, JointStates


def limit_velocity(body, gravity, damping, dt):
    """Limit the velocity of a body to prevent it from going too fast."""
    max_velocity = 1000
    pymunk.Body.update_velocity(body, gravity, damping, dt)
    l = body.velocity.length
    if l > max_velocity:
        scale = max_velocity / l
        body.velocity = body.velocity * scale


class Simulation:
    """Owns the pymunk space and the humanoid bodies."""

    def __init__(self, config: Config):
        self.config = config
        self.ppm = config.rendering.pixels_per_meter
        self.space = pymunk.Space()
        self.space.gravity = (0.0, config.world.gravity_mps2 * self.ppm)
        self.humanoid: list[pymunk.Body] = []
        self.substeps = 8  # physics steps per frame; torques are recomputed each one

    def px(self, meters: float) -> float:
        """Convert meters to pixels."""
        return meters * self.ppm

    def add_circle(self, pos, radius_m, mass=1.0, friction=0.7, static=False, velocity_func=None):
        """Add a circle. pos is in pixels, radius_m in meters."""
        radius = self.px(radius_m)
        if static:
            body = pymunk.Body(body_type=pymunk.Body.STATIC)
        else:
            body = pymunk.Body(mass, pymunk.moment_for_circle(mass, 0, radius))
        body.position = pos
        if velocity_func is not None:
            body.velocity_func = velocity_func
        shape = pymunk.Circle(body, radius)
        shape.friction = friction
        self.space.add(body, shape)
        return body

    def add_segment(
        self,
        pos,
        length_m,
        width_m,
        mass=1.0,
        friction=0.7,
        angle=0.0,
        vertical=True,
        static=False,
        velocity_func=None,
    ):
        """Add a segment centred on pos. Sizes in meters. Vertical by default."""
        half = self.px(length_m) / 2
        a, b = ((0, -half), (0, half)) if vertical else ((-half, 0), (half, 0))
        radius = self.px(width_m) / 2
        if static:
            body = pymunk.Body(body_type=pymunk.Body.STATIC)
        else:
            body = pymunk.Body(mass, pymunk.moment_for_segment(mass, a, b, radius))
        body.position = pos
        if velocity_func is not None:
            body.velocity_func = velocity_func
        body.angle = angle
        shape = pymunk.Segment(body, a, b, radius)
        shape.friction = friction
        self.space.add(body, shape)
        return body

    def setup(self):
        """Create the bodies and joints once."""
        h = self.config.humanoid
        r = self.config.rendering
        fric = h.joint_configs[0].friction
        m = h.part_masses_kg

        # Ground
        self.add_segment(
            (r.screen_width / 2, r.floor_y_px),
            length_m=r.screen_width / self.ppm,
            width_m=0.05,
            friction=0.7,
            vertical=False,
            static=True,
        )

        # Dimensions in pixels
        torso_half = self.px(h.torso_length_m) / 2
        head_r = self.px(h.head_radius_m)
        arm_half = self.px(h.arm_length_m) / 2
        thigh_len = shin_len = h.leg_length_m / 2
        thigh_half = self.px(thigh_len) / 2
        shin_half = self.px(shin_len) / 2
        foot_half = self.px(h.foot_length_m) / 2
        ankle_off = self.px(0.055)  # ankle sits 0.055 m behind the foot's centre (0.07 m from heel)

        # Spawn position: upright, feet resting on the ground surface
        tx = r.screen_width / 2
        ground_top = r.floor_y_px - self.px(0.025)  # ground segment is 0.05 m thick
        below_torso_centre = torso_half + self.px(h.leg_length_m) + self.px(h.foot_width_m) / 2
        ty = ground_top - below_torso_centre - 1  # 1 px gap, so it settles without dropping

        top_y = ty - torso_half
        bottom_y = ty + torso_half

        # Head: sits on the torso's top tip
        self.head = self.add_circle(
            (tx, top_y - head_r),
            h.head_radius_m,
            mass=m["head"],
            velocity_func=limit_velocity,
            friction=fric,
        )

        # Torso
        self.torso = self.add_segment(
            (tx, ty),
            length_m=h.torso_length_m,
            width_m=h.torso_width_m,
            mass=m["torso"],
            velocity_func=limit_velocity,
            friction=fric,
        )

        # Arms: hang down from the top tip
        def make_arm():
            arm = self.add_segment(
                (tx, top_y + arm_half),
                length_m=h.arm_length_m,
                width_m=h.arm_width_m,
                mass=m["arm"],
                velocity_func=limit_velocity,
                friction=fric,
            )
            shoulder = pymunk.PivotJoint(arm, self.torso, (0, -arm_half), (0, -torso_half))
            return arm, shoulder

        self.arm_l, self.shoulder_l = make_arm()
        self.arm_r, self.shoulder_r = make_arm()

        # Legs: thigh, shin, foot
        def make_leg():
            knee_y = bottom_y + 2 * thigh_half
            ankle_y = knee_y + 2 * shin_half

            thigh = self.add_segment(
                (tx, bottom_y + thigh_half),
                length_m=thigh_len,
                width_m=h.leg_width_m,
                mass=m["thigh"],
                velocity_func=limit_velocity,
                friction=fric,
            )
            shin = self.add_segment(
                (tx, knee_y + shin_half),
                length_m=shin_len,
                width_m=h.leg_width_m,
                mass=m["shin"],
                velocity_func=limit_velocity,
                friction=fric,
            )
            foot = self.add_segment(
                (tx + ankle_off, ankle_y),
                length_m=h.foot_length_m,
                width_m=h.foot_width_m,
                mass=m["foot"],
                velocity_func=limit_velocity,
                friction=fric,
                vertical=False,
            )

            hip = pymunk.PivotJoint(thigh, self.torso, (0, -thigh_half), (0, torso_half))
            knee = pymunk.PivotJoint(shin, thigh, (0, -shin_half), (0, thigh_half))
            ankle = pymunk.PivotJoint(foot, shin, (-ankle_off, 0), (0, shin_half))
            return thigh, shin, foot, hip, knee, ankle

        (self.leg_l, self.shin_l, self.foot_l, self.hip_l, self.knee_l, self.ankle_l) = make_leg()
        (self.leg_r, self.shin_r, self.foot_r, self.hip_r, self.knee_r, self.ankle_r) = make_leg()

        # Neck
        self.neck = pymunk.PivotJoint(self.head, self.torso, (0, head_r), (0, -torso_half))

        self.humanoid = [
            self.head,
            self.torso,
            self.arm_l,
            self.arm_r,
            self.leg_l,
            self.leg_r,
            self.shin_l,
            self.shin_r,
            self.foot_l,
            self.foot_r,
        ]

        # No self-collision (still collides with ground, which is group 0)
        for body in self.humanoid:
            for shape in body.shapes:
                shape.filter = pymunk.ShapeFilter(group=1)

        # Joint limits from config
        cfg = {j.name: j for j in h.joint_configs}

        def limit(child, parent, name):
            lo, hi = cfg[name].limits
            return pymunk.RotaryLimitJoint(parent, child, lo, hi)

        self.neck_limit = limit(self.head, self.torso, "neck")
        self.shoulder_l_limit = limit(self.arm_l, self.torso, "shoulder")
        self.shoulder_r_limit = limit(self.arm_r, self.torso, "shoulder")
        self.hip_l_limit = limit(self.leg_l, self.torso, "hip")
        self.hip_r_limit = limit(self.leg_r, self.torso, "hip")
        self.knee_l_limit = limit(self.shin_l, self.leg_l, "knee")
        self.knee_r_limit = limit(self.shin_r, self.leg_r, "knee")
        self.ankle_l_limit = limit(self.foot_l, self.shin_l, "ankle")
        self.ankle_r_limit = limit(self.foot_r, self.shin_r, "ankle")

        # (child, parent) body for each joint. Joint angle = child.angle - parent.angle.
        self.joint_bodies: dict[JointName, tuple[pymunk.Body, pymunk.Body]] = {
            JointName.NECK: (self.head, self.torso),
            JointName.SHOULDER_L: (self.arm_l, self.torso),
            JointName.SHOULDER_R: (self.arm_r, self.torso),
            JointName.HIP_L: (self.leg_l, self.torso),
            JointName.HIP_R: (self.leg_r, self.torso),
            JointName.KNEE_L: (self.shin_l, self.leg_l),
            JointName.KNEE_R: (self.shin_r, self.leg_r),
            JointName.ANKLE_L: (self.foot_l, self.shin_l),
            JointName.ANKLE_R: (self.foot_r, self.shin_r),
        }
        assert set(self.joint_bodies) == set(JointName)

        # Add all joints to space
        self.space.add(
            self.neck,
            self.neck_limit,
            self.shoulder_l,
            self.shoulder_l_limit,
            self.shoulder_r,
            self.shoulder_r_limit,
            self.hip_l,
            self.hip_l_limit,
            self.hip_r,
            self.hip_r_limit,
            self.knee_l,
            self.knee_l_limit,
            self.knee_r,
            self.knee_r_limit,
            self.ankle_l,
            self.ankle_l_limit,
            self.ankle_r,
            self.ankle_r_limit,
        )

        # Mouse dragging
        self.mouse_joint = None
        self.mouse_body = pymunk.Body(body_type=pymunk.Body.KINEMATIC)

    def step(self):
        """Advance the simulation by one substep."""
        self.space.step(self.sub_dt)

    @property
    def sub_dt(self) -> float:
        return 1.0 / (self.config.world.sim_hz * self.substeps)

    def observe(self) -> list[float]:
        """Flat observation vector for all humanoid bodies."""
        obs = []
        for body in self.humanoid:
            obs += [
                body.position.x / self.ppm,
                body.position.y / self.ppm,
                body.angle,
                body.velocity.x / self.ppm,
                body.velocity.y / self.ppm,
                body.angular_velocity,
            ]
        return obs

    def joint_states(self) -> JointStates:
        """Read each joint's angle and angular velocity from the pymunk bodies."""
        dt = 1.0 / self.config.world.sim_hz
        return {
            name: JointState(
                current=child.angle - parent.angle,
                velocity=child.angular_velocity - parent.angular_velocity,
                dt=dt,
            )
            for name, (child, parent) in self.joint_bodies.items()
        }

    def apply_torques(self, torques: dict[JointName, float]) -> None:
        """Apply torques given in N·m (SI) to the pixel-scale world."""
        scale = self.ppm**2
        for name, torque in torques.items():
            child, parent = self.joint_bodies[name]
            child.torque += torque * scale
            parent.torque -= torque * scale

    def control_step(self, body_controller, targets: dict[JointName, float]) -> None:
        """One frame: several substeps of read states, compute torques, apply, step."""
        for _ in range(self.substeps):
            torques = body_controller.compute_torques(targets, self.joint_states())
            self.apply_torques(torques)
            self.step()

    def feet_on_ground(self) -> tuple[bool, bool]:
        """Whether each foot is touching anything (the ground, in practice)."""

        def touching(body):
            hits = []
            body.each_arbiter(hits.append)
            return len(hits) > 0

        return touching(self.foot_l), touching(self.foot_r)
