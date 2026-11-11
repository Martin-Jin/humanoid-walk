"""Defines helper functions using pymunk to render the humanoid + environment in a pygame window."""

from __future__ import annotations
import pygame
import pymunk
from pymunk.vec2d import Vec2d
from typing import TYPE_CHECKING

from humanoid_walk.sim.rules import limit_velocity

if TYPE_CHECKING:
    from humanoid_walk.viewer.render.app import App


class Renderer:
    """A class to handle rendering of the humanoid and environment using pymunk."""

    def __init__(self, app: App):
        self.config = app.config
        self.screen = app.screen
        self.space = app.space
        self.draw_options = app.draw_options
        self.clock = app.clock
        self.ppm = app.config.rendering.pixels_per_meter
        self.humanoid = []  # array of all bodies that make up the humanoid

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

        # Spawn position: feet just above the floor
        tx = r.screen_width / 2
        below_torso_centre = torso_half + self.px(h.leg_length_m) + self.px(h.foot_width_m) / 2
        ty = r.floor_y_px - below_torso_centre - 5

        top_y = ty - torso_half
        bottom_y = ty + torso_half

        # Head: sits on the torso's top tip
        self.head = self.add_circle(
            (tx, top_y - head_r),
            h.head_radius_m,
            mass=5.0,
            velocity_func=limit_velocity,
            friction=fric,
        )

        # Torso
        self.torso = self.add_segment(
            (tx, ty),
            length_m=h.torso_length_m,
            width_m=h.torso_width_m,
            mass=h.mass_kg * 0.5,
            velocity_func=limit_velocity,
            friction=fric,
        )

        # Arms: hang down from the top tip
        def make_arm():
            arm = self.add_segment(
                (tx, top_y + arm_half),
                length_m=h.arm_length_m,
                width_m=h.arm_width_m,
                mass=h.mass_kg * 0.1,
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
                mass=h.mass_kg * 0.15,
                velocity_func=limit_velocity,
                friction=fric,
            )
            shin = self.add_segment(
                (tx, knee_y + shin_half),
                length_m=shin_len,
                width_m=h.leg_width_m,
                mass=h.mass_kg * 0.08,
                velocity_func=limit_velocity,
                friction=fric,
            )
            foot = self.add_segment(
                (tx + foot_half / 2, ankle_y),
                length_m=h.foot_length_m,
                width_m=h.foot_width_m,
                mass=h.mass_kg * 0.02,
                velocity_func=limit_velocity,
                friction=fric,
                vertical=False,
            )

            hip = pymunk.PivotJoint(thigh, self.torso, (0, -thigh_half), (0, torso_half))
            knee = pymunk.PivotJoint(shin, thigh, (0, -shin_half), (0, thigh_half))
            ankle = pymunk.PivotJoint(foot, shin, (-foot_half / 2, 0), (0, shin_half))
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

        def limit(a, b, name):
            lo, hi = cfg[name].limits
            return pymunk.RotaryLimitJoint(a, b, lo, hi)

        self.neck_limit = limit(self.head, self.torso, "neck")
        self.shoulder_l_limit = limit(self.arm_l, self.torso, "shoulder")
        self.shoulder_r_limit = limit(self.arm_r, self.torso, "shoulder")
        self.hip_l_limit = limit(self.leg_l, self.torso, "hip")
        self.hip_r_limit = limit(self.leg_r, self.torso, "hip")
        self.knee_l_limit = limit(self.shin_l, self.leg_l, "knee")
        self.knee_r_limit = limit(self.shin_r, self.leg_r, "knee")
        self.ankle_l_limit = limit(self.foot_l, self.shin_l, "ankle")
        self.ankle_r_limit = limit(self.foot_r, self.shin_r, "ankle")

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

    def handle_event(self, event):
        """Handle a single pygame event (mouse dragging)."""
        if event.type == pygame.MOUSEBUTTONDOWN:
            if self.mouse_joint is not None:
                self.space.remove(self.mouse_joint)
                self.mouse_joint = None

            p = Vec2d(*event.pos)
            hit = self.space.point_query_nearest(p, 5, pymunk.ShapeFilter())
            if hit is not None and hit.shape.body.body_type == pymunk.Body.DYNAMIC:
                shape = hit.shape
                nearest = hit.point if hit.distance > 0 else p
                self.mouse_joint = pymunk.PivotJoint(
                    self.mouse_body,
                    shape.body,
                    (0, 0),
                    shape.body.world_to_local(nearest),
                )
                self.mouse_joint.max_force = 500000
                self.mouse_joint.error_bias = (1 - 0.3) ** 60
                self.space.add(self.mouse_joint)

        elif event.type == pygame.MOUSEBUTTONUP:
            if self.mouse_joint is not None:
                self.space.remove(self.mouse_joint)
                self.mouse_joint = None

    def update(self):
        """Per-frame update: move the mouse body."""
        self.mouse_body.position = pygame.mouse.get_pos()
