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

        # Ground: horizontal static segment across the screen at floor_y_px
        self.add_segment(
            (r.screen_width / 2, r.floor_y_px),
            length_m=r.screen_width / self.ppm,
            width_m=0.05,
            friction=0.7,
            vertical=False,
            static=True,
        )

        # Test bodies: a head and a torso, dropped from above
        self.add_circle((200, 100), h.head_radius_m, mass=5.0, velocity_func=limit_velocity)
        self.add_segment(
            (400, 100),
            length_m=h.torso_length_m,
            width_m=h.torso_width_m,
            mass=h.mass_kg * 0.5,
            velocity_func=limit_velocity,
        )

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
                self.mouse_joint.max_force = 50000
                self.mouse_joint.error_bias = (1 - 0.15) ** 60
                self.space.add(self.mouse_joint)

        elif event.type == pygame.MOUSEBUTTONUP:
            if self.mouse_joint is not None:
                self.space.remove(self.mouse_joint)
                self.mouse_joint = None

    def update(self):
        """Per-frame update: move the mouse body."""
        self.mouse_body.position = pygame.mouse.get_pos()
