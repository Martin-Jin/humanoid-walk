"""Draws a Simulation in a pygame window and handles mouse dragging."""

from __future__ import annotations

from typing import TYPE_CHECKING

import pygame
import pymunk
from pymunk.vec2d import Vec2d
from humanoid_walk.controllers.joint_state import JointName, JointTargets

if TYPE_CHECKING:
    from humanoid_walk.viewer.render.app import App


class Renderer:
    """Handles drawing and mouse interaction for a Simulation."""

    def __init__(self, app: App):
        self.config = app.config
        self.screen = app.screen
        self.space = app.sim.space
        self.draw_options = app.draw_options
        self.clock = app.clock

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

    def get_targets(self) -> JointTargets:
        """Target angle (rad) for every joint. Placeholder: hold all at 3."""
        return {name: 3.0 for name in JointName}

    def update(self):
        """Per-frame update: move the mouse body."""
        self.mouse_body.position = pygame.mouse.get_pos()
