"""The viewer application for the humanoid walk project."""

from humanoid_walk.viewer.ui.button import Button

import pygame
import pymunk.pygame_util

from humanoid_walk.config import Config
from humanoid_walk.controllers.joint_control import base_name, make_body_controller
from humanoid_walk.controllers.joint_state import JointName
from humanoid_walk.sim.physics import Simulation
from humanoid_walk.viewer.render.debug import print_walk_state
from humanoid_walk.viewer.ui.joint_panel import JointPanel
from humanoid_walk.viewer.render.render import Renderer
from humanoid_walk.controllers.gait import GaitController

COM_COLOUR = (220, 30, 30)
COM_RADIUS = 6


class App:
    """The viewer application for the humanoid walk project."""

    def __init__(self, config: Config):
        rendering = config.rendering
        self.config = config
        pygame.init()
        self.screen = pygame.display.set_mode((rendering.screen_width, rendering.screen_height))
        pygame.display.set_caption("Humanoid Walk Viewer")
        self.clock = pygame.time.Clock()
        self.frame = 0  # For debugging, to print info every N frames

        # Headless physics, then the drawing layer on top of it
        self.sim = Simulation(config)
        self.sim.setup()
        self.targets = {}
        joint_cfg_by_base = {j.name: j for j in config.humanoid.joint_configs}
        self.body = make_body_controller(
            config,
            joint_cfg_by_base,
            config.joint_controller_names,
            config.controllers,
        )

        # gait controller for walking
        self.gait = GaitController(config)

        # Manual joint target panel; slider ranges come from humanoid.yaml
        limits = {name: joint_cfg_by_base[base_name(name)].limits for name in JointName}
        self.panel = JointPanel(rendering.screen_width, limits=limits)

        self.draw_options = pymunk.pygame_util.DrawOptions(self.screen)
        self.renderer = Renderer(self)

        # Default policy
        self.policy = self.gait

        # Reset button
        self.reset_button = Button((10, 10, 80, 30), "Reset")  # move if it overlaps the joint panel

    def draw_com(self):
        """Draw a red dot at the whole-body centre of mass."""
        x, y = self.sim.com_world()
        pygame.draw.circle(self.screen, (255, 255, 255), (int(x), int(y)), COM_RADIUS + 2)
        pygame.draw.circle(self.screen, COM_COLOUR, (int(x), int(y)), COM_RADIUS)

    def run(self):
        """Run the main loop of the application."""
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                if self.reset_button.clicked(event):
                    self.reset()
                    continue
                # Panel gets first look; the renderer only sees events it didn't consume
                if not self.panel.handle_event(event):
                    self.renderer.handle_event(event)

            # Policy picks targets, then torques are computed, applied and stepped
            self.targets = self.policy.act(self.sim.joint_states())
            self.gait.set_torso(self.sim.torso.angle, self.sim.torso.angular_velocity)
            print_walk_state(self)
            self.sim.control_step(self.body, self.targets)

            if self.policy is self.gait:
                self.gait.step_clock(1.0 / self.config.world.sim_hz)

            self.renderer.update()

            # Draw everything
            self.screen.fill((255, 255, 255))  # Clear screen with white
            self.sim.space.debug_draw(self.draw_options)
            self.draw_com()
            self.reset_button.draw(self.screen)
            self.panel.draw(self.screen)
            self.clock.tick(self.config.world.sim_hz)
            pygame.display.flip()

        pygame.quit()

    def reset(self):
        """Respawn the humanoid and reset all controller state."""
        self.sim.reset()
        joint_cfg_by_base = {j.name: j for j in self.config.humanoid.joint_configs}
        self.body = make_body_controller(
            self.config,
            joint_cfg_by_base,
            self.config.joint_controller_names,
            self.config.controllers,
        )
        self.gait = GaitController(self.config)
        self.policy = self.gait
        self.targets = {}
        self.frame = 0
