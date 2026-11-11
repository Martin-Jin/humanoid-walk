"""The viewer application for the humanoid walk project."""

import pygame
import pymunk.pygame_util
from humanoid_walk.config import Config
from humanoid_walk.sim.physics import Simulation
from humanoid_walk.viewer.render.render import Renderer
from humanoid_walk.controllers.joint_control import make_body_controller
from humanoid_walk.viewer.ui.joint_panel import JointPanel
from humanoid_walk.controllers.joint_state import JointName
from humanoid_walk.controllers.joint_control import base_name, make_body_controller


class App:
    """The viewer application for the humanoid walk project."""

    def __init__(self, config: Config):
        rendering = config.rendering
        self.config = config
        pygame.init()
        self.screen = pygame.display.set_mode((rendering.screen_width, rendering.screen_height))
        pygame.display.set_caption("Humanoid Walk Viewer")
        self.clock = pygame.time.Clock()

        # Headless physics, then the drawing layer on top of it
        self.sim = Simulation(
            config
        )  # the simulation handles the physics of the humanoid and environment
        self.sim.setup()  # the simulation is set up with the humanoid and environment
        self.targets = {}  # the current target angles for each joint, keyed by JointName
        joint_cfg_by_base = {j.name: j for j in config.humanoid.joint_configs}
        limits = {
            name: joint_cfg_by_base[base_name(name)].limits for name in JointName
        }  # the limits for each joint, keyed by JointName
        self.panel = JointPanel(
            rendering.screen_width, limits=limits
        )  # the control panel for joint targets, with limits for each joint
        self.body = make_body_controller(
            config,
            joint_cfg_by_base,
            config.joint_controller_types,
            config.controllers,
        )  # the body controller that manages all joint controllers
        self.draw_options = pymunk.pygame_util.DrawOptions(
            self.screen
        )  # the draw options for pymunk, used to render the simulation
        self.renderer = Renderer(self)  # the renderer that handles drawing and mouse interaction
        self.policy = self.panel  # the policy that generates target angles for each joint based on user input from the panel

    def run(self):
        """Run the main loop of the application."""
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                # Panel gets first look; the renderer only sees events it didn't consume
                if not self.panel.handle_event(event):
                    self.renderer.handle_event(event)
            self.targets = self.policy.act(self.sim.joint_states())
            # Compute and apply torques based on targets and current states
            # Also rendrer the simulation
            self.sim.control_step(self.body, self.targets)

            self.renderer.update()

            self.screen.fill((255, 255, 255))  # Clear screen with white
            self.sim.space.debug_draw(self.draw_options)
            # Here you would add code to render the humanoid and environment
            self.panel.draw(self.screen)
            pygame.display.flip()
            self.clock.tick(self.config.world.sim_hz)

        pygame.quit()
