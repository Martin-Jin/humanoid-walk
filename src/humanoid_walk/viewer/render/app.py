"""The viewer application for the humanoid walk project."""

import pygame
import pymunk.pygame_util
from humanoid_walk.agents.policy import HoldPolicy
from humanoid_walk.config import Config
from humanoid_walk.sim.physics import Simulation
from humanoid_walk.viewer.render.render import Renderer
from humanoid_walk.controllers.joint_control import make_body_controller


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
        self.sim = Simulation(config)
        self.sim.setup()
        self.targets = {}
        joint_cfg_by_base = {j.name: j for j in config.humanoid.joint_configs}
        self.body = make_body_controller(
            config,
            joint_cfg_by_base,
            config.joint_controller_types,
            config.controllers,
        )
        self.draw_options = pymunk.pygame_util.DrawOptions(self.screen)
        self.renderer = Renderer(self)
        self.policy = HoldPolicy()  # Default policy that holds the humanoid in a standing pose

    def run(self):
        """Run the main loop of the application."""
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                self.renderer.handle_event(event)
            self.targets = self.policy.act(self.sim.joint_states())
            # Compute and apply torques based on targets and current states
            # Also rendrer the simulation
            self.sim.control_step(self.body, self.targets)

            self.renderer.update()

            self.screen.fill((255, 255, 255))  # Clear screen with white
            self.sim.space.debug_draw(self.draw_options)
            # Here you would add code to render the humanoid and environment
            pygame.display.flip()
            self.clock.tick(self.config.world.sim_hz)

        pygame.quit()
