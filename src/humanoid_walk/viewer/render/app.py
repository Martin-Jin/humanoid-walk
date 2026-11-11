"""The viewer application for the humanoid walk project."""

import pygame
import pymunk
import pymunk.pygame_util
from humanoid_walk.config import Config
from humanoid_walk.viewer.render.render import Renderer


class App:
    """The viewer application for the humanoid walk project."""

    def __init__(self, config: Config):
        rendering = config.rendering
        self.config = config
        pygame.init()
        self.screen = pygame.display.set_mode((rendering.screen_width, rendering.screen_height))
        pygame.display.set_caption("Humanoid Walk Viewer")
        self.clock = pygame.time.Clock()

        # Initialize Pymunk space
        self.space = pymunk.Space()
        self.space.gravity = (0.0, 900.0)
        self.draw_options = pymunk.pygame_util.DrawOptions(self.screen)

        # Initialize the renderer and set up the scene
        # note keep at the end of the init function so that the renderer can access the app's attributes
        self.renderer = Renderer(self)
        self.renderer.setup()

    def run(self):
        """Run the main loop of the application."""
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                self.renderer.handle_event(event)

            # Main rendering and simulation step
            self.renderer.update()
            self.space.step(1.0 / self.config.world.sim_hz)

            self.screen.fill((255, 255, 255))  # Clear screen with white
            self.space.debug_draw(self.draw_options)
            # Here you would add code to render the humanoid and environment
            pygame.display.flip()
            self.clock.tick(self.config.world.sim_hz)

        pygame.quit()
