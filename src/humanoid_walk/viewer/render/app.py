"""The viewer application for the humanoid walk project."""

import pygame
from humanoid_walk.config import Config


class App:
    """The viewer application for the humanoid walk project."""

    def __init__(self, config: Config):
        rendering = config.viewer.rendering
        self.config = config
        pygame.init()
        self.screen = pygame.display.set_mode((rendering.screen_width, rendering.screen_height))
        pygame.display.set_caption("Humanoid Walk Viewer")
        self.clock = pygame.time.Clock()

    def run(self):
        """Run the main loop of the application."""
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            self.screen.fill((255, 255, 255))  # Clear screen with white
            # Here you would add code to render the humanoid and environment
            pygame.display.flip()
            self.clock.tick(self.config.game.world.sim_hz)

        pygame.quit()
