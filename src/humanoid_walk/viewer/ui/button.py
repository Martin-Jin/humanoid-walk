"""UI for the humanoid walk viewer."""

import pygame


class Button:
    """A clickable rectangular button with a text label."""

    def __init__(self, rect: tuple[int, int, int, int], label: str):
        self.rect = pygame.Rect(rect)
        self.label = label
        self.font = pygame.font.SysFont(None, 24)  # needs pygame.init() first

    def clicked(self, event: pygame.event.Event) -> bool:
        """True if this event is a left click inside the button."""
        return (
            event.type == pygame.MOUSEBUTTONDOWN
            and event.button == 1
            and self.rect.collidepoint(event.pos)
        )

    def draw(self, screen: pygame.Surface) -> None:
        """Draw the button."""
        pygame.draw.rect(screen, (230, 230, 230), self.rect, border_radius=4)
        pygame.draw.rect(screen, (60, 60, 60), self.rect, 2, border_radius=4)
        text = self.font.render(self.label, True, (0, 0, 0))
        screen.blit(text, text.get_rect(center=self.rect.center))
