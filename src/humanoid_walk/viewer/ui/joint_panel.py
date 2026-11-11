"""Clickable joint diagram for setting target angles by hand.

Click a joint to edit its target (rad). The box opens fully selected, so typing
replaces the value. A slider below it spans the joint's range. Enter or clicking
away saves, Escape cancels. The panel is also a Policy.
"""

from __future__ import annotations

import math

import pygame

from humanoid_walk.controllers.joint_state import JointName, JointStates, JointTargets
from humanoid_walk.agents.policy import Policy

PANEL_W, PANEL_H = 220, 450
MARGIN = 10
JOINT_RADIUS = 9
EDIT_BOX = pygame.Rect(90, 375, 110, 26)  # relative to the panel
SLIDER_X0, SLIDER_X1, SLIDER_Y = 16, PANEL_W - 16, 418  # track, relative to the panel
KNOB_RADIUS = 7

# Joint marker positions inside the panel (px), front-view schematic
JOINT_POS: dict[JointName, tuple[int, int]] = {
    JointName.NECK: (110, 62),
    JointName.SHOULDER_L: (80, 100),
    JointName.SHOULDER_R: (140, 100),
    JointName.HIP_L: (95, 190),
    JointName.HIP_R: (125, 190),
    JointName.KNEE_L: (90, 260),
    JointName.KNEE_R: (130, 260),
    JointName.ANKLE_L: (88, 330),
    JointName.ANKLE_R: (132, 330),
}

HEAD_CENTER, HEAD_RADIUS = (110, 38), 20

# Stick-figure lines, drawn behind the markers
BONES = [
    ((110, 62), (110, 190)),  # spine
    ((80, 100), (140, 100)),  # shoulders
    ((95, 190), (125, 190)),  # hips
    ((80, 100), (60, 170)),  # left arm
    ((140, 100), (160, 170)),  # right arm
    ((95, 190), (90, 260)),  # left thigh
    ((90, 260), (88, 330)),  # left shin
    ((88, 330), (72, 340)),  # left foot
    ((125, 190), (130, 260)),  # right thigh
    ((130, 260), (132, 330)),  # right shin
    ((132, 330), (148, 340)),  # right foot
]

BG = (240, 240, 240, 235)
BORDER = (120, 120, 120)
BONE = (90, 90, 90)
IDLE = (150, 150, 150)
SET = (60, 160, 90)  # target is non-zero
SELECTED = (40, 110, 220)

DEFAULT_LIMITS = (-math.pi, math.pi)


class JointPanel(Policy):
    """Top-right body diagram for editing each joint's target angle."""

    def __init__(
        self,
        screen_width: int,
        pose: JointTargets | None = None,
        limits: dict[JointName, tuple[float, float]] | None = None,
    ):
        self.rect = pygame.Rect(screen_width - PANEL_W - MARGIN, MARGIN, PANEL_W, PANEL_H)
        self.targets: JointTargets = dict(pose) if pose else {n: 0.0 for n in JointName}
        self.limits = limits or {}
        self.selected: JointName | None = None
        self.buffer = ""
        self.select_all = False  # text is highlighted; next key replaces it
        self.dragging = False
        self.font = pygame.font.SysFont(None, 20)
        self.small = pygame.font.SysFont(None, 18)

    # ---- Policy -------------------------------------------------------
    def act(self, states: JointStates) -> JointTargets:
        return dict(self.targets)

    def reset(self) -> None:
        self.targets = {n: 0.0 for n in JointName}
        self.selected = None
        self.dragging = False

    # ---- Helpers ------------------------------------------------------
    def _abs(self, pos: tuple[int, int]) -> tuple[int, int]:
        return pos[0] + self.rect.x, pos[1] + self.rect.y

    def _joint_at(self, mouse: tuple[int, int]) -> JointName | None:
        mx, my = mouse
        for name, pos in JOINT_POS.items():
            x, y = self._abs(pos)
            if (mx - x) ** 2 + (my - y) ** 2 <= (JOINT_RADIUS + 4) ** 2:
                return name
        return None

    def _range(self) -> tuple[float, float]:
        return self.limits.get(self.selected, DEFAULT_LIMITS)

    def _slider_hit(self, mouse: tuple[int, int]) -> bool:
        if self.selected is None:
            return False
        x0, _ = self._abs((SLIDER_X0, 0))
        x1, _ = self._abs((SLIDER_X1, 0))
        _, y = self._abs((0, SLIDER_Y))
        return x0 - KNOB_RADIUS <= mouse[0] <= x1 + KNOB_RADIUS and abs(mouse[1] - y) <= 12

    def _set_from_mouse_x(self, mouse_x: int) -> None:
        """Move the slider to the mouse x and update target and text box live."""
        lo, hi = self._range()
        x0, _ = self._abs((SLIDER_X0, 0))
        x1, _ = self._abs((SLIDER_X1, 0))
        frac = min(max((mouse_x - x0) / (x1 - x0), 0.0), 1.0)
        value = lo + frac * (hi - lo)
        self.targets[self.selected] = value
        self.buffer = f"{value:.2f}"
        self.select_all = False

    def _current_value(self) -> float:
        """Value shown by the slider: the typed number if valid, else the saved target."""
        try:
            return float(self.buffer)
        except ValueError:
            return self.targets[self.selected]

    def _commit(self) -> None:
        """Save the text box value. An unparsable value keeps the old target."""
        if self.selected is None:
            return
        try:
            self.targets[self.selected] = float(self.buffer)
        except ValueError:
            pass

    def _select(self, name: JointName | None) -> None:
        self._commit()
        self.selected = name
        self.buffer = f"{self.targets[name]:.2f}" if name else ""
        self.select_all = name is not None  # highlight so typing replaces it

    # ---- Events -------------------------------------------------------
    def handle_event(self, event: pygame.event.Event) -> bool:
        """Return True if the event was consumed (so the renderer should skip it)."""
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            in_panel = self.rect.collidepoint(event.pos)
            joint = self._joint_at(event.pos) if in_panel else None
            edit_rect = EDIT_BOX.move(self.rect.topleft)
            if in_panel and self._slider_hit(event.pos):
                self.dragging = True
                self._set_from_mouse_x(event.pos[0])
            elif joint is not None:
                self._select(joint)
            elif self.selected is not None and edit_rect.collidepoint(event.pos):
                self.select_all = False  # click into the box: normal caret editing
            else:
                self._select(None)  # clicked away: save and hide
            return in_panel

        if event.type == pygame.MOUSEMOTION and self.dragging:
            self._set_from_mouse_x(event.pos[0])
            return True

        if event.type == pygame.MOUSEBUTTONUP and event.button == 1 and self.dragging:
            self.dragging = False
            return True

        if event.type == pygame.KEYDOWN and self.selected is not None:
            if event.key in (pygame.K_RETURN, pygame.K_KP_ENTER):
                self._select(None)
            elif event.key == pygame.K_ESCAPE:
                self.selected = None
                self.buffer = ""
                self.select_all = False
            elif event.key == pygame.K_BACKSPACE:
                self.buffer = "" if self.select_all else self.buffer[:-1]
                self.select_all = False
            elif event.unicode and event.unicode in "0123456789.-":
                self.buffer = event.unicode if self.select_all else self.buffer + event.unicode
                self.select_all = False
            return True

        return False

    # ---- Drawing ------------------------------------------------------
    def draw(self, screen: pygame.Surface) -> None:
        panel = pygame.Surface(self.rect.size, pygame.SRCALPHA)
        panel.fill(BG)
        pygame.draw.rect(panel, BORDER, panel.get_rect(), 1)

        panel.blit(self.small.render("Joint targets (rad)", True, BONE), (8, 6))
        for a, b in BONES:
            pygame.draw.line(panel, BONE, a, b, 3)
        pygame.draw.circle(panel, BONE, HEAD_CENTER, HEAD_RADIUS, 3)

        for name, pos in JOINT_POS.items():
            if name == self.selected:
                colour = SELECTED
            elif abs(self.targets[name]) > 1e-9:
                colour = SET
            else:
                colour = IDLE
            pygame.draw.circle(panel, colour, pos, JOINT_RADIUS)
            pygame.draw.circle(panel, (255, 255, 255), pos, JOINT_RADIUS, 2)

        if self.selected is not None:
            self._draw_editor(panel)
        else:
            hint = self.small.render("Click a joint to edit", True, IDLE)
            panel.blit(hint, (10, EDIT_BOX.y + 7))

        screen.blit(panel, self.rect.topleft)

    def _draw_editor(self, panel: pygame.Surface) -> None:
        # Label and text box
        label = self.font.render(self.selected.value, True, BONE)
        panel.blit(label, (10, EDIT_BOX.y + 5))
        pygame.draw.rect(panel, (255, 255, 255), EDIT_BOX)
        pygame.draw.rect(panel, SELECTED, EDIT_BOX, 2)

        if self.select_all:
            text = self.font.render(self.buffer, True, (255, 255, 255))
            highlight = pygame.Rect(EDIT_BOX.x + 4, EDIT_BOX.y + 3, text.get_width() + 4, 20)
            pygame.draw.rect(panel, SELECTED, highlight)
            panel.blit(text, (EDIT_BOX.x + 6, EDIT_BOX.y + 5))
        else:
            caret = "|" if (pygame.time.get_ticks() // 500) % 2 == 0 else ""
            text = self.font.render(self.buffer + caret, True, (0, 0, 0))
            panel.blit(text, (EDIT_BOX.x + 6, EDIT_BOX.y + 5))

        # Slider across the joint's range of motion
        lo, hi = self._range()
        pygame.draw.line(panel, IDLE, (SLIDER_X0, SLIDER_Y), (SLIDER_X1, SLIDER_Y), 4)
        if lo < 0 < hi:  # zero tick
            zx = SLIDER_X0 + (0 - lo) / (hi - lo) * (SLIDER_X1 - SLIDER_X0)
            pygame.draw.line(panel, BONE, (zx, SLIDER_Y - 6), (zx, SLIDER_Y + 6), 2)
        frac = min(max((self._current_value() - lo) / (hi - lo), 0.0), 1.0)
        kx = SLIDER_X0 + frac * (SLIDER_X1 - SLIDER_X0)
        pygame.draw.circle(panel, SELECTED, (kx, SLIDER_Y), KNOB_RADIUS)
        pygame.draw.circle(panel, (255, 255, 255), (kx, SLIDER_Y), KNOB_RADIUS, 2)
        panel.blit(self.small.render(f"{lo:.2f}", True, BONE), (SLIDER_X0 - 6, SLIDER_Y + 10))
        hi_text = self.small.render(f"{hi:.2f}", True, BONE)
        panel.blit(hi_text, (SLIDER_X1 - hi_text.get_width() + 6, SLIDER_Y + 10))
