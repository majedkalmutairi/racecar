import numpy as np
import pygame

from racer.dashboard.overlay import mult_colour

MAP_COLOUR = (230, 40, 40)
CENTER_COLOUR = (60, 120, 255)
LINE_COLOUR = (40, 200, 40)
LOOKAHEAD_COLOUR = (255, 230, 0)


class MapOverlay:
    """Occupied cells drawn as red dots, cached in a surface that is rebuilt only while the map grows."""

    def __init__(self, track, min_hits, refresh_s=0.5):
        self.track = track
        self.min_hits = min_hits
        self.refresh_s = refresh_s
        self.surface = pygame.Surface((track.w_px, track.h_px))
        self.surface.set_colorkey((0, 0, 0))
        self.surface.set_alpha(180)
        self.next_refresh = 0.0

    def refresh(self, grid):
        self.surface.fill((0, 0, 0))
        rows, cols = np.nonzero(grid.occupied(self.min_hits))
        xs, ys = grid.cell_to_world(rows, cols)
        for x, y in zip(xs, ys):
            px, py = self.track.world_to_px(x, y)
            pygame.draw.rect(self.surface, MAP_COLOUR, (px - 1, py - 1, 3, 3))

    def draw(self, screen, grid, t, growing):
        if growing and t >= self.next_refresh:
            self.refresh(grid)
            self.next_refresh = t + self.refresh_s
        screen.blit(self.surface, (0, 0))


def draw_loop(screen, track, points, colour, width=1):
    pts = [track.world_to_px(x, y) for x, y in points]
    pygame.draw.lines(screen, colour, True, pts, width)


def draw_coloured_loop(screen, track, points, colours, width=3):
    pts = [track.world_to_px(x, y) for x, y in points]
    for i in range(len(pts)):
        pygame.draw.line(screen, colours[i], pts[i], pts[(i + 1) % len(pts)], width)


def draw_plan(screen, track, state):
    """Centerline in blue, racing line coloured by its learned multiplier, lookahead point in yellow."""
    if state["center"] is not None:
        draw_loop(screen, track, state["center"], CENTER_COLOUR, 1)
    if state["line"] is not None:
        mult = state["mult_per_point"]
        if mult is None:
            draw_loop(screen, track, state["line"], LINE_COLOUR, 3)
        else:
            draw_coloured_loop(screen, track, state["line"], [mult_colour(m) for m in mult])
    if state["lookahead"] is not None:
        pygame.draw.circle(screen, LOOKAHEAD_COLOUR, track.world_to_px(*state["lookahead"]), 5)
