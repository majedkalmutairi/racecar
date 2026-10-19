from collections import deque

import numpy as np
import pygame

from racer.dashboard.overlay import conf_colour, ramp

OUTSIDE = (16, 18, 22)
ASPHALT = (54, 58, 66)
MAP_COLOUR = (230, 60, 60)
CENTER_COLOUR = (90, 110, 160)
AHEAD_COLOUR = (0, 225, 255)
TRAIL_COLOUR = (235, 235, 235)
LOOKAHEAD_COLOUR = (255, 230, 0)
COLOUR_MODES = ["speed", "grip", "confidence"]


class TrackView:
    """The track drawn in calm colours (asphalt and dark surroundings) instead of the black-and-white PNG.
    Repainted only when the wall mask changes, e.g. a box is dropped."""

    def __init__(self, track):
        self.track = track
        self.surface = None
        self.walls_seen = -1

    def draw(self, screen):
        if self.surface is None or self.track.wall_version != self.walls_seen:
            rgb = np.where(self.track.wall[:, :, None], OUTSIDE, ASPHALT).astype(np.uint8)
            self.surface = pygame.surfarray.make_surface(rgb.transpose(1, 0, 2)).convert()
            self.walls_seen = self.track.wall_version
        screen.blit(self.surface, (0, 0))


class MapOverlay:
    """Mapped wall cells as red dots, cached in a surface that is rebuilt only while the map grows."""

    def __init__(self, track, min_hits, refresh_s=0.5):
        self.track = track
        self.min_hits = min_hits
        self.refresh_s = refresh_s
        self.surface = pygame.Surface((track.w_px, track.h_px))
        self.surface.set_colorkey((0, 0, 0))
        self.next_refresh = 0.0

    def refresh(self, grid):
        self.surface.fill((0, 0, 0))
        rows, cols = np.nonzero(grid.occupied(self.min_hits))
        xs, ys = grid.cell_to_world(rows, cols)
        for x, y in zip(xs, ys):
            px, py = self.track.world_to_px(x, y)
            pygame.draw.rect(self.surface, MAP_COLOUR, (px - 1, py - 1, 2, 2))

    def draw(self, screen, grid, t, growing):
        if growing and t >= self.next_refresh:
            self.refresh(grid)
            self.next_refresh = t + self.refresh_s
        self.surface.set_alpha(200 if growing else 70)     # bright while mapping, faint once racing
        screen.blit(self.surface, (0, 0))


class Trail:
    """Where the car really drove over roughly the last lap."""

    def __init__(self, length=900):
        self.points = deque(maxlen=length)

    def add(self, track, x, y):
        self.points.append(track.world_to_px(x, y))

    def draw(self, screen):
        if len(self.points) > 1:
            pygame.draw.lines(screen, TRAIL_COLOUR, False, list(self.points), 1)


def draw_loop(screen, track, points, colour, width=1):
    pts = [track.world_to_px(x, y) for x, y in points]
    pygame.draw.lines(screen, colour, True, pts, width)


def draw_coloured_loop(screen, track, points, colours, width=4):
    pts = [track.world_to_px(x, y) for x, y in points]
    for i in range(len(pts)):
        pygame.draw.line(screen, colours[i], pts[i], pts[(i + 1) % len(pts)], width)


def line_colours(state, mode):
    """Colour of every racing-line point, and the legend that explains it: (colours, legend text, low, high)."""
    if mode == "speed" and state["v_line"] is not None:
        v = state["v_line"]
        top = max(float(v.max()), 1e-6)
        return [ramp(x / top) for x in v], "planned speed (m/s)", 0.0, top
    if mode == "grip" and state["grip"] is not None:
        g = state["grip"][state["section_of"]]
        return [ramp(x / 12.0) for x in g], "grip it trusts (m/s^2)", 0.0, 12.0
    if mode == "confidence" and state["section_conf"] is not None:
        c = state["section_conf"][state["section_of"]]
        return [conf_colour(x) for x in c], "confidence (clean laps in a row)", 0.0, 1.0
    return None, None, None, None


def draw_legend(screen, font, x, y, label, lo, hi):
    w = 160
    back = pygame.Surface((max(w, font.size(label)[0]) + 16, 50), pygame.SRCALPHA)
    back.fill((16, 18, 22, 200))
    screen.blit(back, (x - 8, y - 4))
    for i in range(w):
        pygame.draw.line(screen, ramp(i / (w - 1)), (x + i, y + 16), (x + i, y + 24))
    screen.blit(font.render(label, True, (230, 230, 230)), (x, y))
    screen.blit(font.render(f"{lo:g}", True, (160, 160, 160)), (x, y + 26))
    hi_txt = font.render(f"{hi:.1f}" if hi < 100 else f"{hi:.0f}", True, (160, 160, 160))
    screen.blit(hi_txt, (x + w - hi_txt.get_width(), y + 26))


def draw_plan(screen, track, state, view, font):
    """Centerline thin, racing line coloured by the chosen mode, the path the reflex is checking, lookahead point."""
    if state["center"] is not None:
        draw_loop(screen, track, state["center"], CENTER_COLOUR, 1)
    if state["line"] is not None:
        colours, label, lo, hi = line_colours(state, view["colour"])
        if colours is None:
            draw_loop(screen, track, state["line"], (80, 220, 120), 3)
        else:
            draw_coloured_loop(screen, track, state["line"], colours)
            draw_legend(screen, font, 12, 10, label + "   (C changes)", lo, hi)
    if state["ahead"] is not None and len(state["ahead"]) > 1:
        pygame.draw.lines(screen, AHEAD_COLOUR, False, [track.world_to_px(x, y) for x, y in state["ahead"]], 2)
    if state["lookahead"] is not None:
        pygame.draw.circle(screen, LOOKAHEAD_COLOUR, track.world_to_px(*state["lookahead"]), 4)
