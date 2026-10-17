from collections import deque

import pygame

PANEL_BG = (28, 28, 32)
TEXT = (230, 230, 230)
DIM = (140, 140, 150)
ACTUAL = (255, 255, 255)
TARGET = (80, 220, 80)
ALERT = (255, 80, 80)
MODE_COLOURS = {
    "EXPLORE": (90, 170, 255),
    "PLAN": (255, 200, 0),
    "RACE": (80, 220, 80),
    "TRACK_CHANGED": (255, 80, 80),
}


def mult_colour(m):
    """0.4 -> red, 0.8 -> yellow, 1.2 -> green."""
    u = min(max((m - 0.4) / 0.8, 0.0), 1.0)
    if u < 0.5:
        return (255, int(510 * u), 0)
    return (int(510 * (1 - u)), 255, 0)


def text(screen, font, s, x, y, colour=TEXT):
    screen.blit(font.render(s, True, colour), (x, y))
    return y + font.get_linesize()


def seconds(t):
    return "-" if t is None else f"{t:.2f} s"


class Dashboard:
    """Side panel that only reads state and draws it. It never computes anything the car needs."""

    def __init__(self, cfg, font, big_font):
        self.w = cfg["dashboard"]["panel_w"]
        self.v_max = cfg["car"]["max_speed_mps"]
        self.font = font
        self.big_font = big_font
        n = cfg["dashboard"]["graph_len"]
        self.v_actual = deque(maxlen=n)
        self.v_target = deque(maxlen=n)

    def push(self, v_actual, v_target):
        self.v_actual.append(v_actual)
        self.v_target.append(v_target)

    def draw(self, screen, state):
        x0 = screen.get_width() - self.w
        pygame.draw.rect(screen, PANEL_BG, (x0, 0, self.w, screen.get_height()))
        x = x0 + 14
        y = 12

        y = text(screen, self.big_font, state["mode"], x, y, MODE_COLOURS.get(state["mode"], TEXT))
        y = text(screen, self.font, f"track: {state['track']}", x, y, DIM) + 8
        y = text(screen, self.font, f"lap {state['lap']}   {state['lap_s']:.1f} s", x, y)
        y = text(screen, self.font, f"last lap  {seconds(state['last_lap_s'])}", x, y)
        y = text(screen, self.font, f"best lap  {seconds(state['best_lap_s'])}", x, y) + 8
        y = text(screen, self.font, f"speed  {state['speed']:.2f} m/s   target  {state['v_target']:.2f}", x, y)
        mismatch = state["mismatch"]
        y = text(screen, self.font, f"map mismatch  {'-' if mismatch is None else f'{mismatch:.2f}'}", x, y)
        ttc = state["min_ttc"]
        colour = ALERT if state["safety_active"] else TEXT
        y = text(screen, self.font, f"min ttc  {'inf' if ttc == float('inf') else f'{ttc:.2f} s'}", x, y, colour) + 10

        y = text(screen, self.font, "speed: white = actual, green = target", x, y, DIM)
        self._graph(screen, x, y, self.w - 28, 80)
        y += 90

        y = text(screen, self.font, "last safety event", x, y, DIM)
        e = state["last_event"]
        if e is None:
            y = text(screen, self.font, "none", x, y)
        else:
            y = text(screen, self.font, f"ttc {e['min_ttc']} s, section {e['section']}, lap {e['lap']}", x, y)
            y = text(screen, self.font, f"{e['mode']} at t = {e['t']} s", x, y)
        y += 10

        y = text(screen, self.font, "line colour = learned multiplier", x, y, DIM)
        for i, (m, label) in enumerate([(0.4, "0.4"), (0.8, "0.8"), (1.2, "1.2")]):
            pygame.draw.rect(screen, mult_colour(m), (x + 80 * i, y + 4, 24, 10))
            text(screen, self.font, label, x + 80 * i + 30, y, DIM)
        y += 30

        if state["paused"]:
            y = text(screen, self.big_font, "PAUSED", x, y, ALERT)
        y = screen.get_height() - 4 * self.font.get_linesize() - 10
        y = text(screen, self.font, "R  reset and explore again", x, y, DIM)
        y = text(screen, self.font, "SPACE  pause", x, y, DIM)
        y = text(screen, self.font, "N  next track", x, y, DIM)
        text(screen, self.font, "A  manual (arrow keys)", x, y, DIM)

    def _graph(self, screen, x, y, w, h):
        pygame.draw.rect(screen, DIM, (x, y, w, h), 1)
        for series, colour in ((self.v_target, TARGET), (self.v_actual, ACTUAL)):
            if len(series) < 2:
                continue
            step = w / (series.maxlen - 1)
            points = [(x + i * step, y + h - min(v, self.v_max) / self.v_max * (h - 2)) for i, v in enumerate(series)]
            pygame.draw.lines(screen, colour, False, points, 1)
