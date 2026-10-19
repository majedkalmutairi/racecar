from collections import deque

import numpy as np
import pygame

BG = (22, 24, 29)
CARD = (33, 36, 43)
TEXT = (232, 234, 238)
DIM = (140, 146, 158)
FAINT = (80, 86, 98)
GREEN = (80, 220, 120)
BLUE = (90, 170, 255)
YELLOW = (255, 205, 70)
RED = (255, 90, 90)
MODE_COLOURS = {"EXPLORE": BLUE, "PLAN": YELLOW, "RACE": GREEN, "TRACK_CHANGED": RED}
MODE_TEXT = {
    "EXPLORE": "first lap: slow, keeps to the middle, maps the walls (red dots)",
    "PLAN": "turning the map into a racing line and a speed plan",
    "RACE": "following the racing line, learning each section's grip",
    "TRACK_CHANGED": "the walls no longer match the map: starting over",
}
G = 9.81


def ramp(u):
    """0..1 -> blue, cyan, green, yellow, red: slow to fast, low to high."""
    stops = [(60, 110, 255), (40, 210, 230), (80, 220, 120), (255, 210, 60), (255, 80, 70)]
    u = min(max(float(u), 0.0), 1.0) * (len(stops) - 1)
    i = min(int(u), len(stops) - 2)
    f = u - i
    return tuple(int(a + (b - a) * f) for a, b in zip(stops[i], stops[i + 1]))


def conf_colour(c):
    """Confidence 0..1 -> grey to green."""
    c = min(max(float(c), 0.0), 1.0)
    return tuple(int(a + (b - a) * c) for a, b in zip((95, 100, 112), GREEN))


def secs(t, sign=False):
    if t is None:
        return "-"
    return f"{t:+.2f} s" if sign else f"{t:.2f} s"


class Dashboard:
    """Side panel that only reads state and draws it. It never computes anything the car needs.

    Every number comes with its unit and a few words on what it means; the debug card at the bottom
    shows the raw values worth watching while tuning.
    """

    def __init__(self, cfg, fonts):
        self.w = cfg["dashboard"]["panel_w"]
        self.v_max = cfg["car"]["max_speed_mps"]
        self.ttc_s = cfg["safety"]["ttc_s"]
        self.err_ok = cfg["learning"]["err_ok_m"]
        self.small, self.font, self.big, self.huge = fonts
        n = cfg["dashboard"]["graph_len"]
        self.v_actual = deque(maxlen=n)
        self.v_target = deque(maxlen=n)

    def push(self, v_actual, v_target):
        self.v_actual.append(v_actual)
        self.v_target.append(v_target)

    # --- helpers ----------------------------------------------------------------------------------------------
    def text(self, s, x, y, colour=TEXT, font=None):
        surf = (font or self.font).render(s, True, colour)
        self.screen.blit(surf, (x, y))
        return surf.get_width()

    def card(self, y, h, title=None, hint=None):
        pygame.draw.rect(self.screen, CARD, (self.x0 + 8, y, self.w - 16, h), border_radius=6)
        if title:
            self.text(title, self.x0 + 18, y + 5, DIM, self.small)
        if hint:
            self.text(hint, self.x0 + self.w - 18 - self.small.size(hint)[0], y + 5, FAINT, self.small)
        return y + h + 6

    # --- the panel ----------------------------------------------------------------------------------------------
    def draw(self, screen, state, view):
        self.screen = screen
        self.x0 = screen.get_width() - self.w
        h = screen.get_height()
        pygame.draw.rect(screen, BG, (self.x0, 0, self.w, h))
        y = self.header(6, state)
        y = self.timing(y, state)
        y = self.speed(y, state)
        y = self.learned(y, state)
        y = self.safety(y, state)
        if view["debug"]:
            self.debug(y, state)
        self.text("R reset   SPACE pause   N next track   A drive yourself (arrows)", self.x0 + 12, h - 36, FAINT,
                  self.small)
        self.text("C line colour   L lidar beams   D debug", self.x0 + 12, h - 20, FAINT, self.small)

    def header(self, y, s):
        colour = MODE_COLOURS.get(s["mode"], TEXT)
        label = self.font.render(s["mode"], True, BG)
        pygame.draw.rect(self.screen, colour, (self.x0 + 8, y + 2, label.get_width() + 24, label.get_height() + 6),
                         border_radius=12)
        self.screen.blit(label, (self.x0 + 20, y + 5))
        right = f"{s['track']}   lap {s['lap']}"
        self.text(right, self.x0 + self.w - 12 - self.font.size(right)[0], y + 5, DIM)
        flag = "PAUSED" if s["paused"] else s["recovery"]
        if flag:
            self.text(flag, self.x0 + 40 + label.get_width(), y + 5, YELLOW if s["paused"] else RED)
        y += label.get_height() + 12
        self.text(MODE_TEXT.get(s["mode"], ""), self.x0 + 12, y, DIM, self.small)
        return y + 20

    def timing(self, y, s):
        top = y
        y = self.card(y, 150, "LAP TIMES")
        x = self.x0 + 18
        self.text(f"{s['lap_s']:.2f} s", x, top + 18, TEXT, self.huge)
        self.text("this lap so far", x + 2, top + 70, FAINT, self.small)
        last, best = s["last_lap_s"], s["best_lap_s"]
        col = self.x0 + 210
        self.text("last", col, top + 22, DIM)
        self.text(secs(last), col + 60, top + 22)
        self.text("best", col, top + 42, DIM)
        self.text(secs(best), col + 60, top + 42, GREEN)
        if last is not None and best is not None:
            self.text("vs best", col, top + 64, DIM, self.small)
            self.text(secs(last - best, True), col + 60, top + 64, DIM, self.small)
        if s["plan_lap_s"] is not None:
            self.text(f"what it thinks it can do now:  {s['plan_lap_s']:.2f} s", x, top + 86, DIM, self.small)
        self.lap_chart(x, top + 106, self.w - 36, 38, s["lap_history"])
        return y

    def lap_chart(self, x, y, w, h, history):
        race = [t for n, t, mode in history if mode == "RACE"][-40:]
        explore = [t for n, t, mode in history if mode == "EXPLORE"]
        self.text("every race lap", x, y, FAINT, self.small)
        self.text(f"explore {explore[-1]:.1f} s" if explore else "explore -", x, y + h - 14, FAINT, self.small)
        if not race:
            return
        lo = min(race)
        hi = min(max(race), 1.5 * lo)               # the slow first race laps would flatten everything else
        x0, bw = x + 110, (w - 150) / 40
        for i, t in enumerate(race):
            bh = 4 + (h - 6) * min(t - lo, hi - lo) / max(hi - lo, 1e-6)
            colour = GREEN if t == lo else (TEXT if i == len(race) - 1 else FAINT)
            pygame.draw.rect(self.screen, colour, (x0 + i * bw, y + h - bh, max(bw - 1, 1), bh))
        self.text(f"{hi:.1f}+" if hi < max(race) else f"{hi:.1f}", x + w - 30, y, FAINT, self.small)
        self.text(f"{lo:.1f}", x + w - 28, y + h - 14, GREEN, self.small)

    def speed(self, y, s):
        top = y
        y = self.card(y, 92, "SPEED", f"bar = 0 .. {self.v_max:.0f} m/s top speed")
        x = self.x0 + 18
        v, target = s["speed"], s["v_target"]
        self.text(f"{v:.2f} m/s", x, top + 18, TEXT, self.big)
        self.text(f"wants {target:.2f} m/s", x + 150, top + 26, DIM)
        bar_w = self.w - 36
        frac = min(max(v, 0.0) / self.v_max, 1.0)
        pygame.draw.rect(self.screen, FAINT, (x, top + 54, bar_w, 8), border_radius=4)
        pygame.draw.rect(self.screen, ramp(frac), (x, top + 54, int(bar_w * frac), 8), border_radius=4)
        tx = x + int(bar_w * min(max(target, 0.0) / self.v_max, 1))
        pygame.draw.line(self.screen, TEXT, (tx, top + 50), (tx, top + 65), 2)
        self.graph(x, top + 68, bar_w, 20)
        return y

    def graph(self, x, y, w, h):
        for series, colour in ((self.v_target, GREEN), (self.v_actual, TEXT)):
            if len(series) < 2:
                continue
            step = w / (series.maxlen - 1)
            pts = [(x + i * step, y + h - min(max(v, 0.0), self.v_max) / self.v_max * h) for i, v in enumerate(series)]
            pygame.draw.lines(self.screen, colour, False, pts, 1)

    def learned(self, y, s):
        top = y
        y = self.card(y, 146, "WHAT IT HAS LEARNED", "one bar per track section")
        grip = s["grip"]
        if grip is None:
            self.text("learning starts with the first race lap", self.x0 + 18, top + 50, FAINT)
            return y
        conf, corner = s["section_conf"], s["section_corner"]
        n, x, w, h = len(grip), self.x0 + 34, self.w - 52, 56
        bw, base, scale = w / n, top + 30 + h, 12.0          # scale: m/s^2 at the top of the chart
        for g in (4, 8, 12):
            gy = base - h * g / scale
            pygame.draw.line(self.screen, FAINT, (x, gy), (x + w, gy), 1)
            self.text(f"{g}", x - 18, gy - 8, FAINT, self.small)
        for i in range(n):
            bx, bwi = x + i * bw, max(bw - 1, 1)
            if not corner[i]:                                 # a straight never tests the grip
                pygame.draw.rect(self.screen, FAINT, (bx, base - 5, bwi, 5), 1)
                continue
            bh = max(2, h * min(grip[i], scale) / scale)
            pygame.draw.rect(self.screen, conf_colour(conf[i]), (bx, base - bh, bwi, bh))
            c = s["trouble_grip"][i]
            if np.isfinite(c) and c < scale:
                pygame.draw.line(self.screen, TEXT, (bx, base - h * c / scale), (bx + bwi, base - h * c / scale), 1)
        if s["section"] is not None:
            cx = x + (s["section"] + 0.5) * bw
            pygame.draw.polygon(self.screen, TEXT, [(cx, base + 2), (cx - 4, base + 9), (cx + 4, base + 9)])
        x = self.x0 + 18
        self.text("height = grip trusted (m/s^2)  green = sure  tick = trouble here  box = straight", x, base + 11,
                  FAINT, self.small)
        hit = s["trouble_grip"][corner & np.isfinite(s["trouble_grip"])]
        if len(hit):           # what it measured, not what untested corners trust
            g = float(np.median(hit))
            found = f"trouble at {g:.1f} m/s^2 = {g / G:.2f} g  ({len(hit)} corners)"
        else:
            found = "no trouble anywhere yet"
        self.text(found, x, base + 27, TEXT, self.small)
        conf = f"sure {100 * s['confidence']:.0f} %"
        self.text(conf, self.x0 + self.w - 18 - self.small.size(conf)[0], base + 27, DIM, self.small)
        return y

    def safety(self, y, s):
        top = y
        y = self.card(y, 64, "SAFETY", f"brakes if a hit is under {self.ttc_s:.1f} s away")
        x = self.x0 + 18
        ttc = s["min_ttc"]
        status, colour = ("BRAKING", RED) if s["safety_active"] else ("clear", GREEN)
        self.text(f"reflex {status}", x, top + 20, colour)
        self.text("nothing close" if not ttc < 5 else f"{ttc:.2f} s to a hit", x + 160, top + 20, DIM)
        c = s["counts"]
        self.text(f"reflex {c.get('reflex', 0)}   back-ups {c.get('recover', 0)}   crashes {s['crashes']}   "
                  f"sliding now: {'YES' if s['slipping'] else 'no'}", x, top + 42, DIM, self.small)
        return y

    def debug(self, y, s):
        bottom = self.screen.get_height() - 40
        err, sec = s["track_err"], s["section"]
        ceil = None if sec is None else s["ceiling_grip"][sec]
        rows = [
            ("tracking error", f"{100 * abs(err):.1f} cm", RED if abs(err) > self.err_ok else TEXT),
            ("lookahead", "-" if s["lookahead_m"] is None else f"{s['lookahead_m']:.2f} m", TEXT),
            ("reflex checks", "plan path" if s["ahead"] is not None else "steer arc", TEXT),
            ("map mismatch", "-" if s["mismatch"] is None else f"{100 * s['mismatch']:.0f} %", TEXT),
            ("section", "-" if sec is None else f"{sec} of {len(s['grip'])}", TEXT),
            ("multiplier", "-" if sec is None else f"{s['mult'][sec]:.2f}", TEXT),
            ("ceiling", "none" if ceil is None or not np.isfinite(ceil) else f"{ceil:.1f} m/s^2", TEXT),
            ("replans", f"{s['replans']}", TEXT),
            ("drops / swaps", f"{s['counts'].get('drop', 0)} / {s['counts'].get('swap', 0)}", TEXT),
            ("sim speed", f"{s['sim_speed']:.1f}x real", TEXT),
        ]
        h = min(26 + 16 * ((len(rows) + 1) // 2), bottom - y)
        if h < 40:
            return
        top = y
        self.card(y, h, "DEBUG", "D hides this")
        half = (self.w - 16) // 2
        for i, (k, v, colour) in enumerate(rows):
            col, ly = self.x0 + 18 + (i % 2) * half, top + 22 + 16 * (i // 2)
            if ly + 14 > top + h:
                break
            self.text(k, col, ly, FAINT, self.small)
            self.text(v, col + 92, ly, colour, self.small)
