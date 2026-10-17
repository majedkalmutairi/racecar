import pygame
from PIL import Image


class GifRecorder:
    """Grabs the window at a steady rate of simulated time and writes one GIF when done."""

    def __init__(self, path, seconds, from_lap=1, fps=12, scale=0.5):
        self.path = path
        self.seconds = seconds
        self.from_lap = from_lap
        self.fps = fps
        self.scale = scale
        self.frames = []
        self.t0 = None
        self.next_t = 0.0
        self.done = False

    def capture(self, screen, t, lap):
        if self.done or lap < self.from_lap:
            return
        if self.t0 is None:
            self.t0 = t
        if t - self.t0 > self.seconds:
            self.save()
            return
        if t < self.next_t:
            return
        self.next_t = t + 1 / self.fps
        w, h = screen.get_size()
        img = Image.frombytes("RGB", (w, h), pygame.image.tobytes(screen, "RGB"))
        self.frames.append(img.resize((int(w * self.scale), int(h * self.scale)), Image.LANCZOS))

    def save(self):
        if self.done or not self.frames:
            return
        self.frames[0].save(self.path, save_all=True, append_images=self.frames[1:],
                            duration=int(1000 / self.fps), loop=0, optimize=True)
        self.done = True
        print(f"saved {self.path} ({len(self.frames)} frames)")
