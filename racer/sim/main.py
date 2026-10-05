from pathlib import Path

import pygame
import yaml

CONFIG_PATH = Path(__file__).resolve().parent.parent / "config.yaml"


def load_config(path):
    with open(path) as f:
        return yaml.safe_load(f)


class Car:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.heading = 0.0
        self.speed = 300
        self.w = 40
        self.h = 20

    def update(self, keys, dt, screen_w, screen_h):
        if keys[pygame.K_LEFT]:
            self.x -= self.speed * dt
        if keys[pygame.K_RIGHT]:
            self.x += self.speed * dt
        if keys[pygame.K_UP]:
            self.y -= self.speed * dt
        if keys[pygame.K_DOWN]:
            self.y += self.speed * dt

        self.x %= screen_w
        self.y %= screen_h

    def draw(self, screen):
        rect = pygame.Rect(0, 0, self.w, self.h)
        rect.center = (int(self.x), int(self.y))
        pygame.draw.rect(screen, (220, 60, 60), rect)


def main():
    cfg = load_config(CONFIG_PATH)["sim"]
    w, h = cfg["window_w"], cfg["window_h"]

    pygame.init()
    screen = pygame.display.set_mode((w, h))
    pygame.display.set_caption("racer")
    clock = pygame.time.Clock()

    car = Car(w / 2, h / 2)
    dt = 0.0
    running = True

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        keys = pygame.key.get_pressed()
        car.update(keys, dt, w, h)

        screen.fill((30, 30, 30))
        car.draw(screen)
        pygame.display.flip()

        dt = clock.tick(cfg["fps"]) / 1000

    pygame.quit()


if __name__ == "__main__":
    main()
