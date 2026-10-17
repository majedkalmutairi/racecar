from pathlib import Path

import pygame
import yaml

from racer.sim.car import Car

CONFIG_PATH = Path(__file__).resolve().parent.parent / "config.yaml"
BACKGROUND = (30, 30, 30)


def load_config(path):
    with open(path) as f:
        return yaml.safe_load(f)


def drive(car, keys, dt):
    accel = 0.0
    steer = 0.0

    if keys[pygame.K_UP]:
        accel = car.max_accel
    elif keys[pygame.K_DOWN]:
        accel = -car.max_brake

    if keys[pygame.K_LEFT]:
        steer = car.max_steer
    elif keys[pygame.K_RIGHT]:
        steer = -car.max_steer

    car.step(accel, steer, dt)


def main():
    cfg = load_config(CONFIG_PATH)
    sim = cfg["sim"]
    w, h = sim["window_w"], sim["window_h"]
    px_per_m = sim["px_per_m"]
    world_w, world_h = w / px_per_m, h / px_per_m
    origin_px = (0, h)

    pygame.init()
    screen = pygame.display.set_mode((w, h))
    pygame.display.set_caption("racer")
    clock = pygame.time.Clock()

    car = Car(cfg, world_w / 2, world_h / 2)
    dt = 0.0
    running = True

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        keys = pygame.key.get_pressed()
        drive(car, keys, dt)
        car.x %= world_w
        car.y %= world_h

        screen.fill(BACKGROUND)
        car.draw(screen, px_per_m, origin_px)
        pygame.display.flip()

        dt = min(clock.tick(sim["fps"]) / 1000, sim["max_dt"])

    pygame.quit()


if __name__ == "__main__":
    main()
