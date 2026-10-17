from pathlib import Path

import pygame
import yaml

from racer.sim.car import Car
from racer.sim.track import Track

CONFIG_PATH = Path(__file__).resolve().parent.parent / "config.yaml"


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
    track = Track(sim["track"], sim["px_per_m"])
    world_w = track.w_px / track.px_per_m
    world_h = track.h_px / track.px_per_m

    pygame.init()
    screen = pygame.display.set_mode((track.w_px, track.h_px))
    pygame.display.set_caption("racer")
    clock = pygame.time.Clock()

    car = Car(cfg, *track.start_pose)
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

        track.draw(screen)
        car.draw(screen, track)
        pygame.display.flip()

        dt = min(clock.tick(sim["fps"]) / 1000, sim["max_dt"])

    pygame.quit()


if __name__ == "__main__":
    main()
