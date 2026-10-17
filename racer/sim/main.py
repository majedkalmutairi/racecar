from pathlib import Path

import pygame
import yaml

from racer.modes.explore import ExploreController
from racer.sim.sim_car import SimCar

CONFIG_PATH = Path(__file__).resolve().parent.parent / "config.yaml"


def load_config(path):
    with open(path) as f:
        return yaml.safe_load(f)


def keyboard(keys, cfg):
    """Arrow keys -> (target speed, steer). No key = target 0, so the car brakes to a stop."""
    speed = cfg["car"]["max_speed_mps"] if keys[pygame.K_UP] else 0.0

    steer = 0.0
    if keys[pygame.K_LEFT]:
        steer = cfg["car"]["max_steer_rad"]
    elif keys[pygame.K_RIGHT]:
        steer = -cfg["car"]["max_steer_rad"]
    return speed, steer


def main():
    cfg = load_config(CONFIG_PATH)
    car = SimCar(cfg)

    pygame.init()
    screen = pygame.display.set_mode(car.size_px)
    pygame.display.set_caption("racer")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont(None, 32)

    ctrl = ExploreController(cfg)
    autonomous = False
    dt = 0.0
    running = True

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_a:
                autonomous = not autonomous

        auto_speed, auto_steer = ctrl.step(car.get_scan())   # always run, so the readout is live
        if autonomous:
            speed, steer = auto_speed, auto_steer
        else:
            speed, steer = keyboard(pygame.key.get_pressed(), cfg)
        car.set_drive(speed, steer)
        car.update(dt)

        car.draw(screen)
        left, right, front, ctrl_steer = ctrl.last
        mode = "explore (A)" if autonomous else "manual (A)"
        lines = [mode, f"L: {left:.2f}  R: {right:.2f}  F: {front:.2f}  steer: {ctrl_steer:.2f}"]
        for i, line in enumerate(lines):
            text = font.render(line, True, (255, 255, 255), (0, 0, 0))
            screen.blit(text, (10, 40 + 30 * i))
        pygame.display.flip()

        dt = min(clock.tick(cfg["sim"]["fps"]) / 1000, cfg["sim"]["max_dt"])

    pygame.quit()


if __name__ == "__main__":
    main()
