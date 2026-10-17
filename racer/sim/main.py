from pathlib import Path

import pygame
import yaml

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

    front = len(car.get_scan()) // 2
    dt = 0.0
    running = True

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        speed, steer = keyboard(pygame.key.get_pressed(), cfg)
        car.set_drive(speed, steer)
        car.update(dt)

        car.draw(screen)
        text = font.render(f"front: {car.get_scan()[front]:.2f} m", True, (255, 255, 255), (0, 0, 0))
        screen.blit(text, (10, 40))
        pygame.display.flip()

        dt = min(clock.tick(cfg["sim"]["fps"]) / 1000, cfg["sim"]["max_dt"])

    pygame.quit()


if __name__ == "__main__":
    main()
