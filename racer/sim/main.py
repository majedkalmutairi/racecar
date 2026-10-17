from pathlib import Path

import pygame
import yaml

from racer.sim.car import Car
from racer.sim.lidar import Lidar
from racer.sim.track import Track

CONFIG_PATH = Path(__file__).resolve().parent.parent / "config.yaml"
CRASH_COLOUR = (255, 220, 0)


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

    pygame.init()
    screen = pygame.display.set_mode((track.w_px, track.h_px))
    pygame.display.set_caption("racer")
    clock = pygame.time.Clock()

    font = pygame.font.SysFont(None, 32)

    car = Car(cfg, *track.start_pose)
    lidar = Lidar(cfg)
    front = lidar.n_beams // 2
    crashes = 0
    crash_timer = 0.0
    dt = 0.0
    last_fps_print = 0
    running = True

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        if crash_timer > 0:
            crash_timer -= dt
            if crash_timer <= 0:
                car.x, car.y, car.yaw = track.start_pose
                car.v = 0.0
                crash_timer = 0.0
        else:
            keys = pygame.key.get_pressed()
            drive(car, keys, dt)
            if any(track.is_wall(cx, cy) for cx, cy in car.corners()):
                crashes += 1
                crash_timer = sim["crash_reset_s"]
                car.v = 0.0

        ranges = lidar.scan(track, car.x, car.y, car.yaw)

        track.draw(screen)
        if cfg["lidar"]["draw_beams"]:
            lidar.draw(screen, track, car.x, car.y, car.yaw, ranges)
        if crash_timer > 0 and int(crash_timer * 8) % 2 == 0:
            car.draw(screen, track, CRASH_COLOUR)
        else:
            car.draw(screen, track)
        text = font.render(f"crashes: {crashes}", True, (255, 255, 255), (0, 0, 0))
        screen.blit(text, (10, 10))
        text = font.render(f"front: {ranges[front]:.2f} m", True, (255, 255, 255), (0, 0, 0))
        screen.blit(text, (10, 40))
        pygame.display.flip()

        dt = min(clock.tick(sim["fps"]) / 1000, sim["max_dt"])
        now = pygame.time.get_ticks()
        if now - last_fps_print >= 1000:
            print(f"fps: {clock.get_fps():.1f}", flush=True)
            last_fps_print = now

    pygame.quit()


if __name__ == "__main__":
    main()
