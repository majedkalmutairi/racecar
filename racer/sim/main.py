import argparse
import time
from pathlib import Path

import pygame
import yaml

from racer.laps import LapCounter, open_lap_log
from racer.modes.explore import ExploreController
from racer.sim.sim_car import SimCar

CONFIG_PATH = Path(__file__).resolve().parent.parent / "config.yaml"
REPO_ROOT = Path(__file__).resolve().parents[2]


def parse_args():
    p = argparse.ArgumentParser(description="2D racing simulator")
    p.add_argument("--track", default=None, help="overrides sim.track in config.yaml")
    p.add_argument("--headless", action="store_true", help="no window, fixed dt, as fast as the CPU allows (implies --auto)")
    p.add_argument("--laps", type=int, default=0, help="stop after N laps (0 = run forever)")
    p.add_argument("--auto", action="store_true", help="start in explore mode")
    p.add_argument("--fresh", action="store_true", help="delete logs/laps.csv before starting")
    return p.parse_args()


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
    args = parse_args()
    cfg = load_config(CONFIG_PATH)
    if args.track:
        cfg["sim"]["track"] = args.track
    track_name = cfg["sim"]["track"]
    car = SimCar(cfg)

    pygame.init()
    if not args.headless:
        screen = pygame.display.set_mode(car.size_px)
        pygame.display.set_caption(f"racer - {track_name}")
        clock = pygame.time.Clock()
        font = pygame.font.SysFont(None, 32)

    ctrl = ExploreController(cfg)
    laps = LapCounter(car.start_pose, cfg)
    csv_path = REPO_ROOT / cfg["laps"]["csv_path"]
    if args.fresh and csv_path.exists():
        csv_path.unlink()
    log_file, log = open_lap_log(csv_path)

    autonomous = args.auto or args.headless   # headless has no keyboard
    sim_t = 0.0             # simulated time: the sum of dt, not the wall clock
    crashes_at_lap_start = 0
    dt = 1 / cfg["sim"]["fps"] if args.headless else 0.0
    real_start = time.perf_counter()
    running = True

    while running:
        if not args.headless:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN and event.key == pygame.K_a:
                    autonomous = not autonomous
                elif event.type == pygame.KEYDOWN and event.key == pygame.K_r:
                    car.reset(clear_crashes=True)
                    laps.reset(sim_t)
                    crashes_at_lap_start = 0

        auto_speed, auto_steer = ctrl.step(car.get_scan())   # always run, so the readout is live
        if autonomous:
            speed, steer = auto_speed, auto_steer
        else:
            speed, steer = keyboard(pygame.key.get_pressed(), cfg)
        car.set_drive(speed, steer)
        if car.update(dt):
            laps.restart(sim_t)      # car was put back at the start: the running lap doesn't count
        sim_t += dt

        lap_time = laps.update(car.get_pose(), sim_t)
        if lap_time is not None:
            log.writerow({
                "track": track_name,
                "lap_number": laps.count,
                "lap_time_s": round(lap_time, 2),
                "crashes": car.crashes - crashes_at_lap_start,
                "mode": "explore" if autonomous else "manual",
            })
            log_file.flush()
            crashes_at_lap_start = car.crashes
            if args.laps and laps.count >= args.laps:
                running = False

        if not args.headless:
            car.draw(screen)
            left, right, front, ctrl_steer = ctrl.last
            mode = "explore (A)" if autonomous else "manual (A)"
            last = f"{laps.last_lap_time:.1f} s" if laps.last_lap_time is not None else "-"
            lines = [
                mode,
                f"L: {left:.2f}  R: {right:.2f}  F: {front:.2f}  steer: {ctrl_steer:.2f}",
                f"lap {laps.count + 1}  t: {laps.current_lap_s(sim_t):.1f} s  last: {last}",
            ]
            for i, line in enumerate(lines):
                text = font.render(line, True, (255, 255, 255), (0, 0, 0))
                screen.blit(text, (10, 40 + 30 * i))
            pygame.display.flip()
            dt = min(clock.tick(cfg["sim"]["fps"]) / 1000, cfg["sim"]["max_dt"])

    log_file.close()
    pygame.quit()
    print(f"{track_name}: {laps.count} laps, {car.crashes} crashes, "
          f"{sim_t:.1f} s simulated in {time.perf_counter() - real_start:.1f} s real")


if __name__ == "__main__":
    main()
