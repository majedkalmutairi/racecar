import argparse
import math
import sys
import time
from pathlib import Path

import pygame
import yaml

from racer.dashboard.overlay import Dashboard
from racer.modes.manager import Mode, ModeManager
from racer.sim.draw import MapOverlay, draw_plan
from racer.sim.geometry import car_corners
from racer.sim.gif import GifRecorder
from racer.sim.sim_car import SimCar
from racer.sim.track import Track, track_names
from racer.sim.trackgen import MODES, generate, parse_modes, save

CONFIG_PATH = Path(__file__).resolve().parent.parent / "config.yaml"
REPO_ROOT = Path(__file__).resolve().parents[2]


def parse_args():
    p = argparse.ArgumentParser(description="2D racing simulator")
    p.add_argument("--track", default=None, help="overrides sim.track in config.yaml")
    p.add_argument("--headless", action="store_true", help="no window, fixed dt, as fast as the CPU allows")
    p.add_argument("--laps", type=int, default=0, help="stop after N laps (0 = run forever)")
    p.add_argument("--max-s", type=float, default=0, help="stop after this much simulated time (0 = no limit)")
    p.add_argument("--seed", type=int, default=None, help="lidar noise seed, overrides sim.seed")
    p.add_argument("--fresh", action="store_true", help="delete laps.csv, events.csv and this track's saved plan first")
    p.add_argument("--log-dir", default=None, help="write all logs and plan files here instead of logs/")
    p.add_argument("--manual", action="store_true", help="start with arrow-key driving instead of the mode manager")
    p.add_argument("--obstacles", nargs="?", const="all", default=None,
                   help="named track: the boxes under obstacles.<track> in config.yaml; "
                        "--random: a comma list of known, drop, slalom (no list = all three)")
    p.add_argument("--random", action="store_true", help="generate a new track and drive it")
    p.add_argument("--gen-seed", type=int, default=0, help="the same seed always gives the same track")
    p.add_argument("--difficulty", default="medium", choices=["easy", "medium", "hard"])
    p.add_argument("--size", default="medium", choices=["small", "medium", "large"])
    p.add_argument("--size-m", type=float, nargs=2, metavar=("W", "H"), help="track area in metres, at most 10 x 10")
    p.add_argument("--drop-box", type=float, nargs=4, action="append", metavar=("X", "Y", "W", "H"),
                   help="a box in metres, repeat for more...")
    p.add_argument("--drop-at-lap", type=int, default=3, help="...dropped when this many laps are done")
    p.add_argument("--swap-track", default=None, help="replace the track with this one...")
    p.add_argument("--swap-at-lap", type=int, default=4, help="...when this many laps are done")
    p.add_argument("--gif", default=None, help="record the window to this GIF...")
    p.add_argument("--gif-from-lap", type=int, default=1, help="...starting when this many laps are done")
    p.add_argument("--gif-s", type=float, default=10.0, help="...for this many simulated seconds")
    return p.parse_args()


def load_config(path):
    with open(path) as f:
        return yaml.safe_load(f)


def keyboard(keys, cfg):
    """Arrow keys -> (target speed, steer). No key = target 0, so the car brakes to a stop."""
    speed = 0.0
    if keys[pygame.K_UP]:
        speed = cfg["car"]["max_speed_mps"]
    elif keys[pygame.K_DOWN]:
        speed = -cfg["car"]["max_reverse_mps"]
    steer = 0.0
    if keys[pygame.K_LEFT]:
        steer = cfg["car"]["max_steer_rad"]
    elif keys[pygame.K_RIGHT]:
        steer = -cfg["car"]["max_steer_rad"]
    return speed, steer


def drop_when_clear(drops, car, manager, t, clear_m):
    """Drop every box whose lap has come, unless the car is so close it would land on it. Returns the rest."""
    x, y, _ = car.get_pose()
    left = []
    for box, at_lap in drops:
        bx, by, bw, bh = box
        if manager.lap_total >= at_lap and math.hypot(bx + bw / 2 - x, by + bh / 2 - y) > clear_m:
            car.track.add_box(*box)
            manager.note("drop", t)
            print(f"[sim] box dropped at {box[:2]}")
        else:
            left.append((box, at_lap))
    return left


def main():
    args = parse_args()
    cfg = load_config(CONFIG_PATH)
    if args.track:
        cfg["sim"]["track"] = args.track
    if args.seed is not None:
        cfg["sim"]["seed"] = args.seed
    if args.random:
        modes = MODES if args.obstacles == "all" else parse_modes(args.obstacles)
        layout, _ = generate(args.gen_seed, args.difficulty, args.size, modes, cfg, args.size_m)
        save(layout)
        cfg["sim"]["track"] = layout.name
        args.fresh = True                                       # same name, maybe different track: start clean
        print(f"[gen] {layout.name}: {layout.info['lap_m']} m, {layout.width_m} m wide, boxes {layout.info['boxes']}")
    track_name = cfg["sim"]["track"]
    if args.log_dir:
        log_dir = Path(args.log_dir).resolve()
        cfg["laps"]["csv_path"] = str(log_dir / "laps.csv")
        cfg["safety"]["events_csv"] = str(log_dir / "events.csv")
        cfg["mapping"]["map_dir"] = str(log_dir)
    if args.fresh:
        for rel in (cfg["laps"]["csv_path"], cfg["safety"]["events_csv"]):
            (REPO_ROOT / rel).unlink(missing_ok=True)
        for f in (REPO_ROOT / cfg["mapping"]["map_dir"]).glob(f"{track_name}_*.np*"):
            f.unlink()

    car = SimCar(cfg)
    if args.obstacles and not args.random:
        for box in cfg["obstacles"].get(track_name, []):
            car.track.add_box(*box)
    for box in car.track.boxes_known:
        car.track.add_box(*box)
    drops = list(car.track.boxes_drop)                          # (box, at_lap), dropped once the car is clear
    manager = ModeManager(cfg, car.start_pose, track_name, REPO_ROOT)

    pygame.init()
    headless = args.headless
    if not headless:
        w, h = car.size_px
        screen = pygame.display.set_mode((w + cfg["dashboard"]["panel_w"], h))
        clock = pygame.time.Clock()
        dash = Dashboard(cfg, pygame.font.SysFont(None, 22), pygame.font.SysFont(None, 40))
        map_overlay = MapOverlay(car.track, cfg["mapping"]["min_hits"])
        gif = GifRecorder(args.gif, args.gif_s, args.gif_from_lap) if args.gif else None

    pending_track = None
    if headless:
        screen = map_overlay = None

    def swap_track_when_free():
        """Replace the track on the first frame the car has room around it on the new one."""
        nonlocal pending_track, map_overlay, screen
        if pending_track is None:
            return
        x, y, yaw = car.get_pose()
        m = cfg["sim"]["swap_margin_m"]
        around = car_corners(x, y, yaw, car.car.length + 2 * m, car.car.width + 2 * m, car.car.L / 2)
        if any(pending_track.is_wall(px, py) for px, py in around):
            return
        car.swap_track(pending_track)
        for box in car.track.boxes_known:
            car.track.add_box(*box)
        manager.track_name = car.track.name
        manager.note("swap", sim_t)
        if not headless:
            w, h = car.size_px
            if screen.get_size() != (w + cfg["dashboard"]["panel_w"], h):
                screen = pygame.display.set_mode((w + cfg["dashboard"]["panel_w"], h))
            map_overlay = MapOverlay(car.track, cfg["mapping"]["min_hits"])
        print(f"[sim] track swapped to {car.track.name}")
        pending_track = None

    manual = args.manual and not headless
    paused = False
    sim_t = 0.0
    dt = 1 / cfg["sim"]["fps"] if headless else 0.0
    swapped = dropped = False
    real_start = time.perf_counter()
    running = True

    while running:
        if not headless:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_r:
                        car.reset(clear_crashes=True)
                        manager.reset(car.get_pose(), sim_t)
                    elif event.key == pygame.K_SPACE:
                        paused = not paused
                    elif event.key == pygame.K_n:
                        names = track_names()
                        pending_track = Track(names[(names.index(car.track.name) + 1) % len(names)], cfg["sim"]["px_per_m"])
                    elif event.key == pygame.K_a:
                        manual = not manual
            pygame.display.set_caption(f"racer - {car.track.name}")

        if not paused:
            if manual:
                speed, steer = keyboard(pygame.key.get_pressed(), cfg)
            else:
                speed, steer = manager.step(car, sim_t)
            car.set_drive(speed, steer)
            if car.update(dt):
                manager.car_was_reset(car.get_pose(), sim_t)
            sim_t += dt

            if args.swap_track and not swapped and manager.lap_total >= args.swap_at_lap:
                pending_track = Track(args.swap_track, cfg["sim"]["px_per_m"])
                swapped = True
            swap_track_when_free()
            if args.drop_box and not dropped and manager.lap_total >= args.drop_at_lap:
                for box in args.drop_box:
                    car.track.add_box(*box)
                    manager.note("drop", sim_t)
                    print(f"[sim] box dropped at {box[:2]}")
                dropped = True
            drops = drop_when_clear(drops, car, manager, sim_t, cfg["trackgen"]["drop_clear_m"])
            if args.laps and manager.lap_total >= args.laps:
                running = False
            if args.max_s and sim_t >= args.max_s:
                running = False
            if headless and manager.recovery.gave_up:
                running = False

        if not headless:
            state = manager.status(sim_t)
            state["speed"] = car.get_speed()
            state["paused"] = paused
            car.draw(screen)
            map_overlay.draw(screen, manager.grid, sim_t, manager.mode is Mode.EXPLORE)
            draw_plan(screen, car.track, state)
            dash.push(car.get_speed(), state["v_target"])
            dash.draw(screen, state)
            if gif:
                gif.capture(screen, sim_t, manager.lap_total)
            pygame.display.flip()
            dt = min(clock.tick(cfg["sim"]["fps"]) / 1000, cfg["sim"]["max_dt"])

    if not headless and gif:
        gif.save()
    manager.close()
    pygame.quit()
    print(f"{car.track.name}: {manager.lap_total} laps, {car.crashes} crashes, "
          f"{sim_t:.1f} s simulated in {time.perf_counter() - real_start:.1f} s real")
    if manager.recovery.gave_up:
        print("STUCK")
        sys.exit(3)


if __name__ == "__main__":
    main()
