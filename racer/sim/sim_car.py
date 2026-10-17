import pygame

from racer.interface import CarInterface
from racer.sim.car import Car
from racer.sim.lidar import Lidar
from racer.sim.track import Track

CRASH_COLOUR = (255, 220, 0)
TEXT_COLOUR = (255, 255, 255)
TEXT_BG = (0, 0, 0)


class SimCar(CarInterface):
    def __init__(self, cfg):
        sim = cfg["sim"]
        self.track = Track(sim["track"], sim["px_per_m"])
        self.car = Car(cfg, *self.track.start_pose)
        self.lidar = Lidar(cfg)
        self.draw_beams = cfg["lidar"]["draw_beams"]
        self.crash_reset_s = sim["crash_reset_s"]
        self.speed_kp = cfg["car"]["speed_kp"]
        self.target_speed = 0.0
        self.target_steer = 0.0
        self.crashes = 0
        self.crash_timer = 0.0
        self._scan = self.lidar.scan(self.track, *self.car.pose())
        self._font = None

    # --- CarInterface -------------------------------------------------
    def set_drive(self, speed, steer):
        self.target_speed = speed
        self.target_steer = steer

    def get_scan(self):
        return self._scan

    def get_pose(self):
        return self.car.pose()

    def get_speed(self):
        return self.car.v

    # --- sim only: main.py calls these, racing logic never does --------
    @property
    def start_pose(self):
        return self.track.start_pose

    @property
    def size_px(self):
        return self.track.w_px, self.track.h_px

    def reset(self):
        self.car.x, self.car.y, self.car.yaw = self.track.start_pose
        self.car.v = 0.0
        self.crash_timer = 0.0
        self.target_speed = 0.0
        self.target_steer = 0.0

    def update(self, dt):
        """Advance the sim by dt. Returns True on the frame the car is put back at the start."""
        was_reset = False
        if self.crash_timer > 0:
            self.crash_timer -= dt
            if self.crash_timer <= 0:
                self.reset()
                was_reset = True
        else:
            accel = self.speed_kp * (self.target_speed - self.car.v)
            self.car.step(accel, self.target_steer, dt)
            if any(self.track.is_wall(cx, cy) for cx, cy in self.car.corners()):
                self.crashes += 1
                self.crash_timer = self.crash_reset_s
                self.car.v = 0.0

        self._scan = self.lidar.scan(self.track, *self.car.pose())
        return was_reset

    def draw(self, screen):
        if self._font is None:
            self._font = pygame.font.SysFont(None, 32)
        x, y, yaw = self.car.pose()

        self.track.draw(screen)
        if self.draw_beams:
            self.lidar.draw(screen, self.track, x, y, yaw, self._scan)
        if self.crash_timer > 0 and int(self.crash_timer * 8) % 2 == 0:
            self.car.draw(screen, self.track, CRASH_COLOUR)
        else:
            self.car.draw(screen, self.track)
        text = self._font.render(f"crashes: {self.crashes}", True, TEXT_COLOUR, TEXT_BG)
        screen.blit(text, (10, 10))
