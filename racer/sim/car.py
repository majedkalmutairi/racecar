import math

import pygame

from racer.sim.geometry import car_corners, rotate_point

BODY_COLOUR = (220, 60, 60)
HEADING_COLOUR = (255, 255, 255)


def clamp(value, low, high):
    return max(low, min(high, value))


class Car:
    def __init__(self, cfg, x=0.0, y=0.0, yaw=0.0):
        c = cfg["car"]
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = 0.0
        self.L = c["wheelbase_m"]
        self.max_speed = c["max_speed_mps"]
        self.max_steer = c["max_steer_rad"]
        self.max_accel = c["max_accel"]
        self.max_brake = c["max_brake"]
        self.length = c["length_m"]
        self.width = c["width_m"]
        self.friction = c["friction_decel"]

    def step(self, accel_cmd, steer_cmd, dt):
        steer = clamp(steer_cmd, -self.max_steer, self.max_steer)
        accel = clamp(accel_cmd, -self.max_brake, self.max_accel)
        if accel == 0.0:
            accel = -self.friction

        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.v / self.L * math.tan(steer) * dt
        self.yaw = (self.yaw + math.pi) % (2 * math.pi) - math.pi
        self.v += accel * dt

        self.v = clamp(self.v, 0.0, self.max_speed)

    def pose(self):
        return self.x, self.y, self.yaw

    def corners(self):
        return car_corners(self.x, self.y, self.yaw, self.length, self.width, self.L / 2)

    def draw(self, screen, track, colour=BODY_COLOUR):
        points = [track.world_to_px(cx, cy) for cx, cy in self.corners()]
        pygame.draw.polygon(screen, colour, points)

        centre = self.L / 2
        nose = centre + self.length / 2
        cx, cy = rotate_point(centre, 0, self.yaw)
        nx, ny = rotate_point(nose, 0, self.yaw)
        start = track.world_to_px(self.x + cx, self.y + cy)
        end = track.world_to_px(self.x + nx, self.y + ny)
        pygame.draw.line(screen, HEADING_COLOUR, start, end, 3)
