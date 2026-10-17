import math

import pygame

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
        self.v += accel * dt

        self.v = clamp(self.v, 0.0, self.max_speed)

    def pose(self):
        return self.x, self.y, self.yaw

    def draw(self, screen, track):
        hl = self.length / 2
        hw = self.width / 2
        centre = self.L / 2
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)

        def to_px(fx, fy):
            wx = self.x + fx * cos_yaw - fy * sin_yaw
            wy = self.y + fx * sin_yaw + fy * cos_yaw
            return track.world_to_px(wx, wy)

        corners = [(hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw)]
        points = [to_px(centre + fx, fy) for fx, fy in corners]
        pygame.draw.polygon(screen, BODY_COLOUR, points)
        pygame.draw.line(screen, HEADING_COLOUR, to_px(centre, 0), to_px(centre + hl, 0), 3)
