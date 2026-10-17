import math


def rotate_point(px, py, yaw):
    c = math.cos(yaw)
    s = math.sin(yaw)
    return px * c - py * s, px * s + py * c


def car_corners(x, y, yaw, length, width, offset=0.0):
    hl = length / 2
    hw = width / 2
    local = [(hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw)]
    corners = []
    for lx, ly in local:
        rx, ry = rotate_point(lx + offset, ly, yaw)
        corners.append((x + rx, y + ry))
    return corners
