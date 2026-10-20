import math

from racer.laps import LapCounter

CFG = {"laps": {"gate_half_m": 1.6, "min_lap_s": 3, "min_lap_m": 5}}


def drive_circle(laps, radius=3.0, offset=0.0, reverse=False, steps=400):
    """Drive round a circle that passes through the start (0, 0) heading +x; offset moves the circle sideways."""
    t, done = 0.0, []
    for k in range(1, steps * laps + 2):                 # one step past the end, so the last crossing happens
        a = 2 * math.pi * k / steps * (-1 if reverse else 1)
        pose = (radius * math.sin(a), radius - radius * math.cos(a) + offset, a)
        t += 0.05
        lap = counter.update(pose, t)
        if lap is not None:
            done.append(lap)
    return done


def test_a_lap_crossing_the_gate_counts():
    global counter
    counter = LapCounter((0.0, 0.0, 0.0), CFG, 0.0)
    assert len(drive_circle(3)) == 3


def test_a_wide_line_still_crosses_the_gate():
    global counter
    counter = LapCounter((0.0, 0.0, 0.0), CFG, 0.0)
    assert len(drive_circle(2, offset=1.2)) == 2         # 1.2 m beside the start point: the old 0.5 m circle missed it


def test_driving_backwards_through_it_does_not_count():
    global counter
    counter = LapCounter((0.0, 0.0, 0.0), CFG, 0.0)
    assert drive_circle(2, reverse=True) == []


def test_far_beside_the_gate_does_not_count():
    global counter
    counter = LapCounter((0.0, 0.0, 0.0), CFG, 0.0)
    assert drive_circle(2, offset=3.0) == []
