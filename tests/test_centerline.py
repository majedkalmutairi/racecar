import numpy as np

from racer.mapping.centerline import crosses, remove_loops, smooth_offsets
from racer.planning.racing_line import first_free


def circle(radius, n):
    t = np.linspace(0, 2 * np.pi, n, endpoint=False)
    return np.c_[radius * np.cos(t), radius * np.sin(t)]


def self_crossings(xy, window):
    a, b = xy, np.roll(xy, -1, axis=0)
    k = np.arange(2, window)
    j = (np.arange(len(xy))[:, None] + k[None, :]) % len(xy)
    return int(crosses(a[:, None], b[:, None], a[j], b[j]).sum())


def test_a_small_knot_is_cut_out():
    xy = circle(3.0, 200)
    knot = xy[50] + 0.2 * circle(1.0, 12) + [0.2, 0.0]       # a little curl that crosses the line twice
    xy = np.vstack([xy[:50], knot, xy[50:]])
    assert self_crossings(xy, 40) > 0
    out = remove_loops(xy, 40)
    assert self_crossings(out, 40) == 0
    assert len(out) < len(xy)


def test_a_clean_loop_is_left_alone():
    xy = circle(3.0, 200)
    assert np.array_equal(remove_loops(xy, 40), xy)


def test_room_search_never_crosses_a_wall():
    ok = np.array([0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0], bool)
    wall = np.zeros(14, bool)
    wall[9] = True
    assert first_free(ok, wall, 5, 1) is None                  # free room beyond the wall at 9 does not count
    assert first_free(ok, wall, 5, -1) == 1


def test_room_search_may_leave_the_box_it_starts_in():
    ok = np.array([0, 0, 0, 0, 0, 0, 0, 0, 1, 0], bool)
    wall = np.zeros(10, bool)
    wall[4:7] = True                                           # the point sits inside a box from 4 to 6
    assert first_free(ok, wall, 5, 1) == 8


def test_snapped_offsets_ramp_instead_of_stepping():
    target = np.r_[np.zeros(50), np.full(50, 0.5)]
    off = smooth_offsets(target, np.full(100, -1.0), np.full(100, 1.0), 5.0)
    assert np.abs(np.diff(off)).max() < 0.2
    capped = smooth_offsets(target, np.full(100, -1.0), np.r_[np.full(50, 1.0), np.full(50, 0.2)], 5.0)
    assert capped[50:].max() <= 0.2 + 1e-6                     # never past the wall that bounds it
