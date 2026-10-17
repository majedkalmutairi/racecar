import numpy as np

from racer.mapping.grid import OccupancyGrid, scan_to_world


def test_cell_roundtrip():
    g = OccupancyGrid(20, 0.05)
    row, col = g.world_to_cell(1.23, -4.56)
    x, y = g.cell_to_world(row, col)
    assert abs(x - 1.23) < 0.05 and abs(y + 4.56) < 0.05


def test_origin_is_grid_centre():
    g = OccupancyGrid(20, 0.05)
    assert g.world_to_cell(0.0, 0.0) == (200, 200)


def test_scan_to_world_rotates_with_yaw():
    pts = scan_to_world(np.array([2.0]), (1.0, 1.0, np.pi / 2), np.array([0.0]))
    assert np.allclose(pts[0], [1.0, 3.0])


def test_one_scan_marks_about_one_cell_per_beam():
    g = OccupancyGrid(20, 0.05)
    angles = np.linspace(-2.355, 2.355, 108)
    g.add_scan(np.full(108, 2.0), (0.0, 0.0, 0.0), angles, 12.0)
    n = int(g.occupied(1).sum())
    assert 90 <= n <= 108


def test_beams_at_max_range_are_not_hits():
    g = OccupancyGrid(20, 0.05)
    g.add_scan(np.full(108, 12.0), (0.0, 0.0, 0.0), np.linspace(-2.355, 2.355, 108), 12.0)
    assert g.hits.sum() == 0
