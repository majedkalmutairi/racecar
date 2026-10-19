import copy
import math
from pathlib import Path

import numpy as np
import pytest
import yaml

from racer.loops import resample
from racer.sim.trackcheck import check_layout
from racer.sim.trackgen import MODES, Layout, fillets, generate, render, rounded

CFG = yaml.safe_load(open(Path(__file__).resolve().parent.parent / "racer" / "config.yaml"))
SEEDS = range(15)


def layout_from(corners, r, width=1.3, size=(10, 7), difficulty="medium"):
    """A hand-made layout, the same way the generator builds one."""
    xy = resample(np.array(rounded(corners, r, step_m=0.02, max_frac=0.5)), CFG["plan"]["ds_m"])
    t = xy[1] - xy[-1]
    return Layout("test", size, width, xy, [f[1] for f in fillets(corners, r, 0.5)],
                  (xy[0, 0], xy[0, 1], math.atan2(t[1], t[0])), info={"difficulty": difficulty, "lap_m": 0})


GOOD = [(1.5, 1.5), (8.5, 1.5), (8.5, 5.5), (1.5, 5.5)]


def test_good_rectangle_passes():
    assert check_layout(layout_from(GOOD, 1.5), CFG) == []


def test_same_seed_same_track():
    a, _ = generate(3, "hard", "medium", MODES, CFG)
    b, _ = generate(3, "hard", "medium", MODES, CFG)
    assert np.array_equal(render(a, a.all_boxes()), render(b, b.all_boxes()))
    assert a.start_pose == b.start_pose and a.boxes_drop == b.boxes_drop


@pytest.mark.parametrize("difficulty", ["easy", "medium", "hard"])
def test_generated_tracks_pass_every_check(difficulty):
    for seed in SEEDS:
        layout, _ = generate(seed, difficulty, "medium", MODES, CFG)
        assert check_layout(layout, CFG) == []
        assert min(layout.radii) >= CFG["trackgen"]["min_radius_m"] - 1e-6


def test_tight_corner_is_rejected():
    assert "corner_radius" in check_layout(layout_from(GOOD, 0.6), CFG)


def test_merged_passes_are_rejected():
    pocket = [(1.5, 1.5), (8.5, 1.5), (8.5, 5.5), (5.6, 5.5), (5.6, 2.8), (4.4, 2.8), (4.4, 5.5), (1.5, 5.5)]
    assert "passes_too_close" in check_layout(layout_from(pocket, 0.5), CFG)


def test_box_in_a_narrow_gap_is_rejected():
    layout = layout_from(GOOD, 1.5)
    layout.boxes_known = [[5.0, 0.85 + 0.5, 0.3, 0.3]]     # 0.5 m from the outer wall, about 0.5 m to the inner
    assert "box_gap" in check_layout(layout, CFG)


def test_start_near_another_pass_is_rejected():
    cfg = copy.deepcopy(CFG)
    cfg["trackgen"]["start_clear_m"] = 3.0                   # wider than the gap between the two straights
    assert "start_clear" in check_layout(layout_from(GOOD, 1.5), cfg)


def test_dropped_boxes_over_the_mismatch_budget_are_rejected():
    cfg = copy.deepcopy(CFG)
    cfg["trackgen"]["drop_mismatch_max"] = 0.01
    layout = layout_from(GOOD, 1.5)
    layout.boxes_drop = [([5.0, 0.85, 0.3, 0.3], 3)]
    assert "drop_mismatch" in check_layout(layout, cfg)


def test_hard_does_not_fit_on_a_small_area():
    with pytest.raises(RuntimeError):
        generate(0, "hard", "small", (), CFG)
