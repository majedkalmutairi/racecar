import numpy as np

from racer.learning.sections import SectionLearner

CFG = {
    "learning": {"n_sections": 4, "start_mult": 0.6, "up": 0.1, "down": 0.2,
                 "min_mult": 0.2, "max_mult": 1.2, "err_ok_m": 0.25},
}


def test_learner_grades_sections(tmp_path):
    lr = SectionLearner(200, CFG, tmp_path / "mult.npy")
    for idx in range(200):
        lr.observe(idx, 0.5 if idx < 50 else 0.0, False, False)
    lr.observe(120, 0.0, True, False)
    clean = lr.end_lap()
    assert list(clean) == [False, True, False, True]
    assert np.allclose(lr.mult, [0.48, 0.66, 0.48, 0.66])
    assert np.allclose(np.load(tmp_path / "mult.npy"), lr.mult)

def test_learner_remembers_from_disk(tmp_path):
    np.save(tmp_path / "mult.npy", np.array([1.0, 1.1, 1.2, 0.9]))
    lr = SectionLearner(200, CFG, tmp_path / "mult.npy")
    assert lr.mult[1] == 1.1
    assert len(lr.multiplier_per_point()) == 200


NOSE = 0.33 / 2 + 0.55 / 2


def beam(deg):
    return int(np.argmin(np.abs(ANGLES - np.radians(deg))))
