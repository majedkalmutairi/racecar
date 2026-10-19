import numpy as np

from racer.learning.sections import SectionLearner

CFG = {
    "learning": {"n_sections": 4, "start_mult": 0.6, "up": 0.05, "up_max": 0.15, "down": 0.1, "down_crash": 0.25,
                 "below_ceiling": 0.98, "probe_laps": 3, "probe": 0.04,
                 "min_mult": 0.2, "max_mult": 2.5, "err_ok_m": 0.25},
}


def clean_lap(lr):
    for idx in range(200):
        lr.observe(idx, 0.0, False, False)
    return lr.end_lap()


def test_learner_grades_sections(tmp_path):
    lr = SectionLearner(200, CFG, tmp_path / "mult.npy")
    for idx in range(200):
        lr.observe(idx, 0.5 if idx < 50 else 0.0, False, False)
    lr.observe(120, 0.0, True, False)
    clean = lr.end_lap()
    assert list(clean) == [False, True, False, True]
    assert np.allclose(lr.mult, [0.54, 0.63, 0.54, 0.63])
    assert np.allclose(np.load(tmp_path / "mult.npy"), lr.mult)


def test_confidence_makes_the_steps_bigger(tmp_path):
    lr = SectionLearner(200, CFG, tmp_path / "mult.npy")
    before = lr.mult[0]
    steps = []
    for _ in range(4):
        clean_lap(lr)
        steps.append(lr.mult[0] / before - 1)
        before = lr.mult[0]
    assert np.allclose(steps, [0.05, 0.10, 0.15, 0.15])         # grows with clean laps, capped at up_max
    assert lr.confidence()[0] == 1.0


def test_a_problem_sets_a_ceiling_and_resets_confidence(tmp_path):
    lr = SectionLearner(200, CFG, tmp_path / "mult.npy")
    lr.mult[:] = 1.0
    lr.observe(10, 0.0, True, False)                             # reflex in section 0
    lr.end_lap()
    assert np.isclose(lr.mult[0], 0.9) and lr.ceiling[0] == 1.0 and lr.streak[0] == 0
    for _ in range(2):
        clean_lap(lr)
    assert lr.mult[0] <= 0.98 + 1e-9                             # climbs back to just under the ceiling
    clean_lap(lr)                                                # third clean lap in a row: the ceiling moves up
    assert np.isclose(lr.ceiling[0], 1.04)


def test_a_crash_grades_the_partial_lap_at_once(tmp_path):
    lr = SectionLearner(200, CFG, tmp_path / "mult.npy")
    lr.mult[:] = 1.0
    lr.observe(10, 0.0, True, False)                             # trouble earlier this lap, in section 0
    lr.crashed_at(120)                                           # crash in section 2
    assert np.allclose(lr.mult, [0.9, 0.9, 0.75, 1.0])          # section 1 is the approach to the crash
    assert not lr.problem.any()                                  # the restarted lap starts clean


def test_learner_remembers_from_disk(tmp_path):
    np.save(tmp_path / "mult.npy", np.array([1.0, 1.1, 1.2, 0.9]))
    lr = SectionLearner(200, CFG, tmp_path / "mult.npy")
    assert lr.mult[1] == 1.1
    assert len(lr.multiplier_per_point()) == 200
