import numpy as np

from hand_tracker.one_euro import OneEuroFilter

FS = 30.0


def make():
    # Plan defaults; landmarks are normalized, value_scale = 640 px image width.
    return OneEuroFilter(min_cutoff=1.0, beta=0.007, d_cutoff=1.0, value_scale=640)


def test_constant_passes_unchanged():
    f = make()
    x = np.full((21, 3), 0.37)
    for k in range(50):
        assert np.array_equal(f(x, k / FS), x)


def test_step_settles_within_10_samples():
    f = make()
    step = 0.1
    f(np.zeros(3), 0.0)
    out = [f(np.full(3, step), k / FS) for k in range(1, 11)]
    assert np.all(np.abs(out[-1] - step) < 0.05 * step)


def test_sine_lag_below_3_samples():
    f = make()
    t = np.arange(0, 10, 1 / FS)
    x = 0.5 + 0.1 * np.sin(2 * np.pi * 1.0 * t)
    y = np.array([f(v, ti) for v, ti in zip(x, t)])
    xs, ys = x[150:] - 0.5, y[150:] - 0.5  # steady state
    lags = range(0, 10)
    err = [np.mean((xs[: len(xs) - k] - ys[k:]) ** 2) for k in lags]
    lag = int(np.argmin(err))
    assert lag < 3, f"lag {lag} samples"


def test_vectorized_matches_scalar():
    fv, fs = make(), make()
    rng = np.random.default_rng(0)
    for k in range(30):
        x = rng.uniform(0, 1, 5)
        yv = fv(x, k / FS)
    fs_out = None
    rng = np.random.default_rng(0)
    for k in range(30):
        x = rng.uniform(0, 1, 5)
        fs_out = fs(x[2], k / FS)
    assert np.isclose(yv[2], fs_out)


def test_reset():
    f = make()
    f(np.zeros(2), 0.0)
    f(np.ones(2), 0.1)
    f.reset()
    assert np.array_equal(f(np.full(2, 5.0), 0.2), np.full(2, 5.0))
