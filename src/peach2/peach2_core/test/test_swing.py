import numpy as np
from peach2_core.swing import estimate_swing
import pytest


@pytest.mark.parametrize('amp,period,span,hz', [
    (0.012, 1.3, 2.0, 15.0),
    (0.020, 0.9, 1.5, 10.0),
    (0.005, 1.8, 2.0, 15.0),
])
def test_elliptic_swing_with_drift_and_uneven_sampling(amp, period, span, hz):
    rng = np.random.default_rng(1)
    t = np.sort(rng.uniform(0.0, span, int(span * hz)))
    major = np.array([0.8, 0.6, 0.0])
    minor = np.array([-0.6, 0.8, 0.0])
    w = 2 * np.pi / period
    pos = (np.array([0.5, 0.1, 0.8]) + np.outer(amp * np.sin(w * t + 0.4), major)
           + np.outer(0.3 * amp * np.cos(w * t + 0.4), minor)
           + np.outer(t, [0.002, 0.0, 0.001]) + rng.normal(0.0, 0.001, (t.size, 3)))
    a, T = estimate_swing(t, pos)
    assert a == pytest.approx(amp, rel=0.2, abs=0.0015)
    assert T == pytest.approx(period, rel=0.1)


def test_static_bag_amplitude_is_noise_level():
    rng = np.random.default_rng(2)
    t = np.linspace(0.0, 2.0, 30)
    a, _ = estimate_swing(t, 0.8 + rng.normal(0.0, 0.001, (30, 3)))
    assert a < 0.003


def test_insufficient_samples_return_nan():
    a, T = estimate_swing(np.arange(5, dtype=float), np.zeros((5, 3)))
    assert np.isnan(a) and np.isnan(T)
    a, T = estimate_swing(np.zeros(10), np.zeros((10, 3)))
    assert np.isnan(a) and np.isnan(T)


def test_scalar_signal_and_length_mismatch():
    t = np.linspace(0.0, 2.0, 40)
    a, T = estimate_swing(t, 0.01 * np.sin(2 * np.pi * t / 1.0))
    assert a == pytest.approx(0.01, rel=0.05)
    assert T == pytest.approx(1.0, rel=0.05)
    with pytest.raises(ValueError):
        estimate_swing(t, np.zeros((3, 3)))
