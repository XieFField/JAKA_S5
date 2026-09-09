"""Tests for deterministic MuJoCo Viewer control helpers."""

import numpy as np
import pytest

from massage_mujoco.native_control import limited_linear_velocity


def test_linear_velocity_points_to_target_and_respects_speed_limit():
    velocity, reached = limited_linear_velocity(
        [0.0, 0.0, 0.0],
        [0.3, 0.4, 0.0],
        gain=3.0,
        maximum_speed=0.1,
        tolerance=0.001,
    )

    assert not reached
    np.testing.assert_allclose(velocity, [0.06, 0.08, 0.0])


def test_linear_velocity_stops_inside_position_tolerance():
    velocity, reached = limited_linear_velocity(
        [0.0, 0.0, 0.0],
        [0.0005, 0.0, 0.0],
        gain=3.0,
        maximum_speed=0.1,
        tolerance=0.001,
    )

    assert reached
    np.testing.assert_array_equal(velocity, np.zeros(3))


@pytest.mark.parametrize(
    "current,target,gain,maximum_speed,tolerance,error",
    [
        ([0.0], [0.0, 0.0, 0.0], 3.0, 0.1, 0.001, "three values"),
        ([0.0, 0.0, 0.0], [np.nan, 0.0, 0.0], 3.0, 0.1, 0.001, "finite"),
        ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 0.0, 0.1, 0.001, "positive"),
    ],
)
def test_linear_velocity_rejects_invalid_inputs(
    current,
    target,
    gain,
    maximum_speed,
    tolerance,
    error,
):
    with pytest.raises(ValueError, match=error):
        limited_linear_velocity(
            current,
            target,
            gain,
            maximum_speed,
            tolerance,
        )
