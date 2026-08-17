import math

import pytest

from brov_viz.geometry import (
    inside_pool,
    pool_corners,
    pool_edge_segments,
    quaternion_rotation_matrix,
)


def test_pool_geometry_has_exact_nominal_extents_and_twelve_edges() -> None:
    corners = pool_corners([4.0, 1.7, 1.1])
    assert len(corners) == 8
    assert {point[0] for point in corners} == {0.0, 4.0}
    assert {point[1] for point in corners} == {0.0, 1.7}
    assert {point[2] for point in corners} == {0.0, 1.1}
    assert len(pool_edge_segments([4.0, 1.7, 1.1])) == 12


def test_surveyed_marker_axes_match_observed_opencv_axes() -> None:
    rotation = quaternion_rotation_matrix([-0.5, -0.5, 0.5, 0.5])
    columns = tuple(
        tuple(row[index] for row in rotation) for index in range(3)
    )
    assert columns[0] == pytest.approx((0.0, 1.0, 0.0))
    assert columns[1] == pytest.approx((0.0, 0.0, -1.0))
    assert columns[2] == pytest.approx((-1.0, 0.0, 0.0))

    # R is a proper rotation rather than a reflection.
    determinant = (
        rotation[0][0]
        * (
            rotation[1][1] * rotation[2][2]
            - rotation[1][2] * rotation[2][1]
        )
        - rotation[0][1]
        * (
            rotation[1][0] * rotation[2][2]
            - rotation[1][2] * rotation[2][0]
        )
        + rotation[0][2]
        * (
            rotation[1][0] * rotation[2][1]
            - rotation[1][1] * rotation[2][0]
        )
    )
    assert determinant == pytest.approx(1.0)


def test_marker_black_square_fits_nominal_pool() -> None:
    # Marker +X is pool +Y and marker +Y is pool -Z.
    center = (3.8, 0.85, 0.24)
    half = 0.42 / 2.0
    assert center[1] - half == pytest.approx(0.64)
    assert center[1] + half == pytest.approx(1.06)
    assert center[2] - half == pytest.approx(0.03)
    assert center[2] + half == pytest.approx(0.45)
    assert inside_pool(center, [4.0, 1.7, 1.1])


@pytest.mark.parametrize(
    "position, expected",
    [
        ((0.0, 0.0, 0.0), True),
        ((4.0, 1.7, 1.1), True),
        ((4.01, 0.85, 0.5), False),
        ((2.0, -0.01, 0.5), False),
        ((2.0, 0.85, math.nan), False),
    ],
)
def test_inside_pool(position, expected) -> None:
    if any(not math.isfinite(value) for value in position):
        with pytest.raises(ValueError):
            inside_pool(position, [4.0, 1.7, 1.1])
    else:
        assert inside_pool(position, [4.0, 1.7, 1.1]) is expected
