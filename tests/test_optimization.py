import pytest

from glider.optimization import create_point


def test_create_point():
    max_dim = 5.0
    point = create_point(max_dim_m=max_dim)
    for dim in point:
        assert dim <= max_dim

    assert point[0] != point[1]
    assert point[1] != point[2]
