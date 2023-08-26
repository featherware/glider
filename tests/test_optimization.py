import pytest

from glider.optimization import (
    create_point,
    iterate_population,
    fitness_func,
)


def test_create_point():
    max_dim = 5.0
    point = create_point(max_dim_m=max_dim)
    for dim in point:
        assert dim <= max_dim

    assert point[0] != point[1]
    assert point[1] != point[2]


def test_iterate_population():
    population_size = 10
    survival_weight = 0.5
    population = iterate_population(
        [],
        population_size=10,
        survival_weight=0.5,
    )

    fitnesses = sorted([fitness_func(genes) for genes in population], reverse=True)

    assert fitnesses[0] > fitnesses[1]

    for _ in range(10):
        population = iterate_population(
            population,
            population_size=10,
            survival_weight=0.3,
            cloning_weight=0.3,
        )

        fitnesses_2 = sorted([fitness_func(genes) for genes in population], reverse=True)

        assert fitnesses_2[0] >= fitnesses[0]
