import mujoco
import numpy as np
import pandas as pd

from .constants import DEFAULT_STL_FILEPATH, GLIDER_GEOM_NAME
from .simulation import drop_test_glider
from .vehicle import Vehicle, create_glider_xml

NUM_GENES = 30
GLIDER_MAX_DIM = 5.0


def create_point(max_dim_m: float) -> list[float]:
    return list(np.random.random() * max_dim_m for _ in range(3))


def fitness_func(genes: list[list[float]]) -> float:
    test_vehicle = Vehicle(vertices=genes)

    glider_xml, glider_asset = test_vehicle.create_glider_from_vertices()
    world_xml = drop_test_glider(
        glider_xml=glider_xml,
        glider_asset=glider_asset,
    )

    model = mujoco.MjModel.from_xml_string(world_xml)
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)  # Reset state and time.

    while len(data.contact) < 1:  # Render until landing
        mujoco.mj_step(model, data)

    return abs(data.geom(GLIDER_GEOM_NAME).xpos[0])


def iterate_population(
    population: list[list[list[float]]] | None = None,
    population_size=100,
    survival_weight=0.3,
    cloning_weight=0.4,
):
    # on_start()

    # on_fitness()
    # on_parents()
    # on_crossover()
    # on_mutation()
    # on_generation()

    # on_stop()

    assert cloning_weight + survival_weight <= 1.0

    if not population:
        population = [
            [create_point(GLIDER_MAX_DIM) for _ in range(NUM_GENES)]
            for _ in range(population_size)
        ]

    results: list[float] = []

    for genes in population:
        results.append(fitness_func(genes))

    assert len(population) == len(results)

    # zip together
    ranking = list(zip(population, results))
    ranking.sort(key=lambda x: x[1], reverse=True)

    # Retain survivors
    survivor_results = ranking[: int(population_size * survival_weight)]

    survivors = [result[0] for result in survivor_results]

    clones = []

    for i in range(int(population_size * cloning_weight)):
        clones.append(
            Vehicle(vertices=survivors[i % len(survivors)]).mutate()
        )

    population = [ Vehicle(
        num_vertices=NUM_GENES,
        max_dim_m=GLIDER_MAX_DIM).vertices
        for _ in range(population_size - len(clones) - len(survivors))
    ]

    population += clones + survivors

    return population

    total_weight = mutation_weight + cloning_weight + crossover_weight
    mutation_weight /= total_weight
    cloning_weight /= total_weight
    crossover_weight /= total_weight

    # How many to retain from the previous run
    required = int((population_size * hereditary_weight) - len(survivors))

    new_population = []

    for i in range(int(required * mutation_weight)):
        new_population.append(vehicle.Vehicle.mutate(survivors[i // 2]))

    for i in range(int(required * cloning_weight)):
        new_population.append(vehicle.Vehicle(vertices=survivors[i // 2].vertices))

    for i in range(int(required * crossover_weight)):
        new_population.append(
            vehicle.Vehicle(vertices=survivors[i // 2].cross_over(survivors[i // 3]))
        )

    for i in range(population_size - len(new_population)):
        new_population.append(vehicle.Vehicle(num_vertices=30, max_dim_m=5.0))

    return new_population
