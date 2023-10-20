import mujoco
import numpy as np

from .constants import DEFAULT_MAX_WING_DIMENSION_M
from .simulation import drop_test_glider
from .vehicle import Vehicle

NUM_GENES = 10
DROP_TEST_HEIGHT = 50.0


def create_point(max_dim_m: float) -> list[float]:
    return list(np.random.random() * max_dim_m for _ in range(3))


def fitness_func(test_vehicle: Vehicle) -> float:
    test_xml = drop_test_glider(test_vehicle, height=DROP_TEST_HEIGHT)

    model = mujoco.MjModel.from_xml_string(test_xml)
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)  # Reset state and time.

    while len(data.contact) < 1:  # Render until landing
        mujoco.mj_step(model, data)

    return abs(data.geom("vehicle-wing").xpos[0])


def iterate_population(
    input_population: list[Vehicle],
    survival_weight=0.3,
    cloning_weight=0.4,
    max_dim_m=DEFAULT_MAX_WING_DIMENSION_M,
    pilot: bool = False,
    mass_kg: float | None = None,
):
    """
    Take an input population, and return a new population based on the
    survival and cloning weights. Random gliders are generated to fill
    the remaining population.

    Args:
    survival_weight:    The proportion of the population that survives
    cloning_weight:     The proportion of the population that is cloned
    max_dim_m:          The maximum dimension of a wing
    pilot:              Whether or not to add a pilot
    mass_kg:            The mass of the wing
    """

    population_size = len(input_population)

    assert cloning_weight + survival_weight <= 1.0

    results: list[float] = []

    for v in input_population:
        results.append(fitness_func(v))

    assert len(input_population) == len(results)

    # Ranking is a combination of glider and fitness
    ranking = list(zip(input_population, results))
    ranking.sort(key=lambda x: x[1], reverse=True)

    # Retain survivors
    survivor_results = ranking[: int(population_size * survival_weight)]

    survivors: list[Vehicle] = [result[0] for result in survivor_results]

    clones: list[Vehicle] = []

    for i in range(int(population_size * cloning_weight)):
        target_index = i % len(survivors)

        clones.append(
            Vehicle(
                vertices=(survivors[target_index].mutate()),
                max_dim_m=survivors[target_index].max_dim_m,
                pilot=pilot,
                mass_kg=mass_kg,
            )
        )

    random_population = [
        Vehicle(
            num_vertices=NUM_GENES,
            max_dim_m=max_dim_m,
            pilot=pilot,
            mass_kg=mass_kg,
        )
        for _ in range(population_size - len(clones) - len(survivors))
    ]

    new_population = survivors + clones + random_population

    return ranking, new_population
