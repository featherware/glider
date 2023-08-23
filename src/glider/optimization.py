import mujoco
import pandas as pd

from .constants import DEFAULT_STL_FILEPATH
from .observability import glider_abs_x_position
from .simulation import drop_test_glider
from .vehicle import Vehicle, create_glider_xml


def iterate_population(
    population,
    population_size=100,
    survival_weight=0.3,
):

    if len(population) < population_size:
        # Initiate a new population
        population = population + [Vehicle(num_vertices=30, max_dim_m=5.0) for _ in range(population_size - len(population))]

    results: dict[Vehicle, float] = {}

    for test_vehicle in population:
        result = measure_drop_test(*test_vehicle.create_glider_from_vertices())
        results[test_vehicle] = result

    # Sort by highest fitness
    sorted_results = sorted(list(results.items()), key=lambda x: x[1], reverse=True)

    # Retain survivors
    survivor_results = sorted_results[:int(population_size * survival_weight)]

    survivors = [result[0] for result in survivor_results]

    return survivors

    total_weight = mutation_weight + cloning_weight + crossover_weight
    mutation_weight /= total_weight
    cloning_weight /= total_weight
    crossover_weight /= total_weight

    # How many to retain from the previous run
    required = int((population_size * hereditary_weight)  - len(survivors))

    new_population = []

    for i in range(int(required * mutation_weight)):
        new_population.append(vehicle.Vehicle.mutate(survivors[i//2]))

    for i in range(int(required * cloning_weight)):
        new_population.append(vehicle.Vehicle(vertices = survivors[i//2].vertices))

    for i in range(int(required * crossover_weight)):
        new_population.append(vehicle.Vehicle(
            vertices=survivors[i//2].cross_over(survivors[i//3])))

    for i in range(population_size - len(new_population)):
        new_population.append(vehicle.Vehicle(num_vertices=30, max_dim_m=5.0))

    return new_population


def find_best_orientation(
    stl_filename: str = DEFAULT_STL_FILEPATH,
    granularity_deg: int = 50,
    max_results: int = 10,
) -> list[int]:
    # array = [
    #     pd.DataFrame(
    #         [[x, y, z, distance_score(x,y,z)]],
    #         columns=['x', 'y', 'z', 'distance'],
    #         )
    #     for x in range(0,360, granularity)
    #     for y in range(0,360, granularity)
    #     for z in range(0, 360, granularity)
    # ]

    df = pd.DataFrame(columns=["x", "y", "z", "distance"])
    df = pd.concat(
        [
            pd.DataFrame(
                [[x, y, z, measure_drop_test(orientation=[x, y, z])]],
                columns=["x", "y", "z", "distance"],
            )
            for x in range(0, 360, granularity_deg)
            for y in range(0, 360, granularity_deg)
            for z in range(0, 360, granularity_deg)
        ],
        ignore_index=True,
    )

    print("Furthest distance: ", df["distance"].max())
    print("Best orientation", df[df["distance"] == df["distance"].max()])
    return df[df["distance"] == df["distance"].max()]


def measure_drop_test(
    glider_xml: str,
    glider_asset: str,
    scale: float = 1.0,
    orientation: list[int] = [200, 200, 100],
    **kwargs,
) -> float:
    world_xml = drop_test_glider(
        glider_xml=glider_xml,
        glider_asset=glider_asset,
        **kwargs,
    )

    model = mujoco.MjModel.from_xml_string(world_xml)
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)  # Reset state and time.

    while len(data.contact) < 1:  # Render until landing
        mujoco.mj_step(model, data)

    return glider_abs_x_position(data)
