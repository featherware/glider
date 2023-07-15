def fitness_fn_forwards(data) -> float:
    return abs(data.geom("main_wing").xpos[0])