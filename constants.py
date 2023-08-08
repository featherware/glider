from functools import reduce

AIR_DENSITY = 1.2
AIR_VISCOSITY = 0.00002
WIND = "0 0 0"

# Pilot defaults
PILOT_RGBA = "0.2 0.2 0.8 0.5"
PILOT_DIMENSIONS_M = [1.8, 0.3, 0.6]
PILOT_DENSITY_KG = 68 / reduce(lambda x, y: x * y, PILOT_DIMENSIONS_M)

# Delta wing defaults
WING_RGBA = "0.8 0.2 0.2 0.5"

# Glider defaults
GLIDER_GEOM_NAME = "stl-wing"
DEFAULT_STL_FILEPATH = "assets/delta_plane.stl"

# Simulation
FRAMERATE = 60
TIME_STEP = 0.02

# Genetic Algorithm defaults
MUTATION_CHANCE = 1.0
MUTATION_RATIO = 0.15
