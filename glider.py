from typing import Callable

import wing

import constants


def pilot_xml():
    return f"""<geom name="pilot" type="box" size="{" ".join(
        [ str(dim) for dim in constants.PILOT_DIMENSIONS_M ]
        )}" rgba="{constants.PILOT_RGBA}" pos="0 0 -0.3"/>"""


def glider_xml(
    wing_fn: Callable = wing.box_wing_xml,
    pilot_fn: Callable = pilot_xml,
    pitch_angle: int = 20,
) -> str:
    return f"""
<body name="body" pos="0 0 1" euler="90 0 {pitch_angle}">
    <freejoint/>
    <!-- Main Wing -->
    {wing_fn()}
    <!-- Pilot -->
    {pilot_fn()}
</body>
"""


def drop_test_glider(
    glider_xml: str, assets: list = list(), height=80, wind: str = "0 0 0"
) -> str:
    world_xml = f"""
<mujoco>
    <option density="{
        constants.AIR_DENSITY
    }" viscosity="{
        constants.AIR_VISCOSITY
    }" wind="{
        wind
    }"/>
    <worldbody>
        <light name="top" pos="0 0 5"/>
        <!-- Body -->
        {glider_xml}
        <!-- Landing Platform -->
        <body name="platform" pos="0 0 0">
            <geom name="platform-geom" {
                ""
            }type="box" size="1500 1500 1" {
            ""
            }rgba="1 1 1 1" pos="0 0 {-height}"/>
        </body>
    </worldbody>

    {'/n'.join(assets) if assets else ''}
</mujoco>

"""
    print(world_xml)
    return world_xml


def fitness_fn_forwards(data) -> float:
    return abs(data.geom("main_wing").xpos[0])


if __name__ == "__main__":
    print(glider_xml())
