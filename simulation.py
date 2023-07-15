from glider import glider_xml, pilot_xml, box_wing_xml

DENSITY = 1.2
VISCOSITY = 0.00002
WIND = "0 0 0"


def wrap_glider(glider_xml: str) -> str:
    world_xml = f"""
<mujoco>
    <option density="{DENSITY}" viscosity="{VISCOSITY}" wind="{WIND}"/>
    <worldbody>
        <light name="top" pos="0 0 5"/>
        <!-- Body -->
        {glider_xml}
    </worldbody>
</mujoco>

"""

    return world_xml


def drop_test_glider(glider_xml: str, distance=50) -> str:
    world_xml = f"""
<mujoco>
    <option density="{DENSITY}" viscosity="{VISCOSITY}" wind="{WIND}"/>
    <worldbody>
        <light name="top" pos="0 0 5"/>
        <!-- Body -->
        {glider_xml}
        <!-- Landing Platform -->
        <body name="platform" pos="0 0 0">
            <geom name="platform-geom" type="box" size="1500 1500 1" rgba="1 1 1 1" pos="0 0 {-distance}"/>
        </body>
    </worldbody>
</mujoco>

"""
    print(world_xml)
    return world_xml