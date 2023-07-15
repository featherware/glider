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