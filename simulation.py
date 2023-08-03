import constants
from glider import create_glider_xml


def wrap_glider(glider_xml: str, wind: str = "0 0 0") -> str:
    if not glider_xml:
        glider_xml, glider_asset = create_glider_xml()
    world_xml = f"""
<mujoco>
    <option density="{constants.AIR_DENSITY}" viscosity="{constants.AIR_VISCOSITY}" wind="{wind}"/>
    <worldbody>
        <light name="top" pos="0 0 5"/>
        <!-- Body -->
        {glider_xml}
    </worldbody>
</mujoco>

"""

    return world_xml


def drop_test_glider(
    glider_xml: str, glider_asset: str, height=80, wind: str = "0 0 0"
) -> str:
    if not glider_xml:
        glider_xml, glider_asset = create_glider_xml()
    world_xml = f"""
<mujoco>
    <option density="{constants.AIR_DENSITY}" viscosity="{constants.AIR_VISCOSITY}" wind="{wind}"/>
    <worldbody>
        <light name="top" pos="0 0 5"/>
        <!-- Body -->
        {glider_xml}
        <!-- Landing Platform -->
        <body name="platform" pos="0 0 0">
            <geom name="platform-geom" type="box" size="1500 1500 1" rgba="1 1 1 1" pos="0 0 {-height}"/>
        </body>
    </worldbody>

    {glider_asset}
</mujoco>

"""
    return world_xml
