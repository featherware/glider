import constants
from vehicle import create_glider_xml


def wrap_glider(
    glider_xml: str | None = None, glider_asset: str | None = None, wind: str = "0 0 0"
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
    </worldbody>

    {glider_asset}
</mujoco>

"""

    return world_xml


def drop_test_glider(
    glider_xml: str | None = None,
    glider_asset: str | None = None,
    orientation=None,
    height=80,
    wind: str = "0 0 0",
) -> str:
    if not glider_xml or not glider_asset:
        glider_xml, glider_asset = create_glider_xml(orientation=orientation)

    assert glider_xml
    assert glider_asset

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
