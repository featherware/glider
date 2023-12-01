from .constants import AIR_DENSITY, AIR_VISCOSITY
from .vehicle import Vehicle


def drop_test_glider(
    vehicle: Vehicle,
    height=80,
    wind: str = "0 0 0",
) -> str:
    glider_xml, glider_asset = vehicle.xml()

    world_xml = f"""
<mujoco>
    <option density="{AIR_DENSITY}" viscosity="{AIR_VISCOSITY}" wind="{wind}"/>
    <worldbody>
        <light name="top" pos="0 0 5"/>
        <camera name="fixed" pos="0 -100 100" euler="40 0 0"/>
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
