def wrap_glider(glider_xml: str) -> str:
    world_xml = f"""
<mujoco>
    <option density="1.2" viscosity="0.00002" wind="95 0 0"/>
    <worldbody>
        <light name="top" pos="0 0 5"/>
        <!-- Body -->
        {glider_xml}
    </worldbody>
</mujoco>

"""

    return world_xml