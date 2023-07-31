import mujoco
import mediapy as media

import constants

def glider_forwards_distance(data) -> float:
    return abs(data.geom("main_wing").xpos[0])


def measure_forwards_fitness(
        model,
        data
        ) -> float:
    
    xml = glider.drop_test_glider(
        glider.glider_xml(
            wing_fn=wing.mesh_geom
        ),
        assets=[wing.delta_wing_asset()],
        height=500
    )

    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)

    mujoco.mj_resetData(model, data)  # Reset state and time.

    while len(data.contact) < 1:  # Render until landing
        mujoco.mj_step(model, data)

    return glider_forwards_distance(data)
