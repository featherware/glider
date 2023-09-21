import mediapy as media
import mujoco
import numpy as np

from .constants import AIR_DENSITY, AIR_VISCOSITY


def wrap_glider(
    glider_xml: str, glider_asset: str, wind: str = "0 0 0"
) -> str:

    world_xml = f"""
<mujoco>
    <option density="{AIR_DENSITY}" viscosity="{AIR_VISCOSITY}" wind="{wind}"/>
    <worldbody>
        <light name="sideN" pos="0 1 10"/>
        <light name="sideE" pos="1 0 10"/>
        <light name="sideS" pos="0 -1 10"/>
        <light name="sideW" pos="-1 0 10"/>
        <!-- Body -->
        {glider_xml}
    </worldbody>

    {glider_asset}
</mujoco>

"""

    return world_xml


def render_initial_pixels(
    model,
    data,
) -> np.ndarray:
    renderer = mujoco.Renderer(model)

    mujoco.mj_step(model, data)

    renderer.update_scene(data)
    pixels = renderer.render()
    return pixels


def view_vehicle(glider_xml, glider_asset):
    world_xml = wrap_glider(glider_xml, glider_asset)

    model = mujoco.MjModel.from_xml_string(world_xml)
    data = mujoco.MjData(model)

    return render_initial_pixels(model, data)


def render_to_collision(model, data, framerate=60, show=True) -> list[np.ndarray]:
    renderer = mujoco.Renderer(model)
    frames: list[np.ndarray] = []
    mujoco.mj_resetData(model, data)  # Reset state and time.
    while len(data.contact) < 1:  # Render until landing
        mujoco.mj_step(model, data)
        if len(frames) < data.time * framerate:
            renderer.update_scene(data, "track")
            pixels = renderer.render()
            frames.append(pixels)
    if show:
        media.show_video(frames, fps=framerate)

    return frames


def render_for_time_secs(
    model, data, duration=5, framerate=60, show=True
) -> list[np.ndarray]:
    renderer = mujoco.Renderer(model)
    frames: list[np.ndarray] = []

    mujoco.mj_step(model, data)
    while data.time < duration:
        mujoco.mj_step(model, data)
        if len(frames) < data.time * framerate:
            renderer.update_scene(data)
            pixels = renderer.render()
            frames.append(pixels)
    if show:
        media.show_video(frames, fps=framerate)

    return frames
