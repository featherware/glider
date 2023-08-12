import mediapy as media
import mujoco
import numpy as np

import simulation


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
    world_xml = simulation.wrap_glider(glider_xml, glider_asset)

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
