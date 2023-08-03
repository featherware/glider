import mediapy as media
import mujoco


def render_to_collision(model, data, framerate=60, show=True) -> list:
    renderer = mujoco.Renderer(model)
    frames = []
    mujoco.mj_resetData(model, data)  # Reset state and time.
    while len(data.contact) < 1:  # Render until landing
        mujoco.mj_step(model, data)
        if len(frames) < data.time * framerate:
            renderer.update_scene(data)
            pixels = renderer.render()
            frames.append(pixels)
    if show:
        media.show_video(frames, fps=framerate)

    return frames


def render_for_time_secs(model, data, duration=5, framerate=60, show=True):
    renderer = mujoco.Renderer(model)
    frames = []
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
