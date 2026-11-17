#!/usr/bin/env python3
"""Render a UR5 demo run side by side: RViz on the left, the MuJoCo replay on the right, same camera and sim times.

usage: render_ur5_video.py PREFIX --rviz-frames DIR [--out OUT.mp4] [--fps 30] [--no-text]

Reads PREFIX_labels.json (time window, run_ur5_demo.py), PREFIX_replay.json and PREFIX_obstacles.json
(extract_ur5_replay.py), and the RViz frames DIR/rviz_%05d.png with DIR/rviz_size.json (render_rviz_replay.py, at
start + k / fps). The joint angles are replayed into description/ur5/mujoco/scene.xml and rendered offscreen at RViz's
render-area size, from config/rviz/ur5_video.rviz's camera with MuJoCo's default 45 deg vertical field of view; nothing
is re-simulated. The tracked obstacles do not exist in the simulation (they do not collide); the replay draws them as
spheres where the obstacle messages put them, in the colors of the MPC's RViz markers. --no-text leaves out the labels.

Needs: mujoco (the version the simulator links, 3.13.0), numpy, Pillow, and ffmpeg on PATH.
"""
import argparse
import json
import math
from pathlib import Path
import subprocess

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[2]
SCENE = ROOT / "description/ur5/mujoco/scene.xml"
# config/rviz/ur5_video.rviz's Orbit view: yaw -125 deg, pitch 22 deg -> MuJoCo azimuth yaw + 180, elevation -pitch.
LOOKAT, DISTANCE, AZIMUTH, ELEVATION = (0.12, 0.0, 0.44), 1.9, 55.0, -22.0


def font(size):
    for path in ("/System/Library/Fonts/Helvetica.ttc", "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"):
        if Path(path).is_file():
            return ImageFont.truetype(path, size)
    return ImageFont.load_default(size=size)


def obstacle_color(identifier):
    """The MPC's marker color per obstacle id (src/visualization/dynamic_obstacle_visualization.cpp)."""
    hue = math.fmod(0.08 + identifier * 0.618033988749895, 1.0) * 6.0
    s, v = 0.75, 0.95
    f = hue - math.floor(hue)
    p, q, t = v * (1 - s), v * (1 - s * f), v * (1 - s * (1 - f))
    return [(v, t, p), (q, v, p), (p, v, t), (p, q, v), (t, p, v), (v, p, q)][int(hue) % 6]


def quaternion_matrix(w, x, y, z):
    n = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def axis_angle_matrix(axis, angle):
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * k @ k


def spheres_at(messages, times, t):
    """[(center, radius, id)] from the last obstacle message at or before t, moved on at its velocity."""
    i = int(np.searchsorted(times, t, side="right")) - 1
    if i < 0 or t - times[i] > 0.5:
        return []
    out = []
    dt = t - times[i]
    for o in messages[i]["obstacles"]:
        rotation = quaternion_matrix(*o["quaternion_wxyz"])
        w = np.array(o["angular"])
        if np.linalg.norm(w) > 1e-12:
            rotation = axis_angle_matrix(w / np.linalg.norm(w), np.linalg.norm(w) * dt) @ rotation
        origin = np.array(o["position"]) + dt * np.array(o["linear"])
        for c, r in zip(o["centers"], o["radii"]):
            out.append((origin + rotation @ np.array(c), r, o["id"]))
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("prefix")
    parser.add_argument("--rviz-frames", type=Path, required=True)
    parser.add_argument("--out", help="default: PREFIX.mp4")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--no-text", action="store_true", help="views only: no labels")
    args = parser.parse_args()

    labels = json.loads(Path(args.prefix + "_labels.json").read_text())
    samples = json.loads(Path(args.prefix + "_replay.json").read_text())
    obstacles = json.loads(Path(args.prefix + "_obstacles.json").read_text())
    size = json.loads((args.rviz_frames / "rviz_size.json").read_text())
    w, h = 2 * (size["width"] // 2), 2 * (size["height"] // 2)
    start, end = labels["start"], labels["end"]
    out = args.out or args.prefix + ".mp4"

    model = mujoco.MjModel.from_xml_path(str(SCENE))
    model.vis.global_.offwidth, model.vis.global_.offheight = max(model.vis.global_.offwidth, w), max(model.vis.global_.offheight, h)
    data = mujoco.MjData(model)
    times = np.array([s["time"] for s in samples])
    positions = np.array([s["q"] for s in samples])
    addresses = [model.joint(name).qposadr[0] for name in samples[0]["names"]]
    obstacle_times = np.array([m["time"] for m in obstacles])
    small = font(22)

    ffmpeg = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{2 * w}x{h}",
                               "-r", str(args.fps), "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20", out],
                              stdin=subprocess.PIPE)
    frames = int(round((end - start) * args.fps))
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = LOOKAT
    camera.distance, camera.azimuth, camera.elevation = DISTANCE, AZIMUTH, ELEVATION
    with mujoco.Renderer(model, height=h, width=w) as renderer:
        for k in range(frames + 1):
            t = start + k / args.fps
            for address, j in zip(addresses, range(positions.shape[1])):
                data.qpos[address] = float(np.interp(t, times, positions[:, j]))
            mujoco.mj_forward(model, data)
            renderer.update_scene(data, camera=camera)
            scene = renderer.scene
            for center, radius, identifier in spheres_at(obstacles, obstacle_times, t):
                if scene.ngeom >= scene.maxgeom:
                    break
                rgba = np.array([*obstacle_color(identifier), 0.85], dtype=np.float32)
                mujoco.mjv_initGeom(scene.geoms[scene.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE, np.array([radius, 0.0, 0.0]),
                                    center, np.eye(3).flatten(), rgba)
                scene.ngeom += 1
            right = Image.fromarray(renderer.render())
            left = Image.open(args.rviz_frames / f"rviz_{k:05d}.png").convert("RGB").resize((w, h), Image.LANCZOS)
            image = Image.new("RGB", (2 * w, h))
            image.paste(left, (0, 0))
            image.paste(right, (w, 0))
            if not args.no_text:
                draw = ImageDraw.Draw(image)
                draw.rectangle([0, h - 40, 2 * w, h], fill=(0, 0, 0))
                draw.text((16, h - 33), "RViz: MPC plan, future ghosts, obstacles with clearance and prediction",
                          fill=(220, 220, 220), font=small)
                draw.text((w + 16, h - 33), f"MuJoCo simulation   t = {t - start:5.2f} s", fill=(220, 220, 220), font=small)
            ffmpeg.stdin.write(image.tobytes())
    ffmpeg.stdin.close()
    ffmpeg.wait()
    print("wrote", out, frames + 1, "frames")


if __name__ == "__main__":
    main()
