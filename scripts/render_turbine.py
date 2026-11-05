"""Render the wind turbine parts' previews with their original visual materials.

Run with Blender 5.2+ (native WebP output) after scripts/export_turbine_parts.py:
    blender --background --factory-startup --python scripts/render_turbine.py
    blender --background --factory-startup --python scripts/render_turbine.py -- 205-blade

Uses the laboratory studio, camera and URDF import (render_laboratory.py). Rest
poses use zero joint values; open poses use each movable joint's "inspect" value
from objects/wind-turbine/collection.json (the crane hook's upper limit reaches
the ground 115 m below). Only the healthy model.urdf is rendered. Parts whose
front faces +Y are turned to the camera, and very slender parts are laid along X
so they fill the landscape frame; this changes only the preview, not the model.
Parts too thin to see at full length show only their lowest metres, and white
parts stand on a darker ground.
"""

import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import bpy
from mathutils import Matrix, Vector

sys.path.insert(0, str(Path(__file__).resolve().parent))
import render_laboratory as lab  # noqa: E402

TURBINE = lab.ROOT / "objects" / "wind-turbine"
VIEW = {
    "cabinet_converter": Matrix.Rotation(math.pi, 4, "Z"),   # doors face +Y
    "cabinet_controller": Matrix.Rotation(math.pi, 4, "Z"),
    "blade": Matrix.Rotation(-math.pi / 2, 4, "Y"),          # 63 m tall, 4 m chord
    "cable_loop": Matrix.Rotation(-math.pi / 2, 4, "Y"),
}
CLOSE_UP = {  # metres framed from the part's lowest point; the rest runs out of frame
    "ladder_1": 5.0,
    "ladder_2": 5.0,
    "ladder_3": 5.0,
    "ladder_4": 5.0,
    "lift_rails": 4.0,
}
GROUND = {"blade": (0.12, 0.13, 0.15, 1)}


def close_up(points, length):
    """Points framing the lowest ``length`` metres; a long edge may have no vertex there."""
    low = min(p.z for p in points)
    section = [p for p in points if p.z <= low + length]
    return section + [Vector((p.x, p.y, low + length)) for p in section]


def render_object(folder, joints, view=None, length=None, ground=None):
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    robot = ET.parse(folder / "model.urdf").getroot()
    visuals = lab.import_visuals(folder, robot)
    poses = {"rest": lab.link_transforms(robot, values={})}
    if joints:
        poses["open"] = lab.link_transforms(robot, values={j["name"]: j["inspect"] for j in joints})
    if view is not None:
        poses = {name: {link: view @ m for link, m in transforms.items()} for name, transforms in poses.items()}
    points = []
    for transforms in poses.values():
        lab.set_pose(visuals, transforms)
        points.extend(lab.vertices(visuals))
    lab.setup_studio(close_up(points, length) if length else points)
    if ground:
        shader = bpy.data.objects["Studio ground"].active_material.node_tree.nodes.get("Principled BSDF")
        shader.inputs["Base Color"].default_value = ground
    (folder / "renders").mkdir(exist_ok=True)
    for name, transforms in poses.items():
        lab.set_pose(visuals, transforms)
        bpy.context.scene.render.filepath = str(folder / "renders" / f"{name}.webp")
        bpy.ops.render.render(write_still=True)
    stale = folder / "renders" / "open.webp"
    if not joints and stale.exists():
        stale.unlink()
    print(f"RENDERED {folder.name} {sorted(poses)}", flush=True)


if __name__ == "__main__":
    requested = set(sys.argv[sys.argv.index("--") + 1:]) if "--" in sys.argv else set()
    catalogue = json.loads((TURBINE / "collection.json").read_text(encoding="utf-8"))
    for obj in catalogue["objects"]:
        if not requested or obj["folder"] in requested or obj["slug"] in requested:
            render_object(TURBINE / obj["folder"], obj["joints"], VIEW.get(obj["slug"]),
                          CLOSE_UP.get(obj["slug"]), GROUND.get(obj["slug"]))
