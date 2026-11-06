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
Parts too thin to see at full length show only a few metres at one end, white and
thin steel parts stand on a darker ground, and sets of identical bolts show a single
bolt, with extra room in the frame where that bolt is very small.
"""

import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import bmesh
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
    "fire_extinguisher": Matrix.Rotation(math.pi, 4, "Z"),   # the wall plate is on -Y
}
CLOSE_UP = {  # metres framed from the part's lowest point, or if negative its highest; the rest runs out of frame
    "ladder_1": 5.0,
    "ladder_2": 5.0,
    "ladder_3": 5.0,
    "ladder_4": 5.0,
    "lift_rails": -2.0,                                      # the top anchor with the wires leaving it
}
GROUND = {"blade": (0.12, 0.13, 0.15, 1), "lift_rails": (0.12, 0.13, 0.15, 1)}
ROOM = {"gearbox_cover_bolts": 2.5}  # frame this many times wider than the part
ONE_BOLT = {  # bolt axis, and a radius around it that holds one bolt and no neighbour
    "flange_bolts": ("Z", 0.06),
    "gearbox_cover_bolts": ("Y", 0.04),
    "shrink_disc_bolts": ("X", 0.05),
}


def close_up(points, length):
    """Points framing ``length`` metres from the lowest point, or if negative from the
    highest; a long edge may have no vertex there."""
    if length < 0:
        high = max(p.z for p in points)
        section = [p for p in points if p.z >= high + length]
        return section + [Vector((p.x, p.y, high + length)) for p in section]
    low = min(p.z for p in points)
    section = [p for p in points if p.z <= low + length]
    return section + [Vector((p.x, p.y, low + length)) for p in section]


def keep_one_bolt(visuals, axis, radius):
    """Delete every face further than ``radius`` from the axis of the bolt nearest the camera."""
    across = [i for i in range(3) if i != "XYZ".index(axis)]

    def flat(point):
        return Vector([point[i] for i in across])

    heads = []
    for obj, _, _ in visuals:
        mesh = obj.data
        for face in mesh.polygons:
            if "bolt_black" in mesh.materials[face.material_index].name:
                heads += [flat(obj.matrix_world @ mesh.vertices[v].co) for v in face.vertices]
    toward_camera = flat(Vector((1.25, -2.1, 1.25)))
    centre = max(heads, key=toward_camera.dot)
    for _ in range(8):  # mean shift from the head's edge to its centre
        near = [p for p in heads if (p - centre).length < radius]
        centre = sum(near, Vector((0, 0))) / len(near)
    for obj, _, _ in visuals:
        mesh = bmesh.new()
        mesh.from_mesh(obj.data)
        far = [face for face in mesh.faces
               if (flat(obj.matrix_world @ face.calc_center_median()) - centre).length > radius]
        bmesh.ops.delete(mesh, geom=far, context="FACES")
        mesh.to_mesh(obj.data)
        mesh.free()


def render_object(folder, joints, view=None, length=None, ground=None, one_bolt=None, room=None):
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    robot = ET.parse(folder / "model.urdf").getroot()
    visuals = lab.import_visuals(folder, robot)
    if one_bolt:
        lab.set_pose(visuals, lab.link_transforms(robot, values={}))
        keep_one_bolt(visuals, *one_bolt)
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
    if room:
        bpy.context.scene.camera.data.ortho_scale *= room
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
                          CLOSE_UP.get(obj["slug"]), GROUND.get(obj["slug"]), ONE_BOLT.get(obj["slug"]), ROOM.get(obj["slug"]))
