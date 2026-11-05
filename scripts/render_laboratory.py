"""Render the portable laboratory URDFs with their original visual materials.

Run with Blender 5.2+ (native WebP output):
    blender --background --factory-startup --python scripts/render_laboratory.py
    blender --background --factory-startup --python scripts/render_laboratory.py -- scale

The laboratory GLBs use the URDF's documented Z-up basis. Blender's glTF
importer assumes standard glTF Y-up, so its basis conversion must be undone
before applying URDF visual origins and forward kinematics. Collision shapes
are deliberately excluded; actual URDF primitive visuals are included.

Rest poses use zero joint values. Open poses use each bounded movable joint's
upper limit; continuous joints remain at zero. Both poses share one camera.
World visual bounds (metres, rest pose) are printed as JSON for the catalogue.
"""

import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import bpy
from mathutils import Euler, Matrix, Quaternion, Vector


ROOT = Path(__file__).resolve().parents[1]
LABORATORY = ROOT / "objects" / "laboratory"
WIDTH, HEIGHT = 520, 400
UNDO_GLTF_BASIS = Matrix.Rotation(-math.pi / 2, 4, "X")


def numbers(value, default):
    return tuple(float(v) for v in value.split()) if value else default


def origin_matrix(element):
    origin = element.find("origin")
    if origin is None:
        return Matrix.Identity(4)
    xyz = numbers(origin.get("xyz"), (0, 0, 0))
    rpy = numbers(origin.get("rpy"), (0, 0, 0))
    return Matrix.Translation(Vector(xyz)) @ Euler(rpy, "XYZ").to_matrix().to_4x4()


def link_transforms(robot, opened=False, values=None):
    """``values``: joint name -> position; overrides ``opened`` (upper limits)."""
    joints = robot.findall("joint")
    children = {j.find("child").get("link") for j in joints}
    transforms = {
        link.get("name"): Matrix.Identity(4)
        for link in robot.findall("link") if link.get("name") not in children
    }
    pending = list(joints)
    while pending:
        resolved = []
        for joint in pending:
            parent = joint.find("parent").get("link")
            if parent not in transforms:
                continue
            motion = Matrix.Identity(4)
            kind = joint.get("type")
            limit = joint.find("limit")
            value = float(limit.get("upper", "0")) if opened and limit is not None else 0
            if values is not None:
                value = values.get(joint.get("name"), 0.0)
            axis_element = joint.find("axis")
            axis = Vector(numbers(axis_element.get("xyz") if axis_element is not None else None, (1, 0, 0)))
            if kind in ("revolute", "continuous"):
                motion = Quaternion(axis.normalized(), value).to_matrix().to_4x4()
            elif kind == "prismatic":
                motion = Matrix.Translation(axis * value)
            elif kind != "fixed":
                raise ValueError(f"Unsupported joint type: {kind}")
            child = joint.find("child").get("link")
            transforms[child] = transforms[parent] @ origin_matrix(joint) @ motion
            resolved.append(joint)
        if not resolved:
            raise ValueError("URDF contains an unresolved or cyclic joint graph")
        pending = [joint for joint in pending if joint not in resolved]
    return transforms


def material_for_visual(visual):
    element = visual.find("material")
    color = element.find("color") if element is not None else None
    rgba = numbers(color.get("rgba") if color is not None else None, (0.65, 0.68, 0.68, 1))
    name = element.get("name", "URDF visual") if element is not None else "URDF visual"
    material = bpy.data.materials.new(name)
    material.diffuse_color = rgba
    material.use_nodes = True
    shader = material.node_tree.nodes.get("Principled BSDF")
    shader.inputs["Base Color"].default_value = rgba
    shader.inputs["Roughness"].default_value = 0.38
    if "steel" in name.lower():
        shader.inputs["Metallic"].default_value = 0.65
    return material


def import_visuals(folder, robot):
    """Return (object, link name, transform within link) for each visual mesh."""
    result = []
    for link in robot.findall("link"):
        for visual in link.findall("visual"):
            geometry = visual.find("geometry")
            mesh = geometry.find("mesh")
            visual_origin = origin_matrix(visual)
            if mesh is not None:
                before = set(bpy.data.objects)
                bpy.ops.import_scene.gltf(filepath=str(folder / mesh.get("filename")))
                imported = set(bpy.data.objects) - before
                # Capture world transforms before detaching glTF node parents.
                matrices = {obj: obj.matrix_world.copy() for obj in imported}
                scale = numbers(mesh.get("scale"), (1, 1, 1))
                scale_matrix = Matrix.Diagonal((*scale, 1))
                for obj in imported:
                    if obj.type == "MESH":
                        obj.parent = None
                        local = visual_origin @ scale_matrix @ UNDO_GLTF_BASIS @ matrices[obj]
                        result.append((obj, link.get("name"), local))
                    else:
                        bpy.data.objects.remove(obj, do_unlink=True)
            else:
                box = geometry.find("box")
                cylinder = geometry.find("cylinder")
                sphere = geometry.find("sphere")
                if box is not None:
                    bpy.ops.mesh.primitive_cube_add(size=1)
                    scale = numbers(box.get("size"), (1, 1, 1))
                elif cylinder is not None:
                    bpy.ops.mesh.primitive_cylinder_add(vertices=64, radius=1, depth=1)
                    scale = (float(cylinder.get("radius")),) * 2 + (float(cylinder.get("length")),)
                elif sphere is not None:
                    bpy.ops.mesh.primitive_uv_sphere_add(segments=48, ring_count=24, radius=1)
                    scale = (float(sphere.get("radius")),) * 3
                else:
                    raise ValueError(f"Unsupported visual geometry: {ET.tostring(geometry)}")
                obj = bpy.context.object
                obj.name = visual.get("name", link.get("name"))
                obj.data.materials.append(material_for_visual(visual))
                result.append((obj, link.get("name"), visual_origin @ Matrix.Diagonal((*scale, 1))))
    return result


def set_pose(visuals, transforms):
    for obj, link, local in visuals:
        obj.matrix_world = transforms[link] @ local
    bpy.context.view_layer.update()


def vertices(visuals):
    return [obj.matrix_world @ vertex.co for obj, _, _ in visuals for vertex in obj.data.vertices]


def bounds(points):
    return [Vector([min(p[i] for p in points) for i in range(3)]),
            Vector([max(p[i] for p in points) for i in range(3)])]


def track_to(obj, target):
    obj.rotation_euler = (target - obj.location).to_track_quat("-Z", "Y").to_euler()


def setup_studio(points):
    scene = bpy.context.scene
    scene.render.engine = "CYCLES"
    scene.cycles.samples = 48
    scene.cycles.use_denoising = True
    scene.render.resolution_x = WIDTH
    scene.render.resolution_y = HEIGHT
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "WEBP"
    scene.render.image_settings.color_mode = "RGB"
    scene.render.image_settings.quality = 92
    scene.view_settings.view_transform = "AgX"
    scene.world.use_nodes = True
    background = scene.world.node_tree.nodes.get("Background")
    background.inputs["Color"].default_value = (0.8, 0.8, 0.8, 1)
    background.inputs["Strength"].default_value = 0.6

    lower, upper = bounds(points)
    center = (lower + upper) / 2
    span = max(upper - lower)
    bpy.ops.mesh.primitive_plane_add(size=200 * span, location=(center.x, center.y, lower.z - 0.001 * span))
    ground = bpy.context.object
    ground.name = "Studio ground"
    material = bpy.data.materials.new("Warm ivory background")
    material.diffuse_color = (0.86, 0.85, 0.82, 1)
    material.use_nodes = True
    material.node_tree.nodes.get("Principled BSDF").inputs["Base Color"].default_value = material.diffuse_color
    material.node_tree.nodes.get("Principled BSDF").inputs["Roughness"].default_value = 0.8
    ground.data.materials.append(material)

    for name, offset, energy, size in (
        ("Large softbox", (-2.8, -3.7, 5), 400, 4),
        ("Fill softbox", (3, -0.5, 2.8), 160, 3),
        ("Top softbox", (0, 3, 4), 200, 3),
    ):
        data = bpy.data.lights.new(name, "AREA")
        light = bpy.data.objects.new(name, data)
        scene.collection.objects.link(light)
        light.location = center + Vector(offset) * span
        data.energy = energy * span * span
        data.shape = "DISK"
        data.size = size * span
        track_to(light, center)

    data = bpy.data.cameras.new("Catalogue camera")
    camera = bpy.data.objects.new("Catalogue camera", data)
    scene.collection.objects.link(camera)
    scene.camera = camera
    data.type = "ORTHO"
    direction = Vector((1.25, -2.1, 1.25)).normalized()
    camera.location = center + direction * span * 5
    track_to(camera, center)
    rotation = camera.rotation_euler.to_matrix()
    right, up = rotation.col[0], rotation.col[1]
    horizontal = [p.dot(right) for p in points]
    vertical = [p.dot(up) for p in points]
    # Center the actual projected geometry, including the open pose; a world
    # AABB center alone leaves asymmetrical open doors off-center.
    projected_center = center + right * ((min(horizontal) + max(horizontal)) / 2 - center.dot(right))
    projected_center += up * ((min(vertical) + max(vertical)) / 2 - center.dot(up))
    camera.location = projected_center + direction * span * 5
    # Blender's AUTO sensor fit makes ortho_scale the horizontal span for this
    # landscape output, so account for height using the width/height ratio.
    data.ortho_scale = max(max(horizontal) - min(horizontal), (max(vertical) - min(vertical)) * WIDTH / HEIGHT) * 1.23
    data.clip_start = span * 0.001
    data.clip_end = span * 300


def render_device(folder):
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    robot = ET.parse(folder / "model.urdf").getroot()
    visuals = import_visuals(folder, robot)
    poses = {"rest": link_transforms(robot)}
    if any(joint.get("type") in ("revolute", "prismatic") for joint in robot.findall("joint")):
        poses["open"] = link_transforms(robot, opened=True)
    combined_points = []
    rest_bounds = None
    for name, transforms in poses.items():
        set_pose(visuals, transforms)
        points = vertices(visuals)
        combined_points.extend(points)
        if name == "rest":
            rest_bounds = bounds(points)
    setup_studio(combined_points)
    (folder / "renders").mkdir(exist_ok=True)
    for name, transforms in poses.items():
        set_pose(visuals, transforms)
        bpy.context.scene.render.filepath = str(folder / "renders" / f"{name}.webp")
        bpy.ops.render.render(write_still=True)
    print("CATALOGUE_BOUNDS " + json.dumps({
        "device": folder.name,
        "min": [round(v, 6) for v in rest_bounds[0]],
        "max": [round(v, 6) for v in rest_bounds[1]],
        "size": [round(v, 6) for v in rest_bounds[1] - rest_bounds[0]],
    }), flush=True)


if __name__ == "__main__":
    requested = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    folders = [LABORATORY / name for name in requested] if requested else sorted(LABORATORY.glob("*/model.urdf"))
    for candidate in folders:
        render_device(candidate.parent if candidate.is_file() else candidate)
