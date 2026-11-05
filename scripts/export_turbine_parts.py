#!/usr/bin/env python3
"""Export the wind turbine's parts as independent objects in objects/wind-turbine/.

    python3 scripts/export_turbine_parts.py --source /path/to/cognitive_robot_abstract_machine

The source is a checkout of sorinar329/cognitive_robot_abstract_machine on branch
windturbine_maintenance; it is only read. Its turbine generator (``turbine.parts``)
gives every part's link, parent, joint, collisions, healthy/faulty variants and
fault overlays, and ``models/<group>/*.glb`` holds the meshes in their link frames.

For every object in PARTS this writes, from scratch except for ``renders/``:
assets/*.glb (node transform and placement baked into the vertices, origin at the
centre of the root mesh's footprint), model.urdf, one model_<variant>.urdf per fault
state, metadata.json (placement in the assembled turbine), README.md, NOTICE.md,
LICENSE.txt and rights.json. It also writes collection.json, README.md and
INVENTORY.md for the collection and removes object folders no longer in PARTS.
Previews come from scripts/render_turbine.py (Blender) afterwards.

Requires numpy. Ids are assigned in PARTS order from FIRST_ID, so append new parts
at the end to keep ids stable. Intended to move next to the turbine's other
exporters on windturbine_maintenance once the structure is agreed.
"""
from __future__ import annotations

import argparse
import importlib
import json
import math
import re
import shutil
import struct
import subprocess
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
COLLECTION = "wind-turbine"
OUTPUT = ROOT / "objects" / COLLECTION
FIRST_ID = 170
ADDED = "2026-10-30"
SOURCE_REPO = "sorinar329/cognitive_robot_abstract_machine"
SOURCE_BRANCH = "windturbine_maintenance"

CATEGORY = {
    "key": COLLECTION, "code": "WTB", "title": "Wind Turbine",
    "blurb": "Parts of an onshore wind turbine after the IEA 3.4 MW reference design: tower, "
             "nacelle, drivetrain, rotor and service equipment, each with its healthy and faulty states.",
    "added": ADDED,
}

# Rights: the turbine model's author decides (open question in WP3). Until confirmed,
# objects carry the library's licence and say so in their notice.
AUTHOR = "sorinar329"
COPYRIGHT = "Copyright (c) 2026, sorinar329"
LICENSE_ID = "BSD-3-Clause"
LICENSE_STATUS = "proposed; pending confirmation by the turbine model's author"
IEA = ("IEA Wind TCP Task 37, IEA-3.4-130-RWT reference wind turbine",
       "https://github.com/IEAWindSystems/IEA-3.4-130-RWT", "Apache-2.0")
TEXTURES = ("Concrete034", "Grass004", "Gravel023", "Metal027", "MetalWalkway011",
            "PaintedMetal004", "Plastic010")

# Left out of the library: the technician's origin is unknown (WP3, rights), and the
# 140 m ground plane is scenery, not a turbine part.
EXCLUDED = {"systems/technician.glb": "origin of the model unknown; cleared before inclusion",
            "site/ground.glb": "140 m terrain plane around the site; scenery, not a turbine part"}


def part(slug, name, desc, *links, instances=None, rename=None, meshes=None):
    """``links``: turbine links in this object, root first. ``instances``: further
    turbine links that use the same geometry (the three blades). ``rename``: object
    link name per turbine link. ``meshes``: only these source meshes per link."""
    return dict(slug=slug, name=name, desc=desc, links=list(links), instances=instances or [],
                rename=rename or {}, meshes=meshes or {})


PARTS = [
    # site
    part("foundation", "Turbine Foundation",
         "Round concrete foundation plinth that the tower base stands on.",
         "foundation"),
    part("foundation_grout", "Foundation Grout Joint",
         "Grout bed between the foundation plinth and the tower's base flange, intact or broken out.",
         "foundation_grout"),
    part("foundation_earthing", "Foundation Earthing Strap",
         "Earthing strap that bonds the tower base to the foundation's earth electrode, connected or loose.",
         "foundation_earthing"),
    part("tower_section_1", "Tower Base Section with Door",
         "Lowest steel tower section with its base flange and anchor nuts, the upper flange and an entrance door that swings open.",
         "tower_section_1", "tower_door"),
    part("tower_section_2", "Tower Section 2", "Second steel tower section, with flanges at both ends.",
         "tower_section_2"),
    part("tower_section_3", "Tower Section 3", "Third steel tower section, with flanges at both ends.",
         "tower_section_3"),
    part("tower_section_4", "Tower Top Section", "Top steel tower section, ending at the yaw bearing flange.",
         "tower_section_4"),
    part("yaw_bearing", "Yaw Bearing", "Slewing ring on the tower top that the nacelle turns on.",
         "yaw_bearing"),
    # tower interior
    *[part(f"platform_{k}", f"Tower Platform {k}",
           (f"Grated rest platform under the flange at the top of tower section {k}, with railings and "
            "openings for the ladder and the service lift." if k < 4 else
            "Grated yaw deck at the top of the tower, with railings and openings for the ladder, "
            "the service lift and the cable loop."), f"platform_{k}") for k in range(1, 5)],
    *[part(f"ladder_{k}", f"Tower Ladder {k}",
           f"Ladder with fall-arrest rail, power cables and a wall lamp, running up tower section {k}.", f"ladder_{k}") for k in range(1, 5)],
    part("cable_loop", "Tower Cable Loop",
         "Drooping loop of the power cables below the yaw deck that lets the nacelle turn.", "cable_loop"),
    part("ground_controller", "Tower Base Controller",
         "Ground controller cabinet at the tower base, opposite the door.", "ground_controller"),
    part("lift_rails", "Service Lift Rails", "Guide and hoist wires of the service lift with their top anchor, running up the whole tower.",
         "lift_rails"),
    part("service_lift", "Service Lift Car", "Open service lift car with railings, a roof hoist and a control panel.", "service_lift"),
    *[part(f"flange_{k}_bolts", f"Tower Flange {k} Bolts",
           f"Ring of bolts with torque markings that joins tower section {k} to the section above.",
           f"flange_{k}_bolts") for k in range(1, 4)],
    # nacelle
    part("nacelle_floor", "Nacelle Floor with Hatches",
         "Floor grating of the nacelle with a hinged tower access hatch and the crane hatch at the rear.",
         "nacelle", "nacelle_floor_hatch", "crane_hatch",
         rename={"nacelle": "nacelle_floor"}, meshes={"nacelle": ["nacelle/floor.obj"]}),
    part("nacelle_cover", "Nacelle Cover", "Glass-fibre shell over the nacelle, open at the bottom.",
         "nacelle", rename={"nacelle": "nacelle_cover"}, meshes={"nacelle": ["nacelle/cover.obj"]}),
    part("weather_mast", "Nacelle Weather Mast", "Mast on the nacelle roof that carries the anemometer, the wind vane and the aviation light.",
         "weather_mast"),
    part("anemometer", "Cup Anemometer", "Cup anemometer on the weather mast, intact or damaged.", "anemometer"),
    part("aviation_light", "Aviation Obstruction Light", "Red obstruction light on the weather mast, working or broken.",
         "aviation_light"),
    part("bedplate", "Bedplate", "Cast front bedplate with side girders for the main bearing and the gearbox torque arms, and a welded rear frame for the generator and transformer.", "bedplate"),
    # main shaft
    part("main_bearing", "Main Bearing", "Main bearing housing that carries the rotor shaft.", "main_bearing"),
    part("grease_collector", "Main Bearing Grease Collector",
         "Transparent grease collector under the main bearing, normal or full.", "main_bearing_grease_collector"),
    part("main_shaft", "Main Shaft", "Low-speed rotor shaft between the hub flange and the gearbox.", "main_shaft"),
    part("shrink_disc_bolts", "Shrink Disc Bolts",
         "Bolt ring of the shrink disc that clamps the main shaft to the gearbox, tight or loose.",
         "main_shaft_shrink_disc_bolts"),
    part("rotor_lock", "Rotor Lock with Pin",
         "Rotor lock block beside the main bearing with a locking pin that slides into the hub flange.",
         "rotor_lock", "rotor_lock_pin"),
    # rotor
    part("hub", "Rotor Hub", "Cast rotor hub with three blade flanges and a spinner.", "hub"),
    part("blade", "Rotor Blade", "63 m rotor blade lofted from the IEA-3.4-130-RWT data, from the root flange to the tip.",
         "blade_1", instances=["blade_2", "blade_3"], rename={"blade_1": "blade"}),
    # gearbox
    part("gearbox", "Gearbox", "Gearbox with torque arms and oil cooler, between the main shaft and the generator.",
         "gearbox"),
    part("gearbox_sight_glass", "Gearbox Oil Sight Glass", "Oil level sight glass on the gearbox, full or low.",
         "gearbox_sight_glass"),
    part("gearbox_filter_indicator", "Gearbox Filter Indicator",
         "Pop-up clogging indicator of the gearbox oil filter, normal or clogged.", "gearbox_filter_indicator"),
    part("gearbox_cover_bolts", "Gearbox Cover Bolts", "Bolts of the gearbox inspection cover, complete or one missing.",
         "gearbox_cover_bolts"),
    part("gearbox_bushing", "Gearbox Torque Arm Bushing",
         "Elastomer bushing of the gearbox's left torque arm, intact or cracked.", "gearbox_bushing_left"),
    # generator
    part("fast_shaft", "High-Speed Shaft", "High-speed shaft with coupling hubs, from the gearbox output to the generator.", "fast_shaft"),
    part("brake_disc", "Rotor Brake Disc", "Brake disc on the high-speed shaft, normal or overheated.",
         "fast_shaft_brake_disc"),
    part("coupling", "Shaft Coupling", "Flexible disc-pack coupling to the generator, intact or cracked.",
         "fast_shaft_coupling"),
    part("brake_caliper", "Rotor Brake Caliper", "Brake caliper straddling the top of the brake disc, with normal or worn pads.",
         "brake_caliper"),
    part("generator", "Generator", "Doubly fed induction generator with finned housing, end shields, feet, terminal box and slip-ring housing.",
         "generator"),
    # systems
    part("transformer", "Nacelle Transformer", "Transformer with three cast-resin coils on a core frame inside a mesh cage.", "transformer"),
    part("cabinet_converter", "Converter Cabinet with Door",
         "Power converter cabinet with a hinged door that carries a status light, normal or showing an error.",
         "cabinet_converter", "cabinet_converter_door", "converter_status_light"),
    part("cabinet_controller", "Controller Cabinet with Door",
         "Turbine controller cabinet with a hinged door.", "cabinet_controller", "cabinet_controller_door"),
    part("hydraulic_unit", "Hydraulic Unit", "Hydraulic unit with tank, motor and valve block, and its hose to the brake caliper.",
         "hydraulic_unit"),
    part("cooling_unit", "Cooling Unit", "Cooling unit with pump and expansion tank.", "cooling_unit"),
    part("coolant_level", "Coolant Level Gauge", "Level gauge on the cooling unit's expansion tank, normal or low.",
         "cooling_level"),
    part("service_crane", "Nacelle Service Crane",
         "Crane rail under the nacelle roof with a travelling trolley and a hoist hook carrying a lifting platform.",
         "crane_rail", "crane_trolley", "crane_hook", "lifting_platform"),
    part("fire_extinguisher", "Fire Extinguisher", "Wall-mounted fire extinguisher, present or missing from its bracket.",
         "fire_extinguisher"),
    part("safety_equipment", "First-Aid and Emergency-Stop Boxes", "First-aid box and emergency-stop box on the nacelle walls by the tower hatch.",
         "safety_equipment"),
    part("tool_rack", "Tool Rack", "Steel tool rack with two shelves against the nacelle wall.", "tool_rack"),
    part("tool_tray", "Tool Tray", "Tool tray on a post on the bedplate girder.", "tool_tray"),
]

# Fault overlays whose parent link is not the part they damage (the nacelle link is
# shared by two objects; the cable chafe hangs off the tower section).
OVERLAY_OWNER = {"nacelle_fault_cover_crack": "nacelle_cover", "tower_fault_cable_chafe": "cable_loop"}
# Inspection (open-preview) positions where the upper limit is not a useful view.
INSPECT = {"crane_trolley_joint": 4.0, "crane_hook_joint": 2.5}
FRAME_PURPOSE = {"revolute": "hinged_part", "prismatic": "sliding_part", "continuous": "rotating_part"}


# ---------------------------------------------------------------- transforms

def rpy_matrix(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def pose(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    m = np.eye(4)
    m[:3, :3] = rpy_matrix(rpy)
    m[:3, 3] = xyz
    return m


def translation(xyz):
    return pose(xyz)


def matrix_rpy(m):
    r = m[:3, :3]
    pitch = math.asin(max(-1.0, min(1.0, -r[2, 0])))
    if abs(math.cos(pitch)) > 1e-9:
        return [math.atan2(r[2, 1], r[2, 2]), pitch, math.atan2(r[1, 0], r[0, 0])]
    return [math.atan2(-r[1, 2], r[1, 1]), pitch, 0.0]


def clean(values, digits=6):
    return [0.0 if abs(round(v, digits)) == 0 else round(float(v), digits) for v in values]


def pose_json(m):
    return {"xyz": clean(m[:3, 3]), "rpy": clean(matrix_rpy(m))}


def fmt(values):
    return " ".join(f"{v:.6g}" if abs(v) > 5e-13 else "0" for v in values)


def quaternion_matrix(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


# ---------------------------------------------------------------- GLB

COMPONENTS = {"VEC3": 3, "VEC4": 4}


def read_glb(path):
    data = path.read_bytes()
    magic, version, _ = struct.unpack_from("<III", data, 0)
    if magic != 0x46546C67 or version != 2:
        raise ValueError(f"Not a glTF 2.0 binary: {path}")
    json_length, json_type = struct.unpack_from("<II", data, 12)
    gltf = json.loads(data[20:20 + json_length])
    offset = 20 + json_length
    binary = bytearray()
    if offset < len(data):
        bin_length, _ = struct.unpack_from("<II", data, offset)
        binary = bytearray(data[offset + 8:offset + 8 + bin_length])
    return gltf, binary


def write_glb(path, gltf, binary):
    text = json.dumps(gltf, separators=(",", ":")).encode()
    text += b" " * (-len(text) % 4)
    binary = bytes(binary) + b"\0" * (-len(binary) % 4)
    total = 12 + 8 + len(text) + (8 + len(binary) if binary else 0)
    out = struct.pack("<III", 0x46546C67, 2, total) + struct.pack("<II", len(text), 0x4E4F534A) + text
    if binary:
        out += struct.pack("<II", len(binary), 0x004E4942) + binary
    path.write_bytes(out)


def accessor_view(gltf, binary, index):
    accessor = gltf["accessors"][index]
    if accessor.get("componentType") != 5126 or "sparse" in accessor:
        raise ValueError(f"Unsupported accessor {index}")
    view = gltf["bufferViews"][accessor["bufferView"]]
    width = COMPONENTS[accessor["type"]]
    stride = view.get("byteStride") or 4 * width
    return accessor, np.ndarray((accessor["count"], width), dtype="<f4", buffer=binary,
                                offset=view.get("byteOffset", 0) + accessor.get("byteOffset", 0), strides=(stride, 4))


def node_matrix(node):
    if "matrix" in node:
        return np.array(node["matrix"], dtype=float).reshape(4, 4).T
    m = np.eye(4)
    m[:3, :3] = quaternion_matrix(node.get("rotation", (0, 0, 0, 1))) @ np.diag(node.get("scale", (1, 1, 1)))
    m[:3, 3] = node.get("translation", (0, 0, 0))
    return m


def glb_vertices(path):
    """Vertices of a single-node GLB in its parent (link) frame."""
    gltf, binary = read_glb(path)
    node, = [n for n in gltf["nodes"] if "mesh" in n]
    m = node_matrix(node)
    points = []
    for index in {p["attributes"]["POSITION"] for p in gltf["meshes"][node["mesh"]]["primitives"]}:
        _, values = accessor_view(gltf, binary, index)
        points.append(values @ m[:3, :3].T + m[:3, 3])
    return np.concatenate(points)


def bake_glb(source, target, transform):
    """Copy a single-node GLB with ``transform @ node transform`` baked into its vertices."""
    gltf, binary = read_glb(source)
    nodes = gltf["nodes"]
    if len(nodes) != 1 or "mesh" not in nodes[0] or "children" in nodes[0] or "skins" in gltf:
        raise ValueError(f"Expected one mesh node without children or skins: {source}")
    node = nodes[0]
    m = transform @ node_matrix(node)
    linear = m[:3, :3]
    normal = np.linalg.inv(linear).T
    done = set()
    for primitive in gltf["meshes"][node["mesh"]]["primitives"]:
        if "targets" in primitive:
            raise ValueError(f"Morph targets are not supported: {source}")
        for name, index in primitive["attributes"].items():
            if index in done or name not in ("POSITION", "NORMAL", "TANGENT"):
                continue
            done.add(index)
            accessor, values = accessor_view(gltf, binary, index)
            if name == "POSITION":
                values[:] = values @ linear.T + m[:3, 3]
                accessor["min"] = [float(v) for v in values.min(axis=0)]
                accessor["max"] = [float(v) for v in values.max(axis=0)]
            else:
                rotated = values[:, :3] @ (normal if name == "NORMAL" else linear).T
                values[:, :3] = rotated / np.maximum(np.linalg.norm(rotated, axis=1, keepdims=True), 1e-12)
    for key in ("matrix", "translation", "rotation", "scale"):
        node.pop(key, None)
    gltf.setdefault("asset", {})["generator"] = "private_meshes export_turbine_parts.py"
    write_glb(target, gltf, binary)


def glb_placement(path):
    """Where a GLB's placement in its link frame lives: node transform, vertices or local."""
    gltf, _ = read_glb(path)
    node, = [n for n in gltf["nodes"] if "mesh" in n]
    if any(key in node for key in ("matrix", "translation", "rotation", "scale")):
        return "node transform"
    points = glb_vertices(path)
    lo, hi = points.min(axis=0), points.max(axis=0)
    margin = 0.05 * max(hi - lo) + 0.05
    return "local" if np.all(lo - margin <= 0) and np.all(hi + margin >= 0) else "vertices"


# ---------------------------------------------------------------- turbine model

class Turbine:
    def __init__(self, source: Path):
        self.source = source
        sys.path.insert(0, str(source))
        parts = importlib.import_module("turbine.parts")
        self.links, self.group, self.faults = {}, {}, {}
        for module in parts.ALL:
            group = module.__name__.rsplit(".", 1)[-1]
            for spec in module.LINKS:
                self.links[spec["name"]] = spec
                self.group[spec["name"]] = group
            for key, fault in module.FAULTS.items():
                self.faults[key] = fault
                for overlay in fault["overlays"]:
                    self.links[overlay["name"]] = overlay
                    self.group[overlay["name"]] = group
        self.world = {"world": np.eye(4)}
        for name in self.links:
            self.world_pose(name)

    def world_pose(self, name):
        if name not in self.world:
            spec = self.links[name]
            self.world[name] = self.world_pose(spec["parent"]) @ pose(spec["xyz"], spec["rpy"])
        return self.world[name]

    def glb(self, mesh):
        return self.source / "models" / Path(mesh).with_suffix(".glb")

    def meshes(self, name, only=None):
        """Healthy mesh(es) of a link and its variants {state: mesh}."""
        spec = self.links[name]
        variants = spec.get("variants") or {}
        mesh = spec.get("mesh")
        meshes = [variants[next(iter(variants))]] if variants else ([mesh] if isinstance(mesh, str) else list(mesh or []))
        if only is not None:
            meshes = [m for m in meshes if m in only]
        return meshes, variants

    def commit(self):
        try:
            return subprocess.run(["git", "-C", str(self.source), "rev-parse", "--short", "HEAD"],
                                  capture_output=True, text=True, check=True).stdout.strip()
        except (OSError, subprocess.CalledProcessError):
            return "unknown"


# ---------------------------------------------------------------- objects

def collision_xml(parent, spec, offset):
    xyz = np.asarray(spec.get("xyz", (0, 0, 0)), dtype=float) + offset
    lines = [f'    <collision><origin xyz="{fmt(xyz)}" rpy="{fmt(spec.get("rpy", (0, 0, 0)))}"/><geometry>']
    if spec["type"] == "box":
        lines.append(f'<box size="{fmt(spec["size"])}"/>')
    elif spec["type"] == "cylinder":
        lines.append(f'<cylinder radius="{spec["radius"]:.6g}" length="{spec["length"]:.6g}"/>')
    else:
        raise ValueError(f"Unsupported collision in {parent}: {spec['type']}")
    lines.append("</geometry></collision>")
    return "".join(lines)


def inside(point, lo, hi, margin=0.05):
    return bool(np.all(point >= lo - margin) and np.all(point <= hi + margin))


class Exporter:
    def __init__(self, turbine: Turbine):
        self.t = turbine
        self.objects = []
        owner = {}
        for index, spec in enumerate(PARTS):
            spec = dict(spec, id=str(FIRST_ID + index))
            spec["folder"] = f"{spec['id']}-{spec['slug']}"
            for link in spec["links"] + spec["instances"]:
                owner.setdefault(link, spec)
            self.objects.append(spec)
        self.owner = owner
        self.used_glbs = {}

    # -- geometry -----------------------------------------------------------
    def object_links(self, obj):
        """(turbine link, object link name, healthy meshes, variants) in URDF order."""
        result = []
        for link in obj["links"]:
            meshes, variants = self.t.meshes(link, obj["meshes"].get(link))
            result.append((link, obj["rename"].get(link, link), meshes, variants))
        return result

    def plan(self, obj):
        root = obj["links"][0]
        w_root = self.t.world[root]
        links = self.object_links(obj)
        root_points = np.concatenate([glb_vertices(self.t.glb(m)) for m in links[0][2]])
        lo, hi = root_points.min(axis=0), root_points.max(axis=0)
        margin = 0.05 * max(hi - lo) + 0.05
        if np.all(lo - margin <= 0) and np.all(hi + margin >= 0):
            offset = np.zeros(3)          # the link origin is on the part (blade root, door hinge)
        else:
            offset = np.round([(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, lo[2]], 4)
        shift = translation(-offset)
        # link frames in the object frame (zero joint positions)
        frames = {link: shift @ np.linalg.inv(w_root) @ self.t.world[link] for link, *_ in links}
        obj.update(root=root, offset=offset, frames=frames, object_links=links)
        obj["variants"] = self.variants(obj)
        points = np.concatenate([glb_vertices(self.t.glb(m)) @ frames[link][:3, :3].T + frames[link][:3, 3]
                                 for link, _, meshes, _ in links for m in meshes])
        obj["bounds"] = points.min(axis=0), points.max(axis=0)

    def variants(self, obj):
        """Fault states of this object: {name: {"swap": {link: mesh}, "overlays": [overlay link]}}."""
        names = {link for link, *_ in obj["object_links"]}
        result = {}
        for key, fault in self.t.faults.items():
            swaps = {link: self.t.links[link]["variants"][state]
                     for link, state in fault["set_variants"].items() if link in names}
            overlays = [o["name"] for o in fault["overlays"]]
            if overlays:
                first = self.t.links[overlays[0]]["parent"]
                owner = OVERLAY_OWNER.get(overlays[0], self.owner.get(first, {}).get("slug"))
                if owner != obj["slug"] or (first not in names and overlays[0] not in OVERLAY_OWNER):
                    overlays = []       # another object's, or on a further blade instance
            if not swaps and not overlays:
                continue
            name = key.split(".", 1)[1]
            result[name] = {"fault": key, "description": fault["description"], "severity": fault["severity"],
                            "swap": swaps, "overlays": overlays}
        return result

    def overlay_host(self, obj, overlay):
        parent = self.t.links[overlay]["parent"]
        names = {link for link, *_ in obj["object_links"]}
        return parent if parent in names else obj["root"]

    # -- writing ------------------------------------------------------------
    def write(self, obj):
        folder = OUTPUT / obj["folder"]
        folder.mkdir(parents=True, exist_ok=True)
        for stale in folder.iterdir():
            if stale.name != "renders":
                shutil.rmtree(stale) if stale.is_dir() else stale.unlink()
        (folder / "assets").mkdir()
        root, offset = obj["root"], obj["offset"]
        shift = translation(-offset)
        assets = {}

        def asset(mesh, link, transform):
            name = Path(mesh).stem + ".glb"
            if name not in assets:
                bake_glb(self.t.glb(mesh), folder / "assets" / name, transform)
                assets[name] = mesh
                self.used_glbs[Path(mesh).with_suffix(".glb").as_posix()] = (obj, link)
            return "assets/" + name

        healthy = {}
        for link, _, meshes, variants in obj["object_links"]:
            to_link = shift if link == root else np.eye(4)
            healthy[link] = [asset(m, link, to_link) for m in meshes]
            for mesh in list(variants.values())[1:]:
                asset(mesh, link, to_link)
        overlay_assets = {}
        for variant in obj["variants"].values():
            for overlay in variant["overlays"]:
                host = self.overlay_host(obj, overlay)
                w_host = self.t.world[host] if host != root else self.t.world[root] @ translation(offset)
                transform = np.linalg.inv(w_host) @ self.t.world[overlay]
                overlay_assets[overlay] = (host, asset(self.t.links[overlay]["mesh"], overlay, transform))

        (folder / "model.urdf").write_text(self.urdf(obj, healthy))
        for name, variant in obj["variants"].items():
            visuals = {link: list(paths) for link, paths in healthy.items()}
            for link, mesh in variant["swap"].items():
                visuals[link] = ["assets/" + Path(mesh).stem + ".glb"]
            for overlay in variant["overlays"]:
                host, path = overlay_assets[overlay]
                visuals[host].append(path)
            (folder / f"model_{name}.urdf").write_text(self.urdf(obj, visuals, variant=name))
        self.write_metadata(obj, folder)
        self.write_notices(obj, folder)
        (folder / "README.md").write_text(self.readme(obj))

    def joints(self, obj):
        result = []
        for link, _, _, _ in obj["object_links"][1:]:
            spec = self.t.links[link]
            if spec["joint"] == "fixed":
                continue
            name = f"{obj['rename'].get(link, link)}_joint"
            lower, upper = (spec["limits"] if spec["limits"] else (None, None))
            entry = {"name": name, "type": spec["joint"],
                     "lower": None if lower is None else float(lower),
                     "upper": None if upper is None else float(upper)}
            entry["inspect"] = INSPECT.get(name, entry["upper"] if upper is not None else 0.0)
            result.append(entry)
        return result

    def urdf(self, obj, visuals, variant=None):
        root, offset = obj["root"], obj["offset"]
        name = obj["slug"] + (f"_{variant}" if variant else "")
        lines = ['<?xml version="1.0"?>',
                 f"<!-- Generated by private_meshes scripts/export_turbine_parts.py from {SOURCE_REPO}@{SOURCE_BRANCH}. "
                 f"Author: {AUTHOR}. Licence: {LICENSE_ID} ({LICENSE_STATUS}); see NOTICE.md. -->",
                 f'<robot name="{name}">']
        names = {link for link, *_ in obj["object_links"]}
        for link, link_name, _, _ in obj["object_links"]:
            spec = self.t.links[link]
            local = -offset if link == root else np.zeros(3)
            lines.append(f'  <link name="{link_name}">')
            lines.append('    <inertial><origin xyz="0 0 0" rpy="0 0 0"/><mass value="1.0"/>'
                         '<inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>')
            for path in visuals[link]:
                lines.append(f'    <visual><origin xyz="0 0 0" rpy="0 0 0"/><geometry><mesh filename="{path}"/></geometry></visual>')
            for collision in self.collisions(obj, link):
                lines.append(collision_xml(link_name, collision, local))
            lines.append("  </link>")
            if link != root:
                parent = spec["parent"]
                if parent not in names:
                    raise ValueError(f"{obj['slug']}: parent {parent} of {link} is not in the object")
                origin = pose(spec["xyz"], spec["rpy"])
                if parent == root:
                    origin = translation(-offset) @ origin
                joint_name = f"{link_name}_joint"
                lines.append(f'  <joint name="{joint_name}" type="{spec["joint"]}">')
                lines.append(f'    <parent link="{obj["rename"].get(parent, parent)}"/><child link="{link_name}"/>')
                lines.append(f'    <origin xyz="{fmt(origin[:3, 3])}" rpy="{fmt(matrix_rpy(origin))}"/>')
                if spec["joint"] != "fixed":
                    lines.append(f'    <axis xyz="{fmt(spec["axis"])}"/>')
                if spec["limits"]:
                    lines.append(f'    <limit lower="{spec["limits"][0]:.6g}" upper="{spec["limits"][1]:.6g}" '
                                 f'effort="100" velocity="{spec.get("velocity", 1.0):.6g}"/>')
                lines.append("  </joint>")
        lines.append("</robot>")
        return "\n".join(lines) + "\n"

    def collisions(self, obj, link):
        collisions = self.t.links[link].get("collisions", [])
        only = obj["meshes"].get(link)
        if only is None:
            return collisions
        # A link split over several objects: each box goes to the first of the link's
        # meshes whose bounds hold its centre (or to the first mesh if none does).
        meshes = self.t.meshes(link)[0]
        bounds = [(lambda p: (p.min(axis=0), p.max(axis=0)))(glb_vertices(self.t.glb(m))) for m in meshes]

        def holder(c):
            centre = np.asarray(c.get("xyz", (0, 0, 0)), dtype=float)
            return next((m for m, (lo, hi) in zip(meshes, bounds) if inside(centre, lo, hi)), meshes[0])
        return [c for c in collisions if holder(c) in only]

    def instances(self, obj):
        root, offset = obj["root"], obj["offset"]
        result = []
        for link in [root] + obj["instances"]:
            spec = self.t.links[link]
            origin = self.t.world[link] @ translation(offset)
            parent = spec["parent"]
            parent_obj = self.owner.get(parent)
            entry = {"turbineLink": link, "parentLink": parent,
                     "joint": {"name": f"{link}_joint", "type": spec["joint"]},
                     "originInTurbine": pose_json(origin),
                     "originInParentLink": pose_json(np.linalg.inv(self.t.world[parent]) @ origin)}
            if spec["joint"] != "fixed":
                entry["joint"]["axis"] = clean(spec["axis"])
                entry["joint"]["originInObject"] = clean(-offset)
                if spec["limits"]:
                    entry["joint"]["limits"] = clean(spec["limits"])
            if parent_obj is not None:
                parent_origin = self.t.world[parent_obj["root"]] @ translation(parent_obj["offset"])
                entry["parentObject"] = parent_obj["folder"]
                entry["originInParentObject"] = pose_json(np.linalg.inv(parent_origin) @ origin)
            result.append(entry)
        return result

    def write_metadata(self, obj, folder):
        lo, hi = obj["bounds"]
        variants = {"healthy": {"urdf": "model.urdf"}}
        for name, variant in obj["variants"].items():
            entry = {"urdf": f"model_{name}.urdf", "fault": variant["fault"], "severity": variant["severity"],
                     "description": variant["description"]}
            if variant["swap"]:
                entry["replaces"] = [{"link": obj["rename"].get(link, link), "asset": "assets/" + Path(mesh).stem + ".glb"}
                                     for link, mesh in variant["swap"].items()]
            if variant["overlays"]:
                entry["overlays"] = [{"asset": "assets/" + Path(self.t.links[o]["mesh"]).stem + ".glb",
                                      "link": obj["rename"].get(self.overlay_host(obj, o), self.overlay_host(obj, o))}
                                     for o in variant["overlays"]]
            variants[name] = entry
        metadata = {
            "object": obj["slug"],
            "rootLink": obj["rename"].get(obj["root"], obj["root"]),
            "units": "metres", "upAxis": "Z",
            "origin": ("the turbine link's origin, which lies on the part" if not obj["offset"].any() else
                       "centre of the root mesh's footprint (X/Y bounding-box centre, lowest Z)")
                      + ", axes parallel to the turbine link frame",
            "bounds": {"min": clean(lo), "max": clean(hi)},
            "links": {obj["rename"].get(link, link): link for link, *_ in obj["object_links"]},
            "turbine": {
                "source": f"{SOURCE_REPO}@{SOURCE_BRANCH}",
                "commit": self.t.commit(),
                "group": self.t.group[obj["root"]],
                "frame": "world frame of windturbine_description (tower axis at the origin, ground at z = 0), "
                         "all joints at zero",
                "instances": self.instances(obj),
            },
            "variants": variants,
            "assetRights": self.rights(),
        }
        (folder / "metadata.json").write_text(json.dumps(metadata, indent=2, ensure_ascii=False) + "\n")
        (folder / "rights.json").write_text(json.dumps(self.rights(), indent=2, ensure_ascii=False) + "\n")

    def rights(self):
        return {
            "author": AUTHOR,
            "source": f"https://github.com/{SOURCE_REPO}/tree/{SOURCE_BRANCH}",
            "copyright": COPYRIGHT,
            "license": LICENSE_ID,
            "licenseStatus": LICENSE_STATUS,
            "dimensionsFrom": {"name": IEA[0], "url": IEA[1], "license": IEA[2]},
            "textures": {"source": "ambientCG", "license": "CC0-1.0", "sets": list(TEXTURES)},
            "licenseFile": "LICENSE.txt",
            "noticeFile": "NOTICE.md",
        }

    def write_notices(self, obj, folder):
        textures = ", ".join(f"[{name}](https://ambientcg.com/view?id={name})" for name in TEXTURES)
        (folder / "NOTICE.md").write_text(f"""# Notice: {obj['name']}

Part of the wind turbine model by {AUTHOR}, generated from
[{SOURCE_REPO}](https://github.com/{SOURCE_REPO}/tree/{SOURCE_BRANCH}) (branch `{SOURCE_BRANCH}`)
and exported to this collection with `scripts/export_turbine_parts.py`.

Dimensions derive from the [{IEA[0]}]({IEA[1]}), licensed under {IEA[2]}.
This model is a simplified visual model, not a replica of that design or of any manufacturer's turbine.

Textures are made from CC0 1.0 material sets by [ambientCG](https://ambientcg.com): {textures}.

{COPYRIGHT}. Licensed under the BSD 3-Clause License (SPDX: {LICENSE_ID}), see LICENSE.txt
in this folder. Licence status: {LICENSE_STATUS}. Keep this NOTICE.md and LICENSE.txt with every copy.
""")
        (folder / "LICENSE.txt").write_text(BSD3.format(copyright=COPYRIGHT))

    def readme(self, obj):
        lo, hi = obj["bounds"]
        size = hi - lo
        joints = self.joints(obj)
        instances = self.instances(obj)
        lines = [f"# {obj['name']}", "", obj["desc"], "",
                 f"Part of the wind turbine model by {AUTHOR} (IEA-3.4-130-RWT dimensions), exported from "
                 f"`{SOURCE_REPO}` branch `{SOURCE_BRANCH}`. Generated by `scripts/export_turbine_parts.py`; "
                 "do not edit by hand.", "",
                 "`model.urdf` loads independently using only local `assets/*.glb` files; each fault state has its "
                 "own `model_<state>.urdf`.",
                 "Licensed under BSD-3-Clause, pending confirmation by the model's author (see LICENSE.txt and NOTICE.md); "
                 "keep both with every copy.", "",
                 f"Coordinates are metres, Z up. Size {size[0]:.3f} × {size[1]:.3f} × {size[2]:.3f} m (X × Y × Z). "
                 + ("Origin at the turbine link's origin" if not obj["offset"].any() else
                    "Origin at the centre of the root mesh's footprint (bottom)")
                 + f", axes parallel to the turbine's `{obj['root']}` link frame.",
                 "Use a unique URDF parser prefix for each instance. Masses and inertias are placeholders "
                 "(1 kg, unit inertia), as in the turbine model; collisions are its boxes and cylinders.", ""]
        if joints:
            lines += ["## Moving parts", "", "| Joint | Type | Limits | Inspection position |", "| --- | --- | --- | --- |"]
            lines += [f"| `{j['name']}` | {j['type']} | {j['lower']} … {j['upper']} | {j['inspect']} |" for j in joints]
            lines.append("")
        lines += ["## States", "", "| State | URDF | Fault | Description |", "| --- | --- | --- | --- |",
                  "| healthy | `model.urdf` | — | As built. |"]
        lines += [f"| {name} | `model_{name}.urdf` | `{v['fault']}` ({v['severity']}) | {v['description']} |"
                  for name, v in obj["variants"].items()]
        lines += ["", "## Place in the turbine", "",
                  "`metadata.json` records where this object sits in the assembled turbine "
                  "(all joints at zero), so the turbine can be rebuilt from the objects.", "",
                  "| Turbine link | Parent link | Joint | Origin in turbine (xyz, m) |", "| --- | --- | --- | --- |"]
        lines += [f"| `{i['turbineLink']}` | `{i['parentLink']}` | {i['joint']['type']} | "
                  f"{', '.join(f'{v:.3f}' for v in i['originInTurbine']['xyz'])} |" for i in instances]
        lines += ["", "```python", "from semantic_digital_twin.adapters.urdf import URDFParser", "",
                  f'world = URDFParser.from_file("/path/to/private_meshes/objects/{COLLECTION}/{obj["folder"]}/model.urdf",',
                  f'                             prefix="{obj["slug"]}_one_").parse()', "```", ""]
        return "\n".join(lines)

    def catalogue_entry(self, obj):
        lo, hi = obj["bounds"]
        size = clean(hi - lo, 4)
        joints = self.joints(obj)
        frames = [{"name": obj["rename"].get(obj["root"], obj["root"]), "purpose": "base_origin"}]
        frames += [{"name": j["name"][:-len("_joint")], "purpose": FRAME_PURPOSE[j["type"]]} for j in joints]
        kinds = {}
        for j in joints:
            kinds[j["type"]] = kinds.get(j["type"], 0) + 1
        return {
            "id": obj["id"], "slug": obj["slug"], "name": obj["name"], "desc": obj["desc"], "size": size,
            "size_basis": "Visual bounds of the healthy state at zero joint positions; metres, X/Y/Z.",
            "joints": joints, "frames": frames, "category": COLLECTION,
            "mobility": "articulated" if joints else "rigid", "added": ADDED,
            "fingerprint": {"type": obj["name"].lower(), "joints": kinds, "size_m": size},
            "variants": ["healthy", *obj["variants"]],
            "variant_of": None, "folder": obj["folder"],
        }

    # -- collection ---------------------------------------------------------
    def run(self, only=None):
        OUTPUT.mkdir(parents=True, exist_ok=True)
        for obj in self.objects:
            self.plan(obj)
        for obj in self.objects:
            if only and obj["slug"] not in only:
                continue
            self.write(obj)
            print(f"{obj['id']} {obj['slug']}: {len(obj['object_links'])} links, {len(obj['variants'])} fault states")
        if only:
            return
        keep = {obj["folder"] for obj in self.objects}
        for folder in OUTPUT.iterdir():
            if folder.is_dir() and re.fullmatch(r"\d+-.+", folder.name) and folder.name not in keep:
                shutil.rmtree(folder)
        manifest = {"categories": [dict(CATEGORY, count=len(self.objects))],
                    "objects": [self.catalogue_entry(obj) for obj in self.objects]}
        (OUTPUT / "collection.json").write_text(json.dumps(manifest, indent=1, ensure_ascii=False) + "\n")
        (OUTPUT / "README.md").write_text(self.collection_readme())
        (OUTPUT / "INVENTORY.md").write_text(self.inventory())

    def collection_readme(self):
        lines = ["# Wind turbine parts", "", CATEGORY["blurb"], "",
                 f"{len(self.objects)} objects from the wind turbine model by {AUTHOR} "
                 f"(`{SOURCE_REPO}`, branch `{SOURCE_BRANCH}`). Each folder holds `model.urdf`, one "
                 "`model_<state>.urdf` per fault state, `assets/*.glb`, `metadata.json` with the part's placement "
                 "in the assembled turbine, `README.md`, `NOTICE.md`, `LICENSE.txt`, `rights.json` and "
                 "`renders/rest.webp` (plus `renders/open.webp` for parts with moving joints).", "",
                 "The folders are generated; do not edit them by hand. To regenerate after a new turbine export:", "",
                 "```sh", "python3 scripts/export_turbine_parts.py --source /path/to/cognitive_robot_abstract_machine",
                 "blender --background --factory-startup --python scripts/render_turbine.py", "```", "",
                 "[INVENTORY.md](INVENTORY.md) lists every source GLB and where it went.", "",
                 "| Preview | Object | Size W × D × H (m) | Moving joints | Fault states |", "| --- | --- | --- | --- | --- |"]
        for obj in self.objects:
            lo, hi = obj["bounds"]
            s = hi - lo
            joints = self.joints(obj)
            lines.append(f"| <img src=\"./{obj['folder']}/renders/rest.webp\" width=\"120\"> | "
                         f"**[#{obj['id']} {obj['name']}](./{obj['folder']}/)**<br>{obj['desc']} | "
                         f"{s[0]:.2f} × {s[1]:.2f} × {s[2]:.2f} | {len(joints) or 'none'} | "
                         f"{', '.join(obj['variants']) or '—'} |")
        return "\n".join(lines) + "\n"

    def inventory(self):
        models = self.t.source / "models"
        lines = ["# Inventory of the turbine GLBs", "",
                 f"Source: `{SOURCE_REPO}` branch `{SOURCE_BRANCH}`, commit `{self.t.commit()}`, `models/<group>/*.glb`. "
                 "Generated by `scripts/export_turbine_parts.py`.", "",
                 "*Placement* says where the GLB's position in its turbine link frame lives before export: "
                 "in the glTF node transform, baked into the vertices, or local (the link origin lies in or at "
                 "the mesh). The export bakes all three into the vertices relative to the object's origin.", "",
                 "| Group | GLB | KB | Turbine link | Role | Placement | Object |", "| --- | --- | --- | --- | --- | --- | --- |"]
        roles = {}
        for name, spec in self.t.links.items():
            meshes, variants = self.t.meshes(name)
            if spec.get("mesh") and name in {o["name"] for f in self.t.faults.values() for o in f["overlays"]}:
                roles.setdefault(Path(spec["mesh"]).with_suffix(".glb").as_posix(), []).append((name, "fault overlay"))
                continue
            for mesh in meshes:
                roles.setdefault(Path(mesh).with_suffix(".glb").as_posix(), []).append((name, "healthy" if variants else "part"))
            for state, mesh in list(variants.items())[1:]:
                roles.setdefault(Path(mesh).with_suffix(".glb").as_posix(), []).append((name, f"faulty: {state}"))
        count = 0
        for path in sorted(models.glob("*/*.glb")):
            rel = path.relative_to(models).as_posix()
            count += 1
            uses = roles.get(rel, [])
            links = ", ".join(sorted({f"`{u[0]}`" for u in uses})) or "—"
            role = ", ".join(sorted({u[1] for u in uses})) or "not in the turbine URDF"
            if rel in EXCLUDED:
                target = f"left out: {EXCLUDED[rel]}"
            elif rel in self.used_glbs:
                obj, _ = self.used_glbs[rel]
                target = f"[#{obj['id']} {obj['name']}]({obj['folder']}/)"
            else:
                target = "**not exported**"
            lines.append(f"| {path.parent.name} | `{path.name}` | {path.stat().st_size // 1024} | {links} | {role} | "
                         f"{glb_placement(path)} | {target} |")
        missing = [rel for rel in roles if rel not in self.used_glbs and rel not in EXCLUDED]
        lines += ["", f"{count} GLBs; {len(self.used_glbs)} exported into {len(self.objects)} objects, "
                      f"{len(EXCLUDED)} left out."]
        if missing:
            lines.append("Referenced by the turbine but not exported: " + ", ".join(f"`{m}`" for m in sorted(missing)))
        return "\n".join(lines) + "\n"



BSD3 = """BSD 3-Clause License

{copyright}
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--source", type=Path, required=True,
                        help="checkout of cognitive_robot_abstract_machine on branch windturbine_maintenance")
    parser.add_argument("--only", nargs="*", help="export only these slugs (no collection files)")
    args = parser.parse_args()
    Exporter(Turbine(args.source.resolve())).run(set(args.only) if args.only else None)


if __name__ == "__main__":
    main()
