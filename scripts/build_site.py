#!/usr/bin/env python3
"""Build the object catalogue using Python's standard library only."""
from __future__ import annotations

import json
import hashlib
import math
import re
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path, PurePosixPath
from urllib.parse import quote
from zipfile import ZIP_DEFLATED, ZipFile, ZipInfo


# Exported collections (all but "articulated") use their name as their only category key.
COLLECTIONS = ("articulated", "laboratory", "wind-turbine")


def local_path(base: Path, value: str) -> Path:
    """Accept only ordinary paths contained in base, never symlinks."""
    path = PurePosixPath(value)
    if not path.parts or path.is_absolute() or ".." in path.parts or "\\" in value or ":" in value:
        raise ValueError(f"Unsafe relative path: {value!r}")
    candidate = base.joinpath(*path.parts)
    if not candidate.resolve().is_relative_to(base.resolve()):
        raise ValueError(f"Path escapes source directory: {value}")
    if any(part.is_symlink() for part in (candidate, *candidate.parents)):
        raise ValueError(f"Symlinks are not supported: {value}")
    return candidate


def public_folder(obj: dict) -> str:
    """Keep the established articulated URLs and namespace other collections."""
    prefix = "" if obj["collection"] == "articulated" else obj["collection"] + "/"
    return prefix + obj["folder"]


def validate_joints(obj: dict, robot: ET.Element) -> str:
    """Classify actual movable joints and reject stale catalogue descriptions."""
    ident = obj["id"]
    joints = {j.attrib["name"]: j for j in robot.findall("joint")
              if j.attrib.get("type") != "fixed"}
    declared = {j.get("name"): j for j in obj.get("joints", [])}
    if len(declared) != len(obj.get("joints", [])) or declared.keys() != joints.keys():
        raise ValueError(f"Catalogue/URDF joint mismatch for {ident}")
    for name, joint in joints.items():
        entry = declared[name]
        kind = joint.attrib.get("type")
        if entry.get("type") != kind:
            raise ValueError(f"Catalogue/URDF joint type mismatch for {ident}: {name}")
        limit = joint.find("limit")
        for bound in ("lower", "upper"):
            expected = None if kind == "continuous" else (
                float(limit.attrib[bound]) if limit is not None and bound in limit.attrib else None)
            value = entry.get(bound)
            if expected is None:
                matches = value is None
            else:
                matches = (isinstance(value, (float, int)) and not isinstance(value, bool)
                           and math.isclose(value, expected, abs_tol=1e-6))
            if not matches:
                raise ValueError(f"Catalogue/URDF joint limit mismatch for {ident}: {name}/{bound}")
    mobility = "articulated" if joints else "rigid"
    if obj.get("mobility", mobility) != mobility:
        raise ValueError(f"Catalogue/URDF mobility mismatch for {ident}")
    return mobility


def read_catalogue(root: Path) -> dict:
    data = {"categories": [], "objects": []}
    categories, ids, public_folders = {}, set(), set()
    # An optional collection is absent only when its entire source directory is absent.
    for collection in COLLECTIONS:
        source = root / "objects" / collection
        if not source.exists() and collection != "articulated":
            continue
        manifest = local_path(source, "collection.json")
        if not manifest.is_file():
            raise ValueError(f"Missing {collection}/collection.json")
        current = json.loads(manifest.read_text(encoding="utf-8"))
        for category in current["categories"]:
            key = category["key"]
            if not re.fullmatch(r"[a-z0-9-]+", key):
                raise ValueError(f"Invalid category key: {key}")
            if key in categories:
                raise ValueError(f"Duplicate category keys: {key}")
            categories[key] = category
            data["categories"].append(category)
        collection_categories = {c["key"] for c in current["categories"]}
        folders = set()
        for obj in current["objects"]:
            validate_object(source, collection, collection_categories, obj, ids, folders, public_folders)
            data["objects"].append(obj)
        actual = {p.parent.relative_to(source).as_posix() for p in source.rglob("model.urdf")}
        if actual != folders:
            raise ValueError(f"Catalogue/folder mismatch in {collection}: {sorted(actual ^ folders)}")
    for category in data["categories"]:
        category["count"] = sum(o["category"] == category["key"] for o in data["objects"])
    data["stats"] = {
        "objects": len(data["objects"]), "categories": len(categories),
        "joints": sum(len(o.get("joints", [])) for o in data["objects"]),
        "frames": sum(len(o.get("frames", [])) for o in data["objects"]),
    }
    return data


def validate_object(source: Path, collection: str, categories: set, obj: dict,
                    ids: set, folders: set, public_folders: set) -> None:
    ident = obj["id"]
    if not isinstance(ident, str) or not re.fullmatch(r"[0-9]+", ident):
        raise ValueError(f"Invalid object ID: {ident}")
    if ident in ids or obj["folder"] in folders:
        raise ValueError(f"Duplicate object ID or folder: {ident}")
    ids.add(ident)
    folders.add(obj["folder"])
    if obj["category"] not in categories:
        raise ValueError(f"Unknown category for {ident}")
    folder = local_path(source, obj["folder"])
    if collection == "articulated" and folder.relative_to(source).parts[0] != obj["category"]:
        raise ValueError(f"Category/folder mismatch for {ident}")
    if collection != "articulated" and obj["category"] != collection:
        raise ValueError(f"Category/folder mismatch for {ident}")
    required_files = ["model.urdf", "README.md", "NOTICE.md"]
    if collection == "articulated":
        required_files.append("model.blend")
    else:
        required_files.extend(("LICENSE.txt", "rights.json", "metadata.json"))
    for required in required_files:
        if not local_path(folder, required).is_file():
            raise ValueError(f"Missing {obj['folder']}/{required}")
    for file in folder.rglob("*"):
        local_path(folder, file.relative_to(folder).as_posix())
    robot = ET.parse(folder / "model.urdf").getroot()
    if collection != "articulated":
        links = {link.attrib["name"] for link in robot.findall("link")}
        if any(frame.get("name") not in links for frame in obj.get("frames", [])):
            raise ValueError(f"Catalogue/URDF frame mismatch for {ident}")
    mesh_formats = set()
    for mesh in robot.iter("mesh"):
        filename = mesh.attrib.get("filename", "")
        if not local_path(folder, filename).is_file():
            raise ValueError(f"Missing mesh for {ident}: {filename}")
        mesh_formats.add(PurePosixPath(filename).suffix.lower())
    obj["mobility"] = validate_joints(obj, robot)
    obj["formats"] = ["URDF"]
    if ".glb" in mesh_formats:
        obj["formats"].append("GLB")
    if (folder / "model.blend").is_file():
        obj["formats"].append("Blender")
    obj["previews"] = {"rest": "renders/rest.webp"}
    if obj["mobility"] == "articulated":
        obj["previews"]["open"] = "renders/open.webp"
    for preview in obj["previews"].values():
        if not local_path(folder, preview).is_file():
            raise ValueError(f"Missing {obj['folder']}/{preview}")
    obj["collection"] = collection
    destination = public_folder(obj)
    if destination in public_folders:
        raise ValueError(f"Duplicate public object folder: {destination}")
    public_folders.add(destination)
    obj["url"] = "models/" + quote(destination, safe="/")
    obj["download"] = f"downloads/{ident}-{folder.name.split('-', 1)[-1]}.zip"


def add_to_zip(archive: ZipFile, file: Path, name: str) -> None:
    info = ZipInfo(name, date_time=(2020, 1, 1, 0, 0, 0))
    info.compress_type = ZIP_DEFLATED
    info.external_attr = 0o100644 << 16
    archive.writestr(info, file.read_bytes())


def build(root: Path) -> dict:
    # Validate before touching the previous build. Output is always root/dist.
    root = root.resolve()
    data = read_catalogue(root)
    catalogue = json.dumps(data, ensure_ascii=False).encode("utf-8")
    assets = {name: (root / "site" / name).read_bytes()
              for name in ("index.html", "styles.css", "app.js")}
    # One content-derived version keeps the application and its catalogue in sync.
    version = hashlib.sha256(b"\0".join((catalogue, assets["app.js"], assets["styles.css"]))).hexdigest()[:16]
    for name in ("app.js", "styles.css"):
        assets["index.html"] = assets["index.html"].replace(
            f'"{name}"'.encode(), f'"{name}?v={version}"'.encode())
    assets["app.js"] = assets["app.js"].replace(b"__CATALOGUE_VERSION__", version.encode())
    output = root / "dist"
    if output.is_symlink():
        raise ValueError("dist must not be a symlink")
    if output.exists():
        shutil.rmtree(output)
    output.mkdir()
    for name, content in assets.items():
        (output / name).write_bytes(content)
    (output / ".nojekyll").touch()
    (output / "downloads").mkdir()
    (output / "catalogue.json").write_bytes(catalogue)
    shutil.copyfile(root / "NOTICE.md", output / "NOTICE.md")
    shutil.copyfile(root / "LICENSE.txt", output / "LICENSE.txt")
    for obj in data["objects"]:
        folder = root / "objects" / obj["collection"] / obj["folder"]
        target = output / "models" / public_folder(obj)
        (target / "renders").mkdir(parents=True)
        names = [*obj["previews"].values(), "README.md", "NOTICE.md"]
        # Preserve existing preview URLs even when a rigid asset no longer advertises an open pose.
        if "open" not in obj["previews"] and (folder / "renders/open.webp").is_file():
            names.append("renders/open.webp")
        names.extend(name for name in ("LICENSE.txt", "rights.json", "metadata.json")
                     if (folder / name).is_file())
        for name in names:
            shutil.copyfile(folder / name, target / name)
        with ZipFile(output / obj["download"], "w") as archive:
            for file in sorted(folder.rglob("*")):
                if file.is_file():
                    add_to_zip(archive, file, f"{folder.name}/{file.relative_to(folder).as_posix()}")
            # Preserve the repository's licence scope as well as the object notices.
            add_to_zip(archive, root / "NOTICE.md", f"{folder.name}/_repository/NOTICE.md")
            add_to_zip(archive, root / "LICENSE.txt", f"{folder.name}/_repository/LICENSE.txt")
    return data["stats"]


if __name__ == "__main__":
    stats = build(Path(__file__).resolve().parents[1])
    print(f"Built {stats['objects']} objects in {stats['categories']} categories into dist/")
