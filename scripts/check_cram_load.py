#!/usr/bin/env python3
"""Load object URDFs with CRAM's URDFParser and report what it builds.

Run with a Python that can import semantic_digital_twin, for example:
    python scripts/check_cram_load.py                      # objects/wind-turbine, all states
    python scripts/check_cram_load.py objects/laboratory   # any folder holding model.urdf files

Every model.urdf and model_<state>.urdf below the given folders is parsed with
URDFParser.from_file(...) under a distinct prefix and validated; the joint names
CRAM sees must match the URDF's movable joints. Exits non-zero if any file fails.
"""
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

from semantic_digital_twin.adapters.urdf import URDFParser

ROOT = Path(__file__).resolve().parents[1]


def check(path: Path, index: int):
    world = URDFParser.from_file(str(path), prefix=f"check_{index}_").parse()
    world.validate()
    robot = ET.parse(path).getroot()
    expected = {j.get("name") for j in robot.findall("joint") if j.get("type") != "fixed"}
    movable = {c.name.name.split("/")[-1].removeprefix(f"check_{index}_") for c in world.connections
               if type(c).__name__ != "FixedConnection"}
    missing = {name for name in expected if not any(m.endswith(name) for m in movable)}
    if missing:
        raise ValueError(f"joints not built: {sorted(missing)}")
    return len(world.kinematic_structure_entities), len(world.connections), len(expected)


def main():
    folders = [ROOT / arg for arg in sys.argv[1:]] or [ROOT / "objects" / "wind-turbine"]
    paths = sorted(p for folder in folders for p in folder.rglob("model*.urdf"))
    failed = 0
    for index, path in enumerate(paths):
        try:
            bodies, connections, joints = check(path, index)
            print(f"OK    {path.relative_to(ROOT)}  bodies={bodies} connections={connections} movable={joints}")
        except Exception as error:  # report every file, not just the first failure
            failed += 1
            print(f"FAIL  {path.relative_to(ROOT)}  {type(error).__name__}: {error}")
    print(f"{len(paths) - failed} of {len(paths)} URDFs load in CRAM")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
