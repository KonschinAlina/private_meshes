# private_meshes

Reusable 3D objects and assembled environments, developed for AICOR Solutions.

**Author and design direction:** Vanessa Hassouna (33067562+sunava@users.noreply.github.com).
Licensed under the [BSD 3-Clause License](LICENSE.txt) within the scope described
in [NOTICE.md](NOTICE.md). Objects #001–#150 retain their pending licence status.

## Asset layout

```text
objects/                         independently loadable models
  articulated/                   164 models, organised by category
    collection.json              web-catalogue metadata
    <category>/<id>-<slug>/       Blender, URDF, meshes, previews and notices
  laboratory/                    five portable laboratory devices, articulated and rigid
    collection.json              laboratory web-catalogue metadata
    centrifuge/                  includes manuals/v1.md and manuals/v2.md
    dual_dispenser/
    refrigerator/
    scale/
    tube_rack/
  wind-turbine/                  57 wind turbine parts, generated from the turbine model
    collection.json              wind turbine web-catalogue metadata
    INVENTORY.md                 every source GLB and the object it went into
    <id>-<slug>/                 URDF per healthy/faulty state, meshes, placement, notices
environments/                    assembled scenes and their configuration
  aicor_medical_lab/             room, equipment, placements and sample vessels
```

- **[Objects](objects/README.md):** copy a complete model folder to use an object
  independently. Its URDF keeps relative references to its own meshes and notices.
- **[Environments](environments/README.md):** load an assembled scene with its
  placements, configuration and supporting assets. The medical laboratory is a
  complete environment, even though it contains individually movable objects.
- **[Web catalogue](https://sunava.github.io/private_meshes/):** browse all 162
  models, filter articulated or rigid objects, compare available poses and
  download complete model folders. The Laboratory category includes all five devices;
  the Wind Turbine category holds the turbine's parts.

Model geometry, joint names, materials and scene placements are preserved by the
folder reorganisation. The laboratory environment remains a self-contained
runtime bundle; the portable device exports can be used without it.

## Load an individual model

```python
from semantic_digital_twin.adapters.urdf import URDFParser

world = URDFParser.from_file(
    "/path/to/private_meshes/objects/laboratory/centrifuge/model.urdf",
    prefix="centrifuge_one_",
).parse()
```

Use a distinct prefix for each instance. Coordinates are metres with Z up.
URDF loading supplies geometry and articulation; instrument controllers and
fluid simulation belong to the consuming laboratory application.

## Existing consumers and maintenance

The former top-level asset directories have moved. See [MIGRATION.md](MIGRATION.md)
for the path mapping and an optional helper that restores the old entry points
locally for existing CRAMERA installations. No legacy aliases are tracked in Git.

In a CRAMERA checkout, these assets may live under `cramera/src/cramera/resources`.
The CRAM/CRAMERA application and the PR2 robot description are separate projects;
changes here do not update their source or packaging configuration automatically.

Run `python3 scripts/validate_resources.py` to check model and scene references.
With a CRAM environment, `python scripts/check_cram_load.py` loads every wind turbine
URDF with `URDFParser.from_file(...)`.
The GitHub Actions workflow validates both asset groups before publishing the
catalogue. [site/README.md](site/README.md) describes local previews, the catalogue
build and how to add entries.
