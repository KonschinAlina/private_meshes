# Objects

Independently loadable models live here. Copy the complete model folder,
including its meshes, documentation and notices.

| Collection | Contents | Entry point |
| --- | --- | --- |
| [Articulated objects](articulated/README.md) | 164 furniture, appliance, workshop and storage models, grouped into 12 categories | `<category>/<id>-<slug>/model.urdf` or `model.blend` |
| [Laboratory devices](laboratory/README.md) | Centrifuge, dual dispenser, refrigerator, scale and tube rack | `<device>/model.urdf` |
| [Wind turbine parts](wind-turbine/README.md) | 54 parts of an onshore wind turbine, each with its healthy and faulty states and its place in the assembled turbine | `<id>-<slug>/model.urdf`, `model_<state>.urdf` |

The [web catalogue](https://sunava.github.io/private_meshes/) covers all three
collections, with Laboratory and Wind Turbine categories and an Articulated/Rigid filter.
The centrifuge and refrigerator have moving joints; the dual dispenser, scale
and tube rack are rigid in their current URDFs. Each device has a rendered
preview and a complete ZIP download. Articulated devices also have an inspection
preview. Laboratory exports include URDF and GLB files, but no Blender project.

The wind turbine parts are generated from the turbine model by
`scripts/export_turbine_parts.py`; do not edit their folders by hand. Each part's
`metadata.json` records where it sits in the assembled turbine, and
[`wind-turbine/INVENTORY.md`](wind-turbine/INVENTORY.md) lists every source mesh.

The centrifuge's simulation manuals are stored with that device in
[`laboratory/centrifuge/manuals/`](laboratory/centrifuge/manuals/).

Whole rooms and prearranged scenes belong in [environments/](../environments/README.md).
Keep asset-specific licence notices with each model; see the repository's
[licence scope](../NOTICE.md).
