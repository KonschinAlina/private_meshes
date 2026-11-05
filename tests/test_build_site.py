import copy
import importlib.util
import json
import re
import tempfile
import unittest
from pathlib import Path
from zipfile import ZipFile

SPEC = importlib.util.spec_from_file_location('build_site', Path(__file__).resolve().parents[1] / 'scripts/build_site.py')
builder = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(builder)


class CatalogueBuildTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.source = self.root / 'objects' / 'articulated'
        self.folder = self.source / 'storage/001-test'
        for name in ('model.blend', 'README.md', 'NOTICE.md', 'assets/body.glb',
                     'renders/rest.webp', 'renders/open.webp'):
            file = self.folder / name
            file.parent.mkdir(parents=True, exist_ok=True)
            file.write_bytes(b'fixture')
        (self.folder / 'model.urdf').write_text(
            '<robot name="test"><link name="body"><visual><geometry>'
            '<mesh filename="assets/body.glb"/></geometry></visual></link>'
            '<link name="door"/><joint name="hinge" type="revolute">'
            '<parent link="body"/><child link="door"/><limit lower="0" upper="1.5"/>'
            '</joint></robot>')
        self.data = {'categories': [{'key': 'storage', 'title': 'Storage', 'count': 99}],
                     'objects': [{'id': '001', 'folder': 'storage/001-test', 'category': 'storage',
                                  'joints': [{'name': 'hinge', 'type': 'revolute', 'lower': 0, 'upper': 1.5}],
                                  'frames': [{}, {}]}]}
        self.save()
        (self.root / 'site').mkdir()
        (self.root / 'site/index.html').write_text(
            '<link rel="stylesheet" href="styles.css"><script src="app.js" defer></script>')
        (self.root / 'site/styles.css').write_text('body { color: black; }')
        (self.root / 'site/app.js').write_text("fetch('catalogue.json?v=__CATALOGUE_VERSION__')")
        (self.root / 'NOTICE.md').write_text('Repository licence scope')
        (self.root / 'LICENSE.txt').write_text('Repository licence')

    def save(self):
        (self.source / 'collection.json').write_text(json.dumps(self.data))

    def add_laboratory(self, moving=False):
        source = self.root / 'objects/laboratory'
        self.lab_folder = source / 'centrifuge'
        for name in ('model.urdf', 'assets/body.glb', 'README.md', 'NOTICE.md', 'LICENSE.txt',
                     'rights.json', 'metadata.json', 'manuals/v1.md', 'renders/rest.webp'):
            path = self.lab_folder / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(b'lab fixture')
        joint_type = 'continuous' if moving else 'fixed'
        (self.lab_folder / 'model.urdf').write_text(
            '<robot name="centrifuge"><link name="body"><visual><geometry>'
            '<mesh filename="assets/body.glb"/></geometry></visual></link><link name="handle"/>'
            f'<joint name="joint" type="{joint_type}"><parent link="body"/>'
            '<child link="handle"/></joint></robot>')
        joints = [{'name': 'joint', 'type': 'continuous', 'lower': None, 'upper': None}] if moving else []
        if moving:
            (self.lab_folder / 'renders/open.webp').write_bytes(b'open preview')
        self.lab_data = {
            'categories': [{'key': 'laboratory', 'code': 'LAB', 'title': 'Laboratory'}],
            'objects': [{'id': '158', 'folder': 'centrifuge', 'category': 'laboratory',
                         'mobility': 'articulated' if moving else 'rigid', 'joints': joints, 'frames': []}],
        }
        self.save_laboratory()

    def save_laboratory(self):
        (self.root / 'objects/laboratory/collection.json').write_text(json.dumps(self.lab_data))

    def test_build_packages_complete_relative_meshes_and_notices(self):
        stats = builder.build(self.root)
        self.assertEqual(stats, {'objects': 1, 'categories': 1, 'joints': 1, 'frames': 2})
        data = json.loads((self.root / 'dist/catalogue.json').read_text())
        self.assertEqual(data['categories'][0]['count'], 1)
        self.assertEqual(data['objects'][0]['url'], 'models/storage/001-test')
        self.assertEqual(data['objects'][0]['download'], 'downloads/001-test.zip')
        self.assertEqual(data['objects'][0]['formats'], ['URDF', 'GLB', 'Blender'])
        with ZipFile(self.root / 'dist' / data['objects'][0]['download']) as archive:
            self.assertIsNone(archive.testzip())
            for name in ('model.blend', 'model.urdf', 'assets/body.glb', 'NOTICE.md', '_repository/NOTICE.md'):
                self.assertIn('001-test/' + name, archive.namelist())
        self.assertTrue((self.root / 'dist/models/storage/001-test/renders/rest.webp').is_file())

    def test_missing_asset_fails_before_replacing_good_build(self):
        builder.build(self.root)
        previous = (self.root / 'dist/catalogue.json').read_bytes()
        (self.folder / 'assets/body.glb').unlink()
        with self.assertRaisesRegex(ValueError, 'Missing mesh'):
            builder.build(self.root)
        self.assertEqual((self.root / 'dist/catalogue.json').read_bytes(), previous)

    def test_unlisted_new_model_fails_with_clear_error(self):
        extra = self.source / 'storage/002-unlisted/model.urdf'
        extra.parent.mkdir()
        extra.write_text('<robot/>')
        with self.assertRaisesRegex(ValueError, 'Catalogue/folder mismatch'):
            builder.read_catalogue(self.root)

    def test_unsafe_paths_rejected(self):
        for value in ('', '.', '../outside', '/etc/passwd', 'storage/../../outside', 'storage\\file', 'C:/outside'):
            with self.subTest(value=value), self.assertRaises(ValueError):
                builder.local_path(self.source, value)

    def test_symlinks_rejected(self):
        try:
            (self.folder / 'linked').symlink_to(self.root / 'NOTICE.md')
        except OSError as error:
            if getattr(error, 'winerror', None) == 1314:
                self.skipTest('Windows account cannot create symlinks')
            raise
        with self.assertRaises(ValueError):
            builder.read_catalogue(self.root)

    def test_duplicate_ids_rejected(self):
        self.data['objects'].append(copy.deepcopy(self.data['objects'][0]))
        self.save()
        with self.assertRaisesRegex(ValueError, 'Duplicate object'):
            builder.read_catalogue(self.root)

    def test_missing_preview_and_unknown_category_rejected(self):
        (self.folder / 'renders/open.webp').unlink()
        with self.assertRaisesRegex(ValueError, 'Missing .*renders/open.webp'):
            builder.read_catalogue(self.root)
        self.data['objects'][0]['category'] = 'unknown'
        self.save()
        with self.assertRaisesRegex(ValueError, 'Unknown category'):
            builder.read_catalogue(self.root)

    def test_rebuild_removes_stale_downloads(self):
        builder.build(self.root)
        stale = self.root / 'dist/downloads/deleted-object.zip'
        stale.touch()
        builder.build(self.root)
        self.assertFalse(stale.exists())

    def test_merged_laboratory_rigid_asset_needs_no_blender_or_open_preview(self):
        self.add_laboratory()
        stats = builder.build(self.root)
        self.assertEqual(stats['objects'], 2)
        self.assertEqual(stats['categories'], 2)
        data = json.loads((self.root / 'dist/catalogue.json').read_text())
        lab = data['objects'][1]
        self.assertEqual(lab['url'], 'models/laboratory/centrifuge')
        self.assertEqual(lab['download'], 'downloads/158-centrifuge.zip')
        self.assertEqual(lab['formats'], ['URDF', 'GLB'])
        self.assertEqual(lab['mobility'], 'rigid')
        self.assertEqual(lab['previews'], {'rest': 'renders/rest.webp'})
        self.assertEqual(data['categories'][1]['count'], 1)
        with ZipFile(self.root / 'dist' / lab['download']) as archive:
            self.assertIsNone(archive.testzip())
            for name in ('model.urdf', 'assets/body.glb', 'README.md', 'NOTICE.md', 'LICENSE.txt',
                         'rights.json', 'metadata.json', 'manuals/v1.md', '_repository/LICENSE.txt'):
                self.assertIn('centrifuge/' + name, archive.namelist())
        self.assertTrue((self.root / 'dist/models/laboratory/centrifuge/renders/rest.webp').is_file())
        self.assertFalse((self.root / 'dist/models/laboratory/centrifuge/renders/open.webp').exists())

    def test_existing_rigid_open_preview_url_is_preserved(self):
        path = self.folder / 'model.urdf'
        path.write_text(path.read_text().replace('type="revolute"', 'type="fixed"'))
        self.data['objects'][0]['joints'] = []
        self.save()
        builder.build(self.root)
        obj = json.loads((self.root / 'dist/catalogue.json').read_text())['objects'][0]
        self.assertEqual(obj['mobility'], 'rigid')
        self.assertEqual(obj['previews'], {'rest': 'renders/rest.webp'})
        self.assertTrue((self.root / 'dist/models/storage/001-test/renders/open.webp').is_file())

    def test_laboratory_continuous_joint_keeps_null_limits_and_requires_open_preview(self):
        self.add_laboratory(moving=True)
        obj = builder.read_catalogue(self.root)['objects'][1]
        self.assertEqual(obj['mobility'], 'articulated')
        self.assertIsNone(obj['joints'][0]['upper'])
        self.assertIn('open', obj['previews'])
        (self.lab_folder / 'renders/open.webp').unlink()
        with self.assertRaisesRegex(ValueError, 'Missing .*renders/open.webp'):
            builder.read_catalogue(self.root)

    def test_laboratory_mobility_and_joint_metadata_must_match_urdf(self):
        self.add_laboratory(moving=True)
        original = copy.deepcopy(self.lab_data)
        for change, message in (
            ({'mobility': 'rigid'}, 'mobility mismatch'),
            ({'joints': []}, 'joint mismatch'),
            ({'joints': [{'name': 'joint', 'type': 'revolute'}]}, 'joint type mismatch'),
            ({'joints': [{'name': 'joint', 'type': 'continuous', 'lower': 0, 'upper': 0}]}, 'joint limit mismatch'),
            ({'frames': [{'name': 'missing_link', 'purpose': 'grasp'}]}, 'frame mismatch'),
        ):
            with self.subTest(change=change):
                self.lab_data = copy.deepcopy(original)
                self.lab_data['objects'][0].update(change)
                self.save_laboratory()
                with self.assertRaisesRegex(ValueError, message):
                    builder.read_catalogue(self.root)

    def test_duplicate_ids_and_categories_across_collections_rejected(self):
        self.add_laboratory()
        self.lab_data['objects'][0]['id'] = '001'
        self.save_laboratory()
        with self.assertRaisesRegex(ValueError, 'Duplicate object'):
            builder.read_catalogue(self.root)
        self.lab_data['objects'][0]['id'] = '158'
        self.lab_data['categories'][0]['key'] = 'storage'
        self.save_laboratory()
        with self.assertRaisesRegex(ValueError, 'Duplicate category'):
            builder.read_catalogue(self.root)

    def test_laboratory_inventory_and_required_rights_validated_before_replacing_build(self):
        self.add_laboratory()
        builder.build(self.root)
        previous = (self.root / 'dist/catalogue.json').read_bytes()
        (self.lab_folder / 'rights.json').unlink()
        with self.assertRaisesRegex(ValueError, 'Missing .*rights.json'):
            builder.build(self.root)
        self.assertEqual((self.root / 'dist/catalogue.json').read_bytes(), previous)
        (self.lab_folder / 'rights.json').write_text('{}')
        extra = self.lab_folder.parent / 'unlisted/model.urdf'
        extra.parent.mkdir()
        extra.write_text('<robot/>')
        with self.assertRaisesRegex(ValueError, 'Catalogue/folder mismatch in laboratory'):
            builder.read_catalogue(self.root)

    def test_present_laboratory_requires_manifest(self):
        (self.root / 'objects/laboratory').mkdir()
        with self.assertRaisesRegex(ValueError, 'Missing laboratory/collection.json'):
            builder.read_catalogue(self.root)

    def test_wind_turbine_collection_follows_exported_collection_rules(self):
        source = self.root / 'objects/wind-turbine'
        folder = source / '170-blade'
        for name in ('assets/blade.glb', 'assets/fault.glb', 'README.md', 'NOTICE.md', 'LICENSE.txt',
                     'rights.json', 'metadata.json', 'renders/rest.webp'):
            path = folder / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(b'turbine fixture')
        (folder / 'model.urdf').write_text(
            '<robot name="blade"><link name="blade"><visual><geometry>'
            '<mesh filename="assets/blade.glb"/></geometry></visual></link></robot>')
        # Fault states are extra URDFs beside model.urdf, not separate catalogue folders.
        (folder / 'model_crack.urdf').write_text(
            '<robot name="blade_crack"><link name="blade"><visual><geometry>'
            '<mesh filename="assets/fault.glb"/></geometry></visual></link></robot>')
        data = {'categories': [{'key': 'wind-turbine', 'code': 'WTB', 'title': 'Wind Turbine'}],
                'objects': [{'id': '170', 'folder': '170-blade', 'category': 'wind-turbine', 'joints': [],
                             'frames': [{'name': 'blade', 'purpose': 'base_origin'}]}]}
        (source / 'collection.json').write_text(json.dumps(data))
        builder.build(self.root)
        obj = json.loads((self.root / 'dist/catalogue.json').read_text())['objects'][1]
        self.assertEqual(obj['url'], 'models/wind-turbine/170-blade')
        self.assertEqual(obj['download'], 'downloads/170-blade.zip')
        with ZipFile(self.root / 'dist' / obj['download']) as archive:
            self.assertIn('170-blade/model_crack.urdf', archive.namelist())
        data['objects'][0]['category'] = 'storage'
        (source / 'collection.json').write_text(json.dumps(data))
        with self.assertRaisesRegex(ValueError, 'Unknown category'):
            builder.read_catalogue(self.root)
        data['categories'][0]['key'] = data['objects'][0]['category'] = 'turbine'
        (source / 'collection.json').write_text(json.dumps(data))
        with self.assertRaisesRegex(ValueError, 'Category/folder mismatch'):
            builder.read_catalogue(self.root)

    def test_build_versions_are_stable_and_catalogue_changes_refresh_all_requests(self):
        builder.build(self.root)
        names = ('index.html', 'app.js', 'styles.css', 'catalogue.json')
        first = {name: (self.root / 'dist' / name).read_bytes() for name in names}
        builder.build(self.root)
        self.assertEqual(first, {name: (self.root / 'dist' / name).read_bytes() for name in names})
        version = re.search(r'app\.js\?v=([a-f0-9]+)', first['index.html'].decode()).group(1)
        self.assertIn(f'styles.css?v={version}', first['index.html'].decode())
        self.assertIn(f'catalogue.json?v={version}', first['app.js'].decode())
        self.assertNotIn(b'__CATALOGUE_VERSION__', first['app.js'])
        self.data['categories'][0]['title'] = 'Updated storage category'
        self.save()
        builder.build(self.root)
        html = (self.root / 'dist/index.html').read_text()
        changed = re.search(r'app\.js\?v=([a-f0-9]+)', html).group(1)
        self.assertNotEqual(version, changed)
        self.assertIn(f'styles.css?v={changed}', html)
        self.assertIn(f'catalogue.json?v={changed}', (self.root / 'dist/app.js').read_text())
        self.assertIn('__CATALOGUE_VERSION__', (self.root / 'site/app.js').read_text())

    def test_application_and_style_changes_refresh_build_version(self):
        builder.build(self.root)
        for name in ('app.js', 'styles.css'):
            with self.subTest(asset=name):
                before = (self.root / 'dist/index.html').read_bytes()
                source = self.root / 'site' / name
                source.write_text(source.read_text() + '\n/* updated */')
                builder.build(self.root)
                self.assertNotEqual(before, (self.root / 'dist/index.html').read_bytes())


if __name__ == '__main__':
    unittest.main()
