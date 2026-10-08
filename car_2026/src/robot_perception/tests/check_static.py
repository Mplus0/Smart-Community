#!/usr/bin/env python3
"""Container-only static/model checks; does not import inference engines."""
import ast
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

root = Path(__file__).resolve().parents[1]
for path in (root / 'scripts').glob('*.py'):
    ast.parse(path.read_text(encoding='utf-8'), filename=str(path))
    assert path.stat().st_mode & 0o111, path
for path in [root / 'package.xml', *sorted((root / 'launch').glob('*.launch'))]:
    tree = ET.parse(path)
    assert not any('traffic_light' in str(node.attrib) for node in tree.iter()), path
manifest = json.loads((root / 'models/hyperlpr3/models_manifest.json').read_text())
assert manifest['hyperlpr3_version'] == '0.1.3'
assert manifest['model_version'] == '20230229'
assert len(manifest['models']) == 4
models = [(root / 'models/person_best.pt', 5375301,
           '4d3076a9ca0afd9a35f9d393d0726d9e4207a45155348fb768fccd3d10e28806')]
models.extend((root / 'models/hyperlpr3/20230229/onnx' / entry['name'],
               entry['bytes'], entry['sha256']) for entry in manifest['models'])
for path, size, digest in models:
    assert path.stat().st_size == size, path
    assert hashlib.sha256(path.read_bytes()).hexdigest() == digest, path
    print('MODEL OK', path.name, size, digest)
print('PASS: Python syntax, executable flags, package/launch XML, five model hashes')
