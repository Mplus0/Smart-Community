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
launch = ET.parse(root / 'launch/perception.launch').getroot()
defaults = {node.attrib['name']: node.attrib.get('default') for node in launch.findall('arg')}
assert defaults['enable_person'] == defaults['enable_plate'] == 'true'
assert defaults['enable_traffic_light'] == 'false'
includes = [node for node in launch.findall('include')
            if 'traffic_light_classification.launch' in node.attrib.get('file', '')]
assert len(includes) == 1 and includes[0].attrib['if'] == '$(arg enable_traffic_light)'
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
traffic_model = root / 'models/traffic_light_best.pt'
assert traffic_model.is_file() and traffic_model.stat().st_size > 0, traffic_model
print('TRAFFIC MODEL SHA256', hashlib.sha256(traffic_model.read_bytes()).hexdigest())
source_model = root.parents[2] / 'models/traffic_light/best.pt'
if source_model.is_file():
    assert traffic_model.read_bytes() == source_model.read_bytes(), 'Traffic model differs from source'
print('PASS: syntax, executable flags, XML, default dual launch, five original hashes, traffic model presence')
