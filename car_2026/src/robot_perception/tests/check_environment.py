#!/usr/bin/env python3
"""Run with the selected interpreter after sourcing ROS, before starting nodes."""
import importlib
import importlib.metadata
import contextlib
import json
from pathlib import Path
import sys

result = {'executable': sys.executable, 'python': sys.version, 'home': str(Path.home()),
          'modules': {}}
for name in ('rospy', 'sensor_msgs.msg', 'std_msgs.msg', 'ultralytics', 'torch',
             'torchvision', 'numpy', 'cv2', 'PIL', 'onnxruntime', 'yaml'):
    try:
        with contextlib.redirect_stdout(sys.stderr):
            module = importlib.import_module(name)
        result['modules'][name] = {'ok': True, 'version': getattr(module, '__version__', None),
                                   'path': getattr(module, '__file__', None)}
    except Exception as error:
        result['modules'][name] = {'ok': False, 'error': str(error)}
# Never import HyperLPR before cache validation: the upstream import can download models.
try:
    result['hyperlpr3_metadata'] = importlib.metadata.version('hyperlpr3')
except importlib.metadata.PackageNotFoundError:
    result['hyperlpr3_metadata'] = None
if result['modules']['torch']['ok']:
    import torch
    result['torch_cuda_available'] = torch.cuda.is_available()
    result['torch_cuda_version'] = torch.version.cuda
    result['torch_gpu_count'] = torch.cuda.device_count()
print(json.dumps(result, ensure_ascii=False, indent=2))
sys.exit(0 if all(item['ok'] for item in result['modules'].values())
         and result['hyperlpr3_metadata'] == '0.1.3' else 1)
