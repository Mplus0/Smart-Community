#!/usr/bin/env python3
"""Offline structural validator; does not import ROS or run nodes."""
import ast, hashlib, json, re
from pathlib import Path
import xml.etree.ElementTree as E
import yaml
R=Path(__file__).resolve().parents[1]
errors=[]; warnings=[]
V={'lslidar','lslidar_driver','lslidar_msgs','wit_ros_imu'}
P={E.parse(p).getroot().findtext('name').strip():p.parent for p in R.glob('src/*/package.xml')}
assert set(P)==V|{'community_bringup','community_base','community_perception','community_mission','robot_description'}
S={n:{p.stem for p in d.glob('srv/*.srv')} for n,d in P.items()}
M={p.parent.name for p in R.glob('src/*/src/*/__init__.py')}
OLD={'community_navigation','yolo_onnx_ros','license_plate_ocr','fall_counter','move_nav','demo01_server_client','video_recorder','mowen'}
for n,d in P.items():
 assert n==d.name
 deps={e.text for e in E.parse(d/'package.xml').getroot() if e.tag.endswith('depend')}
 cm=(d/'CMakeLists.txt').read_text(encoding='utf-8')
 if n not in V:
  for s in S[n]:
   assert s+'.srv' in cm,(n,s)
  for m in d.glob('src/*/__init__.py'):
   assert m.parent.name in (d/'setup.py').read_text(encoding='utf-8')
 for p in d.rglob('*'):
  if not p.is_file():continue
  if p.suffix=='.py':
   try:t=ast.parse(p.read_text(encoding='utf-8'))
   except SyntaxError as e:
    (warnings if n in V else errors).append(str(p.relative_to(R))+': '+e.msg);continue
   if n in V:continue
   for x in ast.walk(t):
    if isinstance(x,ast.ImportFrom) and x.module:
     parts=x.module.split('.')
     if parts[0] in OLD:errors.append('Old import: '+x.module)
     if x.level and not (p.parent/(x.module.replace('.','/')+'.py')).exists():errors.append('Missing relative import: '+x.module)
     if x.level==0 and parts[0].startswith('community_') and len(parts)==1 and parts[0] not in M:errors.append('Missing local module: '+x.module)
     if len(parts)==2 and parts[1]=='srv' and parts[0] in P:
      if parts[0]!=n and parts[0] not in deps:errors.append('Missing dependency: '+parts[0])
      for a in x.names:
       if re.sub(r'(Request|Response)$','',a.name) not in S[parts[0]]:errors.append('Missing service: '+a.name)
  if p.suffix in ('.xml','.launch','.urdf','.xacro'):
   try:t=E.parse(p)
   except E.ParseError as e:
    (warnings if p.name == 'mowen.urdf.xacro' else errors).append(str(p.relative_to(R))+': '+str(e));continue
   for el in t.iter():
    for val in el.attrib.values():
     m=re.fullmatch(r'\$\(find ([^)]+)\)/(.*)',val) or re.fullmatch(r'package://([^/]+)/(.*)',val)
     if m:
      if m[1] in OLD:errors.append('Old resource: '+val)
      if m[1] in P and '$(' not in m[2] and not (P[m[1]]/m[2]).exists():errors.append('Missing resource: '+val)
    if el.tag=='node' and el.get('pkg') in P and el.get('type','').endswith('.py') and '$(' not in el.get('type',''):
     if not list(P[el.get('pkg')].rglob(el.get('type'))):errors.append('Missing node: '+el.get('type'))
    if el.tag=='include':
     m=re.fullmatch(r'\$\(find ([^)]+)\)/(.*)',el.get('file',''))
     if m and m[1] in P and (P[m[1]]/m[2]).exists():
      args={a.get('name'):a for a in E.parse(P[m[1]]/m[2]).getroot().findall('arg')}
      got={a.get('name') for a in el.findall('arg')}
      req={k for k,v in args.items() if 'default' not in v.attrib and 'value' not in v.attrib}
      if got-set(args) or req-got:errors.append('Include argument mismatch: '+str(p))
  if p.suffix=='.yaml':
   try:yaml.safe_load(p.read_text(encoding='utf-8'))
   except yaml.YAMLError as e:errors.append(str(p)+str(e))
b=json.loads((R/'docs/structure_integrity.json').read_text(encoding='utf-8'))
for path,h in b['expected'].items():
 p=R/path
 if not p.exists() or hashlib.sha256(p.read_bytes()).hexdigest()!=h:errors.append('Integrity mismatch: '+path)
report='# 静态检查报告\n\n'+f'功能包：{len(P)}；完整性校验：{len(b["expected"])} 个文件。\n\n'
report+='检查 Python 语法、本地模块/服务导入、依赖、XML/YAML、资源引用、入口、include 参数及服务生成。\n\n'
report+='## 项目问题\n\n'+('\n'.join(errors) or '上述静态检查未发现问题。')
report+='\n\n## 厂商原有问题（未修改）\n\n'+'\n'.join(warnings)
report+='\n\n未安装 ROS、未编译、未启动节点；外部依赖、生成服务和运行行为待工控机验证。\n'
(R/'docs/VALIDATION.md').write_text(report,encoding='utf-8');print(report)
raise SystemExit(bool(errors))
