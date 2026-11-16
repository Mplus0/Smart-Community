from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo
import base64
import hashlib
import json
import shutil
import subprocess

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont


workspace = Path('/home/mplus0/workspace/smart-community')
output = workspace / 'backups/navigation' / (datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S') + '_map_cleanup')
before = output / 'before'
after = output / 'after'
before.mkdir(parents=True)
after.mkdir()
remote = '''from pathlib import Path
import base64,json
root=Path('/home/mowen/workspace/Smart-Community/robot_ws/src/community_bringup/maps')
print(json.dumps({n:base64.b64encode((root/n).read_bytes()).decode() for n in ('map.yaml','map.pgm','map_nav.yaml','map_nav.pgm')}))
'''
snapshot = json.loads(subprocess.run(['ssh', 'mowen', 'python3', '-'], input=remote, capture_output=True, text=True, check=True).stdout)
for name, data in snapshot.items():
    (before / name).write_bytes(base64.b64decode(data))
    if name.endswith('.yaml'):
        shutil.copy2(before / name, after / name)

scan = np.array(Image.open(before / 'map.pgm'))
nav = np.array(Image.open(before / 'map_nav.pgm'))
count, labels, stats, _ = cv2.connectedComponentsWithStats((scan == 0).astype('uint8'), 8)
wall_id = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
yy, xx = np.where(labels == wall_id)
# Select only the long central stretches of the four measured outer walls.
# Corners and short internal returns are excluded from the line fit.
regions = {
    'top': (xx > 35) & (xx < 210) & (yy < 45),
    'bottom': (xx > 35) & (xx < 210) & (yy > 205),
    'left': (xx < 35) & (yy > 40) & (yy < 205),
    'right': (xx > 210) & (yy > 40) & (yy < 205),
}
lines = {}
for name, selected in regions.items():
    pts = np.column_stack((xx[selected], yy[selected])).astype('float32')
    vx, vy, x, y = cv2.fitLine(pts, cv2.DIST_HUBER, 0, .001, .001).flatten()
    normal = np.array([-vy, vx], dtype=float)
    lines[name] = (normal, float(normal @ np.array([x, y])))


def intersection(a, b):
    n, c = lines[a]
    m, d = lines[b]
    return np.linalg.solve(np.stack([n, m]), [c, d])


corners = np.array([intersection('top', 'left'), intersection('top', 'right'), intersection('bottom', 'right'), intersection('bottom', 'left')])
gy, gx = np.indices(scan.shape)
center = corners.mean(axis=0)
distances = []
for normal, offset in lines.values():
    sign = 1 if normal @ center - offset > 0 else -1
    distances.append(sign * (normal[0] * gx + normal[1] * gy - offset))
distance = np.min(distances, axis=0)
# Fit straight, continuous wall bands, 2 pixels (4 cm) thick. Leave the
# scanned frame and measured four-wall geometry unchanged.
wall = np.abs(distance) <= 1.0
inside = distance > 1.0
near_wall = distance <= 3.0
clean = scan.copy()
clean[distance < -1.0] = 205
clean[wall] = 0
clean[inside & near_wall] = 254

removed_noise = []
for i in range(1, count):
    area = int(stats[i, cv2.CC_STAT_AREA])
    if i != wall_id and area <= 3:
        mask = (labels == i) & inside
        clean[mask] = 254
        if mask.any():
            removed_noise.append({'pixels': int(mask.sum()), 'box_xywh': stats[i, :4].tolist()})

# Interior unknown islands are cleared only when tiny and completely inside
# the fitted wall. Larger unexplored areas remain unknown.
num, unknown_labels, unknown_stats, _ = cv2.connectedComponentsWithStats(((clean == 205) & inside).astype('uint8'), 8)
unknown_removed = 0
for i in range(1, num):
    if unknown_stats[i, cv2.CC_STAT_AREA] <= 3:
        unknown_removed += int(unknown_stats[i, cv2.CC_STAT_AREA])
        clean[unknown_labels == i] = 254

# Preserve each substantial navigation exclusion zone. Fill small holes
# enclosed in occupied regions; these are scan remnants inside the solid
# forbidden polygons, not the connected navigable corridors.
occupied = (nav == 0).astype('uint8')
n, components, comp_stats, _ = cv2.connectedComponentsWithStats(occupied, 8)
for i in range(1, n):
    if comp_stats[i, cv2.CC_STAT_AREA] <= 3:
        occupied[components == i] = 0
n, holes, hole_stats, _ = cv2.connectedComponentsWithStats(1 - occupied, 8)
filled_holes = 0
for i in range(1, n):
    x, y, w, h, area = hole_stats[i]
    touches_edge = x == 0 or y == 0 or x + w == nav.shape[1] or y + h == nav.shape[0]
    if not touches_edge and area <= 60:
        occupied[holes == i] = 1
        filled_holes += int(area)
contours, hierarchy = cv2.findContours(occupied, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
straight = [cv2.approxPolyDP(c, .8, True) if cv2.contourArea(c) > 100 else c for c in contours]
nav_mask = np.zeros_like(occupied)
cv2.drawContours(nav_mask, straight, -1, 1, cv2.FILLED, hierarchy=hierarchy)
clean_nav = clean.copy()
clean_nav[(nav_mask != 0) & inside] = 0
clean_nav[wall] = 0

for name, array in [('map', clean), ('map_nav', clean_nav)]:
    Image.fromarray(array).save(after / (name + '.pgm'))
    assert array.shape == scan.shape
    assert set(np.unique(array)) <= {0, 205, 254}
assert np.all(clean_nav[clean == 0] == 0), 'Navigation map must retain physical obstacles.'

# A 1-cell contour simplification must not remove the core of any original
# exclusion zone. Physical outer-wall replacement is checked separately.
distance_to_original_free = cv2.distanceTransform((nav == 0).astype('uint8'), cv2.DIST_L2, 5)
cleared_nav = (nav == 0) & (clean_nav != 0) & (distance > 3)
max_clearance = float(distance_to_original_free[cleared_nav].max()) if cleared_nav.any() else 0.0
assert max_clearance <= 1.5, max_clearance

report = {
    'source_host': 'mowen',
    'source_directory': '/home/mowen/workspace/Smart-Community/robot_ws/src/community_bringup/maps',
    'dimensions_pixels': list(scan.shape[::-1]),
    'resolution_m': .02,
    'origin': [-.606413, -.903932, 0.0],
    'fitted_wall_center_corners_image_xy': corners.tolist(),
    'fitted_wall_center_edge_lengths_m': (np.linalg.norm(corners - np.roll(corners, -1, axis=0), axis=1) * .02).tolist(),
    'wall_band_width_m': .04,
    'removed_isolated_occupied_components': removed_noise,
    'removed_tiny_unknown_pixels': unknown_removed,
    'navigation_filled_enclosed_hole_pixels': filled_holes,
    'max_removed_navigation_boundary_depth_pixels': max_clearance,
    'files': {},
}
for name in ('map.pgm', 'map_nav.pgm', 'map.yaml', 'map_nav.yaml'):
    report['files'][name] = {'before_sha256': hashlib.sha256((before / name).read_bytes()).hexdigest(), 'after_sha256': hashlib.sha256((after / name).read_bytes()).hexdigest()}
    if name.endswith('.pgm'):
        a = np.asarray(Image.open(before / name)); b = np.asarray(Image.open(after / name))
        report['files'][name]['changed_cells'] = int(np.count_nonzero(a != b))
(output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
shutil.copy2(__file__, output / 'process.py')

scale = 3
pad, title_height = 24, 40
tile_w, tile_h = scan.shape[1] * scale, scan.shape[0] * scale
canvas = Image.new('RGB', (pad * 3 + tile_w * 2, pad * 3 + (tile_h + title_height) * 2), '#f3f4f6')
draw = ImageDraw.Draw(canvas)
font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 22)
for row, name in enumerate(('map', 'map_nav')):
    for col, folder in enumerate((before, after)):
        x = pad + col * (tile_w + pad); y = pad + row * (tile_h + title_height + pad)
        draw.text((x, y), name + (' | Before' if col == 0 else ' | Cleaned'), fill='#111827', font=font)
        tile = Image.open(folder / (name + '.pgm')).convert('RGB').resize((tile_w, tile_h), Image.Resampling.NEAREST)
        canvas.paste(tile, (x, y + title_height))
canvas.save(output / 'comparison.png')
print('OUTPUT=' + str(output))
print(json.dumps({k: report[k] for k in ('fitted_wall_center_edge_lengths_m', 'removed_isolated_occupied_components', 'navigation_filled_enclosed_hole_pixels', 'max_removed_navigation_boundary_depth_pixels', 'files')}, indent=2))
