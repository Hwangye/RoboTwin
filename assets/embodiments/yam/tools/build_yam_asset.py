#!/usr/bin/env python3
"""Rebuild yam.urdf and collision_yam.yml from the vendor export (yam_raw.urdf).

The upstream i2rt URDF is an Onshape export: visual meshes only, no collision
geometry, and placeholder actuator limits. This script is the single source of
truth for everything RoboTwin adds on top of it -- run it and the asset is
reproduced exactly. See ../README.md for why each step is there.

    python assets/embodiments/yam/tools/build_yam_asset.py
"""
import os
import numpy as np
import trimesh
import transforms3d as t3d
import xml.etree.ElementTree as ET

D = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# ---------------------------------------------------------------- collision --
# link -> number of slabs to cut it into along its longest axis; one minimal
# oriented box per slab. Camera links are skipped entirely: the D405 bracket
# reaches 80 mm sideways past the gripper axis and knocks over anything being
# approached from the side.
SLABS = {
    'base': 2, 'link1': 1, 'link2': 4, 'link3': 5, 'link4': 3, 'link5': 2,
    'gripper': 2, 'tip_left': 4, 'tip_right': 4,
}
# ------------------------------------------------------------------ spheres --
# link -> (voxel pitch, max sphere radius) for the curobo collision model.
# The fingers need a small cap: at r=0.017 a 6 mm pad inflates to 34 mm and the
# planner reads the lower fingertip as buried in the table.
SPHERES = {
    'base':      (0.010, 0.042), 'link1':     (0.010, 0.040),
    'link2':     (0.010, 0.034), 'link3':     (0.010, 0.034),
    'link4':     (0.009, 0.030), 'link5':     (0.008, 0.030),
    'gripper':   (0.008, 0.032), 'tip_left':  (0.005, 0.011),
    'tip_right': (0.005, 0.011),
}
# ------------------------------------------------------------------- limits --
# Upstream ships effort="1" velocity="1" on every joint. Real motor ranges come
# from the vendor MJCF actuator classes: dm4340 +-28 Nm, dm4310 +-10 Nm.
# ------------------------------------------------------------------ colors --
# The Onshape export assigns arbitrary per-part colors (purple base, teal tube,
# pink wrist). The real YAM is matte black with white arm tubes; values follow
# the vendor-tuned MJCF (i2rt via the ABC project): black 0.06, white 0.9.
# link2/link3 are single merged meshes, so the whole tube link goes white.
BLACK, WHITE = "0.06 0.06 0.06 1", "0.9 0.9 0.9 1"
MATERIALS = {
    'base': BLACK, 'link1': BLACK, 'link2': WHITE, 'link3': WHITE,
    'link4': BLACK, 'link5': BLACK, 'gripper': BLACK,
    'tip_left': BLACK, 'tip_right': BLACK,
    'camera_bracket': BLACK, 'camera_cable_holder': BLACK,
    'camera_body': BLACK, 'camera_cover': BLACK,
}

LIMITS = {
    'joint1': (28, 3.14), 'joint2': (28, 3.14), 'joint3': (28, 3.14),
    'joint4': (10, 3.14), 'joint5': (10, 3.14), 'joint6': (10, 3.14),
    'joint7': (100, 0.5), 'joint8': (100, 0.5),
}


def link_mesh(link):
    """Visual mesh of a link, transformed into the link frame."""
    v = link.find('visual')
    if v is None:
        return None
    m = trimesh.load(f"{D}/{v.find('geometry/mesh').get('filename')}", force='mesh')
    o = v.find('origin')
    R = t3d.euler.euler2mat(*np.fromstring(o.get('rpy'), sep=' '), 'sxyz')
    m.apply_transform(t3d.affines.compose(np.fromstring(o.get('xyz'), sep=' '), R, [1, 1, 1]))
    return m


def slab_boxes(points, k):
    ax = int(np.argmax(points.max(0) - points.min(0)))
    edges = np.linspace(points[:, ax].min(), points[:, ax].max(), k + 1)
    out = []
    for lo, hi in zip(edges[:-1], edges[1:]):
        sel = points[(points[:, ax] >= lo - 1e-9) & (points[:, ax] <= hi + 1e-9)]
        if len(sel) < 4:
            continue
        T, ext = trimesh.bounds.oriented_bounds(sel)
        T = np.linalg.inv(T)
        out.append((ext, T[:3, 3], np.array(t3d.euler.mat2euler(T[:3, :3], 'sxyz'))))
    return out


def kd_spheres(points, r_max, floor):
    """Split until every cell fits a sphere of at most r_max; circumscribe each."""
    c = 0.5 * (points.min(0) + points.max(0))
    r = float(np.linalg.norm(points - c, axis=1).max())
    if r <= r_max or len(points) < 8:
        return [(c, max(r, floor))]
    ax = int(np.argmax(points.max(0) - points.min(0)))
    mid = np.median(points[:, ax])
    lo, hi = points[points[:, ax] <= mid], points[points[:, ax] > mid]
    if len(lo) == 0 or len(hi) == 0:
        return [(c, max(r, floor))]
    return kd_spheres(lo, r_max, floor) + kd_spheres(hi, r_max, floor)


tree = ET.parse(f'{D}/yam_raw.urdf')
root = tree.getroot()

n_boxes = 0
for link in root.findall('link'):
    pts_mesh = link_mesh(link)
    if pts_mesh is None or link.get('name') not in SLABS:
        continue
    for ext, xyz, rpy in slab_boxes(pts_mesh.vertices, SLABS[link.get('name')]):
        col = ET.SubElement(link, 'collision')
        o = ET.SubElement(col, 'origin')
        o.set('xyz', ' '.join(f'{x:.6g}' for x in xyz))
        o.set('rpy', ' '.join(f'{x:.6g}' for x in rpy))
        g = ET.SubElement(col, 'geometry')
        ET.SubElement(g, 'box').set('size', ' '.join(f'{x:.6g}' for x in ext))
        n_boxes += 1

for link in root.findall('link'):
    rgba = MATERIALS.get(link.get('name'))
    if rgba:
        for v in link.findall('visual'):
            m = v.find('material')
            if m is not None:
                c = m.find('color')
                if c is not None:
                    c.set('rgba', rgba)

for joint in root.findall('joint'):
    lim = joint.find('limit')
    if lim is not None and joint.get('name') in LIMITS:
        effort, vel = LIMITS[joint.get('name')]
        lim.set('effort', str(effort))
        lim.set('velocity', str(vel))

# The vendor `camera` link is ROS-optical (+z forward, +y down); RoboTwin hands
# this link's pose straight to a sapien camera, which looks down +x with +y left
# and +z up. Keep the vendor frame, add a rotated child under the expected name.
R_OPTICAL_TO_SAPIEN = np.array([[0., -1., 0.], [0., 0., -1.], [1., 0., 0.]])
for e in root.iter():
    if e.tag == 'link' and e.get('name') == 'camera':
        e.set('name', 'camera_optical')
    if e.tag in ('parent', 'child') and e.get('link') == 'camera':
        e.set('link', 'camera_optical')
ET.SubElement(root, 'link').set('name', 'camera')
j = ET.SubElement(root, 'joint')
j.set('name', 'camera_sapien_joint')
j.set('type', 'fixed')
o = ET.SubElement(j, 'origin')
o.set('xyz', '0 0 0')
o.set('rpy', ' '.join(f'{v:.6f}' for v in t3d.euler.mat2euler(R_OPTICAL_TO_SAPIEN, 'sxyz')))
ET.SubElement(j, 'parent').set('link', 'camera_optical')
ET.SubElement(j, 'child').set('link', 'camera')

ET.indent(tree, '    ')
tree.write(f'{D}/yam.urdf', encoding='utf-8', xml_declaration=False)
print(f'yam.urdf: {n_boxes} collision boxes, actuator limits set, camera frame added')

out, total = ['collision_spheres:'], 0
links = {l.get('name'): l for l in ET.parse(f'{D}/yam.urdf').getroot().findall('link')}
for name, (pitch, r_max) in SPHERES.items():
    pts = link_mesh(links[name]).voxelized(pitch).fill().points
    spheres = kd_spheres(pts, r_max, pitch * 0.87)
    out.append(f'    {name}:')
    for c, r in spheres:
        out.append(f'        - "center": [{c[0]:.4f}, {c[1]:.4f}, {c[2]:.4f}]')
        out.append(f'          "radius": {r:.4f}')
    total += len(spheres)
print(f'collision_yam.yml: {total} spheres')
with open(f'{D}/collision_yam.yml', 'w') as f:
    f.write('\n'.join(out) + '\n')
