"""Generate the aloha camera-rig embodiments and their eval task configs.

Single source of truth for every SAPolicy camera rig. Regenerates, from the two
official anchor cameras, the 13 embodiment directories under assets/embodiments/
and the 10 eval_* task configs -- do not edit those by hand; edit this file and
rerun. `python script/gen_aloha_rigs.py --check` verifies the files on disk
match what the spec produces (useful in CI / before committing).

Geometry
  Each ring is derived from an official anchor camera by intersecting its view
  ray with the z = 0.90 table plane (giving the ring target), then revolving
  the anchor around the vertical axis through the target:

    observer ring  anchor = official observer_camera (envs/camera/camera.py):
                   pos (0, 0.23, 1.33), fwd (0, -1, -1.02).  Ring cameras are
                   type MV43_93 (322x238 fovy 93).  Anchor sits at az +90.
    head ring      anchor = stock aloha-agilex head_camera:
                   pos (-0.032, -0.45, 1.35), fwd (0, 0.6, -0.8).  Ring cameras
                   are type HEAD43_37 (322x238 fovy 37).  Anchor sits at az -90.

  24-view index contract (all rings): index 0..20 -> az -150..+150 step 15;
  21 = +165, 22 = -165, 23 = 180.  So third_view_16 = official observer and
  head_view_4 (az -90) = official head camera, exactly.

Rigs
  aloha-agilex-mv2      head_camera + observer ring (third_view_0..23)  collection
  aloha-agilex-mv3head  head + front + head-geometry ring (third_view_0..23)
  aloha-agilex-dual24   observer ring (third_view_*) + head ring (head_view_*)
  aloha-agilex-obs14..18  single observer-ring camera  (eval, az +60..+120)
  aloha-agilex-head2..6   single head-ring camera      (eval, az -120..-60)

Each embodiment directory contains one generated config.yml; everything else
(urdf, meshes, curobo files) is a relative symlink into ../aloha-agilex/.
"""

import argparse
import math
import os
import sys

import yaml

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EMBODIMENTS = os.path.join(REPO, "assets", "embodiments")
TASK_CONFIG = os.path.join(REPO, "task_config")
STOCK = "aloha-agilex"

TABLE_Z = 0.90

RINGS = {
    "obs": {"anchor_pos": (0.0, 0.23, 1.33), "anchor_fwd": (0.0, -1.0, -1.02),
            "cam_type": "MV43_93"},
    "head": {"anchor_pos": (-0.032, -0.45, 1.35), "anchor_fwd": (0.0, 0.6, -0.8),
             "cam_type": "HEAD43_37"},
}

# index -> azimuth (degrees); see the contract in the module docstring
AZ = {i: -150 + 15 * i for i in range(21)}
AZ.update({21: 165, 22: -165, 23: 180})

ALL = list(range(24))
# name -> (base cameras kept from the stock config, [(ring, name_prefix, indices)])
RIGS = {
    "mv2": (["head_camera"], [("obs", "third_view", ALL)]),
    "mv3head": (["head_camera", "front_camera"], [("head", "third_view", ALL)]),
    "dual24": ([], [("obs", "third_view", ALL), ("head", "head_view", ALL)]),
}
for i in (14, 15, 16, 17, 18):
    RIGS[f"obs{i}"] = ([], [("obs", "third_view", [i])])
for i in (2, 3, 4, 5, 6):
    RIGS[f"head{i}"] = ([], [("head", "head_view", [i])])

EVAL_RIGS = [f"obs{i}" for i in (14, 15, 16, 17, 18)] + [f"head{i}" for i in (2, 3, 4, 5, 6)]

EVAL_TEMPLATE = """\
# 闭环评测:与 offrt 训练数据同规格(rt 光追、322x238、锚点 + 双腕三相机)。
# rng_static_camera_compat: 2 让场景生成与官方一致 —— 评测场景必须和官方协议
# 同分布,否则和官方基线的数字没有可比性。
# 机位:{ring_name} 环 az {az:+d}(相对官方锚点 {rel:+d} 度)。由 script/gen_aloha_rigs.py 生成。
render_freq: 0
episode_num: 50
use_seed: false
save_freq: 15
embodiment: [aloha-agilex-{rig}]
language_num: 100
domain_randomization:
  random_background: false
  cluttered_table: false
  clean_background_rate: 1
  random_head_camera_dis: 0
  random_table_height: 0
  random_light: false
  crazy_random_light_rate: 0
camera:
  head_camera_type: D435
  wrist_camera_type: WRIST43_37
  collect_head_camera: false
  collect_wrist_camera: true
  rng_static_camera_compat: 2
data_type:
  rgb: true
  third_view: false
  depth: true
  pointcloud: false
  observer: false
  endpose: true
  qpos: true
  mesh_segmentation: false
  actor_segmentation: false
pcd_down_sample_num: 1024
pcd_crop: true
save_path: ./data_eval
clear_cache_freq: 5
collect_data: true
eval_video_log: false
"""


def _r6(x):
    x = round(float(x), 6)
    return 0.0 if x == 0 else x


def ring_camera(ring, prefix, idx):
    spec = RINGS[ring]
    px, py, pz = spec["anchor_pos"]
    fx, fy, fz = spec["anchor_fwd"]
    n = math.sqrt(fx * fx + fy * fy + fz * fz)
    fx, fy, fz = fx / n, fy / n, fz / n
    t = (TABLE_Z - pz) / fz                       # ray-plane intersection
    target = (px + t * fx, py + t * fy, TABLE_Z)
    rh = math.hypot(px - target[0], py - target[1])   # horizontal ring radius
    az = math.radians(AZ[idx])
    pos = (target[0] + rh * math.cos(az), target[1] + rh * math.sin(az), pz)
    d = tuple(target[k] - pos[k] for k in range(3))
    dn = math.sqrt(sum(v * v for v in d))
    fwd = tuple(v / dn for v in d)
    left = (math.sin(az), -math.cos(az), 0.0)
    return {
        "name": f"{prefix}_{idx}",
        "type": spec["cam_type"],
        "position": [_r6(v) for v in pos],
        "forward": [_r6(v) for v in fwd],
        "left": [_r6(v) for v in left],
    }


def build_embodiment_config(stock_cfg, rig):
    base_names, rings = RIGS[rig]
    stock_cams = {c["name"]: c for c in stock_cfg["static_camera_list"]}
    cams = [stock_cams[n] for n in base_names]
    for ring, prefix, indices in rings:
        cams += [ring_camera(ring, prefix, i) for i in indices]
    cfg = dict(stock_cfg)
    cfg["static_camera_list"] = cams
    return cfg


def render_embodiment(rig, cfg):
    header = f"# Generated by script/gen_aloha_rigs.py (rig: {rig}) -- do not edit by hand.\n"
    return header + yaml.safe_dump(cfg, sort_keys=False, allow_unicode=True,
                                   default_flow_style=None, width=100)


def render_eval_yml(rig):
    ring = "obs" if rig.startswith("obs") else "head"
    idx = int(rig[len(ring):])
    anchor = 90 if ring == "obs" else -90
    ring_name = "observer" if ring == "obs" else "head"
    return EVAL_TEMPLATE.format(rig=rig, ring_name=ring_name, az=AZ[idx],
                                rel=AZ[idx] - anchor)


def targets():
    stock_cfg = yaml.safe_load(open(os.path.join(EMBODIMENTS, STOCK, "config.yml")))
    out = {}
    for rig in RIGS:
        path = os.path.join(EMBODIMENTS, f"{STOCK}-{rig}", "config.yml")
        out[path] = render_embodiment(rig, build_embodiment_config(stock_cfg, rig))
    for rig in EVAL_RIGS:
        out[os.path.join(TASK_CONFIG, f"eval_{rig}.yml")] = render_eval_yml(rig)
    return out


def ensure_symlinks(rig_dir):
    made = []
    for entry in sorted(os.listdir(os.path.join(EMBODIMENTS, STOCK))):
        if entry == "config.yml":
            continue
        link = os.path.join(rig_dir, entry)
        want = os.path.join("..", STOCK, entry)
        if os.path.lexists(link):
            # a correct symlink, or a real copy of the stock file -- leave both alone
            if not os.path.islink(link) or os.readlink(link) == want:
                continue
            os.remove(link)
        os.symlink(want, link)
        made.append(entry)
    return made


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--check", action="store_true",
                    help="verify files on disk match the spec; exit 1 on drift")
    args = ap.parse_args()

    drift = []
    for path, content in targets().items():
        rel = os.path.relpath(path, REPO)
        on_disk = open(path).read() if os.path.exists(path) else None
        if args.check:
            if on_disk != content:
                drift.append(rel)
            continue
        if on_disk != content:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, "w") as f:
                f.write(content)
            print(f"wrote {rel}")
        if path.endswith("config.yml"):
            for entry in ensure_symlinks(os.path.dirname(path)):
                print(f"  linked {entry}")

    if args.check:
        if drift:
            print("drift detected:\n  " + "\n  ".join(drift))
            sys.exit(1)
        print(f"ok: {len(targets())} files match the spec")


if __name__ == "__main__":
    main()
