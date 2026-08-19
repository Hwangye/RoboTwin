#!/usr/bin/env python3
"""Plan a task over an explicit seed list and save the joint trajectories.

    python script/plan_seed_list.py <task_name> <task_config> <seed_file>

collect_data.py's own seed search walks the integers from 0 and skips whatever
fails, so two robots collecting "50 episodes" end up on different scenes. This
walks the given seeds in order instead -- e.g. the seed list of an existing
dataset -- so the episodes line up scene for scene. Failed seeds are dropped,
not substituted.

Writes <save_path>/<task>/<config>/{_traj_data/episodeN.pkl, seed.txt}; then a
plain `python script/collect_data.py <task> <config>` with `use_seed: true` in
the task config replays them into hdf5 + video + instructions.
"""
import sys, os, json, traceback, importlib

sys.path.append("./")

import yaml
from envs import *
from envs._GLOBAL_CONFIGS import CONFIGS_PATH


def main(task_name, task_config, seed_file):
    seeds = [int(x) for x in open(seed_file).read().split()]
    env_cls = getattr(importlib.import_module(f"envs.{task_name}"), task_name)

    args = yaml.safe_load(open(f"./task_config/{task_config}.yml"))
    emb = yaml.safe_load(open(os.path.join(CONFIGS_PATH, "_embodiment_config.yml")))
    et = args["embodiment"]

    def robot_file(name):
        return emb[name]["file_path"]

    if len(et) == 1:
        args.update(left_robot_file=robot_file(et[0]), right_robot_file=robot_file(et[0]),
                    dual_arm_embodied=True, embodiment_name=str(et[0]))
    elif len(et) == 3:
        args.update(left_robot_file=robot_file(et[0]), right_robot_file=robot_file(et[1]),
                    embodiment_dis=et[2], dual_arm_embodied=False,
                    embodiment_name=f"{et[0]}+{et[1]}")
    else:
        raise SystemExit("embodiment must have 1 or 3 entries")

    for side in ("left", "right"):
        cfg_path = os.path.join(args[f"{side}_robot_file"], "config.yml")
        args[f"{side}_embodiment_config"] = yaml.safe_load(open(cfg_path))
    args.update(task_name=task_name, task_config=task_config, dual_arm=False,
                save_data=False, render_freq=0, need_plan=True)
    args["save_path"] = os.path.join(args["save_path"], task_name, task_config)
    os.makedirs(args["save_path"], exist_ok=True)

    kept = []
    for s in seeds:
        idx = len(kept)
        try:
            env = env_cls()
            env.setup_demo(now_ep_num=idx, seed=s, **args)
            env.play_once()
            ok = env.plan_success and env.check_success()
            if ok:
                env.save_traj_data(idx)
                kept.append(s)
            print(f"seed {s:5d} -> {'OK  ep%d' % idx if ok else 'fail'}"
                  f"   ({len(kept)}/{len(seeds)} kept)", flush=True)
            env.close_env()
        except Exception:
            print(f"seed {s:5d} -> CRASH", flush=True)
            traceback.print_exc()
        with open(os.path.join(args["save_path"], "seed.txt"), "w") as f:
            f.write(" ".join(str(x) for x in kept))
    print(f"done: kept {len(kept)}/{len(seeds)}; now run:"
          f"  python script/collect_data.py {task_name} {task_config}", flush=True)


if __name__ == "__main__":
    if len(sys.argv) != 4:
        raise SystemExit(__doc__)
    from test_render import Sapien_TEST  # script/ is on sys.path when run as a file
    Sapien_TEST()
    main(sys.argv[1], sys.argv[2], sys.argv[3])
