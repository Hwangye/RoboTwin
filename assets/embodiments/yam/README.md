# YAM (I2RT) embodiment for RoboTwin

Single-arm 6-DoF YAM with the I2RT `linear_4310` parallel-jaw gripper and a
D405 wrist camera. Used bimanually as `embodiment: [yam, yam, <spacing>]`.

## Reproducing the lift_pot dataset from a fresh clone

```bash
python assets/_download.py                        # objects.zip etc. (standard RoboTwin assets)
python script/update_embodiment_config_path.py    # expand curobo_tmp.yml -> curobo.yml (abs paths)
python script/plan_seed_list.py lift_pot collect_yam task_config/lift_pot_official_seeds.txt
python script/collect_data.py lift_pot collect_yam
```

Step 3 plans the official aloha seed list (the one `replay_official` was
collected on) and writes `_traj_data/` + `seed.txt`; step 4 replays those into
hdf5 + video + instructions under `data/lift_pot/collect_yam/`. Expected:
50/50 kept, 101-110 frames per episode at `save_freq 15`. Note
`collect_data.sh` calls a `script/.update_path.sh` that is not in the repo (it
no-ops silently) -- run `update_embodiment_config_path.py` yourself, once per
checkout location.

## Provenance

| file | origin |
| --- | --- |
| `yam_raw.urdf` | `i2rt/robot_models/arm/yam/v1/yam_linear_4310_d405.urdf`, [i2rt-robotics/i2rt](https://github.com/i2rt-robotics/i2rt), MIT (`LICENSE_i2rt`) |
| `assets/*.stl` | same repo, unmodified |
| `UPSTREAM_README.md` | vendor's physical-properties sheet for the arm |
| `yam.urdf`, `collision_yam.yml` | generated from `yam_raw.urdf` by `tools/build_yam_asset.py` |
| everything else | written by hand for this repo |

`tools/build_yam_asset.py` is the single source of truth for the generated
files -- re-run it and the asset is reproduced exactly. Do not hand-edit
`yam.urdf`; edit the script.

The vendor URDF is an Onshape export: **visual only**, no `<collision>`, and
placeholder actuator limits. Everything needed to simulate it was added here.

## Local edits to the URDF

1. **Collision geometry** — 27 oriented boxes fitted to the visual meshes
   (each link cut into slabs along its longest axis, one minimal OBB per slab).
   The vendor file has none, so the gripper would pass through everything.
2. **Actuator limits** — upstream ships `effort="1" velocity="1"` on every
   joint, i.e. 1 N·m, which cannot hold the arm against gravity. Replaced with
   the MJCF's real motor ranges: 28 N·m for `joint1..3` (dm4340) and 10 N·m
   for `joint4..6` (dm4310), 100 N for the fingers.
3. **Wrist camera frame** — the vendor `camera` link is ROS-optical
   (+z forward, +y down). Renamed to `camera_optical` and a new `camera` link
   added under it rotated into SAPIEN's convention (+x forward, +y left,
   +z up), which is what RoboTwin feeds to `update_wrist_camera`.
4. **Materials** — the Onshape export assigns arbitrary per-part colors
   (purple base, teal tube, pink wrist). Repainted to the real machine's
   scheme, following the vendor-tuned MJCF: matte black (0.06) everywhere,
   white (0.9) for the two arm-tube links. Visual only.
5. **Camera collision removed** — the D405 bracket sticks out 80 mm sideways
   from the gripper axis and knocks table objects over during a side approach.
   The camera links keep their visuals (so the wrist view is right) but carry
   no collision. Deliberate deviation from the real hardware.

## Calibration (all measured, not guessed)

| quantity | value | how |
| --- | --- | --- |
| flange (`gripper` link) → fingertip pad centre | **0.138 m** | pad contact face on the tip meshes spans z ∈ [-0.1468, -0.1292] in the gripper frame |
| finger travel | `joint7=joint8=0` → pads touching; `-0.04695` → 94.0 mm open | swept the tip meshes |
| approach axis | `-z` of the `gripper` link | mesh geometry |
| closing axis | `+y` of the `gripper` link | `joint7` axis |
| `global_trans_matrix` | R(sapien `joint6` frame) → R(`gripper` link) | measured in SAPIEN; the same recipe reproduces aloha-agilex's published `diag(1,-1,-1)` exactly, which is what validates it |
| `delta_matrix` | maps RoboTwin's canonical grasp frame into the `gripper` frame | solved from `D·e_x = approach`, `D·c_aloha = closing` |

`gripper_scale`'s closed end (`0.010`) sits 10 mm *past* the joint limit on
purpose: the joint stops at the limit and the drive keeps pushing, and that
residual is the squeeze force. `aloha-agilex` does the same (limit `0`,
scale `-0.01`).

## Placement

`robot_pose` puts the base at `z = 0.80`, i.e. on a 60 mm riser above the
0.74 table. Base **yaw does not matter** for reach (joint1 absorbs it); base
**spacing does**. At the ABC station's real 0.62 m the canonical side-grasp
poses are unreachable and the planner falls back to approaches rotated by
0.5–1.0 rad, which sweep the fingers into the object. At 0.80 m the canonical
poses come within IK tolerance (rotation error 0.056 → 0.000).

## curobo

`curobo_tmp.yml` → `curobo.yml` via `script/update_embodiment_config_path.py`.
Notes:

* `base` is **not** in `collision_link_names`: it is bolted to the table, so
  its spheres always intersect the table cuboid and every start state would
  report a collision.
* `planner.exact_table: true` — opt-in flag added to `CuroboPlanner`. The
  legacy table pose is mirrored to the wrong side of the base, which arms
  mounted high above the table never noticed but a table-mounted arm does.
* fingers are locked at `-0.0235` (half open), which is the opening the task
  scripts actually approach with (`close_gripper(pos=0.5)`).
* `collision_yam.yml` — 211 spheres, recursive kd-split of each link's solid
  voxelisation with a per-link radius cap, plus `collision_sphere_buffer:
  0.001`. The caps matter more than they look: at r=0.017 a 6 mm fingertip pad
  inflates to 34 mm and the planner reads the lower finger as buried in the
  table, and a forearm cap of 0.042 (vs the real 0.030 half-width) is what
  decides whether the canonical side-grasp poses are reachable at all.

## SRDF

`sapien`'s `URDFLoader.parse_srdf` only honours `reason="Default"` —
`"Never"` and `"Adjacent"` pairs are silently dropped. Every pair that has to
hold in simulation is therefore written as `"Default"`.

## Control accuracy: three settings that were silently costing the grasp

Worth knowing before running any task with this embodiment, because none of
them are YAM-specific -- they were just never noticed on arms with fatter
fingers.

**0. The "straight line in" is not constrained.** `grasp_actor` plans the
pre-grasp -> grasp segment with `constraint_pose=[1, 1, 1, 0, 0, 0]`, which
becomes curobo's `hold_vec_weight`: the first three entries hold rotation, the
last three hold translation. All three translation weights are zero, so
**translation is entirely free** and the planner is at liberty to bow the
gripper sideways on the way in. Traced step by step on `lift_pot`, the handle's
position in the gripper frame during that 35 mm segment:

| | offset along the closing axis | pot displaced |
| --- | --- | --- |
| `[1,1,1,0,0,0]` (default) | +0.2 mm -> **-24.6 mm** -> +1.4 mm | **21.2 mm** |
| `[1,1,1,0,1,1]` | +0.2 mm -> +1.8 mm -> +1.1 mm | **0.0 mm** |

The jaw gap is +/-23.5 mm, so at -24.6 mm the handle has left the jaw entirely
and a fingertip rakes across it -- which is what shoved the pot 21 mm and left
the jaws closing on air. `robot.get_constraint_pose()` already rotates the
translation weights from RoboTwin's canonical grasp frame (+x = approach) into
each embodiment's ee frame, so `[1,1,1,0,1,1]` -- free along the approach, held
laterally -- is portable as written. Set it with `grasp_constraint` in
`config.yml`; embodiments that do not set it keep the old behaviour.

With this alone both arms go from closing to *exactly* 0 on every run to
clamping the handle at an 8.6 mm gap.

## Two more, on top of that

Worth knowing before running any task with this embodiment, because neither is
YAM-specific -- they were just never noticed on arms with fatter fingers.

**1. curobo's goal tolerance.** `CuroboPlanner` calls
`MotionGenConfig.load_from_robot_config()` without `position_threshold` /
`rotation_threshold`, so curobo's defaults apply: **5 mm and 0.05**. A plan
reported as `Success` can therefore end at a joint configuration that is
several millimetres and several degrees away from the pose that was commanded.
Measured on `lift_pot`, before/after, for the left arm's pre-grasp pose:

| | planner's own goal | executed pose |
| --- | --- | --- |
| curobo defaults | 3.05 mm / **4.90 deg** | 5.35 deg |
| `0.001` / `0.005` | 0.01 mm / **0.02 deg** | 0.80 deg |

The error was in the *plan*, not the tracking -- execution follows whatever
goal it is handed to within ~0.4 deg. Set per embodiment under `planner:` in
the curobo yml; embodiments that do not set it keep curobo's defaults.

**2. Moves end on drive lag.** `take_dense_action` and `together_move_to_pose`
return the instant the last waypoint is written, so the arm stops wherever the
drive happens to be, typically 2-3 mm short. `settle_steps` in `config.yml`
holds the final target for N extra physics steps (with gravity compensation);
0, the default, reproduces the original behaviour exactly. With `400`, the
grasp-pose error drops from 2.7 mm / 0.23 deg to **1.6 mm / 0.21 deg**, and the
component along the closing axis -- the one that decides whether a thin plate
ends up between the pads -- from 2.66 mm to **1.07 mm**. For reference,
`aloha-agilex` on a successful run lands 1.3 mm out.

**3. The wrist lands on its stop.** YAM's `joint6` range is only +/-120 deg, and
the canonical grasp orientation puts it at exactly +/-2.09 rad -- reachable, so
`choose_best_pose` accepts it, but then every follow-up move that holds
orientation is infeasible. A parallel jaw grasps identically after a half turn
about its approach axis, so `create_target_pose_list` now emits both rolls when
the embodiment sets `gripper_symmetric: true` (half the candidate budget each,
because the batch planner is warmed up for exactly `ROTATE_NUM` goals), and
`choose_best_pose` ranks by distance from the nearest joint stop when
`prefer_joint_margin: true` rather than taking the first candidate that plans
(its original "shortest trajectory" test never fired -- `now_step` is never
updated). `joint6` after the grasp goes from **0.004 rad of margin to 1.12**.

## Grasp depth, approach constraint, and step budget

Three things settled after the first successful runs; all measured on `lift_pot`
seeds 0/1 (`060_kitchenpot/base0`).

**Grasp depth.** The handle is a rod with a ball on the end and the annotated
contact point is on the ball. Physics contacts after closing put the ball at
**+9..+10 mm** along the approach from the annotation for *both* robots -- but
aloha's pads sit +20..+36 mm past its TCP, so aloha actually clamps the rod
with the ball hooked behind the pads and 26 mm of finger to spare toward the
pot; YAM's pads span only TCP +/-8.8 mm, so at depth 0 it pinches the ball with
the extreme tip. `grasp_depth: 0.020` slides `grasp_actor`'s pre-grasp and
grasp poses 20 mm along the approach: pads on the rod at +13..+24 mm, ball
inside the jaw, fingertips 10-15 mm short of the pot wall. Same picture as
aloha, as deep as YAM's geometry allows.

**Do not hold translation on the approach.** `grasp_constraint` went back to
RoboTwin's `[1,1,1,0,0,0]`. The `[1,1,1,0,1,1]` used earlier fixed a sideways
bow that has since gone away on its own (goal tolerance + settle + finetune
re-timing keep the free segment within 4 mm of a straight line), and it turns
out to be poisonous: replaying the three segments on fresh `MotionGen`
instances,

| approach hold | approach steps | lift afterwards (same instance) |
| --- | --- | --- |
| `[1,1,1,0,1,1]` | 277 | **fails** (`FINETUNE_TRAJOPT_FAIL`, fallback `None`) |
| `[1,1,1,0,0,0]` | 113 | ok, 202 |
| none | 104 | ok, 213 |

A fresh instance plans the very same lift in 191 steps; `reset_metric()` and
`motion_gen.reset()` do not recover the used one. That leak is what the whole
`DT_EXCEPTION` chase was about. `finetune_fallback: true` (try with finetune,
retry without on failure) stays as a safety net -- the finetune stage is also
the time-optimal re-timing, and disabling it outright made every move ~2x
longer.

**Step budget.** Official aloha `lift_pot` episodes are ~112 frames at
`save_freq 15` (~1680 physics steps). Per segment, aloha vs YAM now:

| segment | aloha | YAM |
| --- | --- | --- |
| open to half | 300 | 300 |
| home -> pre-grasp | 593 | 377-391 |
| straight 35 mm | 170 | 179-187 |
| close | 300 | 300 |
| lift | 253 | 231-244 |
| **total** | **1616 = 108 frames** | **1387-1422 = 92-95 frames** |

Two changes got there: `settle_steps` is an upper bound (150) with an early
exit once every arm joint is below 0.02 rad/s, and gripper-only actions do not
settle at all -- the earlier fixed 400 steps on all five actions were 2000 of
YAM's 3986. The rest came back with finetune re-timing.

## Status on `lift_pot`

Everything on the robot side checks out:

* the planner now lands its goal to 0.02 deg and execution tracks it to 0.21 deg
* the approach only disturbs the pot by 3-4 mm
* the closing axis matches RoboTwin's canonical grasp frame to within 1e-3
* grasp-candidate reachability is level with aloha's (left arm takes cand1,
  right arm cand0; aloha takes cand0/cand1)

`060_kitchenpot` comes in two variants: one with ear tabs, one with a straight
rod handle. On the rod variant: `aloha-agilex` on seed 1 finishes
its approach 1.3 mm from the annotated contact point, its jaws stop at a 4.2 mm
gap, and the pot goes to 0.881 m (pass). Before the two fixes above, YAM's jaws
closed to *exactly* 0 on every single run -- both fingers swept past the plate.
Afterwards they do catch it (8.6 mm gap on seed 1, pot to 0.843 m), but the
end-to-end success rate is still not there and varies run to run, because
curobo's plans are stochastic and a marginal grasp either holds or does not.

Where it stands now: the approach is correct, the grasp closes on the handle
(8.6 mm jaw gap on the rod variant, 37 mm on the tab variant, pot displaced
0.0 mm), and the lift completes -- `lift_pot` passes `check_success()`.

Getting the lift to plan took one more fix. curobo returned
`MotionGenStatus.DT_EXCEPTION` from the post-grasp state for *every* target
tried -- +/-20 mm, 100 mm, 300 mm, in every direction, and even for a
zero-length move -- so it was the start state, not the goal. Ruled out along
the way: reach (curobo IK solves every one of those goals to 0.0 mm position
error; the TCP only travels from 536 mm to 563 mm from the base, against a
sampled maximum reach well beyond that), goal tolerance (fails at every value
from 0.001/0.005 to curobo's 0.005/0.05), `max_acceleration`/`max_jerk`
(3/100, 8/300, 15/500 all fail), table collision (lowest sphere 0.803 against
a 0.740 table top), and joint limits (>= 0.84 rad of margin everywhere).

`DT_EXCEPTION` is a sub-status of `FINETUNE_TRAJOPT_FAIL`: it is curobo's
*finetune* trajopt stage that cannot settle on a trajectory dt from this state.
`enable_finetune_trajopt: false` under `planner:` plans all of them. The cost is
a little trajectory smoothness. `finetune_dt_scale` was tried first and does
not help.

RoboTwin discards curobo's failure reason and keeps only `"Fail"`, which is why
this went unseen; set `ROBOTWIN_PLAN_DEBUG=1` to have `CuroboPlanner` print it.

Remaining leads, in order:


1. success rate across seeds -- the grasp is still marginal on the tab variant
   of the pot, where the jaws end up on the rim rather than the tab;
2. the residual 1.6 mm of drive lag; more `settle_steps` would shrink it;
3. `enable_finetune_trajopt: false` is a workaround, not a diagnosis -- worth
   finding out what about the post-grasp state upsets the finetune stage.
