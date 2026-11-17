# map_generator

`src/map_generator`, package `map_generator`. It holds the map tools that work on the SC-PGO output, and that output
itself (`save_data/`). It is a project package, not a fork. It brings together tools that used to live inside the
SC-PGO fork (`SC_PGO_ROS2/utils`), so that the fork keeps only its own fixes (see [sc_pgo.md](sc_pgo.md)).

| Tool | What it does | Needs |
|---|---|---|
| `make_merged_map.py` | Merges the SC-PGO keyframes into one map, without what moved | numpy, scipy, open3d |
| `dynamic_removal.py` | Module: finds the points of things that moved | numpy, scipy |
| `prune_far_graph.py` | Takes the obstacles left by people out of a far_planner graph | numpy, scipy, open3d |
| `map_point_picker.py` | Web page to pick inspection points on the map, with a live view of the robot | numpy (rclpy for the live view) |

The scripts are installed to `lib/map_generator`, together with the files they read (`dynamic_removal.py`,
`index.html`, `jet_table.npy`). open3d is not a system package: use the python of `.venv` in the workspace
(`setup_ranav.sh venv`).

## `save_data/`

This is SC-PGO's `save_directory` (`fast_lio_slam_launch.xml`, argument `save_directory`). It is ignored by git. It
stays in the source folder, so a rebuild never touches it. The install links `share/map_generator/save_data` to it,
so launch files find it through the package: `$(find-pkg-share map_generator)/save_data` (ranav_bringup's
fast_lio_slam_launch.xml, far_planner_launch.xml and localization_launch.xml). `make_merged_map.py` and
`map_point_picker.py` default to it too.

| File | Written by | Read by |
|---|---|---|
| `Scans/`, `odom_poses.txt`, `optimized_poses.txt`, `times.txt` | SC-PGO (`Scans/` is emptied at every start) | `make_merged_map.py` |
| `optimized_map.pcd`, `optimized_map_dynamic.pcd`, `optimized_map.log` | `make_merged_map.py` (run by `tare_finish_merge_map.py` at the end of an exploration) | `map_point_picker.py`, `prune_far_graph.py`, fast_lio_localization (`map`) |
| `warehouse.vgh`, `warehouse_unpruned.vgh`, `warehouse_prune.log` | graph_decoder, then `prune_far_graph.py` | far_planner (`graph_file`), `map_point_picker.py` |
| `selected_points.yaml` | `map_point_picker.py` | ranav_mission far_inspection_goals.py (`goals_file`) |
| `exit_inspection/` | exit_inspection | `map_point_picker.py` |

## End of an exploration

`tare_finish_merge_map.py` (ranav_mission, started by `explore_warehouse_launch.xml`) finds the scripts through this
package. It runs them with `.venv/bin/python` of the workspace; parameter `python` overrides
it. It runs:

1. `make_merged_map.py <save_data> --no-vis --output optimized_map.pcd --dynamic-output optimized_map_dynamic.pcd`;
2. with a `graph_name` and `prune_graph`, `prune_far_graph.py warehouse.vgh optimized_map.pcd optimized_map_dynamic.pcd`.
   `explore_warehouse_launch.xml` sets `graph_name` only with `far:=true` (far_planner builds its graph during the
   exploration); otherwise this is skipped there.

## `make_merged_map.py`

```bash
.venv/bin/python install/map_generator/lib/map_generator/make_merged_map.py
```

The script reads `Scans/*.pcd` and the pose files. It drops points closer than `--near` (0.7 m) to the lidar, runs
the removal of moving objects, transforms each keyframe by its pose, and writes `optimized_map.pcd` (binary
compressed, x y z intensity). Unless `--no-vis` is given, it opens a viewer first. Every keyframe with a pose is used.
If the newest keyframes have no pose yet, only the first `min(scans, poses)` are used.

| Option | Default | |
|---|---|---|
| `data_dir` | `save_data/` of this package | SC-PGO save directory |
| `--output` | `optimized_map.pcd` | Relative: inside `data_dir` |
| `--keep-dynamic` | off | Keep the points of things that moved |
| `--dynamic-output` | none | Also save the removed points |
| `--near` | `0.7` | m; 0 keeps every point |
| `--max-range` | `0` | m; drop points farther from their keyframe, after the removal; 0 keeps them |
| `--no-vis` | off | No viewer (unattended runs) |
| `--color`, `--no-floor`, `--floor-margin`, `--point-size` | | Viewer only |

**Changes from SC-PGO's `makeMergedMap.py`:**
- command-line arguments instead of a hard-coded path;
- open3d instead of pypcd (numpy 2), and a single read per scan;
- fallback to `odom_poses.txt` when there are no optimized poses;
- removal of moving objects;
- **no 1000-keyframe cap and no fixed preallocation**: the fork stopped at 1000 keyframes without a warning;
- default output `optimized_map.pcd`, the name every other tool reads;
- no live viewer, `node_skip` or unused colour table.

On the same keyframes it writes the same map as the version it replaces.

## `dynamic_removal.py`

This is a visibility check like Removert's (Kim & Kim, IROS 2020). Every map point is looked at from every keyframe,
in that keyframe's range image:

- a point in front of what the keyframe saw in that direction is a vote for "dynamic";
- a point at about the range seen is a vote for "static".

Votes are cast per 0.1 m voxel. A voxel goes when it has enough dynamic votes and several times more of them than
static ones. No detector is needed: a person the people filter missed is removed as soon as another keyframe sees
through where they stood.

> **Note.** `Params` is tuned for the Airy as FAST-LIO feeds it: `res_azimuth = 1.6 deg` is every 4th of the 900
> columns (`point_filter_num: 4` in `fast_lio_airy.yaml`), and `res_elevation = 1.0 deg` is the ring spacing. Change
> them together with the lidar or with `point_filter_num`.

## `prune_far_graph.py`

```bash
.venv/bin/python install/map_generator/lib/map_generator/prune_far_graph.py \
  warehouse.vgh optimized_map.pcd optimized_map_dynamic.pcd
```

far_planner keeps in its graph the obstacle polygons of people it saw for long enough. The script judges each
polygon on the points inside its outline on the floor plane. A polygon with at least `--min-dynamic` removed points
inside, and fewer static points than `--static-ratio` times them, is a ghost: its nodes go. The graph is pruned in
place, and the original is kept as `<stem>_unpruned.vgh`.

## `map_point_picker.py`

```bash
ros2 run map_generator map_point_picker.py
```

Open http://localhost:8765. The map is `optimized_map.pcd` in `save_data/` unless given. The page shows it from above:

- click to add a point, drag to move it, right-click to delete it, then Save (Ctrl+S);
- the far_planner graph (`--graph`, default `warehouse.vgh` next to the map) is drawn over the map;
- the exits of `exit_inspection/exits.yaml` are drawn with their last verdicts.

**Live view.** With ROS 2 sourced, and unless `--no-live` is given, the page also shows:

- the robot (`lio_base`), its trail, the FAR and TARE paths, and OptiPessi's plan;
- everything brought to the map frame through TF: `global_map` when fast_lio_localization runs, else `camera_init`.

**Buttons.** **Start inspection** saves the points and sends their file on `/start_inspection` to ranav_mission's
`far_inspection_goals.py`. **Start exit inspection** sends `exits.yaml` on `/start_exit_inspection`.

The PCD reader is the script's own: ascii, binary and binary_compressed, numpy only. The system python3 is enough.
The page can write files, so it listens on 127.0.0.1 unless `--host` says otherwise.

## Tests

```bash
cd src/map_generator && python3 -m pytest test/
```

`test/conftest.py` puts `scripts/` on the path. ROS 2 must be sourced for the live-view tests. The system python3 is
enough: `dynamic_removal` needs numpy and scipy only.
