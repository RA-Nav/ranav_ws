# SC-PGO: changes from the fork

This page covers the `sc_pgo_ros2` package in `FAST_LIO_SLAM_ROS2/SC_PGO_ROS2`: Scan Context loop closure and pose
graph optimization (GTSAM iSAM2) on top of FAST-LIO. It lists what has been changed on top of the fork and why.

- **Fork base:** commit `4835e4e` ("automatic lidar frame id", 2025-09-12).
- **Full diff:** `git -C src/FAST_LIO_SLAM_ROS2 diff 4835e4e -- SC_PGO_ROS2`
- **Started by** `fast_lio_slam_launch.xml` (package `ranav_bringup`, see [ranav_bringup.md](ranav_bringup.md)); the fork's
  `sc_pgo.launch.py` is not used.

## What is in the fork and what is not

Only C++ has changed: `src/laserPosegraphOptimization.cpp` and one include in `include/scancontext/Scancontext.h`.
Every change is a bug fix, except the loop check, which is a new filter.

The map tools built on the SC-PGO output live in their own package, [map_generator](map_generator.md): the keyframe
merge with removal of moving objects, the pruning of the far_planner graph, and the point picker. `utils/python` is as
in the fork. Its `makeMergedMap.py` needs `pypcd` and has a hard-coded data path, so it is not used: use
`map_generator make_merged_map.py` instead.

## Summary

| Change | Kind | Why |
|---|---|---|
| [`optimized_poses.txt` is written](#output-files) | Bug fix | The fork never set its path, so the optimized poses were never saved |
| [Atomic rewrite of the pose files](#output-files) | Bug fix | A shutdown during the 1 Hz rewrite left a truncated file for the readers |
| [`save_directory` created before `times.txt`](#output-files) | Bug fix | `times.txt` was not created when the folder did not exist |
| [Clean exit](#shutdown) | Bug fix | Ctrl+C ended in `std::terminate` |
| [Target submap at its own poses](#loop-closure) | Bug fix | All the scans of the target submap were stacked on one pose |
| [Loop relative pose](#loop-closure) | Bug fix | Wrong for loops between keyframes that are not at the same place |
| [Odometry check of the loops](#loop-closure) | New filter, parameters `loop_max_*` | Rejects false Scan Context matches, e.g. in symmetric rooms |
| `cv_bridge/cv_bridge.hpp` | Build | `cv_bridge.h` is deprecated on Jazzy |

## Output files

The node writes into `save_directory`. In this project it is `save_data/` of the map_generator package
(`$(find-pkg-share map_generator)/save_data/`), set by `fast_lio_slam_launch.xml`.

| File | Content |
|---|---|
| `Scans/NNNNNN.pcd` | Keyframe scans in the lidar frame (`/cloud_registered_body_static`), full resolution. The folder is emptied at start |
| `odom_poses.txt` | FAST-LIO pose of each keyframe, KITTI format (3x4 row-major), rewritten every iSAM cycle (1 Hz) |
| `optimized_poses.txt` | Optimized pose of each keyframe, same format and rate |
| `times.txt` | Stamp of each keyframe |

- **`optimized_poses.txt` was never written.** The fork declared `pgKITTIformat` but never set it, so the file name
  was empty. It is now `save_directory + "optimized_poses.txt"`.
- **The pose files are replaced atomically.** Each file is written to `<name>.tmp` and renamed over the old one, so
  readers such as the merge at the end of an exploration never see a half-written file.
- **The folder is created first.** `mkdir -p <save_directory>Scans/` now runs before `times.txt` is opened.

## Shutdown

The worker threads loop on `rclcpp::ok()`, some sleeping up to 10 s, and are never joined. Returning from `main`
destroyed joinable `std::thread`s, which calls `std::terminate`. `main` now calls `rclcpp::shutdown()` and then
`std::_Exit(0)`. This skips destructors, which is safe here: `times.txt` is flushed at every line (`std::endl`), and
the scans and pose files are written synchronously.

## Loop closure

A loop candidate comes from Scan Context and is verified by ICP in `doICPVirtualRelative`. The current keyframe's
scan is placed at the pose of the loop keyframe (the "virtual" scan) and aligned to a submap around the loop
keyframe.

**Target submap at its own poses.** The fork placed each of the up to 51 keyframes of the target submap
(`historyKeyframeSearchNum`) at the pose of the loop keyframe. Scans taken along up to 50 m of path ended up stacked
on one pose. The result was a smear that any scan fitted under the ICP fitness threshold. Each keyframe is now placed
at its own pose (`root_idx = -1` in `loopFindNearKeyframesCloud`), so the submap is the map around the loop keyframe.

**Relative pose.** ICP returns `correction`, which moves the virtual scan onto the map. The current keyframe is then
at `correction * root`, where `root` is the loop keyframe's pose. The `BetweenFactor(loop, current)` therefore gets:

```
relative = root^-1 * correction * root
```

The fork returned `correction^-1`. That is right only when both keyframes are at the same place: for a loop 3 m away,
it pulled the current keyframe 3 m the wrong way.

**Odometry check.** A loop is kept only if its relative pose agrees with FAST-LIO's relative pose of the same two
keyframes. The disagreement allowed is what the odometry can have drifted over the path between them:

| Parameter | Default | |
|---|---|---|
| `loop_max_correction` | `1.0` | Fixed part of the allowed translation, in m |
| `loop_max_correction_ratio` | `0.05` | Plus this times the odometry path between the two keyframes |
| `loop_max_rot_correction` | `15.0` | Allowed rotation, in deg |

Scan Context is rotation invariant. A symmetric room, such as an empty square, therefore matches from the opposite
corner, 15–30 m and about 180 deg away. Such a loop would fold the map onto itself. Rejected and accepted loops are
logged (`[SC loop] odometry test ...`).

The check is **on by default**, so it changes the fork's behaviour. Very large values turn it off.
`fast_lio_slam_launch.xml` does not set these parameters, so the defaults apply.

## Parameters set by `fast_lio_slam_launch.xml`

| Parameter | Value | |
|---|---|---|
| `save_directory` | `$(find-pkg-share map_generator)/save_data/` | `save_directory` argument of the launch file |
| `keyframe_meter_gap` | `1.0` | A new keyframe every 1 m |
| `keyframe_deg_gap` | `10.0` | or every 10 deg |
| `sc_dist_thres` | `0.3` | Scan Context distance for a loop candidate |
| `sc_max_radius` | `20.0` | Indoor (the Airy reaches 60 m) |
| `mapviz_filter_size` | `0.1` | Voxel size of `/aft_pgo_map` |

Remaps: `/aft_mapped_to_init` becomes `/Odometry`, and `/velodyne_cloud_registered_local` becomes
`/cloud_registered_body_static` (the FAST-LIO body scans without the tracked people, from people_perception).

## Known issues

- **Stale pose files at start.** At start the node empties `Scans/` and `times.txt` but not the pose files. They keep
  the previous session's poses until the first iSAM cycle, at most 1 s after the first keyframe.
  `make_merged_map.py` only stacks `min(scans, poses)` keyframes, so the effect is limited to that window.
- **`Scans/` is emptied at every start.** Launch with `pgo:=false` to keep the keyframes of a previous exploration.
- **Build warning.** `variable 'unused' set but not used` comes from the fork's `system()` calls.
