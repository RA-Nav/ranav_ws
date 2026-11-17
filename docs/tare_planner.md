# TARE planner: changes from the fork

This page covers the `tare_planner` package in `tare_planner/src/tare_planner`: the TARE exploration planner
(`tare_planner_node`), which sends the robot to explore and publishes `/way_point` and its paths. It lists what has
been changed on top of the fork and why.

- **Fork base:** commit `b6760ad` ("minor update", 2024-07-10), branch `humble-jazzy`.
- **Full diff:** `git -C src/tare_planner diff b6760ad`
- **Started by** `explore_warehouse_launch.xml` (package `ranav_bringup`, see [ranav_bringup.md](ranav_bringup.md)),
  with `config/tare_aliengo.yaml` from the same package. The fork's `explore*.launch`, its `config/*.yaml` and its
  rviz files are unchanged and not used.

## What is in the fork and what is not

Only C++ and `CMakeLists.txt` have changed, in about 100 lines. The new behaviours are parameters whose defaults keep
the fork's behaviour. The other changes are bug fixes, plus one topic the fork had commented out.

What used to be added to the fork now lives in project packages:

- the Aliengo parameters, the exploration launch file and its rviz are in [ranav_bringup](ranav_bringup.md);
- the goal bridges to OptiPessi and the end of the exploration (`tare_finish_merge_map.py`) are in
  [ranav_mission](ranav_mission.md).

## Summary

| Change | Kind | Why |
|---|---|---|
| [OR-Tools installed, RPATH set](#build) | Build | The installed node did not find `libortools.so.9` |
| [`kUseFrontier` declared](#parameters) | Bug fix | The parameter was never declared, so the yaml value was ignored and the flag uninitialized |
| [Wait for `/clock`](#start-up) | Bug fix | With `use_sim_time` the node exited at its first tick |
| [Lookahead point initialized](#start-up) | Bug fix | The first `/way_point` could be uninitialized memory |
| [`kInitialWaypointDistance`](#start-up) | New parameter, default `12.0` as in the fork | The fork always sent a waypoint 12 m straight ahead at start |
| [`rolling_occupancy_grid/kClearDynamicObstacle`](#moving-obstacles) | New parameter, default `false` | People who walked away stayed obstacles and coverage targets |
| [`/exploration_path` published](#exploration-path) | Change | The goal bridge follows it; the fork had the publish commented out |

## Build

The node links the prebuilt OR-Tools shipped in `or-tools/lib`. The installed binary is a copy, since the workspace
is built without `--symlink-install`, and its RPATH was empty. So at run time it could not find `libortools.so.9`
unless `LD_LIBRARY_PATH` was set. `libortools.so.9*` is now installed to `install/tare_planner/lib`, and
`tare_planner_node` has the install RPATH `$ORIGIN/..`, which points there.

## Parameters

`PlanningEnv` and `ViewPointManager` read `kUseFrontier` with `get_parameter()`, but no one declared it. On an
undeclared parameter `get_parameter()` returns false and leaves the member untouched, so the flag was uninitialized
and the yaml value was ignored. It is now declared in `ReadParameters()` with default `true`, as in every config of
the fork.

## Start-up

`execute()` runs on a 1 s wall timer.

- **Clock.** With `use_sim_time` (the default of `explore_warehouse_launch.xml`), `now()` is zero until the first
  `/clock` message. The fork then called `exit(1)`. The node now logs a warning and tries again at the next tick.
- **Lookahead point.** `GetLookAheadPoint()` leaves `lookahead_point_` as it was when it finds no point, for example
  while the first global paths are shorter than `kLookAheadDistance / 2`. The fork never initialized it (Eigen does
  not), so the first `/way_point` could be anywhere. It is now set to the robot position at start.
- **`kInitialWaypointDistance`.** Distance straight ahead of the robot of the waypoint sent before the first plan.
  The fork had 12 m hard-coded. A value of 0 or less sends none. `tare_aliengo.yaml` sets `0.0`: indoors, 12 m straight
  ahead is often through a wall.

## Moving obstacles

The occupancy grid (`RollingOccupancyGrid`) marks the cells hit by the scan as `OCCUPIED`, and ray-casts from the
robot to them to mark the cells in between `FREE`. In the fork every ray stops at the first `OCCUPIED` cell. So a cell
once occupied stays so until it rolls out of the grid. A person who walked by leaves a wall, and the surface points
there remain to be covered: TARE keeps planning viewpoints for something that is gone.

With `rolling_occupancy_grid/kClearDynamicObstacle: true`:

- an `OCCUPIED` cell that the current scan does not hit no longer stops the rays, and the rays passing through it set
  it `FREE`. A cell within one cell of the ray's end is kept: a ray grazing a surface on its way to its end point would
  otherwise carve that surface;
- in `PlanningEnv::GetUncoveredArea()`, a surface point lying in a `FREE` cell is not a coverage target. This applies
  only with `kUseFrontier`, the only case in which the grid is updated.

With the parameter `false` (the default), the rays stop at every `OCCUPIED` cell, as in the fork. `tare_aliengo.yaml`
sets it `true`. TARE's scan, `/registered_scan_static`, is already filtered by people_perception. This handles the
people the filter misses, and anything else that moves.

## Exploration path

`/exploration_path` is TARE's local path, oriented in walking direction, or robot to home while rushing home.
`GetLookAheadPoint()` picks `/way_point` on it. Only when that path is too short does it take a node of the global
path, which is published as `/global_path_full`. The fork had the `/exploration_path` publish commented out.

It is read by:

- ranav_mission `opti_pessi_goal_bridge` (parameter `path_topics`): when the straight line to `/way_point`
  is blocked, it puts the goal ahead along the path `/way_point` lies on, which goes round walls and through doors.
  Without `/exploration_path` that path would be found only on the rare cycles when `/way_point` is on the global
  path, and the bridge would fall back to detours and stops;
- `map_generator map_point_picker.py`, in its live view.

## Known issues

- **Build warnings.** The `-Wsign-compare`, `-Wreorder` and `-Wunused-variable` warnings come from the fork's code and
  the OR-Tools headers.
- **History.** The commits on `humble-jazzy` before `305c11d` also added integration files and `__pycache__`
  folders, which have since been moved out or removed. The tree is clean, but `git log` still shows them.
