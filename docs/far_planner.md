# FAR planner: changes from the fork

This page covers the `far_planner` package in `autonomy_stack_go2/src/route_planner/far_planner`: the FAR route
planner, which builds a visibility graph of the obstacles and plans routes on it. It lists what has been changed on
top of the fork and why. The terrain maps it plans on come from the same fork, see
[terrain_analysis.md](terrain_analysis.md).

- **Fork base:** commit `43d5f54` ("update download link", 2026-02-18), branch `foxy-humble`.
- **Full diff:** `git -C src/autonomy_stack_go2 diff 43d5f54 -- src/route_planner`
- **Started by** `far_planner_launch.xml` in [ranav_bringup](ranav_bringup.md), only. The inspections include it.
  With `far:=true`, the TARE exploration (`explore_warehouse_launch.xml`) includes it with `graph_only`. There
  far_planner only builds its graph, which is saved as `warehouse.vgh` at the end. The parameters are the fork's
  `config/default.yaml` (argument `far_config`). The fork's `launch/far_planner.launch` and
  its rviz are not used.

## What is in the fork and what is not

Two lines of C++ have changed in `far_planner`. `graph_decoder`, `boundary_handler` and `visibility_graph_msg` are as
in the fork.

What used to be added to the fork now lives in project packages:

- the launch file and its rviz are in [ranav_bringup](ranav_bringup.md);
- the inspection goals, with the loading and saving of far_planner's graph (`far_inspection_goals.py`), are in
  [ranav_mission](ranav_mission.md).

## Summary

| Change | Kind | Why |
|---|---|---|
| `tf2_geometry_msgs.hpp`, `tf2_sensor_msgs.hpp` in `utility.h` | Build | Jazzy has no `.h` headers any more |
| [Start on free ground](#start-on-free-ground) | Change | On an open floor the planner never started |

## Start on free ground

The planner loop runs only once `is_cloud_init_` and `is_odom_init_` are set. The fork set `is_cloud_init_` in
`TerrainCallBack()` only when the surrounding cloud held an obstacle point. In an open area with nothing but people
around, and the people taken out of the terrain maps (`people_free`, the `far_static` terrain pair), there was none.
So the planner never started, and a saved graph was never loaded either.

It now starts also when the surrounding cloud holds free ground points. With `groundPriorFill` (see
[terrain_analysis.md](terrain_analysis.md)), the terrain maps always hold some. This is not a parameter: where obstacles
are in view from the first terrain message, the planner starts as in the fork.
