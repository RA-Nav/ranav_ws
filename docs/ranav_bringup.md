# ranav_bringup

`src/ranav_bringup`, package `ranav_bringup`. It holds what starts the RaNAV system from the other packages: launch
files, parameter files and RViz configurations. It contains no code. The nodes it starts come from:

- the forks (fast_lio, sc_pgo_ros2, tare_planner, far_planner, terrain_analysis, fast_lio_localization);
- the project packages ([ranav_mission](ranav_mission.md), [map_generator](map_generator.md), [people_perception](people_perception.md)).

These files used to live inside the forks. Keeping them here leaves the forks with only their own fixes.

| File | What it is | Was in |
|---|---|---|
| `launch/explore_warehouse_launch.xml` | **The exploration**: FAST-LIO + SC-PGO, TARE driving OptiPessi; saves the map and shuts down at the end | tare_planner |
| `launch/inspection_launch.xml` | **The inspections**: FAST-LIO, localization, FAR and the map point picker | (new) |
| `launch/fast_lio_slam_launch.xml` | FAST-LIO + SC-PGO on the Airy, people filtering, RViz (included by both) | fast_lio |
| `launch/far_planner_launch.xml` | FAR navigation and inspection of picked points (included by `inspection_launch.xml`) | far_planner |
| `launch/localization_launch.xml` | Localization in the map of an exploration, fast_lio_localization with the Airy values (included by `inspection_launch.xml`) | fast_lio_localization |
| `config/fast_lio_airy.yaml`, `config/fast_lio_airy_real.yaml` | FAST-LIO on the simulated and the real Airy ([fast_lio.md](fast_lio.md)) | legged_gazebo |
| `config/terrain_airy.yaml` | terrainAnalysis / terrainAnalysisExt on the Airy | legged_gazebo |
| `config/tare_aliengo.yaml` | TARE parameters (`tare_planner_node`) | tare_planner (`config/aliengo.yaml`) |
| `rviz/tare_planner_warehouse.rviz` | RViz of the exploration | tare_planner |
| `rviz/far_planner_nav.rviz` | RViz of FAR navigation | far_planner |
| `rviz/fastlio_remote.rviz` | FAST-LIO RViz for a remote machine (not started by the launch files) | fast_lio |

Files are installed as copies: rebuild `ranav_bringup` after editing them. The launch files find every other file
through its package (`$(find-pkg-share ...)`), never through a source path.

## Order

In simulation, start each step in its own terminal (see `README.md`):

1. the world and the controller;
2. once the robot stands still at the spawn (FAST-LIO takes gravity and `camera_init` from the first IMU samples),
   one mission, each a single launch file:
   - `ros2 launch ranav_bringup explore_warehouse_launch.xml`: the exploration. At the end it saves the map in
     `save_data/` of map_generator and shuts down;
   - then, for the inspections, `ros2 launch ranav_bringup inspection_launch.xml`, with the robot at the same spawn.

The other launch files are their parts. Each include is inside a `<group>`: the arguments given to an include would
otherwise stay set for the rest of the including file (for example `rviz:=false`).

## `fast_lio_slam_launch.xml`

It starts:

- the Airy driver (`rslidar_sdk`), with `lidar:=real` only;
- `fastlio_mapping` with the Airy config. It also publishes `odom -> lio_map`, the static base frames and
  `/lio_base_odometry`, see [fast_lio.md](fast_lio.md). `/Laser_map` is off (`map_en: false`): FAST-LIO appends the
  last scan to it every second and never clears it, so it grows for the whole session. The map to look at is
  SC-PGO's `/aft_pgo_map`;
- SC-PGO (`alaserPGO`), with `/aft_mapped_to_init` remapped to `/Odometry` and the keyframe scans taken from
  `/cloud_registered_body_static`, see [sc_pgo.md](sc_pgo.md);
- `people_perception`. Its `dynamic_points_filter` republishes the FAST-LIO scans: `/cloud_registered_body` becomes
  `/cloud_registered_body_static` for SC-PGO, and `/cloud_registered` becomes `/registered_scan_static` for TARE and
  FAR. With `people:=true` the tracked people are removed from those scans and sent to OptiPessi;
- RViz (`fast_lio`'s own `rviz/fastlio.rviz`). Its CloudMap display shows `/Laser_map`, so it stays empty.

| Argument | Default | |
|---|---|---|
| `lidar` | `sim` | `sim`: Gazebo's Airy. `real`: the real Airy; `rslidar_sdk` is started here and wall-clock time is used |
| `use_sim_time` | `true` with `sim`, `false` with `real` | |
| `config` | `config/fast_lio_airy.yaml` or `config/fast_lio_airy_real.yaml` | A custom config must contain the `frames` and `alignment` blocks, otherwise the TF tree stays disconnected |
| `rviz` | `true` | |
| `pgo` | `true` | Run SC-PGO. It empties `save_data/Scans/` at start |
| `people` | `true` | Detect and track people, and keep them out of the SC-PGO and TARE scans |
| `detector` | `auto` | `centerpoint` (TensorRT), `centerpoint_cpu` (ONNX Runtime) or `auto` |
| `centerpoint_model` | `models/lidar_centerpoint/v4.1/short_range` of the workspace | |
| `opti_pessi_obstacles` | `true` | Send the tracked people to `/opti_pessi/obstacles` |
| `tracker` | `people_tracker` | Who tracks the people OptiPessi keeps out of. `autoware`: `autoware_multi_object_tracker` (`config/multi_object_tracker_airy.yaml`, Autoware's file with `world_frame_id: camera_init` and `ego_frame_id: sensor_level`) on `/perception/detector/objects`, and ranav_mission `opti_pessi_obstacle_bridge`; `people_tracker` then sends no obstacles, but still feeds `dynamic_points_filter`. `explore_warehouse_launch.xml` and `inspection_launch.xml` forward it |
| `save_directory` | `$(find-pkg-share map_generator)/save_data/` | SC-PGO output, see [map_generator.md](map_generator.md) |

## `explore_warehouse_launch.xml`

The whole exploration, started once the robot stands still at the spawn.

| What runs | Notes |
|---|---|
| `fast_lio_slam_launch.xml` | Included with SC-PGO (`pgo:=true`, it writes the keyframes of the map) and without its RViz |
| terrainAnalysis, terrainAnalysisExt | `config/terrain_airy.yaml` (with `terrain:=true`) |
| TARE (`tare_planner_node`) | `config/tare_aliengo.yaml`. It plans on `/lio_base_odometry` and `/registered_scan_static` |
| `opti_pessi_goal_bridge` (ranav_mission) | Turns TARE's output into OptiPessi goals in `odom`; with `waypoint:=true`, `/way_point` as it is |
| `tare_finish_merge_map.py` (ranav_mission) | With `merge_map`: at the end, return home, stop, merge the map |
| `far_planner_launch.xml` | With `far`: far_planner and graph_decoder only (`graph_only`), on the terrain maps above; with `people`, also the `far_static` terrain pair |
| exit_inspection nodes | With `exits` (package not in this workspace) |

**far.** By default far_planner does not run here, and its graph is built in the inspections. With `far:=true` it
runs only to build its visibility graph, which is then ready and in the frame of the map before the first
inspection. It steers nothing. Its `/way_point` and `/navigation_boundary` are moved to `/far_planner/...`. The goal
bridge follows TARE's `/way_point`, and TARE takes `/navigation_boundary` as its viewpoint boundary.

**End.** When TARE reports the exploration finished, `tare_finish_merge_map.py` brings the robot home, stops it and
saves the merged map, `optimized_map.pcd`. With `exits` it also saves `exits.yaml`. With `far` it saves
far_planner's graph as `warehouse.vgh` and prunes it with `optimized_map_dynamic.pcd`. This replaces the
`warehouse.vgh` already there. The graph before pruning stays as `warehouse_unpruned.vgh`. Then it exits, and with
`on_exit="shutdown"` the whole launch shuts down: FAST-LIO, SC-PGO, people perception, terrain, RViz. With
`merge_map:=false` nothing is saved and the launch keeps running until Ctrl+C.

Main arguments: `lidar` (`sim` or `real`), `people`, `auto_start` (default `false`: waits for
`/start_exploration`), `waypoint`, `merge_map`, `return_home`, `return_home_timeout`, `map_name`
(`optimized_map.pcd`), `tare_config` (`config/tare_aliengo.yaml`), `far` (default `false`), `exits`, `rviz`.

## `inspection_launch.xml`

The inspections, started once the robot stands still at the spawn of the exploration whose map they use.

| What runs | Notes |
|---|---|
| `fast_lio_slam_launch.xml` | Included without SC-PGO (`pgo:=false`): an inspection does not use it, and it empties `save_data/Scans/`, the keyframes of the exploration, when it starts. Without its RViz |
| `localization_launch.xml` | The robot in the map of the exploration, without its RViz |
| `far_planner_launch.xml` | Terrain maps, far_planner, the goal bridge and the inspection goals, with its RViz |
| `map_point_picker.py` (map_generator) | With `picker`: pick the points on http://localhost:8765, then Start inspection. Started as an executable: it parses its own arguments |

Arguments: `lidar`, `people`, `map` (`optimized_map.pcd` in `save_data/`), `picker`, `rviz`, and those of
`far_planner_launch.xml` (`auto_start`, `goals_file`, `dwell_time`, `loop`, `people_free`, ...).

## `far_planner_launch.xml`

Included by `inspection_launch.xml`, and by `explore_warehouse_launch.xml` with `far:=true`. Alone, it starts after
`fast_lio_slam_launch.xml` and `localization_launch.xml`, with the robot standing at the spawn, so that `camera_init`
is that of the exploration.

| What runs | Notes |
|---|---|
| terrainAnalysis, terrainAnalysisExt | `config/terrain_airy.yaml`. With `people_free` (default): a second pair on the scans without people for far_planner, and the first pair is terrainAnalysis only (`/terrain_map` for the goal bridge) |
| far_planner, graph_decoder | `far_config` of far_planner |
| `far_inspection_goals.py` (ranav_mission) | Sends the points of `goals_file` (or of the Start inspection button of `map_point_picker.py`) as goals, one after the other; loads `graph_file` at the start and saves far_planner's graph there after each inspection |
| `opti_pessi_goal_bridge` (ranav_mission) | Goals for OptiPessi along far_planner's route; with `waypoint:=true`, `/way_point` as it is |

Main arguments: `graph_file` and `goals_file` (both in `save_data/` of map_generator), `load_graph`, `save_graph`,
`inspection`, `auto_start`, `dwell_time`, `loop`, `people_free`, `waypoint`, `terrain`, `rviz`.

`graph_only:=true` starts far_planner and graph_decoder only, with the `far_static` terrain pair when `people_free`:
no goal bridge and no inspection goals. far_planner's `/way_point` and `/navigation_boundary` become
`/far_planner/way_point` and `/far_planner/navigation_boundary`. It is for building the graph next to another
planner (the exploration with `far:=true`).

## `localization_launch.xml`

Included by `inspection_launch.xml`. Alone, it starts after `fast_lio_slam_launch.xml`, with the robot standing at the
spawn. It includes the launch file of
fast_lio_localization ([fast_lio_localization.md](fast_lio_localization.md)), which uses the FAST-LIO already running,
with the values of the Aliengo:

| Value | |
|---|---|
| `map` | `optimized_map.pcd` in `save_data/` of map_generator, left by the exploration |
| `fov` `6.28`, `fov_far` `60.0` | The Airy is a 360 deg LiDAR with a 60 m range: the map is cropped by distance only |
| `init_from_odom` `true` | The initial pose is guessed from the legged estimator (`/odom`): no pose is given by hand |
| `python` | `<workspace>/.venv/bin/python` (open3d) when it exists, else `python3` |

TF: `camera_init -> global_map`, below `odom -> lio_map -> camera_init`. Arguments: `map`, `use_sim_time`, `rviz`.
