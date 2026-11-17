# people_perception

`src/people_perception` (own repository, AlexBarbi/people_perception), package `people_perception`. It finds the
people in the lidar scans of FAST-LIO and tracks them. It is a project package, not a fork. Its README describes
every node, parameter and check; this page is how it fits into the stack.

[ranav_bringup](ranav_bringup.md) `fast_lio_slam_launch.xml` includes `people_perception_launch.xml` (arguments
`people`, `detector`, `centerpoint_model`, `opti_pessi_obstacles`).

## Nodes

| Node | Runs with | Reads | Publishes |
|---|---|---|---|
| `dynamic_points_filter` | always | `/cloud_registered`, `/cloud_registered_body`, `/Odometry`, `/perception/tracks` | `/registered_scan_static`, `/cloud_registered_body_static`, `/perception/dynamic_points`, `/diagnostics` |
| `detector_cloud_adapter` | `people:=true` | `/cloud_registered_body`, `/Odometry` | `/perception/detector/pointcloud`, TF `camera_init -> sensor_level` |
| `autoware_lidar_centerpoint` (TensorRT) or `centerpoint_cpu.py` (ONNX Runtime) | `people:=true` | `/perception/detector/pointcloud` | `/perception/detector/objects` |
| `people_tracker` | `people:=true` | `/perception/detector/objects`, TF | `/perception/tracks`, `/perception/tracks_markers`, `/opti_pessi/obstacles` |

With `people:=false` the filter is a pass-through. It still runs, because SC-PGO, TARE and the people-free terrain
maps of far_planner read their scans from it. With `people:=true` it takes the tracked people out.

## Who reads what

| Topic | Reader |
|---|---|
| `/registered_scan_static` | TARE (`tare_aliengo.yaml`), `far_static/terrainAnalysis(Ext)` for far_planner ([far_planner](far_planner.md), [terrain_analysis](terrain_analysis.md)) |
| `/cloud_registered_body_static` | SC-PGO ([sc_pgo](sc_pgo.md)) |
| `/opti_pessi/obstacles` | OptiPessiController (legged_control), frame `odom`, the confirmed people within 10 m as `HUMAN` |
| `/perception/tracks_markers`, `/perception/dynamic_points` | rviz (`tare_planner_warehouse.rviz`, group People) |

What avoids collisions (`terrainAnalysis`, `terrainAnalysisExt`, the goal bridge, OptiPessi through its obstacles)
keeps reading the clouds with the people in them. A detector mistake can then dirty a map, but cannot cause a hit.

## Choices made on 2026-11-14

- **The tracker publishes the obstacles of the controller itself.** They used to go through `obstacle_adapter` (a
  people_perception message with radius, maximum speed, stationary flag) and a Python relay to `odom`. The controller
  takes type and position only, uses the radius and speed of its `task.info`, and picks the closest obstacles for its
  slots. So the adapter, the relay and the `people_perception_msgs` package went, and with them two nodes and three
  topics. The 10 m range stays (`people_tracker.yaml` `obstacles.max_range`): every obstacle takes an OCP slot and the
  solve time grows with them.
- **Only the messages of legged_controllers.** `CMakeLists.txt` imports the `Obstacle`/`ObstacleArray` targets
  without `find_package(legged_controllers)`, which would make the configure find the whole controller stack (ocs2,
  pinocchio, casadi).
- **The FAST-LIO odometry alignment** (`odom -> lio_map`, [fast_lio](fast_lio.md)) is what lets the people reach the
  controller: without it the tracker publishes no obstacles.
- **Diagnostics.** The CPU detector skips the scans that come while it is busy. Their clouds go out with the tracks of
  the next scan, and that is no longer reported as a warning. WARN means clouds went out unfiltered, with people in
  them.
- **Removed as unused:** the stationary flag of the tracks and the track heading (nothing read them); the detector
  cloud's intensity scaling (the model reads x y z only, so intensity is 0); the `passthrough` and `drop` modes of the
  filter (only "tracks of the closest scan" was ever used).

## Build and test

```bash
colcon build --packages-select people_perception
colcon test --packages-select people_perception --event-handlers console_direct+
```

It builds after `legged_controllers` (its messages) and `autoware_perception_msgs`. The detector runs the model in
`models/lidar_centerpoint/v4.1/short_range` of the workspace: the pretrained Autoware model proves that the chain runs,
but it is not a warehouse people detector.

## Autoware's tracker for OptiPessi (2026-11-15)

`autoware_multi_object_tracker` (Autoware Universe) can track the people OptiPessi keeps out of instead of
`people_tracker`: `fast_lio_slam_launch.xml tracker:=autoware` ([ranav_bringup](ranav_bringup.md)), with ranav_mission
`opti_pessi_obstacle_bridge` ([ranav_mission](ranav_mission.md)). people_perception is unchanged: `people_tracker` keeps
running and `dynamic_points_filter` takes its tracks out of the scans; it only sends no obstacles. On synthetic
detections Autoware's tracker follows the people less and starts more tracks, with fewer false ones; its confidence
thresholds are constants of its code (`~/tracker_trial/README.md`).
