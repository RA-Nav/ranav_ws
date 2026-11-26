# ranav_mission

`src/ranav_mission`, package `ranav_mission`. It holds the project's own mission nodes: the ones that connect the
planners (TARE, FAR) to the controller (OptiPessiController) and run a mission from start to end. They used to live
inside the tare_planner and far_planner forks. [ranav_bringup](ranav_bringup.md) starts them.

| Node | Started by | What it does | Was in |
|---|---|---|---|
| `opti_pessi_goal_bridge` (C++) | both mission launch files | Picks the OptiPessi goal from TARE's `/way_point` and paths or far_planner's route, keeping clearance from `/terrain_map`; with `relay_way_point` (`waypoint:=true`), relays `/way_point` as it is | tare_planner (as `tare_path_to_opti_pessi_goal.py` and `way_point_to_opti_pessi_goal.py`) |
| `opti_pessi_obstacle_bridge` (C++) | `fast_lio_slam_launch.xml` with `tracker:=autoware` | Sends the pedestrians `autoware_multi_object_tracker` tracks to OptiPessi (`/opti_pessi/obstacles`), in place of `people_tracker` | lidar_perception (a trial branch, 2026-11-15) |
| `tare_finish_merge_map.py` | `explore_warehouse_launch.xml` (`merge_map`) | Ends an exploration unattended: return home, stop, merge the map, save and prune the FAR graph | tare_planner |
| `far_inspection_goals.py` | `far_planner_launch.xml` (`inspection`) | Sends the picked inspection points to far_planner, one after the other | far_planner |

The nodes are installed to `lib/ranav_mission` (`ros2 run ranav_mission <node>`). Each one documents its topics and
parameters at its top: the docstring of each script, the header comment of `src/opti_pessi_goal_bridge.cpp`.

The goal bridge is C++ since 2026-11-14: as a Python script it took about 40 % of a core, turning `/odom` (84 Hz),
all of `/tf` (about 70 Hz) and the terrain map into NumPy. The choice of the goal has no ROS dependency
(`include/ranav_mission/opti_pessi_goal.hpp`, `src/opti_pessi_goal.cpp`); the node adds the inputs, TF, parameters
and the marker. It is one node for TARE and far_planner (`route_topic`) and for the plain relay
(`relay_way_point`), and it stays outside the forks, which publish their plans as upstream does.

## Why these nodes exist

**OptiPessi takes its goal in `odom` only and does no TF lookup.** The planners work in `map` (= FAST-LIO's
`camera_init`). The goal bridges therefore bring every point to `odom` through the TF `odom -> lio_map ->
camera_init -> map`, which FAST-LIO publishes (see [fast_lio.md](fast_lio.md)).

**OptiPessi has no local planner.** It walks straight to its goal, so `opti_pessi_goal_bridge` picks the goal
itself. It takes the first of these that keeps clearance from the obstacles:

1. the midpoint between the robot and the waypoint, or the waypoint itself within `waypoint_reach_distance` (0.5 m).
   Midpoints alone stop short of a near waypoint: OptiPessi stands at goals nearer than 0.1 m, and a midpoint that
   moved less than `republish_distance` is not sent. TARE keeps its waypoint on a viewpoint until the robot is there,
   so the robot once stood 0.3 m from it for 20 minutes;
2. a point ahead along the planner's path (TARE's paths, or far_planner's route `/viz_path_topic` with `route_topic`);
3. a detour;
4. a stop at the robot.

With far_planner, the route is followed instead of `/way_point`. far_planner makes `/way_point` for a local planner
that steers in a direction, and it is pushed off the route.

**An exploration must end on its own.** `tare_finish_merge_map.py` waits for `/exploration_finish`. With
`return_home`, it lets TARE walk the robot back to where `/lio_base_odometry` had it at the start. Then it:

1. stops TARE and the goal bridge, and sends a stop goal at the robot;
2. after `flush_time`, runs map_generator's `make_merged_map.py` on SC-PGO's `save_directory`;
3. with `graph_name` set, saves far_planner's graph through graph_decoder (`/save_file_dir`);
4. with `graph_name` and `prune_graph`, runs `prune_far_graph.py`;
5. with `exits`, runs exit_inspection's `finalize_exits.py`.

`explore_warehouse_launch.xml` sets `graph_name` to `warehouse.vgh` only with `far:=true`, when far_planner runs in
the exploration to build its graph. Without it `graph_name` is empty, so steps 3 and 4 are skipped. When the node exits, the whole launch shuts down (`on_exit="shutdown"`).

The scripts of map_generator need open3d. They run with `.venv/bin/python` of the workspace (`setup_ranav.sh venv`);
the parameter `python` overrides it. See [map_generator.md](map_generator.md).

**An inspection follows the picked points.** At the start, `far_inspection_goals.py` loads far_planner's saved
graph (`graph_file`, `save_data/warehouse.vgh`) into far_planner through graph_decoder (`/read_file_dir`). After
each inspection it saves far_planner's graph there again (`/save_file_dir`), so the graph grows with every
inspection; with no `graph_file` yet, far_planner starts with its own. The file is kept in the map frame: it is
converted with the localization when loaded and when saved, turned about z only (far_planner tells pillars by the
surface directions (0, 0, -1), which a tilt would change). Each load and save is one line in the terminal. The node
then waits for a file on `/start_inspection`: the Start inspection button of `map_point_picker.py` sends one, and an
empty message means `goals_file`. It then:

1. publishes each point on `/goal_point`. The points are in the map of the exploration (`global_map`); far_planner
   plans in the frame of the FAST-LIO running now. So each point is brought into it with the latest match of the
   localization (`/map_to_odom`), and the inspection starts only once there is one. The robot then need not start
   from the pose of the exploration. An empty `localization_topic` sends the points as they are;
2. judges it reached (`/far_reach_goal_status`, or within `reach_tolerance`) or failed (no `/way_point` for
   `idle_timeout`, or `goal_timeout`);
3. waits `dwell_time` and goes on to the next point.

**Autoware's tracker can give OptiPessi its obstacles.** `opti_pessi_obstacle_bridge` reads the confirmed tracks of
`autoware_multi_object_tracker` (`/perception/autoware/tracks`, frame `camera_init`) and publishes, for each message,
the pedestrians within `max_range` (10 m) of the sensor as `HUMAN` on `/opti_pessi/obstacles`, in `odom`, as
`people_tracker` does with `tracker:=people_tracker`. The other classes of the road-trained detector are left out:
the controller has no type for them, and in a warehouse they are shelves. Without the TF `odom <- camera_init`
(FAST-LIO, once the legged estimator's `/odom` runs) or `camera_init <- sensor_level` (lidar_perception) at the
stamp of the tracks it publishes nothing, not an empty array. The conversion has no ROS graph
(`include/ranav_mission/opti_pessi_obstacles.hpp`); `legged_controllers` gives it its messages only (`CMakeLists.txt`
imports their targets without `find_package(legged_controllers)`, which would load the controller stack).

## Tests

```bash
python3 -m pytest src/ranav_mission/test      # the scripts, ROS 2 sourced
colcon test --packages-select ranav_mission   # the two bridges (gtest): build/ranav_mission/test_opti_pessi_goal,
                                              # build/ranav_mission/test_opti_pessi_obstacles
```

The Python tests load the scripts from `scripts/` by path and replace the ROS interfaces they need. The gtests run
the choice of the goal and the conversion of the tracks without a ROS graph. None of them needs a running ROS graph.
