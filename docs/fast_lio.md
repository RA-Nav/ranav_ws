# FAST-LIO: changes from the fork

This page covers the `fast_lio` package in `FAST_LIO_SLAM_ROS2/FAST_LIO_ROS2`, a ROS 2 port of FAST-LIO2 bundled with
SC-PGO. It lists what has been added on top of the fork and why. SC-PGO (`SC_PGO_ROS2`) is not covered here.

- **Fork base:** commit `4835e4e` ("automatic lidar frame id", 2025-09-12).
- **Full diff:** `git -C src/FAST_LIO_SLAM_ROS2 diff 4835e4e -- FAST_LIO_ROS2`
- **Configuration used in this project:** `ranav_bringup/config/fast_lio_airy.yaml` (simulated RoboSense Airy) and
  `fast_lio_airy_real.yaml` (real Airy), started by `ranav_bringup/launch/fast_lio_slam_launch.xml`, see
  [ranav_bringup.md](ranav_bringup.md). The fork's own launch files (`mapping.launch.py`) are unchanged.

## Design rule

The project integrates existing packages, so each package is changed as little as possible. Every addition to
`fast_lio` is a parameter, and its default reproduces the fork's behaviour. Two changes are exceptions: the default
point handler now drops invalid points and honours `point_filter_num`, see [Default point handler](#default-point-handler-lidar_type-not-14).

## Summary

| Change | Parameter(s) | Why |
|---|---|---|
| [IMU rotation](#imu-rotation) | `mapping.imu_R` | Keeps `camera_init` level when the IMU is tilted or upside down |
| [Reliable lidar subscription](#reliable-lidar-subscription) | `common.lid_reliable` | Large clouds (Airy: over 2 MB) are lost with best effort |
| [Default point handler](#default-point-handler-lidar_type-not-14) | `point_filter_num` | Removes `inf` points; downsamples 86 400 points per scan to 21 600 |
| [Fixed body frame name](#fixed-body-frame-name) | `common.body_frame` | The lidar's own frame is already in the URDF |
| [Robot frames](#robot-frames) | `frames.*` | Publishes the base-related frames and the base odometry from inside the node (formerly 2 or 3 static TF nodes and `lio_base_odometry.py`) |
| [Odometry alignment](#odometry-alignment) | `alignment.*` | Joins the FAST-LIO TF tree to the legged estimator's (formerly `lio_odom_alignment.py`) |
| [Log file bug fix](#log-file-bug-fix) | none | Crash on shutdown when `Log/` does not exist |

## Sensor input

### IMU rotation

`mapping.imu_R` (row-major 3x3, default identity). It rotates every IMU sample (angular velocity and linear
acceleration) before the sample enters the filter. The `orientation` field is not used by FAST-LIO and is not
rotated. Code: `imu_cbk` in `src/laserMapping.cpp`.

**Why it is needed.** FAST-LIO takes as its world frame (`camera_init`) the orientation of the IMU at the first scan.
A tilted IMU therefore gives a tilted map, and an upside-down IMU an upside-down map. Several nodes downstream (TARE,
terrain_analysis, FAR) assume that z points up in `/cloud_registered`. Rotating the IMU into a frame with the
orientation of the robot base keeps `camera_init`, `/Odometry` and `/cloud_registered_body` level whenever the robot
starts level. FAST-LIO does not align its world frame to gravity, so start it with the robot standing still and level.

After the rotation, FAST-LIO's IMU frame is this rotated body frame. `mapping.extrinsic_R` and `mapping.extrinsic_T`
must therefore give the lidar pose in the **rotated** frame:

| Configuration | IMU vs. lidar | `imu_R` | `extrinsic_R` |
|---|---|---|---|
| Simulated Airy, pitched 5 deg forward (`rpy="0 5deg 0"` in `robot.xacro`) | IMU frame = lidar frame | `Ry(5 deg)` | `Ry(5 deg)` |
| Real Airy, level | IMU turned ~180 deg about (1, -1, 0)/sqrt(2); at rest it reads z = -1 g | `R(q)`, q from the Airy DIFOP packet | identity |
| Real Airy, pitched by p | as above | `Ry(p) * R(q)` | `Ry(p)` |

Here `Ry(p) = [cos p, 0, sin p; 0, 1, 0; -sin p, 0, cos p]`. Keep both matrices in sync with the lidar mount in
`robot.xacro`.

### Reliable lidar subscription

`common.lid_reliable` (default `false`). When `true`, the `PointCloud2` subscription uses a reliable QoS with depth
10 instead of `SensorDataQoS` (best effort, depth 5). The Airy publishes clouds of 900 x 96 points, over 2 MB each.
With best effort, about a third of them are lost to dropped fragments. Both sources of `/rslidar_points` publish
reliably: `AiryPointsRelay` (legged_gazebo, simulation) and `rslidar_sdk` (real lidar). This parameter has no effect on the Livox `CustomMsg` subscription (`lidar_type: 1`).

### Default point handler (`lidar_type` not 1–4)

The handler for plain `x y z intensity` clouds (`Preprocess::default_handler` in `src/preprocess.cpp`) has two
changes. Both are always on.

- **Non-finite points are skipped.** Gazebo's `gpu_lidar` marks rays with no return as `+-inf`, and those points
  passed the `blind` distance check. (`NaN` points were already rejected, because any comparison with `NaN` is
  false.)
- **`point_filter_num` is honoured.** The handler keeps every n-th point, as the other handlers already did; before,
  the parameter was ignored. On an organized cloud sent row by row whose width is a multiple of n (Airy: 900 columns,
  `ros_send_by_rows: true` in `rslidar_sdk`), this keeps every n-th column of every ring. The Airy configs use `4`:
  21 600 of the 86 400 points of a scan go to FAST-LIO and to everything built from `/cloud_registered`.

> **Note.** The node's own default for `point_filter_num` is `2`. A config that selects the default handler without
> setting `point_filter_num` now keeps half the points.

**Limitation.** This handler does no per-point motion compensation (`curvature = 0`). That is right for the
instantaneous Gazebo scans. Real Airy scans last 100 ms and carry a timestamp for each point, so they smear while the
robot turns or walks. A dedicated handler that reads the timestamp field would fix this.

## Frames and TF

### TF tree

The tree with the Airy configs:

```
odom ─► base ─► ... (URDF, robot_state_publisher)                    legged estimator (OptiPessi)
  │
  └─► lio_map ─► camera_init ─► rslidar_lio ─► lio_base               FAST-LIO
       dynamic     static    │    dynamic        static
      alignment.*  frames.*  │   (odometry)      frames.*
                             ├─► map       static, identity (frames.init_aliases)
                             └─► aft_pgo   SC-PGO, loop-closed rslidar_lio
```

| Frame | Meaning |
|---|---|
| `camera_init` | FAST-LIO world frame: the body (IMU) frame at the first scan |
| `rslidar_lio` | FAST-LIO body frame (lidar/IMU origin, base orientation through `imu_R`) |
| `lio_map` | Robot base at the first scan. It is the "world" of `opti_pessi_scenario.py` |
| `lio_base` | Robot base as estimated by FAST-LIO |
| `map` | Same as `camera_init`. terrain_analysis and the autonomy stack call it `map` |

### Fixed body frame name

`common.body_frame` (default `""`). It sets the child frame of `/Odometry`, of the `camera_init` TF and of
`/cloud_registered_body`. With `""`, the fork's behaviour applies: the `frame_id` of the first lidar message is used.

Why it is needed: the Airy's frame `rslidar` is already in the URDF tree (`base -> rslidar`). If FAST-LIO also
published `camera_init -> rslidar`, that frame would have two parents. The Airy configs use `rslidar_lio`.

### Robot frames

These parameters let the node publish the robot base frames itself, as static TFs. Each one is published only when
it is set.

| Parameter | Default | Published TF |
|---|---|---|
| `frames.map_frame` | `""` | `map_frame -> camera_init`, translation `body_in_base` |
| `frames.base_frame` | `""` | `body_frame -> base_frame`, translation `-body_in_base`. Requires `common.body_frame` |
| `frames.body_in_base` | `[0, 0, 0]` | Origin of the FAST-LIO body (IMU) in the base frame |
| `frames.init_aliases` | `[]` | `camera_init -> <alias>`, identity, once per name |
| `frames.base_odom_topic` | `""` | Not a TF: `/Odometry` moved from the body to `base_frame` (same frame, stamp, orientation and covariance), on this topic. Requires `frames.base_frame` |

`body_in_base` is a translation only. The body must already have the base orientation, which `mapping.imu_R`
provides. The Airy configs use `[0.2835, 0.0, 0.066]`, i.e. `lidar_mount_x, 0, lidar_mount_z` of `robot.xacro` for
the Aliengo. Update it if the mount changes.

With the Airy in the slam box on the back of the trunk (`robot.xacro` `slam_box:=true`), `fast_lio_slam_launch.xml
slam_box:=true` lays `fast_lio_airy_slam_box.yaml` (sim) or `fast_lio_airy_real_slam_box.yaml` (real) over the
config: `body_in_base` `[-0.24675, 0.0, 0.1576]`, `extrinsic_R` = Ry(5°), and `imu_R` = Ry(5°) (sim) or
Ry(5°)·R(q) (real). The terrain nodes take the new lidar height from `terrain_airy_slam_box.yaml`, with the same
argument on `explore_warehouse_launch.xml`, `inspection_launch.xml` and `far_planner_launch.xml`.

These parameters replace the `static_transform_publisher` nodes `lio_map_to_camera_init` and `rslidar_lio_to_lio_base`
(formerly in `fast_lio_slam_launch.xml`) and `camera_init_to_map` (formerly in the TARE and FAR launch files). The lidar offset is now
written once, instead of twice with opposite signs.

`frames.base_odom_topic` replaces `lio_base_odometry.py` (formerly in tare_planner). The Airy configs publish
`/lio_base_odometry`, which TARE (`sub_state_estimation_topic_`) and `tare_finish_merge_map.py` use. The controller
and the goal bridge place the robot by its base. When TARE placed it by the lidar, 0.29 m ahead, a waypoint under
the base was reached for the controller but not for TARE, and the robot stood still for good. `/Odometry` stays the
lidar pose: the terrain maps and far_planner keep it, because terrainAnalysis casts the scan rays from it.

### Odometry alignment

| Parameter | Default | |
|---|---|---|
| `alignment.odom_topic` | `""` | Odometry of the other estimator (`nav_msgs/Odometry`, `odom -> base`). `""` disables the alignment |
| `alignment.odom_frame` | `"odom"` | Parent frame of the published TF |
| `alignment.max_stamp_gap` | `0.05` | Maximum gap in seconds between `/Odometry` and the nearest odometry sample |

Requires `frames.map_frame` and `frames.base_frame`. The Airy configs set `odom_topic: "/odom"`, the output of the
legged Kalman filter.

**Why it is needed.** The robot runs two estimators:

- the legged Kalman filter (`/odom`, 200 Hz), which OptiPessi controls on;
- FAST-LIO (`/Odometry`, 10 Hz), which TARE and FAR plan in.

OptiPessi takes goals and obstacles in `odom` only and does no TF lookup. Anything coming from the planners or the
people tracker must therefore be transformed into `odom`. The two estimators drift apart: the Kalman filter
integrates the leg kinematics, slips included, and FAST-LIO does not. A static `odom -> lio_map` would put every
relayed goal off by that drift, about 13 % of the distance walked in simulation.

**What it does.** On every `/Odometry`, the node sets `odom -> lio_map` so that `lio_base` lands on the estimator's
`base` at that stamp:

```
T_odom_lio_map = T_odom_base(t) * T_lio_map_lio_base(t)^-1
T_lio_map_lio_base(t) = T(body_in_base) * T_camera_init_body(t) * T(-body_in_base)
```

Only x, y, z and yaw are kept: both trees are gravity-aligned, and the roll and pitch differences would only add
noise. The robot's position relative to a goal is then the same in both trees, up to the drift accumulated between
two scans. This works like `map -> odom` in REP 105, but in the opposite direction, so `odom` stays the root of the
tree.

**Details.**

- `T_odom_base(t)` is the `/odom` sample nearest in stamp. The node keeps the last 2 s of samples. If time goes back
  (simulation reset, bag loop), the history is cleared.
- The node uses `/odom` matched by stamp, not the `odom -> base` TF. It also uses the pose directly from the filter
  state instead of looking up the static TFs.
- If no `/odom` has arrived yet, one warning is logged and nothing is published. If the nearest sample is more than
  `max_stamp_gap` away, the TF is skipped and a warning is logged every 5 s. This usually means the two estimators
  are on different clocks (`use_sim_time`).
- The first published TF is logged once, so the initial offset between the two estimators can be read there.
- It needs no extra node and no extra thread. The `/odom` callback runs in the node's single-threaded executor and
  only appends to the history.

**What uses it:** ranav_mission `opti_pessi_goal_bridge`, lidar_perception `people_tracker` (the people on
`/opti_pessi/obstacles`), `opti_pessi_scenario.py --person`, `map_point_picker.py`.

**Why not feed `/Odometry` into the Kalman filter instead.** `legged_estimation` already has an input for an external
odometry (`/tracking_camera/odom/sample`), but it does not fit here:

- it overwrites only the position, so the yaw drift of the robot IMU remains;
- it overwrites the current state with a pose that is 50–100 ms old, which gives a sawtooth `/odom` while walking;
- it republishes `/odom` with FAST-LIO's header;
- its TF lookups assume that the sensor frame hangs off `base` in the URDF.

Making it work would mean changing `legged_estimation` substantially, against the design rule.

### Log file bug fix

`fclose(fp)` in the node destructor is now guarded. `fp` is `NULL` when `Log/` does not exist in the package source
folder, and the node crashed on shutdown.

## Known issues

- **No motion compensation for the real Airy**, see [Default point handler](#default-point-handler-lidar_type-not-14).
- **`/Odometry` covariance is one scan old.** `publish_odometry` publishes the message before filling the covariance.
  This comes from the original code and is not fixed here.
- **The lidar mount is written in three places:** `body_in_base`, `imu_R`/`extrinsic_R`, and `robot.xacro`. Keep them
  in sync, for both mounts (front: `fast_lio_airy*.yaml`; slam box: `fast_lio_airy*_slam_box.yaml`).

## Migration from `lio_odom_alignment.py` and `lio_base_odometry.py`

The two scripts and the static TF nodes listed above have been removed. The launch files in this workspace have
already been updated. In other launch files:

- remove the `lio_odom_alignment.py` and `lio_base_odometry.py` nodes and any `static_transform_publisher` for
  `lio_map -> camera_init`, `rslidar_lio -> lio_base` or `camera_init -> map`;
- make sure the FAST-LIO config contains the `frames` and `alignment` blocks, as in
  `ranav_bringup/config/fast_lio_airy.yaml`;
- there is no need to start anything by hand any more: `odom -> lio_map` appears as soon as FAST-LIO and the
  controller's `/odom` are both running.
