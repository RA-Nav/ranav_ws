# FAST-LIO localization: changes from the fork

This page covers the `fast_lio_localization` package in `FAST_LIO_LOCALIZATION`: global localization in a map built
earlier, on top of a FAST-LIO that is already running. It lists what the ROS 2 port contains and why.

- **Fork base:** commit `2bc274e` ("update readme", 2021-08-13), the ROS 1 FAST_LIO_LOCALIZATION, branch `main`.
- **Full diff:** `git -C src/FAST_LIO_LOCALIZATION diff 2bc274e`
- **Started by** `localization_launch.xml` of [ranav_bringup](ranav_bringup.md), which includes the package's own
  `launch/localization_launch.xml` with the values of the Aliengo.

## What is in the package and what is not

The ROS 1 repository is a copy of FAST-LIO plus three localization scripts. The ROS 2 port keeps only the
localization: it runs on the FAST-LIO of `fast_lio_slam_launch.xml` ([fast_lio.md](fast_lio.md)).

| Kept | |
|---|---|
| `scripts/global_localization.py` | Scan-to-map ICP (Open3D) at 0.5 Hz: `map -> camera_init` |
| `scripts/transform_fusion.py` | Publishes the TF and `/localization` at 50 Hz |
| `scripts/publish_initial_pose.py` | Sends `/initialpose` by hand |
| `scripts/se3_utils.py` | SE(3) helpers; they replace ROS 1's `tf.transformations` |
| `launch/localization_launch.xml` | The three nodes, the map publisher (`pcl_ros pcd_to_pointcloud`) and RViz |
| `rviz_cfg/localization.rviz` | Fixed frame `global_map` |

The port had a second FAST-LIO (`fastlio_mapping`: about 11000 lines with IKFoM, matplotlibcpp and the configs of
six LiDARs, plus the ikd-Tree submodule). It was removed on 2026-11-14: RaNAV never ran it (`fast_lio:=false`). Three of its files were identical to
fast_lio's, and its `laserMapping.cpp` lacked the frames of the RaNAV TF tree (`lio_base`, `odom -> lio_map`). The
package now has no C++, no messages and no dependency on livox_ros_driver2.

## Changes from the ROS 1 scripts

| Change | Kind | Why |
|---|---|---|
| rclpy, `sensor_msgs_py`, `se3_utils.py` | Port | ROS 2 |
| [`camera_init -> global_map`](#tf) | Change | A TF frame has one parent, and `camera_init` already has `lio_map` |
| [`init_from_odom`](#initial-pose) | New parameter, default `false` | No initial pose to give by hand |
| [No `/cur_scan_in_map`](#scan) | Removed | It republished every scan unchanged |
| ICP in its own callback group | Port | The scan and odometry callbacks keep coming in while ICP runs |
| [Quiet terminal](#terminal) | Change | Upstream printed 2 lines per ICP run (every 2 s), 4 more when it missed |

### TF

ROS 1 published `map -> camera_init`. In RaNAV, `camera_init` is already a child (`odom -> lio_map -> camera_init`,
published by FAST-LIO), so `transform_fusion.py` publishes the inverse, `camera_init -> global_map`. The map frame is
the `map_frame` argument (`global_map`); `odom_frame` is FAST-LIO's world frame (`camera_init`).

### Initial pose

ROS 1 waits for `/initialpose`. With `init_from_odom: true`, `global_localization.py` guesses it from a second
odometry instead (`legged_odom_topic`, `/odom` of the legged estimator):

```
T_map_to_odom = T_base_lidar^-1 * planar(T_odom_base) * T_base_lidar * T_camera_init_lidar^-1
```

`T_odom_base` is `/odom` at the stamp of the latest `/Odometry`, kept to x, y and yaw. `T_base_lidar` is the TF
`lio_base -> rslidar_lio` from FAST-LIO. `T_camera_init_lidar` is the FAST-LIO pose. The guess holds when the map is
the `camera_init` of an exploration that started with the robot standing at the same spawn, since `/odom` starts there
too. It is retried at every localization tick until ICP passes `localization_th`; `/initialpose` still overrides it.

### Scan

The ROS 1 script republished every `/cloud_registered` as `/cur_scan_in_map`, in the same frame and only restamped,
and converted every scan to a NumPy array. With the Airy, that is about 2 MB at 10 Hz through Python, while ICP runs
at 0.5 Hz. The scan is now kept as it came and converted only when ICP runs. RViz shows `/cloud_registered` itself.

### Terminal

ROS 1 printed two lines for every ICP run ("Global localization by scan-to-map matching", "Time"), and three more
(the matrix, the fitness) when it did not match: 2-5 lines every 2 s for as long as it runs. Now only changes are
printed: one warning the first time the map is missed, or when it is lost after a match (with the fitness and
`localization_th`); one line when it is matched again; the initial guess from `/odom` once; the initialization once.

## Values used in RaNAV

Set by ranav_bringup's `localization_launch.xml`. The other parameters keep the defaults of the scripts.

| Parameter | Value | Default | |
|---|---|---|---|
| `map` | `save_data/optimized_map.pcd` of map_generator | (required) | The merged map of the exploration |
| `fov` | `6.28` | `1.6` | The Airy is a 360 deg LiDAR: the map is cropped by distance only |
| `fov_far` | `60.0` | `150.0` | Airy range, in m |
| `init_from_odom` | `true` | `false` | |
| `python` | `.venv` of the workspace | `python3` | open3d |

## Known issues

- **Ctrl+C prints a traceback.** A second `KeyboardInterrupt` reaches the nodes during `destroy_node()`. It happens
  only at shutdown.
- **Initial guess away from the spawn.** `init_from_odom` assumes the robot starts at the spawn of the exploration. Started elsewhere,
  the guess is off by that distance: send `/initialpose`.
