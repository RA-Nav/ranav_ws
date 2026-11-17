# Terrain analysis: changes from the fork

This page covers the `terrain_analysis` and `terrain_analysis_ext` packages in `autonomy_stack_go2/src/base_autonomy`.
`terrainAnalysis` turns the registered scans into `/terrain_map`: the points near the robot, with their height above
the ground in the intensity. `terrainAnalysisExt` does the same over a larger area, `/terrain_map_ext`, taking
`/terrain_map` within `localTerrainMapRadius` (4 m). TARE, far_planner and the goal bridges read their obstacles from
these maps. The page lists what has been changed on top of the fork and why.

- **Fork base:** commit `43d5f54` ("update download link", 2026-02-18), branch `foxy-humble`.
- **Full diff:** `git -C src/autonomy_stack_go2 diff 43d5f54 -- src/base_autonomy`
- **Started by** `explore_warehouse_launch.xml` and `far_planner_launch.xml` (package `ranav_bringup`, see
  [ranav_bringup.md](ranav_bringup.md)), with the parameters of `config/terrain_airy.yaml`. With `people_free`
  (`far_planner_launch.xml`, also in the exploration with `far:=true` and `people:=true`) a second pair runs in namespace
  `far_static`, on the scans without the people. The fork's `.launch`
  files are not used.

## What is in the fork and what is not

Only C++ has changed: `terrainAnalysis.cpp` and `terrainAnalysisExt.cpp`. Every new behaviour is a parameter whose
default, `false`, keeps the fork's behaviour. The other defaults and the fork's `.launch` files are as in the fork.

## Summary

| Change | Kind | Why |
|---|---|---|
| `tf2_geometry_msgs.hpp` | Build | Jazzy has no `.h` header any more |
| [`carveDyObs`](#moving-obstacles) (terrainAnalysis) | New parameter, default `false` | People who walked away stayed in the map |
| [`groundPrior`](#floor-the-lidar-does-not-see) (both) | New parameter, default `false` | The bottom of a wall seen from above became ground, and the wall vanished |
| [`groundPriorFill`](#floor-the-lidar-does-not-see) (both) | New parameter, default `false` | Around the robot the Airy gives no ground points; far_planner crashed without them |

## Moving obstacles

The fork keeps every point of the map until it decays (`decayTime`) or is cleared by `clearDyObs`. `clearDyObs`
needs a model of the lidar's vertical field of view, which does not fit the pitched Airy dome. So a person who walked
by left a wall in `/terrain_map`, and through it in `/terrain_map_ext`.

With `carveDyObs: true`, a stored point is deleted once the current scan sees through it:

1. the current scan becomes a range image around the sensor (the `/state_estimation` position): the nearest return
   in each bin of `carveBinSize` deg of azimuth and elevation;
2. a stored point is deleted if the 3 x 3 bins around its direction hold at least one return, and every return
   there lies more than `carveMargin` beyond the point.

A point with a nearer return in its direction is occluded, and a point with no return is out of view: both are kept.
Taking the whole neighbourhood keeps the edges of static obstacles, which the sparse scan may miss in one bin but not
in all of them. Only points that are clearly obstacles are checked: they must lie between `carveMinDis` and
`carveMaxDis` from the sensor, and more than `carveMinHeight` above the ground of a cell that holds at least
`minBlockPointNum` points.

| Parameter | Default | |
|---|---|---|
| `carveDyObs` | `false` | `true` in both RaNAV launch files |
| `carveBinSize` | `2.0` | Bin size of the range image, in deg |
| `carveMargin` | `0.3` | How far beyond the point the returns must lie, in m |
| `carveMinDis` | `0.8` | Closer points are in the lidar's blind zone, in m |
| `carveMaxDis` | `8.0` | Farther points are not checked, in m |
| `carveMinHeight` | `0.1` | Ground points are never deleted, in m above the ground |

## Floor the lidar does not see

The Airy dome sits on top of the trunk. Around the robot it sees only what is higher than itself, and the floor only
from a few metres ahead. The fork takes the ground of each cell from its lowest points (`quantileZ`). Where the floor
was not seen, the bottom of a wall became "ground", and the wall vanished from the terrain maps.

- **`groundPrior`.** If a cell's ground estimate lies more than `groundPriorTolerance` above the floor the robot
  stands on, the cell did not show its floor, so its ground is that floor. The floor is at `vehicleZ - sensorHeight`,
  where `sensorHeight` is the height of `/state_estimation` above the floor. This is a flat-floor assumption.
- **`groundPriorFill`** (with `groundPrior`). Cells without points are on that floor too.
  - In `terrainAnalysis` each such cell gets a point at ground level (intensity 0). far_planner stands its graph on
    the ground points and crashed without any.
  - In `terrainAnalysisExt` the connectivity check, which drops what it cannot reach from under the robot, walks
    across them. Without that, obstacles behind more than 4 m of unseen floor were dropped. Beyond
    `localTerrainMapRadius`, which `/terrain_map` fills, such cells get a ground point too.

| Parameter | Default | RaNAV |
|---|---|---|
| `groundPrior` | `false` | `true` |
| `groundPriorFill` | `false` | `true` |
| `sensorHeight` | `0.47` | `0.47`: base height 0.40 + lidar 0.066; `0.55` with the Airy in the slam box (`slam_box:=true`, `terrain_airy_slam_box.yaml`) |
| `groundPriorTolerance` | `0.2` | `0.2` |

`terrain_airy.yaml` also sets the height window to match the dome: `vehicleHeight`, `minRelZ`/`maxRelZ` and
`lowerBoundZ`/`upperBoundZ`. The reasons are in the file's comments.

## Known issues

- **Flat floor.** With `groundPrior`, a ramp that rises more than `groundPriorTolerance` within the map reads as an
  obstacle. With `groundPriorFill`, unseen cells are assumed to be floor: a step down or a hole that the lidar cannot
  see reads as free floor.
- **Parameters the fork's `.launch` files set.** The RaNAV nodes do not use the fork's `.launch` files. A parameter
  that `terrain_airy.yaml` does not set keeps its C++ default, which in a few cases differs from the fork's launch
  value:

  | Node | Parameter | C++ default (used) | Fork `.launch` |
  |---|---|---|---|
  | terrainAnalysis | `noDecayDis` | `4.0` | `0.0` |
  | terrainAnalysis | `minDyObsRelZ` | `-0.5` | `-0.3` |
  | terrainAnalysis | `noDataObstacle` | `false` | `true` |
  | terrainAnalysisExt | `useSorting` | `false` | `true` |
  | terrainAnalysisExt | `quantileZ` | `0.25` | `0.1` |

- **Ctrl+C ends in `terminate`.** The fork's main loop checks `rclcpp::ok()` only after a full iteration, so after
  the signal it calls `spin_some()` on a context that is shut down, which throws `RCLError` ("failed to create guard
  condition"). It happens only at shutdown, and the node holds nothing to save.
- **Build warnings.** `-Wparentheses` in `terrainAnalysis.cpp` and the CMake policy warnings come from the fork.
