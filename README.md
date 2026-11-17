# ranav_ws

Workspace of the RaNAV stack: an Aliengo quadruped, in Gazebo or real, driven by the Opti-Pessi MPC controller, with
FAST-LIO + SC-PGO mapping, lidar people perception, TARE exploration and far_planner inspections. Our packages are in
`src/`, third-party code in `third_party/` (both git submodules), the documentation of the packages in `docs/`.

## Index

- [Requirements](#requirements)
- [Installation and build](#installation-and-build)
  - [Install everything](#install-everything)
  - [Step by step](#step-by-step)
  - [Environment of each terminal](#environment-of-each-terminal)
  - [Rebuild after edits](#rebuild-after-edits)
  - [Acados controller](#acados-controller)
- [Usage](#usage)
  - [A whole run with setup_ranav.sh](#a-whole-run-with-setup_ranavsh)
  - [Closed-loop scenarios (no Gazebo)](#1-scenarios-in-the-closed-loop-simulation)
    - [Run one scenario](#run-one-scenario)
    - [Run all six scenarios](#run-all-six-scenarios)
    - [Use the controller's 4-slot OCP](#11-the-controllers-ocp-4-obstacle-slots)
    - [Scenario 5: four obstacles](#12-scenario-5-four-obstacles)
    - [Scenario 6: random moving obstacles](#13-scenario-6-random-moving-obstacles)
  - [Scenarios in Gazebo](#2-scenarios-in-gazebo)
    - [Obstacle sent directly to the controller](#21-obstacle-given-to-the-controller)
    - [Obstacle detected as a Gazebo person](#22-obstacle-as-a-person-in-gazebo)
  - [Explore a warehouse](#3-exploration)
  - [Inspect points on a saved map](#4-inspections)
  - [Use the Acados controller](#acados-controller-simulation-and-planner-use)
  - [Utilities](#5-utils)
    - [Map point picker](#51-map-point-picker-web-interface)
    - [Publish goals and start/stop commands](#52-goals-and-commands-from-the-terminal)
    - [Common simulation, controller and perception launches](#53-common-launches)
    - [Merge an SC-PGO map](#54-merge-the-sc-pgo-keyframes-into-one-map)
    - [Run checks](#55-checks)
    - [Export a detector dataset](#56-dataset-for-the-detector-model)
    - [Recover from a silent Gazebo launch](#57-gazebo-silent-at-launch)
    - [Replay a ROS bag](#58-replay-a-ros-bag)

---

## Requirements

| | |
|---|---|
| Computer | x86_64 PC (simulation) or the NVIDIA Jetson Orin of the robot (aarch64, JetPack 7) |
| System | Ubuntu 24.04 (noble), a user with sudo (not root). ROS 2 Jazzy and Gazebo Harmonic come with the installation |
| GPU (optional) | NVIDIA, with a driver for CUDA 13 (>= 580), for the TensorRT people detector. The installation never installs a driver (in WSL it is the Windows one). Without it the same model runs on the CPU |
| Memory | About 2 GB per compile job: the build runs as many jobs as cores, at most RAM / 2 GB (`JOBS=<n>` to change it) |
| Disk | CUDA 13.0 + TensorRT alone: 7.5 GB to download, 12 GB installed |
| Network | GitHub, the ROS, robotpkg and NVIDIA apt repositories, PyPI, Hugging Face |
| Real robot | Ethernet to the robot on 192.168.123.0/24 (robot at 192.168.123.10); a wired port at 192.168.1.10 for the Airy lidar (192.168.1.200) |

The real robot and the Jetson build also need changes to `legged_control` and `ocs2` that are in none of their pushed
branches yet (`./setup_ranav.sh --help` lists them); the x86 simulation does not.

Besides the apt packages, the installation brings:

- pinocchio 3.9.0 in `/opt/openrobots` (robotpkg on x86_64; built from source on aarch64, 30-60 min on the Jetson)
- CUDA toolkit 13.0 and TensorRT 10.13.3, held at that version (on the Jetson: those of JetPack)
- `.venv`: python with open3d 0.20.0 and onnxruntime 1.30.0, for the map merge, the localization and the CPU people
  detector
- the pretrained CenterPoint model of Autoware (v4.1, short range) in `models/`

Two things break the controller and must stay away:

- `ros-jazzy-pinocchio` (4.x): built against its headers, the controller segfaults at load with the 3.9 libraries.
  The installation offers to remove it.
- An `LD_LIBRARY_PATH` with another `libhpipm.so` (of acados, for example) ahead of the workspace's: Gazebo crashes at
  the first MPC solve. Use a terminal without it.

---

## Installation and build

### Install everything

```bash
git clone --recurse-submodules https://github.com/AlexBarbi/ranav_ws.git ~/ranav_ws
cd ~/ranav_ws
./setup_ranav.sh install
```

- The folder must be called `ranav_ws`. Everything the installation downloads and builds stays in it, apart from the
  system packages (apt, `/etc`, `/opt/openrobots`).
- `install` runs the steps below in this order: `deps`, `cuda`, `sources`, `venv`, `model`, `build`, `env`. After a
  failure, run it again: each step skips what is already there.
- `--no-cuda`: no CUDA and no TensorRT, the people detector then runs on the CPU. `--camera`: also the camera people
  detector (`opti_pessi_perception`, YOLOs-CPP + ONNX Runtime). `--dry-run`: only print the commands.
- The first start with people detection takes about 2 minutes more: TensorRT builds its engines for the GPU at hand.

Then open a new terminal and [set up its environment](#environment-of-each-terminal).

### Step by step

| Command | What it does |
|---|---|
| `./setup_ranav.sh deps` | apt: ROS 2 Jazzy desktop, Gazebo (ros_gz), ros2_control, build tools; pinocchio 3.9.0; the rosdep keys of the sources |
| `./setup_ranav.sh cuda` | CUDA toolkit 13.0 + TensorRT 10.13.3 on x86_64, those of JetPack on the Jetson. Never a GPU driver |
| `./setup_ranav.sh sources` | Checks out the submodules not checked out yet (one that is stays as it is), writes the messages-only `third_party/livox_ros_driver2` that `fast_lio` needs, sets the Airy point type in `rslidar_sdk`; on the Jetson, the arm64 Unitree SDK and OR-Tools |
| `./setup_ranav.sh venv` | `.venv` with open3d and onnxruntime |
| `./setup_ranav.sh model` | The CenterPoint model in `models/lidar_centerpoint/v4.1/short_range` |
| `./setup_ranav.sh build` | Builds bundled Acados as static libraries, then `colcon build` (Release) of the packages the launches start and their dependencies, including `acados_legged_controller`; without CUDA, the people detector packages are left out |
| `./setup_ranav.sh env` | Writes `ranav_env.sh` and the Fast DDS profile `fastdds_shm_16mb.xml` |

`./setup_ranav.sh --help` lists every command and option. `GIT_PROTOCOL=ssh` reaches GitHub over SSH instead of HTTPS.

Building by hand with colcon:

- The first build of a package needs `--cmake-args -DCMAKE_BUILD_TYPE=Release` (`setup_ranav.sh build` passes it;
  later `colcon build` runs keep it). A single package built without it makes the MPC solve about 40 times slower.
- `opti_pessi_interface` and `legged_controllers` are installed as links (`--symlink-install`), every other package as
  copies. A package keeps the install mode of its first build: never pass `--symlink-install` to one built without it.

### Environment of each terminal

```bash
source ~/ranav_ws/ranav_env.sh
```

It sources ROS 2 Jazzy and the workspace, puts CUDA on the paths, sets the Fast DDS profile and `ROS_LOG_DIR`
(`log/ros`). `./setup_ranav.sh env --bashrc` prints the line to add to `~/.bashrc` instead.

The profile gives Fast DDS a 16 MB shared-memory segment instead of the default 512 KB, for the 1-2 MB point clouds
passed between processes. Reported measurements were 13 ms per hop for a 1 MB cloud by default, with spikes under
load, and 1-4 ms with the profile; they depend on the machine and Fast DDS version. Without `ranav_env.sh`, set it by
hand in each terminal, from the workspace root:

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/fastdds_shm_16mb.xml"
```

### Rebuild after edits

| After editing | Rebuild |
|---|---|
| The Airy xacro / bridge / relay | `colcon build --packages-select legged_gazebo legged_unitree_description` |
| `scenario_room.world`, `ActorPoseFollower.cpp` | `colcon build --packages-select legged_gazebo` |
| A launch, config or rviz of `ranav_bringup` (FAST-LIO, terrain, TARE, Airy driver parameters; installed as copies: an edit does nothing until the rebuild) | `colcon build --packages-select ranav_bringup` |
| A script of `ranav_mission` or `map_generator`, the people_perception config / launch (copies too) | `colcon build --packages-select <package>` |
| `opti_pessi_interface` C++ (the OCP, the obstacle slots, the closed loop) or the controller | `colcon build --packages-select opti_pessi_interface legged_controllers` |
| A new file in `opti_pessi_interface/config` (edits of the existing ones need nothing: they are installed as links) | `colcon build --packages-select opti_pessi_interface` |
| `acados_legged_controller` source or config | `colcon build --packages-select acados_legged_controller --cmake-args -DCMAKE_BUILD_TYPE=Release` |

`opti_pessi_scenario.py` (`opti_pessi_interface`) and `kill_node_sigint_service.py` (`legged_controllers`) are
installed as links: their edits need no rebuild.

`autoware_perception_msgs` is built from `third_party/autoware_msgs` (only that package of it is built).

Build `people_perception` without the optional CUDA/TensorRT packages (CPU detector):

```bash
cd ~/ranav_ws && colcon build --packages-select people_perception \
  --packages-ignore autoware_lidar_centerpoint autoware_tensorrt_common autoware_cuda_utils \
    autoware_cuda_dependency_meta cuda_blackboard negotiated negotiated_interfaces perception_utils \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Acados controller

`setup_ranav.sh build` builds the pinned `third_party/acados` source as static libraries in
`install/acados_static`, then builds `acados_legged_controller` with the rest of the stack. Static linking keeps
Acados' HPIPM and BLASFEO symbols isolated from the different versions used by OCS2.

For a manual rebuild, source the workspace environment first; it sets `ACADOS_SOURCE_DIR` and `ACADOS_STATIC_DIR`:

```bash
source ~/ranav_ws/ranav_env.sh
colcon build --packages-select acados_legged_controller --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Changes to the Acados source need `./setup_ranav.sh build` to update its static libraries. Changes to the model or OCP
definition also need `src/legged_control/acados_legged_controller/scripts/model_gen.py` run with the project's Acados
Python environment before rebuilding; goals and obstacle inputs do not need regeneration.

---

## Usage

Paths are relative to the workspace root (`~/ranav_ws`): run everything from there, in terminals with
[the environment](#environment-of-each-terminal) set. Each `ros2 launch` keeps its terminal: one terminal per launch,
in the order given.

### A whole run with setup_ranav.sh

`setup_ranav.sh start` opens the launches of a run in a tmux session, one window each, and waits for each step (Gazebo
and the controller manager up, controller active, robot standing, FAST-LIO running) before the next:

```bash
./setup_ranav.sh start exploration     # warehouse, controller, explore_warehouse_launch.xml (section 3)
./setup_ranav.sh start inspection      # the same, inspection_launch.xml on the map of an exploration (section 4)
./setup_ranav.sh start scenario 2      # scenario room, controller, opti_pessi_scenario.py 2 (section 2)
./setup_ranav.sh attach                # the windows: Ctrl-b n / Ctrl-b p next / previous, Ctrl-b d detach
./setup_ranav.sh stop                  # Ctrl-C to every window, last first, then closes the session
```

| Option of `start` | |
|---|---|
| `--world empty_room` | The 30 x 30 m room with people instead of the warehouse |
| `--gui` | Gazebo with its GUI (default: headless) |
| `--no-people` | No people detection |
| `--cpu-detector` | The people detector on the CPU even where the TensorRT one is built (automatic where it is not) |
| `--auto-start` | Explore or inspect at once, without `/start_exploration` or the **Start inspection** button |
| `--recompile`, `--no-recompile` | Generate the Opti-Pessi solver libraries again, or not (default: yes for exploration and inspection, no for scenarios) |
| `--person` | Scenario: the obstacle is the Gazebo person, found by the lidar pipeline ([section 2.2](#22-obstacle-as-a-person-in-gazebo)) |
| `--real` | The real robot instead of Gazebo; asks before the controller stands it up ([Requirements](#requirements)) |

`RANAV_SESSION` names the tmux session (default `ranav`), `STAND_WAIT` gives the robot that many seconds to stand
before FAST-LIO starts (default 10). The sections below start the same launches by hand, with all their options.

Gazebo scenarios 1-4 are: 1 static obstacle, 2 patrol, 3 obstacle circling the goal, 4 antagonist (heads for the
robot). Closed-loop scenarios 5 and 6 are [four obstacles](#12-scenario-5-four-obstacles) and
[random-moving obstacles](#13-scenario-6-random-moving-obstacles), respectively; they have no Gazebo scenario script.

---

### 1. Scenarios in the closed-loop simulation

No Gazebo and no ROS graph: `opti_pessi_interface_mpc` runs the Opti-Pessi MPC in closed loop with its own model and
the obstacle plant in `opti_pessi_interface/config/scenario_S<n>.info`. The controller's OCP with 4 obstacle slots:
[section 1.1](#11-the-controllers-ocp-4-obstacle-slots).

#### Run one scenario

```bash
ros2 run opti_pessi_interface opti_pessi_interface_mpc \
  src/legged_control/opti_pessi_interface/config/task.info \
  src/legged_control/opti_pessi_interface/config/scenario_S1.info

python3 src/legged_control/opti_pessi_interface/python/plot_mpc.py 1
```

- The node prints collision, minimum distance from the goal, solve times and failed solves, and writes
  `x_quad_`, `u_`, `y_obs_` and `t_solve_mpc_sim_<n>__opti_pessi.npy` (solve time of every step) in the working
  directory.
- `plot_mpc.py <n>` reads them there, writes a PNG and a trot-animation GIF under
  `src/legged_control/opti_pessi_interface/python/plots/`, and **deletes the `.npy` files**. It finds its files only
  when run from the workspace root. `<n>`: `1`-`6` or `S1`-`S6`, `live` (`scenario_live.info`), or the path of any scenario file; it must
  be the file the node ran on (goal, `figName`, the obstacle slots in the title).

#### Run all six scenarios

The loop generates libraries for one slot on S1 and four slots on S5, then reuses them for scenarios with the same
slot count.

```bash
for s in 1 2 3 4 5 6; do
  run_flags=(--quiet)
  if [ "$s" -ne 1 ] && [ "$s" -ne 5 ]; then
    run_flags+=(--no-recompile)
  fi
  ros2 run opti_pessi_interface opti_pessi_interface_mpc \
    src/legged_control/opti_pessi_interface/config/task.info \
    src/legged_control/opti_pessi_interface/config/scenario_S$s.info "${run_flags[@]}"
  python3 src/legged_control/opti_pessi_interface/python/plot_mpc.py $s
done
```

| Argument | Default | |
|---|---|---|
| third argument: a folder | `/tmp/ocs2/opti_pessi_interface` | Where the solver libraries are generated (not the folder of the controller) |
| `--no-recompile` | | Reuse the libraries already in that folder |
| `--quiet` | | No output at every step |

#### 1.1 The controller's OCP: 4 obstacle slots

`scenario_live.info` is the scenario file of the controller (section 5.3): `obstacles.numSlots 4`, the OCP keeps out
up to 4 obstacles at once, and S1's static obstacle as the plant. Run here, it tries the controller's OCP offline:

```bash
ros2 run opti_pessi_interface opti_pessi_interface_mpc \
  src/legged_control/opti_pessi_interface/config/task.info \
  src/legged_control/opti_pessi_interface/config/scenario_live.info --quiet

python3 src/legged_control/opti_pessi_interface/python/plot_mpc.py live
```

- `obstacles.numSlots` (any scenario file, default `obstacles.numObstacles`): obstacle slots of the OCP, apart from the
  scripted obstacles of the plant. The node prints it at the start (`obstacleSlots=`). A free slot constrains nothing.
- The plant's obstacles reach the OCP the way perception's reach the controller
  (`OptiPessiReferenceManager::setObservedObstacles`): the closest keep-out disks take the slots, and an obstacle keeps
  its slot from one step to the next (within 1 m).
- Collisions are checked at every step against every obstacle (until 2026-10-15 only on failed solves, and only against
  obstacle 0).
- `plot_mpc.py live` writes `python/plots/mpc_sim_live_opti_pessi.png`, the slot count in its title.
- To try more slots on a scenario, add `numSlots` to its `obstacles` block. `numObstacles` is the number of scripted
  obstacles: raising it needs that many rows in `positions`, `movement`, `speed` and `direction`, or the node stops
  with `plant.movement has 1 entries but numObstacles is 4`.
- **With `--no-recompile`, one library folder per slot count**: S1 has 1 slot, `scenario_live.info` 4; libraries
  generated for the other count have the wrong size. Without `--no-recompile` they are generated again every run.

More solver iterations without generating the libraries again (`ipmIteration` is read at run time, not compiled in):

```bash
sed 's/^\(  ipmIteration *\)50/\1300/' src/legged_control/opti_pessi_interface/config/task.info > /tmp/task_it300.info
ros2 run opti_pessi_interface opti_pessi_interface_mpc /tmp/task_it300.info \
  src/legged_control/opti_pessi_interface/config/scenario_S1.info /tmp/ocs2/opti_pessi_s1 --quiet
```

Measured 2026-10-15 (`task.info` with N 6, simTime 20 s): distance left to the goal at the end, no collision in any run.
The closed loop solves once a step, the controller in Gazebo about five times a phase: 50 iterations walk fine there.

| `ipmIteration` | S1 | S2 | S3 | S4 | `scenario_live.info` |
|---|---|---|---|---|---|
| 50 (`task.info`) | 1.14 m | 2.00 m | 1.25 m | 5.13 m | 1.18 m |
| 300 | 0.08 m | 0.38 m | 0.01 m | 4.69 m | 0.27 m |

Solve time at 50 iterations follows the obstacles present, not `numSlots`: each step solves the OCP with one slot per
obstacle (9 ms with one, 15 ms with two, 31-33 ms with four). `scenario_live.info` therefore runs exactly as S1. S4's
obstacle chases the robot: the goal is never reached.

#### 1.2 Scenario 5: four obstacles

`scenario_S5.info`: four obstacles in an OCP with four slots, all of them used. From the start (-0.8, -0.5) to the
goal (3.5, 0): one obstacle static at (0.8, 1.2), left of the way; one patrolling at 0.4 m/s from (2, 1.5), across the
way (y 1.5 to -0.5 and back); one static at (2.6, -1.6), right of the way; one orbiting the goal at 0.4 m/s, 2.5 m
out, through the way. The current `simTime` is 30 s.

```bash
ros2 run opti_pessi_interface opti_pessi_interface_mpc \
  src/legged_control/opti_pessi_interface/config/task.info \
  src/legged_control/opti_pessi_interface/config/scenario_S5.info --quiet

python3 src/legged_control/opti_pessi_interface/python/plot_mpc.py 5
```

Measured 2026-10-15 with `task.info` `wc 1.0` and `simTime 60.0` (the weights are compiled into the libraries: after
changing one, run without `--no-recompile`), no collision in any run. The current `scenario_S5.info` uses
`simTime 30.0`, so use 60.0 to reproduce the timing column:

| `ipmIteration` | Distance left to the goal | Time | Solve time |
|---|---|---|---|
| 50 (`task.info`) | 3.12 m: stuck between the patrol, the right obstacle and the orbit | 60 s | 33 ms |
| 100 | 0.008 m | 36.4 s | 65 ms |
| 300 | 0.006 m | 12.9 s (closest CoM to an obstacle 0.60 m, the patrol) | 187 ms |

- `positions` entries are `(row,col)`: row = obstacle 0-3, col 0 = x, 1 = y. A missing entry is loaded as 0 with
  `WARNING: Loaded at least one default value in matrix: "obstacles.positions"`.
- `opti_pessi_scenario.py` (Gazebo, part 2) knows scenarios 1-4 only.
- Several obstacles may circle the goal: each takes its own orbit from its start position (before 2026-10-15 only the
  first circling obstacle did, a second one got a zero radius).

#### 1.3 Scenario 6: random moving obstacles

`scenario_S6.info` runs four random-walking obstacles inside a corridor, using all four OCP slots for 30 simulated
seconds. It is a closed-loop scenario only; there is no matching Gazebo scenario script.

```bash
ros2 run opti_pessi_interface opti_pessi_interface_mpc \
  src/legged_control/opti_pessi_interface/config/task.info \
  src/legged_control/opti_pessi_interface/config/scenario_S6.info --quiet

python3 src/legged_control/opti_pessi_interface/python/plot_mpc.py 6
```

The random seed, corridor bounds, obstacle speeds and initial positions are in `scenario_S6.info`.

---

### 2. Scenarios in Gazebo

`opti_pessi_scenario.py <1|2|3|4>` sends the goal and the obstacle of the same scenarios to the controller, every
distance x10 (scenario 1: goal (10, 0), obstacle (1, 10)).

```bash
# 1. simulation: an empty 70 x 45 m room with one person, the robot at the world origin (server:=false for the GUI)
ros2 launch legged_unitree_description scenario_room_launch.xml

# 2. controller: the robot stands and waits for a goal
ros2 launch legged_controllers load_optipessicontroller_launch.xml
```

Then one of the two.

#### 2.1 Obstacle given to the controller

```bash
# 3.
ros2 run opti_pessi_interface opti_pessi_scenario.py 2
```

- The obstacle goes straight to `/opti_pessi/obstacles`, everything in `odom`. The orange disk on
  `/opti_pessi/scenario_obstacles` shows it.
- Not next to `people:=true`: the two would erase each other.
- Any other world does ([section 5.3](#53-common-launches)).

#### 2.2 Obstacle as a person in Gazebo

The script does not tell the controller where the obstacle is: the person of the room walks where the obstacle is,
and the lidar people pipeline has to find the person.

```bash
# 3. robot standing where it spawned:
ros2 launch ranav_bringup fast_lio_slam_launch.xml pgo:=false people:=true

# 4. detector running (ros2 topic hz /perception/detector/objects):
ros2 run opti_pessi_interface opti_pessi_scenario.py 2 --person
```

- With `--person` the scenario is laid out in the world of Gazebo, not in `odom`: the estimator drifts by about 13 % of
  the way walked. The goal is sent where the world goal is in `odom` now, the antagonist heads for the robot where
  FAST-LIO has it. Both need FAST-LIO started with the robot standing at the spawn (its `lio_map` is then the world
  frame) and its `odom -> lio_map` (FAST-LIO publishes it from the legged estimator's `/odom`, `alignment.*` in
  `fast_lio_airy.yaml`); without them `odom` is taken for the world.
- `odom -> lio_map` is also what lets the people reach `/opti_pessi/obstacles`.
- The orange disk on `/opti_pessi/scenario_obstacles` is where the person is; the obstacle the controller is given is
  the tracked detection.
- The controller gets the person only while the pipeline finds it, and not farther than 10 m (`people_tracker.yaml`,
  `obstacles.max_range`): the obstacle of scenario 1, at (1, 10), never reaches it. Since 2026-10-13 the Airy is pitched 5 deg
  forward (15 before): near the robot it sees a person only above 0.5 m (33-39 points at 5 m), and the pretrained
  CenterPoint model finds a person 5 m ahead in 1 % of the scans when standing, 8 % walking at 0.25 m/s, 47 % at
  0.6 m/s; behind the robot 5 % at most. Every message on `/opti_pessi/obstacles` replaces the obstacles of the
  controller, an empty one clears them.
- The person goes wherever `/scenario_person/pose` (`geometry_msgs/msg/Pose`, world x and y) says: any node can walk it.
- Test of the script: `python3 -m pytest src/legged_control/opti_pessi_interface/test/test_opti_pessi_scenario.py`

Measured in the headless simulation (2026-10-12, scenario 2, pretrained CenterPoint model, lidar still pitched 15 deg
forward: the detection rows are worse with the 5 deg mount, see above):

| | |
|---|---|
| Person against the position sent, by the lidar points | 0.03-0.09 m standing, 0.12 m walking at 0.5 m/s |
| Obstacle given to the controller against the person | 0.10 m median, 0.17 m p95 |
| Scans with an obstacle for the controller, person ahead within 9 m | 44 % (detector alone 26 %); first one 3.6 s after the start, person 3.8 m from the robot (6.4 m at the start) |
| Same, person behind the robot | 7 % |
| Robot at the end against the world goal (10, 2) | (10.02, 1.92) |

---

### 3. Exploration

```bash
# 1. simulation (empty_room_launch.xml for the room with people)
ros2 launch legged_unitree_description warehouse_launch.xml

# 2. controller
ros2 launch legged_controllers load_optipessicontroller_launch.xml rviz:=false recompile:=true

# 3. robot standing still at the spawn: the whole exploration (FAST-LIO + SC-PGO + people perception, TARE, the goal
#    bridge, one rviz)
ros2 launch ranav_bringup explore_warehouse_launch.xml

# 4. start
ros2 topic pub -1 /start_exploration std_msgs/msg/Bool "{data: true}"
```

When TARE reports the exploration finished, it walks the robot back to where it stood at the launch. Only once the
robot is home (within 0.5 m) are TARE and the robot stopped and the map merged: `src/map_generator/save_data/optimized_map.pcd`.
Then the launch shuts down on its own. SC-PGO saves in `save_data/` of the map_generator package
(`save_directory` of `fast_lio_slam_launch.xml`: `$(find-pkg-share map_generator)/save_data`, a link to
`src/map_generator/save_data`).

| Argument of `explore_warehouse_launch.xml` | Default | |
|---|---|---|
| `auto_start` | `false` | `true`: explore at once, without `/start_exploration` |
| `return_home` | `true` | `false`: stop the robot where the exploration ends, merge at once |
| `return_home_timeout` | `600.0` | seconds given to the return; after that the robot is stopped where it is |
| `lidar` | `sim` | `real`: the real Airy, its driver started too, wall clock |
| `people` | `true` | `false`: no people detection (the scans pass through unchanged) |

Worlds and controller: [section 5.3](#53-common-launches).

---

### 4. Inspections

They need `optimized_map.pcd` of an [exploration](#3-exploration) of the same world, and the robot standing where that
exploration started: the localization takes its first guess from there.

```bash
# 1. simulation: the world of the exploration
ros2 launch legged_unitree_description warehouse_launch.xml

# 2. controller
ros2 launch legged_controllers load_optipessicontroller_launch.xml rviz:=false recompile:=true

# 3. robot standing still at the spawn: the inspections (FAST-LIO + people perception without SC-PGO, localization in
#    the map, far_planner, the goal bridge, the map point picker on http://localhost:8765, one rviz)
ros2 launch ranav_bringup inspection_launch.xml
```

The robot does not move at the launch: the inspection starts on **Start inspection** in the map point picker, with
the points on the page. After the last point it waits for the next start: for the next inspections change the points
on the page and press **Start inspection** again, nothing has to be relaunched.

```bash
# without the picker: the points of goals_file (save_data/selected_points.yaml)
ros2 topic pub -1 /start_inspection std_msgs/msg/String "{data: ''}"
```

The robot follows the route far_planner plans (the line it shows in rviz and in the map point picker): the goal sent
to OptiPessi is the point 3 m ahead of the robot along it, nearer or a detour when something is in the way.

| Argument of `inspection_launch.xml` | Default | |
|---|---|---|
| `picker` | `true` | `false`: no map point picker (start with `/start_inspection`) |
| `map` | `save_data/optimized_map.pcd` | Map of the exploration, for the localization and the picker |
| `auto_start` | `false` | `true`: inspect `goals_file` at once, no start command |
| `goals_file` | `save_data/selected_points.yaml` | Points inspected with `auto_start`, or on a start command that names no file |
| `graph_file` | `save_data/warehouse.vgh` | far_planner's graph: loaded at the start if the file exists (else far_planner builds its own), saved after each inspection (`save_graph:=false`: not saved) |
| `dwell_time` | `2.0` | Seconds at each point |
| `loop` | `false` | `true`: start over after the last point |
| `people_free` | `true` | far_planner plans on terrain maps without the tracked people; the goal bridge and OptiPessi still avoid them. `false`: far_planner sees people as obstacles (it then routes round where they walked, and gives a goal up with a person next to the robot) |
| `waypoint` | `false` | `true`: relay far_planner's `/way_point` straight to OptiPessi, no route following and no obstacle check |
| `lidar`, `people` | `sim`, `true` | As for the exploration |

```bash
# lookahead along the route, while it runs
ros2 param set /opti_pessi_goal_bridge lookahead_distance 2.0
```

SC-PGO is left out (`pgo:=false`): an inspection does not use it, and SC-PGO empties `save_data/Scans/` (the
keyframes of the exploration) when it starts.

---

### 5. Utils

#### 5.1 Map point picker (web interface)

```bash
ros2 run map_generator map_point_picker.py
```

- Page: http://localhost:8765 (`--port`). Click the map to add a point, drag a point to move it; the points are saved
  in `selected_points.yaml` next to the map (`--output`).
- It shows the FAR graph (`warehouse.vgh` next to the map) and, with ROS 2 sourced in its terminal and the same
  `ROS_DOMAIN_ID`, the robot and the paths of FAR, TARE and OptiPessi live.
- **Start inspection** saves the points and starts the inspection of [section 4](#4-inspections), which must be
  running.
- The map is read at the start: restart the picker after an exploration has rewritten it.

#### 5.2 Goals and commands from the terminal

Goal straight to the controller (frame `odom`):

```bash
ros2 topic pub --once /opti_pessi/goal visualization_msgs/msg/MarkerArray \
"{markers: [{header: {frame_id: odom}, ns: goal, type: 2, action: 0, pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}, scale: {x: 0.3, y: 0.3, z: 0.3}, color: {g: 1.0, a: 1.0}}]}"
```

Goal to far_planner, with `inspection_launch.xml` running (frame `map`; `z` is the one of `/Odometry`, 0 on a flat
floor: far_planner measures its distance to the goal in 3D):

```bash
ros2 topic pub -1 /goal_point geometry_msgs/msg/PointStamped "{header: {frame_id: map}, point: {x: 5.0, y: 0.0, z: 0.0}}"
```

Start commands:

```bash
ros2 topic pub -1 /start_exploration std_msgs/msg/Bool "{data: true}"
ros2 topic pub -1 /start_inspection std_msgs/msg/String "{data: ''}"      # '' = goals_file, or the path of a points file
```

Stop the robot where it is during an exploration (TARE waypoint = robot position):

```bash
ros2 topic pub -1 /reset_waypoint std_msgs/msg/Empty "{}"
```

#### 5.3 Common launches

##### Simulation

```bash
ros2 launch legged_unitree_description warehouse_launch.xml       # the warehouse
ros2 launch legged_unitree_description empty_room_launch.xml      # 30 x 30 m room, 7 packages, 4 people on random walks
ros2 launch legged_unitree_description scenario_room_launch.xml   # 70 x 45 m room, one person (section 2)
```

| Argument | Default | Effect |
|---|---|---|
| `server:=false` | `true` | Gazebo with its GUI (default: headless) |
| `lidar:=true` | `true` | RoboSense Airy: `/rslidar_points` (10 Hz, reliable QoS) and `/rslidar_imu_data` (200 Hz) |
| `x`, `y` | `0.0` | Robot spawn in the world (not in the scenario room: always the origin) |

- In the empty room the robot spawns at the centre. The empty and scenario rooms have a ceiling at 3 m, seen from below only:
  the Airy (pitched 5 deg forward since 2026-10-13) sees the floor only from 6 m ahead and FAST-LIO holds its height
  on the ceiling. Around the robot the terrain maps get their floor from a flat-floor prior (`groundPriorFill` in
  `ranav_bringup/config/terrain_airy.yaml`): far_planner crashes without floor points.

##### Controller (choose one)

Opti-Pessi, used by the scenarios, exploration and inspections in this guide by default:

```bash
ros2 launch legged_controllers load_optipessicontroller_launch.xml rviz:=false recompile:=true
```

Base legged controller, without Opti-Pessi:

```bash
ros2 launch legged_controllers load_controller_launch.xml
```

| Argument | Default | Effect |
|---|---|---|
| `rviz` | `true` | rviz of the controller |
| `recompile` | `false` | `true`: generate the solver libraries again (after an edit of `task.info` or of `opti_pessi_interface`) |
| `scenarioFile` | `opti_pessi_interface/config/scenario_live.info` | Obstacle slots (`obstacles.numSlots`, 4) and goal tolerance. `scenarioFile:=$(ros2 pkg prefix opti_pessi_interface)/share/opti_pessi_interface/config/scenario_S1.info` for 1 slot |

- Obstacle slots: the controller keeps out at most `numSlots` obstacles of `/opti_pessi/obstacles` at once, the closest
  keep-out disks; more log `obstacles but 4 OCP slots ... tracking the closest`.
- Each solve uses the OCP with one slot per obstacle present, so the solve time follows the people around, not
  `numSlots` (Gazebo, 50 iterations: 9 ms with one obstacle or none, 15 ms with two, 33 ms with four). The per-phase
  `plan:` line (`phaseDiagnostics:=true`) logs `obstacleSlots=`; the solver statistics (`ipm.printSolverStatistics`)
  count `remapped to other obstacles`.
- Solver libraries: one folder per robot model and slot count,
  `install/opti_pessi_interface/share/opti_pessi_interface/lib/m<mass>_I<inertia>_o<slots>`, with a subfolder
  `slots1` ... `slots4` per OCP size. A missing one is generated at launch (about 5 s a size, 20 s for all four the
  first time), so changing `numSlots` needs no `recompile:=true`; an edit of `task.info` still does.

###### Acados controller: simulation and planner use

Acados is a separate `ros2_control` plugin. The standalone launch starts Gazebo in `empty_world.world`, spawns the
robot, activates the controller, and opens RViz:

```bash
ros2 launch acados_legged_controller sim_full.launch.py
```

Building it: [Acados controller](#acados-controller).

Choose another Gazebo world with `world:=warehouse_world.world`. Wait for the controller to report `STANCE`, then send
a goal in `odom` from another terminal; the first goal starts walking:

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
'{header: {frame_id: odom}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'
```

To use Acados with the RaNAV planner, select its controller configuration when starting Gazebo, then launch its
controller and goal relay:

```bash
# Terminal 1: warehouse simulation configured for Acados
ros2 launch legged_unitree_description warehouse_launch.xml \
  controllers_yaml:=$(ros2 pkg prefix --share acados_legged_controller)/config/ros2_controllers.yaml

# Terminal 2: Acados controller and relay from /opti_pessi/goal to /goal_pose
ros2 launch acados_legged_controller load_controller_launch.xml rviz:=false

# Terminal 3: optional exploration stack; its goals are forwarded to Acados
ros2 launch ranav_bringup explore_warehouse_launch.xml
```

The planner launch is optional; for manual goals use `/goal_pose` as above. The default Acados configuration receives
obstacles on `/opti_pessi/obstacles`. In the standalone simulation, start the configured virtual obstacles with:

```bash
# Terminal 1: virtual obstacles from config.yaml, using simulation time
ros2 run acados_legged_controller obstacle_simulator --ros-args -p use_sim_time:=true
# Terminal 2: start them moving
ros2 service call /obstacle_simulator/set_moving std_srvs/srv/SetBool "{data: true}"
```

For lab hardware, use the setup and startup steps in
[`src/legged_control/acados_legged_controller/instruction.md`](src/legged_control/acados_legged_controller/instruction.md).

##### FAST-LIO + SC-PGO + people perception

```bash
ros2 launch ranav_bringup fast_lio_slam_launch.xml rviz:=false people:=true
```

- Start it with the robot standing still: FAST-LIO takes gravity and its frame from the first IMU samples.
- On the Airy: `ranav_bringup/config/fast_lio_airy.yaml` (`lid_reliable`, `point_filter_num` 4),
  TF `... -> rslidar_lio -> lio_base`.
- It also starts people_perception (`people_perception_launch.xml`). SC-PGO reads `/cloud_registered_body_static`
  and TARE `/registered_scan_static` from its `dynamic_points_filter`. **If the filter is down, neither gets scans.**
  far_planner builds its graph from `/registered_scan_static` too (terrain maps of its own, `/terrain_map_static`
  and `/terrain_map_ext_static`), in the exploration and in the inspections: walking people leave no walls in it.
- The filter has no launch of its own, and `people_perception_launch.xml` must not be launched by hand next to this
  one.

| Argument | Default | Effect |
|---|---|---|
| `lidar` | `sim` | `real`: the real Airy. Starts its driver (`rslidar_sdk`), `use_sim_time` false, `ranav_bringup/config/fast_lio_airy_real.yaml`. See below |
| `config:=<file>` | the one of `lidar` | Parameters of FAST-LIO |
| `people` | `true` | `false`: the two static clouds are the FAST-LIO ones, unchanged. `true`: cloud adapter + detector + tracker run too and the filter takes the tracked people out; tracks and removed points show in the People group of the TARE rviz |
| `pgo` | `true` | `false`: no SC-PGO (no map) |
| `centerpoint_model:=<folder>` | `models/lidar_centerpoint/v4.1/short_range` | Model of the detector, see below (`models/` of the workspace the launch is installed from, not always `~/ranav_ws`) |
| `detector` | `auto` | `centerpoint` (TensorRT), `centerpoint_cpu` (ONNX Runtime) or `auto`: `centerpoint` when `autoware_lidar_centerpoint` is installed, else `centerpoint_cpu` (a build with `--no-cuda`) |
| `opti_pessi_obstacles:=false` | `true` | Do not send the tracked people to the controller: `/opti_pessi/obstacles` is left to the camera detector |

Real Airy (`lidar:=real`), wired to this PC:

```bash
ros2 launch ranav_bringup fast_lio_slam_launch.xml lidar:=real
```

- The Airy (192.168.1.200) sends to 192.168.1.10: the wired port needs that address (NetworkManager profile
  `airy-lidar`, `nmcli connection up airy-lidar`). Its settings: http://192.168.1.200.
- Driver: `src/ranav_bringup/config/rslidar_airy.yaml`, given to `rslidar_sdk_node` as `config_path` (`RSAIRY`, IMU on
  6688, PC clock: the Airy has no time sync and counts from 1970; rows = rings, 96 x 900 as in the simulation).
  Installed as a copy: `colcon build --packages-select ranav_bringup` after an edit.
- `fast_lio_airy_real.yaml`: the IMU of the real Airy is turned 180 deg against the lidar (factory calibration in its
  DIFOP), lidar level. Mounted pitched on the robot, `imu_R` and `extrinsic_R` change as the comment there says.
- No per-point motion compensation yet: the real scans take 100 ms and smear on fast turns.
- `pgo:=false` keeps `save_data/Scans/` of the exploration: SC-PGO empties it when it starts.

Detector (with `people:=true`):

- `autoware_lidar_centerpoint` with the pretrained Autoware model: it finds the people in the points.
- Needs CUDA 13.0 + TensorRT 10.13 (`./setup_ranav.sh cuda`) and their paths (`ranav_env.sh` sets them). The engines
  in the model folder were built for this GPU.
- Not built (`setup_ranav.sh build --no-cuda`): `detector:=auto` runs the same model on the CPU,
  `people_perception centerpoint_cpu.py`, with the python of `.venv` in the workspace (it needs onnxruntime).
- Measured on the Gazebo actors, headless, 51 s:

  | | |
  |---|---|
  | A track on an actor | 97 % of the time |
  | Tracks on nobody | 2 % |
  | Actor points removed | 99 % |
  | Confirmed tracks started for 2 people | 8 |
  | Inference | 45 ms median, 100 ms p95 |
  | GPU memory, with the simulation | 2.3 GB |

People as obstacles of the controller (with `people:=true`):

1. `people_tracker` publishes the confirmed people within 10 m on `/opti_pessi/obstacles` in `odom`, as `HUMAN`.
   Radius and max speed are the controller's (`task.info`).
2. It needs the `odom -> lio_map` TF, which FAST-LIO publishes while the controller's `/odom` runs. Until then it
   publishes nothing.

> **Do not run `yolo26_rgbd_detect` next to it.** Both publish `/opti_pessi/obstacles` and each message replaces the
> obstacles of the other. `opti_pessi_obstacles:=false` leaves the topic to the camera detector.

##### Camera people detector (optional)

```bash
ros2 run opti_pessi_perception yolo26_rgbd_detect --ros-args -p use_sim_time:=true
```

Not together with `people:=true`, unless `opti_pessi_obstacles:=false` (see the warning above).

#### 5.4 Merge the SC-PGO keyframes into one map

By hand; the exploration does it at its end.

```bash
.venv/bin/python install/map_generator/lib/map_generator/make_merged_map.py
```

- Reads and writes `save_data/` of map_generator (first argument to change it): `optimized_map.pcd` (`--output`),
  without what moved (`--keep-dynamic` keeps it). Needs open3d: the python of `.venv` in the workspace.

#### 5.5 Checks

Airy lidar:

```bash
./build/legged_gazebo/test_airy_points_conversion

# sim running and robot standing: layout, 10 Hz, frame gap, self-hits, IMU, the dome looking where its mount (5 deg
# forward) says, returns on every side
python3 src/legged_control/legged_gazebo/test/airy_sim_check.py

# subscribe RELIABLE: best effort (ros2 topic hz, SensorDataQoS) only gets ~1 Hz of the 2 MB clouds
ros2 topic echo --once /rslidar_points --qos-reliability reliable --field header
```

People perception (see `docs/people_perception.md`):

```bash
colcon test --packages-select people_perception --event-handlers console_direct+

# sim and fast_lio_slam_launch.xml people:=false running: the static clouds are the FAST-LIO ones byte for byte, none
# lost; same stamp on both clouds and /Odometry; same number of points in both clouds
python3 src/people_perception/test/dynamic_points_filter_check.py

# with people:=true: the static clouds are the FAST-LIO ones but for the points taken out, none lost, in order;
# /perception/dynamic_points is exactly what is missing; a detector cloud (PointXYZIRC) and tracks for every scan
python3 src/people_perception/test/dynamic_points_filter_check.py --people

# clouds filtered with the tracks of their scan / of another scan (one the CPU detector skipped), clouds unfiltered
# (detector down: the status is WARN), points removed, longest wait
ros2 topic echo /diagnostics | grep -A12 dynamic_points_filter
```

Opti-Pessi obstacle slots (no simulation needed): slot assignment, where a cold start puts the separating planes, free
slots constraining nothing (generates its libraries in `/tmp/ocs2`):

```bash
colcon test --packages-select opti_pessi_interface --ctest-args -R "test_obstacle_slots|test_hyperplane_seed|test_free_slots"
colcon test-result --test-result-base build/opti_pessi_interface --all | grep -E "obstacle_slots|hyperplane_seed|free_slots"

# or one by one, with the output
./build/opti_pessi_interface/test_obstacle_slots
./build/opti_pessi_interface/test_hyperplane_seed
./build/opti_pessi_interface/test_free_slots
```

Tests of the scripts (no simulation needed):

```bash
python3 -m pytest src/legged_control/opti_pessi_interface/test/test_opti_pessi_scenario.py
python3 -m pytest src/ranav_mission/test
python3 -m pytest src/map_generator/test
```

#### 5.6 Dataset for the detector model

With `people:=true` running:

```bash
ros2 bag record -o <bag> /perception/detector/pointcloud /perception/detector/objects /tf /tf_static /Odometry
ros2 run people_perception export_detector_dataset.py <bag> <out_dir> --every-seconds 0.5
```

- Output: `points/*.bin`, `labels/*.txt` (mmdetection3d custom dataset), `ImageSets`, `frames.csv`.
- The labels are the answers of the detector in the bag: a first guess to correct by hand.

#### 5.7 Gazebo silent at launch

Gazebo stays silent at launch and the robot never spawns (seen 2026-10-13 after many nodes had been killed with
`kill -9`): stale Fast DDS shared-memory files make nodes that start together block each other. With every ROS process
stopped (`ros2 daemon stop` too):

```bash
rm /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*
```

#### 5.8 Replay a ROS bag

```bash
# Terminal 1: RViz
rviz2 -d ~/ranav_ws/install/ranav_bringup/share/ranav_bringup/rviz/tare_planner_warehouse.rviz \
  --ros-args -p use_sim_time:=true
# Terminal 2: replay the bag and publish its clock
ros2 bag play --clock <bag_directory>
```

Replace `<bag_directory>` with the path to the bag to replay. `--clock` publishes the bag clock for RViz's simulated
time.
