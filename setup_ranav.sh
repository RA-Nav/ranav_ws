#!/usr/bin/env bash
# Install, build and start the RaNAV stack on Ubuntu 24.04 + ROS 2 Jazzy:
#   legged_control         Aliengo in Gazebo (Harmonic, ros_gz) and the Opti-Pessi MPC controller
#   perception             lidar people detector (Autoware CenterPoint, CUDA 13.0 + TensorRT 10.13), tracker, filter
#                          (lidar_perception), camera detector (camera_perception), emergency exits (exit_inspection)
#   tare_planner           exploration (TARE) and its bridges to Opti-Pessi
#   far_planner            route planner of the inspections (autonomy_stack_go2) + terrain_analysis(_ext), graph_decoder
#   FAST_LIO_SLAM_ROS2     FAST-LIO + SC-PGO
#   FAST_LIO_LOCALIZATION  localization in the map of an exploration
# and everything they need: apt packages, pinocchio, the source repositories, the python with open3d, the
# CenterPoint model, the RoboSense driver (rslidar_sdk) and the Unitree hardware plugin of the real robot.
#
# Machines: x86_64 PC (simulation, as this stack was developed) or the NVIDIA Jetson Orin of the robot (aarch64,
# Ubuntu 24.04 / JetPack 7). On the Jetson: pinocchio is built from source (robotpkg has no arm64 packages), CUDA and
# TensorRT come from JetPack, TARE gets the arm64 OR-Tools and legged_unitree_hw the arm64 Unitree SDK.
#
# Fresh machine: clone the workspace repository (this script is in its root) to ~/ranav_ws, then run
#     git clone --recurse-submodules https://github.com/AlexBarbi/ranav_ws.git ~/ranav_ws
#     cd ~/ranav_ws && ./setup_ranav.sh install
# then open a new terminal and
#     ./setup_ranav.sh start exploration            (simulation)
#     ./setup_ranav.sh start exploration --real     (robot)
#
# Workspace: always the folder containing this script, ~/ranav_ws. Our packages are in src/, third-party code in
# third_party/ (submodules of the repository; sources clones any that is missing), the documentation in docs/.
# Downloaded assets, build output, environment files, temporary downloads and ROS logs are kept under that folder too.
# System packages and required system configuration (apt, /etc and /opt/openrobots) remain system-managed. Run the
# commands below from ~/ranav_ws:
#   install                 deps, cuda, sources, venv, model, build: everything, in this order
#   deps                    apt: ROS 2 Jazzy desktop, Gazebo (ros_gz), ros2_control, build tools; pinocchio 3.9.0
#                           (robotpkg on x86_64, from source on aarch64); rosdep keys of the sources (when cloned)
#   cuda                    x86_64: CUDA toolkit 13.0 + TensorRT 10.13.3 (pinned and held); Jetson: JetPack's
#                           (nvidia-jetpack). Never installs a GPU driver
#   sources                 check out the submodules not checked out yet, then clone the repositories still missing
#                           into src/ and third_party/; a folder already there is never touched
#   venv                    .venv: python with open3d for the localization and the map merge
#   model                   CenterPoint model (Hugging Face AutowareFoundation/lidar_centerpoint v4.1 short_range)
#   build                   colcon build of the stack, Release
#   env [--bashrc]          write ranav_env.sh and the Fast DDS profile in the workspace, then print how to use them;
#                           --bashrc only prints the line to add manually to ~/.bashrc
#   start exploration       simulation, controller, ranav_bringup explore_warehouse_launch.xml (FAST-LIO + SC-PGO +
#                           people, TARE); the merged map goes to src/map_generator/save_data
#   start inspection        simulation, controller, ranav_bringup inspection_launch.xml (FAST-LIO + people,
#                           localization, far_planner, map point picker)
#   start scenario <1-4>    scenario room, controller, opti_pessi_scenario.py (--person: the Gazebo person walks it
#                           and the lidar pipeline has to find it). Simulation only
#   start ... --real        the robot instead of Gazebo: legged_unitree_hw real_robot_launch.xml (Unitree low-level
#                           UDP; one of the changes below), ranav_bringup lidar:=real (the Airy driver, wall clock);
#                           asks before the controller stands the robot up
#   stop                    Ctrl-C to every window of the session, last first, then close it
#   attach                  show the session (tmux: Ctrl-b n / Ctrl-b p next / previous window, Ctrl-b d detach)
#
# Options:
#   --no-cuda               install/cuda/build: no CUDA, no TensorRT: start then runs the CPU people detector
#   --cpu-detector          start: the people detector on the CPU (lidar_perception centerpoint_cpu.py, ONNX Runtime,
#                           +-12.8 m) even where the TensorRT one is built. Automatic where it is not (no NVIDIA GPU)
#   --camera                sources/build: also the camera people detector camera_perception (YOLOs-CPP + ONNX Runtime)
#   --world NAME            start: warehouse (default) or empty_room (the 30 x 30 m room with people)
#   --gui                   start: Gazebo with its GUI (default headless)
#   --no-people             start: people:=false (no detector, the static clouds are the FAST-LIO ones)
#   --recompile, --no-recompile
#                           start: regenerate the Opti-Pessi solver libraries (default: yes for exploration and
#                           inspection, as in README.md; no for scenarios)
#   --auto-start            start exploration/inspection: start at once (auto_start:=true) instead of waiting for
#                           /start_exploration or the Start inspection button
#   --person                start scenario: see above
#   --real                  start: the real robot, see above
#   --dry-run               deps/cuda/sources/venv/model/build: print the commands instead of running them
#
# Environment: RANAV_SESSION (tmux session, default ranav), JOBS (compile jobs, default from cores and memory),
# STAND_WAIT (seconds left to the robot to stand before FAST-LIO starts, default 10),
# GIT_PROTOCOL (https, default, or ssh: how GitHub is reached, see below), ROS_DOMAIN_ID.
#
# Repositories: all public. The submodules of the workspace come from the URLs in its .gitmodules; those of
# legged_control (opti_pessi_interface, acados_legged_controller) have ssh URLs, fetched over https unless
# GIT_PROTOCOL=ssh (which needs an SSH key known to GitHub).
#
# WSL: the GPU driver is the Windows one (CUDA 13 needs driver >= 580). This script installs the toolkit only.
#
# Changes the Jetson build and the real robot need, in none of our pushed branches yet (sources / build list the
# missing ones; the x86 simulation builds without them, the Jetson build stops):
#   ocs2            ocs2_sqp/blasfeo_catkin, hpipm_catkin: ARM targets (they forced x86 AVX kernels)
#   legged_control  legged_unitree_hw: SDK binary per architecture, /contact in the controllers' foot order,
#                   launch/real_robot_launch.xml; legged_unitree_description gazebo.xacro:
#                   sim:=false loads legged_unitree_hw/UnitreeHW_<sdk> (it loaded the empty legged_hw/LeggedHW)
set -Eeo pipefail
trap 'echo "setup_ranav.sh: failed at line $LINENO: $BASH_COMMAND" >&2' ERR

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
SESSION="${RANAV_SESSION:-ranav}"
STAND_WAIT="${STAND_WAIT:-10}"
GIT_PROTOCOL="${GIT_PROTOCOL:-https}"
SETUP_TMP_DIR="$WS/.setup-tmp"

ARCH=${RANAV_ARCH:-$(uname -m)}  # RANAV_ARCH: dry-run the other architecture
JETSON=0
if [ -f /etc/nv_tegra_release ] || dpkg-query -W nvidia-l4t-core >/dev/null 2>&1; then JETSON=1; fi

ROS_SETUP=/opt/ros/jazzy/setup.bash
# robotpkg's pinocchio on x86_64; the same version built from source on aarch64, into the same prefix
PINOCCHIO_DIR=/opt/openrobots/lib/cmake/pinocchio
CASADI_CMAKE_DIR=/opt/openrobots/lib/cmake/casadi
COAL_CMAKE_DIR=/opt/openrobots/lib/cmake/coal
HPP_FCL_CMAKE_DIR=/opt/openrobots/lib/cmake/hpp-fcl
PINOCCHIO_VERSION=v3.9.0
# x86_64: the pinned toolkit; Jetson: JetPack's, behind the /usr/local/cuda link
if [ "$ARCH" = aarch64 ]; then CUDA_DIR=/usr/local/cuda; else CUDA_DIR=/usr/local/cuda-13.0; fi
TRT_VERSION=10.13.3.9-1+cuda13.0
VENV="$WS/.venv"
MODEL_DIR="$WS/models/lidar_centerpoint/v4.1/short_range"
SAVE_DATA="$WS/src/map_generator/save_data"
ENV_FILE="$WS/ranav_env.sh"
FASTDDS_PROFILE="$WS/fastdds_shm_16mb.xml"

# Repositories, as in .gitmodules: folder in the workspace (src/ ours, third_party/ the others) | GitHub repository |
# branch, tag or commit | public or private | sparse checkout. Our forks follow their branch; third-party code is pinned
# to the commit this stack was built and tested with (the one the workspace records for its submodule). Cloned only
# where the folder is missing: a submodule of the workspace that is checked out is left as it is. The sparse checkout
# applies to a submodule too, when sources checks it out.
REPOS=(
  "src/legged_control|idra-lab/legged_control|opti-pessi-acados-real|public|"
  "src/autonomy_stack_go2|RA-Nav/autonomy_stack_go2|foxy-humble|public|"
  "src/tare_planner|RA-Nav/tare_planner|humble-jazzy|public|"
  "src/FAST_LIO_SLAM_ROS2|RA-Nav/FAST_LIO_SLAM_ROS2|main|public|"
  "src/FAST_LIO_LOCALIZATION|RA-Nav/FAST_LIO_LOCALIZATION|main|public|"
  "src/perception|RA-Nav/perception|main|public|"
  "src/map_generator|RA-Nav/map_generator|main|public|"
  "third_party/ocs2|idra-lab/ocs2|ros2|public|"
  "third_party/ocs2_robotic_assets|leggedrobotics/ocs2_robotic_assets|9feab348a1d9d10a0573ca43c7fc4ac2e4565cf2|public|"
  "third_party/autoware_msgs|autowarefoundation/autoware_msgs|1.13.0|public|autoware_perception_msgs"
  "third_party/autoware_universe|autowarefoundation/autoware_universe|8413d4174ca054d272ecf1f84e4c95dfa1f8cf1f|public|common/autoware_cuda_dependency_meta perception/autoware_lidar_centerpoint perception/autoware_multi_object_tracker perception/autoware_tensorrt_common perception/perception_utils sensing/autoware_cuda_utils"
  "third_party/autoware_core|autowarefoundation/autoware_core|8b3b197bb0462b2ad1c1ee5bae307d5f4d75f3bb|public|common/autoware_agnocast_wrapper"
  "third_party/cuda_blackboard|autowarefoundation/cuda_blackboard|a4f330e5c228d713774450b0331b83b94c55a466|public|"
  "third_party/muSSP|tier4/muSSP|c79e98fd5e658f4f90c06d93472faa977bc873b9|public|"
  "third_party/negotiated|osrf/negotiated|eac198b55dcd052af5988f0f174902913c5f20e7|public|negotiated negotiated_interfaces"
  "third_party/rslidar_msg|RoboSense-LiDAR/rslidar_msg|fe8a95cb242bd294cc3d5e3422f2093fb49a56ee|public|"
  "third_party/rslidar_sdk|RoboSense-LiDAR/rslidar_sdk|8b4b4b7ff910799260347821084c59e1c73d50d5|public|"
  "third_party/acados|acados/acados|c15f50a9a95f40728dd8b36d7245fbaa21904206|public|"
)
# arm64 binaries (Jetson): Unitree SDK 3.3.1 for legged_unitree_hw, OR-Tools 9.8 for TARE (its README: replace the
# bundled x86 include/ and lib/ with Google's arm64 release of the same version)
UNITREE_ARM64_URL=https://raw.githubusercontent.com/unitreerobotics/unitree_legged_sdk/v3.3.1/lib/libunitree_legged_sdk_arm64.so
UNITREE_ARM64_SHA=256154915ff8c7a26c8ab6c33183c9bbdf2b52e1aa86fe070bee53eddb752cfc
UNITREE_HW_DIR=src/legged_control/legged_examples/legged_unitree/legged_unitree_hw
ORTOOLS_ARM64_URL=https://github.com/google/or-tools/releases/download/v9.8/or-tools_arm64_debian-11_cpp_v9.8.3296.tar.gz
ORTOOLS_ARM64_SHA=77a88d01eb612ffac0f6e030e30e58a3d171fe75d93a527ec55a06f9070332ab
ORTOOLS_DIR=src/tare_planner/src/tare_planner/or-tools
CAMERA_REPOS=("third_party/YOLOs-CPP|Geekgineer/YOLOs-CPP|2e50d31f0837e96160b72a3311d8a5d4e30b0e47|public|")
ORT_NAME=onnxruntime-linux-x64-gpu-1.20.1
ORT_URL="https://github.com/microsoft/onnxruntime/releases/download/v1.20.1/$ORT_NAME.tgz"

MODEL_URL=https://huggingface.co/AutowareFoundation/lidar_centerpoint/resolve/v4.1/short_range
MODEL_FILES=(
  "pts_voxel_encoder.onnx 2226334c931a4335e49a15bdf82e124c91022a778a92c55de496f03f629de6c7"
  "pts_backbone_neck_head.onnx c41d509c6523da69863bfb63d1cc66420952924b6ed4e0d49461f0ceb31b14b9"
  "ml_package.param.yaml 25556ed14780b2ccd1a3587fe2402db5aee7cf78d99771072013060678fa6370"
  "detection_class_remapper.param.yaml c711f8875ece9b527dfe31ffc75f8c0de2e77945ef67860a959a4e04c36772d5"
)

# Packages started by the launches; colcon builds them and everything they depend on.
BUILD_TARGETS=(
  ranav_bringup ranav_mission map_generator
  legged_control legged_controllers legged_gazebo legged_unitree_description opti_pessi_interface acados_legged_controller
  lidar_perception tare_planner far_planner graph_decoder terrain_analysis terrain_analysis_ext
  fast_lio sc_pgo_ros2 fast_lio_localization
  legged_unitree_hw rslidar_sdk
)
# Left out of the build without CUDA + TensorRT (the people detector and what only it needs).
CUDA_PACKAGES=(
  autoware_lidar_centerpoint autoware_tensorrt_common autoware_cuda_utils autoware_cuda_dependency_meta
  cuda_blackboard negotiated negotiated_interfaces perception_utils
)
# rosdep keys with no rule on noble or that must not be installed: pinocchio would pull ros-jazzy-pinocchio 4.x, whose
# headers mixed with the robotpkg 3.9 libraries crash the controller at load; the others are installed by name below.
ROSDEP_SKIP="pinocchio cmake_clang_tools opencv4 PCL srv_msgs livox_ros_driver2"

APT_SYSTEM=(
  build-essential cmake git curl wget tmux pkg-config
  python3-pip python3-venv python3-dev python3-numpy python3-yaml python3-scipy python3-matplotlib python3-pytest
  python3-colcon-common-extensions python3-rosdep python3-vcstool ros-dev-tools
  libboost-all-dev libeigen3-dev libgoogle-glog-dev libgflags-dev libopencv-dev libpcl-dev libceres-dev
  libtinyxml2-dev liburdfdom-dev pybind11-dev
  liblcm-dev libpcap-dev libyaml-cpp-dev iputils-ping file
)
APT_ROBOTPKG=(robotpkg-pinocchio robotpkg-coal)
APT_ROS=(
  ros-jazzy-desktop
  ros-jazzy-ros-gz ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-interfaces ros-jazzy-gz-sim-vendor
  ros-jazzy-gz-ros2-control ros-jazzy-simulation-interfaces
  ros-jazzy-controller-manager ros-jazzy-controller-interface ros-jazzy-hardware-interface ros-jazzy-ros2-controllers
  ros-jazzy-ros2controlcli ros-jazzy-realtime-tools ros-jazzy-joint-state-broadcaster ros-jazzy-imu-sensor-broadcaster
  ros-jazzy-joint-state-publisher ros-jazzy-robot-state-publisher ros-jazzy-xacro ros-jazzy-kdl-parser
  ros-jazzy-joy ros-jazzy-joy-teleop
  ros-jazzy-hpp-fcl ros-jazzy-gtsam ros-jazzy-pcl-ros ros-jazzy-pcl-conversions ros-jazzy-vision-msgs
  ros-jazzy-diagnostic-updater ros-jazzy-eigen3-cmake-module ros-jazzy-tf-transformations ros-jazzy-rosbag2-storage-mcap
  ros-jazzy-autoware-cmake ros-jazzy-autoware-lint-common ros-jazzy-autoware-utils ros-jazzy-autoware-point-types
  ros-jazzy-autoware-object-recognition-utils ros-jazzy-tensorrt-cmake-module
)

DRY_RUN=0 NO_CUDA=0 CAMERA=0 GUI=0 PEOPLE=true AUTO_START=false PERSON=0 BASHRC=0 RECOMPILE="" WORLD=warehouse IN_INSTALL=0
REAL=0 DETECTOR=centerpoint
ARGS=()

info() { printf '\033[1;34m==>\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33mwarning:\033[0m %s\n' "$*" >&2; }
die() { printf '\033[1;31merror:\033[0m %s\n' "$*" >&2; exit 1; }
[[ "$(basename "$WS")" = ranav_ws ]] || die "setup_ranav.sh must be in the root of the ranav_ws workspace (clone it to ~/ranav_ws); refusing to create workspace files in $WS"
run() { printf '+ %s\n' "$*"; [ "$DRY_RUN" = 1 ] || "$@"; }
# Content on stdin; with --dry-run only says what would be written.
write_file() { if [ "$DRY_RUN" = 1 ]; then echo "+ write $1"; cat >/dev/null; else mkdir -p "$(dirname "$1")"; cat >"$1"; fi; }
sudo_write() { if [ "$DRY_RUN" = 1 ]; then echo "+ sudo write $1"; cat >/dev/null; else sudo tee "$1" >/dev/null; fi; }
setup_tmpdir() {
  local label=$1
  if [ "$DRY_RUN" = 1 ]; then printf '%s/%s.dry-run\n' "$SETUP_TMP_DIR" "$label"; return; fi
  mkdir -p "$SETUP_TMP_DIR"
  mktemp -d "$SETUP_TMP_DIR/$label.XXXXXX"
}
cleanup_tmpdir() {
  rm -rf -- "$1"
  rmdir "$SETUP_TMP_DIR" 2>/dev/null || true
}
installed() { dpkg-query -W -f='${Status}' "$1" 2>/dev/null | grep -q ' ok installed$'; }  # "hold ok installed" too
have_cuda() { [ -x "$CUDA_DIR/bin/nvcc" ] && installed libnvinfer-dev; }

repo_url() {  # <owner/name> <public|private>
  if [ "$2" = private ] && [ "$GIT_PROTOCOL" = ssh ]; then echo "git@github.com:$1.git"; else echo "https://github.com/$1.git"; fi
}
# With GIT_PROTOCOL=https the ssh URLs of the submodules are fetched over https too.
git_cmd() {
  if [ "$GIT_PROTOCOL" = https ]; then git -c "url.https://github.com/.insteadOf=git@github.com:" "$@"; else git "$@"; fi
}
retry() {  # three attempts: GitHub clones sometimes die mid-pack ("fetch-pack: invalid index-pack output")
  local n=1
  until "$@"; do
    ((n >= 3)) && return 1
    warn "attempt $n failed, again in 5 s"; n=$((n + 1)); sleep 5
  done
}

check_os() {
  [ "$(id -u)" != 0 ] || die "run it as your user, not as root: it calls sudo where needed"
  . /etc/os-release
  [ "${VERSION_CODENAME:-}" = noble ] || die "Ubuntu 24.04 (noble) needed, this is ${PRETTY_NAME:-unknown} (on a Jetson: JetPack 7)"
  case "$ARCH" in x86_64 | aarch64) ;; *) die "x86_64 or aarch64 only, this is $ARCH" ;; esac
}

# Compile jobs: ocs2 / CppAD / pinocchio translation units take up to ~2 GB each (on a Jetson the GPU shares it).
compute_jobs() {
  local mem_gb cores
  mem_gb=$(awk '/MemTotal/ {print int($2 / 1048576)}' /proc/meminfo)
  cores=$(nproc)
  # A quarter of the cores (at most one job per 2 GB): the full count froze/crashed a 16-core 30 GB PC. JOBS=n overrides.
  JOBS=${JOBS:-$((mem_gb / 2 < cores / 4 ? mem_gb / 2 : cores / 4))}
  ((JOBS >= 1)) || JOBS=1
}

# Build the bundled Acados as static archives. The controller hides these symbols inside its wrapper library so that
# Acados' HPIPM/BLASFEO cannot collide with the different versions built by OCS2.
build_acados_static() {
  local acados_src="$WS/third_party/acados"
  local acados_build="$WS/build/acados_static"
  local acados_install="$WS/install/acados_static"
  [ -f "$acados_src/CMakeLists.txt" ] || die "Acados sources missing: ./setup_ranav.sh sources"
  [ -f "$acados_src/external/blasfeo/CMakeLists.txt" ] && [ -f "$acados_src/external/hpipm/CMakeLists.txt" ] ||
    die "Acados submodules missing: initialize third_party/acados recursively (./setup_ranav.sh sources)"

  info "build bundled Acados static libraries"
  run cmake -S "$acados_src" -B "$acados_build" -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$acados_install" -DBUILD_SHARED_LIBS=OFF \
    -DACADOS_WITH_SYSTEM_BLASFEO=OFF -DACADOS_WITH_OPENMP=OFF \
    -DACADOS_WITH_QPOASES=OFF -DACADOS_WITH_DAQP=OFF -DACADOS_WITH_HPMPC=OFF -DACADOS_WITH_QORE=OFF \
    -DACADOS_WITH_OOQP=OFF -DACADOS_WITH_QPDUNES=OFF -DACADOS_WITH_OSQP=OFF -DACADOS_WITH_CLARABEL=OFF \
    -DACADOS_EXAMPLES=OFF -DACADOS_UNIT_TESTS=OFF -DACADOS_LINT=OFF -DACADOS_OCTAVE=OFF
  run cmake --build "$acados_build" --parallel "$JOBS"
  run cmake --install "$acados_build"
  if [ "$DRY_RUN" != 1 ]; then
    local lib
    for lib in acados hpipm blasfeo; do
      [ -f "$acados_install/lib/lib$lib.a" ] || die "Acados static library missing: $acados_install/lib/lib$lib.a"
    done
  fi
  export ACADOS_SOURCE_DIR="$acados_src"
  export ACADOS_STATIC_DIR="$acados_install"
}

# ---------------------------------------------------------------------------------------------------------------- deps
cmd_deps() {
  check_os
  info "apt repositories"
  run sudo apt-get update
  run sudo apt-get install -y software-properties-common curl gnupg lsb-release ca-certificates
  run sudo add-apt-repository -y universe
  if ! installed ros2-apt-source; then
    local v tmp
    v=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F '"tag_name"' | awk -F'"' '{print $4}') || true
    [ -n "$v" ] || die "cannot read the latest ros-apt-source release from GitHub"
    tmp=$(setup_tmpdir ros2-apt-source)
    run curl -fL -o "$tmp/ros2-apt-source.deb" \
      "https://github.com/ros-infrastructure/ros-apt-source/releases/download/$v/ros2-apt-source_$v.noble_all.deb"
    run sudo dpkg -i "$tmp/ros2-apt-source.deb"
    cleanup_tmpdir "$tmp"
  fi
  local robotpkg=()
  if [ "$ARCH" = x86_64 ]; then
    robotpkg=("${APT_ROBOTPKG[@]}")
    if ! grep -rqs robotpkg.openrobots.org /etc/apt/sources.list /etc/apt/sources.list.d/; then
      run sudo mkdir -p /etc/apt/keyrings
      run sudo curl -fsSL -o /etc/apt/keyrings/robotpkg.asc http://robotpkg.openrobots.org/packages/debian/robotpkg.asc
      echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub noble robotpkg" |
        sudo_write /etc/apt/sources.list.d/robotpkg.list
    fi
  fi
  run sudo apt-get update

  info "apt packages"
  run sudo apt-get install -y "${APT_SYSTEM[@]}" "${robotpkg[@]}" "${APT_ROS[@]}"
  [ "$ARCH" = x86_64 ] || cmd_pinocchio
  # The libraries in /opt/openrobots/lib are found by the loader through this file (robotpkg does not write it); without
  # it the controller fails to load libpinocchio_*.so
  if ! grep -qsx /opt/openrobots/lib /etc/ld.so.conf.d/openrobots.conf; then
    echo /opt/openrobots/lib | sudo_write /etc/ld.so.conf.d/openrobots.conf
    run sudo ldconfig
  fi

  if [ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]; then run sudo rosdep init; fi
  run rosdep update --rosdistro jazzy
  if [ "$IN_INSTALL" = 1 ]; then return; fi  # install runs rosdep after cloning
  if [ -d "$WS/src/legged_control" ]; then cmd_rosdep; else info "sources not cloned yet: rosdep keys after ./setup_ranav.sh sources"; fi
  check_pinocchio
}

# rosdep keys of the packages the build selects (the repositories hold many more packages than the stack uses).
cmd_rosdep() {
  info "rosdep keys of the stack"
  # shellcheck disable=SC1090
  source "$ROS_SETUP"
  local paths
  # shellcheck disable=SC2046  # one word per target
  mapfile -t paths < <(cd "$WS" && colcon list --paths-only --packages-up-to $(build_targets_line) 2>/dev/null)
  ((${#paths[@]})) || die "colcon finds none of the packages in $WS"
  (cd "$WS" && run rosdep install -y -r --rosdistro jazzy --ignore-src --from-paths "${paths[@]}" --skip-keys "$ROSDEP_SKIP")
  check_pinocchio
}

# robotpkg has no arm64 packages: on aarch64 the same pinocchio is built from source into /opt/openrobots, C++ only,
# with collision support (coal/hpp-fcl of ROS, as ocs2_self_collision needs) and without casadi (nothing uses it).
cmd_pinocchio() {
  if [ -f "$PINOCCHIO_DIR/pinocchioConfig.cmake" ]; then info "pinocchio already in /opt/openrobots"; return; fi
  compute_jobs
  info "pinocchio $PINOCCHIO_VERSION from source into /opt/openrobots ($JOBS jobs; 30-60 min on a Jetson)"
  local tmp; tmp=$(setup_tmpdir pinocchio)
  pinocchio_clone() {
    rm -rf "$tmp/pinocchio"
    git clone --recursive --depth 1 --shallow-submodules --branch "$PINOCCHIO_VERSION" \
      https://github.com/stack-of-tasks/pinocchio.git "$tmp/pinocchio"
  }
  if [ "$DRY_RUN" = 1 ]; then echo "+ git clone --recursive --branch $PINOCCHIO_VERSION https://github.com/stack-of-tasks/pinocchio.git"; else retry pinocchio_clone; fi
  # shellcheck disable=SC1090
  source "$ROS_SETUP"
  run cmake -S "$tmp/pinocchio" -B "$tmp/build" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/openrobots \
    -DCMAKE_PREFIX_PATH=/opt/ros/jazzy -DBUILD_PYTHON_INTERFACE=OFF -DBUILD_WITH_COLLISION_SUPPORT=ON \
    -DBUILD_WITH_CASADI_SUPPORT=OFF -DBUILD_WITH_URDF_SUPPORT=ON -DBUILD_TESTING=OFF -DBUILD_BENCHMARK=OFF -DBUILD_UTILS=OFF
  run cmake --build "$tmp/build" -j "$JOBS"
  run sudo cmake --install "$tmp/build"
  cleanup_tmpdir "$tmp"
}

check_pinocchio() {
  if installed ros-jazzy-pinocchio; then
    warn "ros-jazzy-pinocchio (4.x) is installed. legged_controllers then compiles against its headers but links the"
    warn "robotpkg 3.9 libraries of /opt/openrobots and segfaults when the controller loads. Remove it (apt lists what"
    warn "goes with it and asks first; keep ros-jazzy-coal, liblegged_interface links it):"
    run sudo apt-get remove ros-jazzy-pinocchio
  fi
}

# ---------------------------------------------------------------------------------------------------------------- cuda
cmd_cuda() {
  if [ "$NO_CUDA" = 1 ]; then info "--no-cuda: CUDA and TensorRT skipped"; return; fi
  check_os
  if have_cuda; then info "CUDA and TensorRT already installed ($CUDA_DIR)"; return; fi
  if [ "$ARCH" = aarch64 ]; then
    if [ "$JETSON" != 1 ]; then
      warn "aarch64 but not a Jetson: install CUDA and TensorRT by hand; until then the people detector is not built"
      return
    fi
    # JetPack's own CUDA, cuDNN and TensorRT, matched to the L4T driver of the board (its apt repository is set up by
    # the JetPack image)
    info "Jetson: CUDA, cuDNN and TensorRT of JetPack (nvidia-jetpack)"
    run sudo apt-get update
    run sudo apt-get install -y nvidia-jetpack
    return
  fi
  if grep -qi microsoft /proc/version; then
    info "WSL: the driver comes from Windows (it must support CUDA 13, driver >= 580)"
  elif ! command -v nvidia-smi >/dev/null || ! nvidia-smi >/dev/null 2>&1; then
    warn "no working NVIDIA driver (nvidia-smi). Install one that supports CUDA 13 (>= 580) and run ./setup_ranav.sh cuda"
    warn "again; until then the people detector is not built (people:=false)."
    return
  fi
  info "CUDA toolkit 13.0 and TensorRT $TRT_VERSION (about 7.5 GB to download, 12 GB installed)"
  local tmp; tmp=$(setup_tmpdir cuda-keyring)
  run wget -q -O "$tmp/cuda-keyring.deb" https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb
  run sudo dpkg -i "$tmp/cuda-keyring.deb"
  cleanup_tmpdir "$tmp"
  run sudo apt-get update
  # The toolkit only: never "cuda" or "cuda-drivers" (they bring a driver, wrong inside WSL)
  run sudo apt-get install -y cuda-toolkit-13-0
  local trt=(libnvinfer10 libnvinfer-plugin10 libnvonnxparsers10 libnvinfer-dev libnvinfer-plugin-dev
    libnvinfer-headers-dev libnvinfer-headers-plugin-dev libnvonnxparsers-dev)
  run sudo apt-get install -y "${trt[@]/%/=$TRT_VERSION}"
  # held: an apt upgrade must not move TensorRT away from the version the engines were built with
  run sudo apt-mark hold "${trt[@]}"
}

# ------------------------------------------------------------------------------------------------------------- sources
clone_attempt() {  # <url> <dest> <sparse dirs>
  rm -rf "$2"
  if [ -n "$3" ]; then git_cmd clone --filter=blob:none --no-checkout "$1" "$2"; else git_cmd clone --no-checkout "$1" "$2"; fi
}

clone_repo() {  # <entry of REPOS>
  local dir path ref access sparse dest url tmp
  IFS='|' read -r dir path ref access sparse <<<"$1"
  dest="$WS/$dir"
  if [ -e "$dest" ]; then
    # a submodule has a .git file, a clone a .git folder
    if [ -e "$dest/.git" ]; then
      info "$dir already there ($(git -C "$dest" rev-parse --abbrev-ref HEAD) $(git -C "$dest" rev-parse --short HEAD)), left as it is"
    else
      info "$dir already there, left as it is"
    fi
    return
  fi
  url=$(repo_url "$path" "$access")
  tmp="$dest.partial"
  info "$dir: $url at $ref"
  if [ "$DRY_RUN" = 1 ]; then echo "+ git clone $url $dest; git checkout $ref${sparse:+ (sparse: $sparse)}"; return; fi
  retry clone_attempt "$url" "$tmp" "$sparse"
  # shellcheck disable=SC2086
  [ -z "$sparse" ] || git -C "$tmp" sparse-checkout set $sparse
  git_cmd -C "$tmp" checkout --quiet "$ref"
  if [ -f "$tmp/.gitmodules" ]; then retry git_cmd -C "$tmp" submodule update --init --recursive; fi
  mv "$tmp" "$dest"
}

sparse_of() {  # <folder in the workspace>: the sparse checkout REPOS gives it (empty: the whole repository)
  local r dir path ref access sparse
  for r in "${REPOS[@]}"; do
    IFS='|' read -r dir path ref access sparse <<<"$r"
    if [ "$dir" = "$1" ]; then echo "$sparse"; return; fi
  done
}

# The submodules of the workspace come at the commits it records. A clone without --recurse-submodules leaves their
# folders empty, and an empty folder would pass for "already there" in clone_repo. Only the ones not checked out yet
# are touched, at any depth (legged_control's opti_pessi_* too): one already checked out (on a branch, with local
# changes...) is left as it is. A newly checked out one gets the sparse checkout of its REPOS entry.
init_submodules() {
  [ -f "$WS/.gitmodules" ] && [ -e "$WS/.git" ] || return 0
  local paths p top rel sparse
  mapfile -t paths < <(git -C "$WS" submodule status --recursive | awk '/^-/ {print $2}')
  if ((${#paths[@]} == 0)); then info "no submodule left to check out"; return; fi
  for p in "${paths[@]}"; do
    # the repository that records it: the workspace, or the submodule a nested one belongs to
    top=$(git -C "$WS/$(dirname "$p")" rev-parse --show-toplevel)
    rel="$WS/$p"; rel=${rel#"$top"/}
    sparse=$(sparse_of "$p")
    info "submodule $p${sparse:+ (sparse: $sparse)}"
    if [ "$DRY_RUN" = 1 ]; then echo "+ git -C $top submodule update --init --recursive -- $rel"; continue; fi
    retry git_cmd -C "$top" submodule update --init --recursive -- "$rel"
    # shellcheck disable=SC2086
    [ -z "$sparse" ] || git -C "$WS/$p" sparse-checkout set $sparse
  done
}

cmd_sources() {
  command -v git >/dev/null || die "git missing: ./setup_ranav.sh deps"
  mkdir -p "$WS/src" "$WS/third_party"
  init_submodules
  local r
  for r in "${REPOS[@]}"; do clone_repo "$r"; done
  if [ "$CAMERA" = 1 ]; then
    for r in "${CAMERA_REPOS[@]}"; do clone_repo "$r"; done
    # header-only for camera_perception: its own CMakeLists builds only demos
    local yolo="$WS/third_party/YOLOs-CPP"
    [ -e "$yolo/COLCON_IGNORE" ] || run touch "$yolo/COLCON_IGNORE"
    if [ ! -d "$yolo/$ORT_NAME" ]; then
      info "ONNX Runtime 1.20.1 (GPU build) for the camera detector"
      run curl -fL --retry 3 -o "$yolo/$ORT_NAME.tgz" "$ORT_URL"
      run tar -xzf "$yolo/$ORT_NAME.tgz" -C "$yolo"
      run rm -f "$yolo/$ORT_NAME.tgz"
    fi
  fi
  write_livox_msgs
  configure_rslidar
  [ "$ARCH" = aarch64 ] && arm64_binaries
  # SC-PGO writes its keyframes and maps here (the folder is ignored by git)
  [ -d "$SAVE_DATA" ] || run mkdir -p "$SAVE_DATA"
  check_fixes
}

# rslidar_sdk sets its point type with a plain set() and the IMU parse off: the Airy needs XYZIRT (ring and time per
# point, what FAST-LIO and the simulated Airy use) and its IMU (/rslidar_imu_data feeds FAST-LIO). Edited in the clone.
configure_rslidar() {
  local f="$WS/third_party/rslidar_sdk/CMakeLists.txt"
  [ -f "$f" ] || return 0
  if grep -qx 'set(POINT_TYPE XYZIRT)' "$f" && grep -qE '^option\(ENABLE_IMU_DATA_PARSE .* ON\)$' "$f"; then return; fi
  info "rslidar_sdk: POINT_TYPE XYZIRT, ENABLE_IMU_DATA_PARSE ON"
  run sed -i -e 's/^set(POINT_TYPE XYZI)$/set(POINT_TYPE XYZIRT)/' \
    -e 's/^option(ENABLE_IMU_DATA_PARSE \(.*\) OFF)$/option(ENABLE_IMU_DATA_PARSE \1 ON)/' "$f"
  [ "$DRY_RUN" = 1 ] || { grep -qx 'set(POINT_TYPE XYZIRT)' "$f" && grep -qE '^option\(ENABLE_IMU_DATA_PARSE .* ON\)$' "$f"; } ||
    die "could not set the point type / IMU parse in $f: edit it by hand"
}

download_checked() {  # <url> <sha256> <file>
  [ -f "$3" ] && echo "$2  $3" | sha256sum -c --status && return 0
  run curl -fL --retry 3 -o "$3.part" "$1"
  [ "$DRY_RUN" = 1 ] && return 0
  echo "$2  $3.part" | sha256sum -c --status || { rm -f "$3.part"; die "checksum of $1 does not match"; }
  mv "$3.part" "$3"
}

# x86 binaries in the repositories, replaced by their arm64 builds on the Jetson
arm64_binaries() {
  local lib="$WS/$UNITREE_HW_DIR/lib/libunitree_legged_sdk_arm64_3_3_1.so"
  if [ -d "$WS/$UNITREE_HW_DIR/lib" ]; then
    info "Unitree SDK 3.3.1 for arm64"
    download_checked "$UNITREE_ARM64_URL" "$UNITREE_ARM64_SHA" "$lib"
  fi
  local d="$WS/$ORTOOLS_DIR"
  if [ -d "$d/lib" ] && ! file -L "$d/lib/libortools.so" 2>/dev/null | grep -q aarch64; then
    info "TARE: OR-Tools 9.8 for arm64 in place of the bundled x86 one"
    local tmp; tmp=$(setup_tmpdir or-tools)
    download_checked "$ORTOOLS_ARM64_URL" "$ORTOOLS_ARM64_SHA" "$tmp/ortools.tar.gz"
    run tar -xzf "$tmp/ortools.tar.gz" -C "$tmp"
    if [ "$DRY_RUN" != 1 ]; then
      local top; top=$(find "$tmp" -mindepth 1 -maxdepth 1 -type d -name 'or-tools_*' | head -1)
      [ -f "$top/lib/libortools.so.9.8.3296" ] || die "unexpected OR-Tools archive layout ($top)"
      rm -rf "${d:?}/include" "${d:?}/lib"
      mv "$top/include" "$top/lib" "$d/"
    fi
    cleanup_tmpdir "$tmp"
  fi
}

# The fixes listed at the top of this file, in our repositories: a fresh clone has them only once they are pushed.
check_fixes() {
  [ "$DRY_RUN" = 1 ] && return 0
  local missing=() hw="$WS/$UNITREE_HW_DIR"
  grep -qs ARMV8A_ARM_CORTEX_A57 "$WS/third_party/ocs2/ocs2_sqp/blasfeo_catkin/CMakeLists.txt" || missing+=("ocs2: blasfeo_catkin ARM target")
  grep -qs 'TARGET GENERIC' "$WS/third_party/ocs2/ocs2_sqp/hpipm_catkin/CMakeLists.txt" || missing+=("ocs2: hpipm_catkin ARM target")
  grep -qs UNITREE_ARCH "$hw/CMakeLists.txt" || missing+=("legged_control: legged_unitree_hw SDK per architecture")
  grep -qs 'UNITREE_LEGGED_SDK::FL_, UNITREE_LEGGED_SDK::RL_' "$hw/src/UnitreeHW.cpp" || missing+=("legged_control: UnitreeHW /contact foot order")
  [ -f "$hw/launch/real_robot_launch.xml" ] || missing+=("legged_control: legged_unitree_hw/launch/real_robot_launch.xml")
  # shellcheck disable=SC2016  # literal xacro text
  grep -qs 'UnitreeHW_$(arg unitree_sdk)' "$WS/src/legged_control/legged_examples/legged_unitree/legged_unitree_description/urdf/common/gazebo.xacro" ||
    missing+=("legged_control: gazebo.xacro real hardware plugin")
  ((${#missing[@]})) || return 0
  warn "the workspace lacks fixes this script relies on (were they pushed?):"
  printf '    %s\n' "${missing[@]}" >&2
  if [ "$ARCH" = aarch64 ]; then die "the Jetson build fails without them"; fi
  warn "the simulation builds without them; the real robot (start --real) does not run"
}

# fast_lio needs livox_ros_driver2/msg/CustomMsg to build; the real driver also needs Livox-SDK2, which the simulated
# RoboSense Airy (PointCloud2) does not. This package holds only the two messages.
write_livox_msgs() {
  local d="$WS/third_party/livox_ros_driver2"
  [ -e "$d" ] && return
  info "third_party/livox_ros_driver2: messages only"
  write_file "$d/package.xml" <<'EOF'
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>livox_ros_driver2</name>
  <version>1.0.0</version>
  <description>Message definitions of the Livox ROS 2 driver (CustomMsg, CustomPoint) without the driver itself</description>
  <maintainer email="alejandro.enrique.barbi@gmail.com">abarbi</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <depend>std_msgs</depend>

  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
EOF
  write_file "$d/CMakeLists.txt" <<'EOF'
cmake_minimum_required(VERSION 3.8)
project(livox_ros_driver2)

# Message definitions only, copied from https://github.com/Livox-SDK/livox_ros_driver2.
# fast_lio needs livox_ros_driver2/msg/CustomMsg to build; the real driver also needs
# Livox-SDK2, which is not needed here (the simulated lidar publishes PointCloud2).
# Replace this package with the real driver if a Livox lidar is ever used.

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CustomPoint.msg"
  "msg/CustomMsg.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
EOF
  write_file "$d/msg/CustomMsg.msg" <<'EOF'
# Livox publish pointcloud msg format.

std_msgs/Header header    # ROS standard message header
uint64 timebase           # The time of first point
uint32 point_num          # Total number of pointclouds
uint8  lidar_id           # Lidar device id number
uint8[3]  rsvd            # Reserved use
CustomPoint[] points      # Pointcloud data
EOF
  write_file "$d/msg/CustomPoint.msg" <<'EOF'
# Livox costom pointcloud format.

uint32 offset_time      # offset time relative to the base time
float32 x               # X axis, unit:m
float32 y               # Y axis, unit:m
float32 z               # Z axis, unit:m
uint8 reflectivity      # reflectivity, 0~255
uint8 tag               # livox tag
uint8 line              # laser number in lidar
EOF
}

# ---------------------------------------------------------------------------------------------------------------- venv
# FAST_LIO_LOCALIZATION and makeMergedMap.py (end of an exploration) run with this python: they need open3d; the CPU
# people detector (lidar_perception centerpoint_cpu.py) needs onnxruntime. It sees the system packages (rclpy...).
# Its numpy is 2.x, which breaks the apt scipy/matplotlib built for numpy 1: the venv gets its own copies of those.
cmd_venv() {
  if [ ! -x "$VENV/bin/python" ]; then
    info "python venv $VENV"
    run python3 -m venv --system-site-packages "$VENV"
  fi
  if [ ! -x "$VENV/bin/python" ] || ! "$VENV/bin/python" -c 'import open3d, onnxruntime' >/dev/null 2>&1; then
    run "$VENV/bin/python" -m pip install open3d==0.20.0 onnxruntime==1.30.0 scipy matplotlib
  fi
  # open3d 0.20 has no open3d.__version__: ask the installed metadata
  [ "$DRY_RUN" = 1 ] || "$VENV/bin/python" -c 'import numpy, open3d, onnxruntime; from importlib.metadata import version; print("venv: numpy", numpy.__version__, "open3d", version("open3d"), "onnxruntime", onnxruntime.__version__)'
}

# --------------------------------------------------------------------------------------------------------------- model
# The pretrained Autoware model (automotive data: a functional test on the Airy indoors, not a tuned detector). The
# TensorRT engines are built next to it on the first start with people:=true (about 2 minutes, for the GPU at hand).
cmd_model() {
  [ "$DRY_RUN" = 1 ] || mkdir -p "$MODEL_DIR"
  local entry file sum
  for entry in "${MODEL_FILES[@]}"; do
    read -r file sum <<<"$entry"
    if [ -f "$MODEL_DIR/$file" ] && echo "$sum  $MODEL_DIR/$file" | sha256sum -c --status; then continue; fi
    info "model: $file"
    run curl -fsSL --retry 3 -o "$MODEL_DIR/$file.part" "$MODEL_URL/$file"
    if [ "$DRY_RUN" != 1 ]; then
      echo "$sum  $MODEL_DIR/$file.part" | sha256sum -c --status || { rm -f "$MODEL_DIR/$file.part"; die "checksum of $file does not match"; }
      mv "$MODEL_DIR/$file.part" "$MODEL_DIR/$file"
    fi
  done
  info "CenterPoint model in $MODEL_DIR"
}

# ----------------------------------------------------------------------------------------------------------------- env
write_env() {
  if [ ! -f "$FASTDDS_PROFILE" ]; then
    # Point clouds of 1-2 MB between processes: the default 512 KB shared-memory segment of Fast DDS makes their
    # delivery slow and irregular (13 ms median, seconds under load); 16 MB brings it to 1-4 ms.
    write_file "$FASTDDS_PROFILE" <<'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<!-- Fast DDS profile for a stack that passes point clouds of 1-2 MB between processes on one machine: a 16 MB
     shared-memory segment instead of the default 512 KB (1-4 ms a hop instead of 13 ms, seconds under load).
     Use: export FASTRTPS_DEFAULT_PROFILES_FILE=<this file> in every terminal, before any ros2 launch / ros2 run. -->
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>shm_16mb</transport_id>
      <type>SHM</type>
      <segment_size>16777216</segment_size>
    </transport_descriptor>
    <transport_descriptor>
      <transport_id>udp</transport_id>
      <type>UDPv4</type>
    </transport_descriptor>
  </transport_descriptors>
  <participant profile_name="large_clouds" is_default_profile="true">
    <rtps>
      <userTransports>
        <transport_id>shm_16mb</transport_id>
        <transport_id>udp</transport_id>
      </userTransports>
      <useBuiltinTransports>false</useBuiltinTransports>
    </rtps>
  </participant>
</profiles>
EOF
  fi
  write_file "$ENV_FILE" <<EOF
# Environment of the RaNAV workspace, written by setup_ranav.sh: source it in every terminal that runs the stack.
# RANAV_ROS_ONLY=1 leaves the workspace overlay out (setup_ranav.sh builds that way).
source $ROS_SETUP
if [ -d $CUDA_DIR/bin ]; then
  case ":\$PATH:" in *":$CUDA_DIR/bin:"*) ;; *) export PATH=$CUDA_DIR/bin:\$PATH ;; esac
  case ":\${LD_LIBRARY_PATH:-}:" in *":$CUDA_DIR/lib64:"*) ;; *) export LD_LIBRARY_PATH=$CUDA_DIR/lib64\${LD_LIBRARY_PATH:+:\$LD_LIBRARY_PATH} ;; esac
fi
export FASTRTPS_DEFAULT_PROFILES_FILE=$FASTDDS_PROFILE
export ROS_LOG_DIR=$WS/log/ros
export ACADOS_SOURCE_DIR=$WS/third_party/acados
export ACADOS_STATIC_DIR=$WS/install/acados_static
if [ -z "\${RANAV_ROS_ONLY:-}" ] && [ -f $WS/install/setup.bash ]; then source $WS/install/setup.bash; fi
EOF
}

cmd_env() {
  write_env
  local line="source $ENV_FILE"
  if [ "$BASHRC" = 1 ]; then
    echo "Add this line manually to ~/.bashrc if you want new shells to source the RaNAV environment:"
    echo "  $line"
  else
    echo "In every terminal (or add this line manually to ~/.bashrc):"
    echo "  $line"
  fi
}

load_env() {  # [ros]: ROS (+ CUDA, Fast DDS profile) only, without the workspace overlay
  [ -f "$ROS_SETUP" ] || die "ROS 2 Jazzy missing: ./setup_ranav.sh deps"
  [ -f "$ENV_FILE" ] || [ "$DRY_RUN" = 1 ] || write_env
  # shellcheck disable=SC1090
  if [ ! -f "$ENV_FILE" ]; then source "$ROS_SETUP"
  elif [ "${1:-}" = ros ]; then RANAV_ROS_ONLY=1 source "$ENV_FILE"
  else source "$ENV_FILE"; fi
  check_ld_path
}

# A libhpipm.so ahead of the workspace's (acados ships a newer HPIPM) is loaded by ocs2's hpipm_catkin, which has no
# RPATH: gzserver then segfaults at the first MPC solve (d_ocp_qp_dim_set_all).
check_ld_path() {
  local d dirs
  IFS=: read -ra dirs <<<"${LD_LIBRARY_PATH:-}"
  for d in "${dirs[@]}"; do
    if [ -n "$d" ] && [ -e "$d/libhpipm.so" ] && [[ "$d" != "$WS/install/"* ]]; then
      die "LD_LIBRARY_PATH holds $d/libhpipm.so (acados?): it shadows ocs2's HPIPM and the controller crashes. Use a terminal without it."
    fi
  done
}

# --------------------------------------------------------------------------------------------------------------- build
build_targets_line() {
  local t=("${BUILD_TARGETS[@]}")
  [ "$CAMERA" = 1 ] && t+=(camera_perception)
  echo "${t[*]}"
}

cmd_build() {
  load_env ros
  installed ros-jazzy-pinocchio && { check_pinocchio; installed ros-jazzy-pinocchio && die "ros-jazzy-pinocchio still installed"; }
  [ -f "$PINOCCHIO_DIR/pinocchioConfig.cmake" ] || die "pinocchio missing ($PINOCCHIO_DIR): ./setup_ranav.sh deps"
  [ -d "$WS/src/legged_control" ] && [ -d "$WS/third_party/rslidar_sdk" ] || die "sources missing: ./setup_ranav.sh sources"
  configure_rslidar
  check_fixes
  cd "$WS"

  local workers
  compute_jobs
  build_acados_static
  workers=$((JOBS >= 4 ? 2 : 1))
  export MAKEFLAGS="-j$((JOBS / workers))"

  # Release everywhere: one package left with an empty build type (-O0) is enough for its weak Eigen/CppAD symbols to
  # shadow the optimized ones in the controller process, and the MPC solve gets ~40x slower. pinocchio is the one in
  # /opt/openrobots (robotpkg, or built from source on aarch64), not on CMAKE_PREFIX_PATH.
  # Pinocchio's robotpkg config declares CasADi and Coal as dependencies even for consumers that do not use them.
  # Use its hpp-fcl compatibility package too; mixing it with ROS's legacy hpp/fcl headers conflicts with Coal.
  # Keep /opt/openrobots off CMAKE_PREFIX_PATH (it can shadow ROS/system libraries), and pass package dirs directly.
  local cmake_args=(-DCMAKE_BUILD_TYPE=Release "-Dpinocchio_DIR=$PINOCCHIO_DIR")
  if [ -f "$CASADI_CMAKE_DIR/casadi-config.cmake" ]; then
    cmake_args+=("-Dcasadi_DIR=$CASADI_CMAKE_DIR")
  fi
  if [ -f "$COAL_CMAKE_DIR/coalConfig.cmake" ]; then
    cmake_args+=("-Dcoal_DIR=$COAL_CMAKE_DIR")
  fi
  if [ -f "$HPP_FCL_CMAKE_DIR/hpp-fclConfig.cmake" ]; then
    cmake_args+=("-Dhpp-fcl_DIR=$HPP_FCL_CMAKE_DIR")
  fi
  local common=(--parallel-workers "$workers" --cmake-args "${cmake_args[@]}")
  local skip=()
  if [ "$NO_CUDA" = 1 ] || ! have_cuda; then
    [ "$NO_CUDA" = 1 ] || warn "CUDA / TensorRT not found ($CUDA_DIR): the people detector is not built (./setup_ranav.sh cuda)"
    # ignore, not skip: a skipped package is still a dependency whose install/<pkg>/share/<pkg>/package.sh colcon needs
    # (lidar_perception execs autoware_lidar_centerpoint); an ignored one is left out of the graph altogether
    skip=(--packages-ignore "${CUDA_PACKAGES[@]}")
  fi
  local targets; read -ra targets <<<"$(build_targets_line)"
  info "colcon build ($workers packages at a time, MAKEFLAGS=$MAKEFLAGS): ${targets[*]} and their dependencies"

  # First build only: opti_pessi_interface and legged_controllers are symlink-installed, so edits of the controller's
  # config files and of opti_pessi_scenario.py / kill_node_sigint_service.py need no rebuild. Every other package is
  # a copy install. A package keeps the install mode it was first built with: never pass --symlink-install to a
  # package built without it (the message packages then fail with "failed to create symbolic link ... Is a directory").
  if [ ! -d "$WS/build/opti_pessi_interface" ]; then
    run colcon build --packages-up-to legged_controllers --packages-skip opti_pessi_interface legged_controllers "${common[@]}"
    run colcon build --packages-select opti_pessi_interface legged_controllers --symlink-install "${common[@]}"
  fi
  run colcon build --packages-up-to "${targets[@]}" "${skip[@]}" "${common[@]}"
  [ "$DRY_RUN" = 1 ] || write_env
  info "built. Every terminal: source $ENV_FILE   then: ./setup_ranav.sh start exploration"
}

cmd_install() {
  IN_INSTALL=1
  cmd_deps
  cmd_cuda
  cmd_sources
  cmd_rosdep
  cmd_venv
  cmd_model  # also the CPU detector's (ONNX)
  cmd_build
  cmd_env
  cat <<EOF

Installed. Next:
  ./setup_ranav.sh start exploration     explores and saves the map + FAR graph in src/map_generator/save_data
  ./setup_ranav.sh start inspection      needs that map; points picked on http://localhost:8765
  ./setup_ranav.sh stop
EOF
  if [ "$NO_CUDA" != 1 ] && have_cuda; then echo "The first start with people takes ~2 more minutes: TensorRT builds its engines."; fi
}

# --------------------------------------------------------------------------------------------------------------- start
TMUX_ENV=()
WINDOWS=()

# One tmux window per launch, as in README.md (one terminal each): Ctrl-C and the up arrow restart it there.
win() {  # <name> <command>
  if ((${#WINDOWS[@]} == 0)); then
    tmux new-session -d -s "$SESSION" -n "$1" -c "$WS" "${TMUX_ENV[@]}"
  else
    tmux new-window -t "$SESSION:" -n "$1" -c "$WS"
  fi
  WINDOWS+=("$1")
  tmux send-keys -t "$SESSION:$1" -l "source '$ENV_FILE' && $2"
  tmux send-keys -t "$SESSION:$1" Enter
  printf '  [%s] %s\n' "$1" "$2"
}

# A window with a command typed and not run: Enter in that window runs it.
typed() {  # <name> <command>
  tmux new-window -t "$SESSION:" -n "$1" -c "$WS"
  tmux send-keys -t "$SESSION:$1" -l "source '$ENV_FILE' && clear"
  tmux send-keys -t "$SESSION:$1" Enter
  tmux send-keys -t "$SESSION:$1" -l "$2"
  printf '  [%s] typed, Enter there runs it: %s\n' "$1" "$2"
}

wait_for() {  # <seconds> <what> <command...>
  local end=$((SECONDS + $1)) what=$2
  shift 2
  printf '  waiting for %s ' "$what"
  until "$@" >/dev/null 2>&1; do
    if ((SECONDS >= end)); then echo " timed out"; return 1; fi
    printf .; sleep 3
  done
  echo " ok"
}
controller_manager_up() { ros2 service list --no-daemon | grep -qx /controller_manager/list_controllers; }
controller_active() {
  ros2 control list_controllers 2>/dev/null | sed 's/\x1b\[[0-9;]*m//g' | grep -E '^legged_controller[[:space:]]' | grep -qw active
}
# ros2 topic echo --timeout exits 0 when nothing came: look for the stamp instead. SIGINT, not SIGTERM, so that the
# probe frees its Fast DDS shared memory.
topic_has_message() {  # <topic> <type>
  timeout -s INT -k 5 10 ros2 topic echo --once "$1" "$2" --field header.stamp.sec 2>/dev/null | grep -q '[0-9]'
}

stuck() {  # <what>
  echo
  warn "$1. The session keeps running: ./setup_ranav.sh attach to read the windows, ./setup_ranav.sh stop to close it."
  exit 1
}

# Fast DDS shared-memory files left by processes killed with SIGKILL make the nodes that start together block each
# other (Gazebo silent, the robot never spawns). A segment fastrtps_<id> comes with fastrtps_<id>_el / _sl and
# sem.fastrtps_<id>_mutex: the files of a segment that no running process maps are removed, the others stay.
clean_stale_shm() {
  compgen -G "/dev/shm/*fastrtps_*" >/dev/null || return 0
  ros2 daemon stop >/dev/null 2>&1 || true
  local mapped f key n=0
  mapped=$(cat /proc/[0-9]*/maps 2>/dev/null | grep -o '/dev/shm/[^ ]*fastrtps_[^ ]*' |
    sed 's|.*/||; s/^sem\.//; s/_el$//; s/_sl$//; s/_mutex$//' | sort -u) || true
  for f in /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*; do
    [ -e "$f" ] || continue
    key=${f##*/}; key=${key#sem.}; key=${key%_el}; key=${key%_sl}; key=${key%_mutex}
    grep -qxF "$key" <<<"$mapped" && continue
    rm -f "$f" 2>/dev/null && n=$((n + 1))  # other users' files stay
  done
  ((n == 0)) || info "removed $n stale Fast DDS shared-memory files from /dev/shm"
}

start_sim() {  # <world launch>
  if [ "$REAL" = 1 ]; then start_robot; return; fi
  win sim "ros2 launch legged_unitree_description $1 server:=$([ "$GUI" = 1 ] && echo false || echo true)"
  wait_for 180 "Gazebo and the controller manager" controller_manager_up ||
    stuck "no controller manager after 3 minutes. If the sim window is silent, stale /dev/shm/fastrtps_* files are the usual cause: stop everything, then rm /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*"
}

confirm() {  # <question>: the user types yes, or nothing goes further
  local answer
  [ -r /dev/tty ] || die "--real asks before moving the robot: run it from a terminal"
  read -r -p "$1 Type yes: " answer </dev/tty
  [ "$answer" = yes ] || stuck "not confirmed, nothing else started"
}

# The robot in place of Gazebo: Unitree low-level UDP (legged_unitree_hw) + controller manager. The Airy driver comes
# later, with the ranav_bringup launch (lidar:=real).
start_robot() {
  local robot_ip=192.168.123.10 plugin="$WS/install/legged_unitree_hw/lib/liblegged_unitree_hw_3_3_1.so"
  local hw_launch="$WS/install/legged_unitree_hw/share/legged_unitree_hw/launch/real_robot_launch.xml"
  [ -e "$plugin" ] || die "Unitree hardware plugin not built ($plugin): ./setup_ranav.sh build"
  [ -e "$hw_launch" ] || die "no legged_unitree_hw real_robot_launch.xml: one of the real robot changes legged_control lacks (./setup_ranav.sh --help)"
  ping -c 1 -W 1 "$robot_ip" >/dev/null 2>&1 ||
    die "robot not reachable at $robot_ip: this computer needs an address on 192.168.123.0/24 on the cable to the robot"
  cat <<EOF

Real robot. The hardware plugin sends zero gains until the controller is active (motors limp); when the controller
activates it stands the robot up. Robot lying on flat ground or hung up, nobody next to it, remote at hand.
FAST-LIO uses ranav_bringup config/fast_lio_airy_real.yaml: check its imu_R / extrinsic_R against the real mount
before trusting the map. The Airy driver reads ranav_bringup config/rslidar_airy.yaml.
EOF
  win robot "ros2 launch legged_unitree_hw real_robot_launch.xml"
  wait_for 60 "the controller manager" controller_manager_up || stuck "no controller manager"
  confirm "Start the controller: the robot stands up."
}

start_controller() {  # <recompile true|false>
  win controller "ros2 launch legged_controllers load_optipessicontroller_launch.xml rviz:=false recompile:=$1"
  wait_for 600 "legged_controller active (solver libraries generated first)" controller_active || stuck "legged_controller not active"
  # FAST-LIO takes gravity and its frame from the first IMU samples: the robot must stand still before it starts
  echo "  robot standing up: $STAND_WAIT s"
  sleep "$STAND_WAIT"
}

# ranav_bringup launches: lidar:=sim|real (use_sim_time follows it); people, detector and centerpoint_model reach the
# included fast_lio_slam_launch.xml from the command line
bringup() {  # <window> <launch file> <extra launch arguments>
  win "$1" "ros2 launch ranav_bringup $2 lidar:=$LIDAR people:=$PEOPLE detector:=$DETECTOR centerpoint_model:=$MODEL_DIR $3"
  wait_for 180 "FAST-LIO (/Odometry)" topic_has_message /Odometry nav_msgs/msg/Odometry || stuck "no /Odometry"
}

cmd_start() {
  local what=${ARGS[0]:-}
  [ -n "$what" ] || die "start what? exploration | inspection | scenario <1-4>"
  command -v tmux >/dev/null || die "tmux missing: sudo apt-get install tmux"
  [ -f "$WS/install/setup.bash" ] || die "not built: ./setup_ranav.sh build"
  load_env
  tmux has-session -t "$SESSION" 2>/dev/null && die "session $SESSION already running: ./setup_ranav.sh attach, or ./setup_ranav.sh stop"
  { pgrep -x gzserver >/dev/null || pgrep -f 'bin/gz sim' >/dev/null; } && warn "a Gazebo is already running outside the session"
  clean_stale_shm

  local v
  for v in ROS_DOMAIN_ID GZ_PARTITION RMW_IMPLEMENTATION ROS_AUTOMATIC_DISCOVERY_RANGE ROS_LOCALHOST_ONLY DISPLAY WAYLAND_DISPLAY CUDA_VISIBLE_DEVICES; do
    [ -n "${!v:-}" ] && TMUX_ENV+=(-e "$v=${!v}")
  done
  # The TensorRT detector when it is built (CUDA), else the same model on the CPU (ONNX Runtime, +-12.8 m)
  if [ "$PEOPLE" = true ] && [ "$DETECTOR" = centerpoint ] &&
    [ ! -e "$WS/install/autoware_lidar_centerpoint/lib/libautoware_lidar_centerpoint_component.so" ]; then
    warn "TensorRT people detector not built (no CUDA / TensorRT): the CPU one, detector:=centerpoint_cpu"
    DETECTOR=centerpoint_cpu
  fi
  if [ "$PEOPLE" = true ] && [ "$DETECTOR" = centerpoint_cpu ] &&
    ! "$VENV/bin/python" -c 'import onnxruntime' >/dev/null 2>&1; then
    warn "no onnxruntime in $VENV (./setup_ranav.sh venv): people:=false"
    PEOPLE=false
  fi
  if [ "$PEOPLE" = true ] && [ ! -f "$MODEL_DIR/pts_voxel_encoder.onnx" ]; then
    warn "CenterPoint model missing ($MODEL_DIR, ./setup_ranav.sh model): people:=false"
    PEOPLE=false
  fi
  case "$WORLD" in warehouse | empty_room) ;; *) die "--world: warehouse or empty_room" ;; esac
  LIDAR=sim
  if [ "$REAL" = 1 ]; then
    [ "$what" != scenario ] || die "scenarios are simulation only"
    LIDAR=real
  fi

  info "tmux session $SESSION (./setup_ranav.sh attach to see it)"
  case "$what" in
    exploration)
      start_sim "${WORLD}_launch.xml"
      start_controller "${RECOMPILE:-true}"
      # FAST-LIO + SC-PGO + people perception, TARE, the goal bridge, one rviz; SC-PGO writes into $SAVE_DATA
      bringup explore explore_warehouse_launch.xml "auto_start:=$AUTO_START"
      [ "$AUTO_START" = true ] || typed go "ros2 topic pub -1 /start_exploration std_msgs/msg/Bool '{data: true}'"
      cat <<EOF

Exploring once /start_exploration arrives (window go, or --auto-start). When TARE is done the robot walks back to its
start; once there the map is merged into $SAVE_DATA/optimized_map.pcd and the launch shuts down.
EOF
      ;;
    inspection)
      [ -f "$SAVE_DATA/optimized_map.pcd" ] ||
        die "no map: $SAVE_DATA/optimized_map.pcd comes from an exploration of the same world"
      start_sim "${WORLD}_launch.xml"
      start_controller "${RECOMPILE:-true}"
      # FAST-LIO + people (no SC-PGO: it would empty save_data/Scans), localization, far_planner, the goal bridge, the
      # map point picker, one rviz
      bringup inspect inspection_launch.xml "auto_start:=$AUTO_START"
      cat <<EOF

Map point picker: http://localhost:8765. Pick the points, then Start inspection (or, without the picker:
ros2 topic pub -1 /start_inspection std_msgs/msg/String "{data: ''}" for the points of save_data/selected_points.yaml).
The robot must stand where the exploration started: the map is used in the frame FAST-LIO had then.
EOF
      ;;
    scenario)
      local n=${ARGS[1]:-}
      [[ "$n" =~ ^[1-4]$ ]] || die "scenario 1-4 (1 static obstacle, 2 patrol, 3 circle round the goal, 4 antagonist)"
      if [ "$PERSON" = 1 ] && [ "$PEOPLE" != true ]; then die "--person needs the people detector"; fi
      start_sim scenario_room_launch.xml
      start_controller "${RECOMPILE:-false}"
      if [ "$PERSON" = 1 ]; then
        # FAST-LIO publishes odom -> lio_map itself (alignment.* of its config), which --person needs
        bringup slam fast_lio_slam_launch.xml "pgo:=false rviz:=false"
        wait_for 400 "the people detector (TensorRT engines are built on the first start)" \
          topic_has_message /perception/detector/objects autoware_perception_msgs/msg/DetectedObjects ||
          stuck "no /perception/detector/objects"
        win scenario "ros2 run opti_pessi_interface opti_pessi_scenario.py $n --person"
      else
        # the obstacle goes straight to /opti_pessi/obstacles: no people pipeline next to it
        win scenario "ros2 run opti_pessi_interface opti_pessi_scenario.py $n"
      fi
      ;;
    *) die "start: exploration | inspection | scenario <1-4>" ;;
  esac
  echo
  info "running. ./setup_ranav.sh attach to see the windows, ./setup_ranav.sh stop to end"
}

pane_busy() { local p; p=$(tmux display-message -p -t "$1" '#{pane_pid}' 2>/dev/null) && pgrep -P "$p" >/dev/null; }

cmd_stop() {
  if ! tmux has-session -t "$SESSION" 2>/dev/null; then
    info "no session $SESSION"
  else
    local w names i end
    mapfile -t names < <(tmux list-windows -t "$SESSION" -F '#{window_name}')
    # last started first: planners, then SLAM, controller, simulation
    for ((i = ${#names[@]} - 1; i >= 0; i--)); do
      w="$SESSION:${names[i]}"
      pane_busy "$w" || continue
      printf '  stopping %s ' "${names[i]}"
      tmux send-keys -t "$w" C-c
      end=$((SECONDS + 30))
      while pane_busy "$w" && ((SECONDS < end)); do printf .; sleep 1; done
      if pane_busy "$w"; then echo " still running, closed with the session"; else echo " ok"; fi
    done
    tmux kill-session -t "$SESSION"
    info "session $SESSION closed"
  fi
  # shellcheck disable=SC1090
  if [ -f "$ROS_SETUP" ]; then source "$ROS_SETUP"; clean_stale_shm; fi
}

cmd_attach() {
  tmux has-session -t "$SESSION" 2>/dev/null || die "no session $SESSION"
  if [ -n "${TMUX:-}" ]; then tmux switch-client -t "$SESSION"; else tmux attach -t "$SESSION"; fi
}

usage() { sed -n '2,/^set -Eeo/{/^set -Eeo/d;s/^# \{0,1\}//;p}' "${BASH_SOURCE[0]}" || true; }

# ---------------------------------------------------------------------------------------------------------------- main
CMD=${1:-}
[ $# -gt 0 ] && shift
while [ $# -gt 0 ]; do
  case "$1" in
    --dry-run) DRY_RUN=1 ;;
    --no-cuda) NO_CUDA=1 ;;
    --camera) CAMERA=1 ;;
    --world) [ $# -ge 2 ] || die "--world needs a name"; WORLD=$2; shift ;;
    --gui) GUI=1 ;;
    --no-people) PEOPLE=false ;;
    --recompile) RECOMPILE=true ;;
    --no-recompile) RECOMPILE=false ;;
    --auto-start) AUTO_START=true ;;
    --person) PERSON=1 ;;
    --cpu-detector) DETECTOR=centerpoint_cpu ;;
    --real) REAL=1 ;;
    --bashrc) BASHRC=1 ;;
    -h | --help) usage; exit 0 ;;
    -*) die "unknown option $1 (./setup_ranav.sh --help)" ;;
    *) ARGS+=("$1") ;;
  esac
  shift
done
[[ "$GIT_PROTOCOL" =~ ^(ssh|https)$ ]] || die "GIT_PROTOCOL: ssh or https"

case "$CMD" in
  install) cmd_install ;;
  deps) cmd_deps ;;
  cuda) cmd_cuda ;;
  sources) cmd_sources ;;
  venv) cmd_venv ;;
  model) cmd_model ;;
  build) cmd_build ;;
  env) cmd_env ;;
  start) cmd_start ;;
  stop) cmd_stop ;;
  attach) cmd_attach ;;
  "" | -h | --help | help) usage ;;
  *) die "unknown command $CMD (./setup_ranav.sh --help)" ;;
esac
