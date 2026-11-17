cmake_minimum_required(VERSION 3.8)
project(ranav_mission)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(autoware_perception_msgs REQUIRED)

# The ObstacleArray of OptiPessiController is a message of legged_controllers. find_package(legged_controllers) would
# load the controller stack that package exports (ocs2, pinocchio, casadi): only its message targets are imported.
find_path(legged_controllers_CMAKE_DIR legged_controllers__rosidl_typesupport_cppExport.cmake
  PATH_SUFFIXES share/legged_controllers/cmake REQUIRED)
foreach(export export_legged_controllers__rosidl_generator_c export_legged_controllers__rosidl_generator_cpp
        legged_controllers__rosidl_typesupport_cpp)
  include(${legged_controllers_CMAKE_DIR}/${export}Export.cmake)
endforeach()

# The goal of OptiPessi from TARE or far_planner: the choice (no ROS) and the node
add_library(opti_pessi_goal STATIC src/opti_pessi_goal.cpp)
target_include_directories(opti_pessi_goal PUBLIC include)
target_link_libraries(opti_pessi_goal Eigen3::Eigen)

add_executable(opti_pessi_goal_bridge src/opti_pessi_goal_bridge.cpp)
target_link_libraries(opti_pessi_goal_bridge opti_pessi_goal)
ament_target_dependencies(opti_pessi_goal_bridge rclcpp geometry_msgs nav_msgs sensor_msgs visualization_msgs tf2
  tf2_ros)

# The obstacles of OptiPessi from autoware_multi_object_tracker: the conversion (no ROS graph) and the node
add_library(opti_pessi_obstacles STATIC src/opti_pessi_obstacles.cpp)
target_include_directories(opti_pessi_obstacles PUBLIC include)
target_link_libraries(opti_pessi_obstacles PUBLIC
  Eigen3::Eigen
  legged_controllers::legged_controllers__rosidl_generator_cpp
  legged_controllers::legged_controllers__rosidl_typesupport_cpp
)
ament_target_dependencies(opti_pessi_obstacles PUBLIC autoware_perception_msgs)

add_executable(opti_pessi_obstacle_bridge src/opti_pessi_obstacle_bridge.cpp)
target_link_libraries(opti_pessi_obstacle_bridge opti_pessi_obstacles)
ament_target_dependencies(opti_pessi_obstacle_bridge rclcpp tf2 tf2_ros)

install(TARGETS opti_pessi_goal_bridge opti_pessi_obstacle_bridge DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS
  scripts/tare_finish_merge_map.py
  scripts/far_inspection_goals.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_opti_pessi_goal test/test_opti_pessi_goal.cpp)
  target_link_libraries(test_opti_pessi_goal opti_pessi_goal)
  ament_add_gtest(test_opti_pessi_obstacles test/test_opti_pessi_obstacles.cpp)
  target_link_libraries(test_opti_pessi_obstacles opti_pessi_obstacles)
endif()

ament_package()
