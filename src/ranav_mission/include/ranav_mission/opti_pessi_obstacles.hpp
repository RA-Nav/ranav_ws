// The obstacles OptiPessiController keeps out of, from the tracks of autoware_multi_object_tracker (see
// opti_pessi_obstacle_bridge.cpp for the node). No ROS graph here: messages and transforms in, a message out.
#pragma once

#include <Eigen/Geometry>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <legged_controllers/msg/obstacle_array.hpp>

#include <string>

namespace ranav_mission {

// True if the most probable class of the track is PEDESTRIAN.
bool isPedestrian(const autoware_perception_msgs::msg::TrackedObject& track);

// The pedestrians of `tracks` within max_range of `sensor` (x y in the frame of the tracks) as HUMAN obstacles, brought
// to `frame` by `to_frame` (frame <- frame of the tracks), with the stamp of the tracks. The controller has no type for
// the other classes, and takes type and position only.
legged_controllers::msg::ObstacleArray toObstacles(const autoware_perception_msgs::msg::TrackedObjects& tracks,
                                                   const Eigen::Isometry3d& to_frame, const std::string& frame,
                                                   const Eigen::Vector2d& sensor, double max_range);

}  // namespace ranav_mission
