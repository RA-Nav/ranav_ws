#include "ranav_mission/opti_pessi_obstacles.hpp"

#include <algorithm>

namespace ranav_mission {

using autoware_perception_msgs::msg::ObjectClassification;

bool isPedestrian(const autoware_perception_msgs::msg::TrackedObject& track) {
  const auto best = std::max_element(
      track.classification.begin(), track.classification.end(),
      [](const ObjectClassification& a, const ObjectClassification& b) { return a.probability < b.probability; });
  return best != track.classification.end() && best->label == ObjectClassification::PEDESTRIAN;
}

legged_controllers::msg::ObstacleArray toObstacles(const autoware_perception_msgs::msg::TrackedObjects& tracks,
                                                   const Eigen::Isometry3d& to_frame, const std::string& frame,
                                                   const Eigen::Vector2d& sensor, double max_range) {
  legged_controllers::msg::ObstacleArray out;
  out.header.stamp = tracks.header.stamp;
  out.header.frame_id = frame;
  for (const auto& track : tracks.objects) {
    const auto& p = track.kinematics.pose_with_covariance.pose.position;
    // Each obstacle takes a slot of the controller, and its solve time grows with them
    if (!isPedestrian(track) || (Eigen::Vector2d(p.x, p.y) - sensor).norm() > max_range) {
      continue;
    }
    const Eigen::Vector3d position = to_frame * Eigen::Vector3d(p.x, p.y, p.z);
    legged_controllers::msg::Obstacle obstacle;
    obstacle.type = legged_controllers::msg::Obstacle::HUMAN;
    obstacle.position.x = position.x();
    obstacle.position.y = position.y();
    obstacle.position.z = position.z();
    out.obstacles.push_back(obstacle);
  }
  return out;
}

}  // namespace ranav_mission
