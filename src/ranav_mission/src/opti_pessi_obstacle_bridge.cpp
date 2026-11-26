// opti_pessi_obstacle_bridge: the people autoware_multi_object_tracker tracks, as the obstacles of OptiPessiController
// (/opti_pessi/obstacles). ranav_bringup fast_lio_slam_launch.xml starts it with tracker:=autoware, in place of the
// obstacles people_tracker (lidar_perception) sends with tracker:=people_tracker.
//
//   tracks_topic     autoware_perception_msgs/TrackedObjects   the confirmed tracks of Autoware's tracker, in the
//                    FAST-LIO odometry frame (camera_init), one message per detector message, with its stamp
//   obstacles_topic  legged_controllers/ObstacleArray          the pedestrians within max_range of the sensor, as
//                    HUMAN, in obstacles_frame; one message per tracks message, an empty one included (it clears the
//                    obstacles of the controller)
//
// The controller takes type and position only (radius and maximum speed are those of its type in
// opti_pessi_interface/config/task.info), keeps the closest obstacles for its slots, and takes them in odom only: they
// are brought there with the latest TF (odom -> lio_map -> camera_init, from FAST-LIO while the legged estimator's
// /odom runs), which only follows the slow drift between the two odometries. The range is measured from sensor_frame
// (sensor_level, from lidar_perception detector_cloud_adapter) at the stamp of the tracks. Without either TF nothing
// is published, not an empty array: that would tell the controller there is nobody. Each message replaces the
// obstacles of the previous one: a second publisher of obstacles_topic erases them.

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "ranav_mission/opti_pessi_obstacles.hpp"

namespace {

Eigen::Isometry3d isometryOf(const geometry_msgs::msg::Transform& t) {
  return Eigen::Translation3d(t.translation.x, t.translation.y, t.translation.z) *
         Eigen::Quaterniond(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z);
}

class OptiPessiObstacleBridge : public rclcpp::Node {
 public:
  OptiPessiObstacleBridge() : Node("opti_pessi_obstacle_bridge") {
    const auto tracks_topic = declare_parameter("tracks_topic", std::string("/perception/autoware/tracks"));
    const auto obstacles_topic = declare_parameter("obstacles_topic", std::string("/opti_pessi/obstacles"));
    obstacles_frame_ = declare_parameter("obstacles_frame", std::string("odom"));  // the only frame the controller takes
    sensor_frame_ = declare_parameter("sensor_frame", std::string("sensor_level"));
    max_range_ = declare_parameter("max_range", 10.0);    // [m] from the sensor, as people_tracker's obstacles
    tf_timeout_ = declare_parameter("tf_timeout", 0.05);  // [s] wait for the TF of the sensor at the stamp of the tracks

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    publisher_ = create_publisher<legged_controllers::msg::ObstacleArray>(obstacles_topic, rclcpp::QoS(1));
    subscription_ = create_subscription<autoware_perception_msgs::msg::TrackedObjects>(
        tracks_topic, rclcpp::QoS(20),
        [this](autoware_perception_msgs::msg::TrackedObjects::ConstSharedPtr tracks) { onTracks(*tracks); });
  }

 private:
  void onTracks(const autoware_perception_msgs::msg::TrackedObjects& tracks) {
    Eigen::Vector2d sensor;
    Eigen::Isometry3d to_obstacles;
    try {
      const auto sensor_pose = tf_buffer_->lookupTransform(tracks.header.frame_id, sensor_frame_,
                                                           rclcpp::Time(tracks.header.stamp),
                                                           rclcpp::Duration::from_seconds(tf_timeout_));
      sensor = Eigen::Vector2d(sensor_pose.transform.translation.x, sensor_pose.transform.translation.y);
      to_obstacles = isometryOf(
          tf_buffer_->lookupTransform(obstacles_frame_, tracks.header.frame_id, tf2::TimePointZero).transform);
    } catch (const tf2::TransformException& e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No obstacles for the controller: %s", e.what());
      return;
    }
    if (count_publishers(publisher_->get_topic_name()) > 1) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                           "%s has another publisher: each message replaces the obstacles of the other one",
                           publisher_->get_topic_name());
    }
    publisher_->publish(ranav_mission::toObstacles(tracks, to_obstacles, obstacles_frame_, sensor, max_range_));
  }

  std::string obstacles_frame_, sensor_frame_;
  double max_range_ = 0.0, tf_timeout_ = 0.0;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<legged_controllers::msg::ObstacleArray>::SharedPtr publisher_;
  rclcpp::Subscription<autoware_perception_msgs::msg::TrackedObjects>::SharedPtr subscription_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OptiPessiObstacleBridge>());
  rclcpp::shutdown();
  return 0;
}
