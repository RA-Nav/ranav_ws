// What opti_pessi_obstacle_bridge hands to OptiPessiController for the tracks of autoware_multi_object_tracker.
#include <gtest/gtest.h>

#include <cmath>

#include "ranav_mission/opti_pessi_obstacles.hpp"

using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::TrackedObject;
using autoware_perception_msgs::msg::TrackedObjects;
using legged_controllers::msg::Obstacle;
using legged_controllers::msg::ObstacleArray;
using ranav_mission::toObstacles;

namespace {

TrackedObject track(double x, double y, uint8_t label = ObjectClassification::PEDESTRIAN) {
  TrackedObject object;
  ObjectClassification classification;
  classification.label = label;
  classification.probability = 1.0F;
  object.classification.push_back(classification);
  object.kinematics.pose_with_covariance.pose.position.x = x;
  object.kinematics.pose_with_covariance.pose.position.y = y;
  object.kinematics.pose_with_covariance.pose.position.z = 0.87;
  return object;
}

TrackedObjects tracks(std::initializer_list<TrackedObject> objects) {
  TrackedObjects msg;
  msg.header.frame_id = "camera_init";
  msg.header.stamp.sec = 10;
  msg.header.stamp.nanosec = 699999999;
  msg.objects = objects;
  return msg;
}

// camera_init at (1, 2, 0.5) in odom, turned a quarter to the left
const Eigen::Isometry3d kOdomFromCameraInit =
    Eigen::Translation3d(1.0, 2.0, 0.5) * Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ());
const Eigen::Vector2d kSensor(10.0, 5.0);

}  // namespace

TEST(ToObstacles, PedestriansAreHumansInOdomAtTheStampOfTheirTracks) {
  const TrackedObjects in = tracks({track(10.0, 7.0)});
  const ObstacleArray out = toObstacles(in, kOdomFromCameraInit, "odom", kSensor, 10.0);
  EXPECT_EQ(out.header.frame_id, "odom");  // the controller ignores any other frame
  EXPECT_EQ(out.header.stamp, in.header.stamp);
  ASSERT_EQ(out.obstacles.size(), 1u);
  EXPECT_EQ(out.obstacles[0].type, Obstacle::HUMAN);
  EXPECT_NEAR(out.obstacles[0].position.x, -6.0, 1e-9);
  EXPECT_NEAR(out.obstacles[0].position.y, 12.0, 1e-9);
  EXPECT_NEAR(out.obstacles[0].position.z, 1.37, 1e-9);
}

// The controller has no type for the cars and trucks a road model sees, and in a warehouse they are shelves.
TEST(ToObstacles, OtherClassesAreLeftOut) {
  const ObstacleArray out = toObstacles(tracks({track(10.0, 7.0, ObjectClassification::CAR),
                                                track(11.0, 6.0, ObjectClassification::TRUCK), track(9.0, 6.0)}),
                                        kOdomFromCameraInit, "odom", kSensor, 10.0);
  ASSERT_EQ(out.obstacles.size(), 1u);
  EXPECT_NEAR(out.obstacles[0].position.x, -5.0, 1e-9);  // the pedestrian at (9, 6)
  EXPECT_NEAR(out.obstacles[0].position.y, 11.0, 1e-9);
}

// Each obstacle takes a slot of the controller, and its solve time grows with them.
TEST(ToObstacles, PeopleBeyondMaxRangeOfTheSensorAreLeftOut) {
  const ObstacleArray out =
      toObstacles(tracks({track(10.0, 14.5), track(20.5, 5.0)}), kOdomFromCameraInit, "odom", kSensor, 10.0);
  ASSERT_EQ(out.obstacles.size(), 1u);  // 9.5 m and 10.5 m from the sensor
  EXPECT_NEAR(out.obstacles[0].position.x, -13.5, 1e-9);
}

// An empty array clears the obstacles of the controller.
TEST(ToObstacles, NobodyGivesAnEmptyArrayWithTheStampOfTheTracks) {
  const TrackedObjects in = tracks({});
  const ObstacleArray out = toObstacles(in, kOdomFromCameraInit, "odom", kSensor, 10.0);
  EXPECT_TRUE(out.obstacles.empty());
  EXPECT_EQ(out.header.stamp, in.header.stamp);
  EXPECT_EQ(out.header.frame_id, "odom");
}
