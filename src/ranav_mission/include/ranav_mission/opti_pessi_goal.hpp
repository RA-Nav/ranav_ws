// The goal OptiPessiController walks to, picked from what TARE or far_planner plan (see opti_pessi_goal_bridge.cpp
// for the node). Everything here works in one frame, the goal frame (odom), and has no ROS dependency.
#pragma once

#include <Eigen/Core>

#include <optional>
#include <string>
#include <vector>

namespace ranav_mission {

using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using Path = std::vector<Vec3>;

// Projections within this distance of the nearest one are the same place on the path: a waypoint is a path vertex,
// so it touches two segments, and a looped path passes the robot twice.
constexpr double kMatchSlack = 0.15;
// Where the robot may be on the path: TARE's path starts at the viewpoint nearest the lidar, up to ~0.6 m from base.
constexpr double kRobotMatchRadius = 1.0;

// a - b [rad] wrapped to [-pi, pi)
double angleDifference(double a, double b);
// Yaw [rad] of the line origin -> target, or fallback when target is on origin
double bearing(const Vec2& origin, const Vec2& target, double fallback);
// path without the vertices that repeat the one before (TARE appends the robot or a loop's start again)
Path withoutEmptySegments(const Path& path);

// Distance from a point to each segment of a path and arc length of the projection on it, plus the arc length of
// each vertex. Segments must not be empty.
struct Projection {
  std::vector<double> distance, s, vertex_s;
};
Projection project(const Vec2& point, const Path& path);
// Point at arc length s along path
Vec3 pointAt(const Path& path, const std::vector<double>& vertex_s, double s);
// Yaw [rad] of path at arc length s, walked along it (direction 1) or back (-1): the segment s lies on; at a vertex
// the one after it in walking order, at the end of the walk the last one walked.
double pathYaw(const Path& path, const std::vector<double>& vertex_s, double s, double direction);
// Distance from point to the segment a -> b
double distanceToSegment(const Vec2& point, const Vec2& a, const Vec2& b);

struct PathGoal {
  Vec3 point;
  double yaw;  // of the path there, in walking direction
};
// Point lookahead along path from arc length start towards arc length end, end at most, pulled back to the path
// vertex it would cut past when a vertex in between lies more than max_chord_deviation off the straight line
// robot -> it.
PathGoal aheadOnPath(const Path& path, const std::vector<double>& vertex_s, double start, double end,
                     const Vec2& robot, double lookahead, double max_chord_deviation);
// Pure pursuit along a TARE path towards waypoint. The robot's projection on the path is the nearest one within
// max_waypoint_path_distance of the waypoint's along the path (a TARE path can loop back past the robot). None if the
// waypoint is not on the path.
std::optional<PathGoal> follow(const Path& path, const Vec2& robot, const Vec2& waypoint, double lookahead,
                               double waypoint_tolerance, double max_waypoint_path_distance,
                               double max_chord_deviation);
// Pure pursuit along a route that starts at the robot and ends at the goal, as far_planner's does. The robot is
// where the route is nearest; if it passes the robot again later, at the first of those places.
PathGoal pursue(const Path& route, const Vec2& robot, double lookahead, double max_chord_deviation);

// Whether the segment robot -> goal keeps clearance from obstacles. Obstacles behind the robot do not count: walking
// away from the robot does not bring it closer to them.
bool clear(const Vec2& robot, const Vec2& goal, const std::vector<Vec2>& obstacles, double clearance);
// Point at most distance (and at most as far as target) from robot, in the direction of target turned by the
// smallest multiple of step [rad] up to max_angle each side, whose segment from robot keeps clearance; or none.
std::optional<Vec2> detour(const Vec2& robot, const Vec2& target, const std::vector<Vec2>& obstacles,
                           double clearance, double distance, double step, double max_angle);
// value, then value halved while that stays above minimum, then minimum: the steps a limit is relaxed in
std::vector<double> relaxed(double value, double minimum);

// The node's parameters (see opti_pessi_goal_bridge.cpp for what each one is)
struct Params {
  double clearance = 0.3;
  double obstacle_height = 0.2;
  double self_radius = 0.3;
  double lookahead_distance = 3.0;
  double max_chord_deviation = 0.2;
  double waypoint_tolerance = 0.3;
  double max_waypoint_path_distance = 8.0;
  double min_clearance = 0.2;
  double detour_distance = 1.0;
  double min_detour_distance = 0.3;
  double detour_step = 15.0;       // deg
  double max_detour_angle = 90.0;  // deg
  double jump_distance = 0.5;
  double switch_time = 1.0;
  double reach_distance = 0.3;
  double republish_distance = 0.1;
  double waypoint_reach_distance = 0.5;
  double republish_yaw = 0.1;
  double hint_radius = 0.5;
  bool relay_way_point = false;
};

struct Goal {
  Vec2 position;
  double yaw;
  std::string source;  // midpoint, waypoint, path, detour or stop
};

// What the planners and the robot say this tick, all in the goal frame
struct Inputs {
  Vec2 robot = Vec2::Zero();
  double robot_yaw = 0.0;
  std::optional<Vec2> waypoint;               // /way_point; none while far_planner's route is used
  std::vector<Path> paths;                    // TARE's paths, in path_topics order
  std::optional<Path> route;                  // far_planner's route, two points at least
  std::optional<std::vector<Vec2>> obstacles;  // /terrain_map obstacles; none without a fresh map
  std::optional<Goal> hint;                   // /goal_yaw_hint: position and yaw
};

// The candidate goal of a tick, from obstacles already kept to those that can come near
Goal chooseCandidate(const Params& params, const Inputs& inputs);

// The goal from one tick to the next: a jumping candidate is only taken once it has stayed put for switch_time
// (unless the current goal is reached or no longer clear); the goal is published when it has moved more than
// republish_distance or its yaw has turned by more than republish_yaw.
class GoalBridge {
 public:
  struct Step {
    std::optional<Goal> publish;            // the goal to publish this tick
    std::optional<std::string> new_source;  // where the goal comes from, when that changed
    bool stopped = false;                   // no way keeps min_clearance
  };
  Step step(const Params& params, Inputs inputs, double now);
  // Publish whatever the next step decides (as if nothing had been published yet)
  void forgetPublished() { published_.reset(); }

 private:
  std::optional<Goal> goal_;
  std::optional<Goal> candidate_;
  double candidate_since_ = 0.0;
  std::optional<Goal> published_;
};

}  // namespace ranav_mission
