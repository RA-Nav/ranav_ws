// opti_pessi_goal_bridge: the goal of OptiPessiController (/opti_pessi/goal), from TARE or far_planner.
//
// OptiPessi walks straight to /opti_pessi/goal, with no local planner in between, so the goal is chosen here and
// checked against the obstacles of /terrain_map (points more than obstacle_height above the ground, farther than
// self_radius from the robot):
//   1. /way_point, the paths in path_topics (/exploration_path: TARE's local path, /global_path_full: its global TSP
//      loop), /terrain_map (all in frame map) and the robot (/odom, base) are brought to goal_frame (odom, the only
//      frame the controller takes) with the latest TF (odom -> lio_map -> camera_init -> map, all from FAST-LIO);
//   2. the candidate goal is the first of
//      - midpoint: the middle point between the robot and /way_point, if the straight line robot -> /way_point keeps
//        clearance from every obstacle (walking to the midpoint again and again walks that whole line); within
//        waypoint_reach_distance of /way_point, /way_point itself ("waypoint"). Midpoints alone stop short of it:
//        OptiPessi stands at a goal nearer than 0.1 m, and a midpoint that moved less than republish_distance is not
//        sent, while TARE keeps its /way_point on a viewpoint until the robot is there (in a test the robot stood
//        0.3 m from one for 20 minutes);
//      - path: lookahead_distance ahead of the robot along the TARE path /way_point lies on (pure pursuit), if the
//        line robot -> it keeps clearance: the path goes round walls and through doors, but flickers when TARE
//        replans, so it is only taken when the straight way is blocked;
//      - detour: a point up to detour_distance away, in the direction of the path goal (or else of the midpoint)
//        turned by the smallest multiple of detour_step (up to max_detour_angle each side) that keeps clearance; if
//        there is none, the search is repeated with the distance halved down to min_detour_distance, then all over
//        again with min_clearance;
//      - stop: the robot itself (OptiPessi stands at a reached goal);
//   3. the goal follows the candidate, but a candidate that jumps (more than jump_distance in one tick) is only taken
//      once it has stayed put for switch_time, as long as the current goal is still clear and more than
//      reach_distance away;
//   4. it is published (one ADD ARROW marker in goal_frame, colored by where it comes from) when it has moved more
//      than republish_distance or its yaw has turned by more than republish_yaw.
//
// Every goal has a yaw, in the marker's orientation: on a path or far_planner's route its direction there in
// walking direction; a midpoint or /way_point itself faces /way_point, a detour the way from the robot to it; a stop,
// or the robot at the end of far_planner's route, keeps the robot's yaw. acados_legged_controller turns to it near
// the goal; OptiPessiController reads the position only. A yaw hint (/goal_yaw_hint, PoseStamped, kept until
// replaced; an empty frame_id clears it) overrides that yaw for every goal within hint_radius of its position.
//
// With far_planner (route_topic:=/viz_path_topic) the goal is taken on the route far_planner has planned (a
// LINE_STRIP Marker, empty when it has no route) instead of step 2: lookahead_distance ahead along it, shortened down
// to min_detour_distance when the line to it is blocked, then a detour or a stop. /way_point is only used while
// far_planner publishes no route (goal reached: /way_point is the goal; no way found: /way_point is the robot).
//
// relay_way_point: /way_point is the goal as it is, with no obstacle check and no path (what
// way_point_to_opti_pessi_goal.py did).
//
// Without a fresh /terrain_map nothing is checked. Until /way_point (or a route) and /odom are fresh the last goal is
// kept. kExtendWayPoint must stay false in the TARE config (the path goal needs /way_point on the path). All
// distances can be changed at run time (ros2 param set). The robot comes from /odom by message, not from the
// odom -> base TF, which OptiPessi's visualizer stamps with wall time.
#include <Eigen/Geometry>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "ranav_mission/opti_pessi_goal.hpp"

namespace ranav_mission {

namespace {

double yawOf(const geometry_msgs::msg::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

// Goal marker color by where the goal comes from
const std::map<std::string, std::array<float, 3>> kColors = {{"midpoint", {0.0f, 1.0f, 0.0f}},
                                                             {"waypoint", {0.6f, 1.0f, 0.6f}},
                                                             {"path", {0.0f, 0.5f, 1.0f}},
                                                             {"detour", {1.0f, 0.8f, 0.0f}},
                                                             {"stop", {1.0f, 0.0f, 0.0f}}};

// A rigid transform to goal_frame (frames turned about z only, for yaws)
struct Transform {
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  double yaw = 0.0;
  Vec3 operator()(const Vec3& p) const { return pose * p; }
};

}  // namespace

class OptiPessiGoalBridge : public rclcpp::Node {
 public:
  OptiPessiGoalBridge() : Node("opti_pessi_goal_bridge") {
    number("clearance", 0.3, "distance the line robot -> goal keeps from obstacles [m]", 0.0, 2.0);
    number("obstacle_height", 0.2, "/terrain_map points higher above the ground are obstacles [m]", 0.0, 2.0);
    number("self_radius", 0.3, "obstacles nearer the robot are ignored (the robot's own legs, a wall it brushes) [m]",
           0.0, 1.0);
    number("lookahead_distance", 3.0,
           "path goal distance ahead of the robot along the TARE path or far_planner's route [m]", 0.2, 5.0);
    number("max_chord_deviation", 0.2, "how far path vertices may lie off the line robot -> path goal [m]", 0.0,
           2.0);
    number("waypoint_tolerance", 0.3, "how far /way_point may lie off a path it is taken from [m]", 0.0, 2.0);
    number("max_waypoint_path_distance", 8.0,
           "farthest /way_point along the path from the robot (TARE: kLookAheadDistance plus a path segment) [m]", 1.0,
           100.0);
    number("min_clearance", 0.2, "clearance a detour may shrink to before the robot stops [m]", 0.0, 2.0);
    number("detour_distance", 1.0, "farthest detour goal from the robot [m]", 0.2, 5.0);
    number("min_detour_distance", 0.3,
           "distance a detour, or the lookahead along far_planner's route, may shrink to before the robot stops [m]",
           0.05, 5.0);
    number("detour_step", 15.0, "angle between detour directions [deg]", 1.0, 90.0);
    number("max_detour_angle", 90.0, "largest detour turn each side [deg]", 0.0, 180.0);
    number("jump_distance", 0.5, "candidate move in one tick that is a jump [m]", 0.0, 10.0);
    number("switch_time", 1.0, "how long a jumped candidate must stay put before it is taken [s]", 0.0, 10.0);
    number("reach_distance", 0.3, "goal distance at which the next candidate is taken at once [m]", 0.0, 2.0);
    number("republish_distance", 0.1, "goal change that is published [m]", 0.0, 1.0);
    number("waypoint_reach_distance", 0.5, "/way_point nearer than this is the goal itself, not the midpoint [m]", 0.0,
           5.0);
    number("republish_yaw", 0.1, "goal yaw change that is published, also at the same place [rad]", 0.0, 3.2);
    number("hint_radius", 0.5, "goals this close to the /goal_yaw_hint position take its yaw [m]", 0.0, 5.0);
    number("input_timeout", 3.0, "age after which a path, /way_point, /odom or /terrain_map is ignored [s]", 0.1, 60.0);
    declare_parameter("relay_way_point", false);
    goal_frame_ = declare_parameter("goal_frame", std::string("odom"));
    path_topics_ = declare_parameter("path_topics", std::vector<std::string>{"/exploration_path", "/global_path_full"});
    // far_planner's route (LINE_STRIP Marker): when set and published, the goal is taken on it, not from /way_point
    route_topic_ = declare_parameter("route_topic", std::string(""));
    const double rate = declare_parameter("rate", 10.0);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    goal_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/opti_pessi/goal", 1);

    for (const auto& topic : path_topics_) {
      path_subs_.push_back(create_subscription<nav_msgs::msg::Path>(
          topic, 1, [this, topic](nav_msgs::msg::Path::ConstSharedPtr msg) { paths_[topic] = PathInput{msg, now()}; }));
    }
    if (!route_topic_.empty()) {
      route_sub_ = create_subscription<visualization_msgs::msg::Marker>(
          route_topic_, 1, [this](visualization_msgs::msg::Marker::ConstSharedPtr msg) { route_ = RouteInput{msg, now()}; });
    }
    way_point_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
        "/way_point", 5, [this](geometry_msgs::msg::PointStamped::ConstSharedPtr msg) { way_point_ = WayPointInput{msg, now()}; });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 5, [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { odom_ = OdomInput{msg, now()}; });
    terrain_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/terrain_map", 1, [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { onTerrain(*msg); });
    hint_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_yaw_hint", 5, [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) { onHint(msg); });
    timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(1.0 / rate), [this] { tick(); });
  }

 private:
  template <class Msg>
  struct Input {
    typename Msg::ConstSharedPtr msg;
    rclcpp::Time received;
  };
  struct Terrain {
    std::string frame;
    std::vector<Vec3> obstacles;
  };
  struct TerrainMsg {
    using ConstSharedPtr = std::shared_ptr<const Terrain>;
  };
  using PathInput = Input<nav_msgs::msg::Path>;
  using RouteInput = Input<visualization_msgs::msg::Marker>;
  using WayPointInput = Input<geometry_msgs::msg::PointStamped>;
  using OdomInput = Input<nav_msgs::msg::Odometry>;
  using TerrainInput = Input<TerrainMsg>;

  void number(const std::string& name, double value, const std::string& description, double low, double high) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = low;
    range.to_value = high;
    descriptor.floating_point_range.push_back(range);
    declare_parameter(name, value, descriptor);
  }

  double param(const std::string& name) const { return get_parameter(name).as_double(); }

  Params params() const {
    Params p;
    p.clearance = param("clearance");
    p.obstacle_height = param("obstacle_height");
    p.self_radius = param("self_radius");
    p.lookahead_distance = param("lookahead_distance");
    p.max_chord_deviation = param("max_chord_deviation");
    p.waypoint_tolerance = param("waypoint_tolerance");
    p.max_waypoint_path_distance = param("max_waypoint_path_distance");
    p.min_clearance = param("min_clearance");
    p.detour_distance = param("detour_distance");
    p.min_detour_distance = param("min_detour_distance");
    p.detour_step = param("detour_step");
    p.max_detour_angle = param("max_detour_angle");
    p.jump_distance = param("jump_distance");
    p.switch_time = param("switch_time");
    p.reach_distance = param("reach_distance");
    p.republish_distance = param("republish_distance");
    p.waypoint_reach_distance = param("waypoint_reach_distance");
    p.republish_yaw = param("republish_yaw");
    p.hint_radius = param("hint_radius");
    p.relay_way_point = get_parameter("relay_way_point").as_bool();
    return p;
  }

  void onTerrain(const sensor_msgs::msg::PointCloud2& msg) {
    // terrainAnalysis: intensity is the height above the ground
    const double obstacle_height = param("obstacle_height");
    Terrain terrain{msg.header.frame_id, {}};
    sensor_msgs::PointCloud2ConstIterator<float> x(msg, "x"), y(msg, "y"), z(msg, "z"), i(msg, "intensity");
    for (; x != x.end(); ++x, ++y, ++z, ++i) {
      if (std::isfinite(*x) && std::isfinite(*y) && std::isfinite(*z) && *i > obstacle_height) {
        terrain.obstacles.emplace_back(*x, *y, *z);
      }
    }
    terrain_ = TerrainInput{std::make_shared<const Terrain>(std::move(terrain)), now()};
  }

  void onHint(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
    if (msg->header.frame_id.empty()) {
      hint_.reset();
      RCLCPP_INFO(get_logger(), "goal yaw hint cleared");
      return;
    }
    hint_ = msg;
    RCLCPP_INFO(get_logger(), "goal yaw hint at (%.2f, %.2f) in %s: %.0f deg", msg->pose.position.x,
                msg->pose.position.y, msg->header.frame_id.c_str(), yawOf(msg->pose.orientation) * 180.0 / M_PI);
  }

  template <class Msg>
  typename Msg::ConstSharedPtr fresh(const std::optional<Input<Msg>>& input) const {
    if (!input || now() - input->received > rclcpp::Duration::from_seconds(param("input_timeout"))) return nullptr;
    return input->msg;
  }

  // From frame to goal_frame with the latest TF (a planner message may be stamped ahead of the odom tree); throws
  // tf2::TransformException
  Transform toGoalFrame(const std::string& frame) const {
    Transform t;
    if (frame.empty() || frame == goal_frame_) return t;
    const auto tf = tf_buffer_->lookupTransform(goal_frame_, frame, tf2::TimePointZero).transform;
    const Eigen::Quaterniond q(tf.rotation.w, tf.rotation.x, tf.rotation.y, tf.rotation.z);
    t.pose.linear() = q.toRotationMatrix();
    t.pose.translation() = Vec3(tf.translation.x, tf.translation.y, tf.translation.z);
    t.yaw = yawOf(tf.rotation);
    return t;
  }

  void tick() {
    const Params p = params();
    const auto odom = fresh(odom_);
    const auto way_point = fresh(way_point_);
    visualization_msgs::msg::Marker::ConstSharedPtr route;
    if (!route_topic_.empty() && !p.relay_way_point) route = fresh(route_);
    if (route && route->points.size() < 2) route = nullptr;  // far_planner has no route now: /way_point says where to stand
    if (!odom || (!way_point && !route)) {
      // once per wait: an idle planner (inspection done, no goal) publishes nothing for as long as it is idle
      if (!waiting_) RCLCPP_INFO(get_logger(), "waiting for /odom and /way_point");
      waiting_ = true;
      return;
    }
    waiting_ = false;
    const auto terrain = fresh(terrain_);
    Inputs in;
    double robot_z = 0.0;
    try {
      if (hint_) {
        const Transform t = toGoalFrame(hint_->header.frame_id);
        const auto& h = hint_->pose.position;
        in.hint = Goal{t(Vec3(h.x, h.y, h.z)).head<2>(), yawOf(hint_->pose.orientation) + t.yaw, "hint"};
      }
      const Transform to_robot = toGoalFrame(odom->header.frame_id);
      const auto& o = odom->pose.pose.position;
      const Vec3 robot = to_robot(Vec3(o.x, o.y, o.z));
      in.robot = robot.head<2>();
      robot_z = robot.z();
      in.robot_yaw = yawOf(odom->pose.pose.orientation) + to_robot.yaw;
      if (route) {
        const Transform t = toGoalFrame(route->header.frame_id);
        Path path;
        for (const auto& q : route->points) path.push_back(t(Vec3(q.x, q.y, q.z)));
        in.route = std::move(path);
      } else {
        const auto& w = way_point->point;
        in.waypoint = toGoalFrame(way_point->header.frame_id)(Vec3(w.x, w.y, w.z)).head<2>();
        if (!p.relay_way_point) {
          for (const auto& topic : path_topics_) {
            const auto it = paths_.find(topic);
            const auto path_msg = it == paths_.end() ? nav_msgs::msg::Path::ConstSharedPtr() : fresh(std::optional(it->second));
            if (!path_msg || path_msg->poses.size() < 2) continue;
            const Transform t = toGoalFrame(path_msg->header.frame_id);
            Path path;
            for (const auto& q : path_msg->poses) {
              path.push_back(t(Vec3(q.pose.position.x, q.pose.position.y, q.pose.position.z)));
            }
            in.paths.push_back(std::move(path));
          }
        }
      }
      if (terrain && !p.relay_way_point) {
        const Transform t = toGoalFrame(terrain->frame);
        std::vector<Vec2> obstacles;
        obstacles.reserve(terrain->obstacles.size());
        for (const Vec3& q : terrain->obstacles) obstacles.push_back(t(q).head<2>());
        in.obstacles = std::move(obstacles);
      }
    } catch (const tf2::TransformException& e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "no TF to %s, goal kept: %s", goal_frame_.c_str(),
                           e.what());
      return;
    }
    if (!terrain && !p.relay_way_point) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "no /terrain_map, goal not checked for obstacles");
    }

    const auto step = bridge_.step(p, in, now().seconds());
    if (step.stopped) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "no way %s keeps %.2f m from /terrain_map obstacles (detours down to %.2f m): standing "
                           "until it changes",
                           route ? "along far_planner's route" : "towards /way_point", p.min_clearance,
                           p.min_detour_distance);
    }
    // A new TARE waypoint, in TARE's frame: in goal_frame it moves with every odom -> lio_map update
    if (way_point && !route) {
      const std::pair<long, long> rounded{std::lround(way_point->point.x * 10.0), std::lround(way_point->point.y * 10.0)};
      if (rounded != last_way_point_) {
        last_way_point_ = rounded;
        RCLCPP_INFO(get_logger(), "/way_point (%.2f, %.2f), %.2f m away", in.waypoint->x(), in.waypoint->y(),
                    (*in.waypoint - in.robot).norm());
      }
    }
    if (step.new_source) RCLCPP_INFO(get_logger(), "goal from %s", step.new_source->c_str());
    if (step.publish) publish(*step.publish, robot_z);
  }

  void publish(const Goal& goal, double z) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = goal_frame_;
    marker.header.stamp = now();
    marker.ns = "goal";
    marker.type = visualization_msgs::msg::Marker::ARROW;  // the goal yaw: acados turns to it near the goal
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = goal.position.x();
    marker.pose.position.y = goal.position.y();
    marker.pose.position.z = z;
    marker.pose.orientation.z = std::sin(0.5 * goal.yaw);
    marker.pose.orientation.w = std::cos(0.5 * goal.yaw);
    marker.scale.x = 0.5;
    marker.scale.y = marker.scale.z = 0.1;
    const auto& color = kColors.at(goal.source);
    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = 1.0f;
    visualization_msgs::msg::MarkerArray markers;
    markers.markers.push_back(marker);
    goal_pub_->publish(markers);
  }

  std::string goal_frame_, route_topic_;
  std::vector<std::string> path_topics_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr goal_pub_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr> path_subs_;
  rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr route_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr way_point_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr terrain_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr hint_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::map<std::string, PathInput> paths_;
  std::optional<RouteInput> route_;
  std::optional<WayPointInput> way_point_;
  std::optional<OdomInput> odom_;
  std::optional<TerrainInput> terrain_;
  geometry_msgs::msg::PoseStamped::ConstSharedPtr hint_;
  std::pair<long, long> last_way_point_{std::numeric_limits<long>::min(), 0};
  bool waiting_ = false;
  GoalBridge bridge_;
};

}  // namespace ranav_mission

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ranav_mission::OptiPessiGoalBridge>());
  rclcpp::shutdown();
  return 0;
}
