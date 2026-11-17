#include "ranav_mission/opti_pessi_goal.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ranav_mission {

namespace {

// Index of a vertex arc length: numpy.searchsorted(vertex_s, s, side) - 1 kept to a segment
size_t segmentIndex(const std::vector<double>& vertex_s, double s, bool right) {
  const auto it = right ? std::upper_bound(vertex_s.begin(), vertex_s.end(), s)
                        : std::lower_bound(vertex_s.begin(), vertex_s.end(), s);
  const long i = static_cast<long>(it - vertex_s.begin()) - 1;
  return static_cast<size_t>(std::clamp(i, 0L, static_cast<long>(vertex_s.size()) - 2));
}

Goal detourOrStop(const Params& p, const Inputs& in, const std::vector<Vec2>& targets,
                  const std::vector<Vec2>& obstacles);

}  // namespace

double angleDifference(double a, double b) {
  double d = std::fmod(a - b + M_PI, 2.0 * M_PI);
  if (d < 0.0) d += 2.0 * M_PI;
  return d - M_PI;
}

double bearing(const Vec2& origin, const Vec2& target, double fallback) {
  const Vec2 offset = target - origin;
  return offset.norm() > 1e-3 ? std::atan2(offset.y(), offset.x()) : fallback;
}

Path withoutEmptySegments(const Path& path) {
  Path out;
  for (size_t i = 0; i < path.size(); ++i) {
    if (i == 0 || (path[i].head<2>() - path[i - 1].head<2>()).norm() > 1e-6) out.push_back(path[i]);
  }
  return out;
}

Projection project(const Vec2& point, const Path& path) {
  Projection p;
  const size_t n = path.size();
  p.vertex_s.assign(n, 0.0);
  p.distance.resize(n - 1);
  p.s.resize(n - 1);
  for (size_t j = 0; j + 1 < n; ++j) {
    const Vec2 a = path[j].head<2>();
    const Vec2 ab = path[j + 1].head<2>() - a;
    const double length = ab.norm();
    const double t = length > 0.0 ? std::clamp((point - a).dot(ab) / (length * length), 0.0, 1.0) : 0.0;
    p.distance[j] = (a + t * ab - point).norm();
    p.vertex_s[j + 1] = p.vertex_s[j] + length;
    p.s[j] = p.vertex_s[j] + t * length;
  }
  return p;
}

Vec3 pointAt(const Path& path, const std::vector<double>& vertex_s, double s) {
  const size_t i = segmentIndex(vertex_s, s, true);
  const double t = std::clamp((s - vertex_s[i]) / (vertex_s[i + 1] - vertex_s[i]), 0.0, 1.0);
  return path[i] + t * (path[i + 1] - path[i]);
}

double pathYaw(const Path& path, const std::vector<double>& vertex_s, double s, double direction) {
  for (double v : vertex_s) {
    if (std::abs(v - s) < 1e-9) {
      s = v;
      break;
    }
  }
  Vec3 d;
  if (direction > 0) {
    const size_t i = segmentIndex(vertex_s, s, true);
    d = path[i + 1] - path[i];
  } else {
    const size_t i = segmentIndex(vertex_s, s, false);
    d = path[i] - path[i + 1];
  }
  return std::atan2(d.y(), d.x());
}

double distanceToSegment(const Vec2& point, const Vec2& a, const Vec2& b) {
  const Vec2 ab = b - a;
  const double t = std::clamp((point - a).dot(ab) / std::max(ab.dot(ab), 1e-12), 0.0, 1.0);
  return (a + t * ab - point).norm();
}

PathGoal aheadOnPath(const Path& path, const std::vector<double>& vertex_s, double start, double end,
                     const Vec2& robot, double lookahead, double max_chord_deviation) {
  const double direction = end >= start ? 1.0 : -1.0;
  // Vertices passed walking min(lookahead, remaining) from the robot's projection, in walking order, and the point
  // reached, with their arc lengths
  const double length = std::min(lookahead, std::abs(end - start));
  std::vector<size_t> passed;
  for (size_t v = 0; v < vertex_s.size(); ++v) {
    const double ahead = (vertex_s[v] - start) * direction;
    if (ahead > 0.0 && ahead < length) passed.push_back(v);
  }
  if (direction < 0) std::reverse(passed.begin(), passed.end());
  std::vector<Vec3> candidates;
  std::vector<double> candidates_s;
  for (size_t v : passed) {
    candidates.push_back(path[v]);
    candidates_s.push_back(vertex_s[v]);
  }
  candidates.push_back(pointAt(path, vertex_s, start + direction * length));
  candidates_s.push_back(start + direction * length);
  // Farthest candidate whose straight line from the robot stays within max_chord_deviation of the vertices before it
  size_t k = 0;
  for (size_t c = candidates.size() - 1; c > 0; --c) {
    double worst = 0.0;
    for (size_t j = 0; j < c; ++j) {
      worst = std::max(worst, distanceToSegment(candidates[j].head<2>(), robot, candidates[c].head<2>()));
    }
    if (worst <= max_chord_deviation) {
      k = c;
      break;
    }
  }
  return {candidates[k], pathYaw(path, vertex_s, candidates_s[k], direction)};
}

std::optional<PathGoal> follow(const Path& path, const Vec2& robot, const Vec2& waypoint, double lookahead,
                               double waypoint_tolerance, double max_waypoint_path_distance,
                               double max_chord_deviation) {
  const Projection w = project(waypoint, path);
  const double w_min = *std::min_element(w.distance.begin(), w.distance.end());
  if (w_min > waypoint_tolerance) return std::nullopt;
  std::vector<double> waypoint_s;
  for (size_t j = 0; j < w.s.size(); ++j) {
    if (w.distance[j] <= w_min + kMatchSlack) waypoint_s.push_back(w.s[j]);
  }
  const Projection r = project(robot, path);
  const double r_min = *std::min_element(r.distance.begin(), r.distance.end());
  std::vector<double> robot_distance, robot_s;
  for (size_t j = 0; j < r.s.size(); ++j) {
    if (r.distance[j] <= std::max(r_min + kMatchSlack, kRobotMatchRadius)) {
      robot_distance.push_back(r.distance[j]);
      robot_s.push_back(r.s[j]);
    }
  }
  // The robot is where the path is nearest, unless that is farther from the waypoint along the path than TARE puts
  // it: a TARE path loops back past the robot, and on its way out or back may pass closer than where TARE starts it.
  std::vector<double> gap_min(robot_s.size(), std::numeric_limits<double>::infinity());
  for (size_t i = 0; i < robot_s.size(); ++i) {
    for (double s : waypoint_s) gap_min[i] = std::min(gap_min[i], std::abs(s - robot_s[i]));
  }
  std::optional<size_t> best;
  for (size_t i = 0; i < robot_s.size(); ++i) {
    if (gap_min[i] <= max_waypoint_path_distance && (!best || robot_distance[i] < robot_distance[*best])) best = i;
  }
  if (!best) {
    best = static_cast<size_t>(std::min_element(gap_min.begin(), gap_min.end()) - gap_min.begin());
  }
  const size_t i = *best;
  size_t wi = 0;
  for (size_t j = 1; j < waypoint_s.size(); ++j) {
    if (std::abs(waypoint_s[j] - robot_s[i]) < std::abs(waypoint_s[wi] - robot_s[i])) wi = j;
  }
  return aheadOnPath(path, w.vertex_s, robot_s[i], waypoint_s[wi], robot, lookahead, max_chord_deviation);
}

PathGoal pursue(const Path& route, const Vec2& robot, double lookahead, double max_chord_deviation) {
  const Projection p = project(robot, route);
  const double d_min = *std::min_element(p.distance.begin(), p.distance.end());
  double start = std::numeric_limits<double>::infinity();
  for (size_t j = 0; j < p.s.size(); ++j) {
    if (p.distance[j] <= d_min + kMatchSlack) start = std::min(start, p.s[j]);
  }
  return aheadOnPath(route, p.vertex_s, start, p.vertex_s.back(), robot, lookahead, max_chord_deviation);
}

bool clear(const Vec2& robot, const Vec2& goal, const std::vector<Vec2>& obstacles, double clearance) {
  const Vec2 ab = goal - robot;
  if (ab.dot(ab) < 1e-12) return true;
  for (const Vec2& o : obstacles) {
    if ((o - robot).dot(ab) > 0.0 && distanceToSegment(o, robot, goal) < clearance) return false;
  }
  return true;
}

std::optional<Vec2> detour(const Vec2& robot, const Vec2& target, const std::vector<Vec2>& obstacles,
                           double clearance, double distance, double step, double max_angle) {
  const Vec2 offset = target - robot;
  const double length = std::min(offset.norm(), distance);
  if (length < 1e-6) return std::nullopt;
  const double heading = std::atan2(offset.y(), offset.x());
  // nearbyint rounds half to even, as Python's round()
  const long turns = static_cast<long>(std::nearbyint(max_angle / step));
  for (long k = 0; k <= turns; ++k) {
    for (double sign : {1.0, -1.0}) {
      if (k == 0 && sign < 0) continue;
      const double angle = heading + sign * static_cast<double>(k) * step;
      const Vec2 goal = robot + length * Vec2(std::cos(angle), std::sin(angle));
      if (clear(robot, goal, obstacles, clearance)) return goal;
    }
  }
  return std::nullopt;
}

std::vector<double> relaxed(double value, double minimum) {
  std::vector<double> values{value};
  // the floor keeps a minimum of 0 (a parameter range allows it) from halving forever
  while (values.back() / 2.0 > std::max(minimum, 1e-3)) values.push_back(values.back() / 2.0);
  if (minimum < values.back()) values.push_back(minimum);
  return values;
}

namespace {

std::optional<PathGoal> pathGoal(const Params& p, const Inputs& in) {
  for (const Path& raw : in.paths) {
    const Path path = withoutEmptySegments(raw);
    if (path.size() < 2) continue;
    if (auto found = follow(path, in.robot, *in.waypoint, p.lookahead_distance, p.waypoint_tolerance,
                            p.max_waypoint_path_distance, p.max_chord_deviation)) {
      return found;
    }
  }
  return std::nullopt;
}

std::optional<Vec2> relaxedDetour(const Params& p, const Inputs& in, const std::vector<Vec2>& targets,
                                  const std::vector<Vec2>& obstacles) {
  const double step = p.detour_step * M_PI / 180.0, max_angle = p.max_detour_angle * M_PI / 180.0;
  for (double clearance : relaxed(p.clearance, p.min_clearance)) {
    for (double distance : relaxed(p.detour_distance, p.min_detour_distance)) {
      for (const Vec2& target : targets) {
        if (auto goal = detour(in.robot, target, obstacles, clearance, distance, step, max_angle)) return goal;
      }
    }
  }
  return std::nullopt;
}

Goal detourOrStop(const Params& p, const Inputs& in, const std::vector<Vec2>& targets,
                  const std::vector<Vec2>& obstacles) {
  if (auto goal = relaxedDetour(p, in, targets, obstacles)) {
    return {*goal, bearing(in.robot, *goal, in.robot_yaw), "detour"};
  }
  return {in.robot, in.robot_yaw, "stop"};
}

// TARE (or far_planner without a route): /way_point, the path it lies on, a detour, a stop
Goal chooseTowardsWayPoint(const Params& p, const Inputs& in) {
  const Vec2& waypoint = *in.waypoint;
  // Near /way_point the goal is /way_point itself: midpoints alone stop short of it (OptiPessi stands at goals
  // nearer than 0.1 m, a midpoint that moved less than republish_distance is not sent), while TARE keeps its
  // /way_point on a viewpoint until the robot is there
  const bool near = (waypoint - in.robot).norm() <= p.waypoint_reach_distance;
  const Vec2 straight = near ? waypoint : Vec2(0.5 * (in.robot + waypoint));
  const Goal straight_goal{straight, bearing(in.robot, waypoint, in.robot_yaw), near ? "waypoint" : "midpoint"};
  std::optional<Goal> path_goal;
  if (auto found = pathGoal(p, in)) path_goal = Goal{found->point.head<2>(), found->yaw, "path"};
  if (!in.obstacles) return path_goal ? *path_goal : straight_goal;
  const auto& obstacles = *in.obstacles;
  if (clear(in.robot, waypoint, obstacles, p.clearance)) return straight_goal;
  if (path_goal && clear(in.robot, path_goal->position, obstacles, p.clearance)) return *path_goal;
  std::vector<Vec2> targets;
  if (path_goal) targets.push_back(path_goal->position);
  targets.push_back(straight);
  return detourOrStop(p, in, targets, obstacles);
}

// far_planner's route: lookahead along it, shortened down to min_detour_distance, a detour, a stop
Goal chooseOnRoute(const Params& p, const Inputs& in) {
  const Path route = withoutEmptySegments(*in.route);
  if (route.size() < 2) return {route[0].head<2>(), in.robot_yaw, "path"};  // the robot is at the goal
  std::vector<Goal> goals;
  for (double lookahead : relaxed(p.lookahead_distance, p.min_detour_distance)) {
    const PathGoal g = pursue(route, in.robot, lookahead, p.max_chord_deviation);
    goals.push_back({g.point.head<2>(), g.yaw, "path"});
  }
  if (!in.obstacles) return goals[0];
  for (const Goal& goal : goals) {
    if (clear(in.robot, goal.position, *in.obstacles, p.clearance)) return goal;
  }
  return detourOrStop(p, in, {goals[0].position}, *in.obstacles);
}

}  // namespace

Goal chooseCandidate(const Params& params, const Inputs& inputs) {
  if (params.relay_way_point) {
    return {*inputs.waypoint, bearing(inputs.robot, *inputs.waypoint, inputs.robot_yaw), "waypoint"};
  }
  return inputs.route ? chooseOnRoute(params, inputs) : chooseTowardsWayPoint(params, inputs);
}

GoalBridge::Step GoalBridge::step(const Params& p, Inputs in, double now) {
  Step result;
  if (in.obstacles) {
    // Only obstacles a goal (at most /way_point, lookahead_distance, detour_distance or the current goal away) can
    // come near
    const double reach = std::max({in.waypoint ? (*in.waypoint - in.robot).norm() : 0.0, p.lookahead_distance,
                                   p.detour_distance, goal_ ? (goal_->position - in.robot).norm() : 0.0}) +
                         p.clearance;
    std::vector<Vec2> near;
    for (const Vec2& o : *in.obstacles) {
      const double d = (o - in.robot).norm();
      if (d > p.self_radius && d < reach) near.push_back(o);
    }
    in.obstacles = std::move(near);
  }
  const Goal candidate = chooseCandidate(p, in);
  result.stopped = candidate.source == "stop";

  if (!candidate_ || (candidate.position - candidate_->position).norm() > p.jump_distance) candidate_since_ = now;
  candidate_ = candidate;
  const bool keep_goal = !p.relay_way_point && goal_ && now - candidate_since_ < p.switch_time &&
                         (goal_->position - in.robot).norm() > p.reach_distance &&
                         (!in.obstacles || clear(in.robot, goal_->position, *in.obstacles, p.clearance));
  if (!keep_goal) {
    if (!goal_ || candidate.source != goal_->source) result.new_source = candidate.source;
    goal_ = candidate;
  }
  if (in.hint && (goal_->position - in.hint->position).norm() <= p.hint_radius) goal_->yaw = in.hint->yaw;

  if (published_ && (goal_->position - published_->position).norm() <= p.republish_distance &&
      std::abs(angleDifference(goal_->yaw, published_->yaw)) <= p.republish_yaw) {
    return result;
  }
  published_ = goal_;
  result.publish = goal_;
  return result;
}

}  // namespace ranav_mission
