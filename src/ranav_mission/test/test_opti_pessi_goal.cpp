// Which /opti_pessi/goal opti_pessi_goal_bridge picks from TARE's /way_point and paths or far_planner's route, and
// which yaw it gives it.
#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include "ranav_mission/opti_pessi_goal.hpp"

using namespace ranav_mission;

namespace {

constexpr double kLookahead = 2.0;
constexpr double kChord = 0.2;

Path route(std::initializer_list<std::pair<double, double>> xy) {
  Path path;
  for (const auto& [x, y] : xy) path.emplace_back(x, y, 0.0);
  return path;
}

PathGoal pursueOn(const Path& path, double x, double y, double lookahead = kLookahead) {
  return pursue(path, Vec2(x, y), lookahead, kChord);
}

PathGoal followOn(const Path& path, double x, double y, double wx, double wy) {
  auto goal = follow(path, Vec2(x, y), Vec2(wx, wy), kLookahead, 0.3, 8.0, kChord);
  EXPECT_TRUE(goal.has_value());
  return *goal;
}

void expectAt(const Vec3& point, double x, double y, double tolerance = 1e-3) {
  EXPECT_NEAR(point.x(), x, tolerance);
  EXPECT_NEAR(point.y(), y, tolerance);
}

// The bridge with everything already in odom, as the node gives it
struct Bridge {
  Params params;
  Inputs inputs;
  GoalBridge bridge;
  double now = 50.0;
  std::vector<Goal> published;

  Bridge() { params.lookahead_distance = kLookahead; }

  void robot(double x, double y, double yaw = 0.0) {
    inputs.robot = Vec2(x, y);
    inputs.robot_yaw = yaw;
  }
  void wayPoint(double x, double y) { inputs.waypoint = Vec2(x, y); }
  void tarePath(std::initializer_list<std::pair<double, double>> xy) { inputs.paths = {route(xy)}; }
  void farRoute(std::initializer_list<std::pair<double, double>> xy) { inputs.route = route(xy); }
  void obstacles(std::initializer_list<std::pair<double, double>> xy = {}) {
    std::vector<Vec2> points;
    for (const auto& [x, y] : xy) points.emplace_back(x, y);
    inputs.obstacles = points;
  }
  // Obstacles all round the robot at (0, 0): no way out, the robot stops
  void ring(double radius = 0.4) {
    std::vector<Vec2> points;
    for (int a = 0; a < 360; a += 5) points.emplace_back(radius * std::cos(a * M_PI / 180), radius * std::sin(a * M_PI / 180));
    inputs.obstacles = points;
  }
  void hint(double x, double y, double yaw) { inputs.hint = Goal{Vec2(x, y), yaw, "hint"}; }
  void clearHint() { inputs.hint.reset(); }

  // One tick; returns whether it published
  bool tick() {
    now += 0.1;
    const auto step = bridge.step(params, inputs, now);
    if (step.publish) published.push_back(*step.publish);
    return step.publish.has_value();
  }
  // Goal and where it comes from, after one tick that publishes whatever it decides
  std::pair<std::pair<double, double>, std::string> goal() {
    bridge.forgetPublished();
    EXPECT_TRUE(tick());
    const Goal& g = published.back();
    return {{std::round(g.position.x() * 100) / 100, std::round(g.position.y() * 100) / 100}, g.source};
  }
  double yaw() const { return published.back().yaw; }
};

using At = std::pair<std::pair<double, double>, std::string>;

}  // namespace

// ---------------------------------------------------------------- the point of the route to walk to

TEST(Pursue, OnAStraightRouteTheGoalIsLookaheadAheadOnIt) {
  expectAt(pursueOn(route({{0, 0}, {10, 0}}), 1.0, 0.0).point, 3.0, 0.0);
}

TEST(Pursue, TheGoalIsMeasuredFromWhereTheRobotIsAlongTheRouteNotFromItsStart) {
  // far_planner's route starts where the robot was when it planned; the robot has walked on, a little to the side
  expectAt(pursueOn(route({{0, 0}, {10, 0}}), 4.0, 0.3).point, 6.0, 0.0);
}

TEST(Pursue, TheGoalNeverGoesPastTheEndOfTheRoute) {
  expectAt(pursueOn(route({{0, 0}, {1.2, 0}}), 0.0, 0.0).point, 1.2, 0.0);
  expectAt(pursueOn(route({{0, 0}, {1.2, 0}}), 1.2, 0.0).point, 1.2, 0.0);
}

TEST(Pursue, BeforeACornerTheGoalStopsAtTheCornerInsteadOfCuttingIt) {
  // the route turns left round an obstacle corner at (3, 0): the straight line to a point past it would cut inside
  expectAt(pursueOn(route({{0, 0}, {3, 0}, {3, 5}}), 2.0, 0.0).point, 3.0, 0.0);
}

TEST(Pursue, AtTheCornerTheGoalMovesOnAlongTheNextStretch) {
  expectAt(pursueOn(route({{0, 0}, {3, 0}, {3, 5}}), 3.0, 0.0).point, 3.0, 2.0);
}

TEST(Pursue, AGentleBendIsCutAsFarAsMaxChordDeviationAllows) {
  // the vertex (1, 0) lies 0.05 m off the line robot -> (2, 0.1)
  expectAt(pursueOn(route({{0, 0}, {1, 0}, {3, 0.2}}), 0.0, 0.0).point, 1.995, 0.1, 0.01);
}

TEST(Pursue, ARouteThatComesBackPastTheRobotIsFollowedFromItsStart) {
  // out and back round the end of a wall: the return stretch passes 0.1 m from the robot too
  expectAt(pursueOn(route({{0, 0}, {4, 0}, {4, 0.1}, {0, 0.1}}), 0.0, 0.0).point, 2.0, 0.0);
}

// ---------------------------------------------------------------- the yaw of a goal on a path

TEST(PathYaw, TheYawOfAGoalOnARouteIsTheDirectionOfTheRouteThere) {
  EXPECT_NEAR(pursueOn(route({{0, 0}, {10, 0}}), 1.0, 0.0).yaw, 0.0, 1e-9);
  EXPECT_NEAR(pursueOn(route({{0, 0}, {0, 10}}), 0.0, 1.0).yaw, M_PI / 2, 1e-9);
}

TEST(PathYaw, AGoalStoppedAtACornerFacesTheStretchAfterIt) {
  // the robot turns there: facing where it goes next, not where it comes from
  const PathGoal g = pursueOn(route({{0, 0}, {3, 0}, {3, 5}}), 2.0, 0.0);
  expectAt(g.point, 3.0, 0.0);
  EXPECT_NEAR(g.yaw, M_PI / 2, 1e-9);
}

TEST(PathYaw, AGoalAtTheEndOfTheRouteFacesItsLastStretch) {
  EXPECT_NEAR(pursueOn(route({{0, 0}, {1.2, 0}}), 0.0, 0.0).yaw, 0.0, 1e-9);
  const PathGoal g = pursueOn(route({{0, 0}, {1, 0}, {1, 1}}), 1.0, 0.9);
  expectAt(g.point, 1.0, 1.0);
  EXPECT_NEAR(g.yaw, M_PI / 2, 1e-9);
}

TEST(PathYaw, ATarePathWalkedBackwardsGivesTheWalkingDirection) {
  // TARE's path can lead from the waypoint to the robot: the robot walks it from its end to its start
  const PathGoal g = followOn(route({{0, 0}, {4, 0}}), 4.0, 0.0, 0.0, 0.0);
  expectAt(g.point, 2.0, 0.0);
  EXPECT_NEAR(std::abs(g.yaw), M_PI, 1e-9);
}

TEST(PathYaw, ACornerOfATarePathWalkedBackwardsFacesTheStretchAfterItInWalkingOrder) {
  const PathGoal g = followOn(route({{0, 0}, {0, 3}, {4, 3}}), 1.0, 3.0, 0.0, 0.0);
  expectAt(g.point, 0.0, 3.0);
  EXPECT_NEAR(g.yaw, -M_PI / 2, 1e-9);
}

TEST(Relaxed, AMinimumOfZeroEnds) {
  // min_clearance may be set to 0: halving towards it must stop
  const auto values = relaxed(0.3, 0.0);
  EXPECT_LT(values.size(), 20u);
  EXPECT_EQ(values.back(), 0.0);
}

// ---------------------------------------------------------------- the goal picked

TEST(Bridge, TheGoalIsOnFarPlannersRouteNotTowardsItsWayPoint) {
  Bridge b;
  b.robot(0.0, 0.0);
  b.farRoute({{0, 0}, {10, 0}});
  b.obstacles();
  EXPECT_EQ(b.goal(), At({2.0, 0.0}, "path"));
}

TEST(Bridge, AnObstacleOnTheWayToTheGoalShortensTheStepAlongTheRoute) {
  Bridge b;
  b.robot(0.0, 0.0);
  b.farRoute({{0, 0}, {3, 0}, {3, 5}});
  b.obstacles({{1.6, 0.1}});  // somebody standing on the route
  EXPECT_EQ(b.goal(), At({1.0, 0.0}, "path"));  // the lookahead halved: still on the route, short of the obstacle
}

TEST(Bridge, ARouteBlockedRightAheadIsLeftByADetour) {
  Bridge b;
  b.robot(0.0, 0.0);
  b.farRoute({{0, 0}, {10, 0}});
  b.obstacles({{0.5, 0.0}});
  const auto [position, source] = b.goal();
  EXPECT_EQ(source, "detour");
  EXPECT_NE(position, std::make_pair(0.0, 0.0));
}

TEST(Bridge, WithoutARouteFarPlannersWayPointIsUsedAsBefore) {
  Bridge b;  // far_planner publishes an empty route when it has none: the node passes no route then
  b.robot(0.0, 0.0);
  b.wayPoint(4.0, 0.0);
  b.obstacles();
  EXPECT_EQ(b.goal(), At({2.0, 0.0}, "midpoint"));
}

TEST(Bridge, ARouteIsEnoughWithoutAnyWayPoint) {
  Bridge b;
  b.robot(0.0, 0.0);
  b.farRoute({{0, 0}, {0, 10}});
  b.obstacles();
  EXPECT_EQ(b.goal(), At({0.0, 2.0}, "path"));
}

TEST(Bridge, WithTareThePathIsStillTakenOnlyWhenTheStraightWayToTheWayPointIsBlocked) {
  Bridge b;
  b.robot(0.0, 0.0);
  b.tarePath({{0, 0}, {0, 1.5}, {4, 1.5}, {4, 0}});  // round something standing between the robot and the waypoint
  b.wayPoint(4.0, 0.0);
  b.obstacles();
  EXPECT_EQ(b.goal(), At({2.0, 0.0}, "midpoint"));
  b.obstacles({{2.0, 0.0}});
  EXPECT_EQ(b.goal(), At({0.0, 1.5}, "path"));  // the first corner of the path: no cutting it
}

TEST(Bridge, NearTheWayPointTheGoalIsTheWayPointItself) {
  // The stall seen in the warehouse: the robot 0.31 m from a TARE /way_point on a viewpoint. The midpoint, 0.155 m
  // away, was within republish_distance of the goal already sent, which OptiPessi stood at (within 0.1 m): nobody
  // moved. The way point itself is walked to.
  Bridge b;
  b.robot(0.0, 0.0, 2.5);
  b.wayPoint(0.31, 0.0);
  b.obstacles();
  EXPECT_EQ(b.goal(), At({0.31, 0.0}, "waypoint"));
  EXPECT_NEAR(b.yaw(), 0.0, 1e-9);
}

TEST(Bridge, RelayWayPointSendsTheWayPointAsItIs) {
  Bridge b;
  b.params.relay_way_point = true;
  b.robot(0.0, 0.0);
  b.wayPoint(4.0, 0.0);
  b.obstacles({{2.0, 0.0}});  // not checked
  EXPECT_EQ(b.goal(), At({4.0, 0.0}, "waypoint"));
}

// ---------------------------------------------------------------- the yaw of the goal published

TEST(Bridge, AGoalOnATarePathFacesThePathNotTheLineFromTheRobot) {
  Bridge b;
  b.robot(0.0, 0.0);
  b.tarePath({{0, 0}, {0, 1.5}, {-4, 1.5}, {-4, 0}});  // round something between the robot and a waypoint to the west
  b.wayPoint(-4.0, 0.0);
  b.obstacles({{-2.0, 0.0}});
  EXPECT_EQ(b.goal(), At({0.0, 1.5}, "path"));
  EXPECT_NEAR(std::abs(b.yaw()), M_PI, 1e-9);  // the stretch after the corner, the line robot -> goal is pi/2
}

TEST(Bridge, AGoalOnFarPlannersRouteFacesTheRoute) {
  Bridge b;
  b.robot(2.0, 0.0);
  b.farRoute({{0, 0}, {3, 0}, {3, 5}});
  b.obstacles();
  EXPECT_EQ(b.goal(), At({3.0, 0.0}, "path"));
  EXPECT_NEAR(b.yaw(), M_PI / 2, 1e-9);
}

TEST(Bridge, AMidpointGoalFacesTheWayPoint) {
  Bridge b;
  b.robot(0.0, 0.0, 2.5);
  b.wayPoint(0.0, 4.0);
  b.obstacles();
  EXPECT_EQ(b.goal(), At({0.0, 2.0}, "midpoint"));
  EXPECT_NEAR(b.yaw(), M_PI / 2, 1e-9);
}

TEST(Bridge, ADetourGoalFacesTheWayFromTheRobotToIt) {
  Bridge b;
  b.robot(0.0, 0.0);
  b.farRoute({{0, 0}, {10, 0}});
  b.obstacles({{0.5, 0.0}});
  const auto [position, source] = b.goal();
  EXPECT_EQ(source, "detour");
  EXPECT_NEAR(b.yaw(), std::atan2(position.second, position.first), 0.01);
}

TEST(Bridge, AStoppedRobotKeepsItsOwnYaw) {
  Bridge b;
  b.robot(0.0, 0.0, 1.0);
  b.farRoute({{0, 0}, {10, 0}});
  b.ring();
  EXPECT_EQ(b.goal(), At({0.0, 0.0}, "stop"));
  EXPECT_NEAR(b.yaw(), 1.0, 1e-9);
}

TEST(Bridge, ARobotAtTheEndOfFarPlannersRouteKeepsItsOwnYaw) {
  Bridge b;
  b.robot(1.0, 1.0, -2.0);
  b.farRoute({{1, 1}, {1, 1}});  // far_planner's route once the robot is at its goal
  b.obstacles();
  EXPECT_EQ(b.goal(), At({1.0, 1.0}, "path"));
  EXPECT_NEAR(b.yaw(), -2.0, 1e-9);
}

TEST(Bridge, AYawChangeAloneIsPublishedWhenLargerThanRepublishYaw) {
  Bridge b;
  b.farRoute({{0, 0}, {10, 0}});
  b.ring();
  b.robot(0.0, 0.0, 1.0);
  EXPECT_TRUE(b.tick());
  b.robot(0.0, 0.0, 1.05);  // a standing robot's yaw noise: same goal
  EXPECT_FALSE(b.tick());
  b.robot(0.0, 0.0, 1.3);
  EXPECT_TRUE(b.tick());
  EXPECT_NEAR(b.yaw(), 1.3, 1e-9);
}

// ---------------------------------------------------------------- goal yaw hint (exit inspection viewpoints)

TEST(Hint, AGoalAtTheHintTakesItsYawInsteadOfTheRouteDirection) {
  Bridge b;
  b.robot(0.0, 0.0);
  b.farRoute({{0, 0}, {1.5, 0}});  // the end of the route is within the lookahead: the goal is the viewpoint
  b.obstacles();
  EXPECT_EQ(b.goal(), At({1.5, 0.0}, "path"));
  EXPECT_NEAR(b.yaw(), 0.0, 1e-9);
  b.hint(1.5, 0.0, M_PI / 2);  // face the door on the left
  b.goal();
  EXPECT_NEAR(b.yaw(), M_PI / 2, 1e-9);
}

TEST(Hint, AHintFarFromTheGoalChangesNothing) {
  Bridge b;
  b.robot(0.0, 0.0);
  b.farRoute({{0, 0}, {10, 0}});
  b.obstacles();
  b.hint(10.0, 0.0, M_PI / 2);
  EXPECT_EQ(b.goal(), At({2.0, 0.0}, "path"));
  EXPECT_NEAR(b.yaw(), 0.0, 1e-9);
}

TEST(Hint, StandingAtTheReachedViewpointTheRobotTurnsToTheHint) {
  Bridge b;
  b.robot(5.0, 0.0, 0.3);
  b.wayPoint(5.0, 0.0);  // far_planner reached the goal: no route, /way_point is the goal
  b.obstacles();
  b.goal();
  EXPECT_NEAR(b.yaw(), 0.3, 1e-9);  // without a hint: the robot's own yaw
  b.hint(5.0, 0.0, -2.0);
  b.goal();
  EXPECT_NEAR(b.yaw(), -2.0, 1e-9);
  b.clearHint();
  b.goal();
  EXPECT_NEAR(b.yaw(), 0.3, 1e-9);
}
