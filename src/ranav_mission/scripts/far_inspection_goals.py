#!/usr/bin/env python3
"""
far_inspection_goals.py (ROS 2 Python)

Inspection mission: sends the points picked with map_point_picker (selected_points.yaml) to far_planner as goals,
one after the other, in the order of the file:
  0. waits for the start command: a file of points on /start_inspection (std_msgs/String; empty: goals_file), which
     the Start inspection button of map_point_picker sends with the file it has just saved. The points are read
     then, not at the launch. A command during an inspection starts over with the points of its file; a file that
     cannot be read or has no points changes nothing. By hand:
       ros2 topic pub -1 /start_inspection std_msgs/msg/String "{data: ''}"
     With auto_start there is no waiting: goals_file is inspected at once, and the node exits after the last point;
  1. waits for far_planner's v-graph (/robot_vgraph) and, with load_graph and a graph_file that exists, for that
     saved graph to be in it, so that the first route is planned on the whole graph and not only on what the robot
     sees. It is loaded once, at the start (not at the start command): converted into far_planner's frame, handed
     to graph_decoder (/read_file_dir), which passes it on to far_planner (/decoded_vgraph);
  2. publishes goal i on /goal_point and watches it:
       reached: far_planner reports /far_reach_goal_status true, or /Odometry comes within reach_tolerance (xy);
       failed:  far_planner stops publishing /way_point for idle_timeout s (it gives up on an unreachable goal
                without saying so), or the goal takes longer than goal_timeout s.
     far_planner drops goals that arrive before its v-graph is initialized, so a goal that gets no /way_point
     within ack_timeout s is published again;
  3. waits dwell_time s at the point (reached or not) and goes on to the next; with loop it starts over after the
     last one, otherwise it logs a summary and waits for the next start command;
  4. after the last point, with save_graph, saves far_planner's graph to graph_file for the next inspections:
     graph_decoder writes it (/save_file_dir) to a temporary file, which is converted into the map frame and then
     replaces graph_file. The graph so grows with every inspection.

Frames: the points are in the frame of the map they were picked on (optimized_map.pcd, i.e. camera_init of the
exploration), which fast_lio_localization calls global_map. far_planner plans in the camera_init of the FAST-LIO
running now (static camera_init -> map), which matches that map only if FAST-LIO started from the same pose as in
the exploration. So each point is brought into far_planner's frame with the latest match of the localization
(localization_topic, /map_to_odom of global_localization.py: the pose of camera_init in global_map), when its goal is
sent: p_far = R^T (p - t). The inspection starts only once the robot is localized. An empty localization_topic sends
the points as they are (then the robot must start from the pose of the exploration). graph_file is kept in the map
frame too, converted when loaded and when saved, turned about z only: far_planner tells pillars by the surface
directions (0, 0, -1), which a tilt would change.
The z of the file (ground height under the point) is not used: far_planner compares the 3D distance between
/Odometry (the lidar, about 0.3 m above the floor) and the goal with converge_distance (0.25 m), so each goal is
sent at the robot's current height.

The goals and their status are published on /inspection_goals (MarkerArray, transient local) for RViz.

Usage: ros2 run ranav_mission far_inspection_goals.py --ros-args -p goals_file:=/path/selected_points.yaml
"""

import math
import os
import tempfile

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String
from visibility_graph_msg.msg import Graph
from visualization_msgs.msg import Marker, MarkerArray

STATUS_COLORS = {  # rgba
    "pending": (0.85, 0.85, 0.85, 0.9),
    "active": (1.0, 0.31, 0.85, 1.0),
    "reached": (0.29, 0.87, 0.5, 1.0),
    "failed": (1.0, 0.36, 0.36, 1.0),
}


def quaternion_matrix(x, y, z, w):
    """Rotation matrix of a unit quaternion."""
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def transform_vgh(lines, yaw, translation, inverse=False):
    """Lines of a .vgh file (graph_decoder: id, free_direct, position, two surface directions, four flags, index
    lists) with positions and surface directions turned by yaw about z, positions shifted by translation:
    p' = R p + t, or with inverse p' = R^T (p - t). The index lists are kept as they are."""
    c, s = math.cos(yaw), math.sin(yaw)
    sign = -1.0 if inverse else 1.0
    turn = lambda x, y: (c * x - sign * s * y, sign * s * x + c * y)
    out = []
    for line in lines:
        tokens = line.split()
        if len(tokens) < 15:
            continue
        v = [float(x) for x in tokens[2:11]]
        position = np.array(v[0:3])
        if inverse:
            position = position - translation
        position[0], position[1] = turn(position[0], position[1])
        if not inverse:
            position = position + translation
        first, second = turn(v[3], v[4]), turn(v[6], v[7])
        values = (*position, first[0], first[1], v[5], second[0], second[1], v[8])
        tokens[2:11] = ["%f" % (x + 0.0) for x in values]  # + 0.0: no "-0.000000"
        out.append(" ".join(tokens) + " \n")  # graph_decoder drops the last token unless the line ends with a space
    return out


def load_goals(path):
    """Points of a map_point_picker .yaml or .json file (JSON is YAML too), as (x, y, z) tuples."""
    with open(path) as f:
        data = yaml.safe_load(f)
    points = data.get("points") if isinstance(data, dict) else data
    return [(float(p["x"]), float(p["y"]), float(p.get("z", 0.0))) for p in points or []]


class FarInspectionGoals(Node):
    def __init__(self):
        super().__init__("far_inspection_goals")
        # dynamic typing: a launch argument like dwell_time:=3 arrives as an int
        param = lambda name, default: self.declare_parameter(
            name, default, ParameterDescriptor(dynamic_typing=True)).value
        self.goals_file = os.path.abspath(os.path.expanduser(param("goals_file", "")))
        self.frame_id = param("frame_id", "map")
        # Pose of far_planner's frame in the map of the points (see the module docstring); empty: points sent as they are
        self.localization_topic = param("localization_topic", "/map_to_odom")
        # The saved FAR graph, in the map frame: loaded at the start, saved after each inspection
        graph_file = param("graph_file", "")
        self.graph_file = os.path.abspath(os.path.expanduser(graph_file)) if graph_file else ""
        load_graph = bool(param("load_graph", True)) and bool(self.graph_file)
        self.save_graph = bool(param("save_graph", True)) and bool(self.graph_file)
        self.graph_timeout = float(param("graph_timeout", 120.0))
        self.reach_tolerance = float(param("reach_tolerance", 0.3))
        self.dwell_time = float(param("dwell_time", 2.0))
        self.ack_timeout = float(param("ack_timeout", 5.0))
        self.idle_timeout = float(param("idle_timeout", 10.0))
        self.goal_timeout = float(param("goal_timeout", 600.0))
        self.loop = bool(param("loop", False))
        self.auto_start = bool(param("auto_start", False))

        self.goals = []
        self.status = []
        self.index = 0
        self.odom = None
        self.graph_size = 0          # nodes in the last /robot_vgraph
        # graph_file into far_planner: none (nothing to load) or pending -> sent (to graph_decoder) -> merging -> done
        self.graph_load = "pending" if load_graph and os.path.isfile(self.graph_file) else "none"
        self.graph_load_file = None  # the converted copy handed to graph_decoder, and when
        self.graph_load_sent = None
        self.graph_save = None       # while saving: the file graph_decoder writes, since when, its last size
        self.last_waypoint = None    # ROS time of the last /way_point
        self.reached = False         # far_planner reported the active goal reached
        self.localization = None     # (R, t) of the last localization match: p_map = R p_far + t
        self.target = None           # the active goal in far_planner's frame (x, y)
        # idle (no start command yet) -> waiting (for the graph) -> active -> dwell -> active ...
        # -> idle again after the last point (with auto_start: done)
        self.state = "idle"
        self.started = None
        self.goal_started = None     # first publication of the active goal
        self.sent_at = None          # last publication of the active goal
        self.dwell_until = None

        self.create_subscription(Odometry, "/Odometry", lambda msg: setattr(self, "odom", msg.pose.pose.position), 5)
        self.create_subscription(Graph, "/robot_vgraph", self.on_vgraph, 5)
        self.create_subscription(Graph, "/decoded_vgraph", self.on_decoded, 5)
        self.create_subscription(PointStamped, "/way_point", self.on_waypoint, 5)
        self.create_subscription(Bool, "/far_reach_goal_status", self.on_reach, 5)
        self.create_subscription(String, "/start_inspection", self.on_start, 5)
        if self.localization_topic:
            self.create_subscription(Odometry, self.localization_topic, self.on_localization, 5)
        self.goal_pub = self.create_publisher(PointStamped, "/goal_point", 5)
        self.read_pub = self.create_publisher(String, "/read_file_dir", 5)   # graph_decoder's
        self.save_pub = self.create_publisher(String, "/save_file_dir", 5)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(MarkerArray, "/inspection_goals", latched)
        if load_graph and self.graph_load == "none":
            self.get_logger().info(f"no saved FAR graph yet ({self.graph_file}): far_planner starts with its own"
                                   + (", saved after the inspection" if self.save_graph else ""))
        if self.auto_start:
            self.start(self.goals_file)
        else:
            self.get_logger().info("waiting for the start command (Start inspection in map_point_picker, "
                                   "/start_inspection)")

    # ------------------------------------------------------------ callbacks

    def on_vgraph(self, msg):
        self.graph_size = len(msg.nodes)
        if self.graph_load == "merging":  # the first v-graph after the loaded one: far_planner has it
            self.graph_load = "done"

    def on_decoded(self, msg):
        if self.graph_load == "sent":
            self.get_logger().info(f"FAR graph loaded: {len(msg.nodes)} nodes from {self.graph_file}")
            self.graph_load = "merging"
            self.remove(self.graph_load_file)

    def on_waypoint(self, msg):
        self.last_waypoint = self.now()

    def on_reach(self, msg):
        # far_planner publishes false on every planning loop with a goal, and true once, on the loop that reaches it
        if msg.data and self.state == "active":
            self.reached = True

    def on_start(self, msg):
        self.start(os.path.abspath(os.path.expanduser(msg.data.strip())) if msg.data.strip() else self.goals_file)

    def on_localization(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.localization = (quaternion_matrix(q.x, q.y, q.z, q.w), np.array([p.x, p.y, p.z]))

    def to_far_frame(self, x, y, z):
        """Point of the map (x, y) in far_planner's frame, with the last localization match."""
        if not self.localization_topic:
            return x, y
        rotation, translation = self.localization
        p = rotation.T @ (np.array([x, y, z]) - translation)
        return float(p[0]), float(p[1])

    def localized(self):
        return not self.localization_topic or self.localization is not None

    # ------------------------------------------------------------ saved graph

    def map_yaw_translation(self):
        """Yaw and translation of far_planner's frame in the map (the localization turned about z only)."""
        if not self.localization_topic:
            return 0.0, np.zeros(3)
        rotation, translation = self.localization
        return math.atan2(rotation[1, 0], rotation[0, 0]), translation

    def decoder_ready(self, topic):
        return self.count_subscribers(topic) > 0

    @staticmethod
    def remove(path):
        try:
            os.remove(path)
        except (OSError, TypeError):
            pass

    def tick_graph(self, t):
        if self.graph_load == "pending" and self.graph_size > 0 and self.localized() and \
                self.decoder_ready("/read_file_dir"):
            # far_planner is up (its first v-graph): hand it the saved graph, in its frame
            yaw, translation = self.map_yaw_translation()
            with open(self.graph_file) as f:
                lines = transform_vgh(f.readlines(), yaw, translation, inverse=True)
            fd, self.graph_load_file = tempfile.mkstemp(prefix="far_graph_", suffix=".vgh")
            with os.fdopen(fd, "w") as f:
                f.writelines(lines)
            self.read_pub.publish(String(data=self.graph_load_file))
            self.graph_load, self.graph_load_sent = "sent", t
        elif self.graph_load in ("sent", "merging") and t - self.graph_load_sent > 10.0:
            self.get_logger().warning(f"FAR graph {self.graph_file} not loaded (no answer from graph_decoder)")
            self.graph_load = "done"
            self.remove(self.graph_load_file)
        if self.graph_save is not None:
            self.tick_save(t)

    def save(self, t):
        """Asks graph_decoder for far_planner's graph; tick_save() converts it and replaces graph_file."""
        if not self.save_graph or not self.localized():
            return
        if not self.decoder_ready("/save_file_dir"):
            self.get_logger().warning("FAR graph not saved: graph_decoder is not running")
            return
        fd, path = tempfile.mkstemp(prefix="far_graph_", suffix=".vgh")
        os.close(fd)
        os.remove(path)  # graph_decoder creates it: a file that is there is its
        self.save_pub.publish(String(data=path))
        self.graph_save = {"path": path, "since": t, "size": -1}

    def tick_save(self, t):
        path = self.graph_save["path"]
        size = os.path.getsize(path) if os.path.isfile(path) else 0
        # graph_decoder writes the whole file in its callback: a non-empty file whose size holds is complete
        if size > 0 and size == self.graph_save["size"]:
            yaw, translation = self.map_yaw_translation()
            with open(path) as f:
                lines = transform_vgh(f.readlines(), yaw, translation)
            os.makedirs(os.path.dirname(self.graph_file), exist_ok=True)
            with open(self.graph_file + ".tmp", "w") as f:
                f.writelines(lines)
            os.replace(self.graph_file + ".tmp", self.graph_file)
            self.get_logger().info(f"FAR graph saved: {len(lines)} nodes to {self.graph_file}")
        elif t - self.graph_save["since"] <= 10.0:
            self.graph_save["size"] = size
            return
        else:
            self.get_logger().warning("FAR graph not saved: graph_decoder wrote nothing within 10 s")
        self.remove(path)
        self.graph_save = None

    # ------------------------------------------------------------ mission

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def start(self, path):
        """Begins the inspection of the points of path, from the first one. False, and nothing changes, when there
        are none to read."""
        if not os.path.isfile(path):
            self.get_logger().warning(f"goals file '{path}' not found: no inspection "
                                      "(far_planner still takes goals from RViz)")
            return False
        try:
            goals = load_goals(path)
        except Exception as e:
            self.get_logger().error(f"cannot read goals file '{path}': {e}")
            return False
        if not goals:
            self.get_logger().warning(f"no points in '{path}': no inspection")
            return False
        if self.state in ("waiting", "active", "dwell"):
            self.get_logger().info(f"start command during the inspection of {self.goals_file}: starting over")
        self.goals_file, self.goals = path, goals
        self.status = ["pending"] * len(goals)
        self.index = 0
        self.state, self.started = "waiting", None
        self.publish_markers()
        return True

    def send_goal(self):
        # converted at every send: a resent goal takes the latest localization match
        self.target = self.to_far_frame(*self.goals[self.index])
        goal = PointStamped()
        goal.header.frame_id = self.frame_id
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.point.x, goal.point.y, goal.point.z = self.target[0], self.target[1], self.odom.z
        self.goal_pub.publish(goal)
        self.sent_at = self.now()

    def start_goal(self):
        self.status[self.index] = "active"
        self.reached = False
        self.last_waypoint = None
        self.send_goal()
        self.goal_started = self.sent_at
        x, y, _ = self.goals[self.index]
        converted = f", ({self.target[0]:.2f}, {self.target[1]:.2f}) in {self.frame_id}" if self.localization_topic else ""
        self.get_logger().info(f"goal {self.index + 1}/{len(self.goals)}: ({x:.2f}, {y:.2f}){converted}")
        self.state = "active"
        self.publish_markers()

    def finish_goal(self, status, why):
        self.status[self.index] = status
        x, y, _ = self.goals[self.index]
        text = f"goal {self.index + 1}/{len(self.goals)} ({x:.2f}, {y:.2f}) {status} " \
               f"after {self.now() - self.goal_started:.0f} s: {why}"
        # rclpy refuses two severities from the same line of code
        if status == "reached":
            self.get_logger().info(text)
        else:
            self.get_logger().warning(text)
        self.state, self.dwell_until = "dwell", self.now() + self.dwell_time
        self.publish_markers()

    def tick(self):
        t = self.now()
        if t == 0.0:  # use_sim_time and no /clock yet
            return
        self.tick_graph(t)
        if self.state == "waiting":
            if self.started is None:
                self.started = t
            graph_ready = self.graph_size > 0 and self.graph_load in ("none", "done")
            if self.odom is None or not graph_ready or not self.localized():
                if t - self.started < self.graph_timeout:
                    return
                if self.odom is None or self.graph_size == 0 or not self.localized():
                    self.get_logger().error(f"no /Odometry, no far_planner v-graph or no localization match on "
                                            f"{self.localization_topic} after {self.graph_timeout:.0f} s: "
                                            "inspection not started")
                    self.state = "done" if self.auto_start else "idle"
                    return
                self.get_logger().warning(f"saved graph not loaded after {self.graph_timeout:.0f} s: "
                                          "starting on far_planner's own graph")
                self.graph_load = "done"
            self.get_logger().info(f"far_planner ready ({self.graph_size} nodes): inspecting {len(self.goals)} points "
                                   f"from {self.goals_file}")
            self.start_goal()
        elif self.state == "active":
            x, y = self.target
            if self.reached:
                self.finish_goal("reached", "far_planner reached the goal")
            elif self.odom is not None and math.hypot(self.odom.x - x, self.odom.y - y) < self.reach_tolerance:
                self.finish_goal("reached", f"within {self.reach_tolerance:.2f} m")
            elif t - self.goal_started > self.goal_timeout:
                self.finish_goal("failed", f"not reached within goal_timeout ({self.goal_timeout:.0f} s)")
            elif self.last_waypoint is None or self.last_waypoint < self.sent_at:
                if t - self.sent_at > self.ack_timeout:  # dropped: v-graph not initialized yet, or no subscriber yet
                    self.send_goal()
            elif t - self.last_waypoint > self.idle_timeout:
                self.finish_goal("failed", f"far_planner stopped planning (no /way_point for {self.idle_timeout:.0f} s): "
                                           "no route found")
        elif self.state == "dwell" and t >= self.dwell_until:
            self.index += 1
            if self.index == len(self.goals):
                self.log_summary()
                self.save(t)
                if not self.loop:
                    self.state = "done" if self.auto_start else "idle"
                    return
                self.index = 0
                self.status = ["pending"] * len(self.goals)
            self.start_goal()

    def log_summary(self):
        failed = [str(i + 1) for i, s in enumerate(self.status) if s == "failed"]
        reached = len(self.goals) - len(failed)
        self.get_logger().info(f"inspection finished: {reached}/{len(self.goals)} points reached"
                               + (f", failed: {', '.join(failed)}" if failed else ""))

    def publish_markers(self):
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for i, ((x, y, z), status) in enumerate(zip(self.goals, self.status)):
            if self.localized():  # where far_planner is sent; before the first match, where the file has them
                x, y = self.to_far_frame(x, y, z)
            for kind in ("sphere", "label"):
                m = Marker()
                m.header.frame_id, m.header.stamp = self.frame_id, stamp
                m.ns, m.id, m.action = "inspection_" + kind, i, Marker.ADD
                m.pose.position.x, m.pose.position.y = x, y
                m.pose.orientation.w = 1.0
                if kind == "sphere":
                    m.type = Marker.SPHERE
                    m.pose.position.z = z + 0.15
                    m.scale.x = m.scale.y = m.scale.z = 0.4 if status == "active" else 0.3
                    m.color.r, m.color.g, m.color.b, m.color.a = STATUS_COLORS[status]
                else:
                    m.type = Marker.TEXT_VIEW_FACING
                    m.pose.position.z = z + 0.7
                    m.scale.z = 0.4
                    m.text = str(i + 1)
                    m.color.r = m.color.g = m.color.b = m.color.a = 1.0
                markers.markers.append(m)
        self.marker_pub.publish(markers)


def main():
    rclpy.init()
    node = FarInspectionGoals()
    try:
        if node.auto_start and node.state == "idle":  # nothing to inspect in goals_file
            return
        while rclpy.ok() and (node.state != "done" or node.graph_save is not None):
            rclpy.spin_once(node, timeout_sec=0.2)
            node.tick()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
