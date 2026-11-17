"""When far_inspection_goals.py sends the picked points to far_planner: on the start command of map_point_picker."""
import importlib.util
import math
import os

import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from visibility_graph_msg.msg import Graph
from visibility_graph_msg.msg import Node as GraphNode

os.environ["ROS_DOMAIN_ID"] = "93"  # the node publishes goals: never on the domain of a robot

_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts', 'far_inspection_goals.py')
_spec = importlib.util.spec_from_file_location('far_inspection_goals', _path)
script = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(script)


def points_file(path, *points):
    path.write_text("map: /maps/optimized_map.pcd\npoints:\n"
                    + "".join("  - {x: %.4f, y: %.4f, z: 0.0500}\n" % p for p in points))
    return str(path)


class Inspection:
    """The node with far_planner ready (graph loaded, robot localized), a clock the test moves, goals caught."""

    def __init__(self, goals_file, auto_start, graph_file=''):
        graph = ['-p', f'graph_file:={graph_file}'] if graph_file else []
        rclpy.init(args=['--ros-args', '-p', f'goals_file:={goals_file}', *graph,
                         '-p', f'auto_start:={str(auto_start).lower()}', '-p', 'dwell_time:=2.0'])
        self.node = script.FarInspectionGoals()
        self.clock = 50.0
        self.node.now = lambda: self.clock
        self.sent = []
        self.node.goal_pub.publish = lambda goal: self.sent.append((goal.point.x, goal.point.y))
        self.read, self.saved = [], []  # files handed to graph_decoder to read, and to write
        self.node.read_pub.publish = lambda msg: self.read.append(msg.data)
        self.node.save_pub.publish = lambda msg: self.saved.append(msg.data)
        self.node.decoder_ready = lambda topic: True
        self.node.graph_size = 40
        self.node.odom = Point(x=0.0, y=0.0, z=0.47)
        self.node.localization = (np.eye(3), np.zeros(3))  # localized, FAST-LIO started where the exploration did

    def localize(self, x, y, yaw):
        """A /map_to_odom match: far_planner's frame (camera_init now) at (x, y, yaw) in the map of the points."""
        msg = Odometry()
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.node.on_localization(msg)

    def start_command(self, path=''):
        self.node.on_start(String(data=path))

    def tick(self, seconds=0.2):
        self.clock += seconds
        self.node.tick()

    def reach_goal(self):
        self.node.on_reach(Bool(data=True))
        self.tick()          # reached -> dwell
        self.tick(2.1)       # dwell over -> next point, or the end


@pytest.fixture
def inspection():
    made = []

    def make(goals_file, auto_start=False, graph_file=''):
        made.append(Inspection(goals_file, auto_start, graph_file))
        return made[0]

    yield make
    for i in made:
        i.node.destroy_node()
    rclpy.try_shutdown()


def test_no_goal_is_sent_before_the_start_command(inspection, tmp_path):
    i = inspection(points_file(tmp_path / 'selected_points.yaml', (4.0, 1.0), (6.0, -2.0)))
    for _ in range(20):
        i.tick()

    assert i.sent == []
    assert i.node.state == 'idle'


def test_the_start_command_sends_the_first_point_of_the_file_it_names(inspection, tmp_path):
    i = inspection(points_file(tmp_path / 'selected_points.yaml', (4.0, 1.0)))
    other = points_file(tmp_path / 'round_two.yaml', (-3.0, 5.0), (6.0, -2.0))
    i.start_command(other)
    i.tick()

    assert i.sent == [(-3.0, 5.0)]
    assert i.node.state == 'active'


def test_the_points_are_sent_in_far_planners_frame_through_the_localization(inspection, tmp_path):
    # FAST-LIO started 1 m, 2 m and 90 deg away from where it did in the exploration: (4, 1) of the map is 3 m ahead
    # and 1 m to the right of that start, i.e. (-1, -3) in the frame far_planner plans in now
    i = inspection(points_file(tmp_path / 'selected_points.yaml', (4.0, 1.0)))
    i.localize(1.0, 2.0, math.pi / 2)
    i.start_command()
    i.tick()

    assert i.sent == [pytest.approx((-1.0, -3.0))]


def test_the_inspection_waits_for_the_first_localization_match(inspection, tmp_path):
    i = inspection(points_file(tmp_path / 'selected_points.yaml', (4.0, 1.0)))
    i.node.localization = None
    i.start_command()
    for _ in range(20):
        i.tick()
    assert i.sent == []

    i.localize(0.0, 0.0, 0.0)
    i.tick()
    assert i.sent == [(4.0, 1.0)]


def test_a_point_is_reached_where_far_planner_was_sent(inspection, tmp_path):
    i = inspection(points_file(tmp_path / 'selected_points.yaml', (4.0, 1.0), (6.0, -2.0)))
    i.localize(1.0, 2.0, math.pi / 2)
    i.start_command()
    i.tick()
    i.node.odom = Point(x=4.0, y=1.0, z=0.47)  # at the point as the file has it: not there
    i.tick()
    assert i.node.state == 'active'

    i.node.odom = Point(x=-1.0, y=-3.1, z=0.47)
    i.tick()
    assert i.node.status[0] == 'reached'


def test_a_command_without_a_file_takes_goals_file(inspection, tmp_path):
    i = inspection(points_file(tmp_path / 'selected_points.yaml', (4.0, 1.0), (6.0, -2.0)))
    i.start_command()
    i.tick()

    assert i.sent == [(4.0, 1.0)]


def test_the_points_are_the_ones_in_the_file_when_the_command_comes(inspection, tmp_path):
    path = tmp_path / 'selected_points.yaml'
    i = inspection(points_file(path, (4.0, 1.0)))
    points_file(path, (7.5, 2.5))  # picked and saved after the launch
    i.start_command()
    i.tick()

    assert i.sent == [(7.5, 2.5)]


def test_it_goes_through_the_points_then_waits_for_the_next_command(inspection, tmp_path):
    path = tmp_path / 'selected_points.yaml'
    i = inspection(points_file(path, (4.0, 1.0), (6.0, -2.0)))
    i.start_command()
    i.tick()
    i.reach_goal()
    assert i.sent == [(4.0, 1.0), (6.0, -2.0)]
    i.reach_goal()

    assert i.node.state == 'idle'  # still running: the next round needs no relaunch
    for _ in range(20):
        i.tick()
    assert i.sent == [(4.0, 1.0), (6.0, -2.0)]

    points_file(path, (1.0, 1.0))
    i.start_command()
    i.tick()
    assert i.sent[-1] == (1.0, 1.0)


def test_a_command_during_an_inspection_starts_over_with_the_new_points(inspection, tmp_path):
    path = tmp_path / 'selected_points.yaml'
    i = inspection(points_file(path, (4.0, 1.0), (6.0, -2.0)))
    i.start_command()
    i.tick()
    points_file(path, (-2.0, -2.0), (0.0, 9.0))
    i.start_command()
    i.tick()

    assert i.sent == [(4.0, 1.0), (-2.0, -2.0)]
    assert i.node.index == 0 and len(i.node.goals) == 2


def test_a_command_for_a_missing_or_empty_file_leaves_things_as_they_are(inspection, tmp_path):
    i = inspection(points_file(tmp_path / 'selected_points.yaml', (4.0, 1.0)))
    i.start_command(str(tmp_path / 'nowhere.yaml'))
    i.tick()
    assert i.sent == [] and i.node.state == 'idle'

    i.start_command(points_file(tmp_path / 'empty.yaml'))
    i.tick()
    assert i.sent == [] and i.node.state == 'idle'


def test_auto_start_inspects_goals_file_at_once_and_ends_after_the_last_point(inspection, tmp_path):
    i = inspection(points_file(tmp_path / 'selected_points.yaml', (4.0, 1.0)), auto_start=True)
    i.tick()
    assert i.sent == [(4.0, 1.0)]
    i.reach_goal()

    assert i.node.state == 'done'


def vgh_line(i, position, first=(0.0, 0.0, -1.0), second=(0.0, 0.0, -1.0)):
    """A graph_decoder line: id, free_direct, position, surface directions, four flags, empty index lists."""
    values = " ".join("%f" % v for v in (*position, *first, *second))
    return f"{i} 1 {values} 1 0 0 0 | | | \n"


def vgh_nodes(path):
    """(position, first surface direction) of each line of a .vgh file."""
    nodes = []
    for line in open(path):
        v = [float(x) for x in line.split()[2:8]]
        nodes.append((tuple(v[:3]), tuple(v[3:6])))
    return nodes


def test_a_saved_graph_is_loaded_into_far_planners_frame_before_the_first_goal(inspection, tmp_path):
    graph = tmp_path / 'warehouse.vgh'
    graph.write_text(vgh_line(0, (4.0, 1.0, 0.2)) + vgh_line(1, (4.0, 1.0, 0.2), first=(1.0, 0.0, 0.0)))
    i = inspection(points_file(tmp_path / 'selected_points.yaml', (4.0, 1.0)), graph_file=str(graph))
    i.localize(1.0, 2.0, math.pi / 2)  # FAST-LIO started 1 m, 2 m and 90 deg away from the exploration's start
    i.start_command()
    i.tick()
    assert i.sent == []  # not before far_planner has the whole graph

    assert len(i.read) == 1
    (position, pillar), (_, surface) = vgh_nodes(i.read[0])
    assert position == pytest.approx((-1.0, -3.0, 0.2), abs=1e-5)  # where (4, 1) of the map is now
    assert pillar == (0.0, 0.0, -1.0)  # exactly: far_planner tells pillars by it
    assert surface == pytest.approx((0.0, -1.0, 0.0), abs=1e-5)

    i.node.on_decoded(Graph(nodes=[GraphNode()] * 2))   # graph_decoder read the file
    i.node.on_vgraph(Graph(nodes=[GraphNode()] * 42))   # and far_planner merged it
    i.tick()
    assert i.sent == [pytest.approx((-1.0, -3.0))]


def test_without_a_saved_graph_the_inspection_starts_at_once(inspection, tmp_path):
    i = inspection(points_file(tmp_path / 'selected_points.yaml', (4.0, 1.0)),
                   graph_file=str(tmp_path / 'warehouse.vgh'))  # none yet: far_planner builds its own
    i.start_command()
    i.tick()

    assert i.read == []
    assert i.sent == [(4.0, 1.0)]


def test_after_the_last_point_the_graph_is_saved_in_the_map_frame(inspection, tmp_path):
    graph = tmp_path / 'warehouse.vgh'
    i = inspection(points_file(tmp_path / 'selected_points.yaml', (4.0, 1.0)), graph_file=str(graph))
    i.localize(1.0, 2.0, math.pi / 2)
    i.start_command()
    i.tick()
    i.reach_goal()
    assert len(i.saved) == 1 and not graph.exists()

    # graph_decoder writes far_planner's graph, in far_planner's frame
    with open(i.saved[0], 'w') as f:
        f.write(vgh_line(0, (-1.0, -3.0, 0.2), first=(0.0, -1.0, 0.0)) + vgh_line(1, (0.0, 0.0, 0.2)))
    i.tick()
    i.tick()  # its size held: it is complete

    (position, surface), (start, pillar) = vgh_nodes(graph)
    assert position == pytest.approx((4.0, 1.0, 0.2), abs=1e-5)
    assert surface == pytest.approx((1.0, 0.0, 0.0), abs=1e-5)
    assert start == pytest.approx((1.0, 2.0, 0.2), abs=1e-5)
    assert pillar == (0.0, 0.0, -1.0)
    assert not os.path.exists(i.saved[0])
