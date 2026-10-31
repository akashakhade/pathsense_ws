#!/usr/bin/env python3
"""Topological (node + edge) route following with live SLAM.

Draws the route graph in RViz (/topo_markers), plans the shortest route on the graph
(Dijkstra), shows it (/topo_route) and drives it node by node with Nav2
(NavigateThroughPoses). Run together with slam_nav.launch.py, so the map is built
while the robot drives.

    ros2 run pathsense_description graph_navigator.py               # interactive
    ros2 run pathsense_description graph_navigator.py --room 125    # one or more rooms
    ros2 run pathsense_description graph_navigator.py --tour        # every room, back to lobby

Interactive mode: type a room number (e.g. 125), 'tour', 'lobby', 'stop', 'list' or 'q',
click near a room node in RViz with the "Publish Point" tool, or use the phone remote
(phone_remote.launch.py), which sends the same commands on /robot_command.
Status for the phone is published on /robot_status, the room list on /robot_rooms.
"""
import argparse
import heapq
import json
import math
import os
import select
import sys
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Path
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA, Float64, String
import tf2_ros
from rclpy.time import Time
from visualization_msgs.msg import Marker, MarkerArray

ARM_FOLDED = 3.0
PAUSE_IN_ROOM = 2.0

C_EDGE = ColorRGBA(r=0.62, g=0.52, b=1.0, a=1.0)       # purple edges
C_NODE = ColorRGBA(r=0.0, g=0.85, b=0.1, a=1.0)        # green nodes
C_SMALL = ColorRGBA(r=0.0, g=0.55, b=0.1, a=1.0)       # corridor / door nodes
C_VISITED = ColorRGBA(r=0.15, g=0.45, b=1.0, a=1.0)    # blue = visited room
C_TARGET = ColorRGBA(r=1.0, g=0.8, b=0.0, a=1.0)       # yellow = current goal
C_TEXT = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)       # white labels (dark RViz background)


class GraphNavigator(BasicNavigator):
    def __init__(self, graph_file):
        super().__init__(node_name='graph_navigator')
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        g = yaml.safe_load(open(graph_file))
        self.nodes = {str(k): v for k, v in g['nodes'].items()}
        self.adj = {n: [] for n in self.nodes}
        for a, b in g['edges']:
            a, b = str(a), str(b)
            d = self.dist(a, b)
            self.adj[a].append((b, d))
            self.adj[b].append((a, d))
        self.rooms = sorted([n for n, v in self.nodes.items() if v['type'] == 'room'])
        self.current = 'lobby'
        self.visited = set()
        self.target = None
        self.last_xy = (0.0, 0.0)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.marker_pub = self.create_publisher(MarkerArray, '/topo_markers', latched)
        self.route_pub = self.create_publisher(Path, '/topo_route', latched)
        self.arm_l = self.create_publisher(Float64, '/arm_left/cmd_pos', 10)
        self.arm_r = self.create_publisher(Float64, '/arm_right/cmd_pos', 10)
        self.clicks = []
        self.create_subscription(PointStamped, '/clicked_point', lambda m: self.clicks.append(m), 10)

        # phone remote / external commands and status
        self.commands = []
        self.abort = False
        self.status_text = 'Idle: at lobby'
        self.last_phone = 0.0
        self.create_subscription(String, '/robot_command', lambda m: self.commands.append(m.data.strip()), 10)
        self.create_subscription(Twist, '/cmd_vel_phone', self.on_phone, 10)
        self.status_pub = self.create_publisher(String, '/robot_status', 10)
        self.rooms_pub = self.create_publisher(String, '/robot_rooms', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_timer(1.0, self.publish_status)

    # ------------------------------------------------------------ status / pose
    def on_phone(self, msg):
        if abs(msg.linear.x) > 1e-3 or abs(msg.angular.z) > 1e-3:
            self.last_phone = time.time()

    def set_status(self, text):
        self.status_text = text
        self.publish_status()

    def publish_status(self):
        text = self.status_text
        if time.time() - self.last_phone < 1.0:
            text = 'Manual: phone driving' + (' (automation paused)' if self.target else '')
        self.status_pub.publish(String(data=text))
        self.rooms_pub.publish(String(data=json.dumps(['lobby'] + self.rooms)))

    def robot_xy(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', Time())
            return t.transform.translation.x, t.transform.translation.y
        except Exception:
            return None

    # ------------------------------------------------------------ graph
    def xy(self, n):
        return self.nodes[n]['x'], self.nodes[n]['y']

    def dist(self, a, b):
        (ax, ay), (bx, by) = self.xy(a), self.xy(b)
        return math.hypot(bx - ax, by - ay)

    def dijkstra(self, start):
        best = {start: 0.0}
        prev = {}
        pq = [(0.0, start)]
        while pq:
            d, u = heapq.heappop(pq)
            if d > best.get(u, 1e18):
                continue
            for v, w in self.adj[u]:
                nd = d + w
                if nd < best.get(v, 1e18):
                    best[v], prev[v] = nd, u
                    heapq.heappush(pq, (nd, v))
        return best, prev

    def route(self, start, goal):
        best, prev = self.dijkstra(start)
        if goal not in best:
            return None, None
        path = [goal]
        while path[-1] != start:
            path.append(prev[path[-1]])
        return path[::-1], best[goal]

    def nearest(self, x, y, kinds=None):
        cands = [n for n, v in self.nodes.items() if kinds is None or v['type'] in kinds]
        return min(cands, key=lambda n: math.hypot(self.nodes[n]['x'] - x, self.nodes[n]['y'] - y))

    def tour_order(self, rooms, start='lobby'):
        """Nearest-neighbour + 2-opt on graph distances, closed back to start."""
        pts = [start] + list(rooms)
        D = {p: self.dijkstra(p)[0] for p in pts}
        order = [start]
        left = set(rooms)
        while left:
            nxt = min(left, key=lambda r: D[order[-1]].get(r, 1e9))
            order.append(nxt)
            left.remove(nxt)

        def length(o):
            return sum(D[o[i]][o[i + 1]] for i in range(len(o) - 1)) + D[o[-1]][start]
        best, improved = length(order), True
        while improved:
            improved = False
            for i in range(1, len(order) - 1):
                for k in range(i + 1, len(order)):
                    new = order[:i] + order[i:k + 1][::-1] + order[k + 1:]
                    L = length(new)
                    if L < best - 1e-6:
                        order, best, improved = new, L, True
        return order[1:], best

    # ------------------------------------------------------------ RViz
    def publish_graph(self):
        now = self.get_clock().now().to_msg()
        ma = MarkerArray()
        clear = Marker(action=Marker.DELETEALL)
        ma.markers.append(clear)

        edges = Marker(type=Marker.LINE_LIST, ns='edges', id=0, action=Marker.ADD)
        edges.header.frame_id, edges.header.stamp = 'map', now
        edges.scale.x = 0.10
        edges.color = C_EDGE
        edges.pose.orientation.w = 1.0
        seen = set()
        for a, nb in self.adj.items():
            for b, _ in nb:
                if (b, a) in seen:
                    continue
                seen.add((a, b))
                for n in (a, b):
                    x, y = self.xy(n)
                    edges.points.append(Point(x=x, y=y, z=0.05))
        ma.markers.append(edges)

        for i, (n, v) in enumerate(self.nodes.items()):
            big = v['type'] in ('room', 'lobby')
            m = Marker(type=Marker.SPHERE, ns='nodes', id=i, action=Marker.ADD)
            m.header.frame_id, m.header.stamp = 'map', now
            m.pose.position.x, m.pose.position.y, m.pose.position.z = v['x'], v['y'], 0.1
            m.pose.orientation.w = 1.0
            s = 0.55 if big else 0.25
            m.scale.x = m.scale.y = m.scale.z = s
            m.color = C_TARGET if n == self.target else C_VISITED if n in self.visited else C_NODE if big else C_SMALL
            ma.markers.append(m)
            if big:
                t = Marker(type=Marker.TEXT_VIEW_FACING, ns='labels', id=i, action=Marker.ADD)
                t.header.frame_id, t.header.stamp = 'map', now
                t.pose.position.x, t.pose.position.y, t.pose.position.z = v['x'], v['y'] + 0.6, 0.6
                t.pose.orientation.w = 1.0
                t.scale.z = 0.7
                t.color = C_TEXT
                t.text = n
                ma.markers.append(t)
        self.marker_pub.publish(ma)

    def publish_route(self, path_nodes):
        p = Path()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        for a, b in zip(path_nodes, path_nodes[1:]):
            (ax, ay), (bx, by) = self.xy(a), self.xy(b)
            steps = max(1, int(math.hypot(bx - ax, by - ay) / 0.25))
            yaw = math.atan2(by - ay, bx - ax)
            for k in range(steps + 1):
                ps = PoseStamped()
                ps.header = p.header
                ps.pose.position.x = ax + (bx - ax) * k / steps
                ps.pose.position.y = ay + (by - ay) * k / steps
                ps.pose.position.z = 0.08
                ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
                p.poses.append(ps)
        self.route_pub.publish(p)

    # ------------------------------------------------------------ driving
    def fold_arms(self):
        # wait (up to 15 s) until the Gazebo bridge is listening, otherwise the command is lost
        t_end = time.time() + 15.0
        while (self.arm_l.get_subscription_count() == 0 or self.arm_r.get_subscription_count() == 0) \
                and time.time() < t_end:
            rclpy.spin_once(self, timeout_sec=0.1)
        for _ in range(20):
            self.arm_l.publish(Float64(data=ARM_FOLDED))
            self.arm_r.publish(Float64(data=ARM_FOLDED))
            rclpy.spin_once(self, timeout_sec=0.1)

    def go_to(self, goal):
        pos = self.robot_xy()
        if pos is not None:                       # start from wherever the robot really is
            self.current = self.nearest(*pos)
        path_nodes, length = self.route(self.current, goal)
        if path_nodes is None:
            self.get_logger().error(f'No route to {goal}')
            return False
        if len(path_nodes) < 2:
            self.get_logger().info(f'Already at {goal}')
            return True
        self.target = goal
        self.publish_graph()
        self.publish_route(path_nodes)
        hops = ' -> '.join(n for n in path_nodes if self.nodes[n]['type'] in ('room', 'lobby'))
        self.get_logger().info(f'Route {self.current} -> {goal}: {len(path_nodes)} nodes, {length:.1f} m  ({hops})')

        poses = []
        stamp = self.get_clock().now().to_msg()
        for a, b in zip(path_nodes, path_nodes[1:]):
            (ax, ay), (bx, by) = self.xy(a), self.xy(b)
            yaw = math.atan2(by - ay, bx - ax)
            ps = PoseStamped()
            ps.header.frame_id, ps.header.stamp = 'map', stamp
            ps.pose.position.x, ps.pose.position.y = bx, by
            ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            poses.append(ps)

        self.goThroughPoses(poses)
        last = 0.0
        self.set_status(f'Going to {goal}: {length:.1f} m')
        while not self.isTaskComplete():
            if 'stop' in self.commands:
                self.commands = [c for c in self.commands if c != 'stop']
                self.cancelTask()
                self.abort = True
                self.target = None
                self.publish_graph()
                self.set_status('Stopped: automation cancelled')
                self.get_logger().warn('Stop command received, route cancelled')
                return False
            fb = self.getFeedback()
            if fb is not None:
                cp = getattr(fb, 'current_pose', None)
                if cp is not None:
                    self.last_xy = (cp.pose.position.x, cp.pose.position.y)
                self.status_text = f'Going to {goal}: {fb.distance_remaining:.1f} m left'
                if time.time() - last > 3.0:
                    left = getattr(fb, 'number_of_poses_remaining', '?')
                    self.get_logger().info(f'  to {goal}: {fb.distance_remaining:5.1f} m, {left} nodes left')
                    last = time.time()
        result = self.getResult()
        if result == TaskResult.SUCCEEDED:
            self.current = goal
            if self.nodes[goal]['type'] == 'room':
                self.visited.add(goal)
            self.target = None
            self.publish_graph()
            self.set_status(f'Idle: at {goal}')
            return True
        self.current = self.nearest(*self.last_xy)
        self.target = None
        self.publish_graph()
        self.get_logger().warn(f'Could not reach {goal} ({result}); robot is near node {self.current}')
        self.set_status(f'Failed: could not reach {goal}')
        return False

    def visit(self, rooms):
        failed = []
        self.abort = False
        for i, r in enumerate(rooms, 1):
            if self.abort:
                break
            self.get_logger().info(f'[{i}/{len(rooms)}] Room {r}')
            if self.go_to(r):
                self.get_logger().info(f'  inside room {r}')
                t_end = time.time() + PAUSE_IN_ROOM
                while time.time() < t_end:
                    rclpy.spin_once(self, timeout_sec=0.1)
            else:
                failed.append(r)
        return failed

    def tour(self):
        order, length = self.tour_order(self.rooms, self.current)
        self.get_logger().info(f'Tour of {len(order)} rooms, about {length:.0f} m on the graph: {" ".join(order)}')
        t0 = time.time()
        failed = self.visit(order)
        if self.abort:
            return
        self.go_to('lobby')
        self.get_logger().info(f'Tour finished in {(time.time() - t0) / 60:.1f} min. '
                               f'Visited {len(self.visited)}/{len(order)} rooms.')
        if failed:
            self.get_logger().warn('Could not reach: ' + ', '.join(failed))

    def handle_command(self, cmd):
        cmd = cmd.strip()
        if not cmd:
            return True
        if cmd in ('q', 'quit', 'exit'):
            return False
        if cmd == 'stop':
            self.set_status('Idle: nothing to stop')
        elif cmd == 'list':
            print('Rooms:', ' '.join(self.rooms))
        elif cmd == 'tour':
            self.tour()
        elif cmd in self.nodes and self.nodes[cmd]['type'] in ('room', 'lobby'):
            self.visit([cmd]) if cmd != 'lobby' else self.go_to('lobby')
        else:
            print(f'Unknown "{cmd}". Type a room number, tour, lobby, list or q.')
        print('\nRoom number / tour / lobby / list / q  (or click a node in RViz with "Publish Point"): ', end='', flush=True)
        return True

    def interactive(self):
        print('\nRoom number / tour / lobby / list / q  (or click a node in RViz with "Publish Point"): ', end='', flush=True)
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.clicks:
                c = self.clicks.pop()
                self.clicks.clear()
                n = self.nearest(c.point.x, c.point.y, kinds=('room', 'lobby'))
                if math.hypot(self.nodes[n]['x'] - c.point.x, self.nodes[n]['y'] - c.point.y) < 3.0:
                    print(f'\nClicked: {n}')
                    self.handle_command(n)
            while self.commands:
                c = self.commands.pop(0)
                print(f'\nPhone command: {c}')
                self.handle_command(c)
            ready, _, _ = select.select([sys.stdin], [], [], 0.0)
            if ready:
                line = sys.stdin.readline()
                if not line or not self.handle_command(line):
                    break


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--room', nargs='*', help='room number(s) to visit in this order')
    ap.add_argument('--tour', action='store_true', help='visit every room, then return to the lobby')
    ap.add_argument('--graph', default=os.path.join(
        get_package_share_directory('pathsense_description'), 'config', 'college_graph.yaml'))
    args, _ = ap.parse_known_args()

    rclpy.init()
    nav = GraphNavigator(args.graph)
    nav.publish_graph()
    nav.get_logger().info(f'Graph: {len(nav.nodes)} nodes, {len(nav.rooms)} rooms. Waiting for SLAM + Nav2 ...')
    nav.fold_arms()
    nav.waitUntilNav2Active(localizer='robot_localization')   # SLAM provides map->odom, no AMCL to wait for
    nav.publish_graph()
    nav.get_logger().info('Nav2 is active.')
    try:
        if args.tour:
            nav.tour()
        elif args.room:
            bad = [r for r in args.room if r not in nav.nodes]
            if bad:
                raise SystemExit(f'Unknown room(s): {bad}')
            nav.visit(args.room)
        else:
            nav.interactive()
    except KeyboardInterrupt:
        nav.cancelTask()
    finally:
        nav.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
