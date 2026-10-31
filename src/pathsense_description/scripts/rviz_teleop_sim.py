#!/usr/bin/env python3
"""Tiny kinematic simulator so the robot can be driven in RViz without Gazebo.

Subscribes : /cmd_vel (geometry_msgs/Twist), /arm_left/cmd_pos, /arm_right/cmd_pos (std_msgs/Float64)
Publishes  : /joint_states, TF odom -> base_footprint, /odom
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster

TRACK_R = 0.089          # outer track radius (m)
TRACK_SEP = 0.251        # distance between track centres (m)
ARM_SPEED = 1.0          # rad/s
ARM_LIMITS = (-0.60, 3.00)
CMD_TIMEOUT = 0.5        # s without /cmd_vel -> stop

LEFT_WHEELS = ['wheel_front_left_joint', 'wheel_rear_left_joint',
               'support_front_left_joint', 'support_rear_left_joint']
RIGHT_WHEELS = ['wheel_front_right_joint', 'wheel_rear_right_joint',
                'support_front_right_joint', 'support_rear_right_joint']


class RvizTeleopSim(Node):
    def __init__(self):
        super().__init__('rviz_teleop_sim')
        self.x = self.y = self.yaw = 0.0
        self.v = self.w = 0.0
        self.last_cmd = self.get_clock().now()
        self.left_angle = self.right_angle = 0.0
        self.arm = {'arm_left_joint': 0.40, 'arm_right_joint': 0.40}
        self.arm_target = dict(self.arm)

        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.create_subscription(Float64, '/arm_left/cmd_pos', lambda m: self.on_arm('arm_left_joint', m), 10)
        self.create_subscription(Float64, '/arm_right/cmd_pos', lambda m: self.on_arm('arm_right_joint', m), 10)
        self.js_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf = TransformBroadcaster(self)
        self.dt = 0.02
        self.create_timer(self.dt, self.step)
        self.get_logger().info('RViz teleop sim ready: publish /cmd_vel and /arm_*/cmd_pos')

    def on_cmd(self, msg):
        self.v = max(min(msg.linear.x, 1.0), -1.0)
        self.w = max(min(msg.angular.z, 2.0), -2.0)
        self.last_cmd = self.get_clock().now()

    def on_arm(self, joint, msg):
        self.arm_target[joint] = max(min(msg.data, ARM_LIMITS[1]), ARM_LIMITS[0])

    def step(self):
        now = self.get_clock().now()
        if (now - self.last_cmd).nanoseconds * 1e-9 > CMD_TIMEOUT:
            self.v = self.w = 0.0
        dt = self.dt
        self.yaw += self.w * dt
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        v_l = self.v - self.w * TRACK_SEP / 2
        v_r = self.v + self.w * TRACK_SEP / 2
        self.left_angle += v_l / TRACK_R * dt
        self.right_angle += v_r / TRACK_R * dt
        for j in self.arm:
            err = self.arm_target[j] - self.arm[j]
            self.arm[j] += max(min(err, ARM_SPEED * dt), -ARM_SPEED * dt)

        stamp = now.to_msg()
        js = JointState()
        js.header.stamp = stamp
        js.name = LEFT_WHEELS + RIGHT_WHEELS + list(self.arm)
        js.position = [self.left_angle] * 4 + [self.right_angle] * 4 + list(self.arm.values())
        self.js_pub.publish(js)

        q_z, q_w = math.sin(self.yaw / 2), math.cos(self.yaw / 2)
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = q_z
        t.transform.rotation.w = q_w
        self.tf.sendTransform(t)

        od = Odometry()
        od.header = t.header
        od.child_frame_id = 'base_footprint'
        od.pose.pose.position.x = self.x
        od.pose.pose.position.y = self.y
        od.pose.pose.orientation.z = q_z
        od.pose.pose.orientation.w = q_w
        od.twist.twist.linear.x = self.v
        od.twist.twist.angular.z = self.w
        self.odom_pub.publish(od)


def main():
    rclpy.init()
    node = RvizTeleopSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
