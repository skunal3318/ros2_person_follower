#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PointStamped, Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def clamp(value, limit):
    return max(-limit, min(limit, value))


class ControllerNode(Node):

    def __init__(self):
        super().__init__('controller_node')

        self.declare_parameter('target_distance', 1.5)
        self.declare_parameter('kp_linear', 0.5)
        self.declare_parameter('kp_angular', 1.2)
        self.declare_parameter('max_linear_vel', 0.6)
        self.declare_parameter('max_angular_vel', 1.5)
        self.declare_parameter('detection_timeout_sec', 0.8)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('publish_odom', True)
        self.declare_parameter('search_angular_vel', 0.5)
        self.declare_parameter('search_reverse_interval_sec', 4.0)

        self.target_distance = self.get_parameter('target_distance').value
        self.kp_linear = self.get_parameter('kp_linear').value
        self.kp_angular = self.get_parameter('kp_angular').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.detection_timeout_sec = self.get_parameter('detection_timeout_sec').value
        self.publish_odom = self.get_parameter('publish_odom').value
        self.search_angular_vel = self.get_parameter('search_angular_vel').value
        self.search_reverse_interval_sec = self.get_parameter('search_reverse_interval_sec').value
        control_rate_hz = self.get_parameter('control_rate_hz').value

        self.last_position = None
        self.last_position_time = None
        self.last_teleop = Twist()
        self.last_teleop_time = None
        self.manual_override = False
        self.last_bearing_sign = 1.0
        self.node_start_time = self.get_clock().now()

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_odom_time = self.get_clock().now()

        self.position_subscriber = self.create_subscription(
            PointStamped, '/person/position', self.position_callback, 10)
        self.teleop_subscriber = self.create_subscription(
            Twist, '/cmd_vel_teleop', self.teleop_callback, 10)
        self.override_subscriber = self.create_subscription(
            Bool, '/manual_override', self.override_callback, 10)

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        if self.publish_odom:
            self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
            self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(1.0 / control_rate_hz, self.control_loop)

    def position_callback(self, msg):
        self.last_position = msg.point
        self.last_position_time = Time.from_msg(msg.header.stamp)

    def teleop_callback(self, msg):
        self.last_teleop = msg
        self.last_teleop_time = self.get_clock().now()

    def override_callback(self, msg):
        self.manual_override = msg.data

    def control_loop(self):
        now = self.get_clock().now()
        cmd = Twist()

        if self.manual_override and self._is_fresh(self.last_teleop_time, now, 1.0):
            cmd = self.last_teleop
        elif self._is_fresh(self.last_position_time, now, self.detection_timeout_sec):
            distance = math.hypot(self.last_position.x, self.last_position.y)
            bearing = math.atan2(self.last_position.y, self.last_position.x)
            heading_gate = max(0.0, math.cos(bearing))
            linear_cmd = self.kp_linear * (distance - self.target_distance) * heading_gate
            cmd.linear.x = clamp(linear_cmd, self.max_linear_vel)
            cmd.angular.z = clamp(self.kp_angular * bearing, self.max_angular_vel)
            self.last_bearing_sign = 1.0 if bearing >= 0.0 else -1.0
        else:
            cmd.angular.z = self._search_angular_vel(now)

        self.cmd_vel_publisher.publish(cmd)

        if self.publish_odom:
            self.integrate_odometry(cmd, now)

    def _is_fresh(self, stamp, now, timeout_sec):
        if stamp is None:
            return False
        return (now - stamp) < rclpy.duration.Duration(seconds=timeout_sec)

    def _search_angular_vel(self, now):
        reference = self.last_position_time or self.node_start_time
        elapsed_sec = (now - reference).nanoseconds / 1e9
        phase = int(elapsed_sec // self.search_reverse_interval_sec)
        direction = self.last_bearing_sign if phase % 2 == 0 else -self.last_bearing_sign
        return direction * self.search_angular_vel

    def integrate_odometry(self, cmd, now):
        dt = (now - self.last_odom_time).nanoseconds / 1e9
        self.last_odom_time = now
        if dt <= 0.0:
            return

        v = cmd.linear.x
        w = cmd.angular.z
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += w * dt

        qx, qy, qz, qw = yaw_to_quaternion(self.theta)
        stamp = now.to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.odom_publisher.publish(odom)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
