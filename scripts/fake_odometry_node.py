#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math

class FakeOdometryNode(Node):
    def __init__(self):
        super().__init__('fake_odometry_node')
        self.get_logger().info("Initializing FakeOdometryNode...")
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 1)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 5)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Timer for odometry updates
        self.timer = self.create_timer(0.1, self.publish_odometry)

    def cmd_vel_callback(self, msg):
        self.x += msg.linear.x * 0.1 * math.cos(self.theta)
        self.y += msg.linear.x * 0.1 * math.sin(self.theta)
        self.theta += msg.angular.z * 0.1

    def publish_odometry(self):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'odom'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        self.odom_pub.publish(odom)
        self.publish_transform()

    def publish_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = FakeOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
