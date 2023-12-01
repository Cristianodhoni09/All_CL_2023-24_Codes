#!/usr/bin/env python3
"""
Example of using MoveIt 2 Servo to perform a circular motion.
`ros2 run pymoveit2 ex_servo.py`
"""
from pymoveit2 import MoveIt2Servo

from math import cos, sin
import math, time
from copy import deepcopy
import rclpy
import tf2_ros
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from pymoveit2.robots import ur5
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

# Initialize message based on passed arguments

linear_speed = 1.0
angular_speed = 1.0

__twist_msg = TwistStamped()
__twist_msg.header.frame_id = ur5.base_link_name()
__twist_msg.twist.linear.x = linear_speed
__twist_msg.twist.linear.y = linear_speed
__twist_msg.twist.linear.z = linear_speed
__twist_msg.twist.angular.x = angular_speed
__twist_msg.twist.angular.y = angular_speed
__twist_msg.twist.angular.z = angular_speed

def main():
    rclpy.init()

    # Create node for this example
    node = Node("ex_servo")

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    twist_pub = node.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)

    linear = [0.35, 0.1, 0.68]
    angular = [0.0, 0.0, 0.0]

    def servo_circular_motion():
        """Move the servo in circular motion"""
        now_sec = node.get_clock().now().nanoseconds * 1e-9

        twist_msg = deepcopy(__twist_msg)
        twist_msg.header.stamp = node.get_clock().now().to_msg()
        twist_msg.twist.linear.x *= linear[0]
        twist_msg.twist.linear.y *= linear[1]
        twist_msg.twist.linear.z *= linear[2]
        twist_msg.twist.angular.x *= angular[0]
        twist_msg.twist.angular.y *= angular[1]
        twist_msg.twist.angular.z *= angular[2]

        twist_pub.publish(twist_msg)

    # Create timer for moving in a circular motion
    node.create_timer(0.2, servo_circular_motion)

    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor.spin()

    rclpy.shutdown()
    exit(0)

if __name__ == "__main__":
    main()