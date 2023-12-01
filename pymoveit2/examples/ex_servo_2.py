#!/usr/bin/env python3
"""
Example of using MoveIt 2 Servo to perform a circular motion.
`ros2 run pymoveit2 ex_servo.py`
"""
from pymoveit2 import MoveIt2Servo
import geometry_msgs.msg
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf2_msgs.msg import TFMessage

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



class TFSubscriber(Node):
    def __init__(self):
        super().__init__('tf_subscriber')
        self.subscription = self.create_subscription(TFMessage, '/tf', self.tf_callback, 10)
        self.subscription  # prevent unused variable warning

    def tf_callback(self, msg):
        print('HI from callback')
        for transform in msg.transforms:
            if transform.header.frame_id == "base_link" and transform.child_frame_id == "obj_1":
                obj1_trans = transform.transform.translation
                obj1_rot = transform.transform.rotation
                # print(obj1_rot)
                print(f"Frame ID: {transform.header.frame_id}")
                print(f"Child Frame ID: {transform.child_frame_id}")
                print(f"Translation: {transform.transform.translation}")
                print(f"Rotation: {transform.transform.rotation}")

            elif transform.header.frame_id == "base_link" and transform.child_frame_id == "obj_3":
                obj3_trans = transform.transform.translation
                obj3_rot = transform.transform.rotation
                transform.transform.rotation
                print(f"Frame ID: {transform.header.frame_id}")
                print(f"Child Frame ID: {transform.child_frame_id}")
                print(f"Translation: {transform.transform.translation}")
                print(f"Rotation: {transform.transform.rotation}")

            elif transform.header.frame_id == "base_link" and transform.child_frame_id == "obj_49":
                obj49_trans = transform.transform.translation
                obj49_rot = transform.transform.rotation
                print(f"Frame ID: {transform.header.frame_id}")
                print(f"Child Frame ID: {transform.child_frame_id}")
                print(f"Translation: {transform.transform.translation}")
                print(f"Rotation: {transform.transform.rotation}")

            elif transform.header.frame_id == "base_link" and transform.child_frame_id == "tool0":
                tool0_trans = transform.transform.translation
                tool0_rot = transform.transform.rotation
                print(f"Frame ID: {transform.header.frame_id}")
                print(f"Child Frame ID: {transform.child_frame_id}")
                print(f"Translation: {transform.transform.translation}")
                print(f"Rotation: {transform.transform.rotation}")


def main():
    # Create node for this example
    try:
        rclpy.init()
        node = Node("ex_servo")
        node2 = rclpy.create_node('tf_subscriber')
        
        node2.get_logger().info('Node created: TF Subscriber')

        tf_subscriber_class = TFSubscriber()

    except Exception as e:
        if 'node' not in locals():
            rclpy.init()  # Initialize rclpy if not already done
            node = Node("ex_servo")
        node.get_logger().error(f"An error occurred: {str(e)}")

    # finally:
    #     if 'tf_subscriber' in locals():
    #         tf_subscriber.destroy_node()
    #     if 'node' in locals():
    #         node.destroy_node()
    #     rclpy.shutdown()

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    twist_pub = node.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
    

    linear = [0.0, 0.0, 0.5]
    angular = [0.0, 0.0, 0.0]

    def servo_circular_motion():
        """Move the servo in circular motion"""

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

    rclpy.spin(tf_subscriber_class)

    tf_subscriber_class.destroy_node()

    rclpy.shutdown()
    exit(0)

if __name__ == "__main__":
    main()