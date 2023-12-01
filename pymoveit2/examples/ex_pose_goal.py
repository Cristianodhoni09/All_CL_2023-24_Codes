#!/usr/bin/env python3
"""
Example of moving to a pose goal.
`ros2 run pymoveit2 ex_pose_goal.py --ros-args -p position:="[0.25, 0.0, 1.0]" -p quat_xyzw:="[0.0, 0.0, 0.0, 1.0]" -p cartesian:=False`
"""

from threading import Thread
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5
import time

obj1_trans=[]

class TFSubscriber(Node):
    def __init__(self):
        super().__init__('tf_subscriber')
        self.subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            10  # QoS profile depth
        )
        self.subscription  # prevent unused variable warning

    def tf_callback(self, msg):
        for transform in msg.transforms:
            if transform.header.frame_id == "base_link" and transform.child_frame_id == "obj_1":
                obj1_trans = transform.transform.translation
                obj1_rot = transform.transform.rotation
                print(obj1_rot)
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
    try:
        rclpy.init()
        node = Node("ex_pose_goal")  # Move the Node creation to this scope
        tf_subscriber = TFSubscriber()
        #rclpy.spin(tf_subscriber)

        # Create callback group that allows execution of callbacks in parallel without restriction
        movetopose(node)
       

    except Exception as e:
        if 'node' not in locals():
            rclpy.init()  # Initialize rclpy if not already done
            node = Node("ex_pose_goal")
        node.get_logger().error(f"An error occurred: {str(e)}")

    finally:
        if 'tf_subscriber' in locals():
            tf_subscriber.destroy_node()
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

def movetopose(node):
        callback_group = ReentrantCallbackGroup()
     # Create MoveIt 2 interface
        moveit2 = MoveIt2(
            node=node,
            joint_names=ur5.joint_names(),
            base_link_name=ur5.base_link_name(),
            end_effector_name=ur5.end_effector_name(),
            group_name=ur5.MOVE_GROUP_ARM,
            callback_group=callback_group,
        )

        points = [[0.35, 0.1, 0.68], [-0.37, 0.12, 0.397], [0.194, -0.43, 0.701], [-0.37, 0.12, 0.397]]
        quat = [[0.5, 0.5, 0.5, 0.5], [-0.5, 0.5, 0.5, -0.5], [0.7071068, 0, 0, 0.7071068], [-0.5, 0.5, 0.5, -0.5]]

        for i in range(0, 4):
            # Define the new position and quat_xyzw
            position = points[i]
            quat_xyzw = quat[i]
            #position=obj1_trans
            cartesian = False

            # Move to the new pose
            #node.get_logger().info(
            #    f"Moving to {{position: {list(position)}, quat_xyzw: {list(quat_xyzw)}}}"
            #)
            print(
                f"Moving to {{position: {list(position)}, quat_xyzw: {list(quat_xyzw)}}}"
            )
            moveit2.move_to_pose(position=position, quat_xyzw=quat_xyzw, cartesian=cartesian)
            # moveit2.wait_until_executed()
            time.sleep(10)


if __name__ == "__main__":
    main()
