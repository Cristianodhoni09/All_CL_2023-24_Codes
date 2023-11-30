#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ebot_docking_client import DockingClientNode
#from ebot_undocking_client import UnDockingClientNode
from link_attach_client import AttachLinkClientNode
from link_detach_client import DetachLinkClientNode

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from linkattacher_msgs.srv import AttachLink
from linkattacher_msgs.srv import DetachLink
import rclpy
from rclpy.duration import Duration
import math


class Nav2CMD(Node):
    def __init__(self):
        super().__init__('my_robot_nav_controller')

        self.callback_group = ReentrantCallbackGroup()

        self.navigator = BasicNavigator()
        self.dock_client = DockingClientNode()
        self.undock_client = UnDockingClientNode()
        self.link_attach_client = AttachLinkClientNode()
        self.link_detach_client = DetachLinkClientNode()

        self.timer = self.create_timer(0.1, self.run)


    def dock_and_attach_rack(self, goal_pose, rack_name):
        # Go to the pre-dock pose
        self.navigator.goToPose(goal_pose)
        while not self.navigator.isTaskComplete():
            pass

        # Dock using your client node
        self.dock_client.send_docking_request()
        # Your docking client code here

        # Attach the rack
        attach_request = AttachLink.Request()
        attach_request.model1_name = 'ebot'
        attach_request.link1_name = 'ebot_base_link'
        attach_request.model2_name = rack_name  # Specify the rack name
        attach_request.link2_name = 'link'
        self.link_attach_client.attach_link(attach_request.model1_name, attach_request.link1_name, attach_request.model2_name, attach_request.link2_name)

        # while not self.link_attach_client.service_is_ready():
        #     pass

    def pre_place_and_detach_rack(self, goal_pose, rack_name):
        # Go to the pre-place pose
        self.navigator.goToPose(goal_pose)
        while not self.navigator.isTaskComplete():
            pass

        # self.undock_client.send_undocking_request()
        # pose = PoseStamped()
        # pose.header.frame_id = 'map'
        # pose.header.stamp = self.navigator.get_clock().now().to_msg()
        # pose.pose.position.x = 0.5
        # pose.pose.position.y = -2.455
        # pose.pose.orientation.w = 0.0
        # pose.pose.orientation.z = 1.0

        # self.navigator.goToPose(pose)
        # while not self.navigator.isTaskComplete():
        #     pass
        # print("Intermediate Pose Complete!")

        # Detach the rack

        self.dock_client.send_docking_request()

        detach_request = DetachLink.Request()
        detach_request.model1_name = 'ebot'
        detach_request.link1_name = 'ebot_base_link'
        detach_request.model2_name = rack_name  # Specify the rack name
        detach_request.link2_name = 'link'
        self.link_detach_client.detach_link(detach_request.model1_name, detach_request.link1_name, detach_request.model2_name, detach_request.link2_name)

        # while not self.link_detach_client.service_is_ready():
        #     pass


    def run(self):
        # Set your initial pose
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = 0.0
        initial_pose.pose.position.y = 0.0
        initial_pose.pose.orientation.z = 1.0
        initial_pose.pose.orientation.w = 0.0
        self.navigator.setInitialPose(initial_pose)

        self.navigator.waitUntilNav2Active()

        # Define your goal poses
        goal_poses = []

        goal_pose1 = PoseStamped()
        goal_pose1.header.frame_id = 'map'
        goal_pose1.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose1.pose.position.x = 0.5
        goal_pose1.pose.position.y = 4.65
        goal_pose1.pose.orientation.w = 0.0
        goal_pose1.pose.orientation.z = 1.0
        goal_poses.append(goal_pose1)

        # additional goals can be appended
        goal_pose2 = PoseStamped()
        goal_pose2.header.frame_id = 'map'
        goal_pose2.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose2.pose.position.x = 0.8
        goal_pose2.pose.position.y = -2.455
        goal_pose2.pose.orientation.w = 1.0
        goal_pose2.pose.orientation.z = 0.0
        goal_poses.append(goal_pose2)

        # goal_pose_int = PoseStamped()
        # goal_pose_int.header.frame_id = 'map'
        # goal_pose_int.header.stamp = self.navigator.get_clock().now().to_msg()
        # goal_pose_int.pose.position.x = 0.9
        # goal_pose_int.pose.position.y = -0.5
        # goal_pose_int.pose.orientation.w = 0.0
        # goal_pose_int.pose.orientation.z = 1.0
        
        goal_pose3 = PoseStamped()
        goal_pose3.header.frame_id = 'map'
        goal_pose3.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose3.pose.position.x = 0.0
        goal_pose3.pose.position.y = 0.0
        goal_pose3.pose.orientation.w = 0.0
        goal_pose3.pose.orientation.z = 1.0
        goal_poses.append(goal_pose3)

        # ...
        rack_name = 'rack1'
        # for pose in goal_poses:
            # Dock and attach rack
        self.dock_and_attach_rack(goal_pose1, rack_name)
            # Pre-place and detach rack
        self.pre_place_and_detach_rack(goal_pose2, rack_name)

        # self.navigator.goToPose(goal_pose_int)
        # while not self.navigator.isTaskComplete():
        #     pass

        self.navigator.goToPose(goal_pose3)
        while not self.navigator.isTaskComplete():
            pass

        result = self.navigator.getResult()
        # ...

        self.navigator.lifecycleShutdown()
        exit(0)

def main():
    rclpy.init()
    nav2node = Nav2CMD()
    rclpy.spin(nav2node)
    nav2node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
