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

import rclpy
from ebot_docking_client import DockingClientNode
# from link_attach_client import USBRelayAttachClientNode
# from usb_relay.srv import RelaySw
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from tf_transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from linkattacher_msgs.srv import AttachLink
from link_attach_client import AttachLinkClientNode

class DockingNode(Node):
    def __init__(self):
        super().__init__('docking_node')

        self.callback_group = ReentrantCallbackGroup()

        self.dock_client = DockingClientNode()
        # self.link_attach_client = USBRelayAttachClientNode()
        self.link_attach_client = AttachLinkClientNode()
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)

        self.timer = self.create_timer(0.1, self.dock_robot)

        self.robot_pose = [0.0, 0.0, 0.0]       

    def odometry_callback(self, msg):
        # Extract and update robot pose information from odometry message
        self.robot_pose[0] = msg.pose.pose.position.x
        self.robot_pose[1] = msg.pose.pose.position.y
        quaternion_array = msg.pose.pose.orientation
        orientation_list = [quaternion_array.x, quaternion_array.y, quaternion_array.z, quaternion_array.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.robot_pose[2] = yaw

    def dock_robot(self):
        rack_name = 'rack1'
        # Dock the robot using the docking client
        self.dock_client.send_docking_request(rack_name)

        #Attaching the Rack with USB Relaw
        # attach_request = RelaySw.Request()
        # attach_request.relaychannel = 1
        # attach_request.relaystate = True
        # self.link_attach_client.relay_attach(attach_request.relaychannel, attach_request.relaystate)

        attach_request = AttachLink.Request()
        attach_request.model1_name = 'ebot'
        attach_request.link1_name = 'ebot_base_link'
        attach_request.model2_name = rack_name # Specify the rack name
        attach_request.link2_name = 'link'
        self.link_attach_client.attach_link(attach_request.model1_name, attach_request.link1_name, attach_request.model2_name, attach_request.link2_name)

        attach_pose = self.robot_pose[0]

        msg = Twist()
        while (self.robot_pose[0] - attach_pose) < 0.5:
            msg.linear.x = 0.1
            msg.angular.z = 0.0
            self.velocity_publisher.publish(msg)

def main():
    rclpy.init()
    docking_node = DockingNode()
    rclpy.spin(docking_node)
    docking_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()