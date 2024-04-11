#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ebot_docking.srv import DockSw, UnDockSw
from sensor_msgs.msg import Imu, Range
from tf_transformations import euler_from_quaternion
import math

class DockingClientNode(Node):

    def __init__(self):
        super().__init__('docking_client')
        self.dock_client = self.create_client(DockSw, 'dock_control')
        self.undock_client = self.create_client(UnDockSw, 'undock_control')

        while not self.dock_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")

        while not self.undock_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")        

        # Subscribe to IMU and ultrasonic topics
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.ultrasonic_rl_sub = self.create_subscription(Range, '/ultrasonic_rl/scan', self.ultrasonic_rl_callback, 10)
        self.ultrasonic_rr_sub = self.create_subscription(Range, '/ultrasonic_rr/scan', self.ultrasonic_rr_callback, 10)
        
        # Initialize variables to store data
        self.imu_data = 0.0
        self.ultrasonic_rl_data = 0.0
        self.ultrasonic_rr_data = 0.0


    def imu_callback(self, msg):
        # Extract and process IMU data (orientation)
        quaternion_array = msg.orientation
        orientation_list = [quaternion_array.x, quaternion_array.y, quaternion_array.z, quaternion_array.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.imu_data = yaw


    def ultrasonic_rl_callback(self, msg):
        self.ultrasonic_rl_data = msg.range


    def ultrasonic_rr_callback(self, msg):
        self.ultrasonic_rr_data = msg.range

    # def send_docking_request(self, linear_dock, orientation_dock, distance, orientation, rack_no):


    def send_docking_request(self, rack_name):
        # Check if the sensor data is available before using it
        self.get_logger().info("Docking Started ...")
        # if self.ultrasonic_rl_data is not None and self.ultrasonic_rr_data is not None and self.imu_data is not None:
        #Calculating Distance for docking
        # ultrasonic_difference = self.ultrasonic_rl_data - self.ultrasonic_rr_data
        # print("ur_rl_data: ", self.ultrasonic_rl_data)
        # print("ur_rr_data: ", self.ultrasonic_rr_data)
        # print("ultrasonic_difference: ", ultrasonic_difference)
            
        # #Calculating the Angular distance
        # ebot_yaw = self.imu_data
        # rack_yaw = 3.14  # Known orientation of Rack1
        # # Calculating angular difference (normalized to -π to π)
        # angular_difference = ebot_yaw - rack_yaw
        # print("ebot_yaw: ", ebot_yaw)
        # print("rack_yaw: ", rack_yaw)
        # print("angular_difference: ", angular_difference)
        # angular_difference = (angular_difference + math.pi) % (2 * math.pi) - math.pi
        # print("normalised angular_difference: ", angular_difference)

        #Sending request for docking
        request = DockSw.Request()
        request.linear_dock = True
        request.orientation_dock = True
        request.distance = self.ultrasonic_rl_data
        request.orientation = self.imu_data
        request.rack_no = rack_name

        future = self.dock_client.call_async(request)
        self.get_logger().info("After Call async ...")
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info("Docking service call was successful: %s" % future.result().message)
        else:
            self.get_logger().info("Docking service call failed")


    def send_undocking_request(self, rack_name):
        # Check if the sensor data is available before using it
        self.get_logger().info("Undocking Started ...")
        # if self.ultrasonic_rl_data is not None and self.ultrasonic_rr_data is not None and self.imu_data is not None:
        #Calculating Distance for docking
        # ultrasonic_difference = self.ultrasonic_rl_data - self.ultrasonic_rr_data
        # print("ur_rl_data: ", self.ultrasonic_rl_data)
        # print("ur_rr_data: ", self.ultrasonic_rr_data)
        # print("ultrasonic_difference: ", ultrasonic_difference)
            
        # #Calculating the Angular distance
        # ebot_yaw = self.imu_data
        # rack_yaw = 3.14  # Known orientation of Rack1
        # # Calculating angular difference (normalized to -π to π)
        # angular_difference = ebot_yaw - rack_yaw
        # print("ebot_yaw: ", ebot_yaw)
        # print("rack_yaw: ", rack_yaw)
        # print("angular_difference: ", angular_difference)
        # angular_difference = (angular_difference + math.pi) % (2 * math.pi) - math.pi
        # print("normalised angular_difference: ", angular_difference)

        #Sending request for docking
        request = UnDockSw.Request()
        request.linear_dock = True
        request.orientation_dock = True
        request.distance = self.ultrasonic_rl_data
        request.orientation = self.imu_data
        request.rack_no = rack_name

        future = self.undock_client.call_async(request)
        self.get_logger().info("After Call async ...")
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info("Undocking service call was successful: %s" % future.result().message)
        else:
            self.get_logger().info("Undocking service call failed")



def main(args=None):
    rclpy.init(args=args)
    docking_client_node = DockingClientNode()
    rclpy.spin(docking_client_node)
    docking_client_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()