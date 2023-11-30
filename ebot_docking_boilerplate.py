#!/usr/bin/env python3

## Overview

# ###
# This ROS2 script is designed to control a robot's docking behavior with a rack. 
# It utilizes odometry data, ultrasonic sensor readings, and provides docking control through a custom service. 
# The script handles both linear and angular motion to achieve docking alignment and execution.
# ###

# Import necessary ROS2 packages and message types
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, Range
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf_transformations import euler_from_quaternion
from ebot_docking.srv import DockSw, UnDockSw  # Import custom service message
import math, statistics

# Define a class for your ROS2 node
class MyRobotDockingController(Node):

    def __init__(self):
        # Initialize the ROS2 node with a unique name
        super().__init__('my_robot_docking_controller')

        # Create a callback group for managing callbacks
        self.callback_group = ReentrantCallbackGroup()

        # Subscribe to odometry data for robot pose information
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)

        # Subscribe to ultrasonic sensor data for distance measurements
        self.ultrasonic_rl_sub = self.create_subscription(Range, '/ultrasonic_rl/scan', self.ultrasonic_rl_callback, 10)
        # Add another one here
        self.ultrasonic_rr_sub = self.create_subscription(Range, '/ultrasonic_rr/scan', self.ultrasonic_rr_callback, 10)

        #IMU sub
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)

        # Create a ROS2 service for controlling docking behavior, can add another custom service message
        self.dock_control_srv = self.create_service(DockSw, 'dock_control', self.dock_control_callback, callback_group=self.callback_group)

        # Creating a ROS2 service for controlling un-docking behaviour
        self.undock_control_srv = self.create_service(DockSw, 'undock_control', self.undock_control_callback, callback_group=self.callback_group)

        # Create a publisher for sending velocity commands to the robot
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Initialize all  flags and parameters here
        linear_dock = False
        orientation_dock = False
        distance = 0.0
        orientation = 0.0
        rack_no = None

        self.is_docking = False
        self.robot_pose = [0.0, 0.0, 0.0]  # Initialize robot pose (x, y, yaw)
        self.usrleft_value = 0.0  # Initialize ultrasonic sensor value
        self.usrright_value = 0.0
        self.dock_aligned = False  # Flag to indicate if the robot is aligned for docking 

        # Initialize a timer for the main control loop
        self.controller_timer = self.create_timer(0.1, self.controller_loop)

    #Callback fro IMU
    def imu_callback(self, msg):
        # Extract and process IMU data (orientation)
        quaternion_array = msg.orientation
        orientation_list = [quaternion_array.x, quaternion_array.y, quaternion_array.z, quaternion_array.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.imu_data = yaw

    # Callback function for odometry data
    def odometry_callback(self, msg):
        # Extract and update robot pose information from odometry message
        self.robot_pose[0] = msg.pose.pose.position.x
        self.robot_pose[1] = msg.pose.pose.position.y
        quaternion_array = msg.pose.pose.orientation
        orientation_list = [quaternion_array.x, quaternion_array.y, quaternion_array.z, quaternion_array.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.robot_pose[2] = yaw

    # Callback function for the left ultrasonic sensor
    def ultrasonic_rl_callback(self, msg):
        self.usrleft_value = msg.range

    # Callback function for the right ultrasonic sensor
    def ultrasonic_rr_callback(self, msg):
        self.usrright_value = msg.range

    # Utility function to normalize angles within the range of -π to π (OPTIONAL)
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
        #pass

    # Main control loop for managing docking behavior

    def undock_controller_loop(self):
        if self.is_docking:
            target_x = 0.5
            target_y = -2.455
            target_yaw = 3.14

            x_error = target_x - self.robot_pose[0]
            y_error = target_y - self.robot_pose[1]
            yaw_error = self.normalize_angle(target_yaw - self.robot_pose[2])

            angular_velocity = 0.5 * yaw_error

            linear_velocity = -0.2

            if x_error < 0.02 and y_error < 0.02 :
                linear_velocity = 0.0
                print("RUK GAYA")
                self.dock_aligned = True
                self.is_docking = False
            
            msg = Twist()
            msg.linear.x = linear_velocity
            msg.angular.z = angular_velocity
            self.velocity_publisher.publish(msg)


    def controller_loop(self):
        # The controller loop manages the robot's linear and angular motion 
        # control to achieve docking alignment and execution
        if self.is_docking:
            # ...
            # Implement control logic here for linear and angular motion
            # For example P-controller is enough, what is P-controller go check it out !
            target_yaw = 3.5  # Replace with the desired yaw angle for docking
            yaw_error = self.normalize_angle(target_yaw - self.imu_data)
            print(yaw_error)
            angular_velocity = 0.5 * yaw_error  # Adjust the gain as needed
            # angular_velocity = 0.5 * (target_yaw - self.robot_pose[2])

            # print("imu_data: ", self.imu_data)
            # print("yaw_error: ", yaw_error)

            # Linear velocity can be calculated based on sensor readings
            linear_velocity = -0.2  # Modify as needed
            
            # print("ursleft_value: ", self.usrleft_value)
            # Dynamic linear velocity adjustment based on ultrasonic sensor data

            if self.usrleft_value < 0.1 or self.usrright_value < 0.1:
                print("robot_pose: ", self.robot_pose[0], self.robot_pose[1], self.robot_pose[2])
                # If an obstacle is too close, reduce linear velocity
                linear_velocity = 0.0
                print("ruk gaya")

                self.dock_aligned = True
                self.is_docking = False

                if abs(0.5 - self.robot_pose[0]) < 0.3 or abs(-2.455 - self.robot_pose[1]) < 0.3:
                    print("hi")

                    yaw_error_2 = self.normalize_angle(3.14 - self.robot_pose[2])
                    print(yaw_error_2)

                    # if(yaw_error_2 < -0.1):
                    #     self.dock_aligned = True
                    #     self.is_docking = False

                    angular_velocity = 0.5 * yaw_error_2
                    self.dock_aligned = False
                    self.is_docking = True

                    if(yaw_error_2 > -0.02):
                        self.dock_aligned = True
                        self.is_docking = False

                
                elif self.dock_aligned == False and self.is_docking == True:
                    self.dock_aligned = False
                    self.is_docking = True
                else:
                    self.dock_aligned = True
                    self.is_docking = False

                print("robot_pose: ", self.robot_pose[0], self.robot_pose[1], self.robot_pose[2])

            # Publish velocity commands to the robot
            msg = Twist()
            msg.linear.x = linear_velocity
            msg.angular.z = angular_velocity
            self.velocity_publisher.publish(msg)
            # ...
            #pass

    # Callback function for the DockControl service
    def dock_control_callback(self, request, response):
        # Extract desired docking parameters from the service request
        linear_dock = request.linear_dock
        orientation_dock = request.orientation_dock
        distance = request.distance
        orientation = request.orientation
        rack_no = request.rack_no

        # Reset flags and start the docking process
        self.is_docking = True
        self.dock_aligned = False  # Initially, the robot is not aligned
        self.get_logger().info("Docking started!!!")

        self.controller_loop()

        # Log a message indicating that docking has started
        self.get_logger().info("Docking started!")

        # Create a rate object to control the loop frequency
        rate = self.create_rate(2, self.get_clock())

        # Wait until the robot is aligned for docking
        while not self.dock_aligned:
            self.get_logger().info("Waiting for alignment...")
            rate.sleep()

        # Set the service response indicating success
        response.success = True
        response.message = "Docking control initiated"
        return response
    

    # Un-dock control callback function
    def undock_control_callback(self, request, response):
        linear_dock = request.linear_dock
        orientation_dock = request.orientation_dock
        distance = request.distance
        orientation = request.orientation
        rack_no = request.rack_no

        self.is_docking = True
        self.dock_aligned = False
        self.get_logger().info("UNDOCKING STARTED!")

        self.undock_controller_loop()

        self.get_logger().info("UnDocking started!")

        rate = self.create_rate(2, self.get_clock())

        while not self.dock_aligned:
            self.get_logger().info("Waiting for alignment...")
            rate.sleep()

        response.success = True
        response.message = "UnDocking control initiated"
        return response


# Main function to initialize the ROS2 node and spin the executor
def main(args=None):
    rclpy.init(args=args)

    my_robot_docking_controller = MyRobotDockingController()

    executor = MultiThreadedExecutor()
    executor.add_node(my_robot_docking_controller)

    executor.spin()

    my_robot_docking_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()