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
from ebot_docking.srv import DockSw, UnDockSw # Import custom service message
import math, statistics
import yaml

# asj='k'
# rack_No = 'rack1000'

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

        self.undock_control_srv = self.create_service(UnDockSw, 'undock_control', self.undock_control_callback, callback_group=self.callback_group)

        # Creating a ROS2 service for controlling un-docking behaviour
        #self.undock_control_srv = self.create_service(DockSw, 'undock_control', self.undock_control_callback, callback_group=self.callback_group)

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
        # self.controller_timer = self.create_timer(0.1, self.controller_loop)

        with open("/home/akash/colcon_ws/src/ebot_nav2/scripts/config.yaml", "r") as f:
            self.pose_data = yaml.safe_load(f)

        self.rack1_yaw = self.pose_data['position'][0]['rack1'][2]
        self.rack2_yaw = self.pose_data['position'][1]['rack2'][2]
        self.rack3_yaw = self.pose_data['position'][2]['rack3'][2]
        # print(rack_coordinates)

    #Callback fro IMU
    def imu_callback(self, msg):
        # Extract and process IMU data (orientation)
        quaternion_array = msg.orientation
        orientation_list = [quaternion_array.x, quaternion_array.y, quaternion_array.z, quaternion_array.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        # if yaw<0.0:
        #     yaw=3.14-yaw
        # elif yaw>0.0:
        #     yaw=yaw
        
        # yaw1=6.28- yaw
        # self.imu_data = yaw1
        if yaw<0.0:    
            yaw1=-yaw
        else:
            yaw1=6.28- yaw
    
        self.imu_data = yaw1
        # self.imu_data = yaw

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


    def controller_loop(self, rack_No):
        print(rack_No)
        
        if rack_No =='rack1':
            target_yaw = self.rack1_yaw
            print(self.rack1_yaw)

        elif rack_No == 'rack2':
            target_yaw = self.rack2_yaw
            print(self.rack2_yaw)

        elif rack_No == 'rack3':
            target_yaw = self.rack3_yaw
            print(self.rack3_yaw)

        if target_yaw<0.0:    
            target_yaw=-target_yaw
        else:
            target_yaw=6.28-target_yaw

        # if target_yaw < 0.0:    
        #     target_yaw = 3.14-target_yaw
        # else:
        #     target_yaw = target_yaw

        # target_yaw = 6.28 - target_yaw
        
        # print(asj)
        # if rack_No == 'rack1000':
        #     return
        # The controller loop manages the robot's linear and angular motion 
        # control to achieve docking alignment and execution
        print(rack_No)
        msg = Twist()
        # The controller loop manages the robot's linear and angular motion 
        # control to achieve docking alignment and execution
        if self.is_docking:
            print('Docking Started')
            # ...
            # Implement control logic here for linear and angular motion
            # For example P-controller is enough, what is P-controller go check it out !
            # target_yaw = 3.14 # Replace with the desired yaw angle for docking
            normalized_yaw= self.normalize_angle( self.imu_data)
            print('imu_data=  ',self.imu_data)
            #print('normalised_yaw=  ',normalized_yaw)

            yaw_error1 = target_yaw - normalized_yaw

            #print('yaw_error my method = ', yaw_error1)
            yaw_error= self.normalize_angle(target_yaw-self.imu_data)
            
            
            print('yaw error=  ',yaw_error)
            angular_velocity = 2 * yaw_error  # Adjust the gain as needed
            # angular_velocity = 0.5 * (target_yaw - self.robot_pose[2])

            if abs(yaw_error)>=0.025:
                print('angular docking')
                msg.linear.x = 0.0
                msg.angular.z = -angular_velocity
                # msg.angular.z = 0.5
                self.velocity_publisher.publish(msg)
                return
            
            elif abs(yaw_error)<0.025 :
                if self.usrleft_value > 0.15 or self.usrright_value > 0.15:
                    print('linear docking')
                    print('usr_left = ',self.usrleft_value,'usr_right = ', self.usrright_value)
                    msg.linear.x = -0.2
                    msg.angular.z = 0.0
                
                
                if self.usrleft_value < 0.15 or self.usrright_value < 0.15:
                    msg.linear.x = 0.0 
                    msg.angular.z = 0.0
                    self.velocity_publisher.publish(msg)
                    print('Docking completed')    
                    self.dock_aligned = True
                    self.is_docking = False

                self.velocity_publisher.publish(msg)
                return


    def undock_controller_loop(self, rack_No):
        print(rack_No)
        
        if rack_No =='rack1':
            target_yaw = -1.57
            print(self.rack1_yaw)

        elif rack_No == 'rack2':
            target_yaw = 3.14
            print(self.rack2_yaw)

        elif rack_No == 'rack3':
            target_yaw = self.rack3_yaw
            print(self.rack3_yaw)

        if target_yaw<0.0:    
            target_yaw=-target_yaw
        else:
            target_yaw=6.28-target_yaw

        # if target_yaw < 0.0:    
        #     target_yaw = 3.14-target_yaw
        # else:
        #     target_yaw = target_yaw

        # target_yaw = 6.28 - target_yaw
        
        # print(asj)
        # if rack_No == 'rack1000':
        #     return
        # The controller loop manages the robot's linear and angular motion 
        # control to achieve docking alignment and execution
        print(rack_No)
        msg = Twist()
        # The controller loop manages the robot's linear and angular motion 
        # control to achieve docking alignment and execution
        if self.is_docking:
            print('Undocking Started')
            # ...
            # Implement control logic here for linear and angular motion
            # For example P-controller is enough, what is P-controller go check it out !
            # target_yaw = 3.14 # Replace with the desired yaw angle for docking
            normalized_yaw= self.normalize_angle(self.imu_data)
            print('imu_data=  ',self.imu_data)
            #print('normalised_yaw=  ',normalized_yaw)

            yaw_error1 = target_yaw - normalized_yaw

            #print('yaw_error my method = ', yaw_error1)
            yaw_error= self.normalize_angle(target_yaw-self.imu_data)
            
            
            print('yaw error=  ',yaw_error)
            angular_velocity = 2 * yaw_error  # Adjust the gain as needed
            # angular_velocity = 0.5 * (target_yaw - self.robot_pose[2])

            if abs(yaw_error)>=0.025:
                print('angular undocking')
                msg.linear.x = 0.0
                msg.angular.z = -angular_velocity
                # msg.angular.z = 0.5
                self.velocity_publisher.publish(msg)
                return
            
            elif abs(yaw_error)<0.025 :
                # if self.usrleft_value > 0.1 or self.usrright_value > 0.1:
                #     print('linear docking')
                #     print('usr_left = ',self.usrleft_value,'usr_right = ', self.usrright_value)
                #     msg.linear.x = -0.2
                #     msg.angular.z = 0.0
                
                if self.usrleft_value < 0.15 or self.usrright_value < 0.15:
                    print(self.robot_pose[0], self.robot_pose[1])

                    if rack_No == 'rack1':
                        print(abs(-3.30 - self.robot_pose[1]))
                        if abs(-3.30 - self.robot_pose[1]) > 0.05:
                            msg.linear.x = -0.2 
                            msg.angular.z = 0.0
                            self.velocity_publisher.publish(msg)

                        elif abs(-3.30 - self.robot_pose[1]) < 0.05:
                            msg.linear.x = 0.0 
                            msg.angular.z = 0.0
                            self.velocity_publisher.publish(msg)
                            print('Undocking completed')    
                            self.dock_aligned = True
                            self.is_docking = False


                    elif rack_No == 'rack2':
                        print(abs(0.7 - self.robot_pose[0]))
                        if abs(0.7 - self.robot_pose[0]) > 0.05:
                            msg.linear.x = -0.2
                            msg.angular.z = 0.0
                            self.velocity_publisher.publish(msg)
                        elif abs(0.7 - self.robot_pose[0]) < 0.05:
                            msg.linear.x = 0.0 
                            msg.angular.z = 0.0
                            self.velocity_publisher.publish(msg)
                            print('Undocking completed')    
                            self.dock_aligned = True
                            self.is_docking = False

                # self.velocity_publisher.publish(msg)
                return
        

    # Callback function for the DockControl service
    def dock_control_callback(self, request, response):
        # global rack_No
        # Extract desired docking parameters from the service request
        linear_dock = request.linear_dock
        orientation_dock = request.orientation_dock
        distance = request.distance
        orientation = request.orientation
        rack_No = request.rack_no
        # asj = 'DOCK CALLBACK'
        # if rack_No == 'rack1000':
        #     return

        # Reset flags and start the docking process
        self.is_docking = True
        self.dock_aligned = False  # Initially, the robot is not aligned
        self.get_logger().info("Docking started!!!")

        # # Log a message indicating that docking has started
        # self.get_logger().info("Docking started!")

        # Create a rate object to control the loop frequency
        rate = self.create_rate(2, self.get_clock())

        # Wait until the robot is aligned for docking
        while self.is_docking == True:
            self.controller_loop(rack_No)
            rate.sleep()

        # elif self.dock_aligned:
        response.success = True
        response.message = "Docking control completed"
        return response

        # Set the service response indicating success
        
    
    # Un-dock control callback function
    def undock_control_callback(self, request, response):
        # global rack_No
        # Extract desired docking parameters from the service request
        linear_dock = request.linear_dock
        orientation_dock = request.orientation_dock
        distance = request.distance
        orientation = request.orientation
        rack_No = request.rack_no
        # asj = 'DOCK CALLBACK'
        # if rack_No == 'rack1000':
        #     return

        # Reset flags and start the docking process
        self.is_docking = True
        self.dock_aligned = False  # Initially, the robot is not aligned
        self.get_logger().info("Undocking started!!!")

        # # Log a message indicating that docking has started
        # self.get_logger().info("Docking started!")

        # Create a rate object to control the loop frequency
        rate = self.create_rate(2, self.get_clock())

        # Wait until the robot is aligned for docking
        while self.is_docking == True:
            self.undock_controller_loop(rack_No)
            rate.sleep()

        # elif self.dock_aligned:
        response.success = True
        response.message = "Unocking control completed"
        return response

        # Set the service response indicating success

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