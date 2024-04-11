#!/usr/bin/env python3


'''
*****************************************************************************************
*
*        		===============================================
*           		    Cosmo Logistic (CL) Theme (eYRC 2023-24)
*        		===============================================
*
*  This script should be used to implement Task 1A of Cosmo Logistic (CL) Theme (eYRC 2023-24).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:          eYRC#CL#2935
# Author List:		Akash Verma, Amit Kumar, Ashay Jadhav, Pushpak Kumar
# Filename:		    task1a.py
# Functions:        rotation_matrix_to_euler_angles, get_depth_from_depth_image, calculate_rectangle_area, detect_aruco
#			        [ Comma separated list of functions in this file ]
# Nodes:		    Add your publishing and subscribing node
#                   aruco_tf
#			        Publishing Topics  - [ /tf ]
#                   Subscribing Topics - [ /camera/aligned_depth_to_color/image_raw, /camera/color/image_raw ]


################### IMPORT MODULES #######################

from turtle import distance
import rclpy
import sys
import cv2
import math
import tf2_ros
import numpy as np
import rclpy.logging

import tf_transformations
from tf2_msgs.msg import TFMessage
import geometry_msgs.msg
from geometry_msgs.msg import TransformStamped

from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CompressedImage, Image


##################### FUNCTION DEFINITIONS #######################



def rotation_matrix_to_euler_angles(R):
    sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    if sy < 1e-6:
        # Singular case: cos(y) is close to 0
        # You can choose any convention for the remaining angles
        # In this example, we assume a ZYX rotation sequence
        rx = np.arctan2(R[1, 2], R[2, 2])
        ry = np.arctan2(-R[0, 2], sy)
        rz = 0.0
    else:
        rx = np.arctan2(R[2, 1], R[2, 2])
        ry = np.arctan2(-R[2, 0], sy)
        rz = np.arctan2(R[1, 0], R[0, 0])

    return rx, ry, rz


def get_depth_from_depth_image(depth_image, center_aruco_list):
    # Assuming depth_image is the depth image from the RealSense camera
    # center_aruco_list contains the pixel coordinates (x, y) of Aruco markers

    depth_values = []

    for center in center_aruco_list:
        # Extract the pixel coordinates
        x, y = center

        # Access the depth value at the corresponding pixel coordinates
        depth = depth_image[y, x]  # Note the order (y, x) because of OpenCV conventions

        # Convert depth value from millimeters to meters by dividing by 1000
        depth_meters = depth / 1000.0

        depth_values.append(depth_meters)

    return depth_values



def calculate_rectangle_area(coordinates):
    '''
    Description:    Function to calculate area or detected aruco

    Args:
        coordinates (list):     coordinates of detected aruco (4 set of (x,y) coordinates)

    Returns:
        area        (float):    area of detected aruco
        width       (float):    width of detected aruco
    '''

    ############ Function VARIABLES ############

    # You can remove these variables after reading the instructions. These are just for sample.

    area = None
    width = None

    ############ ADD YOUR CODE HERE ############

    area = cv2.contourArea(coordinates)
    
    x1 = coordinates[0][0]
    x2 = coordinates[1][0]
    y1 = coordinates[0][1]
    y2 = coordinates[1][1]

    # print(x1, x2, y1, y2)

    width = np.sqrt((x2-x1)**2 + (y2-y1)**2)

    # INSTRUCTIONS & HELP : 
    #	->  Recevice coordiantes from 'detectMarkers' using cv2.aruco library 
    #       and use these coordinates to calculate area and width of aruco detected.
    #	->  Extract values from input set of 4 (x,y) coordinates 
    #       and formulate width and height of aruco detected to return 'area' and 'width'.

    ############################################

    return area, width


def detect_aruco(image):
    '''
    Description:    Function to perform aruco detection and return each detail of aruco detected 
                    such as marker ID, distance, angle, width, center point location, etc.

    Args:
        image                   (Image):    Input image frame received from respective camera topic

    Returns:
        center_aruco_list       (list):     Center points of all aruco markers detected
        distance_from_rgb_list  (list):     Distance value of each aruco markers detected from RGB camera
        angle_aruco_list        (list):     Angle of all pose estimated for aruco marker
        width_aruco_list        (list):     Width of all detected aruco markers
        ids                     (list):     List of all aruco marker IDs detected in a single frame 
    '''

    ############ Function VARIABLES ############

    # ->  You can remove these variables if needed. These are just for suggestions to let you get started

    # Use this variable as a threshold value to detect aruco markers of certain size.
    # Ex: avoid markers/boxes placed far away from arm's reach position  
    aruco_area_threshold = 1500

    # The camera matrix is defined as per camera info loaded from the plugin used. 
    # You may get this from /camer_info topic when camera is spawned in gazebo.
    # Make sure you verify this matrix once if there are calibration issues.
    cam_mat = np.array([[931.1829833984375, 0.0, 640.0], [0.0, 931.1829833984375, 360.0], [0.0, 0.0, 1.0]])
    #cam_mat = np.array([[915.3003540039062, 0.0, 642.724365234375], [0.0, 914.0320434570312, 361.9780578613281], [0.0, 0.0, 1.0]])

    # The distortion matrix is currently set to 0. 
    # We will be using it during Stage 2 hardware as Intel Realsense Camera provides these camera info.
    dist_mat = np.array([0.0,0.0,0.0,0.0,0.0])

    # We are using 150x150 aruco marker size
    size_of_aruco_m = 0.15

    # You can remove these variables after reading the instructions. These are just for sample.
    center_aruco_list = []
    distance_from_rgb_list = []
    angle_aruco_list = []
    width_aruco_list = []
    ids = []
    image_with_markers=None
    # rvec=[]
    # tvec=[]
    rvec=None
    tvec=None
    flags=0
 
    ############ ADD YOUR CODE HERE ############
    #parameters = cv2.aruco.DetectorParameters_create()
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    #image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    #kernel_size = (5, 5)  # Adjust the kernel size as needed
    #blurred_image = cv2.GaussianBlur(gray_image, kernel_size, 0)
    #blurred_image=gray_image
    #blurred_image = cv2.blur(gray_image, kernel_size)
    #blurred_image = cv2.medianBlur(gray_image, 3)
    #cv2.imshow('blurimage', blurred_image)
    #cv2.waitKey(0)
 

    #gray_image = cv2.cvtColor(blurred_image, cv2.COLOR_BGR2GRAY)
    #gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    #ret,thresh1 = cv2.threshold(blurred_image,70,255,cv2.THRESH_BINARY)
    #thresh1 = cv2.adaptiveThreshold(blurred_image,255,cv2.ADAPTIVE_THRESH_MEAN_C,\
    #        cv2.THRESH_BINARY,72,30)
    #thresh1 = cv2.adaptiveThreshold(blurred_image,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
    #        cv2.THRESH_BINARY,41, -5)
    #ret2,thresh1 = cv2.threshold(blurred_image,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    #cv2.imshow('grayimage', thresh1)
    #cv2.waitKey(0)

    '''parameters = cv2.aruco.DetectorParameters_create()
    parameters.markerBorderBits = 1
    parameters.minMarkerPerimeterRate = 0.02
    parameters.maxMarkerPerimeterRate = 4.0
    parameters.polygonalApproxAccuracyRate = 0.05'''


    
    corners, marker_ids, rejected = cv2.aruco.detectMarkers(gray_image,dictionary,)
    #corners, marker_ids, rejected = cv2.aruco.detectMarkers(thresh1,dictionary,parameters=parameters)
    #print(rejected)
   
    areas_and_widths = []
    # corners_new = []

    if marker_ids is not None:
        image_with_markers = cv2.aruco.drawDetectedMarkers(image.copy(), corners, marker_ids)
        # image_with_markers = image.copy()
        valid_markers_corners = []
        
        ''' calculating area '''
        for i in range(len(corners)):
            for j in range(len(corners[i])):
                areas_and_widths.append(calculate_rectangle_area(corners[i][j]))
            width_aruco_list.append(areas_and_widths[i][1])
        
        # print(areas_and_widths)

        '''checking threshold'''
        for i in range(len(corners)):
            if areas_and_widths[i][0] > aruco_area_threshold:
                for j in range(len(corners[i])):
                    valid_markers_corners.append(corners[i][j])


                '''calculating centers for threshold satisfied markers'''
                for m in range(len(corners[i])):                  
                    cx = int(np.mean(corners[i][m][:,0]))
                    cy = int(np.mean(corners[i][m][:,1]))
                    center_aruco_list.append((cx, cy))

                '''making boundary for aruco, circles at center for valid markers'''
                for v in range(len(valid_markers_corners)):
                    
                    for centers in center_aruco_list:
                        radius = 2
                        color = (0,0,255)
                        thickness = 1
                        
                        cv2.circle(image_with_markers, centers, radius, color, thickness)         # circle on centre
                        
                        '''Valid Marker Text'''
                        tag_text = "Center"
                        tag_x = centers[0] - 10
                        tag_y = centers[1] - 10
                        font = cv2.FONT_HERSHEY_SIMPLEX
                        font_scale = 0.7
                        font_color = (255, 255, 255)  # White color
                        font_thickness = 2
                        cv2.putText(image_with_markers, tag_text, (tag_x, tag_y), font, font_scale, font_color, font_thickness)

                    '''pose estimation'''
                    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(valid_markers_corners, 10, cam_mat, dist_mat)
                    
                    cv2.drawFrameAxes(image_with_markers, cam_mat, dist_mat, rvec[v], tvec[v], 10)

                    # if rvec is not None and tvec is not None:
                    #     for v in range(len(valid_markers_corners)):
                    #         ids.append(marker_ids[v][0])

                    #     for p in range(len(rvec)):
                    #         angle_aruco_list.append(rvec[p][0][2])

                    #     distance_from_rgb_list = tvec

                    # else:
                    #     flags=1
                    #     return


                '''distance from rgb'''
                distance_from_rgb_list = tvec
                
        # print('Valid Marker Length: ', len(valid_markers))
        if rvec is not None and tvec is not None:
            # rvec=None
            for v in range(len(valid_markers_corners)):
                ids.append(marker_ids[v][0])
                # print(marker_ids[v][0])

            # rvec
            for p in range(len(rvec)):
                angle_aruco_list.append(rvec[p][0][2])
        else:
            flags=1
            return
        
        
        

    # print('all rvecs :', rvec)  #rvec print
    # print("all tvecs : ", tvec)  #tvec print
    # print("all aruco centres : ", center_aruco_list) #center list print

    #cv2.imshow('Markers', image_with_markers)
    #cv2.waitKey(0)
  

    # INSTRUCTIONS & HELP : 

    #	->  Convert input BGR image to GRAYSCALE for aruco detection

    #   ->  Use these aruco parameters-
    #       ->  Dictionary: 4x4_50 (4x4 only until 50 aruco IDs)

    #   ->  Detect aruco marker in the image and store 'corners' and 'ids'
    #       ->  HINT: Handle cases for empty markers detection. 

    #   ->  Draw detected marker on the image frame which will be shown later

    #   ->  Loop over each marker ID detected in frame and calculate area using function defined above (calculate_rectangle_area(coordinates))

    #   ->  Remove tags which are far away from arm's reach positon based on some threshold defined

    #   ->  Calculate center points aruco list using math and distance from RGB camera using pose estimation of aruco marker
    #       ->  HINT: You may use numpy for center points and 'estimatePoseSingleMarkers' from cv2 aruco library for pose estimation

    #   ->  Draw frame axes from coordinates received using pose estimation
    #       ->  HINT: You may use 'cv2.drawFrameAxes'

    ############################################

    return center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids, image_with_markers,flags


##################### CLASS DEFINITION #######################

class aruco_tf(Node):
    '''
    ___CLASS___

    Description:    Class which servers purpose to define process for detecting aruco marker and publishing tf on pose estimated.
    '''

    def __init__(self):
        '''
        Description:    Initialization of class aruco_tf
                        All classes have a function called __init__(), which is always executed when the class is being initiated.
                        The __init__() function is called automatically every time the class is being used to create a new object.
                        You can find more on this topic here -> https://www.w3schools.com/python/python_classes.asp
        '''

        super().__init__('aruco_tf_publisher')                                          # registering node

        ############ Topic SUBSCRIPTIONS ############
        self.images_received = False
        self.Images_received = False
      
        self.color_cam_sub = self.create_subscription(Image, '/camera/color/image_raw', self.colorimagecb, 10)
        self.depth_cam_sub = self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.depthimagecb, 10)

        ############ Constructor VARIABLES/OBJECTS ############

        image_processing_rate = 0.1                                                     # rate of time to process image (seconds)
        self.bridge = CvBridge()                                                        # initialise CvBridge object for image conversion
        self.tf_buffer = tf2_ros.buffer.Buffer()                                        # buffer time used for listening transforms
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)                                    # object as transform broadcaster to send transform wrt some frame_id
       
        
        self.timer = self.create_timer(image_processing_rate, self.process_image)       # creating a timer based function which gets called on every 0.2 seconds (as defined by 'image_processing_rate' variable)
            
        self.cv_image = None                                                            # colour raw image variable (from colorimagecb())
        self.depth_image = None                                                         # depth image variable (from depthimagecb())


    def depthimagecb(self, data):
        '''
        Description:    Callback function for aligned depth camera topic. 
                        Use this function to receive image depth data and convert to CV2 image

        Args:
            data (Image):    Input depth image frame received from aligned depth camera topic

        Returns:
        '''

        try:
            
            self.depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
            self.images_received = True
            # cv2.imshow('Depth Image', self.depth_image)
            # cv2.waitKey(1)

        except CvBridgeError as e:
            self.get_logger().error(f'Error: {e}')

        
        

        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP : 

        #	->  Use data variable to convert ROS Image message to CV2 Image type

        #   ->  HINT: You may use CvBridge to do the same

        ############################################


    def colorimagecb(self, data):
        '''
        Description:    Callback function for colour camera raw topic.
                        Use this function to receive raw image data and convert to CV2 image

        Args:
            data (Image):    Input coloured raw image frame received from image_raw camera topic

        Returns:
        '''

        try:
            
            self.cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
            # cv2.imshow('Color Image', self.cv_image)
            self.Images_received = True
            # cv2.waitKey(1)


        except CvBridgeError as er:
            self.get_logger().error(f'Error: {er}')

        # detect_aruco(self.cv_image)

        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP : 

        #	->  Use data variable to convert ROS Image message to CV2 Image type

        #   ->  HINT:   You may use CvBridge to do the same
        #               Check if you need any rotation or flipping image as input data maybe different than what you expect to be.
        #               You may use cv2 functions such as 'flip' and 'rotate' to do the same

        ############################################

        
   

    def process_image(self):
        '''
        Description:    Timer function used to detect aruco markers and publish tf on estimated poses.

        Args:
        Returns:
        '''

        ############ Function VARIABLES ############

        # These are the variables defined from camera info topic such as image pixel size, focalX, focalY, etc.
        # Make sure you verify these variable values once. As it may affect your result.
        # You can find more on these variables here -> http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html
        

        #if not self.images_received and not self.Images_received:
        #    self.get_logger().info('Waiting for images...')
        #    return
        if self.depth_image is None or self.cv_image is None:
            print('waiting')
            return
        
        sizeCamX = 1280
        sizeCamY = 720
        centerCamX = 640 
        centerCamY = 360
        focalX = 931.1829833984375
        focalY = 931.1829833984375
            

        ############ ADD YOUR CODE HERE ############


        result = detect_aruco(self.cv_image)
        if result is not None:
            center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids, img, flags = result
        else:
            # Handle the case where Aruco markers are not detected
            # For example, you can print a message and return from the function.
            print("Aruco markers not detected in the image.")
            return

        # center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids, img,flags = detect_aruco(self.cv_image)
        if center_aruco_list == [] or distance_from_rgb_list == [] or angle_aruco_list == [] or width_aruco_list == [] or ids == [] or img is None or flags==1:
            return
    

        
         
        #center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids, img = detect_aruco(image)


        # print(f'\nHELLLO WORLDLDDLDLD!!!!!!!')
        # print(f'\nCenter Aruco List: {center_aruco_list}')
        # print(f'\nDistance From RGB List: {distance_from_rgb_list}')
        # print(f'\nWidth Aruco List: {width_aruco_list}')
        # self.get_logger().info(f'\nAngle Aruco List: {angle_aruco_list}')
        # self.get_logger().info(f'\nIDs: {ids}')

        quaternion = []

        realsense_depth = get_depth_from_depth_image(self.depth_image, center_aruco_list)
        # print('realsesne depth : ', realsense_depth)

        for id in range(len(ids)):
            
            marker_id = ids[id]
            print('###############MARKER : ', marker_id, '######################')
            center_x, center_y = center_aruco_list[id]
            distance_from_rgb = realsense_depth[id]
            
            corrected_yaw = (0.788 * angle_aruco_list[id]) - ((angle_aruco_list[id] ** 2) / 3160)

            print("initial yaw : ", angle_aruco_list[id], "corrected yaw : ", corrected_yaw)

            rot_mat = np.array([[0, -0.2029, 0.9781], [1, 0, 0], [0, 0.9781, 0.2029]])

            # yaw_mat = np.array([[np.cos(-np.pi/2), 0, np.sin(-np.pi/2)], [0, 1, 0], [-np.sin(-np.pi/2), 0, np.cos(-np.pi/2)]])
            yaw_mat = np.array([[np.cos(-corrected_yaw), 0, np.sin(-corrected_yaw)], [0, 1, 0], [-np.sin(-corrected_yaw), 0, np.cos(-corrected_yaw)]])

            fin_mat = np.dot(rot_mat, yaw_mat)

            roll, pitch, yaw = rotation_matrix_to_euler_angles(fin_mat)

            quaternion = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
            print(quaternion)
                
            x = distance_from_rgb
            y = (distance_from_rgb * (sizeCamX - center_x - centerCamX) / focalX)
            z = (distance_from_rgb * (sizeCamY - center_y - centerCamY) / focalY)

            # self.get_logger().info(f'x : {x}')
            # self.get_logger().info(f'y : {y}')
            # self.get_logger().info(f'z : {z}')

            transform = geometry_msgs.msg.TransformStamped()

            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = 'camera_link'
            transform.child_frame_id = '2935_cam_' + str(marker_id)

            transform.transform.translation.x = x
            transform.transform.translation.y = y
            transform.transform.translation.z = z

            transform.transform.rotation.x = quaternion[0]
            transform.transform.rotation.y = quaternion[1]
            transform.transform.rotation.z = quaternion[2]
            transform.transform.rotation.w = quaternion[3]

            self.br.sendTransform(transform)

        for i in range(len(ids)):
            marker_id = ids[i]
            try:
                # Lookup the transform from base_link to obj_frame
                target_base = 'base_link'
                source_base = '2935_cam_' + str(marker_id)
                t2 = self.tf_buffer.lookup_transform(target_base, source_base, rclpy.time.Time())

                tf_msg = geometry_msgs.msg.TransformStamped()

                tf_msg.header.stamp = self.get_clock().now().to_msg()
                tf_msg.header.frame_id = 'base_link'
                tf_msg.child_frame_id = '2935_base_' + str(marker_id)
                
                tf_msg.transform.translation.x = t2.transform.translation.x
                tf_msg.transform.translation.y = t2.transform.translation.y
                tf_msg.transform.translation.z = t2.transform.translation.z

                tf_msg.transform.rotation.x = t2.transform.rotation.x
                tf_msg.transform.rotation.y = t2.transform.rotation.y
                tf_msg.transform.rotation.z = t2.transform.rotation.z
                tf_msg.transform.rotation.w = t2.transform.rotation.w

                self.br.sendTransform(tf_msg)

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                self.get_logger().info(f"Transform lookup failed: {e}")


        ############TASK 2############## END EFFECTOR #################3
        # try:
        #     t3 = self.tf_buffer.lookup_transform('base_link', 'tool0', rclpy.time.Time())
        #     end_tf = geometry_msgs.msg.TransformStamped()

        #     end_tf.header.stamp = self.get_clock().now().to_msg()
        #     end_tf.header.frame_id = 'base_link'
        #     end_tf.child_frame_id = 'tool0'

        #     end_tf.transform.translation.x = t3.transform.translation.x
        #     end_tf.transform.translation.y = t3.transform.translation.y
        #     end_tf.transform.translation.z = t3.transform.translation.z

        #     end_tf.transform.rotation.x = t3.transform.rotation.x
        #     end_tf.transform.rotation.y = t3.transform.rotation.y
        #     end_tf.transform.rotation.z = t3.transform.rotation.z
        #     end_tf.transform.rotation.w = t3.transform.rotation.w

        #     self.br.sendTransform(end_tf)

        # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        #     self.get_logger().info(f"Transform lookup failed: {e}")
        

        # INSTRUCTIONS & HELP : 

        #	->  Get aruco center, distance from rgb, angle, width and ids list from 'detect_aruco_center' defined above

        #   ->  Loop over detected box ids received to calculate position and orientation transform to publish TF 

        #   ->  Use this equation to correct the input aruco angle received from cv2 aruco function 'estimatePoseSingleMarkers' here
        #       It's a correction formula- 
        #       angle_aruco = (0.788*angle_aruco) - ((angle_aruco**2)/3160)

        #   ->  Then calculate quaternions from roll pitch yaw (where, roll and pitch are 0 while yaw is corrected aruco_angle)

        #   ->  Use center_aruco_list to get realsense depth and log them down. (divide by 1000 to convert mm to m)

        #   ->  Use this formula to rectify x, y, z based on focal length, center value and size of image
        #       x = distance_from_rgb * (sizeCamX - cX - centerCamX) / focalX
        #       y = distance_from_rgb * (sizeCamY - cY - centerCamY) / focalY
        #       z = distance_from_rgb
        #       where, 
        #               cX, and cY from 'center_aruco_list'
        #               distance_from_rgb is depth of object calculated in previous step
        #               sizeCamX, sizeCamY, centerCamX, centerCamY, focalX and focalY are defined above

        #   ->  Now, mark the center points on image frame using cX and cY variables with help of 'cv2.cirle' function 

        #   ->  Here, till now you receive coordinates from camera_link to aruco marker center position. 
        #       So, publish this transform w.r.t. camera_link using Geometry Message - TransformStamped 
        #       so that we will collect it's position w.r.t base_link in next step.
        #       Use the following frame_id-
        #           frame_id = 'camera_link'
        #           child_frame_id = 'cam_<marker_id>'          Ex: cam_20, where 20 is aruco marker ID

        #   ->  Then finally lookup transform between base_link and obj frame to publish the TF
        #       You may use 'lookup_transform' function to pose of obj frame w.r.t base_link 

        #   ->  And now publish TF between object frame and base_link
        #       Use the following frame_id-
        #           frame_id = 'base_link'
        #           child_frame_id = 'obj_<marker_id>'          Ex: obj_20, where 20 is aruco marker ID

        #   ->  At last show cv2 image window having detected markers drawn and center points located using 'cv2.imshow' function.
        #       Refer MD book on portal for sample image -> https://portal.e-yantra.org/

        #   ->  NOTE:   The Z axis of TF should be pointing inside the box (Purpose of this will be known in task 1B)
        #               Also, auto eval script will be judging angular difference aswell. So, make sure that Z axis is inside the box (Refer sample images on Portal - MD book)

        ############################################


##################### FUNCTION DEFINITION #######################

def main():
    '''
    Description:    Main function which creates a ROS node and spin around for the aruco_tf class to perform it's task
    '''

    rclpy.init(args=sys.argv)                                       # initialisation

    node = rclpy.create_node('aruco_tf_process')                    # creating ROS node

    node.get_logger().info('Node created: Aruco tf process')        # logging information

    aruco_tf_class = aruco_tf()                                     # creating a new object for class 'aruco_tf'

    rclpy.spin(aruco_tf_class)                                      # spining on the object to make it alive in ROS 2 DDS

    aruco_tf_class.destroy_node()                                   # destroy node after spin ends

    rclpy.shutdown()                                                # shutdown process


if __name__ == '__main__':
    '''
    Description:    If the python interpreter is running that module (the source file) as the main program, 
                    it sets the special __name__ variable to have a value “__main__”. 
                    If this file is being imported from another module, __name__ will be set to the module’s name.
                    You can find more on this here -> https://www.geeksforgeeks.org/what-does-the-if-__name__-__main__-do/
    '''

    main()