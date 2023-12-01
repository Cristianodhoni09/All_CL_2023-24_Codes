#!/usr/bin/env python3
import tf2_ros
import geometry_msgs
import tf2_msgs
from tf_transformations import euler_from_quaternion
import time

from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5
from link_attach import AttachLinkNode
from link_detach import DetachLinkNode

from math import cos, sin
import math
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

import pymoveit2.moveit2_servo
from pymoveit2.moveit2_servo import MoveIt2Servo

# obj1_trans = []
# obj1_rot = []
# obj3_trans = []
# obj3_rot = []
# obj49_trans = []
# obj49_rot = []
# tool0_trans = []
# tool0_rot = []
i1=0
j1=1

i3=0
j3=1

i49=0
j49=1

m=1
k=[0, 0, 0]

box1_complete = 0
box3_complete = 0
box49_complete = 0

#obj1_trans_list = [[0.222,-0.455,0.656]]
#obj1_rot_list = [[0.705,-0.0073,-0.047,0.707]]
obj1_trans_list=[]
obj1_rot_list =[]
obj1_trans = None
obj1_rot = None

obj3_trans_list=[]
obj3_rot_list =[]
obj3_trans = None
obj3_rot = None

obj49_trans_list=[]
obj49_rot_list =[]
obj49_trans = None
obj49_rot = None

box1_align = 0
box3_align = 0
box49_align = 0

box1_afterpick_aligned = 0
box3_afterpick_aligned = 0
box49_afterpick_aligned = 0

box1_aligned = 0
box3_aligned = 0
box49_aligned = 0
duration=2.0


class TFSubscriber(Node):
    def __init__(self):
        super().__init__('tf_subscriber')

        self.tf_buffer = tf2_ros.buffer.Buffer()
        self.twist_pub = self.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
        self.br = tf2_ros.TransformBroadcaster(self)
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.subscription = self.create_subscription(TFMessage, '/tf', self.tf_callback, 10)
        self.subscription  # prevent unused variable warning

        self.link_attach_client = AttachLinkNode()
        self.link_detach_client = DetachLinkNode()

        callback_group = ReentrantCallbackGroup()
        self.moveit2 = MoveIt2(node=self, joint_names=ur5.joint_names(), base_link_name=ur5.base_link_name(), end_effector_name=ur5.end_effector_name(), group_name=ur5.MOVE_GROUP_ARM, callback_group=callback_group)

        # self.timer = self.create_timer(0.2, self.servo_motion)

    '''def movetoconfig(self):
        transit_pose = [0.0, -2.07694181, 2.670353756, -3.735004599, -1.5882496, 3.141592654]
        self.moveit2.move_to_configuration(transit_pose)'''

    def servo_motion(self, obj_trans_rel_x, obj_trans_rel_y, obj_trans_rel_z, roll, pitch, yaw):
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()

        twist_msg.twist.linear.x = obj_trans_rel_x
        twist_msg.twist.linear.y = obj_trans_rel_y
        twist_msg.twist.linear.z = obj_trans_rel_z

        twist_msg.twist.angular.x = roll
        twist_msg.twist.angular.y = pitch
        twist_msg.twist.angular.z = yaw

        self.twist_pub.publish(twist_msg)
              
        # lin = (obj_trans_rel_x, obj_trans_rel_y, obj_trans_rel_z)
        # ang = (roll, pitch, yaw)

        # MoveIt2Servo.servo(self, lin, ang)

    def tf_callback(self, msg):
        global i1
        global j1
        global i3
        global j3
        global i49
        global j49
        global m
        global k

        global box1_complete
        global box3_complete
        global box49_complete

        global obj1_trans
        global obj1_rot

        global obj3_trans
        global obj3_rot

        global obj49_trans
        global obj49_rot

        global box1_align
        global box3_align
        global box49_align

        global box1_aligned
        global box3_aligned
        global box49_aligned

        global box1_afterpick_aligned 
        global box3_afterpick_aligned 
        global box49_afterpick_aligned
        
        global duration

        #if len(obj1_trans_list) == 0 and len(obj1_rot_list)==0:
        if m==1 :
        #if not obj1_trans and not obj1_rot:
            for transform in msg.transforms:
                if transform.header.frame_id == "base_link" and transform.child_frame_id == "obj_1":
                    obj1_trans = transform.transform.translation
                    obj1_rot = transform.transform.rotation
                    
                    if obj1_trans is not None  :
                        obj1_trans_list.append(obj1_trans)

                    if obj1_rot is not None:
                        obj1_rot_list.append(obj1_rot)

                    box1_roll = math.atan2(2.0 * (obj1_rot_list[0].z * obj1_rot_list[0].y + obj1_rot_list[0].w * obj1_rot_list[0].x) , 1.0 - 2.0 * (obj1_rot_list[0].x * obj1_rot_list[0].x + obj1_rot_list[0].y * obj1_rot_list[0].y))
                    box1_pitch = math.asin(2.0 * (obj1_rot_list[0].y * obj1_rot_list[0].w - obj1_rot_list[0].z * obj1_rot_list[0].x))
                    box1_yaw = math.atan2(2.0 * (obj1_rot_list[0].z * obj1_rot_list[0].w + obj1_rot_list[0].x * obj1_rot_list[0].y) , - 1.0 + 2.0 * (obj1_rot_list[0].w * obj1_rot_list[0].w + obj1_rot_list[0].x * obj1_rot_list[0].x))
                    
                    if abs(box1_yaw) <= 3.14 and abs(box1_yaw) >= 3.00:
                        box1_align = -1 # in left rack

                    elif abs(box1_yaw) <= 1.50 and abs(box1_yaw) >= 1.62:
                        box1_align = 0  # in centre
                    
                    elif abs(box1_yaw) <= 0.10 and abs(box1_yaw) >= 0.00:
                        box1_align = 1 # in right rack

                    print("obj1_trans_list[0]", obj1_trans_list[0])
                    print("obj1_rot_list[0]", obj1_rot_list[0])
                    print("box1 roll, pitch, yaw: ", box1_roll, box1_pitch, box1_yaw)
                    print("box1_align: ", box1_align)


                if transform.header.frame_id == "base_link" and transform.child_frame_id == "obj_3":
                    obj3_trans = transform.transform.translation
                    obj3_rot = transform.transform.rotation

                    if obj3_trans is not None  :
                        obj3_trans_list.append(obj3_trans)
                        
                    if obj3_rot is not None:
                        obj3_rot_list.append(obj3_rot)

                    box3_roll = math.atan2(2.0 * (obj3_rot_list[0].z * obj3_rot_list[0].y + obj3_rot_list[0].w * obj3_rot_list[0].x) , 1.0 - 2.0 * (obj3_rot_list[0].x * obj3_rot_list[0].x + obj3_rot_list[0].y * obj3_rot_list[0].y))
                    box3_pitch = math.asin(2.0 * (obj3_rot_list[0].y * obj3_rot_list[0].w - obj3_rot_list[0].z * obj3_rot_list[0].x))
                    box3_yaw = math.atan2(2.0 * (obj3_rot_list[0].z * obj3_rot_list[0].w + obj3_rot_list[0].x * obj3_rot_list[0].y) , - 1.0 + 2.0 * (obj3_rot_list[0].w * obj3_rot_list[0].w + obj3_rot_list[0].x * obj3_rot_list[0].x))

                    if abs(box3_yaw) <= 3.14 and abs(box3_yaw) >= 3.00:
                        box3_align = -1 # in left rack

                    elif abs(box3_yaw) <= 1.50 and abs(box3_yaw) >= 1.62:
                        box3_align = 0  # in centre
                    
                    elif abs(box3_yaw) <= 0.10 and abs(box3_yaw) >= 0.00:
                        box3_align = 1 # in right rack

                    print("obj3_trans_list[0]", obj3_trans_list[0])
                    print("obj3_rot_list[0]", obj3_rot_list[0])
                    print("box3 roll, pitch, yaw: ", box3_roll, box3_pitch, box3_yaw)
                    print("box3_align: ", box3_align)
                    

                if transform.header.frame_id == "base_link" and transform.child_frame_id == "obj_49":
                    obj49_trans = transform.transform.translation
                    obj49_rot = transform.transform.rotation

                    if obj49_trans is not None  :
                        obj49_trans_list.append(obj49_trans)
                        
                    if obj49_rot is not None:
                        obj49_rot_list.append(obj49_rot)
                    
                    box49_roll = math.atan2(2.0 * (obj49_rot_list[0].z * obj49_rot_list[0].y + obj49_rot_list[0].w * obj49_rot_list[0].x) , 1.0 - 2.0 * (obj49_rot_list[0].x * obj49_rot_list[0].x + obj49_rot_list[0].y * obj49_rot_list[0].y))
                    box49_pitch = math.asin(2.0 * (obj49_rot_list[0].y * obj49_rot_list[0].w - obj49_rot_list[0].z * obj49_rot_list[0].x))
                    box49_yaw = math.atan2(2.0 * (obj49_rot_list[0].z * obj49_rot_list[0].w + obj49_rot_list[0].x * obj49_rot_list[0].y) , - 1.0 + 2.0 * (obj49_rot_list[0].w * obj49_rot_list[0].w + obj49_rot_list[0].x * obj49_rot_list[0].x))
                    
                    if abs(box49_yaw) <= 3.14 and abs(box49_yaw) >= 3.00:
                        box49_align = -1 # in left rack

                    elif abs(box49_yaw) <= 1.50 and abs(box49_yaw) >= 1.62:
                        box49_align = 0  # in centre
                    
                    elif abs(box49_yaw) <= 0.10 and abs(box49_yaw) >= 0.00:
                        box49_align = 1 # in right rack

                    print("obj49_trans_list[0]", obj49_trans_list[0])
                    print("obj49_rot_list[0]", obj49_rot_list[0])
                    print("box49 roll, pitch, yaw: ", box49_roll, box49_pitch, box49_yaw)
                    print("box49_align: ", box49_align)

            # elif transform.header.frame_id == "wrist_3_link" and transform.child_frame_id == "tool0":
            #     tool0_trans = transform.transform.translation
            #     tool0_rot = transform.transform.rotation

        # END EFFECTOR LOOKUP
        
        end_tf = TransformStamped()

        ############## OBJ 1 ############
        if obj1_rot is not None and obj1_trans is not None and box1_complete == 0 and box49_complete==2 and box3_complete==2:
            try:
                if box1_align == -1 and box1_aligned == 0:  # left rack
                    self.moveit2.move_to_configuration([0.0 + 1.57, -2.07694181, 2.670353756, -3.735004599, -1.5882496, 3.141592654])
                    time.sleep(5)
                    box1_aligned = 1

                elif box1_align == 1 and box1_aligned == 0: # right rack
                    self.moveit2.move_to_configuration([0.0 - 1.57, -2.07694181, 2.670353756, -3.735004599, -1.5882496, 3.141592654])
                    time.sleep(5)
                    box1_aligned = 1

                t3 = self.tf_buffer.lookup_transform('base_link', 'tool0', rclpy.time.Time())

                end_tf.header.stamp = self.get_clock().now().to_msg()
                end_tf.header.frame_id = 'base_link'
                end_tf.child_frame_id = 'tool0'

                end_tf.transform.translation.x = t3.transform.translation.x
                end_tf.transform.translation.y = t3.transform.translation.y
                end_tf.transform.translation.z = t3.transform.translation.z

                end_tf.transform.rotation.x = t3.transform.rotation.x
                end_tf.transform.rotation.y = t3.transform.rotation.y
                end_tf.transform.rotation.z = t3.transform.rotation.z
                end_tf.transform.rotation.w = t3.transform.rotation.w

                #print("end_tf", end_tf.transform.rotation.x, end_tf.transform.rotation.y, end_tf.transform.rotation.z, end_tf.transform.rotation.w)
            
                roll_end_tf = math.atan2(2.0 * (end_tf.transform.rotation.z * end_tf.transform.rotation.y + end_tf.transform.rotation.w * end_tf.transform.rotation.x) , 1.0 - 2.0 * (end_tf.transform.rotation.x * end_tf.transform.rotation.x + end_tf.transform.rotation.y * end_tf.transform.rotation.y))
                pitch_end_tf = math.asin(2.0 * (end_tf.transform.rotation.y * end_tf.transform.rotation.w - end_tf.transform.rotation.z * end_tf.transform.rotation.x))
                yaw_end_tf = math.atan2(2.0 * (end_tf.transform.rotation.z * end_tf.transform.rotation.w + end_tf.transform.rotation.x * end_tf.transform.rotation.y) , - 1.0 + 2.0 * (end_tf.transform.rotation.w * end_tf.transform.rotation.w + end_tf.transform.rotation.x * end_tf.transform.rotation.x))
                
                # servo to obj_1
                # OBJ REL TRANS
                obj1_trans_rel_x = obj1_trans_list[0].x - end_tf.transform.translation.x
                obj1_trans_rel_y = obj1_trans_list[0].y - end_tf.transform.translation.y
                obj1_trans_rel_z = obj1_trans_list[0].z - end_tf.transform.translation.z

                # OBJ REL ROLL, PITCH YAW
                roll1 = math.atan2(2.0 * (obj1_rot_list[0].z * obj1_rot_list[0].y + obj1_rot_list[0].w * obj1_rot_list[0].x) , 1.0 - 2.0 * (obj1_rot_list[0].x * obj1_rot_list[0].x + obj1_rot_list[0].y * obj1_rot_list[0].y))
                pitch1 = math.asin(2.0 * (obj1_rot_list[0].y * obj1_rot_list[0].w - obj1_rot_list[0].z * obj1_rot_list[0].x))
                yaw1 = math.atan2(2.0 * (obj1_rot_list[0].z * obj1_rot_list[0].w + obj1_rot_list[0].x * obj1_rot_list[0].y) , - 1.0 + 2.0 * (obj1_rot_list[0].w * obj1_rot_list[0].w + obj1_rot_list[0].x * obj1_rot_list[0].x))

                rel_roll1 = roll1 - roll_end_tf
                rel_pitch1 = pitch1 - pitch_end_tf
                rel_yaw1 = yaw1 - yaw_end_tf
                print('.....................')
                print('Box1', roll1, pitch1, yaw1)
                print('End Eff', roll_end_tf, pitch_end_tf, yaw_end_tf)
                print('Rel', rel_roll1, rel_pitch1, rel_yaw1)
      
                # SERVO, ATTACH, DETACH BOX 1                
                if i1<3 and j1<3:
                    if obj1_trans_rel_x > 0.0015 and i1==0:
                        self.servo_motion(obj1_trans_rel_x, obj1_trans_rel_y, obj1_trans_rel_z, rel_roll1, rel_pitch1, rel_yaw1)
                        print('Moving to Box 1: ', i1, j1)    # 0, 1
                    
                    else:
                        self.link_attach_client.attach_link('box1', 'link', 'ur5', 'wrist_3_link')
                        print("Attached to Box 1: ", i1, j1)
                        time.sleep(1)
                        i1=i1+1
                
                    if  i1==1 and j1==1:
                        if box1_align == -1 and box1_afterpick_aligned == 0:  # left rack
                            start_time=time.time()
                            while time.time() - start_time < duration:
                                self.servo_motion(0.0, -0.25, 0.0, 0.0, 0.0, 0.0)
                            #self.moveit2.move_to_configuration([13*3.1416/180, -136*3.1416/180, 102*3.1416/180, -146*3.1416/180, -104*3.1416/180, 180*3.1416/180])
                            #time.sleep(5)
                            box1_afterpick_aligned = 1

                        elif box1_align == 1 and box1_afterpick_aligned == 0: # right rack
                            start_time=time.time()
                            while time.time() - start_time < duration:
                                self.servo_motion(0.0, 0.25, 0.0, 0.0, 0.0, 0.0)
                            #self.moveit2.move_to_configuration([0*3.1416/180, -119*3.1416/180, 153*3.1416/180, -214*3.1416/180, -91*3.1416/180, 180*3.1416/180])
                            #time.sleep(5)
                            box1_afterpick_aligned == 1
                        elif box1_align == 0 and box1_afterpick_aligned == 0: # front rack
                            start_time=time.time()
                            while time.time() - start_time < duration:
                                self.servo_motion(-0.25, 0.0, 0.0, 0.0, 0.0, 0.0)
                            #self.moveit2.move_to_configuration([13*3.1416/180, -136*3.1416/180, 102*3.1416/180, -146*3.1416/180, -104*3.1416/180, 180*3.1416/180])
                            self.moveit2.move_to_configuration([-38*3.1416/180, -137*3.1416/180, 103*3.1416/180, -146*3.1416/180, -52*3.1416/180, 180*3.1416/180])
                            time.sleep(5)
                            box1_afterpick_aligned == 1

                        self.moveit2.move_to_configuration([2*3.1416/180, -136*3.1416/180, 111*3.1416/180, -155*3.1416/180, -93*3.1416/180, 180*3.1416/180])
                        j1=j1+1

                    elif i1==2 and j1==2:
                        self.moveit2.move_to_configuration([0*3.1416/180, -108*3.1416/180, -72*3.1416/180, -178*3.1416/180, -91*3.1416/180, 180*3.1416/180])
                        time.sleep(5)
                        j1=j1+1
                    
                else:
                    time.sleep(2)
                    self.link_detach_client.detach_link('box1', 'link', 'ur5', 'wrist_3_link')
                    #time.sleep(5)
                    j1=j1+1
                    box1_complete = 1
                    print("Detached from Box 1: ", i1, j1) # 2, 4

                if box1_complete == 1:
                    self.moveit2.move_to_configuration([0.0, -2.07694181, 2.670353756, -3.735004599, -1.5882496, 3.141592654])
                    box1_complete = 2
                    time.sleep(5)

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                self.get_logger().info(f"Transform lookup failed: {e}")
            
        
        ############# OBJ 3 ################

        if obj3_rot is not None and obj3_trans is not None and box3_complete == 0 :
            try:
                if box3_align == -1 and box3_aligned == 0:  # left rack
                    self.moveit2.move_to_configuration([0.0 + 1.57, -2.07694181, 2.670353756, -3.735004599, -1.5882496, 3.141592654])
                    time.sleep(5)
                    box3_aligned = 1

                elif box3_align == 1 and box3_aligned == 0: # right rack
                    self.moveit2.move_to_configuration([0.0 - 1.57, -2.07694181, 2.670353756, -3.735004599, -1.5882496, 3.141592654])
                    time.sleep(5)
                    box3_aligned = 1

                t3 = self.tf_buffer.lookup_transform('base_link', 'tool0', rclpy.time.Time())

                end_tf.header.stamp = self.get_clock().now().to_msg()
                end_tf.header.frame_id = 'base_link'
                end_tf.child_frame_id = 'tool0'

                end_tf.transform.translation.x = t3.transform.translation.x
                end_tf.transform.translation.y = t3.transform.translation.y
                end_tf.transform.translation.z = t3.transform.translation.z

                end_tf.transform.rotation.x = t3.transform.rotation.x
                end_tf.transform.rotation.y = t3.transform.rotation.y
                end_tf.transform.rotation.z = t3.transform.rotation.z
                end_tf.transform.rotation.w = t3.transform.rotation.w

                #print("end_tf", end_tf.transform.rotation.x, end_tf.transform.rotation.y, end_tf.transform.rotation.z, end_tf.transform.rotation.w)
                roll_end_tf = math.atan2(2.0 * (end_tf.transform.rotation.z * end_tf.transform.rotation.y + end_tf.transform.rotation.w * end_tf.transform.rotation.x) , 1.0 - 2.0 * (end_tf.transform.rotation.x * end_tf.transform.rotation.x + end_tf.transform.rotation.y * end_tf.transform.rotation.y))
                pitch_end_tf = math.asin(2.0 * (end_tf.transform.rotation.y * end_tf.transform.rotation.w - end_tf.transform.rotation.z * end_tf.transform.rotation.x))
                yaw_end_tf = math.atan2(2.0 * (end_tf.transform.rotation.z * end_tf.transform.rotation.w + end_tf.transform.rotation.x * end_tf.transform.rotation.y) , - 1.0 + 2.0 * (end_tf.transform.rotation.w * end_tf.transform.rotation.w + end_tf.transform.rotation.x * end_tf.transform.rotation.x))

                # OBJ REL TRANS
                obj3_trans_rel_x = obj3_trans_list[0].x - end_tf.transform.translation.x
                obj3_trans_rel_y = obj3_trans_list[0].y - end_tf.transform.translation.y
                obj3_trans_rel_z = obj3_trans_list[0].z - end_tf.transform.translation.z

                # OBJ ROLL, PITCH, YAW
                roll3 = math.atan2(2.0 * (obj3_rot_list[0].z * obj3_rot_list[0].y + obj3_rot_list[0].w * obj3_rot_list[0].x) , 1.0 - 2.0 * (obj3_rot_list[0].x * obj3_rot_list[0].x + obj3_rot_list[0].y * obj3_rot_list[0].y))
                pitch3 = math.asin(2.0 * (obj3_rot_list[0].y * obj3_rot_list[0].w - obj3_rot_list[0].z * obj3_rot_list[0].x))
                yaw3 = math.atan2(2.0 * (obj3_rot_list[0].z * obj3_rot_list[0].w + obj3_rot_list[0].x * obj3_rot_list[0].y) , - 1.0 + 2.0 * (obj3_rot_list[0].w * obj3_rot_list[0].w + obj3_rot_list[0].x * obj3_rot_list[0].x))
                
                rel_roll3 = roll3 - roll_end_tf
                rel_pitch3 = pitch3 - pitch_end_tf
                rel_yaw3 = yaw3 - yaw_end_tf

                print('.....................')
                print('Box3', roll3, pitch3, yaw3)
                print('End Eff', roll_end_tf, pitch_end_tf, yaw_end_tf)
                print('Rel', rel_roll3, rel_pitch3, rel_yaw3)
      
                # SERVO, ATTACH, DETACH BOX 3                
                if i3<3 and j3<3:
                    if obj3_trans_rel_x > 0.0015 and i3==0:
                        self.servo_motion(obj3_trans_rel_x, obj3_trans_rel_y, obj3_trans_rel_z, rel_roll3, rel_pitch3, rel_yaw3)
                        print('Moving to Box 3: ', i3, j3)    # 0, 1
                    
                    else:
                        self.link_attach_client.attach_link('box3', 'link', 'ur5', 'wrist_3_link')
                        print("Attached to Box 3: ", i3, j3)
                        time.sleep(1)
                        i3=i3+1
                
                    if  i3==1 and j3==1:
                        if box3_align == -1 and box3_afterpick_aligned == 0:  # left rack
                            start_time=time.time()
                            while time.time() - start_time < duration:
                                self.servo_motion(0.0, -0.25, 0.0, 0.0, 0.0, 0.0)
                            #self.moveit2.move_to_configuration([13*3.1416/180, -136*3.1416/180, 102*3.1416/180, -146*3.1416/180, -104*3.1416/180, 180*3.1416/180])
                            #time.sleep(5)
                            box3_afterpick_aligned = 1

                        elif box3_align == 1 and box3_afterpick_aligned == 0: # right rack
                            start_time=time.time()
                            while time.time() - start_time < duration:
                                self.servo_motion(0.0, 0.25, 0.0, 0.0, 0.0, 0.0)
                            #self.moveit2.move_to_configuration([0*3.1416/180, -119*3.1416/180, 153*3.1416/180, -214*3.1416/180, -91*3.1416/180, 180*3.1416/180])
                            #time.sleep(5)
                            box3_afterpick_aligned == 1
                        elif box3_align == 0 and box3_afterpick_aligned == 0: # front rack
                            start_time=time.time()
                            while time.time() - start_time < duration:
                                self.servo_motion(-0.25, 0.0, 0.0, 0.0, 0.0, 0.0)
                            #self.moveit2.move_to_configuration([13*3.1416/180, -136*3.1416/180, 102*3.1416/180, -146*3.1416/180, -104*3.1416/180, 180*3.1416/180])
                            self.moveit2.move_to_configuration([-38*3.1416/180, -137*3.1416/180, 103*3.1416/180, -146*3.1416/180, -52*3.1416/180, 180*3.1416/180])
                            time.sleep(5)
                            box3_afterpick_aligned == 1

                        self.moveit2.move_to_configuration([2*3.1416/180, -136*3.1416/180, 111*3.1416/180, -155*3.1416/180, -93*3.1416/180, 180*3.1416/180])
                        j3=j3+1

                    elif i3==2 and j3==2:
                        self.moveit2.move_to_configuration([0*3.1416/180, -108*3.1416/180, -72*3.1416/180, -178*3.1416/180, -91*3.1416/180, 180*3.1416/180])
                        time.sleep(5)
                        j3=j3+1
                    
                else:
                    time.sleep(2)
                    self.link_detach_client.detach_link('box3', 'link', 'ur5', 'wrist_3_link')
                    #time.sleep(5)
                    j3=j3+1
                    box3_complete = 1
                    print("Detached from Box 3: ", i3, j3) # 2, 4
                
                if box3_complete == 1:
                    self.moveit2.move_to_configuration([0.0, -2.07694181, 2.670353756, -3.735004599, -1.5882496, 3.141592654])
                    box3_complete = 2
                    time.sleep(5)

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                self.get_logger().info(f"Transform lookup failed: {e}")
        
        ############# OBJ 49 ################

        if obj49_rot is not None and obj49_trans is not None and box49_complete == 0 and box3_complete == 2:
            try:
                if box49_align == -1 and box49_aligned == 0:  # left rack
                    self.moveit2.move_to_configuration([0.0 + 1.57, -2.07694181, 2.670353756, -3.735004599, -1.5882496, 3.141592654])
                    time.sleep(5)
                    box49_aligned = 1

                elif box49_align == 1 and box49_aligned == 0: # right rack
                    self.moveit2.move_to_configuration([0.0 - 1.57, -2.07694181, 2.670353756, -3.735004599, -1.5882496, 3.141592654])
                    time.sleep(5)
                    box49_aligned = 1
                
                t3 = self.tf_buffer.lookup_transform('base_link', 'tool0', rclpy.time.Time())

                end_tf.header.stamp = self.get_clock().now().to_msg()
                end_tf.header.frame_id = 'base_link'
                end_tf.child_frame_id = 'tool0'

                end_tf.transform.translation.x = t3.transform.translation.x
                end_tf.transform.translation.y = t3.transform.translation.y
                end_tf.transform.translation.z = t3.transform.translation.z

                end_tf.transform.rotation.x = t3.transform.rotation.x
                end_tf.transform.rotation.y = t3.transform.rotation.y
                end_tf.transform.rotation.z = t3.transform.rotation.z
                end_tf.transform.rotation.w = t3.transform.rotation.w

                #print("end_tf", end_tf.transform.rotation.x, end_tf.transform.rotation.y, end_tf.transform.rotation.z, end_tf.transform.rotation.w)
            
                roll_end_tf = math.atan2(2.0 * (end_tf.transform.rotation.z * end_tf.transform.rotation.y + end_tf.transform.rotation.w * end_tf.transform.rotation.x) , 1.0 - 2.0 * (end_tf.transform.rotation.x * end_tf.transform.rotation.x + end_tf.transform.rotation.y * end_tf.transform.rotation.y))
                pitch_end_tf = math.asin(2.0 * (end_tf.transform.rotation.y * end_tf.transform.rotation.w - end_tf.transform.rotation.z * end_tf.transform.rotation.x))
                yaw_end_tf = math.atan2(2.0 * (end_tf.transform.rotation.z * end_tf.transform.rotation.w + end_tf.transform.rotation.x * end_tf.transform.rotation.y) , - 1.0 + 2.0 * (end_tf.transform.rotation.w * end_tf.transform.rotation.w + end_tf.transform.rotation.x * end_tf.transform.rotation.x))
                
                # OBJ REL TRANS

                obj49_trans_rel_x = obj49_trans_list[0].x - end_tf.transform.translation.x
                obj49_trans_rel_y = obj49_trans_list[0].y - end_tf.transform.translation.y
                obj49_trans_rel_z = obj49_trans_list[0].z - end_tf.transform.translation.z

                # OBJ REL ROLL, PITCH YAW
               
                roll49 = math.atan2(2.0 * (obj49_rot_list[0].z * obj49_rot_list[0].y + obj49_rot_list[0].w * obj49_rot_list[0].x) , 1.0 - 2.0 * (obj49_rot_list[0].x * obj49_rot_list[0].x + obj49_rot_list[0].y * obj49_rot_list[0].y))
                pitch49 = math.asin(2.0 * (obj49_rot_list[0].y * obj49_rot_list[0].w - obj49_rot_list[0].z * obj49_rot_list[0].x))
                yaw49 = math.atan2(2.0 * (obj49_rot_list[0].z * obj49_rot_list[0].w + obj49_rot_list[0].x * obj49_rot_list[0].y) , - 1.0 + 2.0 * (obj49_rot_list[0].w * obj49_rot_list[0].w + obj49_rot_list[0].x * obj49_rot_list[0].x))
                
                rel_roll49 = roll49 - roll_end_tf
                rel_pitch49 = pitch49 - pitch_end_tf
                rel_yaw49 = yaw49 - yaw_end_tf

                print('..................')
                print('Box49', roll49, pitch49, yaw49)
                print('End Eff', roll_end_tf, pitch_end_tf, yaw_end_tf)
                print('Rel', rel_roll49, rel_pitch49, rel_yaw49)
      
                # SERVO, ATTACH, DETACH BOX 49
                if i49<3 and j49<3:
                    if obj49_trans_rel_x > 0.0015 and i49==0:
                        self.servo_motion(obj49_trans_rel_x, obj49_trans_rel_y, obj49_trans_rel_z, rel_roll49, rel_pitch49, yaw49)
                        print('Moving to Box 49: ', i49, j49)    # 0, 1
                    
                    else:
                        self.link_attach_client.attach_link('box49', 'link', 'ur5', 'wrist_3_link')
                        print("Attached to Box 49: ", i49, j49)
                        time.sleep(1)
                        i49=i49+1
                
                    if  i49==1 and j49==1:
                        if box49_align == -1 and box49_afterpick_aligned == 0:  # left rack
                            start_time=time.time()
                            while time.time() - start_time < duration:
                                self.servo_motion(0.0, -0.25, 0.0, 0.0, 0.0, 0.0)
                            #self.moveit2.move_to_configuration([13*3.1416/180, -136*3.1416/180, 102*3.1416/180, -146*3.1416/180, -104*3.1416/180, 180*3.1416/180])
                            #time.sleep(5)
                            box49_afterpick_aligned = 1

                        elif box49_align == 1 and box49_afterpick_aligned == 0: # right rack
                            start_time=time.time()
                            while time.time() - start_time < duration:
                                self.servo_motion(0.0, 0.25, 0.0, 0.0, 0.0, 0.0)
                            #self.moveit2.move_to_configuration([0*3.1416/180, -119*3.1416/180, 153*3.1416/180, -214*3.1416/180, -91*3.1416/180, 180*3.1416/180])
                            #time.sleep(5)
                            box49_afterpick_aligned == 1
                        elif box49_align == 0 and box49_afterpick_aligned == 0: # front rack
                            start_time=time.time()
                            while time.time() - start_time < duration:
                                self.servo_motion(-0.25, 0.0, 0.0, 0.0, 0.0, 0.0)
                            #self.moveit2.move_to_configuration([13*3.1416/180, -136*3.1416/180, 102*3.1416/180, -146*3.1416/180, -104*3.1416/180, 180*3.1416/180])
                            self.moveit2.move_to_configuration([-38*3.1416/180, -137*3.1416/180, 103*3.1416/180, -146*3.1416/180, -52*3.1416/180, 180*3.1416/180])
                            time.sleep(5)
                            box49_afterpick_aligned == 1
                        self.moveit2.move_to_configuration([2*3.1416/180, -136*3.1416/180, 111*3.1416/180, -155*3.1416/180, -93*3.1416/180, 180*3.1416/180])
                        j49=j49+1

                    elif i49==2 and j49==2:
                        self.moveit2.move_to_configuration([0*3.1416/180, -134*3.1416/180, -41*3.1416/180, -185*3.1416/180, -91*3.1416/180, 180*3.1416/180])
                        time.sleep(5)
                        j49=j49+1
                    
                else:
                    time.sleep(2)
                    self.link_detach_client.detach_link('box49', 'link', 'ur5', 'wrist_3_link')
                    #time.sleep(5)
                    j49=j49+1
                    box49_complete = 1
                    print("Detached from Box 49: ", i49, j49) # 2, 4  

                if box49_complete == 1:
                    self.moveit2.move_to_configuration([0.0, -2.07694181, 2.670353756, -3.735004599, -1.5882496, 3.141592654])
                    box49_complete = 2
                    time.sleep(5) 

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                self.get_logger().info(f"Transform lookup failed: {e}")


def main():
    try:
        rclpy.init()
        node = Node("ex_servo")

        tf_subscriber_class = TFSubscriber()
        callback_group = ReentrantCallbackGroup()
        moveit2 = MoveIt2(node, joint_names=ur5.joint_names(), base_link_name=ur5.base_link_name(), end_effector_name=ur5.end_effector_name(), group_name=ur5.MOVE_GROUP_ARM, callback_group=callback_group)

        transit_pose = [0.0, -2.07694181, 2.670353756, -3.735004599, -1.5882496, 3.141592654]
        # moveit2.move_to_configuration([0.0 , -2.07694181, 2.670353756, -3.735004599, -1.5882496, 3.141592654])
        # time.sleep(3)
        # moveit2.move_to_configuration(transit_pose)
        # time.sleep(2)

        executor = rclpy.executors.MultiThreadedExecutor(2)
        executor.add_node(tf_subscriber_class)
        executor.add_node(node)
        executor.spin()

    except Exception as e:
        node.get_logger().error(f"An error occurred: {str(e)}")

    finally:
        if 'tf_subscriber_class' in locals():
            tf_subscriber_class.destroy_node()
            node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()