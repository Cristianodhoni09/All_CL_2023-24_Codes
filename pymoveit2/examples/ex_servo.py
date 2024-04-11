#!/usr/bin/env python3


from distutils.command.build_scripts import first_line_re
import tf2_ros
import geometry_msgs
import tf2_msgs
from tf_transformations import euler_from_quaternion
import time

from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5
#from pymoveit2_msgs.msg import PlanningParameters


from link_attach import AttachLinkNode
from link_detach import DetachLinkNode
#from gripper_magnet import AttachLinkNode
from std_srvs.srv import Trigger
from std_msgs.msg import Bool

from switch_controller import SwitchControl

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
m=1
obj_translations = []
obj_rotations = []
obj_Id=[]
box_align=0
box_picked=0
duration=1.5
BoxNumber=0
processed_boxes = []
first_loop=0
speed=2

class TFSubscriber(Node):
    def __init__(self):
        super().__init__('tf_subscriber')
        self.servo_start()

        #time.sleep(3)

        self.tf_buffer = tf2_ros.buffer.Buffer()
        self.twist_pub = self.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
        self.br = tf2_ros.TransformBroadcaster(self)
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.subscription = self.create_subscription(TFMessage, '/tf', self.tf_callback, 10)
        self.subscription  # prevent unused variable warning

        self.link_attach_client = AttachLinkNode()
        self.link_detach_client = DetachLinkNode()

        #self.link_attach_client = AttachLinkNode()

        self.switch = SwitchControl()

        callback_group = ReentrantCallbackGroup()
        self.moveit2 = MoveIt2(node=self, joint_names=ur5.joint_names(), base_link_name=ur5.base_link_name(), end_effector_name=ur5.end_effector_name(), group_name=ur5.MOVE_GROUP_ARM, callback_group=callback_group)
        # self.timer = self.create_timer(0.2, self.servo_motion)
        self.status=False
        # print(self.status)
        self.attach_status_subscription = self.create_subscription(
            Bool,
            '/attach_status',
            self.attach_status_callback,
            10
        )
        self.attach_status_subscription  # prevent unused variable warning
        # print(self.msg)
    def attach_status_callback(self, msg):
        self.status=msg.data
        # self.status=False
        # print('CallBack FUnction Called')
        
        # self.timer = self.create_timer(0.1, self.arm_flag)

     

    '''def movetoconfig(self):
        transit_pose = [0.0, -2.07694181, 2.670353756, -3.735004599, -1.5882496, 3.141592654]
        self.moveit2.move_to_configuration(transit_pose)'''

    def servo_motion(self, obj_trans_rel_x, obj_trans_rel_y, obj_trans_rel_z, roll, pitch, yaw):
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()

        twist_msg.twist.linear.x = obj_trans_rel_x
        twist_msg.twist.linear.y = obj_trans_rel_y
        twist_msg.twist.linear.z = obj_trans_rel_z
        twist_msg.header.frame_id = 'base_link'  # The frame of reference

        twist_msg.twist.angular.x = roll
        twist_msg.twist.angular.y = pitch
        twist_msg.twist.angular.z = yaw

        self.twist_pub.publish(twist_msg)
              
        # lin = (obj_trans_rel_x, obj_trans_rel_y, obj_trans_rel_z)
        # ang = (roll, pitch, yaw)

        # MoveIt2Servo.servo(self, lin, ang)'''
    def lookup(self,obj_translations,obj_rotations,box1_align):
        end_tf = TransformStamped()

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

        
        box_roll = math.atan2(2.0 * (obj_rotations[-1].z * obj_rotations[-1].y + obj_rotations[-1].w * obj_rotations[-1].x) , 1.0 - 2.0 * (obj_rotations[-1].x * obj_rotations[-1].x + obj_rotations[-1].y * obj_rotations[-1].y))
        box_pitch = math.asin(2.0 * (obj_rotations[-1].y * obj_rotations[-1].w - obj_rotations[-1].z * obj_rotations[-1].x))
        box_yaw = math.atan2(2.0 * (obj_rotations[-1].z * obj_rotations[-1].w + obj_rotations[-1].x * obj_rotations[-1].y) , - 1.0 + 2.0 * (obj_rotations[-1].w * obj_rotations[-1].w + obj_rotations[-1].x * obj_rotations[-1].x))

                                # OBJ REL TRANS
        obj_trans_rel_x = obj_translations[-1].x - end_tf.transform.translation.x
        obj_trans_rel_y = obj_translations[-1].y - end_tf.transform.translation.y
        obj_trans_rel_z = obj_translations[-1].z - end_tf.transform.translation.z

        if box_align==0 :
            rel_roll=0.0
            rel_pitch=0.0
            rel_yaw = (box_yaw) - (yaw_end_tf)

        if box_align==1:
            rel_roll=0.0
            rel_pitch=0.0
            rel_yaw = (box_yaw) - (yaw_end_tf)


        if box_align==-1:
            rel_roll=0.0
            rel_pitch=0.0
            rel_yaw = abs(box_yaw) - abs(yaw_end_tf)

        return speed*obj_trans_rel_x,speed*obj_trans_rel_y,speed*obj_trans_rel_z,speed*rel_roll,speed*rel_pitch,speed*rel_yaw
    

    def servo_start(self):
            
            self.get_logger().info('Starting Servo please wait')

            # Use the correct service client instance
            self.servo_node_start = self.create_client(Trigger, '/servo_node/start_servo')

            while not self.servo_node_start.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn('/start_servo not available. Waiting for /servo_node to become available.')

            self.request_servo_start = Trigger.Request()
            # Use the correct service client instance
            self.servo_start_resp = self.servo_node_start.call_async(self.request_servo_start)
            rclpy.spin_until_future_complete(self, self.servo_start_resp)

            if self.servo_start_resp.result().success:
                self.get_logger().info(self.servo_start_resp.result().message)
            else:
                self.get_logger().warn(self.servo_start_resp.result().message)

    
    def tf_callback(self,msg):

        global m
        global obj_translations 
        global obj_rotations 
        global box_align
        global box_picked
        global BoxNumber
        global processed_boxes 
        global first_loop
        global speed

        # print('Status of box is ;', self.status )

        if self.status== True:
            return
        
  
        for i in range(1, 50):
            obj_frame_id = f"obj_{i}"
            stored_trans=0
            stored_rot=0

            if obj_frame_id in processed_boxes:
               continue

            for transform in msg.transforms:
                
                if transform.header.frame_id == "base_link" and transform.child_frame_id == f"2935_base_{i}":
                    print(f'Box Detected, obj{i}') 
                    
                    obj_trans = transform.transform.translation
                    obj_rot = transform.transform.rotation
                    if obj_trans is not None and obj_rot is not None:
                        if self.status== True:
                            return
                        print('obj not empty',obj_frame_id)  
                        obj_translations.append(obj_trans)
                        obj_rotations.append(obj_rot)
                        obj_Id.append(i)
                        box_picked=0
                        processed_boxes.append(obj_frame_id)
                        first_loop=first_loop+1
                        print('Object Id list is',obj_Id)
                    else:
                        continue

                    '''if first_loop==0 or first_loop==1:
                        continue

                    if (obj_rotations[-1].x - obj_rotations[-2].x > 0.01) or \
                        (obj_rotations[-1].y - obj_rotations[-2].y > 0.01) or \
                        (obj_rotations[-1].z - obj_rotations[-2].z > 0.01) or \
                        (obj_rotations[-1].w - obj_rotations[-2].w > 0.01):
                        continue'''

                    print('Object Id is',obj_Id[-1])    
                    #box_roll = math.atan2(2.0 * (obj_rotations[-1].z * obj_rotations[-1].y + obj_rotations[-1].w * obj_rotations[-1].x) , 1.0 - 2.0 * (obj_rotations[-1].x * obj_rotations[-1].x + obj_rotations[-1].y * obj_rotations[-1].y))
                    #box_pitch = math.asin(2.0 * (obj_rotations[-1].y * obj_rotations[-1].w - obj_rotations[-1].z * obj_rotations[-1].x))
                    box_yaw = math.atan2(2.0 * (obj_rotations[-1].z * obj_rotations[-1].w + obj_rotations[-1].x * obj_rotations[-1].y) , - 1.0 + 2.0 * (obj_rotations[-1].w * obj_rotations[-1].w + obj_rotations[-1].x * obj_rotations[-1].x))
                    
                    if abs(box_yaw) <= 3.14 and abs(box_yaw) >= 2.65:
                        box_align = -1 # in left rack

                    elif abs(box_yaw) <= 2.2 and abs(box_yaw) >= 1.00:
                        box_align = 0  # in centre
                    
                    elif abs(box_yaw) <= 0.45 and abs(box_yaw) >= 0.00:
                        box_align = 1 # in right rack
                    
                    
                    print(f"obj_{i} translation", obj_translations[-1])
                    print(f"obj_{i} rotation", obj_rotations[-1])
                    #print(f"obj_{i} roll, pitch, yaw: ", box_roll, box_pitch, box_yaw)
                    print(f"obj_{i} align", box_align,obj_frame_id, box_yaw)
                    #print('hello world')
                    if box_picked==0:
                        box_picked=1
                        #end_tf = TransformStamped()
                        
                        if box_align == -1:  # left rack
                            self.moveit2.move_to_configuration([0.0 + 1.57, -2.07694181, 2.670353756, -3.735004599, -1.5882496, 3.141592654])
                            self.moveit2.wait_until_executed()
                            #time.sleep(4)
                                

                        elif box_align == 1: # right rack
                            self.moveit2.move_to_configuration([0.0 - 1.57, -2.07694181, 2.670353756, -3.735004599, -1.5882496, 3.141592654])
                            self.moveit2.wait_until_executed()
                            #time.sleep(4)
                            
                        elif box_align == 0:  # front rack
                            self.moveit2.move_to_configuration([0*3.1416/180, -137*3.1416/180, 138*3.1416/180, -180*3.1416/180, -91*3.1416/180, 180*3.1416/180])
                            self.moveit2.wait_until_executed()
                            # time.sleep(4)

                        print('Moved to pre pick pose')
                        obj_trans_rel_x,obj_trans_rel_y,obj_trans_rel_z,rel_roll,rel_pitch,rel_yaw=self.lookup(obj_translations,obj_rotations,box_align)
                        distance=math.sqrt(obj_trans_rel_x**2 + obj_trans_rel_y**2 + obj_trans_rel_z**2)
                        #self.switch.switch_control(1)



                        obj_trans_rel_x1=obj_trans_rel_x
                        obj_trans_rel_y1=obj_trans_rel_y
                        obj_trans_rel_z1=obj_trans_rel_z
                        if box_align==0:
                            while  obj_trans_rel_x> 0.015 :
                                self.servo_motion(obj_trans_rel_x, obj_trans_rel_y, obj_trans_rel_z, rel_roll, rel_pitch, rel_yaw)
                                obj_trans_rel_x,obj_trans_rel_y,obj_trans_rel_z,rel_roll,rel_pitch,rel_yaw=self.lookup(obj_translations,obj_rotations,box_align)
                                #obj_trans_rel_x1,obj_trans_rel_y1,obj_trans_rel_z1,rel_roll1,rel_pitch1,rel_yaw1=self.lookup(obj_translations,obj_rotations,box_align)
                                #distance=math.sqrt(obj_trans_rel_x**2 + obj_trans_rel_y**2 + obj_trans_rel_z**2)
                                # print(f'Moving to Box{i},   distance_x = ',obj_trans_rel_x, 'distance_y = ',obj_trans_rel_y,'distance_z = ',obj_trans_rel_z)    # 0, 1
                            print('reached box')
                        if box_align==1 or box_align==-1:
                            while  abs(obj_trans_rel_y)> 0.015 :
                                self.servo_motion(obj_trans_rel_x, obj_trans_rel_y, obj_trans_rel_z, rel_roll, rel_pitch, rel_yaw)
                                obj_trans_rel_x,obj_trans_rel_y,obj_trans_rel_z,rel_roll,rel_pitch,rel_yaw=self.lookup(obj_translations,obj_rotations,box_align)
                                #obj_trans_rel_x1,obj_trans_rel_y1,obj_trans_rel_z1,rel_roll1,rel_pitch1,rel_yaw1=self.lookup(obj_translations,obj_rotations,box_align)
                                #distance=math.sqrt(obj_trans_rel_x**2 + obj_trans_rel_y**2 + obj_trans_rel_z**2)
                                # print(f'Moving to Box{i},   distance_x = ',obj_trans_rel_x, 'distance_y = ',obj_trans_rel_y,'distance_z = ',obj_trans_rel_z)    # 0, 1
                            print('reached box')

                        
                        distance=math.sqrt(obj_trans_rel_x**2 + obj_trans_rel_y**2 + obj_trans_rel_z**2)
                        self.link_attach_client.attach_link(f'box{i}', 'link', 'ur5', 'wrist_3_link')
                        # self.link_attach_client.gripper_call(True)
                        print(f"Attached to Box{i}")
                        #time.sleep(1)
                        

                        if box_align == -1 :  # left rack
                            '''start_time=time.time()
                            while time.time() - start_time < duration:
                                self.servo_motion(0.0, -0.25, 0.05, 0.0, 0.0, 0.0)
                                #print('Moving out')
                            print('Moved out')'''
                            #self.switch.switch_control(0)
                            #self.moveit2.move_to_configuration([-38*3.1416/180, -137*3.1416/180, 103*3.1416/180, -146*3.1416/180, -52*3.1416/180, 180*3.1416/180])
                            # self.moveit2.move_to_configuration([13*3.1416/180, -136*3.1416/180, 102*3.1416/180, -146*3.1416/180, -104*3.1416/180, 180*3.1416/180]) #new
                            self.moveit2.move_to_configuration([33*3.1416/180, -97*3.1416/180, 84*3.1416/180, -167*3.1416/180, -37*3.1416/180, 180*3.1416/180])
                            self.moveit2.wait_until_executed()
                            self.moveit2.move_to_configuration([90*3.1416/180, -136*3.1416/180, 102*3.1416/180, -146*3.1416/180, -104*3.1416/180, 180*3.1416/180])
                            self.moveit2.wait_until_executed()
                            self.moveit2.move_to_configuration([13*3.1416/180, -136*3.1416/180, 102*3.1416/180, -146*3.1416/180, -104*3.1416/180, 180*3.1416/180])
                            self.moveit2.wait_until_executed()
                            # time.sleep(2)
                            #self.moveit2.move_to_configuration([13*3.1416/180, -136*3.1416/180, 102*3.1416/180, -146*3.1416/180, -104*3.1416/180, 180*3.1416/180])
                            #time.sleep(5)

                        elif box_align == 1 : # right rack
                            '''start_time=time.time()
                            while time.time() - start_time < duration:
                                self.servo_motion(0.0, 0.25, 0.05, 0.0, 0.0, 0.0)
                                #print('Moving out')
                            print('Moved out')'''
                            #self.switch.switch_control(0)
                            # self.moveit2.move_to_configuration([-38*3.1416/180, -137*3.1416/180, 103*3.1416/180, -146*3.1416/180, -52*3.1416/180, 180*3.1416/180])
                            # self.moveit2.move_to_configuration([13*3.1416/180, -136*3.1416/180, 102*3.1416/180, -146*3.1416/180, -104*3.1416/180, 180*3.1416/180]) #new
                            self.moveit2.move_to_configuration([-108*3.1416/180, -124*3.1416/180, 101*3.1416/180, -156*3.1416/180, -71*3.1416/180, 180*3.1416/180])
                            self.moveit2.wait_until_executed()
                            self.moveit2.move_to_configuration([-90*3.1416/180, -136*3.1416/180, 102*3.1416/180, -146*3.1416/180, -104*3.1416/180, 180*3.1416/180])
                            self.moveit2.wait_until_executed()
                            self.moveit2.move_to_configuration([13*3.1416/180, -136*3.1416/180, 102*3.1416/180, -146*3.1416/180, -104*3.1416/180, 180*3.1416/180])
                            self.moveit2.wait_until_executed()
                            # time.sleep(2)
                            #self.moveit2.move_to_configuration([0*3.1416/180, -119*3.1416/180, 153*3.1416/180, -214*3.1416/180, -91*3.1416/180, 180*3.1416/180])
                            #time.sleep(5)
                            
                        elif box_align == 0 : # front rack
                            '''start_time=time.time()
                            while time.time() - start_time < duration:
                                self.servo_motion(-0.25, 0.0, 0.05, 0.0, 0.0, 0.0)
                                # print('Moving Out')
                            print('Moved out')'''
                            #self.switch.switch_control(0)
                            self.moveit2.move_to_configuration([13*3.1416/180, -136*3.1416/180, 102*3.1416/180, -146*3.1416/180, -104*3.1416/180, 180*3.1416/180])
                            # self.moveit2.move_to_configuration([-38*3.1416/180, -137*3.1416/180, 103*3.1416/180, -146*3.1416/180, -52*3.1416/180, 180*3.1416/180])
                            self.moveit2.wait_until_executed()
                            # self.moveit2.move_to_configuration([1*3.1416/180, -147*3.1416/180, 134*3.1416/180, -167*3.1416/180, -91*3.1416/180, 180*3.1416/180])
                            # self.moveit2.wait_until_executed()
                            # # time.sleep(2)
                        
                        print('Moved to post pick pose')

                            #self.moveit2.move_to_configuration([13*3.1416/180, -136*3.1416/180, 102*3.1416/180, -146*3.1416/180, -104*3.1416/180, 180*3.1416/180])
                        '''self.moveit2.move_to_configuration([-38*3.1416/180, -137*3.1416/180, 103*3.1416/180, -146*3.1416/180, -52*3.1416/180, 180*3.1416/180])
                        time.sleep(2)
                        self.moveit2.move_to_configuration([2*3.1416/180, -136*3.1416/180, 111*3.1416/180, -155*3.1416/180, -93*3.1416/180, 180*3.1416/180])
                        time.sleep(2)'''

                        if BoxNumber==0:
                            self.moveit2.move_to_configuration([0*3.1416/180, -108*3.1416/180, -72*3.1416/180, -178*3.1416/180, -91*3.1416/180, 180*3.1416/180])
                            self.moveit2.wait_until_executed()
                            # time.sleep(5)
                            BoxNumber=1
                        elif BoxNumber==1:
                            self.moveit2.move_to_configuration([0*3.1416/180, -145*3.1416/180, -21*3.1416/180, -194*3.1416/180, -90*3.1416/180, 180*3.1416/180])
                            self.moveit2.wait_until_executed()
                            # time.sleep(5)
                            BoxNumber=2
                        elif BoxNumber==2:
                            self.moveit2.move_to_configuration([0*3.1416/180, -108*3.1416/180, -72*3.1416/180, -178*3.1416/180, -91*3.1416/180, 180*3.1416/180])
                            self.moveit2.wait_until_executed()
                            # time.sleep(6)
                            BoxNumber=0
                        print('Moved to drop pose')


                        '''self.moveit2.move_to_configuration([0*3.1416/180, -108*3.1416/180, -72*3.1416/180, -178*3.1416/180, -91*3.1416/180, 180*3.1416/180])
                        time.sleep(5)'''

                        self.link_detach_client.detach_link(f'box{i}', 'link', 'ur5', 'wrist_3_link')
                        # self.link_attach_client.gripper_call(False)
                        print(f'box{i} complete')
                        self.moveit2.move_to_configuration([0.0, -2.07694181, 2.670353756, -3.735004599, -1.5882496, 3.141592654])
                        self.moveit2.wait_until_executed()
                        # time.sleep(3)




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


