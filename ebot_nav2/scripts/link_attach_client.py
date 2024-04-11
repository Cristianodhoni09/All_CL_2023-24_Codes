# from linkattacher_msgs.srv import AttachLink

# import rclpy
# from rclpy.node import Node
# from ebot_docking.srv import DockSw
# from sensor_msgs.msg import Imu, Range
# from tf_transformations import euler_from_quaternion
# import math

# link_attach_cli = self.create_client(AttachLink, '/ATTACH_LINK')

# while not self.link_attach_cli.wait_for_service(timeout_sec=1.0):
#       self.get_logger().info('Link attacher service not available, waiting again...')

# req = AttachLink.Request()
# req.model1_name =  'ebot'     
# req.link1_name  = 'ebot_base_link'       
# req.model2_name =  'RACK1'       
# req.link2_name  = 'link'  

# link_attach_cli.call_async(req)

# class LinkAttachClientNode(Node):
#       def __init__(self)
            

import rclpy
from rclpy.node import Node
from linkattacher_msgs.srv import AttachLink  # Import the custom service message

class AttachLinkClientNode(Node):

    def __init__(self):
        super().__init__('attach_link_client')
        self.link_attach_cli = self.create_client(AttachLink, '/ATTACH_LINK')

        while not self.link_attach_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Link attacher service not available, waiting again...')

    def attach_link(self, model1_name, link1_name, model2_name, link2_name):
        req = AttachLink.Request()
        req.model1_name = model1_name
        req.link1_name = link1_name
        req.model2_name = model2_name
        req.link2_name = link2_name
        
        future = self.link_attach_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info("Link attachment service call was successful: %s" % future.result().message)
        else:
            self.get_logger().info("Link attachment service call failed")

def main(args=None):
    rclpy.init(args=args)

    attach_link_client_node = AttachLinkClientNode()
    attach_link_client_node.attach_link('ebot', 'ebot_base_link', 'rack1', 'link')

    rclpy.shutdown()

if __name__ == '__main__':
    main()