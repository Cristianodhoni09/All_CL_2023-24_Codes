import rclpy
from rclpy.node import Node
from linkattacher_msgs.srv import DetachLink


class DetachLinkNode(Node):
    def __init__(self):
        super().__init__('attach_link')
        self.gripper_control = self.create_client(DetachLink, '/GripperMagnetOFF')

        while not self.gripper_control.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('EEF service not available, waiting again...')

    def detach_link(self, model1_name, link1_name, model2_name, link2_name):
        req = DetachLink.Request()
        req.model1_name =  model1_name
        req.link1_name  = link1_name
        req.model2_name = model2_name
        req.link2_name  = link2_name

        future = self.gripper_control.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info("Link detachment service call was successful: %s" % future.result().message)
        else:
            self.get_logger().info("Link detachment service call failed")

def main(args=None):
    rclpy.init(args=args)

    # attach_link_client_node = DetachLinkNode()
    # attach_link_client_node.attach_link('ebot', 'ebot_base_link', 'rack1', 'link')

    rclpy.shutdown()

if __name__ == '__main__':
    main()

