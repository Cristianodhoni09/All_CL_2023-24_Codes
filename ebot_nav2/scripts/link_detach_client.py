import rclpy
from rclpy.node import Node
from linkattacher_msgs.srv import DetachLink  # Import the custom service message

class DetachLinkClientNode(Node):

    def __init__(self):
        super().__init__('detach_link_client')
        self.link_detach_cli = self.create_client(DetachLink, '/DETACH_LINK')

        while not self.link_detach_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Link detacher service not available, waiting again...')

    def detach_link(self, model1_name, link1_name, model2_name, link2_name):
        req = DetachLink.Request()
        req.model1_name = model1_name
        req.link1_name = link1_name
        req.model2_name = model2_name
        req.link2_name = link2_name

        future = self.link_detach_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info("Link detachment service call was successful: %s" % future.result().message)
        else:
            self.get_logger().info("Link detachment service call failed")

def main(args=None):
    rclpy.init(args=args)

    detach_link_client_node = DetachLinkClientNode()
    detach_link_client_node.detach_link('ebot', 'ebot_base_link', 'rack1', 'link')

    rclpy.shutdown()

if __name__ == '__main__':
    main()