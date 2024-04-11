import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import SwitchController # module call


class SwitchControl(Node):
    def __init__(self):
        super().__init__('switch_controller')

        self.__contolMSwitch = self.create_client(SwitchController, "/controller_manager/switch_controller")
        
        while not self.__contolMSwitch.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(f"Service control Manager is not yet available...")

    def switch_control(self, state):
        # Parameters to switch controller
        switchParam = SwitchController.Request()
        if state == 0:
            switchParam.activate_controllers = ["scaled_joint_trajectory_controller"] # for normal use of moveit
            switchParam.deactivate_controllers = ["forward_position_controller"] # for servoing
        
        else:
            switchParam.activate_controllers = ["forward_position_controller"] # for servoing
            switchParam.deactivate_controllers = ["scaled_joint_trajectory_controller"] # for normal use of moveit

        
        switchParam.strictness = 2
        switchParam.start_asap = False

        # calling control manager service after checking its availability
        # while not self.__contolMSwitch.wait_for_service(timeout_sec=5.0):
        #     self.get_logger().warn(f"Service control Manager is not yet available...")

        self.__contolMSwitch.call_async(switchParam)
        print("[CM]: Switching Complete")   

def main(args=None):
    rclpy.init(args=args)

    # attach_link_client_node = AttachLinkNode()
    # attach_link_client_node.attach_link('ebot', 'ebot_base_link', 'rack1', 'link')

    rclpy.shutdown()

if __name__ == '__main__':
    main()

