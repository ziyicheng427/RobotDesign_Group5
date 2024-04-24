import rclpy

from rclpy.node import Node

from example_interfaces.msg import Bool


from geometry_msgs.msg import PoseStamped, PointStamped

class FlagControlNode(Node):
    def __init__(self):
        super().__init__('flag_control_node')

        self.armControl = self.create_publisher(
            msg_type = Bool,
            topic = '/armControl/get/flag',
            qos_profile = 10)
        self.get_logger().info(f'Publishing to {self.armControl.topic}...')

        bool_flag = Bool
        bool_flag = True

        self.armControl.publish(bool_flag)




def main(args=None):

    """
    The main function.
    :param args: Not used directly by the user, but used by ROS2 to configure
    certain aspects of the Node.
    """
    try:
        rclpy.init(args=args)
        flag_control_node = FlagControlNode()
        rclpy.spin(flag_control_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)
if __name__ == '__main__':
    main()
