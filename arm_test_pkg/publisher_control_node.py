import rclpy
import numpy as np
from rclpy.node import Node

from example_interfaces.srv import SetBool

from geometry_msgs.msg import PoseStamped, PointStamped


class PubisherControlNode(Node):
    def __init__(self):
        super().__init__('publisher_control_node')

        self.armControl = self.create_publisher(
            msg_type = PoseStamped,
            topic = '/armControl/set/pose',
            qos_profile = 10)
        self.get_logger().info(f'Publishing to {self.armControl.topic}...')

        pose = PoseStamped()
        pose.pose.position.x = 1.00
        pose.pose.position.y = 0.20
        pose.pose.position.z = 0.20

        pose.pose.orientation.w = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0

        self.armControl.publish(pose)




def main(args=None):

    """
    The main function.
    :param args: Not used directly by the user, but used by ROS2 to configure
    certain aspects of the Node.
    """
    try:
        rclpy.init(args=args)
        publisher_control_node = PubisherControlNode()
        rclpy.spin(publisher_control_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)
if __name__ == '__main__':
    main()
