import rclpy
import numpy as np

from rclpy.node import Node

from example_interfaces.srv import SetBool

from geometry_msgs.msg import PoseStamped, PointStamped


class Subscriber(Node):
    def __init__(self):
        super().__init__('subscriber_control_node')

        self.pose_subscription = self.create_subscription(
            msg_type=PoseStamped,
            topic='/armcontrol/set/pose',
            callback=self.position_callback,
            qos_profile=10)
        self.get_logger().info(f'Subscribing to {self.pose_subscription.topic}...')

    def position_callback(self, msg : PoseStamped):
        # recieves the current pose.position and pose.orientation data
        position = msg.pose.position
        orientation = msg.pose.orientation
        x = position.x
        y = position.y
        z = position.z  # z = 0

        w_or = orientation.w
        z_or = orientation.z
        x_or = orientation.x  # x_or = 0
        y_or = orientation.y  # y_or = 0

        self.get_logger().info(f'Received Robot Current Position: ')
        self.get_logger().info(f'X Position = {x} Y Position = {y} Z Position = {z}')
        return position


def main(args=None):
    """
    The main function.
    :param args: Not used directly by the user, but used by ROS2 to configure
    certain aspects of the Node.
    """
    try:
        rclpy.init(args=args)
        subscriber_control_node = Subscriber()
        rclpy.spin(subscriber_control_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()