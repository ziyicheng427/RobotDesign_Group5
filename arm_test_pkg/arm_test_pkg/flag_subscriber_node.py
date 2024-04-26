
import sys

from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

import time


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

    def position_callback(self, msg : PoseStamped, bot):
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
        bot.arm.go_to_home_pose()
        bot.gripper.release()
        bot.arm.set_single_joint_position(joint_name='shoulder', position=np.deg2rad(15))
        bot.arm.set_single_joint_position(joint_name='elbow', position=np.deg2rad(10))
        bot.arm.set_single_joint_position(joint_name='wrist_angle', position=np.deg2rad(55))
        bot.gripper.grasp()
        bot.arm.set_single_joint_position(joint_name='wrist_angle', position=np.deg2rad(0))
        bot.arm.set_single_joint_position(joint_name='elbow', position=np.deg2rad(-10))
        bot.arm.set_single_joint_position(joint_name='waist', position=np.deg2rad(30))

        bot.gripper.release()
        bot.arm.set_single_joint_position(joint_name='waist', position=np.deg2rad(0))
        bot.arm.go_to_home_pose()
        bot.arm.set_ee_pose_components(x=0.2, z=0.2)
        bot.arm.go_to_sleep_pose()

        bot.shutdown()

        return position

def main(args=None):
    bot = InterbotixManipulatorXS(
        robot_model='px150',
        group_name='arm',
        gripper_name='gripper'
    )

    if (bot.arm.group_info.num_joints < 5):
        bot.core.get_logger().fatal('This demo requires the robot to have at least 5 joints!')
        bot.shutdown()
        sys.exit()

    bot.arm.go_to_sleep_pose()
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

