import sys
import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

class Subscriber(Node):
    def __init__(self, bot):
        super().__init__('subscriber_control_node')
        self.bot = bot
        self.pose_subscription = self.create_subscription(
            msg_type=PoseStamped,
            topic='/armControl/set/pose',
            callback=self.position_callback,
            qos_profile=10)
        self.get_logger().info(f'Subscribing to {self.pose_subscription.topic}...')

    def position_callback(self, msg):
        position = msg.pose.position
        x = position.x
        y = position.y
        z = position.z
        self.get_logger().info(f'Received Robot Current Position: X={x}, Y={y}, Z={z}')

        # Call your control logic here based on received position
        self.control_robot(position)

    def control_robot(self, position):
        if position.x >= 1:
            self.bot.arm.go_to_sleep_pose()
            self.bot.arm.go_to_home_pose()
            self.bot.gripper.release()
            self.bot.arm.set_single_joint_position(joint_name='shoulder', position=np.deg2rad(15))
            self.bot.arm.set_single_joint_position(joint_name='elbow', position=np.deg2rad(10))
            self.bot.arm.set_single_joint_position(joint_name='wrist_angle', position=np.deg2rad(55))
            self.bot.gripper.grasp()
            self.bot.arm.set_single_joint_position(joint_name='wrist_angle', position=np.deg2rad(0))
            self.bot.arm.set_single_joint_position(joint_name='elbow', position=np.deg2rad(-10))
            self.bot.arm.set_single_joint_position(joint_name='waist', position=np.deg2rad(30))

            self.bot.gripper.release()
            self.bot.arm.set_single_joint_position(joint_name='waist', position=np.deg2rad(0))
            self.bot.arm.go_to_home_pose()
            self.bot.arm.set_ee_pose_components(x=0.2, z=0.2)
            self.bot.arm.go_to_sleep_pose()


def main(args=None):
    ##rclpy.init(args=args)  # Initialize ROS 2 context here
    bot = InterbotixManipulatorXS(
        robot_model='px150',
        group_name='arm',
        gripper_name='gripper'
    )

    if bot.arm.group_info.num_joints < 5:
        bot.core.get_logger().fatal('This demo requires the robot to have at least 5 joints!')
        bot.shutdown()
        sys.exit()

    try:
        subscriber_control_node = Subscriber(bot)
        rclpy.spin(subscriber_control_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)
    finally:
        bot.shutdown()
        rclpy.shutdown()  # Shutdown ROS 2 context


if __name__ == '__main__':
    main()
