import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
class ImuNode(Node):
    """A ROS2 Node that publishes an amazing quote."""

    def __init__(self):
        super().__init__('EKF_Node')

        self.imu_publisher = self.create_publisher(
            msg_type=Odometry,
            topic='/odom',
            qos_profile=1)
            
        self.imu_subscriber = self.create_subscription(
            msg_type=Imu,
            topic='/imu_raw',
            callback=self.imu_subscriber_callback,
            qos_profile=1)
            
        self.wheel_subscriber = self.create_subscription(
            msg_type=Odometry,
            topic='/wheel_odom_with_covariance',
            callback=self.wheel_subscriber_callback,
            qos_profile=1)

        self.timer = self.create_timer(1.0, self.publish_imu_data)


    def imu_subscriber_callback(self, msg: Imu):

        """
        Callback function for processing Image messages received from the /camera/depth/color/points topic.
        """
      
        self.point = msg
        
    def wheel_subscriber_callback(self, msg: Odometry):

        """
        Callback function for processing Image messages received from the /camera/depth/color/points topic.
        """
      
        self.point = msg
    def publish_imu_data(self):
        imu_msg = Odometry()
        # Populate imu_msg with your data
       
        #imu_msg.orientation.x = 0.1
        #imu_msg.orientation.y = 0.2
        #imu_msg.orientation.z = 0.3
        #imu_msg.orientation.w = 0.4

        #imu_msg.angular_velocity.x = 1.0
        #imu_msg.angular_velocity.y = 2.0
        #imu_msg.angular_velocity.z = 3.0

        #imu_msg.linear_acceleration.x = 0.5
        #imu_msg.linear_acceleration.y = 1.5
        #imu_msg.linear_acceleration.z = 2.5
        
        self.imu_publisher.publish(imu_msg)
        self.get_logger().info('Publishing IMU data')


def main(args=None):
    """
    The main function.
    :param args: Not used directly by the user, but used by ROS2 to configure
    certain aspects of the Node.
    """
    try:
        rclpy.init(args=args)

        imu_node = ImuNode()

        rclpy.spin(imu_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()

