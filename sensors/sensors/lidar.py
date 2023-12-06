import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan 

class LidarFilterNode(Node):
    """A ROS2 Node that publishes an amazing quote."""

    def __init__(self):
        super().__init__('Lidar_Filter_Node')


        self.lidar_subscriber = self.create_subscription(
            msg_type=LaserScan,
            topic='/scan',
            callback=self.lidar_subscriber_callback,
            qos_profile=1)
            
        self.lidar_publisher = self.create_publisher(
            msg_type=LaserScan,
            topic='/lidarfilter',
            qos_profile=1)

        self.timer = self.create_timer(1.0, self.publish_lidar_data)



    def lidar_subscriber_callback(self, msg: LaserScan):

        """
        Callback function for processing Image messages received from the /scan topic.
        """
      
        self.point = msg
        
        
  

    def publish_lidar_data(self):
        lidar_msg = LaserScan()
        # Populate imu_msg with your data
        self.lidar_publisher.publish(lidar_msg)
        self.get_logger().info('Publishing Lidar data')

      


def main(args=None):
    """
    The main function.
    :param args: Not used directly by the user, but used by ROS2 to configure
    certain aspects of the Node.
    """
    try:
        rclpy.init(args=args)

        lidar_filter_node = LidarFilterNode()

        rclpy.spin(lidar_filter_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()

