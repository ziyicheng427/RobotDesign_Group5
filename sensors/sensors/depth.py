import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

class DepthFilterNode(Node):
    """A ROS2 Node that publishes an amazing quote."""

    def __init__(self):
        super().__init__('Depth_Filter_Node')


        self.depthcam_subscriber = self.create_subscription(
            msg_type=PointCloud2,
            topic='/camera/camera/depth/color/points',
            callback=self.depthcam_subscriber_callback,
            qos_profile=1)
            
        self.depthcam_publisher = self.create_publisher(
            msg_type=PointCloud2,
            topic='/depthfilter',
            qos_profile=1)

        self.timer = self.create_timer(1.0, self.publish_depthcam_data)



    def depthcam_subscriber_callback(self, msg: PointCloud2):

        """
        Callback function for processing Image messages received from the /camera/depth/color/points topic.
        """
      
        self.point = msg
        
        
  

    def publish_depthcam_data(self):
        depthcam_msg = PointCloud2()
        # Populate imu_msg with your data
        self.depthcam_publisher.publish(depthcam_msg)
        self.get_logger().info('Publishing Depthcam data')

      


def main(args=None):
    """
    The main function.
    :param args: Not used directly by the user, but used by ROS2 to configure
    certain aspects of the Node.
    """
    try:
        rclpy.init(args=args)

        depth_filter_node = DepthFilterNode()

        rclpy.spin(depth_filter_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()
