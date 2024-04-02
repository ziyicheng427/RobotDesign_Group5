import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class CameraFilterNode(Node):
    """A ROS2 Node that publishes an amazing quote."""

    def __init__(self):
        super().__init__('Camera_Filter_Node')


        self.camera_subscriber = self.create_subscription(
            msg_type=Image,
            topic='/camera/image_raw',
            callback=self.camera_subscriber_callback,
            qos_profile=1)
            
        self.camera_publisher = self.create_publisher(
            msg_type=Image,
            topic='/camerafilter',
            qos_profile=1)

        self.timer = self.create_timer(1.0, self.publish_camera_data)



    def camera_subscriber_callback(self, msg: Image):

        """
        Callback function for processing Image messages received from the /camera/image_raw topic.
        """
      
        self.image = msg
        
        
  

    def publish_camera_data(self):
        camera_msg = Image()
        # Populate imu_msg with your data
        self.camera_publisher.publish(camera_msg)
        self.get_logger().info('Publishing Camera data')

      


def main(args=None):
    """
    The main function.
    :param args: Not used directly by the user, but used by ROS2 to configure
    certain aspects of the Node.
    """
    try:
        rclpy.init(args=args)

        camera_filter_node = CameraFilterNode()

        rclpy.spin(camera_filter_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()

