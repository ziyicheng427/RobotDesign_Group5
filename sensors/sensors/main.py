import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan 
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

class MainNode(Node):
    """A ROS2 Node that publishes an amazing quote."""

    def __init__(self):
        super().__init__('Main_Node')


        self.lidarfilter_subscriber = self.create_subscription(
            msg_type=LaserScan,
            topic='/lidarfilter',
            callback=self.lidarfilter_subscriber_callback,
            qos_profile=1)
            
        self.camerafilter_subscriber = self.create_subscription(
            msg_type=Image,
            topic='/camerafilter',
            callback=self.camerafilter_subscriber_callback,
            qos_profile=1)
          
        self.depthfilter_subscriber = self.create_subscription(
            msg_type=PointCloud2,
            topic='/depthfilter',
            callback=self.depthfilter_subscriber_callback,
            qos_profile=1)
            
        self.imu_subscriber = self.create_subscription(
            msg_type=Imu,
            topic='/imu',
            callback=self.imu_subscriber_callback,
            qos_profile=1)
            
        # Add a publisher for the /cmd_vel topic
        self.cmd_vel_publisher = self.create_publisher(
            msg_type=Twist,
            topic='/cmd_vel',
            qos_profile=1)
      
        # Add a publisher for the /cmd_vel topic
        self.arm_publisher = self.create_publisher(
            msg_type=JointState,
            topic='/px150/joint_states',
            qos_profile=1)
            
        self.timer = self.create_timer(1.0, self.publish_velocity)

	
    def publish_velocity(self):
        twist_msg = Twist()
        
        twist_msg.linear.x = 0.1
        twist_msg.angular.z = 0.2
        self.cmd_vel_publisher.publish(twist_msg)
        self.get_logger().info('Publishing Twist data')
        
        joint_state_msg = JointState()
        
        joint_state_msg.position = [0.0, 0.0, 0.0]

        self.arm_publisher.publish(joint_state_msg)
        self.get_logger().info('Publishing Arm data')


    def lidarfilter_subscriber_callback(self, msg: LaserScan):

        """
        Callback function for processing Image messages received from the /lidarfilter topic.
        """
      
        self.point = msg
        

    def camerafilter_subscriber_callback(self, msg: Image):

        """
        Callback function for processing Image messages received from the /camerafilter topic.
        """
      
        self.image = msg
        
    def depthfilter_subscriber_callback(self, msg: PointCloud2):

        """
        Callback function for processing Image messages received from the /depthfilter topic.
        """
      
        self.point = msg
        
    def imu_subscriber_callback(self, msg: Imu):

        """
        Callback function for processing Image messages received from the /depthfilter topic.
        """
      
        self.dir = msg
  


def main(args=None):
    """
    The main function.
    :param args: Not used directly by the user, but used by ROS2 to configure
    certain aspects of the Node.
    """
    try:
        rclpy.init(args=args)

        main_node = MainNode()

        rclpy.spin(main_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()

