from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import SetParameter, Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    ld = LaunchDescription()
    

    node_lidar = Node(
            package='sensors',  
            executable='lidar',       
            name='lidar',
            output='screen',
        ),
    node_camera = Node(
            package='sensors', 
            executable='camera',    
            name='camera',
            output='screen',
        ),
    node_depth = Node(
            package='sensors', 
            executable='depth',       
            name='depth',
            output='screen',
        ),
    node_imu = Node(
            package='sensors',  
            executable='imu',       
            name='imu',
            output='screen',
        ),
    node_main = Node(
            package='sensors',  
            executable='main',       
            name='main',
            output='screen',
        )
 
    # Add actions to LaunchDescription
    ld.add_action(SetParameter(name='use_sim_time', value=True))
    ld.add_action(node_lidar)
    ld.add_action(node_camera)
    ld.add_action(node_depth)
    ld.add_action(node_imu)
    ld.add_action(node_main)

    return ld
   
        
