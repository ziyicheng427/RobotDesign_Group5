from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 创建launch描述对象
    ld = LaunchDescription()

    # 定义ImageSubscriber节点，假设hsv_selector是其正确的入口点名
    hsv_selector_node = Node(
        package='object_detect_opencv',
        executable='hsv_selector',
        name='hsv_selector_node',
        output='screen'
        # parameters=[{'your_parameter_name': 'your_parameter_value'}]  # 如有必要，添加你的参数
    )

    # 定义StartDetection节点
    start_detection_node = Node(
        package='object_detect_opencv',
        executable='start_detection',
        name='start_detection_node',
        output='screen'
        # parameters=[{'your_parameter_name': 'your_parameter_value'}]  # 如有必要，添加你的参数
    )

    # 将节点添加到launch描述中
    ld.add_action(hsv_selector_node)
    ld.add_action(start_detection_node)

    return ld
