# Manchester Uni Robotic Systems Design Project Team5 (AERO62520)

## Introduction

Welcome to the GitHub repository of the Manchester Robotics Team5! We are a group of four passionate Robotics postgraduate students from the University of Manchester: Alexander Morley, Michalis Iacovides, Geetik Mamillapalli, and Ziyi Cheng.

This repository serves as our collaborative workspace for developing an innovative robotics project. Our primary objective is to create a robot capable of autonomously picking up objects from the environment.

## Project Overview

The project aims to design and implement a robotic system that combines the mobility of the LeoRover with the functionality of a mechanical arm. This system will be equipped with sensors and AI algorithms to detect, approach, and manipulate objects in various settings.

### Key Features(To be completed)

- **Autonomous Navigation:** 
- **Object Detection:**

## Getting Started(To be completed)

To get started with our project, please follow the instructions below:

1. **Clone the Repository:** Use `git clone [repository link]` to clone this repository to your local machine.
2. **Installation:** 
3. **Running the Project:**

## Documentation

**NODES/TOPICS:**

1. **Rplidar_node:** The RPLIDAR A2M12 utilises the RPLIDAR ROS Package for reading the scanned data, with the ‘rplidar_a2m12_launch.py’ launch file. The launch file then launches the ‘rplidar_node’ node that publishes the data through the '/scan' topic  in the form of 'sensor_msgs/LaserScan' messages. Since the lidar is placed underneath the robot  the minimum detection distance was changed to 0.3 meters(range_min value from rplidar_node.cpp file). This prevents the lidar from detecting the robot's wheels, ensuring the avoidance of inaccurate data.
2. **object_detect:** designed to interface with a Realsense Camera D435. It subscribes to two topics published by the camera: "/camera/camera/color/image_raw" and "/camera/camera/depth/image_rect_raw," both of which publish messages of the type Image. These messages contain the color and depth information captured by the camera, respectively. The "object_detect" node processes this information to determine the position and orientation of objects within the camera's field of view. The processed data is then published to the "object_pose_to_camera" topic, which emits messages of the type Point. This information is intended for use by robotic arms or other devices that require spatial information about objects in their environment.
3. **EKF_Node:** designed to read the Leo core board’s built-in IMU data which include angular velocity and linear acceleration(‘/imu_raw’ topic) and the covariance of wheel odomentry data(‘wheel_odom_with_covariance’ topic). It then implements an extended kalman filter and publishes the new odometry( ‘/odom’ topic) in the form of ‘nav_msgs/Odometry’ message, which is used for the Navigation. 
4. **Main_Node:** designed to process various sensor and navigation message types, including ‘OccupancyGrid‘, 'Odometry', ‘Path', ’LaserScan’,’Image’,’PointCloud2’ and ‘IMU’ messages. The node receives messages from topics such as '/map', '/local_costmap', '/global_costmap' and '/odom'. These data, in conjunction with the SLAM-Toolbox and Nav2 Packages, will be utilised for performing localisation and mapping of the environment. Moreover, the node features two publishers for sending messages. The first publisher controls the robot’s movement by publishing '/geometry_msgs/msg/PoseStamped' messages to the '/goal_pose' topic. The second publisher manages the robot arm’s movement by publishing '/sensor_msgs/JointState' messages to the '/px150/joint_states' topic..

**PACKAGES:**

1. **RPLIDAR ROS** - <https://github.com/Slamtec/rplidar_ros>
2. **REALSENSE2** - <https://github.com/IntelRealSense/realsense-ros>
3. **interbotix_ros_manipulators** - <https://github.com/Interbotix/interbotix_ros_manipulators>
4. **SLAM-Toolbox** - <https://github.com/SteveMacenski/slam_toolbox>
5. **Nav2** - <https://index.ros.org/p/nav2_map_server>
6. **Wavefront Exploration** - <https://github.com/gjcliff/SLAM-Frontier-Exploration>

## Team Members

- **Alexander Morley** - <https://shkibby.github.io/AlexanderMorley.github.io/>
- **Michalis Iacovides** - <https://michalisiakovides.github.io/>
- **Geetik Mamillapalli** - <https://mgeetik.github.io/>
- **Ziyi Cheng** - <https://ziyicheng427.github.io>

## Contact

For any queries or collaborations, feel free to contact us at emails below.

<alexander.morley@student.manchester.ac.uk>

<michalis.iakovides@student.manchester.ac.uk>

<geetik.mamillapalli@postgrad.manchester.ac.uk>

<ziyi.cheng@student.manchester.ac.uk>

## Acknowledgments

Special thanks to the University of Manchester and the Robotics Department for their support and resources in making this project possible.

## Videos
