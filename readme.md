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

1. **LiDAR_Filter_Node:** designed to filter and republish LIDAR data coming from the
RPLIDAR A2M12, which leverages the RPLIDAR ROS Package. It subscribes to the '/scan' topic, which
receives LIDAR data in the form of 'sensor_msgs/LaserScan' messages. The node filters the LaserScan
message and publishes it to the '/lidarfilter' topic.
2. **Camera_Filter_Node:** designed to process image data received from the '/camera/image_raw'
topic and publish the results to the '/camerafilter' topic. It receives 'sensor_msgs/Image' messages, representing image data captured by the camera in the Leo Rover's electronics box. The node then publishes
the processed image data as a 'sensor_msgs/Image' message to the '/camerafilter' topic.
3. **Depth_Filter_Node:** designed to process depth data from the Intel Realsense depth camera.
It subscribes to the '/camera/camera/depth/colour/points' topic, which provides 'sensor_msgs/PointCloud2'
messages containing depth information.The node then publishes the processed depth data as a 'sensor_msgs/PointCloud2'
message, to the '/depthfilter' topic.
4. **Imu_Node:** designed to read and publish Inertial Measurement Unit (IMU) data from
the Sparkfun IMU. This node publishes the IMU data, including orientation, angular velocity, and linear
acceleration, in the form of a 'sensor_msgs/Imu' message to the '/imu' topic.
5. **Main_Node:** designed to process various sensor message types, including 'LaserScan',
'Image', 'PointCloud2', and 'Imu' messages. The node receives messages from topics such as '/lidarfilter',
'/camerafilter', '/depthfilter', and '/imu'. These data, in conjunction with the SLAM-Toolbox and Nav2
Packages, will be utilised for performing localisation and mapping of the environment.
Moreover, the node features two publishers for sending messages. The first publisher controls the
robot's movement by publishing '/geometry_msgs/Twist' messages to the '/cmd_vel' topic. The second
publisher manages the robot arm's movement by publishing '/sensor_msgs/JointState' messages to the
'/px150/joint_states' topic.

**PACKAGES:**

1. **RPLIDAR ROS** - <https://github.com/Slamtec/rplidar_ros>
2. **REALSENSE2** - <https://github.com/IntelRealSense/realsense-ros>
3. **interbotix_ros_manipulators** - <https://github.com/Interbotix/interbotix_ros_manipulators>
4. **SLAM-Toolbox** - <https://github.com/SteveMacenski/slam_toolbox>
5. **Nav2** - <https://index.ros.org/p/nav2_map_server>

## Team Members

- **Alexander Morley** - <https://shkibby.github.io/AlexanderMorley.github.io/>
- **Michalis Iacovides** - <https://michalisiakovides.github.io/>
- **Geetik Mamillapalli** - 
- **Ziyi Cheng** - <https://ziyicheng427.github.io>

## Contact

For any queries or collaborations, feel free to contact us at emails below.

<alexander.morley@student.manchester.ac.uk>

<michalis.iakovides@student.manchester.ac.uk>

<geetik.mamillapalli@postgrad.manchester.ac.uk>

<ziyi.cheng@student.manchester.ac.uk>

## Acknowledgments

Special thanks to the University of Manchester and the Robotics Department for their support and resources in making this project possible.
