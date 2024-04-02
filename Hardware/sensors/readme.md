# 'sensors' introduction

'sensors' is a ROS2 package. It is used in the early design phase of development and contains five custom nodes. four of these nodes create subscribers and publishers of data from the involved sensors (camera, depth camera, imu and rplidar). The main node subscribes to multiple sensor data sources and publishes control commands for the robot. In summary, this package integrates sensors at an early stage and allows sensors to communicate with the robot.
