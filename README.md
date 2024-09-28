##Description
This repository contains webots simulation for omni-wheel drive robot, focussing on RoboCup MSL competition. The motion of omni-wheel robot is controlled by ros2.

The module publishes:
- /gps_data: global x, y and z coordinates of the robot
- /imu_data: global x, y and z angular orientations of the robot
- /encoder_data: encoder values of each of the 4 motors of the robot

The module subscribes to:
- /cmd_vel: linear.x, linear.y and angular.z are used to control the motion of robot

##Dependencies:
1. Ubuntu 24.04
2. ROS 2 Jazzy Jalisco
3. Webots
4. Webots-Ros2 package

##Run the code
'''bash
colcon build
source install/local_setup.bash
ros2 launch my_package robot_launch.py
'''
