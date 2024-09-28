## Description
This repository contains a Webots simulation for an omni-wheel drive robot, focusing on the RoboCup MSL competition. The motion of the omni-wheel robot is controlled by ROS2.

### The module publishes:
- `/gps_data`: Global x, y, and z coordinates of the robot.
- `/imu_data`: Global x, y, and z angular orientations of the robot.
- `/encoder_data`: Encoder values of each of the 4 motors of the robot.

### The module subscribes to:
- `/cmd_vel`: The `linear.x`, `linear.y`, and `angular.z` values are used to control the motion of the robot.

## Dependencies:
1. Ubuntu 22.04
2. ROS 2 Humble Hawksbill
3. Webots
4. Webots-ROS2 package

## Run the code
```bash
colcon build
source install/local_setup.bash
ros2 launch my_package robot_launch.py
```
