#include "my_package/MyRobotDriver.hpp"

#include "rclcpp/rclcpp.hpp"
#include <cstdio>
#include <functional>
#include <webots/motor.h>
#include <webots/robot.h>

#define DISTANCE_OF_WHEELS_FROM_CENTER 0.045 // Diagonal/2
#define WHEEL_RADIUS 0.025 // Adjust to your robot's wheel radius
#define ROOT_2 1.41421356237 // Pre-calculated value of sqrt(2)

namespace my_robot_driver {

void MyRobotDriver::init(
    webots_ros2_driver::WebotsNode *node,
    std::unordered_map<std::string, std::string> &parameters) {

  // Initialize all four motors
  wheel_1 = wb_robot_get_device("wheel1");
  wheel_2 = wb_robot_get_device("wheel2");
  wheel_3 = wb_robot_get_device("wheel3");
  wheel_4 = wb_robot_get_device("wheel4");

  // Set the motors to velocity control mode
  wb_motor_set_position(wheel_1, INFINITY);
  wb_motor_set_velocity(wheel_1, 0.0);
  
  wb_motor_set_position(wheel_2, INFINITY);
  wb_motor_set_velocity(wheel_2, 0.0);
  
  wb_motor_set_position(wheel_3, INFINITY);
  wb_motor_set_velocity(wheel_3, 0.0);
  
  wb_motor_set_position(wheel_4, INFINITY);
  wb_motor_set_velocity(wheel_4, 0.0);

  // Subscribe to cmd_vel to receive velocity commands
  cmd_vel_subscription_ = node->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::SensorDataQoS().reliable(),
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        this->cmd_vel_msg.linear = msg->linear;
        this->cmd_vel_msg.angular = msg->angular;
      }
  );
}

void MyRobotDriver::step() {
  // Get the desired forward, sideways, and rotational velocities
  auto vx = cmd_vel_msg.linear.x;  // Linear velocity in x-direction (forward)
  auto vy = cmd_vel_msg.linear.y;  // Linear velocity in y-direction (sideways)
  auto omega = cmd_vel_msg.angular.z; // Angular velocity around z-axis (rotation)

  // Compute individual wheel velocities based on omni-wheel kinematics with 45-degree orientation
  auto v1 = (1 / WHEEL_RADIUS) * ((vx - vy) / ROOT_2 + DISTANCE_OF_WHEELS_FROM_CENTER * omega);
  auto v2 = (1 / WHEEL_RADIUS) * ((vx + vy) / ROOT_2 + DISTANCE_OF_WHEELS_FROM_CENTER * omega);
  auto v3 = (1 / WHEEL_RADIUS) * ((-vx + vy) / ROOT_2 + DISTANCE_OF_WHEELS_FROM_CENTER * omega);
  auto v4 = (1 / WHEEL_RADIUS) * ((-vx - vy) / ROOT_2 + DISTANCE_OF_WHEELS_FROM_CENTER * omega);

  // Set the velocity for each motor
  wb_motor_set_velocity(wheel_1, v1);
  wb_motor_set_velocity(wheel_2, v2);
  wb_motor_set_velocity(wheel_3, v3);
  wb_motor_set_velocity(wheel_4, v4);
}
} // namespace my_robot_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(my_robot_driver::MyRobotDriver, webots_ros2_driver::PluginInterface)

