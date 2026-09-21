#pragma once

#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/logger.hpp>

namespace ur3_d
{
bool moveToNamedTarget(
  moveit::planning_interface::MoveGroupInterface & move_group,
  const std::string & target_name,
  const rclcpp::Logger & logger,
  bool execute_motion);

bool executeCartesianPath(
  moveit::planning_interface::MoveGroupInterface & move_group,
  const std::vector<geometry_msgs::msg::Pose> & waypoints,
  const rclcpp::Logger & logger,
  double eef_step,
  double min_fraction,
  bool avoid_collisions,
  bool execute_motion);
}  // namespace ur3_d
