#include "ur3_d/moveit_executor.hpp"

#include <moveit_msgs/msg/robot_trajectory.hpp>

namespace ur3_d
{
bool moveToNamedTarget(
  moveit::planning_interface::MoveGroupInterface & move_group,
  const std::string & target_name,
  const rclcpp::Logger & logger,
  const bool execute_motion)
{
  move_group.setStartStateToCurrentState();
  move_group.setNamedTarget(target_name);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (!static_cast<bool>(move_group.plan(plan))) {
    RCLCPP_ERROR(logger, "Could not plan to named target '%s'.", target_name.c_str());
    return false;
  }
  if (!execute_motion) {
    RCLCPP_INFO(logger, "Named target plan succeeded; execute_motion is false.");
    return true;
  }

  const bool executed = static_cast<bool>(move_group.execute(plan));
  if (!executed) {
    RCLCPP_ERROR(logger, "Could not execute named target '%s'.", target_name.c_str());
  }
  return executed;
}

bool executeCartesianPath(
  moveit::planning_interface::MoveGroupInterface & move_group,
  const std::vector<geometry_msgs::msg::Pose> & waypoints,
  const rclcpp::Logger & logger,
  const double eef_step,
  const double min_fraction,
  const bool avoid_collisions,
  const bool execute_motion)
{
  move_group.setStartStateToCurrentState();

  moveit_msgs::msg::RobotTrajectory trajectory;
  constexpr double jump_threshold = 0.0;
  const double fraction = move_group.computeCartesianPath(
    waypoints, eef_step, jump_threshold, trajectory, avoid_collisions);

  RCLCPP_INFO(logger, "Cartesian path fraction: %.2f%%", fraction * 100.0);
  if (fraction < min_fraction) {
    RCLCPP_ERROR(
      logger,
      "Cartesian path fraction is too low. Required %.2f%%, got %.2f%%.",
      min_fraction * 100.0,
      fraction * 100.0);
    return false;
  }
  if (!execute_motion) {
    RCLCPP_INFO(logger, "Cartesian plan succeeded; execute_motion is false.");
    return true;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory_ = trajectory;
  const bool executed = static_cast<bool>(move_group.execute(plan));
  if (!executed) {
    RCLCPP_ERROR(logger, "Could not execute Cartesian path.");
  }
  return executed;
}
}  // namespace ur3_d
