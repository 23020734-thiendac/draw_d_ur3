#include <chrono>
#include <exception>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "ur3_d/end_effector_tracer.hpp"
#include "ur3_d/letter_d_path.hpp"
#include "ur3_d/moveit_executor.hpp"

namespace
{
bool hasNonEmptyStringParameter(
  const rclcpp::Node::SharedPtr & node, const std::string & parameter_name)
{
  if (!node->has_parameter(parameter_name)) {
    return false;
  }

  std::string value;
  return node->get_parameter(parameter_name, value) && !value.empty();
}

bool declareOrSetStringParameter(
  const rclcpp::Node::SharedPtr & node,
  const std::string & parameter_name,
  const std::string & value)
{
  if (node->has_parameter(parameter_name)) {
    return node->set_parameter(rclcpp::Parameter(parameter_name, value)).successful;
  }
  node->declare_parameter<std::string>(parameter_name, value);
  return true;
}

bool copyStringParameterFromRunningNode(
  const rclcpp::Node::SharedPtr & node,
  const std::string & parameter_name,
  const std::vector<std::string> & source_nodes,
  const rclcpp::Logger & logger)
{
  if (hasNonEmptyStringParameter(node, parameter_name)) {
    return true;
  }

  for (const auto & source_node : source_nodes) {
    auto client = std::make_shared<rclcpp::SyncParametersClient>(node, source_node);
    if (!client->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_WARN(
        logger, "Parameter service for %s is not available while loading %s.",
        source_node.c_str(), parameter_name.c_str());
      continue;
    }

    try {
      const auto parameters = client->get_parameters({parameter_name});
      if (
        !parameters.empty() &&
        parameters.front().get_type() == rclcpp::ParameterType::PARAMETER_STRING &&
        !parameters.front().as_string().empty())
      {
        if (!declareOrSetStringParameter(node, parameter_name, parameters.front().as_string())) {
          RCLCPP_ERROR(logger, "Could not set local parameter %s.", parameter_name.c_str());
          return false;
        }
        RCLCPP_INFO(
          logger, "Loaded %s from %s.", parameter_name.c_str(), source_node.c_str());
        return true;
      }
    } catch (const std::exception & exception) {
      RCLCPP_WARN(
        logger, "Could not read %s from %s: %s",
        parameter_name.c_str(), source_node.c_str(), exception.what());
    }
  }

  RCLCPP_ERROR(
    logger,
    "Could not load required MoveIt parameter %s. Start the simulation first with "
    "'ros2 launch ur3_d ur3_d.launch.py', wait until RViz is OK, then run this node "
    "from a second terminal in the same ROS_DOMAIN_ID.",
    parameter_name.c_str());
  return false;
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  const std::string writer_config =
    ament_index_cpp::get_package_share_directory("ur3_d") + "/config/writer.yaml";
  auto node_options = rclcpp::NodeOptions();
  std::vector<std::string> node_arguments = {"--ros-args", "--params-file", writer_config};
  node_arguments.insert(node_arguments.end(), argv + 1, argv + argc);
  node_options.use_global_arguments(false);
  node_options.arguments(node_arguments);
  node_options.append_parameter_override("use_sim_time", true);
  node_options.append_parameter_override(
    "robot_description_kinematics.ur_manipulator.kinematics_solver",
    "kdl_kinematics_plugin/KDLKinematicsPlugin");
  node_options.append_parameter_override(
    "robot_description_kinematics.ur_manipulator.kinematics_solver_search_resolution", 0.005);
  node_options.append_parameter_override(
    "robot_description_kinematics.ur_manipulator.kinematics_solver_timeout", 0.005);
  node_options.append_parameter_override(
    "robot_description_kinematics.ur_manipulator.kinematics_solver_attempts", 3);

  const auto node = rclcpp::Node::make_shared("write_letter_d_node", node_options);
  const auto logger = node->get_logger();

  if (!copyStringParameterFromRunningNode(
      node, "robot_description", {"/robot_state_publisher", "/move_group"}, logger) ||
    !copyStringParameterFromRunningNode(
      node, "robot_description_semantic", {"/move_group"}, logger))
  {
    rclcpp::shutdown();
    return 1;
  }

  const std::string planning_group =
    node->declare_parameter<std::string>("planning_group", "ur_manipulator");
  const double letter_height = node->declare_parameter<double>("letter_height", 0.13);
  const double letter_width = node->declare_parameter<double>("letter_width", 0.08);
  const double lift_height = node->declare_parameter<double>("lift_height", 0.06);
  const double eef_step = node->declare_parameter<double>("eef_step", 0.005);
  const double min_fraction = node->declare_parameter<double>("min_fraction", 0.999);
  const double trace_z_tolerance =
    node->declare_parameter<double>("trace_z_tolerance", 0.004);
  const double trace_plane_tolerance =
    node->declare_parameter<double>("trace_plane_tolerance", trace_z_tolerance);
  const double trace_max_path_deviation =
    node->declare_parameter<double>("trace_max_path_deviation", 0.015);
  const double trace_min_point_distance =
    node->declare_parameter<double>("trace_min_point_distance", 0.001);
  const bool avoid_collisions = node->declare_parameter<bool>("avoid_collisions", true);
  const bool execute_motion = node->declare_parameter<bool>("execute_motion", true);
  const double marker_latch_seconds =
    node->declare_parameter<double>("marker_latch_seconds", 60.0);
  const bool move_to_named_start =
    node->declare_parameter<bool>("move_to_named_start", true);
  const std::string named_start =
    node->declare_parameter<std::string>("named_start", "test_configuration");
  const std::string drawing_plane_name =
    node->declare_parameter<std::string>("drawing_plane", "xy");
  const double planned_marker_r = node->declare_parameter<double>("planned_marker_r", 0.05);
  const double planned_marker_g = node->declare_parameter<double>("planned_marker_g", 0.75);
  const double planned_marker_b = node->declare_parameter<double>("planned_marker_b", 1.0);
  const double planned_marker_a = node->declare_parameter<double>("planned_marker_a", 1.0);
  const double trace_marker_r = node->declare_parameter<double>("trace_marker_r", 0.0);
  const double trace_marker_g = node->declare_parameter<double>("trace_marker_g", 0.2);
  const double trace_marker_b = node->declare_parameter<double>("trace_marker_b", 0.9);
  const double trace_marker_a = node->declare_parameter<double>("trace_marker_a", 1.0);

  ur3_d::DrawingPlane drawing_plane;
  try {
    drawing_plane = ur3_d::drawingPlaneFromString(drawing_plane_name);
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(logger, "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});

  moveit::planning_interface::MoveGroupInterface move_group(node, planning_group);
  move_group.setPlanningTime(10.0);
  move_group.setNumPlanningAttempts(10);
  move_group.setMaxVelocityScalingFactor(0.1);
  move_group.setMaxAccelerationScalingFactor(0.1);
  move_group.setPoseReferenceFrame("world");

  RCLCPP_INFO(logger, "Planning frame: %s", move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(logger, "End-effector link: %s", move_group.getEndEffectorLink().c_str());

  if (!move_group.getCurrentState(10.0)) {
    RCLCPP_ERROR(logger, "Could not get current robot state.");
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  if (move_to_named_start) {
    RCLCPP_INFO(logger, "Moving to named start target '%s'.", named_start.c_str());
    if (!ur3_d::moveToNamedTarget(move_group, named_start, logger, execute_motion)) {
      executor.cancel();
      spinner.join();
      rclcpp::shutdown();
      return 1;
    }
    rclcpp::sleep_for(std::chrono::milliseconds(500));
  }

  const auto current_pose = move_group.getCurrentPose().pose;
  RCLCPP_INFO(
    logger, "Current TCP pose: x=%.3f y=%.3f z=%.3f",
    current_pose.position.x, current_pose.position.y, current_pose.position.z);
  RCLCPP_INFO(
    logger, "Drawing plane: %s", ur3_d::drawingPlaneToString(drawing_plane).c_str());

  const auto strokes = ur3_d::createLetterDStrokes(
    current_pose, letter_height, letter_width, drawing_plane);
  const auto motion_waypoints = ur3_d::createMotionWaypoints(
    strokes, lift_height, drawing_plane);
  const auto marker = ur3_d::createPathMarker(
    strokes, "world",
    planned_marker_r, planned_marker_g, planned_marker_b, planned_marker_a);

  auto marker_pub = node->create_publisher<visualization_msgs::msg::Marker>(
    "letter_path_marker", rclcpp::QoS(1).transient_local().reliable());
  auto marker_array_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "letter_path_markers", rclcpp::QoS(1).transient_local().reliable());
  visualization_msgs::msg::MarkerArray marker_array;
  marker_array.markers.push_back(marker);
  marker_pub->publish(marker);
  marker_array_pub->publish(marker_array);
  auto marker_timer = node->create_wall_timer(
    std::chrono::milliseconds(500),
    [marker_pub, marker_array_pub, marker, marker_array]() {
      marker_pub->publish(marker);
      marker_array_pub->publish(marker_array);
    });

  ur3_d::EndEffectorTracer actual_path_tracer(
    node, marker_array_pub, "world", move_group.getEndEffectorLink(),
    strokes, drawing_plane, ur3_d::planeNormalCoordinate(current_pose.position, drawing_plane),
    trace_plane_tolerance, trace_max_path_deviation, trace_min_point_distance,
    trace_marker_r, trace_marker_g, trace_marker_b, trace_marker_a);

  RCLCPP_INFO(logger, "Writing letter D with crossbar using %zu strokes.", strokes.size());
  if (execute_motion) {
    actual_path_tracer.start();
  }
  const bool path_succeeded = ur3_d::executeCartesianPath(
    move_group, motion_waypoints, logger, eef_step, min_fraction, avoid_collisions,
    execute_motion);
  actual_path_tracer.stop();
  if (path_succeeded) {
    RCLCPP_INFO(logger, "Complete Cartesian letter path executed successfully.");
  } else {
    RCLCPP_ERROR(logger, "Could not execute the complete Cartesian letter path.");
  }

  if (marker_latch_seconds > 0.0) {
    RCLCPP_INFO(logger, "Keeping letter marker alive for %.1f seconds.", marker_latch_seconds);
    rclcpp::sleep_for(
      std::chrono::milliseconds(static_cast<int>(marker_latch_seconds * 1000.0)));
  }

  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return path_succeeded ? 0 : 1;
}
