#pragma once

#include <atomic>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "ur3_d/letter_d_path.hpp"

namespace ur3_d
{
class EndEffectorTracer
{
public:
  using MarkerArrayPublisher = rclcpp::Publisher<visualization_msgs::msg::MarkerArray>;

  EndEffectorTracer(
    const rclcpp::Node::SharedPtr & node,
    const MarkerArrayPublisher::SharedPtr & publisher,
    const std::string & frame_id,
    const std::string & end_effector_link,
    const Strokes & planned_strokes,
    DrawingPlane drawing_plane,
    double draw_normal,
    double normal_tolerance,
    double max_path_deviation,
    double min_point_distance,
    double red = 0.0,
    double green = 0.2,
    double blue = 0.9,
    double alpha = 1.0);

  void start();
  void stop();

private:
  void sample();
  void publishMarker();
  bool projectToPlannedPath(
    const geometry_msgs::msg::Point & measured_point,
    geometry_msgs::msg::Point & projected_point,
    std::size_t & stroke_index) const;

  struct PlannedSegment
  {
    geometry_msgs::msg::Point start;
    geometry_msgs::msg::Point end;
    std::size_t stroke_index;
  };

  rclcpp::Node::SharedPtr node_;
  MarkerArrayPublisher::SharedPtr publisher_;
  std::string frame_id_;
  std::string end_effector_link_;
  DrawingPlane drawing_plane_;
  double draw_normal_;
  double normal_tolerance_;
  double max_path_deviation_;
  double min_point_distance_;
  std::vector<PlannedSegment> planned_segments_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  visualization_msgs::msg::Marker marker_;
  geometry_msgs::msg::Point previous_point_;
  std::size_t previous_stroke_index_{0};
  bool previous_point_valid_{false};
  std::atomic_bool recording_{false};
  std::atomic_bool reset_requested_{false};
};
}  // namespace ur3_d
