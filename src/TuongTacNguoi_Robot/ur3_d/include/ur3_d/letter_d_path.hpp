#pragma once

#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace ur3_d
{
using Stroke = std::vector<geometry_msgs::msg::Pose>;
using Strokes = std::vector<Stroke>;

enum class DrawingPlane
{
  XY,
  XZ,
  YZ
};

DrawingPlane drawingPlaneFromString(const std::string & value);

std::string drawingPlaneToString(DrawingPlane plane);

double planeHeightCoordinate(
  const geometry_msgs::msg::Point & point,
  DrawingPlane plane);

double planeWidthCoordinate(
  const geometry_msgs::msg::Point & point,
  DrawingPlane plane);

double planeNormalCoordinate(
  const geometry_msgs::msg::Point & point,
  DrawingPlane plane);

geometry_msgs::msg::Point makePointInPlane(
  DrawingPlane plane,
  double height,
  double width,
  double normal);

Strokes createLetterDStrokes(
  const geometry_msgs::msg::Pose & reference_pose,
  double letter_height,
  double letter_width,
  DrawingPlane plane = DrawingPlane::XY);

std::vector<geometry_msgs::msg::Pose> createMotionWaypoints(
  const Strokes & strokes,
  double lift_height,
  DrawingPlane plane = DrawingPlane::XY);

visualization_msgs::msg::Marker createPathMarker(
  const Strokes & strokes,
  const std::string & frame_id,
  double red = 0.05,
  double green = 0.75,
  double blue = 1.0,
  double alpha = 1.0);
}  // namespace ur3_d
