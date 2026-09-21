#include "ur3_d/letter_d_path.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace ur3_d
{
namespace
{
geometry_msgs::msg::Point toPoint(const geometry_msgs::msg::Pose & pose)
{
  geometry_msgs::msg::Point point;
  point.x = pose.position.x;
  point.y = pose.position.y;
  point.z = pose.position.z;
  return point;
}

std::string normalizedPlaneName(std::string value)
{
  value.erase(
    std::remove_if(value.begin(), value.end(), [](const unsigned char character) {
      return character == '_' || character == '-' || std::isspace(character);
    }),
    value.end());
  std::transform(value.begin(), value.end(), value.begin(), [](const unsigned char character) {
    return static_cast<char>(std::tolower(character));
  });
  return value;
}

geometry_msgs::msg::Pose makePlanePose(
  const geometry_msgs::msg::Pose & reference,
  const DrawingPlane plane,
  const double height,
  const double width,
  const double normal)
{
  auto pose = reference;
  const auto point = makePointInPlane(plane, height, width, normal);
  pose.position = point;
  return pose;
}

void offsetAlongPlaneNormal(
  geometry_msgs::msg::Pose & pose,
  const DrawingPlane plane,
  const double offset)
{
  switch (plane) {
    case DrawingPlane::XY:
      pose.position.z += offset;
      break;
    case DrawingPlane::XZ:
      pose.position.y += offset;
      break;
    case DrawingPlane::YZ:
      pose.position.x += offset;
      break;
  }
}
}  // namespace

DrawingPlane drawingPlaneFromString(const std::string & value)
{
  const auto normalized = normalizedPlaneName(value);
  if (normalized == "xy" || normalized == "horizontal") {
    return DrawingPlane::XY;
  }
  if (normalized == "xz" || normalized == "verticalxz" || normalized == "vertical") {
    return DrawingPlane::XZ;
  }
  if (normalized == "yz" || normalized == "verticalyz") {
    return DrawingPlane::YZ;
  }
  throw std::invalid_argument(
    "drawing_plane must be one of: xy, xz, yz, horizontal, vertical");
}

std::string drawingPlaneToString(const DrawingPlane plane)
{
  switch (plane) {
    case DrawingPlane::XY:
      return "xy";
    case DrawingPlane::XZ:
      return "xz";
    case DrawingPlane::YZ:
      return "yz";
  }
  return "unknown";
}

double planeHeightCoordinate(
  const geometry_msgs::msg::Point & point,
  const DrawingPlane plane)
{
  switch (plane) {
    case DrawingPlane::XY:
      return point.x;
    case DrawingPlane::XZ:
    case DrawingPlane::YZ:
      return point.z;
  }
  return point.x;
}

double planeWidthCoordinate(
  const geometry_msgs::msg::Point & point,
  const DrawingPlane plane)
{
  switch (plane) {
    case DrawingPlane::XY:
      return point.y;
    case DrawingPlane::XZ:
      return point.x;
    case DrawingPlane::YZ:
      return point.y;
  }
  return point.y;
}

double planeNormalCoordinate(
  const geometry_msgs::msg::Point & point,
  const DrawingPlane plane)
{
  switch (plane) {
    case DrawingPlane::XY:
      return point.z;
    case DrawingPlane::XZ:
      return point.y;
    case DrawingPlane::YZ:
      return point.x;
  }
  return point.z;
}

geometry_msgs::msg::Point makePointInPlane(
  const DrawingPlane plane,
  const double height,
  const double width,
  const double normal)
{
  geometry_msgs::msg::Point point;
  switch (plane) {
    case DrawingPlane::XY:
      point.x = height;
      point.y = width;
      point.z = normal;
      break;
    case DrawingPlane::XZ:
      point.x = width;
      point.y = normal;
      point.z = height;
      break;
    case DrawingPlane::YZ:
      point.x = normal;
      point.y = width;
      point.z = height;
      break;
  }
  return point;
}

Strokes createLetterDStrokes(
  const geometry_msgs::msg::Pose & reference_pose,
  const double letter_height,
  const double letter_width,
  const DrawingPlane plane)
{
  const double center_height = planeHeightCoordinate(reference_pose.position, plane);
  const double width_left =
    planeWidthCoordinate(reference_pose.position, plane) - letter_width / 2.0;
  const double height_top = center_height + letter_height / 2.0;
  const double height_bottom = center_height - letter_height / 2.0;
  const double normal_draw = planeNormalCoordinate(reference_pose.position, plane);

  Strokes strokes;
  strokes.push_back(
    {
      makePlanePose(reference_pose, plane, height_top, width_left, normal_draw),
      makePlanePose(reference_pose, plane, height_bottom, width_left, normal_draw)});

  Stroke round_stroke;
  constexpr int arc_points = 18;
  constexpr double pi = 3.14159265358979323846;
  for (int i = 0; i <= arc_points; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(arc_points);
    const double theta = pi / 2.0 - t * pi;
    const double height = center_height + (letter_height / 2.0) * std::sin(theta);
    const double width = width_left + letter_width * std::cos(theta);
    round_stroke.push_back(makePlanePose(reference_pose, plane, height, width, normal_draw));
  }
  strokes.push_back(round_stroke);

  const double crossbar_half_length = letter_width * 0.22;
  strokes.push_back(
    {
      makePlanePose(
        reference_pose, plane, center_height, width_left - crossbar_half_length, normal_draw),
      makePlanePose(
        reference_pose, plane, center_height, width_left + crossbar_half_length, normal_draw)});
  return strokes;
}

std::vector<geometry_msgs::msg::Pose> createMotionWaypoints(
  const Strokes & strokes,
  const double lift_height,
  const DrawingPlane plane)
{
  std::vector<geometry_msgs::msg::Pose> waypoints;
  for (const auto & stroke : strokes) {
    if (stroke.empty()) {
      continue;
    }

    auto above_start = stroke.front();
    offsetAlongPlaneNormal(above_start, plane, lift_height);
    auto above_end = stroke.back();
    offsetAlongPlaneNormal(above_end, plane, lift_height);

    waypoints.push_back(above_start);
    waypoints.push_back(stroke.front());
    waypoints.insert(waypoints.end(), stroke.begin(), stroke.end());
    waypoints.push_back(above_end);
  }
  return waypoints;
}

visualization_msgs::msg::Marker createPathMarker(
  const Strokes & strokes,
  const std::string & frame_id,
  const double red,
  const double green,
  const double blue,
  const double alpha)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.ns = "letter_d_path";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.006;
  marker.color.r = red;
  marker.color.g = green;
  marker.color.b = blue;
  marker.color.a = alpha;

  for (const auto & stroke : strokes) {
    for (std::size_t i = 1; i < stroke.size(); ++i) {
      marker.points.push_back(toPoint(stroke[i - 1]));
      marker.points.push_back(toPoint(stroke[i]));
    }
  }
  return marker;
}
}  // namespace ur3_d
