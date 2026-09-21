#include "ur3_d/end_effector_tracer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

#include <tf2/exceptions.h>
#include <tf2/time.h>

namespace ur3_d
{
EndEffectorTracer::EndEffectorTracer(
  const rclcpp::Node::SharedPtr & node,
  const MarkerArrayPublisher::SharedPtr & publisher,
  const std::string & frame_id,
  const std::string & end_effector_link,
  const Strokes & planned_strokes,
  const DrawingPlane drawing_plane,
  const double draw_normal,
  const double normal_tolerance,
  const double max_path_deviation,
  const double min_point_distance,
  const double red,
  const double green,
  const double blue,
  const double alpha)
: node_(node),
  publisher_(publisher),
  frame_id_(frame_id),
  end_effector_link_(end_effector_link),
  drawing_plane_(drawing_plane),
  draw_normal_(draw_normal),
  normal_tolerance_(normal_tolerance),
  max_path_deviation_(max_path_deviation),
  min_point_distance_(min_point_distance),
  tf_buffer_(node_->get_clock()),
  tf_listener_(tf_buffer_, node_, false)
{
  marker_.header.frame_id = frame_id_;
  marker_.ns = "letter_d_actual_path";
  marker_.id = 0;
  marker_.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker_.action = visualization_msgs::msg::Marker::ADD;
  marker_.pose.position = makePointInPlane(drawing_plane_, 0.0, 0.0, 0.001);
  marker_.pose.orientation.w = 1.0;
  marker_.scale.x = 0.006;
  marker_.color.r = red;
  marker_.color.g = green;
  marker_.color.b = blue;
  marker_.color.a = alpha;

  for (std::size_t stroke_index = 0; stroke_index < planned_strokes.size(); ++stroke_index) {
    const auto & stroke = planned_strokes[stroke_index];
    for (std::size_t i = 1; i < stroke.size(); ++i) {
      planned_segments_.push_back(
        {stroke[i - 1].position, stroke[i].position, stroke_index});
    }
  }

  timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(40),
    [this]() {sample();});
}

void EndEffectorTracer::start()
{
  reset_requested_.store(true);
  recording_.store(true);
}

void EndEffectorTracer::stop()
{
  recording_.store(false);
}

void EndEffectorTracer::sample()
{
  if (reset_requested_.exchange(false)) {
    marker_.points.clear();
    previous_point_valid_ = false;
    publishMarker();
  }
  if (!recording_.load()) {
    return;
  }

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_.lookupTransform(
      frame_id_, end_effector_link_, tf2::TimePointZero);
  } catch (const tf2::TransformException &) {
    return;
  }

  geometry_msgs::msg::Point current_point;
  current_point.x = transform.transform.translation.x;
  current_point.y = transform.transform.translation.y;
  current_point.z = transform.transform.translation.z;

  if (std::abs(planeNormalCoordinate(current_point, drawing_plane_) - draw_normal_) >
    normal_tolerance_)
  {
    previous_point_valid_ = false;
    return;
  }

  geometry_msgs::msg::Point projected_point;
  std::size_t stroke_index = 0;
  if (!projectToPlannedPath(current_point, projected_point, stroke_index)) {
    previous_point_valid_ = false;
    return;
  }
  if (!previous_point_valid_) {
    previous_point_ = projected_point;
    previous_stroke_index_ = stroke_index;
    previous_point_valid_ = true;
    return;
  }

  if (stroke_index != previous_stroke_index_) {
    previous_point_ = projected_point;
    previous_stroke_index_ = stroke_index;
    return;
  }

  const double dx = projected_point.x - previous_point_.x;
  const double dy = projected_point.y - previous_point_.y;
  const double dz = projected_point.z - previous_point_.z;
  const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
  if (distance < min_point_distance_) {
    return;
  }

  marker_.points.push_back(previous_point_);
  marker_.points.push_back(projected_point);
  previous_point_ = projected_point;
  publishMarker();
}

bool EndEffectorTracer::projectToPlannedPath(
  const geometry_msgs::msg::Point & measured_point,
  geometry_msgs::msg::Point & projected_point,
  std::size_t & stroke_index) const
{
  auto find_closest = [this, &measured_point](
    const bool restrict_stroke,
    const std::size_t required_stroke,
    geometry_msgs::msg::Point & best_point,
    std::size_t & best_stroke)
    {
      double best_distance_squared = std::numeric_limits<double>::max();
      for (const auto & segment : planned_segments_) {
        if (restrict_stroke && segment.stroke_index != required_stroke) {
          continue;
        }

        const double segment_height =
          planeHeightCoordinate(segment.end, drawing_plane_) -
          planeHeightCoordinate(segment.start, drawing_plane_);
        const double segment_width =
          planeWidthCoordinate(segment.end, drawing_plane_) -
          planeWidthCoordinate(segment.start, drawing_plane_);
        const double length_squared =
          segment_height * segment_height + segment_width * segment_width;
        if (length_squared <= 0.0) {
          continue;
        }

        const double offset_height =
          planeHeightCoordinate(measured_point, drawing_plane_) -
          planeHeightCoordinate(segment.start, drawing_plane_);
        const double offset_width =
          planeWidthCoordinate(measured_point, drawing_plane_) -
          planeWidthCoordinate(segment.start, drawing_plane_);
        const double raw_projection =
          (offset_height * segment_height + offset_width * segment_width) / length_squared;
        const double projection = std::max(0.0, std::min(raw_projection, 1.0));
        const double candidate_height =
          planeHeightCoordinate(segment.start, drawing_plane_) + projection * segment_height;
        const double candidate_width =
          planeWidthCoordinate(segment.start, drawing_plane_) + projection * segment_width;
        const auto candidate = makePointInPlane(
          drawing_plane_, candidate_height, candidate_width, draw_normal_);

        const double height_error =
          planeHeightCoordinate(measured_point, drawing_plane_) - candidate_height;
        const double width_error =
          planeWidthCoordinate(measured_point, drawing_plane_) - candidate_width;
        const double distance_squared =
          height_error * height_error + width_error * width_error;
        if (distance_squared < best_distance_squared) {
          best_distance_squared = distance_squared;
          best_point = candidate;
          best_stroke = segment.stroke_index;
        }
      }
      return best_distance_squared;
    };

  const double maximum_distance_squared = max_path_deviation_ * max_path_deviation_;
  if (previous_point_valid_) {
    const double same_stroke_distance = find_closest(
      true, previous_stroke_index_, projected_point, stroke_index);
    if (same_stroke_distance <= maximum_distance_squared) {
      return true;
    }
  }

  const double closest_distance = find_closest(false, 0, projected_point, stroke_index);
  return closest_distance <= maximum_distance_squared;
}

void EndEffectorTracer::publishMarker()
{
  visualization_msgs::msg::MarkerArray marker_array;
  marker_array.markers.push_back(marker_);
  publisher_->publish(marker_array);
}
}  // namespace ur3_d
