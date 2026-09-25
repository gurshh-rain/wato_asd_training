#include "dynamic_tracker_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include "visualization_msgs/msg/marker.hpp"

namespace robot
{

DynamicTrackerCore::DynamicTrackerCore(const rclcpp::Logger& logger,
                                       double cluster_threshold,
                                       int minimum_cluster_size,
                                       double maximum_cluster_radius,
                                       double association_gate,
                                       double dynamic_speed_threshold,
                                       double process_noise,
                                       double measurement_noise,
                                       double prediction_horizon,
                                       double occupancy_horizon,
                                       double prediction_step,
                                       double robot_radius,
                                       double safety_margin,
                                       double motion_min_x,
                                       double motion_max_x)
  : logger_(logger),
    cluster_threshold_(cluster_threshold),
    minimum_cluster_size_(minimum_cluster_size),
    maximum_cluster_radius_(maximum_cluster_radius),
    association_gate_(association_gate),
    dynamic_speed_threshold_(dynamic_speed_threshold),
    process_noise_(process_noise),
    measurement_noise_(measurement_noise),
    prediction_horizon_(prediction_horizon),
    occupancy_horizon_(occupancy_horizon),
    prediction_step_(prediction_step),
    robot_radius_(robot_radius),
    safety_margin_(safety_margin),
    motion_min_x_(motion_min_x),
    motion_max_x_(motion_max_x),
    next_id_(1)
{
}

TrackerOutput DynamicTrackerCore::process(const sensor_msgs::msg::LaserScan& scan,
                                          const nav_msgs::msg::Odometry& odom,
                                          const nav_msgs::msg::OccupancyGrid& map,
                                          const rclcpp::Time& stamp)
{
  auto detections = detect(scan, odom);
  double minimum_x = map.info.origin.position.x + 1.0;
  double minimum_y = map.info.origin.position.y + 1.0;
  double maximum_x = map.info.origin.position.x + map.info.width * map.info.resolution - 1.0;
  double maximum_y = map.info.origin.position.y + map.info.height * map.info.resolution - 1.0;
  detections.erase(std::remove_if(detections.begin(), detections.end(),
                                  [&](const Detection& detection) {
                                    return detection.x < minimum_x || detection.x > maximum_x ||
                                           detection.y < minimum_y || detection.y > maximum_y;
                                  }), detections.end());
  if (!tracks_.empty() && stamp < tracks_.front().stamp) {
    tracks_.clear();
  }
  for (auto& track : tracks_) {
    double dt = std::max(0.0, (stamp - track.stamp).seconds());
    predict(track, dt);
    track.stamp = stamp;
  }
  std::vector<bool> track_used(tracks_.size(), false);
  std::vector<bool> detection_used(detections.size(), false);
  while (true) {
    double best_distance = association_gate_;
    int best_track = -1;
    int best_detection = -1;
    for (std::size_t i = 0; i < tracks_.size(); ++i) {
      if (track_used[i]) {
        continue;
      }
      for (std::size_t j = 0; j < detections.size(); ++j) {
        if (detection_used[j]) {
          continue;
        }
        double distance = std::hypot(tracks_[i].x.position - detections[j].x,
                                     tracks_[i].y.position - detections[j].y);
        if (distance < best_distance) {
          best_distance = distance;
          best_track = static_cast<int>(i);
          best_detection = static_cast<int>(j);
        }
      }
    }
    if (best_track < 0) {
      break;
    }
    auto& track = tracks_[static_cast<std::size_t>(best_track)];
    update(track, detections[static_cast<std::size_t>(best_detection)]);
    track.hits++;
    track.misses = 0;
    double speed = std::hypot(track.x.velocity, track.y.velocity);
    if (speed >= dynamic_speed_threshold_) {
      track.moving_hits++;
      if (track.moving_hits >= 2) {
        track.dynamic_until = stamp + rclcpp::Duration::from_seconds(2.0);
      }
    } else {
      track.moving_hits = std::max(0, track.moving_hits - 1);
    }
    track_used[static_cast<std::size_t>(best_track)] = true;
    detection_used[static_cast<std::size_t>(best_detection)] = true;
  }
  for (std::size_t i = 0; i < tracks_.size(); ++i) {
    if (!track_used[i]) {
      tracks_[i].misses++;
    }
  }
  for (std::size_t i = 0; i < detections.size(); ++i) {
    if (detection_used[i]) {
      continue;
    }
    const auto& detection = detections[i];
    AxisFilter x{detection.x, 0.0, 0.25, 0.0, 4.0};
    AxisFilter y{detection.y, 0.0, 0.25, 0.0, 4.0};
    tracks_.push_back({next_id_++, x, y, detection.radius, 1, 0, 0, stamp, stamp});
  }
  tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                               [](const ObstacleTrack& track) {
                                 return track.misses > 5;
                               }), tracks_.end());
  TrackerOutput output;
  output.dynamic_costmap.header.stamp = stamp;
  output.dynamic_costmap.header.frame_id = map.header.frame_id;
  output.dynamic_costmap.info = map.info;
  output.dynamic_costmap.info.origin.position.z = 0.1;
  output.dynamic_costmap.data.assign(map.info.width * map.info.height, 0);
  for (const auto& track : tracks_) {
    if (!isDynamic(track, stamp)) {
      continue;
    }
    for (double t = 0.0; t <= occupancy_horizon_; t += prediction_step_) {
      auto predicted = predictPosition(track, t);
      double variance = std::max(0.0, track.x.p00 + track.y.p00 +
                                 t * t * (track.x.p11 + track.y.p11));
      double uncertainty = std::min(0.5, std::sqrt(variance));
      double radius = track.radius + robot_radius_ + safety_margin_ + uncertainty;
      rasterize(output.dynamic_costmap, predicted.x, predicted.y, radius, 100);
    }
  }
  output.markers = makeMarkers(stamp);
  return output;
}

std::vector<Detection> DynamicTrackerCore::detect(
    const sensor_msgs::msg::LaserScan& scan,
    const nav_msgs::msg::Odometry& odom) const
{
  std::vector<Detection> detections;
  std::vector<geometry_msgs::msg::Point> cluster;
  double qx = odom.pose.pose.orientation.x;
  double qy = odom.pose.pose.orientation.y;
  double qz = odom.pose.pose.orientation.z;
  double qw = odom.pose.pose.orientation.w;
  double yaw = std::atan2(2.0 * (qw * qz + qx * qy),
                          1.0 - 2.0 * (qy * qy + qz * qz));
  auto finish_cluster = [&]() {
    if (static_cast<int>(cluster.size()) < minimum_cluster_size_) {
      cluster.clear();
      return;
    }
    double cx = 0.0;
    double cy = 0.0;
    for (const auto& point : cluster) {
      cx += point.x;
      cy += point.y;
    }
    cx /= static_cast<double>(cluster.size());
    cy /= static_cast<double>(cluster.size());
    double radius = 0.0;
    for (const auto& point : cluster) {
      radius = std::max(radius, std::hypot(point.x - cx, point.y - cy));
    }
    if (radius <= maximum_cluster_radius_) {
      detections.push_back({cx, cy, std::max(0.35, radius)});
    }
    cluster.clear();
  };
  geometry_msgs::msg::Point previous;
  bool has_previous = false;
  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    double range = scan.ranges[i];
    if (!std::isfinite(range) || range < scan.range_min || range >= scan.range_max) {
      finish_cluster();
      has_previous = false;
      continue;
    }
    double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    double lx = range * std::cos(angle);
    double ly = range * std::sin(angle);
    geometry_msgs::msg::Point point;
    point.x = odom.pose.pose.position.x + std::cos(yaw) * lx - std::sin(yaw) * ly;
    point.y = odom.pose.pose.position.y + std::sin(yaw) * lx + std::cos(yaw) * ly;
    if (has_previous && std::hypot(point.x - previous.x, point.y - previous.y) >
                            cluster_threshold_) {
      finish_cluster();
    }
    cluster.push_back(point);
    previous = point;
    has_previous = true;
  }
  finish_cluster();
  return detections;
}

void DynamicTrackerCore::predict(ObstacleTrack& track, double dt) const
{
  auto predict_axis = [&](AxisFilter& axis) {
    double old_p00 = axis.p00;
    double old_p01 = axis.p01;
    double old_p11 = axis.p11;
    axis.position += axis.velocity * dt;
    axis.p00 = old_p00 + 2.0 * dt * old_p01 + dt * dt * old_p11 +
               process_noise_ * dt * dt * dt * dt / 4.0;
    axis.p01 = old_p01 + dt * old_p11 + process_noise_ * dt * dt * dt / 2.0;
    axis.p11 = old_p11 + process_noise_ * dt * dt;
  };
  predict_axis(track.x);
  predict_axis(track.y);
}

void DynamicTrackerCore::update(ObstacleTrack& track, const Detection& detection) const
{
  updateAxis(track.x, detection.x);
  updateAxis(track.y, detection.y);
  track.radius = 0.8 * track.radius + 0.2 * detection.radius;
}

void DynamicTrackerCore::updateAxis(AxisFilter& axis, double measurement) const
{
  double variance = measurement_noise_ * measurement_noise_;
  double innovation = measurement - axis.position;
  double innovation_variance = axis.p00 + variance;
  double k0 = axis.p00 / innovation_variance;
  double k1 = axis.p01 / innovation_variance;
  double old_p00 = axis.p00;
  double old_p01 = axis.p01;
  double old_p11 = axis.p11;
  axis.position += k0 * innovation;
  axis.velocity += k1 * innovation;
  axis.p00 = (1.0 - k0) * old_p00;
  axis.p01 = (1.0 - k0) * old_p01;
  axis.p11 = old_p11 - k1 * old_p01;
}

bool DynamicTrackerCore::isDynamic(const ObstacleTrack& track,
                                   const rclcpp::Time& now) const
{
  return track.hits >= 3 && track.dynamic_until > now;
}

geometry_msgs::msg::Point DynamicTrackerCore::predictPosition(
    const ObstacleTrack& track, double time) const
{
  geometry_msgs::msg::Point point;
  point.x = track.x.position + track.x.velocity * time;
  point.y = track.y.position + track.y.velocity * time;
  if (motion_max_x_ > motion_min_x_) {
    while (point.x > motion_max_x_ || point.x < motion_min_x_) {
      if (point.x > motion_max_x_) {
        point.x = motion_max_x_ - (point.x - motion_max_x_);
      }
      if (point.x < motion_min_x_) {
        point.x = motion_min_x_ + (motion_min_x_ - point.x);
      }
    }
  }
  return point;
}

void DynamicTrackerCore::rasterize(nav_msgs::msg::OccupancyGrid& grid,
                                   double x, double y, double radius,
                                   int8_t cost) const
{
  double resolution = grid.info.resolution;
  int center_x = static_cast<int>((x - grid.info.origin.position.x) / resolution);
  int center_y = static_cast<int>((y - grid.info.origin.position.y) / resolution);
  int cells = static_cast<int>(std::ceil(radius / resolution));
  for (int dy = -cells; dy <= cells; ++dy) {
    for (int dx = -cells; dx <= cells; ++dx) {
      if (std::hypot(static_cast<double>(dx), static_cast<double>(dy)) * resolution > radius) {
        continue;
      }
      int gx = center_x + dx;
      int gy = center_y + dy;
      if (gx < 0 || gy < 0 || gx >= static_cast<int>(grid.info.width) ||
          gy >= static_cast<int>(grid.info.height)) {
        continue;
      }
      grid.data[static_cast<std::size_t>(gy) * grid.info.width +
                static_cast<std::size_t>(gx)] = cost;
    }
  }
}

visualization_msgs::msg::MarkerArray DynamicTrackerCore::makeMarkers(
    const rclcpp::Time& stamp) const
{
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker clear;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);
  for (const auto& track : tracks_) {
    if (!isDynamic(track, stamp)) {
      continue;
    }
    visualization_msgs::msg::Marker body;
    body.header.stamp = stamp;
    body.header.frame_id = "sim_world";
    body.ns = "tracked_obstacles";
    body.id = track.id * 10;
    body.type = visualization_msgs::msg::Marker::SPHERE;
    body.action = visualization_msgs::msg::Marker::ADD;
    body.pose.position.x = track.x.position;
    body.pose.position.y = track.y.position;
    body.pose.position.z = 0.75;
    body.pose.orientation.w = 1.0;
    body.scale.x = 2.0 * track.radius;
    body.scale.y = 2.0 * track.radius;
    body.scale.z = 1.5;
    body.color.r = 0.1f;
    body.color.g = 1.0f;
    body.color.b = 0.2f;
    body.color.a = 0.8f;
    markers.markers.push_back(body);
    visualization_msgs::msg::Marker prediction;
    prediction.header = body.header;
    prediction.ns = "predictions";
    prediction.text = std::to_string(prediction_step_);
    prediction.id = track.id * 10 + 1;
    prediction.type = visualization_msgs::msg::Marker::LINE_STRIP;
    prediction.action = visualization_msgs::msg::Marker::ADD;
    prediction.pose.orientation.w = 1.0;
    prediction.scale.x = 0.12;
    prediction.color.r = 1.0f;
    prediction.color.g = 0.45f;
    prediction.color.b = 0.0f;
    prediction.color.a = 1.0f;
    for (double t = 0.0; t <= prediction_horizon_; t += prediction_step_) {
      auto point = predictPosition(track, t);
      point.z = 1.0;
      prediction.points.push_back(point);
    }
    markers.markers.push_back(prediction);
    visualization_msgs::msg::Marker velocity;
    velocity.header = body.header;
    velocity.ns = "velocities";
    velocity.id = track.id * 10 + 2;
    velocity.type = visualization_msgs::msg::Marker::ARROW;
    velocity.action = visualization_msgs::msg::Marker::ADD;
    velocity.pose.orientation.w = 1.0;
    velocity.scale.x = 0.15;
    velocity.scale.y = 0.25;
    velocity.scale.z = 0.3;
    velocity.color.r = 0.0f;
    velocity.color.g = 0.8f;
    velocity.color.b = 1.0f;
    velocity.color.a = 1.0f;
    geometry_msgs::msg::Point start;
    start.x = track.x.position;
    start.y = track.y.position;
    start.z = 1.0;
    geometry_msgs::msg::Point end = start;
    end.x += track.x.velocity * 2.0;
    end.y += track.y.velocity * 2.0;
    velocity.points.push_back(start);
    velocity.points.push_back(end);
    markers.markers.push_back(velocity);
  }
  return markers;
}

}
