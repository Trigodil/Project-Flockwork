#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include <Eigen/Dense>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "swarm_control/obstacle_avoidance.hpp"

namespace swarm_control
{

class LidarObstacleDetector
{
public:
  LidarObstacleDetector() = default;

  void configure(
    double cluster_gap, int min_cluster_points, double radius_padding,
    double self_exclusion_radius = 0.0)
  {
    cluster_gap_ = cluster_gap;
    min_cluster_points_ = min_cluster_points;
    radius_padding_ = radius_padding;
    self_exclusion_radius_ = self_exclusion_radius;
  }

  std::vector<Obstacle> detect(
    const sensor_msgs::msg::LaserScan & scan,
    const Eigen::Vector2d & sensor_position,
    double sensor_yaw) const
  {
    const double min_range = std::max<double>(scan.range_min, self_exclusion_radius_);
    std::vector<Eigen::Vector2d> points;
    points.reserve(scan.ranges.size());
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
      const float range = scan.ranges[i];
      if (!std::isfinite(range) || range < min_range || range > scan.range_max) {
        continue;
      }
      const double world_angle =
        sensor_yaw + scan.angle_min + static_cast<double>(i) * scan.angle_increment;
      points.emplace_back(
        sensor_position.x() + range * std::cos(world_angle),
        sensor_position.y() + range * std::sin(world_angle));
    }

    std::vector<Obstacle> obstacles;
    size_t start = 0;
    while (start < points.size()) {
      size_t end = start + 1;
      while (end < points.size() && (points[end] - points[end - 1]).norm() <= cluster_gap_) {
        ++end;
      }

      if (static_cast<int>(end - start) >= min_cluster_points_) {
        Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
        for (size_t i = start; i < end; ++i) {
          centroid += points[i];
        }
        centroid /= static_cast<double>(end - start);

        double max_dist = 0.0;
        for (size_t i = start; i < end; ++i) {
          max_dist = std::max(max_dist, (points[i] - centroid).norm());
        }
        obstacles.push_back({centroid, max_dist + radius_padding_});
      }
      start = end;
    }

    return obstacles;
  }

private:
  double cluster_gap_ = 0.3;
  int min_cluster_points_ = 2;
  double radius_padding_ = 0.1;
  double self_exclusion_radius_ = 0.0;
};

}  // namespace swarm_control
