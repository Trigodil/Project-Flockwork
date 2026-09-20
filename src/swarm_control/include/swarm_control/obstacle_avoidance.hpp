#pragma once

#include <algorithm>
#include <vector>

#include <Eigen/Dense>

namespace swarm_control
{

struct Obstacle
{
  Eigen::Vector2d position;
  double radius;
};

// Pure function of (position, desired direction, obstacles), no neighbor state.
class ObstacleAvoidance
{
public:
  ObstacleAvoidance() = default;

  void configure(
    double detection_radius, double safety_margin,
    double radial_gain, double tangential_gain)
  {
    detection_radius_ = detection_radius;
    safety_margin_ = safety_margin;
    radial_gain_ = radial_gain;
    tangential_gain_ = tangential_gain;
  }

  // desired_direction need not be normalized, only its sign matters.
  Eigen::Vector2d compute(
    const Eigen::Vector2d & position,
    const Eigen::Vector2d & desired_direction,
    const std::vector<Obstacle> & obstacles) const
  {
    Eigen::Vector2d avoidance = Eigen::Vector2d::Zero();

    for (const auto & obs : obstacles) {
      const Eigen::Vector2d offset = position - obs.position;
      const double dist = offset.norm();
      if (dist < 1e-6) {
        continue;
      }
      const double clearance = dist - obs.radius - safety_margin_;
      if (clearance >= detection_radius_) {
        continue;
      }

      const Eigen::Vector2d radial = offset / dist;
      const Eigen::Vector2d tangent(-radial.y(), radial.x());
      const Eigen::Vector2d chosen_tangent =
        tangent.dot(desired_direction) >= 0.0 ? tangent : -tangent;

      const double proximity = std::clamp(1.0 - clearance / detection_radius_, 0.0, 1.0);
      double radial_magnitude = radial_gain_ * proximity;
      if (clearance <= 0.0) {
        radial_magnitude = std::max(radial_magnitude, radial_gain_ * 5.0);
      }

      avoidance += radial_magnitude * radial + tangential_gain_ * proximity * chosen_tangent;
    }

    return avoidance;
  }

private:
  double detection_radius_ = 0.0;
  double safety_margin_ = 0.0;
  double radial_gain_ = 0.0;
  double tangential_gain_ = 0.0;
};

}  // namespace swarm_control
