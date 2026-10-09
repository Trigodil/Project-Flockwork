#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include <Eigen/Dense>

namespace swarm_control
{

struct Obstacle
{
  Eigen::Vector2d position;
  double radius;
};

class ObstacleAvoidance
{
public:
  ObstacleAvoidance() = default;

  void configure(
    double detection_radius, double safety_margin,
    double radial_gain, double lateral_gain, double min_closing_speed = 0.3)
  {
    detection_radius_ = detection_radius;
    safety_margin_ = safety_margin;
    radial_gain_ = radial_gain;
    lateral_gain_ = lateral_gain;
    min_closing_speed_ = min_closing_speed;
  }

  // open_side points away from crowded neighbors, breaks ties on head-on approaches.
  Eigen::Vector2d compute(
    const Eigen::Vector2d & position,
    const Eigen::Vector2d & velocity,
    const Eigen::Vector2d & travel_direction,
    const Eigen::Vector2d & open_side,
    const std::vector<Obstacle> & obstacles) const
  {
    Eigen::Vector2d avoidance = Eigen::Vector2d::Zero();
    constexpr double FORCE_CAP = 40.0;
    constexpr double EMERGENCY_CAP = 100.0;
    if (detection_radius_ <= 0.0) {
      return avoidance;
    }

    const bool have_heading = travel_direction.squaredNorm() > 1e-6;
    const Eigen::Vector2d forward =
      have_heading ? Eigen::Vector2d(travel_direction.normalized()) : Eigen::Vector2d::Zero();
    const Eigen::Vector2d left(-forward.y(), forward.x());
    const double closing_speed = std::max(velocity.dot(forward), min_closing_speed_);
    const double crowd_preference = std::clamp(open_side.dot(left), -1.0, 1.0);

    for (const auto & obs : obstacles) {
      const Eigen::Vector2d to_obstacle = obs.position - position;
      const double dist = to_obstacle.norm();
      if (dist < 1e-6) {
        continue;
      }

      const double clearance = dist - obs.radius - safety_margin_;
      if (clearance < 0.0 && safety_margin_ > 0.0) {
        // Zero at the margin edge, unbounded at the surface, so no kick on entry.
        const double depth = std::min(-clearance / safety_margin_, 0.999);
        const double magnitude =
          std::min(radial_gain_ * safety_margin_ * depth / (1.0 - depth), EMERGENCY_CAP);
        avoidance -= magnitude * to_obstacle / dist;
      }

      if (!have_heading) {
        continue;
      }
      const double along = to_obstacle.dot(forward);
      const double lateral = to_obstacle.dot(left);
      const double required = obs.radius + safety_margin_;
      if (along <= 0.0 || along - required > detection_radius_ || std::abs(lateral) >= required) {
        continue;
      }

      // Geometry decides once off-center, neighbors decide when nearly head-on.
      const double side_score = -lateral + 0.25 * required * crowd_preference;
      const double side = side_score >= 0.0 ? 1.0 : -1.0;
      const double shift_needed = required + side * lateral;
      const double time_to_reach = std::max(along, 0.2) / closing_speed;
      const double lateral_speed =
        std::clamp(lateral_gain_ * shift_needed / time_to_reach, 0.0, FORCE_CAP);
      avoidance += lateral_speed * side * left;
    }
    if (avoidance.norm() > EMERGENCY_CAP) {
      avoidance = avoidance.normalized() * EMERGENCY_CAP;
    }

    return avoidance;
  }

  // Caps the speed into each obstacle so the drone can still stop before the margin,
  // redirecting the removed speed along the surface so it slides around instead of stalling.
  Eigen::Vector2d limitApproach(
    const Eigen::Vector2d & position, Eigen::Vector2d cmd,
    const std::vector<Obstacle> & obstacles, double max_decel) const
  {
    for (const auto & obs : obstacles) {
      const Eigen::Vector2d offset = position - obs.position;
      const double dist = offset.norm();
      if (dist < 1e-6) {
        continue;
      }
      const Eigen::Vector2d outward = offset / dist;
      const double speed = cmd.norm();
      cmd = capInwardSpeed(cmd, outward, dist - obs.radius - safety_margin_, max_decel);

      const double normal = cmd.dot(outward);
      const Eigen::Vector2d tangential = cmd - normal * outward;
      if (tangential.norm() > 1e-3) {
        const double tangential_speed =
          std::sqrt(std::max(speed * speed - normal * normal, 0.0));
        cmd = normal * outward + tangential.normalized() * tangential_speed;
      }
    }
    return cmd;
  }

  // Removes only the inward excess, tangential motion is left untouched.
  static Eigen::Vector2d capInwardSpeed(
    Eigen::Vector2d cmd, const Eigen::Vector2d & outward, double clearance, double max_decel)
  {
    const double allowed_inward = std::sqrt(2.0 * max_decel * std::max(clearance, 0.0));
    const double inward = -cmd.dot(outward);
    if (inward > allowed_inward) {
      cmd += (inward - allowed_inward) * outward;
    }
    return cmd;
  }

  double nearbyFactor(const Eigen::Vector2d & position, const std::vector<Obstacle> & obstacles) const
  {
    double max_proximity = 0.0;
    if (detection_radius_ <= 0.0) {
      return max_proximity;
    }
    for (const auto & obs : obstacles) {
      const double clearance =
        (position - obs.position).norm() - obs.radius - safety_margin_;
      const double proximity = std::clamp(1.0 - clearance / detection_radius_, 0.0, 1.0);
      max_proximity = std::max(max_proximity, proximity);
    }
    return max_proximity;
  }

private:
  double detection_radius_ = 0.0;
  double safety_margin_ = 0.0;
  double radial_gain_ = 0.0;
  double lateral_gain_ = 0.0;
  double min_closing_speed_ = 0.3;
};

}  // namespace swarm_control
