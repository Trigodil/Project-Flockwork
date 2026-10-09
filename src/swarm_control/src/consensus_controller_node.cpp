#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "swarm_control/lidar_obstacle_detector.hpp"
#include "swarm_control/obstacle_avoidance.hpp"
#include "swarm_control/pid_controller.hpp"

namespace
{
struct NeighborEstimate
{
  Eigen::Vector2d position = Eigen::Vector2d::Zero();
  Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
  Eigen::Vector2d accel = Eigen::Vector2d::Zero();  // low-pass filtered estimate
  bool have_measurement = false;
  bool have_prev_velocity = false;
  rclcpp::Time last_measurement_time;
};
}  // namespace

// Timestamps use our own now(), not the sender's stamp, so dead-reckoning
// stays consistent across clock differences.
// Acceleration is low-pass filtered, not raw-differenced, to avoid
// amplifying broadcast noise.
class ConsensusControllerNode : public rclcpp::Node
{
public:
  ConsensusControllerNode()
  : Node("consensus_controller"),
    pid_z_(1.5, 0.1, 0.4, 1.0)
  {
    // interaction_range must stay below the inter-group gap or separation never switches off.
    neighbor_names_ = declare_parameter("neighbor_names", std::vector<std::string>{});
    // X3 rotors need >=~0.71m center-to-center (0.256m offset + 0.1m radius each);
    // min_safe_distance must stay above that.
    interaction_range_ = declare_parameter("interaction_range", 2.5);
    desired_spacing_ = declare_parameter("desired_spacing", 1.5);
    min_safe_distance_ = declare_parameter("min_safe_distance", 0.9);
    barrier_gain_ = declare_parameter("barrier_gain", 3.0);
    barrier_tangential_gain_ = declare_parameter("barrier_tangential_gain", 3.0);
    tether_gain_ = declare_parameter("tether_gain", 0.5);
    tether_force_cap_ = declare_parameter("tether_force_cap", 5.0);
    free_zone_margin_ = declare_parameter("free_zone_margin", 0.3);
    collision_lookahead_time_ = declare_parameter("collision_lookahead_time", 1.5);
    sidestep_decay_time_ = declare_parameter("sidestep_decay_time", 1.0);
    sidestep_gain_ = declare_parameter("sidestep_gain", 3.0);
    alignment_gain_ = declare_parameter("alignment_gain", 0.5);
    accel_feedforward_gain_ = declare_parameter("accel_feedforward_gain", 0.25);
    navigation_gain_ = declare_parameter("navigation_gain", 0.6);
    accel_filter_k_ = declare_parameter("accel_filter_k", 0.2);
    max_horizontal_speed_ = declare_parameter("max_horizontal_speed", 2.0);
    max_cmd_accel_ = declare_parameter("max_cmd_accel", 3.0);
    idle_deadband_ = declare_parameter("idle_deadband", 0.05);
    arrival_radius_ = declare_parameter("arrival_radius", 0.5);
    yaw_hold_gain_ = declare_parameter("yaw_hold_gain", 1.0);
    takeoff_fraction_ = declare_parameter("takeoff_fraction", 0.8);
    neighbor_brake_decel_ = declare_parameter("neighbor_brake_decel", 1.5);
    active_speed_threshold_ = declare_parameter("active_speed_threshold", 0.3);
    min_active_drones_ = declare_parameter("min_active_drones", 2);
    settle_gain_scale_ = declare_parameter("settle_gain_scale", 0.3);
    // Below the plugin's maximumLinearAcceleration (2.0) so the braking is achievable.
    obstacle_brake_decel_ = declare_parameter("obstacle_brake_decel", 1.5);

    if (desired_spacing_ <= min_safe_distance_) {
      RCLCPP_WARN(
        get_logger(),
        "desired_spacing %.3f is not above min_safe_distance %.3f, raising it to %.3f",
        desired_spacing_, min_safe_distance_, min_safe_distance_ + 0.1);
      desired_spacing_ = min_safe_distance_ + 0.1;
    }
    const double max_free_zone = (desired_spacing_ - min_safe_distance_) * 0.5;
    if (free_zone_margin_ > max_free_zone) {
      RCLCPP_WARN(
        get_logger(),
        "free_zone_margin %.3f would collapse the barrier band, clamping to %.3f",
        free_zone_margin_, max_free_zone);
      free_zone_margin_ = max_free_zone;
    }

    const auto obstacle_x = declare_parameter("obstacle_x", std::vector<double>{});
    const auto obstacle_y = declare_parameter("obstacle_y", std::vector<double>{});
    const auto obstacle_radius = declare_parameter("obstacle_radius", std::vector<double>{});
    for (size_t i = 0; i < obstacle_x.size(); ++i) {
      obstacles_.push_back({Eigen::Vector2d(obstacle_x[i], obstacle_y[i]), obstacle_radius[i]});
    }
    obstacle_avoidance_.configure(
      declare_parameter("obstacle_detection_radius", 3.0),
      // Must exceed the X3 rotor reach (~0.36m) with room for tracking lag.
      declare_parameter("obstacle_safety_margin", 0.8),
      declare_parameter("obstacle_radial_gain", 1.5),
      declare_parameter("obstacle_lateral_gain", 2.0));

    use_lidar_sensing_ = declare_parameter("use_lidar_sensing", false);
    lidar_detector_.configure(
      declare_parameter("lidar_cluster_gap", 0.3),
      declare_parameter("lidar_min_cluster_points", 2),
      declare_parameter("lidar_radius_padding", 0.1),
      declare_parameter("lidar_self_exclusion_radius", 0.45));

    target_x_ = declare_parameter("target_x", 0.0);
    target_y_ = declare_parameter("target_y", 0.0);
    target_z_ = declare_parameter("target_z", 2.0);

    use_formation_ = declare_parameter("use_formation", false);
    formation_gain_ = declare_parameter("formation_gain", 0.8);
    formation_deadband_ = declare_parameter("formation_deadband", 0.15);
    // Neighbor slot lists are aligned index-for-index with neighbor_names.
    const auto neighbor_slot_x = declare_parameter("neighbor_slot_x", std::vector<double>{});
    const auto neighbor_slot_y = declare_parameter("neighbor_slot_y", std::vector<double>{});
    if (use_formation_) {
      if (neighbor_slot_x.size() != neighbor_names_.size() ||
        neighbor_slot_y.size() != neighbor_names_.size())
      {
        RCLCPP_WARN(get_logger(), "neighbor slot lists don't match neighbor_names, formation disabled");
        use_formation_ = false;
      } else {
        own_slot_ = Eigen::Vector2d(
          declare_parameter("slot_x", 0.0), declare_parameter("slot_y", 0.0));
        for (size_t i = 0; i < neighbor_slot_x.size(); ++i) {
          neighbor_slots_.emplace_back(neighbor_slot_x[i], neighbor_slot_y[i]);
        }
      }
    }

    own_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10,
      std::bind(&ConsensusControllerNode::own_odom_callback, this, std::placeholders::_1));

    target_override_sub_ = create_subscription<geometry_msgs::msg::Point>(
      "target_override", 10,
      [this](const geometry_msgs::msg::Point::SharedPtr msg) {
        if (manual_target_active_) {
          return;
        }
        target_x_ = msg->x;
        target_y_ = msg->y;
      });

    manual_target_sub_ = create_subscription<geometry_msgs::msg::Point>(
      "manual_target", 10,
      [this](const geometry_msgs::msg::Point::SharedPtr msg) {
        manual_target_active_ = true;
        target_x_ = msg->x;
        target_y_ = msg->y;
      });

    if (use_lidar_sensing_) {
      scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", rclcpp::SensorDataQoS(),
        std::bind(&ConsensusControllerNode::scan_callback, this, std::placeholders::_1));
    }

    neighbors_.resize(neighbor_names_.size());
    for (size_t i = 0; i < neighbor_names_.size(); ++i) {
      neighbor_subs_.push_back(create_subscription<nav_msgs::msg::Odometry>(
        "/" + neighbor_names_[i] + "/odom", 10,
        [this, i](const nav_msgs::msg::Odometry::SharedPtr msg) {
          neighbor_callback(i, msg);
        }));
    }

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    last_time_ = now();

    RCLCPP_INFO(
      get_logger(), "consensus_controller started with %zu neighbors",
      neighbor_names_.size());
  }

private:
  void neighbor_callback(size_t i, const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    NeighborEstimate & nb = neighbors_[i];
    const rclcpp::Time receipt_time = now();
    const Eigen::Vector2d new_velocity(msg->twist.twist.linear.x, msg->twist.twist.linear.y);

    if (nb.have_prev_velocity) {
      const double dt = (receipt_time - nb.last_measurement_time).seconds();
      if (dt > 1e-4) {
        const Eigen::Vector2d raw_accel = (new_velocity - nb.velocity) / dt;
        nb.accel = nb.accel * (1.0 - accel_filter_k_) + raw_accel * accel_filter_k_;
      }
    } else {
      nb.have_prev_velocity = true;
    }

    nb.position = Eigen::Vector2d(msg->pose.pose.position.x, msg->pose.pose.position.y);
    nb.velocity = new_velocity;
    nb.last_measurement_time = receipt_time;
    nb.have_measurement = true;
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    sensed_obstacles_ = lidar_detector_.detect(*msg, own_position_, own_yaw_);
  }

  // Real hardware: needs the lidar's actual tf2 mounting offset, not zero.
  // Eigen::Vector2d ConsensusControllerNode::lidarOffset() const
  // {
  //   auto tf = tf_buffer_->lookupTransform("base_link", "lidar_link", tf2::TimePointZero);
  //   return Eigen::Vector2d(tf.transform.translation.x, tf.transform.translation.y);
  // }

  void own_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const rclcpp::Time current_time = now();
    const double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;
    if (dt <= 0.0) {
      return;
    }

    const Eigen::Vector2d own_position(msg->pose.pose.position.x, msg->pose.pose.position.y);
    const Eigen::Vector2d own_velocity(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
    own_position_ = own_position;
    const auto & q = msg->pose.pose.orientation;
    own_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));

    constexpr double FORCE_CAP = 20.0;

    std::vector<swarm_control::Obstacle> active_obstacles = obstacles_;
    if (use_lidar_sensing_) {
      active_obstacles.insert(
        active_obstacles.end(), sensed_obstacles_.begin(), sensed_obstacles_.end());
    }
    const double obstacle_proximity =
      obstacle_avoidance_.nearbyFactor(own_position, active_obstacles);

    // Free zone around desired_spacing where separation is off entirely, so a
    // drone at a good distance isn't constantly nudged by barrier/tether noise.
    const double spacing_inner = std::max(min_safe_distance_ + 1e-3, desired_spacing_ - free_zone_margin_);
    const double spacing_outer = desired_spacing_ + free_zone_margin_;

    const Eigen::Vector2d goal = Eigen::Vector2d(target_x_, target_y_) + own_slot_;
    // Capped so a far target can't drown out separation and avoidance.
    Eigen::Vector2d navigation = navigation_gain_ * (goal - own_position);
    if (navigation.norm() > max_horizontal_speed_) {
      navigation = navigation.normalized() * max_horizontal_speed_;
    }

    Eigen::Vector2d separation = Eigen::Vector2d::Zero();
    Eigen::Vector2d emergency = Eigen::Vector2d::Zero();
    std::vector<Eigen::Vector2d> neighbor_points;
    int moving_drones = own_velocity.norm() > active_speed_threshold_ ? 1 : 0;
    Eigen::Vector2d sum_velocity = Eigen::Vector2d::Zero();
    Eigen::Vector2d sum_accel = Eigen::Vector2d::Zero();
    Eigen::Vector2d open_side = Eigen::Vector2d::Zero();
    Eigen::Vector2d formation_error = Eigen::Vector2d::Zero();
    int formation_count = 0;
    int active_neighbors = 0;
    bool repelled = false;
    bool conflict_found = false;
    double most_urgent = collision_lookahead_time_;
    Eigen::Vector2d conflict_tangent = Eigen::Vector2d::Zero();

    for (size_t i = 0; i < neighbors_.size(); ++i) {
      const NeighborEstimate & nb = neighbors_[i];
      if (!nb.have_measurement) {
        continue;
      }
      // Dead reckon through latency using the last broadcast velocity.
      const double dt_since = (current_time - nb.last_measurement_time).seconds();
      const Eigen::Vector2d predicted_position = nb.position + nb.velocity * dt_since;
      neighbor_points.push_back(predicted_position);
      if (nb.velocity.norm() > active_speed_threshold_) {
        ++moving_drones;
      }

      const Eigen::Vector2d offset = own_position - predicted_position;
      const double dist = offset.norm();
      if (dist < 1e-6) {
        continue;
      }
      const Eigen::Vector2d direction = offset / dist;
      const Eigen::Vector2d tangent(-direction.y(), direction.x());
      const Eigen::Vector2d chosen_tangent =
        tangent.dot(navigation) >= 0.0 ? tangent : -tangent;

      // Summed over every neighbor, not just nearest, to avoid a discontinuous switch.
      if (dist <= min_safe_distance_) {
        emergency += FORCE_CAP * direction + barrier_tangential_gain_ * chosen_tangent;
        repelled = true;
      } else if (dist <= interaction_range_) {
        if (dist < spacing_inner) {
          double magnitude = barrier_gain_ *
            (1.0 / (dist - min_safe_distance_) - 1.0 / (spacing_inner - min_safe_distance_));
          const double proximity = std::clamp(
            1.0 - (dist - min_safe_distance_) / (spacing_inner - min_safe_distance_), 0.0, 1.0);
          separation += std::clamp(magnitude, 0.0, FORCE_CAP) * direction +
            barrier_tangential_gain_ * proximity * chosen_tangent;
          repelled = true;
        } else if (dist > spacing_outer && !use_formation_) {
          // Near an obstacle, loosen cohesion so avoidance isn't fighting the tether.
          const double effective_tether_gain = tether_gain_ * (1.0 - obstacle_proximity);
          double magnitude =
            std::clamp(effective_tether_gain * (dist - spacing_outer), 0.0, tether_force_cap_);
          separation -= magnitude * direction;
        }
      }

      // Predict closest approach from current velocities, sidestep early if it's a hit.
      // Skipped inside the barrier band, the barrier already owns that case.
      const Eigen::Vector2d rel_vel = own_velocity - nb.velocity;
      const double rel_speed_sq = rel_vel.squaredNorm();
      if (dist >= spacing_inner && rel_speed_sq > 1e-6) {
        const double t_closest = -offset.dot(rel_vel) / rel_speed_sq;
        if (t_closest > 0.0 && t_closest < most_urgent &&
          (offset + rel_vel * t_closest).norm() < min_safe_distance_)
        {
          most_urgent = t_closest;
          conflict_tangent = chosen_tangent;
          conflict_found = true;
        }
      }

      if (dist > interaction_range_) {
        continue;
      }
      if (use_formation_) {
        formation_error +=
          (predicted_position - neighbor_slots_[i]) - (own_position - own_slot_);
        ++formation_count;
      }
      open_side += direction / dist;
      sum_velocity += nb.velocity;
      sum_accel += nb.accel;
      ++active_neighbors;
    }
    // Settle mode: gentle corrections when the swarm is mostly still, avoids chain reactions.
    const double settle_scale = moving_drones < min_active_drones_ ? settle_gain_scale_ : 1.0;
    separation = settle_scale * separation + emergency;
    if (separation.norm() > FORCE_CAP) {
      separation = separation.normalized() * FORCE_CAP;
    }

    if (sidestep_decay_time_ > 0.0) {
      sidestep_ *= std::exp(-dt / sidestep_decay_time_);
    } else {
      sidestep_.setZero();
    }
    if (conflict_found) {
      sidestep_ = sidestep_gain_ * conflict_tangent;
    }

    Eigen::Vector2d alignment = Eigen::Vector2d::Zero();
    Eigen::Vector2d accel_feedforward = Eigen::Vector2d::Zero();
    if (active_neighbors > 0) {
      alignment = alignment_gain_ * (sum_velocity / active_neighbors - own_velocity);
      accel_feedforward = accel_feedforward_gain_ * (sum_accel / active_neighbors);
    }

    Eigen::Vector2d formation = Eigen::Vector2d::Zero();
    if (formation_count > 0) {
      formation_error /= formation_count;
      const double error = formation_error.norm();
      if (error > formation_deadband_) {
        // Loosened near obstacles so the shape can bend around them.
        formation = formation_gain_ * (1.0 - obstacle_proximity) *
          (error - formation_deadband_) * formation_error / error;
        if (formation.norm() > max_horizontal_speed_) {
          formation = formation.normalized() * max_horizontal_speed_;
        }
      }
    }

    const bool sidestepping = sidestep_.squaredNorm() > idle_deadband_ * idle_deadband_;
    const Eigen::Vector2d desired =
      separation + alignment + accel_feedforward + navigation +
      settle_scale * (sidestep_ + formation);
    const Eigen::Vector2d avoidance =
      obstacle_avoidance_.compute(
      own_position, own_velocity, navigation, open_side, active_obstacles);

    Eigen::Vector2d cmd = desired + avoidance;
    const double dist_to_target = (goal - own_position).norm();
    const bool arrived_and_clear =
      dist_to_target < arrival_radius_ && !repelled && !sidestepping &&
      avoidance.squaredNorm() < 1e-12;
    if (arrived_and_clear || cmd.norm() < idle_deadband_) {
      cmd = Eigen::Vector2d::Zero();
    }
    if (!airborne_ && msg->pose.pose.position.z >= takeoff_fraction_ * target_z_) {
      airborne_ = true;
    }
    if (!airborne_) {
      cmd = Eigen::Vector2d::Zero();
    }
    const double speed = cmd.norm();
    if (speed > max_horizontal_speed_) {
      cmd *= max_horizontal_speed_ / speed;
    }

    // Rate-limit: cmd_vel cannot jump discontinuously between ticks.
    const Eigen::Vector2d cmd_delta = cmd - last_cmd_;
    const double max_delta = max_cmd_accel_ * dt;
    if (cmd_delta.norm() > max_delta) {
      cmd = last_cmd_ + cmd_delta.normalized() * max_delta;
    }
    // After the rate limit so braking is never delayed. Repeated passes so
    // clipping against one constraint can't push into another.
    for (int pass = 0; pass < 3; ++pass) {
      cmd = obstacle_avoidance_.limitApproach(
        own_position, cmd, active_obstacles, obstacle_brake_decel_);
      for (const auto & point : neighbor_points) {
        const Eigen::Vector2d offset = own_position - point;
        const double dist = offset.norm();
        if (dist < 1e-6) {
          continue;
        }
        cmd = swarm_control::ObstacleAvoidance::capInwardSpeed(
          cmd, offset / dist, dist - min_safe_distance_, neighbor_brake_decel_);
      }
    }
    last_cmd_ = cmd;

    // The velocity plugin takes body-frame commands.
    const double cos_yaw = std::cos(own_yaw_);
    const double sin_yaw = std::sin(own_yaw_);
    geometry_msgs::msg::Twist cmd_msg;
    cmd_msg.linear.x = cos_yaw * cmd.x() + sin_yaw * cmd.y();
    cmd_msg.linear.y = -sin_yaw * cmd.x() + cos_yaw * cmd.y();
    cmd_msg.angular.z = -yaw_hold_gain_ * own_yaw_;
    cmd_msg.linear.z = std::clamp(
      pid_z_.update(target_z_, msg->pose.pose.position.z, dt),
      -max_horizontal_speed_, max_horizontal_speed_);
    cmd_vel_pub_->publish(cmd_msg);
  }

  swarm_control::PidController pid_z_;
  swarm_control::ObstacleAvoidance obstacle_avoidance_;
  std::vector<swarm_control::Obstacle> obstacles_;

  bool use_lidar_sensing_;
  swarm_control::LidarObstacleDetector lidar_detector_;
  std::vector<swarm_control::Obstacle> sensed_obstacles_;
  Eigen::Vector2d own_position_ = Eigen::Vector2d::Zero();
  double own_yaw_ = 0.0;

  std::vector<std::string> neighbor_names_;
  double interaction_range_, desired_spacing_;
  double min_safe_distance_, barrier_gain_, barrier_tangential_gain_;
  double tether_gain_, tether_force_cap_, free_zone_margin_;
  double collision_lookahead_time_, sidestep_decay_time_, sidestep_gain_;
  Eigen::Vector2d sidestep_ = Eigen::Vector2d::Zero();
  double alignment_gain_, accel_feedforward_gain_, navigation_gain_;
  double accel_filter_k_;
  double max_horizontal_speed_;
  double max_cmd_accel_;
  double idle_deadband_;
  double arrival_radius_;
  double yaw_hold_gain_;
  double takeoff_fraction_;
  double neighbor_brake_decel_;
  double active_speed_threshold_;
  int64_t min_active_drones_;
  double settle_gain_scale_;
  bool airborne_ = false;
  double obstacle_brake_decel_;
  Eigen::Vector2d last_cmd_ = Eigen::Vector2d::Zero();

  double target_x_, target_y_, target_z_;
  bool manual_target_active_ = false;

  bool use_formation_ = false;
  double formation_gain_, formation_deadband_;
  Eigen::Vector2d own_slot_ = Eigen::Vector2d::Zero();
  std::vector<Eigen::Vector2d> neighbor_slots_;

  std::vector<NeighborEstimate> neighbors_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> neighbor_subs_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr own_odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_override_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr manual_target_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  rclcpp::Time last_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ConsensusControllerNode>());
  rclcpp::shutdown();
  return 0;
}
