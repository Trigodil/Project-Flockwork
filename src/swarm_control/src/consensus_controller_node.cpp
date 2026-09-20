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
    barrier_gain_ = declare_parameter("barrier_gain", 2.0);
    tether_gain_ = declare_parameter("tether_gain", 0.5);
    alignment_gain_ = declare_parameter("alignment_gain", 0.5);
    accel_feedforward_gain_ = declare_parameter("accel_feedforward_gain", 0.25);
    navigation_gain_ = declare_parameter("navigation_gain", 0.6);
    accel_filter_k_ = declare_parameter("accel_filter_k", 0.2);
    max_horizontal_speed_ = declare_parameter("max_horizontal_speed", 2.0);
    max_cmd_accel_ = declare_parameter("max_cmd_accel", 3.0);

    const auto obstacle_x = declare_parameter("obstacle_x", std::vector<double>{});
    const auto obstacle_y = declare_parameter("obstacle_y", std::vector<double>{});
    const auto obstacle_radius = declare_parameter("obstacle_radius", std::vector<double>{});
    for (size_t i = 0; i < obstacle_x.size(); ++i) {
      obstacles_.push_back({Eigen::Vector2d(obstacle_x[i], obstacle_y[i]), obstacle_radius[i]});
    }
    obstacle_avoidance_.configure(
      declare_parameter("obstacle_detection_radius", 2.0),
      declare_parameter("obstacle_safety_margin", 0.4),
      declare_parameter("obstacle_radial_gain", 1.5),
      declare_parameter("obstacle_tangential_gain", 1.0));

    use_lidar_sensing_ = declare_parameter("use_lidar_sensing", false);
    lidar_detector_.configure(
      declare_parameter("lidar_cluster_gap", 0.3),
      declare_parameter("lidar_min_cluster_points", 2),
      declare_parameter("lidar_radius_padding", 0.1),
      declare_parameter("lidar_self_exclusion_radius", 0.45));

    target_x_ = declare_parameter("target_x", 0.0);
    target_y_ = declare_parameter("target_y", 0.0);
    target_z_ = declare_parameter("target_z", 2.0);

    own_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10,
      std::bind(&ConsensusControllerNode::own_odom_callback, this, std::placeholders::_1));

    target_override_sub_ = create_subscription<geometry_msgs::msg::Point>(
      "target_override", 10,
      [this](const geometry_msgs::msg::Point::SharedPtr msg) {
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

    Eigen::Vector2d separation = Eigen::Vector2d::Zero();
    Eigen::Vector2d sum_velocity = Eigen::Vector2d::Zero();
    Eigen::Vector2d sum_accel = Eigen::Vector2d::Zero();
    int active_neighbors = 0;

    for (auto & nb : neighbors_) {
      if (!nb.have_measurement) {
        continue;
      }
      // Dead reckon through latency using the last broadcast velocity.
      const double dt_since = (current_time - nb.last_measurement_time).seconds();
      const Eigen::Vector2d predicted_position = nb.position + nb.velocity * dt_since;

      const Eigen::Vector2d offset = own_position - predicted_position;
      const double dist = offset.norm();
      if (dist < 1e-6) {
        continue;
      }
      const Eigen::Vector2d direction = offset / dist;

      // Summed over every neighbor, not just nearest, to avoid a discontinuous switch.
      if (dist <= min_safe_distance_) {
        separation += FORCE_CAP * direction;
      } else if (dist < desired_spacing_) {
        double magnitude = barrier_gain_ *
          (1.0 / (dist - min_safe_distance_) - 1.0 / (desired_spacing_ - min_safe_distance_));
        separation += std::clamp(magnitude, 0.0, FORCE_CAP) * direction;
      } else {
        double magnitude = std::clamp(tether_gain_ * (dist - desired_spacing_), 0.0, FORCE_CAP);
        separation -= magnitude * direction;
      }

      if (dist > interaction_range_) {
        continue;
      }
      sum_velocity += nb.velocity;
      sum_accel += nb.accel;
      ++active_neighbors;
    }
    if (separation.norm() > FORCE_CAP) {
      separation = separation.normalized() * FORCE_CAP;
    }

    Eigen::Vector2d alignment = Eigen::Vector2d::Zero();
    Eigen::Vector2d accel_feedforward = Eigen::Vector2d::Zero();
    if (active_neighbors > 0) {
      alignment = alignment_gain_ * (sum_velocity / active_neighbors - own_velocity);
      accel_feedforward = accel_feedforward_gain_ * (sum_accel / active_neighbors);
    }

    const Eigen::Vector2d navigation =
      navigation_gain_ * (Eigen::Vector2d(target_x_, target_y_) - own_position);

    const Eigen::Vector2d desired = separation + alignment + accel_feedforward + navigation;
    std::vector<swarm_control::Obstacle> active_obstacles = obstacles_;
    if (use_lidar_sensing_) {
      active_obstacles.insert(
        active_obstacles.end(), sensed_obstacles_.begin(), sensed_obstacles_.end());
    }
    const Eigen::Vector2d avoidance =
      obstacle_avoidance_.compute(own_position, desired, active_obstacles);

    Eigen::Vector2d cmd = desired + avoidance;
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
    last_cmd_ = cmd;

    geometry_msgs::msg::Twist cmd_msg;
    cmd_msg.linear.x = cmd.x();
    cmd_msg.linear.y = cmd.y();
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
  double min_safe_distance_, barrier_gain_, tether_gain_;
  double alignment_gain_, accel_feedforward_gain_, navigation_gain_;
  double accel_filter_k_;
  double max_horizontal_speed_;
  double max_cmd_accel_;
  Eigen::Vector2d last_cmd_ = Eigen::Vector2d::Zero();

  double target_x_, target_y_, target_z_;

  std::vector<NeighborEstimate> neighbors_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> neighbor_subs_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr own_odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_override_sub_;
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
