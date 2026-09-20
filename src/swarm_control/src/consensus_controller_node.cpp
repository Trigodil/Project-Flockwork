#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"

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

// Stage 3: decentralized flocking. No central authority, each drone only
// reacts to its neighbors' broadcasts. Three local forces, separation,
// alignment, acceleration feedforward, plus a navigation term toward a
// shared goal (fed by mean_field_controller_node's target_override if
// stage 5 is running, otherwise the launch-time target_x/y parameters).
//
// Neighbor timestamps are recorded using our own now()(lol), not the sender's
// embedded header stamp, so the dead-reckoning stays self-consistent even if
// clocks differ node to node, same reason stage 1 always used now() for
// both sides of its own dt calculation (coolest thing).
//
// Acceleration is a low-pass filtered derivative of the neighbor's
// broadcast velocity, not raw differentiation, which would amplify
// broadcast noise the same way an unfiltered D term would in a PID loop.
class ConsensusControllerNode : public rclcpp::Node
{
public:
  ConsensusControllerNode()
  : Node("consensus_controller"),
    pid_z_(1.5, 0.1, 0.4, 1.0)
  {
    // interaction_range must stay smaller than the eventual inter-group
    // gap, otherwise cross-group separation never switches off.
    neighbor_names_ = declare_parameter("neighbor_names", std::vector<std::string>{});
    interaction_range_ = declare_parameter("interaction_range", 2.5);
    desired_spacing_ = declare_parameter("desired_spacing", 1.5);
    separation_gain_ = declare_parameter("separation_gain", 1.0);
    alignment_gain_ = declare_parameter("alignment_gain", 0.5);
    accel_feedforward_gain_ = declare_parameter("accel_feedforward_gain", 0.25);
    navigation_gain_ = declare_parameter("navigation_gain", 0.6);
    accel_filter_k_ = declare_parameter("accel_filter_k", 0.2);
    max_horizontal_speed_ = declare_parameter("max_horizontal_speed", 2.0);

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

    Eigen::Vector2d separation = Eigen::Vector2d::Zero();
    Eigen::Vector2d sum_velocity = Eigen::Vector2d::Zero();
    Eigen::Vector2d sum_accel = Eigen::Vector2d::Zero();
    int active_neighbors = 0;

    for (auto & nb : neighbors_) {
      if (!nb.have_measurement) {
        continue;
      }
      // Dead reckon through communication latency using the last
      // broadcast velocity, then measure distance to that prediction.
      const double dt_since = (current_time - nb.last_measurement_time).seconds();
      const Eigen::Vector2d predicted_position = nb.position + nb.velocity * dt_since;

      const Eigen::Vector2d offset = own_position - predicted_position;
      const double dist = offset.norm();
      if (dist > interaction_range_ || dist < 1e-6) {
        continue;
      }

      const Eigen::Vector2d direction = offset / dist;
      const double gap = desired_spacing_ - dist;
      separation += separation_gain_ * gap / desired_spacing_ * direction;

      sum_velocity += nb.velocity;
      sum_accel += nb.accel;
      ++active_neighbors;
    }

    Eigen::Vector2d alignment = Eigen::Vector2d::Zero();
    Eigen::Vector2d accel_feedforward = Eigen::Vector2d::Zero();
    if (active_neighbors > 0) {
      alignment = alignment_gain_ * (sum_velocity / active_neighbors - own_velocity);
      accel_feedforward = accel_feedforward_gain_ * (sum_accel / active_neighbors);
    }

    const Eigen::Vector2d navigation =
      navigation_gain_ * (Eigen::Vector2d(target_x_, target_y_) - own_position);

    Eigen::Vector2d cmd = separation + alignment + accel_feedforward + navigation;
    const double speed = cmd.norm();
    if (speed > max_horizontal_speed_) {
      cmd *= max_horizontal_speed_ / speed;
    }

    geometry_msgs::msg::Twist cmd_msg;
    cmd_msg.linear.x = cmd.x();
    cmd_msg.linear.y = cmd.y();
    cmd_msg.linear.z = pid_z_.update(target_z_, msg->pose.pose.position.z, dt);
    cmd_vel_pub_->publish(cmd_msg);
  }

  swarm_control::PidController pid_z_;

  std::vector<std::string> neighbor_names_;
  double interaction_range_, desired_spacing_;
  double separation_gain_, alignment_gain_, accel_feedforward_gain_, navigation_gain_;
  double accel_filter_k_;
  double max_horizontal_speed_;

  double target_x_, target_y_, target_z_;

  std::vector<NeighborEstimate> neighbors_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> neighbor_subs_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr own_odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_override_sub_;
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
