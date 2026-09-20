#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point.hpp"

// Centralized mean-field density control, Lloyd's algorithm / Cortes et al.
// coverage control. This treats the swarm as a collective: builds a target
// density function, partitions space into a Voronoi diagram from current
// drone positions, and moves each drone toward its cell's density-weighted
// centroid(Kinda like fluid dynamics). This node only decides WHERE each drone should go, the
// individual PID in single_drone_controller_node still decides HOW to get
// there, unchanged. Global info is secondary input to local control.
class MeanFieldControllerNode : public rclcpp::Node
{
public:
  MeanFieldControllerNode()
  : Node("mean_field_controller")
  {
    drone_names_ = declare_parameter(
      "drone_names", std::vector<std::string>{"x3_0", "x3_1", "x3_2"});
    target_x_ = declare_parameter("target_x", std::vector<double>{-3.0, 3.0});
    target_y_ = declare_parameter("target_y", std::vector<double>{0.0, 0.0});
    target_weight_ = declare_parameter("target_weight", std::vector<double>{1.0, 1.0});
    target_sigma_ = declare_parameter("target_sigma", 1.5);
    domain_min_ = declare_parameter("domain_min", -6.0);
    domain_max_ = declare_parameter("domain_max", 6.0);
    grid_steps_ = declare_parameter("grid_steps", 40);
    update_period_sec_ = declare_parameter("update_period_sec", 0.5);
    orbit_speed_ = declare_parameter("orbit_speed", 0.0);  // rad/s, 0 = static targets

    start_time_ = now();

    const size_t n = drone_names_.size();
    positions_.assign(n, geometry_msgs::msg::Point());
    have_position_.assign(n, false);

    for (size_t i = 0; i < n; ++i) {
      odom_subs_.push_back(create_subscription<nav_msgs::msg::Odometry>(
        "/" + drone_names_[i] + "/odom", 10,
        [this, i](const nav_msgs::msg::Odometry::SharedPtr msg) {
          positions_[i] = msg->pose.pose.position;
          have_position_[i] = true;
        }));
      target_pubs_.push_back(create_publisher<geometry_msgs::msg::Point>(
        "/" + drone_names_[i] + "/target_override", 10));
    }

    // Slow, global layer. Does not need to run every control tick, the
    // target density is not changing at flocking speed.
    timer_ = create_wall_timer(
      std::chrono::duration<double>(update_period_sec_),
      std::bind(&MeanFieldControllerNode::update, this));

    RCLCPP_INFO(get_logger(), "mean_field_controller tracking %zu drones", n);
  }

private:
  double targetDensity(double x, double y) const
  {
    double density = 0.0;
    for (size_t k = 0; k < rotated_x_.size(); ++k) {
      const double dx = x - rotated_x_[k];
      const double dy = y - rotated_y_[k];
      const double d2 = dx * dx + dy * dy;
      density += target_weight_[k] * std::exp(-d2 / (2.0 * target_sigma_ * target_sigma_));
    }
    return density;
  }

  // Rotates each target blob's base position around the domain center, so
  // the swarm chases a moving formation instead of a static one when
  // orbit_speed is nonzero.
  void updateRotatedTargets()
  {
    const double angle = orbit_speed_ * (now() - start_time_).seconds();
    const double c = std::cos(angle), s = std::sin(angle);
    rotated_x_.resize(target_x_.size());
    rotated_y_.resize(target_y_.size());
    for (size_t k = 0; k < target_x_.size(); ++k) {
      rotated_x_[k] = target_x_[k] * c - target_y_[k] * s;
      rotated_y_[k] = target_x_[k] * s + target_y_[k] * c;
    }
  }

  void update()
  {
    const size_t n = drone_names_.size();
    for (size_t i = 0; i < n; ++i) {
      if (!have_position_[i]) {
        return;  // wait for at least one odom reading from every drone
      }
    }

    updateRotatedTargets();
    std::vector<double> sum_x(n, 0.0), sum_y(n, 0.0), sum_w(n, 0.0);

    const double step = (domain_max_ - domain_min_) / grid_steps_;
    const double cell_area = step * step;

    for (int gi = 0; gi < grid_steps_; ++gi) {
      const double gx = domain_min_ + (gi + 0.5) * step;
      for (int gj = 0; gj < grid_steps_; ++gj) {
        const double gy = domain_min_ + (gj + 0.5) * step;

        // Nearest drone owns this cell, that is the Voronoi partition.
        size_t nearest = 0;
        double best_d2 = std::numeric_limits<double>::max();
        for (size_t i = 0; i < n; ++i) {
          const double dx = gx - positions_[i].x;
          const double dy = gy - positions_[i].y;
          const double d2 = dx * dx + dy * dy;
          if (d2 < best_d2) {
            best_d2 = d2;
            nearest = i;
          }
        }

        const double mass = targetDensity(gx, gy) * cell_area;
        sum_x[nearest] += gx * mass;
        sum_y[nearest] += gy * mass;
        sum_w[nearest] += mass;
      }
    }

    for (size_t i = 0; i < n; ++i) {
      geometry_msgs::msg::Point centroid;
      if (sum_w[i] > 1e-9) {
        centroid.x = sum_x[i] / sum_w[i];
        centroid.y = sum_y[i] / sum_w[i];
      } else {
        centroid.x = positions_[i].x;  // no target mass in this cell, hold
        centroid.y = positions_[i].y;
      }
      target_pubs_[i]->publish(centroid);
    }
  }

  std::vector<std::string> drone_names_;
  std::vector<double> target_x_, target_y_, target_weight_;
  std::vector<double> rotated_x_, rotated_y_;
  double target_sigma_;
  double domain_min_, domain_max_;
  int grid_steps_;
  double update_period_sec_;
  double orbit_speed_;
  rclcpp::Time start_time_;

  std::vector<geometry_msgs::msg::Point> positions_;
  std::vector<bool> have_position_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subs_;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr> target_pubs_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MeanFieldControllerNode>());
  rclcpp::shutdown();
  return 0;
}
