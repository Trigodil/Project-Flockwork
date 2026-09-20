// Distributed version of mean_field_controller_node.cpp. Not built, and not
// wired into CMakeLists.txt.
//
// Each drone computes its own Voronoi cell and centroid from neighbor
// broadcasts only, no central node, Consequently it is exact if "neighbors" covers everyone
// who could border your cell, approximate otherwise (Cortes et al 2004).
//
// To enable: remove #if 0/#endif, add to CMakeLists.txt, validate the
// local centroid matches the centralized version's for the same state.

#if 0

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point.hpp"

class DistributedDensityControllerNode : public rclcpp::Node
{
public:
  DistributedDensityControllerNode()
  : Node("distributed_density_controller")
  {
    own_name_ = declare_parameter("own_name", std::string("x3_0"));
    neighbor_names_ = declare_parameter("neighbor_names", std::vector<std::string>{});
    target_x_ = declare_parameter("target_x", std::vector<double>{-3.0, 3.0});
    target_y_ = declare_parameter("target_y", std::vector<double>{0.0, 0.0});
    target_weight_ = declare_parameter("target_weight", std::vector<double>{1.0, 1.0});
    target_sigma_ = declare_parameter("target_sigma", 1.5);
    domain_min_ = declare_parameter("domain_min", -6.0);
    domain_max_ = declare_parameter("domain_max", 6.0);
    grid_steps_ = declare_parameter("grid_steps", 40);
    update_period_sec_ = declare_parameter("update_period_sec", 0.5);

    have_own_position_ = false;

    own_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/" + own_name_ + "/odom", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        own_position_ = msg->pose.pose.position;
        have_own_position_ = true;
      });

    neighbor_positions_.assign(neighbor_names_.size(), geometry_msgs::msg::Point());
    have_neighbor_.assign(neighbor_names_.size(), false);
    for (size_t i = 0; i < neighbor_names_.size(); ++i) {
      neighbor_subs_.push_back(create_subscription<nav_msgs::msg::Odometry>(
        "/" + neighbor_names_[i] + "/odom", 10,
        [this, i](const nav_msgs::msg::Odometry::SharedPtr msg) {
          neighbor_positions_[i] = msg->pose.pose.position;
          have_neighbor_[i] = true;
        }));
    }

    target_pub_ = create_publisher<geometry_msgs::msg::Point>(
      "/" + own_name_ + "/target_override", 10);

    timer_ = create_wall_timer(
      std::chrono::duration<double>(update_period_sec_),
      std::bind(&DistributedDensityControllerNode::update, this));
  }

private:
  double targetDensity(double x, double y) const
  {
    double density = 0.0;
    for (size_t k = 0; k < target_x_.size(); ++k) {
      const double dx = x - target_x_[k];
      const double dy = y - target_y_[k];
      const double d2 = dx * dx + dy * dy;
      density += target_weight_[k] * std::exp(-d2 / (2.0 * target_sigma_ * target_sigma_));
    }
    return density;
  }

  void update()
  {
    if (!have_own_position_) {
      return;
    }
    for (bool ok : have_neighbor_) {
      if (!ok) {
        return;  // wait to hear from every known neighbor at least once
      }
    }

    double sum_x = 0.0, sum_y = 0.0, sum_w = 0.0;
    const double step = (domain_max_ - domain_min_) / grid_steps_;
    const double cell_area = step * step;

    for (int gi = 0; gi < grid_steps_; ++gi) {
      const double gx = domain_min_ + (gi + 0.5) * step;
      for (int gj = 0; gj < grid_steps_; ++gj) {
        const double gy = domain_min_ + (gj + 0.5) * step;

        // Is this cell ours? Only correct if neighbor_names_ covers
        // everyone who could actually border our cell, see file header.
        double own_d2 = std::pow(gx - own_position_.x, 2) + std::pow(gy - own_position_.y, 2);
        bool cell_is_ours = true;
        for (size_t i = 0; i < neighbor_positions_.size(); ++i) {
          const double dx = gx - neighbor_positions_[i].x;
          const double dy = gy - neighbor_positions_[i].y;
          if (dx * dx + dy * dy < own_d2) {
            cell_is_ours = false;
            break;
          }
        }
        if (!cell_is_ours) {
          continue;
        }

        const double mass = targetDensity(gx, gy) * cell_area;
        sum_x += gx * mass;
        sum_y += gy * mass;
        sum_w += mass;
      }
    }

    geometry_msgs::msg::Point centroid;
    if (sum_w > 1e-9) {
      centroid.x = sum_x / sum_w;
      centroid.y = sum_y / sum_w;
    } else {
      centroid.x = own_position_.x;
      centroid.y = own_position_.y;
    }
    target_pub_->publish(centroid);
  }

  std::string own_name_;
  std::vector<std::string> neighbor_names_;
  std::vector<double> target_x_, target_y_, target_weight_;
  double target_sigma_;
  double domain_min_, domain_max_;
  int grid_steps_;
  double update_period_sec_;

  geometry_msgs::msg::Point own_position_;
  bool have_own_position_;
  std::vector<geometry_msgs::msg::Point> neighbor_positions_;
  std::vector<bool> have_neighbor_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr own_odom_sub_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> neighbor_subs_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr target_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DistributedDensityControllerNode>());
  rclcpp::shutdown();
  return 0;
}

#endif  // if 0
