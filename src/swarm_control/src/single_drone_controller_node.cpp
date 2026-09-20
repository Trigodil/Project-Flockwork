#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "swarm_control/pid_controller.hpp"

// Stage 1: hold a drone at a fixed setpoint with three PID loops (x, y, z),
// commanding velocity instead of raw rotor thrust for now :sideye:.
class SingleDroneControllerNode : public rclcpp::Node
{
public:
  SingleDroneControllerNode()
  : Node("single_drone_controller"),
    pid_x_(1.0, 0.0, 0.3, 1.0),
    pid_y_(1.0, 0.0, 0.3, 1.0),
    pid_z_(1.5, 0.1, 0.4, 1.0)
  {
    target_x_ = declare_parameter("target_x", 0.0);
    target_y_ = declare_parameter("target_y", 0.0);
    target_z_ = declare_parameter("target_z", 2.0);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10,
      std::bind(&SingleDroneControllerNode::odom_callback, this, std::placeholders::_1));

    // Global layer (e.g. mean_field_controller_node) can override the x,y
    // target at runtime. Individual PID control stays exactly as-is, this
    // just changes what it chases.
    target_override_sub_ = create_subscription<geometry_msgs::msg::Point>(
      "target_override", 10,
      [this](const geometry_msgs::msg::Point::SharedPtr msg) {
        target_x_ = msg->x;
        target_y_ = msg->y;
      });

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    last_time_ = now();

    RCLCPP_INFO(
      get_logger(), "single_drone_controller started, target=(%.2f, %.2f, %.2f)",
      target_x_, target_y_, target_z_);
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const rclcpp::Time current_time = now();
    const double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

    if (dt <= 0.0) {
      return;  // guard against the first callback or a clock jump.
    }

    const auto & p = msg->pose.pose.position;

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = pid_x_.update(target_x_, p.x, dt);
    cmd.linear.y = pid_y_.update(target_y_, p.y, dt);
    cmd.linear.z = pid_z_.update(target_z_, p.z, dt);

    cmd_vel_pub_->publish(cmd);
  }

  swarm_control::PidController pid_x_;
  swarm_control::PidController pid_y_;
  swarm_control::PidController pid_z_;

  double target_x_;
  double target_y_;
  double target_z_;

  rclcpp::Time last_time_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_override_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SingleDroneControllerNode>());
  rclcpp::shutdown();
  return 0;
}
