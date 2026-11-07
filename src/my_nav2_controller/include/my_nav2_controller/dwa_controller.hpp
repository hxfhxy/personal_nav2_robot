#ifndef MY_NAV2_CONTROLLER__DWA_CONTROLLER_HPP_
#define MY_NAV2_CONTROLLER__DWA_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace my_nav2_controller
{

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

class MyDWAController : public nav2_core::Controller
{
public:
  MyDWAController() = default;
  ~MyDWAController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path & path) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

protected:
  geometry_msgs::msg::TwistStamped makeCmd(const geometry_msgs::msg::PoseStamped & pose) const;
  static double getYaw(const geometry_msgs::msg::Pose & pose);
  bool inCollision(double wx, double wy) const;
  std::vector<Pose2D> projectTrajectory(
    const Pose2D & start, double vx, double wz) const;

  // 在 computeBestTwist() 里写 DWA：采样速度、仿真轨迹、打分、选最优。
  geometry_msgs::msg::Twist computeBestTwist(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  std::string name_;
  nav_msgs::msg::Path global_plan_;
  rclcpp::Logger logger_{rclcpp::get_logger("MyDWAController")};
  rclcpp::Clock::SharedPtr clock_;

  double max_vel_x_{0.5};
  double min_vel_x_{0.0};
  double max_vel_theta_{1.0};
  double acc_lim_x_{2.5};
  double acc_lim_theta_{3.2};
  double sim_time_{1.7};
  double sim_granularity_{0.05};
  int vx_samples_{20};
  int vtheta_samples_{20};
  unsigned int lethal_cost_{253};
  double speed_limit_{0.5};
};

}  // namespace my_nav2_controller

#endif  // MY_NAV2_CONTROLLER__DWA_CONTROLLER_HPP_
