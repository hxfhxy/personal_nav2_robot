#include "my_nav2_controller/dwa_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"

namespace my_nav2_controller
{

void MyDWAController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros->getCostmap();

  auto node = parent.lock();
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  auto declare = [&](const std::string & key, auto value) {
      nav2_util::declare_parameter_if_not_declared(node, name + "." + key, rclcpp::ParameterValue(value));
    };
  declare("max_vel_x", 0.5);
  declare("min_vel_x", 0.0);
  declare("max_vel_theta", 1.0);
  declare("acc_lim_x", 2.5);
  declare("acc_lim_theta", 3.2);
  declare("sim_time", 1.7);
  declare("sim_granularity", 0.05);
  declare("vx_samples", 20);
  declare("vtheta_samples", 20);
  declare("lethal_cost", 253);

  node->get_parameter(name + ".max_vel_x", max_vel_x_);
  node->get_parameter(name + ".min_vel_x", min_vel_x_);
  node->get_parameter(name + ".max_vel_theta", max_vel_theta_);
  node->get_parameter(name + ".acc_lim_x", acc_lim_x_);
  node->get_parameter(name + ".acc_lim_theta", acc_lim_theta_);
  node->get_parameter(name + ".sim_time", sim_time_);
  node->get_parameter(name + ".sim_granularity", sim_granularity_);
  node->get_parameter(name + ".vx_samples", vx_samples_);
  node->get_parameter(name + ".vtheta_samples", vtheta_samples_);
  int lethal_cost = 253;
  node->get_parameter(name + ".lethal_cost", lethal_cost);
  lethal_cost_ = static_cast<unsigned int>(std::max(0, std::min(lethal_cost, 254)));
  speed_limit_ = max_vel_x_;

  RCLCPP_INFO(logger_, "Configuring plugin %s of type MyDWAController", name_.c_str());
}

void MyDWAController::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up plugin %s of type MyDWAController", name_.c_str());
  costmap_ = nullptr;
}

void MyDWAController::activate()
{
  RCLCPP_INFO(logger_, "Activating plugin %s of type MyDWAController", name_.c_str());
}

void MyDWAController::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating plugin %s of type MyDWAController", name_.c_str());
}

void MyDWAController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
}

void MyDWAController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (percentage) {
    speed_limit_ = max_vel_x_ * speed_limit / 100.0;
  } else {
    speed_limit_ = speed_limit;
  }
}

geometry_msgs::msg::TwistStamped MyDWAController::makeCmd(
  const geometry_msgs::msg::PoseStamped & pose) const
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = clock_->now();
  cmd.header.frame_id = pose.header.frame_id;
  return cmd;
}

double MyDWAController::getYaw(const geometry_msgs::msg::Pose & pose)
{
  return tf2::getYaw(pose.orientation);
}

//判断给定世界坐标点是否在局部 costmap 的障碍物里
bool MyDWAController::inCollision(double wx, double wy) const
{
  unsigned int mx = 0, my = 0;
  if (!costmap_->worldToMap(wx, wy, mx, my)) {
    return true;
  }
  const unsigned char cost = costmap_->getCost(mx, my);
  if (cost == nav2_costmap_2d::NO_INFORMATION) {
    return false;
  }
  return cost >= lethal_cost_;
}

//给定一组候选速度，仿真轨迹，碰撞检测，打分，返回最优速度
std::vector<Pose2D> MyDWAController::projectTrajectory(
  const Pose2D & start, double vx, double wz) const
{
  std::vector<Pose2D> traj;
  const int steps = std::max(1, static_cast<int>(std::ceil(sim_time_ / sim_granularity_)));
  const double dt = sim_time_ / static_cast<double>(steps);
  Pose2D p = start;
  traj.reserve(static_cast<size_t>(steps) + 1);
  traj.push_back(p);
  for (int i = 0; i < steps; ++i) {
    p.x += vx * std::cos(p.yaw) * dt;
    p.y += vx * std::sin(p.yaw) * dt;
    p.yaw += wz * dt;
    traj.push_back(p);
  }
  return traj;
}

geometry_msgs::msg::TwistStamped MyDWAController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  auto cmd = makeCmd(pose);

  if (global_plan_.poses.empty()) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "Empty global plan, commanding zero velocity");
    return cmd;
  }

  const auto & goal_pose = global_plan_.poses.back().pose;
  if (goal_checker != nullptr &&
    goal_checker->isGoalReached(pose.pose, goal_pose, velocity))
  {
    return cmd;
  }

  cmd.twist = computeBestTwist(pose, velocity);
  return cmd;
}

geometry_msgs::msg::Twist MyDWAController::computeBestTwist(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity)
{
  geometry_msgs::msg::Twist best;//默认0

  Pose2D start;
  start.x = pose.pose.position.x;
  start.y = pose.pose.position.y;
  start.yaw = getYaw(pose.pose);

  //算动态窗口
  const double dt = 0.2;
  const double vx_hi = std::min({max_vel_x_, speed_limit_,
                               velocity.linear.x + acc_lim_x_ * dt});//线速度上边界
  const double vx_lo = std::max(min_vel_x_,
                                velocity.linear.x - acc_lim_x_ * dt);//线速度下边界
  const double wz_hi = std::min(max_vel_theta_,
                                velocity.angular.z + acc_lim_theta_ * dt);//角速度上边界
  const double wz_lo = std::max(-max_vel_theta_,
                                velocity.angular.z - acc_lim_theta_ * dt);//角速度下边界
  if (vx_lo > vx_hi || wz_lo > wz_hi) {
    return best;
  }

  //两层循环采样
  const int nvx = std::max(1, vx_samples_);
  const int nwz = std::max(1, vtheta_samples_);
  double best_score = -std::numeric_limits<double>::infinity();
  bool found = false;
  const double vx_step = (nvx > 1) ? (vx_hi - vx_lo) / static_cast<double>(nvx - 1) : 0.0;
  const double wz_step = (nwz > 1) ? (wz_hi - wz_lo) / static_cast<double>(nwz - 1) : 0.0;

  for (int i = 0; i < nvx; ++i) {
    const double vx = vx_lo + vx_step * static_cast<double>(i);
    for (int j = 0; j < nwz; ++j) {
      const double wz = wz_lo + wz_step * static_cast<double>(j);

      const auto traj = projectTrajectory(start, vx, wz);
      bool hit = false;
      for (const auto & p : traj) {
        if (inCollision(p.x, p.y)) {
          hit = true;
          break;
        }
      }
      if (hit) continue;

      const auto & end = traj.back();
      double path_err = std::numeric_limits<double>::infinity();
      for (const auto & ps : global_plan_.poses) {
        path_err = std::min(
          path_err,
          std::hypot(ps.pose.position.x - end.x,
                     ps.pose.position.y - end.y));
      }
      const double score = -path_err + 0.2 * vx;  // 越大越好

      if (!found || score > best_score) {
        best_score = score;
        found = true;
        best.linear.x = vx;
        best.angular.z = wz;
      }
    }
  }

  if (!found) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "DWA found no valid trajectory");
  }
  return best;
}

}  // namespace my_nav2_controller

PLUGINLIB_EXPORT_CLASS(my_nav2_controller::MyDWAController, nav2_core::Controller)
