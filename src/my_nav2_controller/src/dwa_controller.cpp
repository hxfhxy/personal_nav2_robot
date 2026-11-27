// 局部控制：基础 DWA。
// 一个控制周期里：按当前速度开动态窗口 → 采样 (vx, wz) → 各自向前滚一条 sim_time_ 长的轨迹
// → 撞到致命格的丢掉 → 剩下的打分：离路径多远、离终点多远、朝向差多少、速度大小。
// 只有这四项，权重都在 yaml 的 FollowPath 下面。
#include "my_nav2_controller/dwa_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_core/exceptions.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
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
  costmap_ = costmap_ros->getCostmap();  // 真正的二维栅格，碰撞检查用这块

  auto node = parent.lock();
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  // 参数名带插件实例前缀，对应 yaml 里 FollowPath 下面的键
  auto declare = [&](const std::string & key, auto value) {
      nav2_util::declare_parameter_if_not_declared(node, name + "." + key, rclcpp::ParameterValue(value));
    };
  declare("max_vel_x", 0.5);
  declare("min_vel_x", 0.0);
  declare("max_vel_theta", 1.0);
  declare("acc_lim_x", 2.5);
  declare("acc_lim_theta", 3.2);
  declare("sim_time", 0.8);
  declare("sim_granularity", 0.05);
  declare("vx_samples", 20);
  declare("vtheta_samples", 20);
  declare("lethal_cost", 253);
  declare("goal_tolerance", 0.25);
  declare("goal_yaw_tolerance", 0.10);
  declare("path_weight", 1.0);
  declare("goal_weight", 1.0);
  declare("heading_weight", 0.5);
  declare("velocity_weight", 0.2);

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
  node->get_parameter(name + ".goal_tolerance", goal_tolerance_);
  node->get_parameter(name + ".goal_yaw_tolerance", goal_yaw_tolerance_);
  node->get_parameter(name + ".path_weight", path_weight_);
  node->get_parameter(name + ".goal_weight", goal_weight_);
  node->get_parameter(name + ".heading_weight", heading_weight_);
  node->get_parameter(name + ".velocity_weight", velocity_weight_);
  speed_limit_ = max_vel_x_;  // 没收到外部限速前，上沿就是最大线速度

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
  global_plan_ = path;  // 整段换掉，控制器自己不改这条路径
}

void MyDWAController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (percentage) {
    speed_limit_ = max_vel_x_ * speed_limit / 100.0;  // 这时传进来的是百分比，不是 m/s
  } else {
    speed_limit_ = speed_limit;
  }
}

geometry_msgs::msg::TwistStamped MyDWAController::makeCmd(
  const geometry_msgs::msg::PoseStamped & pose) const
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = clock_->now();            // 仿真时钟
  cmd.header.frame_id = pose.header.frame_id;  // 跟当前位姿同一个坐标系
  return cmd;                                  // twist 还是 0
}

double MyDWAController::getYaw(const geometry_msgs::msg::Pose & pose)
{
  return tf2::getYaw(pose.orientation);
}

double MyDWAController::normAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

// 整条路径逐段投影，取最近的一段。路径不长（几十个点），每周期全扫一遍没问题。
MyDWAController::PathMatch MyDWAController::matchPath(double x, double y) const
{
  PathMatch match;
  match.distance = std::numeric_limits<double>::infinity();

  const auto & poses = local_plan_.poses;
  if (poses.empty()) {
    return match;
  }
  if (poses.size() == 1) {
    match.distance = std::hypot(poses[0].pose.position.x - x, poses[0].pose.position.y - y);
    match.yaw = getYaw(poses[0].pose);
    return match;
  }

  for (size_t i = 0; i + 1 < poses.size(); ++i) {
    const double ax = poses[i].pose.position.x;
    const double ay = poses[i].pose.position.y;
    const double bx = poses[i + 1].pose.position.x;
    const double by = poses[i + 1].pose.position.y;
    const double dx = bx - ax;
    const double dy = by - ay;
    const double len2 = dx * dx + dy * dy;

    double t = 0.0;
    if (len2 > 1e-8) {
      t = std::clamp(((x - ax) * dx + (y - ay) * dy) / len2, 0.0, 1.0);  // 垂足落在线段上
    }
    const double distance = std::hypot(x - (ax + t * dx), y - (ay + t * dy));
    if (distance >= match.distance) {
      continue;
    }

    match.distance = distance;
    if (i + 2 == poses.size() && t > 0.5) {
      match.yaw = getYaw(poses.back().pose);  // 已经贴到最后一段的前半，按目标朝向要求车头
    } else if (len2 > 1e-8) {
      match.yaw = std::atan2(dy, dx);  // 这段路径往哪指
    } else {
      match.yaw = getYaw(poses[i].pose);  // 重合点
    }
  }
  return match;
}

// 返回 true = 这个点不能走。
bool MyDWAController::inCollision(double wx, double wy) const
{
  unsigned int mx = 0, my = 0;
  if (!costmap_->worldToMap(wx, wy, mx, my)) {
    return true;  // 局部地图外面
  }
  const unsigned char cost = costmap_->getCost(mx, my);
  if (cost == nav2_costmap_2d::NO_INFORMATION) {
    return false;  // 未知区先当可走，否则没扫到的地方车会停住
  }
  // 253=贴身膨胀，254=致命障碍，都当撞上。只查中心这一点
  return cost >= lethal_cost_;
}

// 差速模型向前滚。这一段里 vx、wz 保持不变。
std::vector<Pose2D> MyDWAController::projectTrajectory(
  const Pose2D & start, double vx, double wz) const
{
  std::vector<Pose2D> traj;
  // sim_granularity_ 是时间步长。步数至少 1，避免 sim_time_ 比步长还短
  const int steps = std::max(1, static_cast<int>(std::ceil(sim_time_ / sim_granularity_)));
  const double dt = sim_time_ / static_cast<double>(steps);
  Pose2D p = start;
  traj.reserve(static_cast<size_t>(steps) + 1);
  traj.push_back(p);  // 起点也放进去，第 0 步就能查出已经在障碍里
  for (int i = 0; i < steps; ++i) {
    p.x += vx * std::cos(p.yaw) * dt;  // 沿当前航向走
    p.y += vx * std::sin(p.yaw) * dt;
    p.yaw += wz * dt;                   // 航向再加上角速度
    traj.push_back(p);
  }
  return traj;
}

geometry_msgs::msg::TwistStamped MyDWAController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  auto cmd = makeCmd(pose);  // 先是零速度，后面只改 twist

  if (global_plan_.poses.empty()) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "Empty global plan, commanding zero velocity");
    return cmd;  // 没有路径，停车
  }

  // AMCL 的 map -> odom 会变化，每周期重新变换整条路径，不能直接比较 map 和 odom 数值。
  local_plan_ = global_plan_;
  const auto target_frame = costmap_ros_->getGlobalFrameID();
  if (pose.header.frame_id != target_frame) {
    throw nav2_core::PlannerException("DWA robot pose is not in the local costmap frame");
  }
  if (global_plan_.header.frame_id != target_frame) {
    try {
      const auto transform = tf_->lookupTransform(
        target_frame, global_plan_.header.frame_id, tf2::TimePointZero);
      for (auto & point : local_plan_.poses) {
        geometry_msgs::msg::PoseStamped transformed;
        tf2::doTransform(point, transformed, transform);
        point = transformed;
      }
      local_plan_.header.frame_id = target_frame;
    } catch (const tf2::TransformException & ex) {
      throw nav2_core::PlannerException(std::string("DWA path transform failed: ") + ex.what());
    }
  }
  // 到点判断交给 goal_checker，这里不再自己算距离
  const auto & goal_pose = local_plan_.poses.back().pose;
  if (goal_checker != nullptr &&
    goal_checker->isGoalReached(pose.pose, goal_pose, velocity))
  {
    return cmd;  // 已经到了，停车
  }

  cmd.twist = computeBestTwist(pose, velocity);
  return cmd;
}

geometry_msgs::msg::Twist MyDWAController::computeBestTwist(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity)
{
  geometry_msgs::msg::Twist best;  // 默认全 0。一条都选不出来就停车

  Pose2D start;
  start.x = pose.pose.position.x;
  start.y = pose.pose.position.y;
  start.yaw = getYaw(pose.pose);

  // 位置已经进到点容差：只原地转到目标朝向，不再平移。
  // 收尾不参与打分。靠权重去权衡的话，车会在容差边界上慢慢逼近却永远差一点，
  // 表现出来就是在终点附近来回蹭或者转不停。
  const auto & goal_pose = local_plan_.poses.back().pose;
  const double goal_distance =
    std::hypot(goal_pose.position.x - start.x, goal_pose.position.y - start.y);
  if (goal_distance <= goal_tolerance_) {
    const double yaw_error = normAngle(getYaw(goal_pose) - start.yaw);
    if (std::abs(yaw_error) <= goal_yaw_tolerance_) {
      return best;  // 位置和朝向都够了，交给 goal_checker 收尾
    }
    const double wz_hi = std::min(max_vel_theta_, velocity.angular.z + acc_lim_theta_ * 0.2);
    const double wz_lo = std::max(-max_vel_theta_, velocity.angular.z - acc_lim_theta_ * 0.2);
    best.angular.z = std::clamp(2.0 * yaw_error, wz_lo, wz_hi);  // 仍在动态窗口内加速
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_, 1000, "At goal, turning %.2f rad in place", yaw_error);
    return best;
  }

  // 动态窗口：当前速度 ± 加速度*dt，再卡在最大速度里
  const double dt = 0.2;  // 只用来收窗口，不是控制周期
  const double vx_hi = std::min({max_vel_x_, speed_limit_,
                               velocity.linear.x + acc_lim_x_ * dt});  // 线速度上边界
  const double vx_lo = std::max(min_vel_x_,
                                velocity.linear.x - acc_lim_x_ * dt);  // 线速度下边界
  const double wz_hi = std::min(max_vel_theta_,
                                velocity.angular.z + acc_lim_theta_ * dt);  // 角速度上边界
  const double wz_lo = std::max(-max_vel_theta_,
                                velocity.angular.z - acc_lim_theta_ * dt);  // 角速度下边界
  if (vx_lo > vx_hi || wz_lo > wz_hi) {
    return best;  // 窗口是空的
  }

  const auto & goal = local_plan_.poses.back().pose.position;
  const int nvx = std::max(1, vx_samples_);
  const int nwz = std::max(1, vtheta_samples_);
  const double vx_step = (nvx > 1) ? (vx_hi - vx_lo) / static_cast<double>(nvx - 1) : 0.0;  // 相邻采样间隔
  const double wz_step = (nwz > 1) ? (wz_hi - wz_lo) / static_cast<double>(nwz - 1) : 0.0;

  double best_score = -std::numeric_limits<double>::infinity();
  bool found = false;

  for (int i = 0; i < nvx; ++i) {
    const double vx = vx_lo + vx_step * static_cast<double>(i);
    for (int j = 0; j < nwz; ++j) {
      const double wz = wz_lo + wz_step * static_cast<double>(j);

      const auto traj = projectTrajectory(start, vx, wz);
      bool hit = false;
      for (const auto & p : traj) {
        if (inCollision(p.x, p.y)) {
          hit = true;  // 任意一个点撞上，整条丢掉
          break;
        }
      }
      if (hit) {
        continue;
      }

      // 只对轨迹末端打分：离路径多远、离终点多远、朝向差多少、以及速度大小
      const auto & end = traj.back();
      const auto match = matchPath(end.x, end.y);
      const double goal_distance = std::hypot(goal.x - end.x, goal.y - end.y);
      const double heading_error = std::abs(normAngle(end.yaw - match.yaw));
      const double score =
        -path_weight_ * match.distance -
        goal_weight_ * goal_distance -
        heading_weight_ * heading_error * heading_error +
        velocity_weight_ * vx;

      if (!found || score > best_score) {
        best_score = score;
        found = true;
        best.linear.x = vx;   // 留下这一对速度
        best.angular.z = wz;
      }
    }
  }

  if (!found) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "DWA found no valid trajectory");
    // best 还是 0
  }
  return best;
}

}  // namespace my_nav2_controller

PLUGINLIB_EXPORT_CLASS(my_nav2_controller::MyDWAController, nav2_core::Controller)
