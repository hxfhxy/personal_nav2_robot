// 局部控制：DWA。
// 每个控制周期按当前速度开动态窗口，采样 (vx, wz)，各自向前滚一条 sim_time_ 长的轨迹；
// 撞墙的直接丢掉，剩下的按「离前方路径多远 + 朝向差多少 + 速度大小」打分，取最高的一组。
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
  plan_index_ = 0;      // 新路径从头对齐
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
  cmd.header.stamp = clock_->now();          // 仿真时钟
  cmd.header.frame_id = pose.header.frame_id;  // 跟当前位姿同一个坐标系
  return cmd;                                // twist 还是 0
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

void MyDWAController::advancePlanIndex(double x, double y)
{
  const auto & poses = global_plan_.poses;
  const size_t n = poses.size();
  if (n == 0) {
    return;
  }
  if (plan_index_ >= n) {
    plan_index_ = n - 1;
  }

  // 只在前方一小段里找最近点。整条路径最近会把车吸回身后，或吸到旁边走廊
  double best = std::hypot(
    poses[plan_index_].pose.position.x - x,
    poses[plan_index_].pose.position.y - y);
  size_t best_i = plan_index_;
  double walked = 0.0;
  for (size_t i = plan_index_ + 1; i < n && walked < 1.5; ++i) {
    const auto & prev = poses[i - 1].pose.position;
    const auto & cur = poses[i].pose.position;
    walked += std::hypot(cur.x - prev.x, cur.y - prev.y);
    const double d = std::hypot(cur.x - x, cur.y - y);
    if (d < best) {
      best = d;
      best_i = i;
    }
  }
  plan_index_ = best_i;
}

void MyDWAController::matchAhead(double x, double y, double & dist, double & yaw) const
{
  const auto & poses = global_plan_.poses;
  const size_t n = poses.size();
  dist = std::numeric_limits<double>::infinity();
  yaw = 0.0;
  if (n == 0) {
    return;
  }

  const size_t i0 = std::min(plan_index_, n - 1);
  if (i0 + 1 >= n) {
    dist = std::hypot(poses.back().pose.position.x - x, poses.back().pose.position.y - y);
    yaw = getYaw(poses.back().pose);  // 只剩终点，用目标朝向
    return;
  }

  double walked = 0.0;
  for (size_t i = i0; i + 1 < n && walked < 1.5; ++i) {
    const double ax = poses[i].pose.position.x;
    const double ay = poses[i].pose.position.y;
    const double bx = poses[i + 1].pose.position.x;
    const double by = poses[i + 1].pose.position.y;
    const double dx = bx - ax;
    const double dy = by - ay;
    const double len2 = dx * dx + dy * dy;
    walked += std::sqrt(len2);

    double t = 0.0;
    if (len2 > 1e-8) {
      t = std::clamp(((x - ax) * dx + (y - ay) * dy) / len2, 0.0, 1.0);  // 垂足落在线段上
    }
    const double d = std::hypot(x - (ax + t * dx), y - (ay + t * dy));
    if (d >= dist) {
      continue;
    }
    dist = d;
    if (i + 1 == n - 1 && t > 0.5) {
      yaw = getYaw(poses.back().pose);  // 贴到终点这一段，用目标朝向
    } else if (len2 > 1e-8) {
      yaw = std::atan2(dy, dx);  // 这段路径往哪指
    } else {
      yaw = getYaw(poses[i].pose);
    }
  }
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

  // 到点判断交给 goal_checker，这里不再自己算距离
  const auto & goal_pose = global_plan_.poses.back().pose;
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
  advancePlanIndex(start.x, start.y);  // 这一拍先对齐到前方路径，采样都用这个下标

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

  // vx、wz 各自均匀取样，两层循环就是全部候选
  const int nvx = std::max(1, vx_samples_);
  const int nwz = std::max(1, vtheta_samples_);
  double best_score = -std::numeric_limits<double>::infinity();
  bool found = false;
  const double vx_step = (nvx > 1) ? (vx_hi - vx_lo) / static_cast<double>(nvx - 1) : 0.0;  // 相邻采样间隔
  const double wz_step = (nwz > 1) ? (wz_hi - wz_lo) / static_cast<double>(nwz - 1) : 0.0;

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
      if (hit) continue;

      // 终点到前方路径的横向距离。不再和整条路径的顶点比，否则会左右来回追
      const auto & end = traj.back();
      double path_err = 0.0;
      double path_yaw = 0.0;
      matchAhead(end.x, end.y, path_err, path_yaw);
      const double heading_err = std::abs(normAngle(end.yaw - path_yaw));  // 终点朝向和路径差多少
      // 朝向用平方：小偏差还能修正，大角度跨过路径就亏。0.05*|wz| 让差不多的轨迹挑更直的
      const double score =
        -path_err - 0.5 * heading_err * heading_err - 0.05 * std::abs(wz) + 0.8 * vx;

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
