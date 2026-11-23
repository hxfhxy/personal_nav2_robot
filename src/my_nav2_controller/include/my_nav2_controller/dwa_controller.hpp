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

// Nav2 局部控制插件：在动态窗口里采样速度，滚轨迹打分，选出这一拍的 cmd_vel。
class MyDWAController : public nav2_core::Controller
{
public:
  MyDWAController() = default;
  ~MyDWAController() override = default;

  // 插件加载时调用：保存局部 costmap，读速度和采样参数。
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;     // 卸载 / 销毁前
  void activate() override;    // 生命周期进入 active
  void deactivate() override;  // 生命周期退出 active
  void setPlan(const nav_msgs::msg::Path & path) override;  // FollowPath 把全局路径交过来
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;  // 外部限速

  // BT 的 FollowPath 每个控制周期调这里：当前位姿 + 当前速度 → Twist。
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

protected:
  geometry_msgs::msg::TwistStamped makeCmd(const geometry_msgs::msg::PoseStamped & pose) const;  // 零速度壳子
  static double getYaw(const geometry_msgs::msg::Pose & pose);  // 四元数转航向角
  // 点是否不可走：越界、代价 >= lethal_cost_ 都算撞上。未知格子暂当可走。
  bool inCollision(double wx, double wy) const;
  static double normAngle(double angle);  // 收到 (-pi, pi]
  // 车走到哪了。只在 plan_index_ 往前约 1.5 m 里找，避免吸到身后或另一条走廊
  void advancePlanIndex(double x, double y);
  // 点到前方折线的距离，以及这段路径的朝向
  void matchAhead(double x, double y, double & dist, double & yaw) const;
  // vx、wz 不变，向前滚 sim_time_ 秒，返回轨迹点。
  std::vector<Pose2D> projectTrajectory(
    const Pose2D & start, double vx, double wz) const;

  // 采样、仿真、打分。一条都走不了就返回全零。
  geometry_msgs::msg::Twist computeBestTwist(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};  // 局部代价地图（栅格）
  std::string name_;                               // 插件实例名，对应 yaml 里 FollowPath
  nav_msgs::msg::Path global_plan_;                // 当前要跟的全局路径
  size_t plan_index_{0};                           // 车上一次对齐到的路径点，只往前走
  rclcpp::Logger logger_{rclcpp::get_logger("MyDWAController")};
  rclcpp::Clock::SharedPtr clock_;

  double max_vel_x_{0.5};          // 前进速度上限，m/s
  double min_vel_x_{0.0};          // 不后退
  double max_vel_theta_{1.0};      // 角速度上限，rad/s
  double acc_lim_x_{2.5};          // 线加速度，收动态窗口用
  double acc_lim_theta_{3.2};      // 角加速度
  double sim_time_{0.8};           // 每条轨迹向前看多久。太长时终点离开弯道，高速候选被丢掉
  double sim_granularity_{0.05};   // 轨迹点的时间间隔，秒
  int vx_samples_{20};             // 线速度采样数
  int vtheta_samples_{20};         // 角速度采样数
  unsigned int lethal_cost_{253};  // >=253 当墙
  double speed_limit_{0.5};        // 外部限速，和 max_vel_x_ 一起卡上沿
};

}  // namespace my_nav2_controller

#endif  // MY_NAV2_CONTROLLER__DWA_CONTROLLER_HPP_
