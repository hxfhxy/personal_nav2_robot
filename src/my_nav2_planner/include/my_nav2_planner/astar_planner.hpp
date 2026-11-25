// MyAStarPlanner：Nav2 全局规划插件（对应 yaml 里的 GridBased）。
// 声明生命周期回调、行为树调用的 createPlan，以及内部的栅格 A* 和可走性判断。

#ifndef MY_NAV2_PLANNER__ASTAR_PLANNER_HPP_
#define MY_NAV2_PLANNER__ASTAR_PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace my_nav2_planner
{

// Nav2 全局规划插件：在全局代价地图上用 A* 搜栅格路径，再转成 nav_msgs::Path。
class MyAStarPlanner : public nav2_core::GlobalPlanner
{
public:
  // 插件加载时调用：保存 costmap、坐标系、logger、clock。
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;     // 卸载 / 销毁前
  void activate() override;    // 生命周期进入 active
  void deactivate() override;  // 生命周期退出 active

  // BT 的 ComputePathToPose 会调这里：世界坐标起终点 → Path。
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};  // 全局代价地图（栅格）
  std::string global_frame_;                       // 一般是 "map"
  std::string name_;                               // 插件实例名，对应 yaml 里 GridBased
  rclcpp::Logger logger_{rclcpp::get_logger("MyAStarPlanner")};
  rclcpp::Clock::SharedPtr clock_;

  // 栅格 A*：输入起终点格子，输出从起点到终点的格子序列；失败返回空。
  std::vector<std::pair<unsigned int, unsigned int>> searchAstar(
  unsigned int start_x, unsigned int start_y,
  unsigned int goal_x, unsigned int goal_y);

  // 格子是否不可走：越界、致命障碍、膨胀层都算挡住。未知格子暂当可走。
  bool isBlocked(unsigned int mx, unsigned int my) const;

};

}  // namespace my_nav2_planner
#endif
