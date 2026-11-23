#ifndef MY_NAV2_SMOOTHER__QP_SMOOTHER_HPP_
#define MY_NAV2_SMOOTHER__QP_SMOOTHER_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "nav2_core/smoother.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace my_nav2_smoother
{

struct Point2D
{
  double x{0.0};
  double y{0.0};
};

// Nav2 路径平滑插件：QP 求出控制点，再用三次 B 样条画成曲线。起点和终点固定。
class MyQpSmoother : public nav2_core::Smoother
{
public:
  MyQpSmoother() = default;
  ~MyQpSmoother() override = default;

  // 插件加载时调用：读平滑权重、贴合权重和采样间距。
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub,
    std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub) override;
  void cleanup() override;     // 卸载 / 销毁前
  void activate() override;    // 生命周期进入 active
  void deactivate() override;  // 生命周期退出 active

  // BT 的 SmoothPath 会调这里：原地改 path，做完返回 true。
  bool smooth(
    nav_msgs::msg::Path & path,
    const rclcpp::Duration & max_time) override;

private:
  // 一维 QP。ref 含固定的首尾，成功时 out 与 ref 等长。
  bool solveAxis(const std::vector<double> & ref, std::vector<double> & out) const;
  // 高斯消元解 H y = rhs。失败返回 false。
  bool solveLinear(
    std::vector<std::vector<double>> h,
    std::vector<double> rhs,
    std::vector<double> & y) const;

  // 线段是否扫到墙：越界、代价 >= lethal_cost_ 都算挡住。未知格子暂当可走。
  bool segmentBlocked(
    const nav2_costmap_2d::Costmap2D * costmap,
    double x0, double y0, double x1, double y1) const;
  // 折线任一段撞上就返回 true。
  bool polylineBlocked(
    const nav2_costmap_2d::Costmap2D * costmap,
    const std::vector<Point2D> & points) const;
  // 在原路径和 QP 控制点之间取最靠 QP、曲线又不进墙的一份。
  std::vector<Point2D> blend(
    const nav2_costmap_2d::Costmap2D * costmap,
    const std::vector<Point2D> & smoothed,
    const std::vector<Point2D> & reference) const;
  // 按 sample_distance_ 沿折线重采样。最后一个点保持终点。
  std::vector<Point2D> resample(const std::vector<Point2D> & points) const;
  // 控制点少于 4 个时退回折线。否则画夹紧的三次 B 样条，再按间距取样。
  std::vector<Point2D> sampleCurve(const std::vector<Point2D> & controls) const;
  // 曲线采样点连成的折线是否扫到墙。
  bool curveBlocked(
    const nav2_costmap_2d::Costmap2D * costmap,
    const std::vector<Point2D> & controls) const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
  std::string name_;  // 插件实例名，对应 yaml 里 qp_smoother
  rclcpp::Logger logger_{rclcpp::get_logger("MyQpSmoother")};
  rclcpp::Clock::SharedPtr clock_;

  double smooth_weight_{10.0};     // 二阶差分的权重，越大越想拉直
  double reference_weight_{1.0};   // 贴近 A* 点的权重
  double sample_distance_{0.1};    // 输出点间距，米
  unsigned int lethal_cost_{253};  // >=253 当墙
};

}  // namespace my_nav2_smoother

#endif  // MY_NAV2_SMOOTHER__QP_SMOOTHER_HPP_
