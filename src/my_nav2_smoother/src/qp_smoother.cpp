// 路径平滑：一维 QP + 三次 B 样条。
// 未知数是折线上的中间点（首尾固定），代价 = 贴近原 A* 点 + 相邻三点的二阶差分。
// 两项都是二次的，正规方程化成线性方程组，用高斯消元解。解出的点当 B 样条控制点，
// 夹紧结点让曲线从起点收到终点；曲线鼓进墙里就按比例往原路径退。
#include "my_nav2_smoother/qp_smoother.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <utility>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace my_nav2_smoother
{

void MyQpSmoother::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> /*tf*/,
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub,
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> /*footprint_sub*/)
{
  node_ = parent;
  name_ = name;
  costmap_sub_ = costmap_sub;  // 碰撞检查用全局代价地图，平滑本身不改它

  auto node = parent.lock();
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  // 参数名带插件实例前缀，对应 yaml 里 qp_smoother 下面的键
  auto declare = [&](const std::string & key, auto value) {
      nav2_util::declare_parameter_if_not_declared(
        node, name + "." + key, rclcpp::ParameterValue(value));
    };
  declare("smooth_weight", 10.0);
  declare("reference_weight", 1.0);
  declare("sample_distance", 0.1);
  declare("lethal_cost", 253);

  node->get_parameter(name + ".smooth_weight", smooth_weight_);
  node->get_parameter(name + ".reference_weight", reference_weight_);
  node->get_parameter(name + ".sample_distance", sample_distance_);
  int lethal_cost = 253;
  node->get_parameter(name + ".lethal_cost", lethal_cost);
  lethal_cost_ = static_cast<unsigned int>(std::max(0, std::min(lethal_cost, 254)));
  if (sample_distance_ < 1e-3) {
    sample_distance_ = 0.1;  // 间距太小会采出一堆重合点
  }

  RCLCPP_INFO(logger_, "Configuring plugin %s of type MyQpSmoother", name_.c_str());
}

void MyQpSmoother::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up plugin %s of type MyQpSmoother", name_.c_str());
  costmap_sub_.reset();
}

void MyQpSmoother::activate()
{
  RCLCPP_INFO(logger_, "Activating plugin %s of type MyQpSmoother", name_.c_str());
}

void MyQpSmoother::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating plugin %s of type MyQpSmoother", name_.c_str());
}

// 高斯消元。h 按值传来，消元时会被改掉。
bool MyQpSmoother::solveLinear(
  std::vector<std::vector<double>> h,
  std::vector<double> rhs,
  std::vector<double> & y) const
{
  const int n = static_cast<int>(rhs.size());
  y.assign(static_cast<size_t>(n), 0.0);
  if (n == 0) {
    return true;  // 没有未知数
  }

  for (int col = 0; col < n; ++col) {
    int pivot = col;
    double best = std::abs(h[static_cast<size_t>(col)][static_cast<size_t>(col)]);
    for (int row = col + 1; row < n; ++row) {
      const double value = std::abs(h[static_cast<size_t>(row)][static_cast<size_t>(col)]);
      if (value > best) {
        best = value;
        pivot = row;  // 这一列绝对值最大的当主元
      }
    }
    if (best < 1e-12) {
      return false;  // 矩阵奇异，解不了
    }
    if (pivot != col) {
      std::swap(h[static_cast<size_t>(pivot)], h[static_cast<size_t>(col)]);
      std::swap(rhs[static_cast<size_t>(pivot)], rhs[static_cast<size_t>(col)]);  // 行和右端一起换
    }
    const double diag = h[static_cast<size_t>(col)][static_cast<size_t>(col)];
    for (int row = col + 1; row < n; ++row) {
      const double factor = h[static_cast<size_t>(row)][static_cast<size_t>(col)] / diag;
      for (int c = col; c < n; ++c) {
        h[static_cast<size_t>(row)][static_cast<size_t>(c)] -=
          factor * h[static_cast<size_t>(col)][static_cast<size_t>(c)];  // 消掉这一列
      }
      rhs[static_cast<size_t>(row)] -= factor * rhs[static_cast<size_t>(col)];
    }
  }

  for (int row = n - 1; row >= 0; --row) {
    double sum = rhs[static_cast<size_t>(row)];
    for (int c = row + 1; c < n; ++c) {
      sum -= h[static_cast<size_t>(row)][static_cast<size_t>(c)] * y[static_cast<size_t>(c)];  // 减去已经求出来的
    }
    y[static_cast<size_t>(row)] = sum / h[static_cast<size_t>(row)][static_cast<size_t>(row)];  // 回代
  }
  return true;
}

bool MyQpSmoother::solveAxis(const std::vector<double> & ref, std::vector<double> & out) const
{
  const int n = static_cast<int>(ref.size());//路径总点数
  const int k = n - 2;  // 去掉固定的首尾，剩下的才是未知数
  if (k <= 0) {
    out = ref;
    return true;  // 没有中间点
  }

  // 代价是 w*s^2。H 是 Hessian，g 是线性项，后面解 H y = -g
  std::vector<std::vector<double>> h(static_cast<size_t>(k), std::vector<double>(static_cast<size_t>(k), 0.0));
  std::vector<double> g(static_cast<size_t>(k), 0.0);
  // 把 w*(coeff·y + constant)^2 累进 H 和 g
  auto add_square = [&](const std::vector<std::pair<int, double>> & terms, double constant, double weight) {
      if (weight == 0.0) {
        return;
      }
      for (const auto & a : terms) {
        g[static_cast<size_t>(a.first)] += 2.0 * weight * a.second * constant;  // 线性项
        for (const auto & b : terms) {
          h[static_cast<size_t>(a.first)][static_cast<size_t>(b.first)] +=
            2.0 * weight * a.second * b.second;  // 二次项
        }
      }
    };

  for (int i = 0; i < k; ++i) {
    // 自由点 x_{i+1} 贴近 A* 的对应点
    add_square({{i, 1.0}}, -ref[static_cast<size_t>(i + 1)], reference_weight_);
  }

  for (int i = 1; i <= n - 2; ++i) {
    // 二阶差分 x_{i-1} - 2 x_i + x_{i+1}。首尾是常数，不进未知数
    std::vector<std::pair<int, double>> terms;
    double constant = 0.0;
    auto consider = [&](int point, double coeff) {
        if (point == 0 || point == n - 1) {
          constant += coeff * ref[static_cast<size_t>(point)];  // 固定点挪到常数里
        } else {
          terms.emplace_back(point - 1, coeff);  // 中间点才是 y 的一维
        }
      };
    consider(i - 1, 1.0);
    consider(i, -2.0);
    consider(i + 1, 1.0);
    add_square(terms, constant, smooth_weight_);
  }

  std::vector<double> rhs(static_cast<size_t>(k));
  for (int i = 0; i < k; ++i) {
    rhs[static_cast<size_t>(i)] = -g[static_cast<size_t>(i)];  // H y = -g
  }
  std::vector<double> y;
  if (!solveLinear(std::move(h), std::move(rhs), y)) {
    return false;
  }

  out.resize(static_cast<size_t>(n));
  out.front() = ref.front();  // 起点固定
  out.back() = ref.back();    // 终点固定
  for (int i = 0; i < k; ++i) {
    out[static_cast<size_t>(i + 1)] = y[static_cast<size_t>(i)];
  }
  return true;
}

// 返回 true = 这条线段扫到墙。
bool MyQpSmoother::segmentBlocked(
  const nav2_costmap_2d::Costmap2D * costmap,
  double x0, double y0, double x1, double y1) const
{
  if (costmap == nullptr) {
    return false;  // 没有地图就不查
  }

  const double resolution = std::max(costmap->getResolution(), 1e-3);
  const double length = std::hypot(x1 - x0, y1 - y0);
  const int steps = std::max(1, static_cast<int>(std::ceil(length / resolution)));  // 按格子取样
  for (int step = 0; step <= steps; ++step) {
    const double ratio = static_cast<double>(step) / static_cast<double>(steps);
    const double x = x0 + (x1 - x0) * ratio;
    const double y = y0 + (y1 - y0) * ratio;
    unsigned int mx = 0;
    unsigned int my = 0;
    if (!costmap->worldToMap(x, y, mx, my)) {
      return true;  // 地图外面
    }
    const unsigned char cost = costmap->getCost(mx, my);
    if (cost == nav2_costmap_2d::NO_INFORMATION) {
      continue;  // 未知区先当可走
    }
    if (cost >= lethal_cost_) {
      return true;  // 253=贴身膨胀，254=致命障碍
    }
  }
  return false;
}

bool MyQpSmoother::polylineBlocked(
  const nav2_costmap_2d::Costmap2D * costmap,
  const std::vector<Point2D> & points) const
{
  if (points.size() < 2) {
    return false;
  }
  for (size_t i = 1; i < points.size(); ++i) {
    if (segmentBlocked(costmap, points[i - 1].x, points[i - 1].y, points[i].x, points[i].y)) {
      return true;  // 任意一段撞上，整条不行
    }
  }
  return false;
}

std::vector<Point2D> MyQpSmoother::blend(
  const nav2_costmap_2d::Costmap2D * costmap,
  const std::vector<Point2D> & smoothed,
  const std::vector<Point2D> & reference) const
{
  if (smoothed.size() != reference.size() || smoothed.empty()) {
    return reference;
  }
  if (!curveBlocked(costmap, smoothed)) {
    return smoothed;  // QP 控制点画出的曲线本身能走
  }
  // 原路径来自 A*。在两者之间取最靠 QP、曲线又不进墙的一份
  if (curveBlocked(costmap, reference)) {
    return reference;  // 连 A* 点的曲线也过不去，调用处改走折线
  }

  double feasible = 0.0;  // 目前知道能走的混合比例
  double blocked = 1.0;   // 目前知道会撞的混合比例
  std::vector<Point2D> best = reference;
  for (int iteration = 0; iteration < 12; ++iteration) {
    const double alpha = 0.5 * (feasible + blocked);  // alpha=1 是纯 QP，0 是原路径
    std::vector<Point2D> mixed;
    mixed.reserve(smoothed.size());
    for (size_t i = 0; i < smoothed.size(); ++i) {
      mixed.push_back(Point2D{
        (1.0 - alpha) * reference[i].x + alpha * smoothed[i].x,
        (1.0 - alpha) * reference[i].y + alpha * smoothed[i].y});
    }
    if (curveBlocked(costmap, mixed)) {
      blocked = alpha;  // 曲线鼓进墙里，往回退
    } else {
      feasible = alpha;
      best = std::move(mixed);  // 还能再靠近一点
    }
  }
  return best;
}

std::vector<Point2D> MyQpSmoother::resample(const std::vector<Point2D> & points) const
{
  if (points.size() < 2) {
    return points;
  }
  std::vector<Point2D> out;
  out.push_back(points.front());
  double leftover = 0.0;  // 上一段没走完的距离
  for (size_t i = 1; i < points.size(); ++i) {
    const double dx = points[i].x - points[i - 1].x;
    const double dy = points[i].y - points[i - 1].y;
    const double length = std::hypot(dx, dy);
    if (length < 1e-6) {
      continue;  // 重合点
    }
    const double ux = dx / length;  // 单位方向
    const double uy = dy / length;
    double traveled = 0.0;
    double step = sample_distance_ - leftover;  // 先补上一段欠的间距
    while (traveled + step <= length) {
      traveled += step;
      out.push_back(Point2D{
        points[i - 1].x + ux * traveled,
        points[i - 1].y + uy * traveled});
      step = sample_distance_;
    }
    leftover = length - traveled;
  }
  if (std::hypot(out.back().x - points.back().x, out.back().y - points.back().y) > 1e-3) {
    out.push_back(points.back());  // 终点是原来的终点
  } else {
    out.back() = points.back();
  }
  return out;
}

namespace
{

// 夹紧结点：前 degree+1 个和后 degree+1 个重复，曲线过首尾控制点。
std::vector<double> clampedKnots(int count, int degree)
{
  const int knot_count = count + degree + 1;
  std::vector<double> knots(static_cast<size_t>(knot_count), 0.0);
  const double end = static_cast<double>(std::max(count - degree, 0));
  for (int i = 0; i < knot_count; ++i) {
    if (i <= degree) {
      knots[static_cast<size_t>(i)] = 0.0;
    } else if (i >= count) {
      knots[static_cast<size_t>(i)] = end;  // 尾部重复，保证收到最后一个控制点
    } else {
      knots[static_cast<size_t>(i)] = static_cast<double>(i - degree);
    }
  }
  return knots;
}

// Cox-de Boor。分母为 0 的项丢掉。u 等于最后一个结点时算进最后一段。
double bsplineBasis(int i, int degree, double u, const std::vector<double> & knots)
{
  if (degree == 0) {
    const double left = knots[static_cast<size_t>(i)];
    const double right = knots[static_cast<size_t>(i + 1)];
    if (left <= u && u < right) {
      return 1.0;
    }
    if (u == knots.back() && left < right && right == knots.back()) {
      return 1.0;  // 终点落在半开区间外面，补进最后一段
    }
    return 0.0;
  }
  double value = 0.0;
  const double left_span = knots[static_cast<size_t>(i + degree)] - knots[static_cast<size_t>(i)];
  if (left_span > 1e-12) {
    value += (u - knots[static_cast<size_t>(i)]) / left_span *
      bsplineBasis(i, degree - 1, u, knots);
  }
  const double right_span =
    knots[static_cast<size_t>(i + degree + 1)] - knots[static_cast<size_t>(i + 1)];
  if (right_span > 1e-12) {
    value += (knots[static_cast<size_t>(i + degree + 1)] - u) / right_span *
      bsplineBasis(i + 1, degree - 1, u, knots);
  }
  return value;
}

Point2D evalBspline(
  const std::vector<Point2D> & controls, double u, const std::vector<double> & knots, int degree)
{
  Point2D point{0.0, 0.0};
  for (int i = 0; i < static_cast<int>(controls.size()); ++i) {
    const double weight = bsplineBasis(i, degree, u, knots);
    point.x += weight * controls[static_cast<size_t>(i)].x;
    point.y += weight * controls[static_cast<size_t>(i)].y;
  }
  return point;
}

}  // namespace

std::vector<Point2D> MyQpSmoother::sampleCurve(const std::vector<Point2D> & controls) const
{
  if (controls.size() < 4) {
    return resample(controls);  // 三次曲线至少要 4 个控制点
  }
  const int degree = 3;
  const auto knots = clampedKnots(static_cast<int>(controls.size()), degree);
  const double u0 = knots[static_cast<size_t>(degree)];
  const double u1 = knots[controls.size()];  // 参数走到这里就是终点

  double polygon = 0.0;
  for (size_t i = 1; i < controls.size(); ++i) {
    polygon += std::hypot(
      controls[i].x - controls[i - 1].x, controls[i].y - controls[i - 1].y);
  }
  // 先密采，再交给 resample 拉成 sample_distance_
  const int count = std::max(
    static_cast<int>(controls.size()),
    static_cast<int>(std::ceil(polygon / std::max(sample_distance_ * 0.5, 1e-3))));
  std::vector<Point2D> dense;
  dense.reserve(static_cast<size_t>(count) + 1);
  for (int k = 0; k <= count; ++k) {
    const double u = u0 + (u1 - u0) * static_cast<double>(k) / static_cast<double>(count);
    dense.push_back(evalBspline(controls, u, knots, degree));
  }
  dense.front() = controls.front();  // 夹紧：起点就是第一个控制点
  dense.back() = controls.back();    // 终点就是最后一个控制点
  return resample(dense);
}

bool MyQpSmoother::curveBlocked(
  const nav2_costmap_2d::Costmap2D * costmap,
  const std::vector<Point2D> & controls) const
{
  return polylineBlocked(costmap, sampleCurve(controls));
}

bool MyQpSmoother::smooth(
  nav_msgs::msg::Path & path,
  const rclcpp::Duration & max_time)
{
  const auto started = clock_ ? clock_->now() : rclcpp::Time(0, 0, RCL_ROS_TIME);
  auto expired = [&]() {
      if (!clock_ || max_time.nanoseconds() <= 0) {
        return false;  // 没给时限就一直算
      }
      return (clock_->now() - started) > max_time;
    };

  if (path.poses.size() < 3) {
    return true;  // 没有中间点，不用平滑
  }

  std::vector<Point2D> reference;
  reference.reserve(path.poses.size());
  for (size_t i = 0; i < path.poses.size(); ++i) {
    const auto & pose = path.poses[i];
    const Point2D point{pose.pose.position.x, pose.pose.position.y};
    const bool last = i + 1 == path.poses.size();
    if (!reference.empty() &&
      std::hypot(point.x - reference.back().x, point.y - reference.back().y) < 1e-4)
    {
      if (last) {
        reference.back() = point;  // 重复点丢掉，但终点必须留下
      }
      continue;
    }
    reference.push_back(point);
  }
  if (reference.size() < 3) {
    return true;
  }
  const auto goal_orientation = path.poses.back().pose.orientation;  // 最后一点的朝向留给目标

  std::vector<double> ref_x;
  std::vector<double> ref_y;
  ref_x.reserve(reference.size());
  ref_y.reserve(reference.size());
  for (const auto & point : reference) {
    ref_x.push_back(point.x);  // x、y 各解一次，两项代价都不交叉
    ref_y.push_back(point.y);
  }

  std::vector<double> smooth_x;
  std::vector<double> smooth_y;
  if (expired() || !solveAxis(ref_x, smooth_x) || !solveAxis(ref_y, smooth_y)) {
    RCLCPP_WARN(logger_, "QP smoother failed, keeping the A* path");
    return !expired();  // 解失败就留着原路径。超时才告诉行为树没做完
  }

  std::vector<Point2D> smoothed;
  smoothed.reserve(reference.size());
  for (size_t i = 0; i < reference.size(); ++i) {
    smoothed.push_back(Point2D{smooth_x[i], smooth_y[i]});  // 拼回二维点
  }
  const nav2_costmap_2d::Costmap2D * costmap = nullptr;
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_holder;
  if (costmap_sub_) {
    try {
      costmap_holder = costmap_sub_->getCostmap();
      costmap = costmap_holder.get();
    } catch (const std::exception & ex) {
      RCLCPP_WARN(logger_, "Costmap not ready, smoothing without a collision check: %s", ex.what());
      // 地图还没到，先不查碰撞
    }
  }
  const auto controls = blend(costmap, smoothed, reference);  // 这些点是 B 样条控制点，曲线不一定穿过中间点
  auto samples = sampleCurve(controls);
  if (polylineBlocked(costmap, samples)) {
    samples = resample(reference);  // 曲线鼓进墙里，退回 A* 折线
  }

  path.poses.clear();
  path.poses.reserve(samples.size());
  for (size_t i = 0; i < samples.size(); ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;  // 时间戳 + 坐标系跟 path 一致
    pose.pose.position.x = samples[i].x;
    pose.pose.position.y = samples[i].y;
    pose.pose.position.z = 0.0;
    double yaw = 0.0;
    if (i + 1 == samples.size()) {
      pose.pose.orientation = goal_orientation;  // 最后一个点用目标朝向
    } else {
      yaw = std::atan2(samples[i + 1].y - samples[i].y, samples[i + 1].x - samples[i].x);  // 指向下一点
      tf2::Quaternion quaternion;
      quaternion.setRPY(0.0, 0.0, yaw);
      pose.pose.orientation.x = quaternion.x();
      pose.pose.orientation.y = quaternion.y();
      pose.pose.orientation.z = quaternion.z();
      pose.pose.orientation.w = quaternion.w();
    }
    path.poses.push_back(pose);
  }
  return true;
}

}  // namespace my_nav2_smoother

PLUGINLIB_EXPORT_CLASS(my_nav2_smoother::MyQpSmoother, nav2_core::Smoother)
