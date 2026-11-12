#include "my_nav2_planner/astar_planner.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <cmath>
#include <queue>
#include <vector>
#include <limits>
#include <algorithm>

namespace my_nav2_planner
{
  void MyAStarPlanner::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> /*tf*/,  // 全局规划在 map 坐标系做，这里不用 TF
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros){
      costmap_ = costmap_ros->getCostmap();                 // 真正的二维栅格
      global_frame_ = costmap_ros->getGlobalFrameID();      // 通常 "map"
      name_ = name;
      auto node = parent.lock();
      clock_ = node->get_clock();
      logger_ = node->get_logger();
    }

  void MyAStarPlanner::cleanup(){
    RCLCPP_INFO(logger_, "Cleaning up MyAStarPlanner");
  }

  void MyAStarPlanner::activate(){
    RCLCPP_INFO(logger_, "Activating MyAStarPlanner");
  }
  void MyAStarPlanner::deactivate(){
    RCLCPP_INFO(logger_, "Deactivating MyAStarPlanner");
  }

  // 返回 true = 这格不能走。
  bool MyAstarPlanner::isBlocked(unsigned int mx, unsigned int my) const {
    if(mx>=costmap_->getSizeInCellsX() || my>=costmap_->getSizeInCellsY()){
      return true;  // 地图外面
    }
    const unsigned char cost = costmap_->getCost(mx,my);
    if(cost==nav2_costmap_2d::NO_INFORMATION){
      return false;  // 未知区先当可走，否则没扫到的地方永远规划不过去
    }
    // 253=贴身膨胀，254=致命障碍，都当墙
    return cost>=nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
  }

  nav_msgs::msg::Path MyAStarPlanner::createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal){
      nav_msgs::msg::Path path;
      path.header.frame_id = global_frame_;
      path.header.stamp=clock_->now();
      //坐标系检查
      if(start.header.frame_id != global_frame_ || goal.header.frame_id != global_frame_){
        RCLCPP_ERROR(logger_, "Start or goal is not in the global frame");
        return path;
      }
      double sx = start.pose.position.x;
      double sy = start.pose.position.y;
      double gx = goal.pose.position.x;
      double gy = goal.pose.position.y;

      // 已经站在终点附近，一格都不用搜
      if(std::hypot(gx-sx,gy-sy)<costmap_->getResolution()){
        geometry_msgs::msg::PoseStamped pose;      // 草稿点
        pose.header = path.header;                 // 时间戳 + 坐标系跟 path 一致
        pose.pose = start.pose;                    // 位置抄起点
        pose.pose.orientation = goal.pose.orientation;   // 朝向改成终点的
        path.poses.push_back(pose);                // 装进数组，此时 size()==1
        return path;                               // 直接出去，不跑 A*
      }
      // 世界坐标 格子索引。worldToMap 第三个参数必须是 unsigned int&
      unsigned int smx = 0, smy = 0, gmx = 0, gmy = 0;
      if(!costmap_->worldToMap(sx,sy,smx,smy) || !costmap_->worldToMap(gx,gy,gmx,gmy)){
        RCLCPP_ERROR(logger_, "Start or goal is out of the costmap bounds");
        return path;
      }
      // 机器人当前格子可能被自己的足迹标成障碍，先清掉才能搜出去
      costmap_->setCost(smx, smy, nav2_costmap_2d::FREE_SPACE);
      const auto cells= searchAstar(smx, smy, gmx, gmy);
      if (cells.empty()) {
        RCLCPP_WARN(logger_, "A* failed to find a path");
        return path;
      }
      // 格子序列 Path：每个格子取中心点世界坐标
      path.poses.reserve(cells.size());
      for (size_t i = 0; i < cells.size(); ++i) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        costmap_->mapToWorld(
          cells[i].first, cells[i].second,
          pose.pose.position.x, pose.pose.position.y);
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0;  // 中间点朝向先填单位四元数
        if (i + 1 == cells.size()) {
          pose.pose.orientation = goal.pose.orientation;  // 最后一个点用目标朝向
        }
        path.poses.push_back(pose);
      }
      return path;
  }

  std::vector<std::pair<unsigned int, unsigned int>> MyAStarPlanner::searchAstar(
    unsigned int start_x, unsigned int start_y,
    unsigned int goal_x, unsigned int goal_y){
      const int nx=static_cast<int>(costmap_->getSizeInCellsX());
      const int ny=static_cast<int>(costmap_->getSizeInCellsY());
      const int ns=nx*ny;  // 格子总数，一维数组按下标 y*nx+x 排
      auto idx=[nx](int x, int y){return y*nx+x;};//坐标转索引

      const int start = idx(static_cast<int>(start_x), static_cast<int>(start_y));
      const int goal  = idx(static_cast<int>(goal_x),   static_cast<int>(goal_y));

      std::vector<double> g(ns, std::numeric_limits<double>::infinity());  // 起点到该格的实际代价
      std::vector<int>    parent(ns, -1);  // 回溯用：谁把我更新的
      std::vector<char>   closed(ns, 0);   // 1=已经弹出过，不再扩展

      auto valid = [nx, ny](int x, int y) {
        return x >= 0 && y >= 0 && x < nx && y < ny;
      };
      if (isBlocked(goal_x, goal_y) && start != goal) {
        return {};  // 终点在墙里，搜也没用
      }
      // 八邻域对角距离（octile）：不高估真实步长，A* 才保证最优
      auto heuristic = [&](int x, int y) {
        const int dx = std::abs(x - static_cast<int>(goal_x));
        const int dy = std::abs(y - static_cast<int>(goal_y));
        return (std::sqrt(2.0) - 1.0) * std::min(dx, dy) + std::max(dx, dy);
      };
      // 四直 + 四斜
      const int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
      const int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};
      const double step[8] = {1, 1, 1, 1, std::sqrt(2.0), std::sqrt(2.0), std::sqrt(2.0), std::sqrt(2.0)};

      // 小根堆：pair.first = f = g+h，pair.second = 格子下标
      std::priority_queue<std::pair<double, int>,
                          std::vector<std::pair<double, int>>,
                          std::greater<std::pair<double, int>>> open;
      g[start] = 0.0;
      open.push({heuristic(static_cast<int>(start_x), static_cast<int>(start_y)), start});

      while (!open.empty()) {
        const int cur = open.top().second;
        open.pop();
        if (closed[cur]) continue;  // 同一格可能入堆多次，只处理第一次弹出
        closed[cur] = 1;
        if (cur == goal) break;     // 终点第一次弹出时路径已最优

        const int cx = cur % nx;
        const int cy = cur / nx;
        for (int k = 0; k < 8; ++k) {
          const int x = cx + dx[k];
          const int y = cy + dy[k];
          if (!valid(x, y) || isBlocked(x, y)) continue;
          // 斜走时两侧直邻有墙就剪掉，避免穿墙角
          if (dx[k] && dy[k] &&
              (isBlocked(cx + dx[k], cy) || isBlocked(cx, cy + dy[k]))) continue;
          const int nxt = idx(x, y);
          if (closed[nxt]) continue;
          // 步长 + 一点代价值，让路径更愿意走空地而不是贴膨胀
          const double tent = g[cur] + step[k] + costmap_->getCost(x, y) / 253.0;
          if (tent >= g[nxt]) continue;  // 没有更优，丢掉
          g[nxt] = tent;
          parent[nxt] = cur;
          open.push({tent + heuristic(x, y), nxt});
        }
      }

      if (parent[goal] < 0 && start != goal) return {};  // 终点从未被更新过

      // 从终点顺着 parent 走回起点，再反转成正序
      std::vector<std::pair<unsigned int, unsigned int>> cells;
      for (int p = goal; p >= 0; p = parent[p]) {
        cells.emplace_back(p % nx, p / nx);
        if (p == start) break;
      }
      if (cells.empty() || cells.back() != std::make_pair(start_x, start_y)) return {};
      std::reverse(cells.begin(), cells.end());
      return cells;

      return path;
  }
}



PLUGINLIB_EXPORT_CLASS(my_nav2_planner::MyAStarPlanner, nav2_core::GlobalPlanner)
