#include "my_nav2_planner/astar_planner.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <cmath>
#include <queue>
#include <vector>
#include <limits>

namespace my_nav2_planner
{
  void MyAStarPlanner::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> /*tf*/,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros){
      costmap_ = costmap_ros->getCostmap();
      global_frame_ = costmap_ros->getGlobalFrameID();
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

      if(std::hypot(gx-sx,gy-sy)<costmap_->getResolution()){
        geometry_msgs::msg::PoseStamped pose;      // 草稿点
        pose.header = path.header;                 // 时间戳 + 坐标系跟 path 一致
        pose.pose = start.pose;                    // 位置抄起点
        pose.pose.orientation = goal.pose.orientation;   // 朝向改成终点的
        path.poses.push_back(pose);                // 装进数组，此时 size()==1
        return path;                               // 直接出去，不跑 A*
      }
      unsigned int smx = 0, smy = 0, gmx = 0, gmy = 0;
      if(!costmap_->worldToMap(sx,sy,smx,smy) || !costmap_->worldToMap(gx,gy,gmx,gmy)){
        RCLCPP_ERROR(logger_, "Start or goal is out of the costmap bounds");
        return path;
      }
      costmap_->setCost(smx, smy, nav2_costmap_2d::FREE_SPACE);
      return path;
  }
}

PLUGINLIB_EXPORT_CLASS(my_nav2_planner::MyAStarPlanner, nav2_core::GlobalPlanner)
