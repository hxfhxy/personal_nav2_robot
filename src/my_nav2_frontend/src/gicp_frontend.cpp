// 雷达前端：相邻两帧点云做 GICP，把增量位姿乘起来当里程计。
// 订阅 /points（16 线雷达，PointCloud2），发布 /lidar_odom（nav_msgs::Odometry）
// /lidar_odom 保留雷达原始结果；/odom 和 odom -> base_footprint 供 Nav2 使用。
// /ground_truth 只用来在终端里对照误差，不参与计算。
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <stdexcept>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pcl/filters/voxel_grid.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "small_gicp/pcl/pcl_registration.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace
{

pcl::PointCloud<pcl::PointXYZ>::Ptr finitePoints(
  const sensor_msgs::msg::PointCloud2 & msg)
{
  pcl::PointCloud<pcl::PointXYZ> raw;
  pcl::fromROSMsg(msg, raw);  // PointCloud2 收成 PCL 的 PointXYZ
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cloud->reserve(raw.size());
  for (const auto & point : raw.points) {
    if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
      cloud->push_back(point);  // 丢掉没打中的 inf
    }
  }
  return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr downsample(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, double leaf)
{
  auto filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud(cloud);
  voxel.setLeafSize(leaf, leaf, leaf);  // 一个格子只留一个点
  voxel.filter(*filtered);
  return filtered;
}

}  // namespace

// 订阅 /points，相邻两帧做 GICP；AMCL 独立维护 map -> odom。
class GicpFrontend : public rclcpp::Node
{
public:
  GicpFrontend()
  : Node("gicp_frontend"),
    accumulated_(Eigen::Isometry3d::Identity())
  {
    declare_parameter("points_topic", "/points");
    declare_parameter("ground_truth_topic", "/ground_truth");
    declare_parameter("voxel_leaf", 0.15);
    declare_parameter("max_correspondence_distance", 1.0);
    declare_parameter("num_threads", 4);
    declare_parameter("publish_navigation_odom", true);
    declare_parameter("odom_frame", "odom");
    declare_parameter("base_frame", "base_footprint");
    declare_parameter("self_filter_min", std::vector<double>{-0.30, -0.21, -0.05});
    declare_parameter("self_filter_max", std::vector<double>{0.22, 0.21, 0.25});
    declare_parameter("min_registration_height", 0.05);

    voxel_leaf_ = get_parameter("voxel_leaf").as_double();
    max_correspondence_distance_ = get_parameter("max_correspondence_distance").as_double();
    num_threads_ = get_parameter("num_threads").as_int();
    publish_navigation_odom_ = get_parameter("publish_navigation_odom").as_bool();
    odom_frame_ = get_parameter("odom_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    min_registration_height_ = get_parameter("min_registration_height").as_double();
    const auto self_min = get_parameter("self_filter_min").as_double_array();
    const auto self_max = get_parameter("self_filter_max").as_double_array();
    if (self_min.size() != 3 || self_max.size() != 3) {
      throw std::invalid_argument("Self filter bounds must contain x, y and z");
    }
    for (int axis = 0; axis < 3; ++axis) {
      if (self_min[axis] >= self_max[axis]) {
        throw std::invalid_argument("Self filter minimum must be below maximum");
      }
      self_min_[axis] = self_min[axis];
      self_max_[axis] = self_max[axis];
    }
    if (voxel_leaf_ <= 0.0 || max_correspondence_distance_ <= 0.0 || num_threads_ < 1) {
      throw std::invalid_argument("GICP leaf size, correspondence distance and threads must be positive");
    }

    const auto points_topic = get_parameter("points_topic").as_string();
    const auto ground_truth_topic = get_parameter("ground_truth_topic").as_string();

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/lidar_odom", 10);
    if (publish_navigation_odom_) {
      navigation_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    }
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    points_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      // 配准可能赶不上雷达频率，只处理最新点云，不能把历史帧排队送给控制闭环。
      points_topic, rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&GicpFrontend::onCloud, this, std::placeholders::_1));
    ground_truth_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      ground_truth_topic, 10,
      std::bind(&GicpFrontend::onGroundTruth, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "GICP listens to %s; navigation odometry %s. Ground truth is evaluation only.",
      points_topic.c_str(), publish_navigation_odom_ ? "enabled" : "disabled");
  }

private:
  struct Frame
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    Eigen::Isometry3d ground_truth{Eigen::Isometry3d::Identity()};
    bool have_ground_truth{false};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  };

  void onGroundTruth(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    tf2::fromMsg(msg->pose.pose, pose);  // 真值位姿，只用来对照
    std::lock_guard<std::mutex> lock(mutex_);
    ground_truth_ = pose;
    ground_truth_stamp_ = rclcpp::Time(msg->header.stamp);
    have_ground_truth_ = true;
  }

  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // 外参从 URDF 的静态 TF 获取，不能直接把雷达位置当成底盘位置。
    if (!have_extrinsic_) {
      try {
        base_to_lidar_ = tf2::transformToEigen(tf_buffer_->lookupTransform(
          base_frame_, msg->header.frame_id, tf2::TimePointZero));
        lidar_frame_ = msg->header.frame_id;
        have_extrinsic_ = true;
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Waiting for lidar extrinsic: %s", ex.what());
        return;
      }
    }
    const rclcpp::Time stamp(msg->header.stamp);
    if (msg->header.frame_id != lidar_frame_ ||
      (previous_.cloud && stamp <= previous_.stamp))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Ignoring cloud with a changed frame or non-increasing timestamp");
      return;
    }
    auto cloud = finitePoints(*msg);
    // 车尾会被雷达扫到。这些点固定在车上，不能作为环境约束参与里程计配准。
    auto environment = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    environment->reserve(cloud->size());
    for (const auto & point : *cloud) {
      const Eigen::Vector3d base = base_to_lidar_ *
        Eigen::Vector3d(point.x, point.y, point.z);
      // 平地小车使用离地结构配准，避免稀疏空地上的地面采样圆环压制水平运动。
      if (base.z() < min_registration_height_) {
        continue;
      }
      if ((base.array() >= self_min_.array()).all() &&
        (base.array() <= self_max_.array()).all())
      {
        continue;
      }
      environment->push_back(point);
    }
    cloud = environment;
    if (cloud->size() < 50) {
      return;  // 点太少，协方差估不出来
    }
    cloud = downsample(cloud, voxel_leaf_);
    if (cloud->size() < 50) {
      return;
    }

    Frame current;
    current.cloud = cloud;
    current.stamp = stamp;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      current.ground_truth = ground_truth_;
      current.have_ground_truth = have_ground_truth_ &&
        std::abs((stamp - ground_truth_stamp_).seconds()) < 0.05;
    }

    if (!previous_.cloud) {
      previous_ = current;  // 第一帧只当目标，没有相对运动
      publish(msg->header.stamp, accumulated_, 0.0);
      return;
    }

    Eigen::Isometry3d delta = Eigen::Isometry3d::Identity();
    if (!align(previous_.cloud, current.cloud, delta)) {
      // 保留最后成功帧，恢复时仍能补回中间的运动；不伪造新的 odom / TF。
      return;
    }

    const double step = delta.translation().norm();
    const double turn = Eigen::AngleAxisd(delta.rotation()).angle();
    if (!delta.matrix().allFinite() || step > 1.0 || std::abs(turn) > 1.0) {
      RCLCPP_WARN(
        get_logger(), "Reject GICP step %.2f m, %.2f rad", step, turn);
      return;
    }

    accumulated_ = accumulated_ * delta;  // 当前帧在起始雷达坐标系里的位姿
    publish(msg->header.stamp, accumulated_, (current.stamp - previous_.stamp).seconds());
    logComparison(previous_, current, delta);
    previous_ = current;
  }

  bool align(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & target,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & source,
    Eigen::Isometry3d & delta) const
  {
    small_gicp::RegistrationPCL<pcl::PointXYZ, pcl::PointXYZ> registration;
    registration.setRegistrationType("GICP");  // 用局部协方差，不是点到点 ICP
    registration.setNumThreads(static_cast<int>(num_threads_));
    registration.setNumNeighborsForCovariance(20);
    registration.setMaxCorrespondenceDistance(max_correspondence_distance_);
    registration.setMaximumIterations(25);
    registration.setInputTarget(target);  // 上一帧固定
    registration.setInputSource(source);  // 当前帧被对齐过去

    pcl::PointCloud<pcl::PointXYZ> aligned;
    registration.align(aligned);  // 初值是单位阵。
    if (!registration.hasConverged() || registration.getRegistrationResult().num_inliers < 50) {
      RCLCPP_WARN(get_logger(), "GICP did not converge");
      return false;
    }
    delta.matrix() = registration.getFinalTransformation().cast<double>();
    return true;
  }

  void logComparison(
    const Frame & previous, const Frame & current, const Eigen::Isometry3d & delta)
  {
    if (!previous.have_ground_truth || !current.have_ground_truth) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "GICP moved %.3f m", delta.translation().norm());
      return;
    }
    const Eigen::Isometry3d truth = base_to_lidar_.inverse() *
      previous.ground_truth.inverse() * current.ground_truth * base_to_lidar_;
    const Eigen::Isometry3d error = truth.inverse() * delta;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "GICP %.3f m, ground truth %.3f m, error %.3f m",
      delta.translation().norm(), truth.translation().norm(), error.translation().norm());
  }

  void publish(
    const builtin_interfaces::msg::Time & stamp, const Eigen::Isometry3d & pose, double dt)
  {
    const geometry_msgs::msg::Pose pose_msg = tf2::toMsg(pose);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "lidar_odom";  // 单独的坐标系，不接到 map
    odom.child_frame_id = "lidar_base";
    odom.pose.pose = pose_msg;
    odom_pub_->publish(odom);

    // odom 原点是第一帧的底盘位姿。E * T_L0_Lt * E^-1 同时补偿雷达杆臂。
    Eigen::Isometry3d base_pose = base_to_lidar_ * pose * base_to_lidar_.inverse();
    const double yaw = std::atan2(base_pose.linear()(1, 0), base_pose.linear()(0, 0));
    base_pose.translation().z() = 0.0;
    base_pose.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    if (publish_navigation_odom_) {
      odom.header.frame_id = odom_frame_;
      odom.child_frame_id = base_frame_;
      odom.pose.pose = tf2::toMsg(base_pose);
      if (dt > 0.0) {
        const Eigen::Vector3d velocity = base_pose.linear().transpose() *
          (base_pose.translation() - previous_base_pose_.translation()) / dt;
        odom.twist.twist.linear.x = velocity.x();
        odom.twist.twist.linear.y = velocity.y();
        const Eigen::Matrix3d rotation = previous_base_pose_.linear().transpose() * base_pose.linear();
        odom.twist.twist.angular.z = std::atan2(rotation(1, 0), rotation(0, 0)) / dt;
      }
      // 固定协方差是仿真初始值，还不是由配准 Hessian 估计的置信度。
      odom.pose.covariance[0] = odom.pose.covariance[7] = 0.02;
      odom.pose.covariance[35] = 0.03;
      odom.twist.covariance[0] = odom.twist.covariance[7] = 0.04;
      odom.twist.covariance[35] = 0.06;
      for (const auto index : {14, 21, 28}) {
        odom.pose.covariance[index] = odom.twist.covariance[index] = 1e6;
      }
      navigation_odom_pub_->publish(odom);
    }
    previous_base_pose_ = base_pose;

    geometry_msgs::msg::TransformStamped transform;
    transform.header = odom.header;
    transform.child_frame_id = odom.child_frame_id;
    transform.transform.translation.x = odom.pose.pose.position.x;
    transform.transform.translation.y = odom.pose.pose.position.y;
    transform.transform.translation.z = odom.pose.pose.position.z;
    transform.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(transform);
  }

  double voxel_leaf_{0.15};
  double max_correspondence_distance_{1.0};
  int64_t num_threads_{4};
  bool publish_navigation_odom_{true};
  bool have_extrinsic_{false};
  std::string odom_frame_;
  std::string base_frame_;
  std::string lidar_frame_;
  Eigen::Vector3d self_min_;
  Eigen::Vector3d self_max_;
  double min_registration_height_{0.05};

  Eigen::Isometry3d accumulated_;
  Eigen::Isometry3d base_to_lidar_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d previous_base_pose_{Eigen::Isometry3d::Identity()};
  Frame previous_;
  Eigen::Isometry3d ground_truth_{Eigen::Isometry3d::Identity()};
  bool have_ground_truth_{false};
  rclcpp::Time ground_truth_stamp_{0, 0, RCL_ROS_TIME};
  std::mutex mutex_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr navigation_odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ground_truth_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GicpFrontend>());
  rclcpp::shutdown();
  return 0;
}
