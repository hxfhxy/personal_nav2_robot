// 雷达前端：相邻两帧点云做 GICP，把增量位姿乘起来当里程计。
// 订阅 /points（16 线雷达，PointCloud2），发布 /lidar_odom（nav_msgs::Odometry）
// 和 lidar_odom -> lidar_base 的 TF。位姿只在自己这个坐标系里累积，不接到 map。
// /ground_truth 只用来在终端里对照误差，不参与计算。
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

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

// 订阅 /points，相邻两帧做 GICP，累积成 /lidar_odom。不改 map -> odom。
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

    voxel_leaf_ = get_parameter("voxel_leaf").as_double();
    max_correspondence_distance_ = get_parameter("max_correspondence_distance").as_double();
    num_threads_ = get_parameter("num_threads").as_int();

    const auto points_topic = get_parameter("points_topic").as_string();
    const auto ground_truth_topic = get_parameter("ground_truth_topic").as_string();

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/lidar_odom", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    points_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      points_topic, rclcpp::SensorDataQoS(),
      std::bind(&GicpFrontend::onCloud, this, std::placeholders::_1));
    ground_truth_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      ground_truth_topic, 10,
      std::bind(&GicpFrontend::onGroundTruth, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "GICP frontend listens to %s and publishes /lidar_odom. map -> odom stays on ground truth.",
      points_topic.c_str());
  }

private:
  struct Frame
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    Eigen::Isometry3d ground_truth{Eigen::Isometry3d::Identity()};
    bool have_ground_truth{false};
  };

  void onGroundTruth(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    tf2::fromMsg(msg->pose.pose, pose);  // 真值位姿，只用来对照
    std::lock_guard<std::mutex> lock(mutex_);
    ground_truth_ = pose;
    have_ground_truth_ = true;
  }

  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    auto cloud = finitePoints(*msg);
    if (cloud->size() < 50) {
      return;  // 点太少，协方差估不出来
    }
    cloud = downsample(cloud, voxel_leaf_);
    if (cloud->size() < 50) {
      return;
    }

    Frame current;
    current.cloud = cloud;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      current.ground_truth = ground_truth_;
      current.have_ground_truth = have_ground_truth_;
    }

    if (!previous_.cloud) {
      previous_ = current;  // 第一帧只当目标，没有相对运动
      publish(msg->header.stamp, accumulated_);
      return;
    }

    Eigen::Isometry3d delta = Eigen::Isometry3d::Identity();
    if (!align(previous_.cloud, current.cloud, delta)) {
      previous_ = current;  // 这帧没配上，丢掉这次增量
      return;
    }

    const double step = delta.translation().norm();
    const double turn = Eigen::AngleAxisd(delta.rotation()).angle();
    if (step > 1.0 || std::abs(turn) > 1.0) {
      RCLCPP_WARN(
        get_logger(), "Reject GICP step %.2f m, %.2f rad", step, turn);
      previous_ = current;
      return;
    }

    accumulated_ = accumulated_ * delta;  // 当前帧在起始雷达坐标系里的位姿
    publish(msg->header.stamp, accumulated_);
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
    registration.setMaximumIterations(15);
    registration.setInputTarget(target);  // 上一帧固定
    registration.setInputSource(source);  // 当前帧被对齐过去

    pcl::PointCloud<pcl::PointXYZ> aligned;
    registration.align(aligned);  // 初值是单位阵。
    if (!registration.hasConverged()) {
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
    const Eigen::Isometry3d truth =
      previous.ground_truth.inverse() * current.ground_truth;  // 同一段时间的真值增量
    const Eigen::Isometry3d error = truth.inverse() * delta;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "GICP %.3f m, ground truth %.3f m, error %.3f m",
      delta.translation().norm(), truth.translation().norm(), error.translation().norm());
  }

  void publish(const builtin_interfaces::msg::Time & stamp, const Eigen::Isometry3d & pose)
  {
    const geometry_msgs::msg::Pose pose_msg = tf2::toMsg(pose);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "lidar_odom";  // 单独的坐标系，不接到 map
    odom.child_frame_id = "lidar_base";
    odom.pose.pose = pose_msg;
    odom_pub_->publish(odom);

    geometry_msgs::msg::TransformStamped transform;
    transform.header = odom.header;
    transform.child_frame_id = odom.child_frame_id;
    transform.transform.translation.x = pose_msg.position.x;
    transform.transform.translation.y = pose_msg.position.y;
    transform.transform.translation.z = pose_msg.position.z;
    transform.transform.rotation = pose_msg.orientation;
    tf_broadcaster_->sendTransform(transform);
  }

  double voxel_leaf_{0.15};
  double max_correspondence_distance_{1.0};
  int64_t num_threads_{4};

  Eigen::Isometry3d accumulated_;
  Frame previous_;
  Eigen::Isometry3d ground_truth_{Eigen::Isometry3d::Identity()};
  bool have_ground_truth_{false};
  std::mutex mutex_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
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
