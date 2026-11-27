#!/usr/bin/env python3
"""Publish /odom and the base TF in the optional ground-truth comparison mode."""

# 仅真值对照模式启动；正常导航由 GICP 发布底盘里程计和 TF。
# 话题 /ground_truth 是 libgazebo_ros_p3d 发的，frame_name 写的是 world；
# 这里直接当成 odom 发出去，因为 map -> odom 在导航模式下是固定的单位变换。

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class GroundTruthTf(Node):

    def __init__(self):
        super().__init__('ground_truth_tf')
        self.broadcaster = TransformBroadcaster(self)
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.create_subscription(Odometry, '/ground_truth', self._on_odom, 10)

    def _on_odom(self, msg):
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_footprint'
        self.odom_publisher.publish(msg)
        # 位姿原样搬过来，不改坐标系也不加噪声：世界系原点就是 map 的原点
        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_footprint'
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self.broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    node = GroundTruthTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
