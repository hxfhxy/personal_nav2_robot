#!/usr/bin/env python3
"""Exercise the real frontend with known point clouds and no ground-truth input.

Run after sourcing ROS and this workspace. All test topics are isolated by remapping.
"""
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest

import numpy as np
import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster


def pose(x, y, yaw, z=0.0):
    value = np.eye(4)
    c, s = math.cos(yaw), math.sin(yaw)
    value[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    value[:3, 3] = [x, y, z]
    return value


class GicpOdometryTest(unittest.TestCase):
    def test_navigation_odometry(self):
        namespace = f'/gicp_regression_{os.getpid()}'
        ros_args = ['--ros-args']
        for topic in ('points', 'odom', 'lidar_odom', 'ground_truth', 'tf', 'tf_static'):
            ros_args.extend(['-r', f'/{topic}:={namespace}/{topic}'])
        rclpy.init(args=ros_args)
        node = rclpy.create_node('gicp_regression')
        received, transforms = [], []
        node.create_subscription(Odometry, '/odom', received.append, 10)
        node.create_subscription(TFMessage, '/tf', transforms.append, 10)
        publisher = node.create_publisher(PointCloud2, '/points', 10)
        broadcaster = StaticTransformBroadcaster(node)
        static = TransformStamped()
        static.header.frame_id = 'base_footprint'
        static.child_frame_id = 'test_laser'
        static.transform.translation.x = 0.2
        static.transform.translation.z = 0.3
        static.transform.rotation.z = math.sin(0.35 / 2)
        static.transform.rotation.w = math.cos(0.35 / 2)
        broadcaster.sendTransform(static)
        extrinsic = pose(0.2, 0, 0.35, 0.3)
        # Three asymmetric perpendicular planes constrain translation and rotation.
        points = []
        for a in np.linspace(-1.5, 1.5, 25):
            for b in np.linspace(0.0, 2.0, 20):
                points.extend(((3, a, b, 1), (a, -2, b, 1), (a, b, 0, 1)))
        world = np.asarray(points, dtype=np.float64)
        log = tempfile.TemporaryFile(mode='w+')
        executable = Path(get_package_prefix('my_nav2_frontend')) / 'lib/my_nav2_frontend/gicp_frontend'
        env = dict(os.environ, ROS_LOG_DIR=tempfile.mkdtemp(prefix='gicp_test_logs_'))
        process = subprocess.Popen([str(executable)] + ros_args, stdout=log, stderr=log,
                                   env=env, start_new_session=True)

        def wait_for(predicate, timeout=8.0):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                if predicate():
                    return
                if process.poll() is not None:
                    break
            log.seek(0)
            self.fail('Timed out waiting for frontend\n' + log.read())

        def cloud(stamp, base, bad=False):
            xyz = (np.linalg.inv(base @ extrinsic) @ world.T).T[:, :3]
            if bad:
                xyz += 100.0  # no correspondences; must not replace the accepted reference
            header = Header(frame_id='test_laser')
            header.stamp.sec, header.stamp.nanosec = stamp
            return point_cloud2.create_cloud_xyz32(header, xyz.astype(np.float32).tolist())

        try:
            wait_for(lambda: publisher.get_subscription_count() > 0)
            broadcaster.sendTransform(static)
            first = cloud((10, 0), pose(0, 0, 0))
            deadline = time.monotonic() + 8
            while not received and time.monotonic() < deadline:
                publisher.publish(first)
                rclpy.spin_once(node, timeout_sec=0.15)
            self.assertTrue(received, 'No initial odometry')
            self.assertAlmostEqual(received[-1].pose.pose.position.x, 0.0, delta=0.01)
            for i in range(1, 6):
                stamp = (10 + i, 0)
                if i == 3:
                    publisher.publish(cloud((12, 500000000), pose(0, 0, 0), bad=True))
                    until = time.monotonic() + 0.3
                    while time.monotonic() < until:
                        rclpy.spin_once(node, timeout_sec=0.05)
                    self.assertEqual(received[-1].header.stamp.sec, 12)
                expected = pose(0.06 * i, 0.02 * i, 0.05 * i)
                publisher.publish(cloud(stamp, expected))
                wait_for(lambda: received[-1].header.stamp.sec == stamp[0])
                odom = received[-1]
                self.assertEqual((odom.header.frame_id, odom.child_frame_id),
                                 ('odom', 'base_footprint'))
                self.assertAlmostEqual(odom.pose.pose.position.x, expected[0, 3], delta=0.015)
                self.assertAlmostEqual(odom.pose.pose.position.y, expected[1, 3], delta=0.015)
                self.assertAlmostEqual(odom.pose.pose.position.z, 0.0, delta=1e-9)
                q = odom.pose.pose.orientation
                self.assertAlmostEqual(2 * math.atan2(q.z, q.w), 0.05 * i, delta=0.015)
                self.assertAlmostEqual(odom.twist.twist.angular.z, 0.05, delta=0.015)
                velocity = expected[:3, :3].T @ np.array([0.06, 0.02, 0.0])
                self.assertAlmostEqual(odom.twist.twist.linear.x, velocity[0], delta=0.015)
                self.assertGreater(odom.pose.covariance[0], 0)
            wait_for(lambda: any(t.child_frame_id == 'base_footprint' and
                                 t.header.frame_id == 'odom' and t.header.stamp.sec == 15
                                 for message in transforms for t in message.transforms))
            final_tf = next(t for message in transforms for t in message.transforms
                            if t.child_frame_id == 'base_footprint' and t.header.stamp.sec == 15)
            self.assertAlmostEqual(final_tf.transform.translation.x,
                                   received[-1].pose.pose.position.x, delta=1e-9)
            self.assertAlmostEqual(final_tf.transform.translation.y,
                                   received[-1].pose.pose.position.y, delta=1e-9)
            self.assertAlmostEqual(final_tf.transform.translation.z, 0.0, delta=1e-9)
            # Duplicate timestamps must not produce fresh odometry.
            count = len(received)
            publisher.publish(cloud((15, 0), pose(1, 1, 1)))
            until = time.monotonic() + 0.3
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=0.05)
            self.assertEqual(len(received), count)
        finally:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
            node.destroy_node()
            rclpy.shutdown()
            log.close()


if __name__ == '__main__':
    unittest.main()
