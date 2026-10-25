#!/usr/bin/env python3
"""把传感器系点云变换到 odom 系并发布 (仿真里替代 FAST-LIO 的 /cloud_registered)

为什么需要它:
  Gazebo 的 /livox/lidar 是 livox_frame(传感器)系原始点云, 直接喂给 3D 重定位时,
  GICP 解出的是"雷达在 map 中的绝对位姿", 随车移动, 与实车链路(FAST-LIO 的
  /cloud_registered 已注册到 odom 系, 解出的是 map->odom 修正量)语义不一致。
  这里先用 TF 把点云搬到 odom 系, 让仿真侧和实车侧走同一条语义。

用法:
  ros2 run fishbot_description cloud_to_odom.py
  ros2 run fishbot_description cloud_to_odom.py --ros-args \
    -p input_topic:=/livox/lidar -p output_topic:=/cloud_registered \
    -p target_frame:=odom
"""

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
import tf2_ros


def quat_to_rot(x, y, z, w):
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1.0 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1.0 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1.0 - s * (x * x + y * y)],
    ])


class CloudToOdom(Node):
    def __init__(self):
        super().__init__('cloud_to_odom')
        self.input_topic = self.declare_parameter(
            'input_topic', '/livox/lidar').value
        self.output_topic = self.declare_parameter(
            'output_topic', '/cloud_registered').value
        self.target_frame = self.declare_parameter('target_frame', 'odom').value
        self.tf_timeout = self.declare_parameter('tf_timeout', 0.05).value

        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)
        self.pub = self.create_publisher(PointCloud2, self.output_topic, 10)
        self.group = ReentrantCallbackGroup()
        self.create_subscription(
            PointCloud2, self.input_topic, self.on_cloud,
            qos_profile_sensor_data, callback_group=self.group)
        self.get_logger().info(
            '%s -> %s (frame: %s)' % (
                self.input_topic, self.output_topic, self.target_frame))

    def on_cloud(self, msg):
        src_frame = msg.header.frame_id
        try:
            tf = self.buffer.lookup_transform(
                self.target_frame, src_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self.tf_timeout))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                'TF %s -> %s 不可用: %s' % (
                    src_frame, self.target_frame, exc),
                throttle_duration_sec=5.0)
            return

        pts = np.array(
            [[p[0], p[1], p[2]] for p in point_cloud2.read_points(
                msg, field_names=('x', 'y', 'z'))], dtype=np.float64)
        if pts.size == 0:
            return
        t = tf.transform.translation
        q = tf.transform.rotation
        R = quat_to_rot(q.x, q.y, q.z, q.w)
        out = pts @ R.T + np.array([t.x, t.y, t.z])

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = self.target_frame
        cloud = point_cloud2.create_cloud_xyz32(
            header, out.astype(np.float32))
        self.pub.publish(cloud)


def main():
    rclpy.init()
    node = CloudToOdom()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
