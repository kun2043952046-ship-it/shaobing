#!/usr/bin/env python3
"""合成 FAST-LIO 数据源 (带可调漂移): 用于验证锁定后的位姿修正稳定性。

参数:
  vx       前进速度 (m/s)
  drift    人为 LIO 漂移速度 (m/s), 连续累积 (中途修改不会瞬跳)
  yaw_deg  车体偏航 (deg), 用于验证旋转下的配准
点云: 墙 x=5 (y: -3..3) + 柱 (2, 2) 半径 0.3, 全部 z=0.0 (在 zmin..zmax 内),
      并整体随漂移平移, 模拟 LIO 位姿漂移导致的世界系点云错位。
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
import std_msgs.msg


class FakeDrift(Node):

    def __init__(self):
        super().__init__('fake_drift')
        self.declare_parameter('vx', 0.0)
        self.declare_parameter('drift', 0.0)
        self.declare_parameter('yaw_deg', 0.0)
        self.odom_pub = self.create_publisher(Odometry, '/Odometry', 10)
        self.cloud_pub = self.create_publisher(
            PointCloud2, '/cloud_registered', qos_profile_sensor_data)
        self.x = 0.0
        self.drift_acc = 0.0
        self.last = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.tick)

    def tick(self):
        now = self.get_clock().now()
        dt = (now - self.last).nanoseconds * 1e-9
        self.last = now

        vx = float(self.get_parameter('vx').value)
        drift = float(self.get_parameter('drift').value)
        yaw = math.radians(float(self.get_parameter('yaw_deg').value))

        self.x += vx * dt
        self.drift_acc += drift * dt

        stamp = now.to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.pose.pose.position.x = self.x + self.drift_acc
        odom.pose.pose.orientation.z = math.sin(yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.odom_pub.publish(odom)

        # 世界系点云 (随漂移整体平移): 墙 + 柱
        c, s = math.cos(yaw), math.sin(yaw)
        pts = []
        raw = []
        for i in range(300):
            raw.append((5.0, -3.0 + 6.0 * i / 299.0))
        for i in range(72):
            a = 2.0 * math.pi * i / 72.0
            raw.append((2.0 + 0.3 * math.cos(a), 2.0 + 0.3 * math.sin(a)))
        for (wx, wy) in raw:
            x = c * wx - s * wy + self.drift_acc
            y = s * wx + c * wy
            pts.append((x, y, 0.0))

        fields = []
        for name, offset in (('x', 0), ('y', 4), ('z', 8)):
            f = PointField()
            f.name = name
            f.offset = offset
            f.datatype = PointField.FLOAT32
            f.count = 1
            fields.append(f)

        header = std_msgs.msg.Header()
        header.stamp = stamp
        header.frame_id = 'map'
        self.cloud_pub.publish(pc2.create_cloud(header, fields, pts))


def main():
    rclpy.init()
    node = FakeDrift()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
