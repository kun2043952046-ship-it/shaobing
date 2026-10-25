#!/usr/bin/env python3
"""实时点云 -> 2D 占据栅格 (/realtime_grid), 仅作局部避障/观察参考

为什么单独做:
  全局规划底图用静态 map.pgm (/map); 这里把实时点云累积成一张 2D 栅格另开话题,
  方便观察"当前真实障碍"与先验图的差异, 不参与 Nav2 规划 (规划仍用 /map)。

坐标: 点云本来就在 odom 系 (/cloud_registered), 栅格也发在 odom 系, 不需要 TF。
      RViz 里会自动用 map->odom 变换到 map 显示。

用法:
  ros2 run fishbot_description realtime_grid.py
  ros2 run fishbot_description realtime_grid.py --ros-args -p resolution:=0.05 -p extent:=15.0
  清空重来: ros2 param set /realtime_grid reset true
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


class RealtimeGrid(Node):
    def __init__(self):
        super().__init__('realtime_grid')
        self.cloud_topic = self.declare_parameter(
            'cloud_topic', '/cloud_registered').value
        self.output_topic = self.declare_parameter(
            'output_topic', '/realtime_grid').value
        self.resolution = self.declare_parameter('resolution', 0.05).value
        self.extent = self.declare_parameter('extent', 15.0).value
        self.zmin = self.declare_parameter('zmin', -0.30).value
        self.zmax = self.declare_parameter('zmax', 2.00).value
        self.min_hits = self.declare_parameter('min_hits', 2).value
        self.publish_rate = self.declare_parameter('publish_rate', 1.0).value
        self.declare_parameter('reset', False)

        n = int(2 * self.extent / self.resolution)
        self.n = n
        self.hits = np.zeros((n, n), dtype=np.uint16)

        self.sub = self.create_subscription(
            PointCloud2, self.cloud_topic, self.on_cloud,
            qos_profile_sensor_data)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(OccupancyGrid, self.output_topic, qos)
        self.timer = self.create_timer(1.0 / max(1e-3, self.publish_rate),
                                       self.publish)
        self.add_on_set_parameters_callback(self.on_param)
        self.get_logger().info(
            '%s -> %s  %dx%d @ %.2fm  范围 ±%.1fm' % (
                self.cloud_topic, self.output_topic, n, n, self.resolution,
                self.extent))

    def on_param(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == 'reset' and p.value:
                self.hits[:] = 0
                self.get_logger().info('已清空实时栅格')
        return SetParametersResult(successful=True)

    def on_cloud(self, msg):
        pts = np.array([[p[0], p[1], p[2]] for p in pc2.read_points(
            msg, field_names=('x', 'y', 'z'))], dtype=np.float32)
        if pts.size == 0:
            return
        pts = pts[np.isfinite(pts).all(axis=1)]
        pts = pts[(pts[:, 2] >= self.zmin) & (pts[:, 2] <= self.zmax)]
        if pts.size == 0:
            return
        half = self.extent
        ix = ((pts[:, 0] + half) / self.resolution).astype(np.int32)
        iy = ((pts[:, 1] + half) / self.resolution).astype(np.int32)
        ok = (ix >= 0) & (ix < self.n) & (iy >= 0) & (iy < self.n)
        np.add.at(self.hits, (iy[ok], ix[ok]), 1)

    def publish(self):
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = 'odom'
        grid.info.resolution = float(self.resolution)
        grid.info.width = self.n
        grid.info.height = self.n
        grid.info.origin.position.x = -self.extent
        grid.info.origin.position.y = -self.extent
        grid.info.origin.orientation.w = 1.0
        data = np.full(self.n * self.n, -1, dtype=np.int8)
        occ = self.hits >= max(1, self.min_hits)
        data[occ.reshape(-1)] = 100
        grid.data = data.tolist()
        self.pub.publish(grid)


def main():
    rclpy.init()
    node = RealtimeGrid()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
