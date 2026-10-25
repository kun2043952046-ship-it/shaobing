#!/usr/bin/env python3
"""点云高度直方图：把 /livox/lidar 的点云变换到 base_footprint（地面=0），
统计各高度段的点数分布，用来决定 pointcloud_to_laserscan 的切片带。

用法（仿真跑起来后）：
    python3 scripts/height_hist.py
每 2 秒打印一次直方图，Ctrl+C 退出。
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
import tf2_ros

Z_MIN = -0.5      # 统计范围下限（米，相对地面）
Z_MAX = 2.0       # 统计范围上限
BINS = 25         # 分 25 段 → 每段 0.1m


class HeightHist(Node):
    def __init__(self):
        super().__init__('height_hist')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.sub = self.create_subscription(
            PointCloud2, '/livox/lidar', self.cb, qos_profile_sensor_data)
        self.last_print = 0.0
        self.get_logger().info('等待点云与 TF ... (每 2 秒输出一次直方图)')

    def cb(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_print < 2.0:
            return

        # 点到 base_footprint 的变换
        try:
            tf = self.tf_buffer.lookup_transform(
                'base_footprint', msg.header.frame_id, Time())
        except Exception as exc:
            self.get_logger().warn(f'TF 未就绪: {exc}', throttle_duration_sec=2.0)
            return

        # 取 x/y/z（不依赖字段顺序与 padding）
        off = {f.name: f.offset for f in msg.fields}
        if not {'x', 'y', 'z'} <= set(off):
            self.get_logger().error('点云缺 x/y/z 字段')
            return
        dt = np.dtype({'names': ['x', 'y', 'z'], 'formats': ['<f4'] * 3,
                       'offsets': [off['x'], off['y'], off['z']],
                       'itemsize': msg.point_step})
        a = np.frombuffer(msg.data, dtype=dt)
        pts = np.stack([a['x'], a['y'], a['z']], axis=1).astype(np.float64)

        # 旋转 + 平移（四元数 → 旋转矩阵）
        t, q = tf.transform.translation, tf.transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        R = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        p = pts @ R.T + np.array([t.x, t.y, t.z])
        zz = p[:, 2]

        # 水平距离（用于排除车身：半径 < 0.4m 的点）
        rr = np.hypot(p[:, 0], p[:, 1])
        mask_far = rr > 0.40

        hist, edges = np.histogram(zz[mask_far], bins=BINS, range=(Z_MIN, Z_MAX))
        self.last_print = now
        print('\n' + '=' * 72)
        print(f'点云 {len(zz)} 点 | 地面(=base_footprint 原点)为 0 | 已排除车身 0.4m 内 '
              f'({int((~mask_far).sum())} 点)')
        print(f'Z 范围（全部）: {zz.min():+.2f} ~ {zz.max():+.2f} m')
        print('-' * 72)
        hmax = max(hist.max(), 1)
        for h, e0, e1 in zip(hist, edges[:-1], edges[1:]):
            bar = '#' * int(50 * h / hmax)
            print(f'  {e0:+.2f} ~ {e1:+.2f} m | {h:6d} {bar}')
        print('-' * 72)
        print('提示：0.00 附近的高柱 = 地面；0.12~0.22 是底盘；')
        print('      切片带应选「有大量点、且下界在地面之上」的那一段（通常是墙体）')
        print('=' * 72)


def main():
    rclpy.init()
    node = HeightHist()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
