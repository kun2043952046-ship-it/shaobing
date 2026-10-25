#!/usr/bin/env python3
"""采样 /pose_correction, 输出 CSV: t x y yaw_deg, 并给出稳定性统计。
用法: python3 sample_corr.py <duration_sec>
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class Sampler(Node):

    def __init__(self):
        super().__init__('sample_corr')
        self.rows = []
        self.create_subscription(PoseStamped, '/pose_correction', self.cb, 10)

    def cb(self, m):
        q = m.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.rows.append((
            time.time(), m.pose.position.x, m.pose.position.y,
            math.degrees(yaw)))


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0
    rclpy.init()
    node = Sampler()
    end = time.time() + duration
    while rclpy.ok() and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)

    rows = node.rows
    t0 = rows[0][0] if rows else 0.0
    for (t, x, y, yaw) in rows:
        print("CSV %.2f %+.3f %+.3f %+.2f" % (t - t0, x, y, yaw))

    if rows:
        xs = [r[1] for r in rows]
        ys = [r[2] for r in rows]
        print("N=%d" % len(rows))
        print("x: min=%+.3f max=%+.3f span=%.3f" %
              (min(xs), max(xs), max(xs) - min(xs)))
        print("y: min=%+.3f max=%+.3f span=%.3f" %
              (min(ys), max(ys), max(ys) - min(ys)))
        # 相邻样本跳变量 (抖动指标)
        jumps = [math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1])
                 for i in range(1, len(xs))]
        if jumps:
            print("相邻跳变: max=%.3f mean=%.3f" %
                  (max(jumps), sum(jumps) / len(jumps)))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
