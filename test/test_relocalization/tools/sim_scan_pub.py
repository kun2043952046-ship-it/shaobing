#!/usr/bin/env python3
"""离线生成与 Gazebo Mid-360 等效的点云帧，并发布为 /livox/lidar（验证用）

用途：没有 GPU / 显示（gpu_ray 传感器不出点云）或不想开整套仿真时，
用官方场地 STL 做射线投射，得到与 mid360.xacro 参数一致的点云帧
（720x16、垂直视场 ±29.5° 经上仰 22.5° 后等效 -7°~+52°、0.1~40m、2mm 噪声），
喂给 relocalization_3d 验证 GICP 的收敛性 / 收敛域。

用法：
    # 只生成一帧并保存，便于离线检查
    python3 tools/sim_scan_pub.py --stl <field.stl> --no-pub --save /tmp/scan.pcd
    # 按 2Hz 持续发布，配合 relocalization_3d 使用
    python3 tools/sim_scan_pub.py --stl <field.stl> --hz 2

注意：点云在 livox_frame 下（与真实驱动一致）。
      节点侧需把 odom_frame 设成 livox_frame、broadcast_tf 设 false，
      否则会与 robot_state_publisher 抢 livox_frame 的父坐标系。
"""
import argparse
import math
import struct
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def load_stl(path):
    with open(path, 'rb') as f:
        data = f.read()
    n = struct.unpack('<I', data[80:84])[0]
    if 84 + 50 * n == len(data):
        dtype = np.dtype([('normal', '<f4', 3), ('v', '<f4', (3, 3)), ('attr', '<u2')])
        return np.frombuffer(data, dtype=dtype, count=n, offset=84)['v'].astype(np.float64)
    verts = []
    for line in data.decode('ascii', 'ignore').splitlines():
        parts = line.split()
        if parts and parts[0] == 'vertex' and len(parts) == 4:
            verts.append([float(v) for v in parts[1:4]])
    return np.array(verts, dtype=np.float64).reshape(-1, 3, 3)


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def raycast(tris, origin, R_wl, h_samples, v_samples, vfov, rmin, rmax, noise, rng):
    """Möller–Trumbore 逐三角形向量化求交，返回 livox_frame 下的点云 (N,3)"""
    az = np.linspace(-math.pi, math.pi, h_samples, endpoint=False)
    el = np.linspace(-vfov / 2.0, vfov / 2.0, v_samples)
    AZ, EL = np.meshgrid(az, el)
    d_l = np.stack([
        np.cos(EL) * np.cos(AZ), np.cos(EL) * np.sin(AZ), np.sin(EL)
    ], axis=-1).reshape(-1, 3)
    d_w = d_l @ R_wl.T

    best_t = np.full(len(d_w), np.inf)
    v0 = tris[:, 0]
    e1 = tris[:, 1] - tris[:, 0]
    e2 = tris[:, 2] - tris[:, 0]

    for i in range(len(tris)):
        pvec = np.cross(d_w, e2[i])
        det = pvec @ e1[i]
        ok = np.abs(det) > 1e-9
        if not ok.any():
            continue
        inv_det = np.zeros_like(det)
        inv_det[ok] = 1.0 / det[ok]
        tvec = origin - v0[i]
        u = (pvec @ tvec) * inv_det
        qvec = np.cross(tvec, e1[i])
        v = (d_w @ qvec) * inv_det
        t = (e2[i] @ qvec) * inv_det
        hit = ok & (u >= 0.0) & (v >= 0.0) & (u + v <= 1.0) & (t > rmin) & (t < rmax)
        best_t = np.where(hit & (t < best_t), t, best_t)

    valid = np.isfinite(best_t)
    t = best_t[valid]
    d = d_l[valid]
    if noise > 0.0:
        t = t + rng.normal(0.0, noise, t.shape[0])
    return (d * t[:, None]).astype(np.float64)


def write_pcd(path, pts):
    n = len(pts)
    header = (
        '# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n'
        'FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
        f'WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA binary\n'
    )
    with open(path, 'wb') as f:
        f.write(header.encode('ascii'))
        f.write(np.asarray(pts, dtype='<f4').tobytes())


class ScanPublisher(Node):
    def __init__(self, pts, frame, topic, hz):
        super().__init__('sim_scan_pub')
        self.pts = np.asarray(pts, dtype='<f4')
        self.frame = frame
        self.pub = self.create_publisher(PointCloud2, topic, 10)
        self.create_timer(1.0 / hz, self.tick)
        self.get_logger().info(f'{len(self.pts)} 点 -> {topic} ({hz} Hz)')

    def tick(self):
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame
        msg.height = 1
        msg.width = self.pts.shape[0]
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * msg.width
        msg.is_dense = True
        msg.data = self.pts.tobytes()
        self.pub.publish(msg)


def main():
    ap = argparse.ArgumentParser(description='STL 射线投射 → /livox/lidar')
    ap.add_argument('--stl', required=True)
    ap.add_argument('--pose', type=float, nargs=3, default=[-5.0, 3.0, 0.0],
                    help='车体在世界系的位置 x y z')
    ap.add_argument('--yaw', type=float, default=-1.5708, help='车体朝向 (rad)')
    ap.add_argument('--laser', type=float, nargs=3, default=[-0.239, -0.158, 0.45],
                    help='雷达在车体系下的偏移')
    ap.add_argument('--pitch', type=float, default=-0.3927,
                    help='livox_frame 相对父 link 的 Y 旋转 (rad)')
    ap.add_argument('--h-samples', type=int, default=720)
    ap.add_argument('--v-samples', type=int, default=16)
    ap.add_argument('--vfov', type=float, default=1.0298, help='垂直视场 (rad)')
    ap.add_argument('--range-min', type=float, default=0.1)
    ap.add_argument('--range-max', type=float, default=40.0)
    ap.add_argument('--noise', type=float, default=0.002)
    ap.add_argument('--offset', type=float, nargs=3, default=[-6.363, -4.343, 0.0],
                    help='场地模型位姿偏移（同 model.sdf）')
    ap.add_argument('--frame', default='livox_frame')
    ap.add_argument('--topic', default='/livox/lidar')
    ap.add_argument('--hz', type=float, default=1.0)
    ap.add_argument('--save', default='', help='保存一份 PCD')
    ap.add_argument('--no-pub', action='store_true', help='只生成不发布')
    args = ap.parse_args()

    tris = load_stl(args.stl) + np.asarray(args.offset, dtype=np.float64)
    R_wb = rot_z(args.yaw)
    origin = np.asarray(args.pose, dtype=np.float64) + R_wb @ np.asarray(args.laser)
    R_wl = R_wb @ rot_y(args.pitch)

    t0 = time.time()
    pts = raycast(tris, origin, R_wl, args.h_samples, args.v_samples,
                  args.vfov, args.range_min, args.range_max, args.noise,
                  np.random.default_rng(0))
    print(f'射线投射完成: {len(pts)} 点, 用时 {time.time() - t0:.2f}s')
    print(f'  雷达原点(世界) = {origin.round(3)}')
    print(f'  车体位姿 yaw = {args.yaw:.4f} rad ({math.degrees(args.yaw):.1f} deg)')

    if args.save:
        write_pcd(args.save, pts)
        print(f'  已保存 {args.save}')

    if args.no_pub:
        return 0

    rclpy.init()
    node = ScanPublisher(pts, args.frame, args.topic, args.hz)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
