#!/usr/bin/env python3
"""STL 表面采样 → 先验 3D 点云（PCD），供 GICP scan-to-map 重定位使用

与 stl2pgm.py 同源：都从官方场地 STL 生成，几何即真实场地几何，
不经过仿真传感器，因此不存在 sim-to-real 的几何差异。

用法：
    python3 scripts/stl2pcd.py ../models/rm_field/meshes/sk_rmmap.stl -o map.pcd
    python3 scripts/stl2pcd.py mesh.stl -o map.pcd --density 800

默认值与当前仿真一致：
    位姿偏移 (-6.363, -4.343, 0)（同 models/rm_field/model.sdf）；
    密度 500 点/m^2（约 Mid-360 实际面密度的量级）；
    剔除 z < -0.05 的下朝面（底板下表面，雷达看不到）。
只依赖 numpy。
"""
import argparse
import os
import struct
import sys

import numpy as np


def load_stl(path):
    """读取 STL，返回 (n, 3, 3) 顶点数组"""
    with open(path, 'rb') as f:
        data = f.read()
    if len(data) < 84:
        raise ValueError('STL 文件过短')

    n = struct.unpack('<I', data[80:84])[0]
    if 84 + 50 * n == len(data):                  # 二进制 STL
        dtype = np.dtype([('normal', '<f4', 3), ('v', '<f4', (3, 3)), ('attr', '<u2')])
        return np.frombuffer(data, dtype=dtype, count=n, offset=84)['v'].astype(np.float64)

    verts = []                                    # ASCII STL
    for line in data.decode('ascii', 'ignore').splitlines():
        parts = line.split()
        if parts and parts[0] == 'vertex' and len(parts) == 4:
            verts.append([float(v) for v in parts[1:4]])
    if not verts or len(verts) % 3 != 0:
        raise ValueError('ASCII STL 顶点解析失败')
    return np.array(verts, dtype=np.float64).reshape(-1, 3, 3)


def sample_surface(tris, density, rng, z_keep_min=None):
    """按三角形面积加权均匀采样，返回 (m, 3) 点集"""
    v0, v1, v2 = tris[:, 0], tris[:, 1], tris[:, 2]
    area = 0.5 * np.linalg.norm(np.cross(v1 - v0, v2 - v0), axis=1)
    keep = area > 1e-9

    # 剔除整体位于 z_keep_min 以下的面（底板下表面），雷达看不到
    if z_keep_min is not None:
        keep &= tris[:, :, 2].max(axis=1) > z_keep_min

    idx = np.nonzero(keep)[0]
    if idx.size == 0:
        return np.zeros((0, 3))

    per_tri = np.maximum(np.round(area[idx] * density).astype(np.int64), 0)
    total = int(per_tri.sum())
    if total == 0:
        return np.zeros((0, 3))

    # 逐三角形按比例分配采样点（重心坐标均匀采样）
    tri_ids = np.repeat(idx, per_tri)
    counts = per_tri[per_tri > 0]
    u = rng.random(total)
    v = rng.random(total)
    # 保证每个三角形内部的 (u, v) 在同一个三角形内配对，这里用分段随机即可：
    # u、v 各自独立均匀，配合 sqrt 变换即得到三角形面内均匀分布
    s = np.sqrt(u)
    pts = ((1.0 - s)[:, None] * v0[tri_ids] +
           (s * (1.0 - v))[:, None] * v1[tri_ids] +
           (s * v)[:, None] * v2[tri_ids])
    del counts
    return pts


def write_pcd(path, pts):
    """写二进制 PCD (v0.7, FIELDS x y z, float32)"""
    n = len(pts)
    header = (
        '# .PCD v0.7 - Point Cloud Data file format\n'
        'VERSION 0.7\n'
        'FIELDS x y z\n'
        'SIZE 4 4 4\n'
        'TYPE F F F\n'
        'COUNT 1 1 1\n'
        f'WIDTH {n}\n'
        'HEIGHT 1\n'
        'VIEWPOINT 0 0 0 1 0 0 0\n'
        f'POINTS {n}\n'
        'DATA binary\n'
    )
    with open(path, 'wb') as f:
        f.write(header.encode('ascii'))
        f.write(np.asarray(pts, dtype='<f4').tobytes())


def main():
    ap = argparse.ArgumentParser(description='STL 表面采样生成先验 PCD')
    ap.add_argument('stl', help='STL 文件路径')
    ap.add_argument('-o', '--output', default='map.pcd', help='输出 PCD 路径')
    ap.add_argument('--density', type=float, default=500.0,
                    help='采样密度（点/m^2），默认 500')
    ap.add_argument('--pose', type=float, nargs=3, default=[-6.363, -4.343, 0.0],
                    metavar=('X', 'Y', 'Z'), help='模型位姿平移，默认 -6.363 -4.343 0.0')
    ap.add_argument('--z-keep-min', type=float, default=-0.05,
                    help='剔除最高点低于该高度的面（底板下表面），默认 -0.05；传 1e9 可关闭')
    ap.add_argument('--seed', type=int, default=0, help='随机种子，默认 0')
    args = ap.parse_args()

    tris = load_stl(args.stl) + np.asarray(args.pose, dtype=np.float64)
    rng = np.random.default_rng(args.seed)
    z_keep = None if args.z_keep_min > 1e8 else args.z_keep_min

    pts = sample_surface(tris, args.density, rng, z_keep)
    if len(pts) == 0:
        print('[ERROR] 采样结果为空', file=sys.stderr)
        return 1

    write_pcd(args.output, pts)

    print(f'{args.stl} -> {args.output}')
    print(f'  三角面 {len(tris)}，采样 {len(pts)} 点（密度 {args.density} 点/m^2）')
    print(f'  x {pts[:, 0].min():.3f}~{pts[:, 0].max():.3f}'
          f' / y {pts[:, 1].min():.3f}~{pts[:, 1].max():.3f}'
          f' / z {pts[:, 2].min():.3f}~{pts[:, 2].max():.3f}')
    print(f'  文件大小 {os.path.getsize(args.output) / 1024 / 1024:.2f} MB')
    return 0


if __name__ == '__main__':
    sys.exit(main())
