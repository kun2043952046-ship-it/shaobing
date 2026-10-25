#!/usr/bin/env python3
# diag_logger.py
#
# 位姿诊断记录节点: 专治"摇晃过后地图上小车位置漂移 / 无法重定位"一类问题。
#
# 现有的 cloud_frames.csv (C++ 内建调试日志) 只记录了"点云"的统计量,
# 缺少两样定位排查最需要的东西:
#   1. /Odometry 的姿态角 (roll/pitch/yaw) —— 判断摇晃时 FAST-LIO 位姿怎么错的
#   2. /pose_correction 的修正量       —— 判断 2D 配准是成功、被拒, 还是跳飞
# 本节点把这两路数据按同一时间基准落到一张 CSV 里, 便于事后对齐分析。
#
# 记录内容 (每收到一帧 /Odometry 写一行):
#   stamp          消息时间戳 (秒, 保留到纳秒)
#   dt             与上一帧的时间差 (s)
#   odom_x/y/z     原始里程计位置 (m, odom 系)
#   roll/pitch/yaw 原始里程计姿态 (deg, ZYX)
#   vx/vy/wz       线速度 x/y 与角速度 z (m/s, rad/s)
#   dxy            与上一帧的位置差 (m)  -> 帧间跳变检测
#   dyaw_deg       与上一帧的偏航差 (deg)
#   corr_x/y/yaw   /pose_correction 当前值 (m, deg)
#   dcorr          修正量相对上一帧的变化 (m) -> 配准振荡检测
#   corr_age_s     距最近一次收到 /pose_correction 的时间 (s)
#   map_x/y/yaw    叠加修正后的"地图位姿" (即网页上看到的位置)
#   flag           异常标签: - / odom_jump / yaw_jump / corr_jump (可组合)
#
# 同时: 终端对超阈值的帧打印 WARN, 便于实时肉眼盯。
#
# 运行: ros2 run test_draw_pgm_realtime diag_logger.py
# (realtime_pgm.launch.py 会随 with_diag:=true 自动启动本节点)

import math
import os
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


def quat_to_euler(x, y, z, w):
    """四元数 -> ZYX 欧拉角 (roll, pitch, yaw), 单位 rad。"""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def wrap_deg(deg):
    """把角度归一化到 (-180, 180]。"""
    while deg > 180.0:
        deg -= 360.0
    while deg <= -180.0:
        deg += 360.0
    return deg


class DiagLogger(Node):

    def __init__(self):
        super().__init__('diag_logger')

        # ---------- 话题 ----------
        self.declare_parameter('odom_topic', '/Odometry')
        self.declare_parameter('pose_correction_topic', '/pose_correction')

        # ---------- 落盘 ----------
        self.declare_parameter('log_dir', '')
        self.declare_parameter('log_file', 'diag_frames.csv')
        # True = 追加到已有文件; False = 每次启动覆盖 (与 cloud_frames.csv 一致)
        self.declare_parameter('append', False)

        # ---------- 跳变告警阈值 (0 或负数 = 关闭该项) ----------
        self.declare_parameter('odom_jump_m', 0.30)
        self.declare_parameter('odom_jump_yaw_deg', 10.0)
        self.declare_parameter('corr_jump_m', 0.30)
        # WARN 节流间隔 (s): 避免摇晃期间刷屏
        self.declare_parameter('warn_throttle_sec', 1.0)
        # 周期性状态打印间隔 (s): 0 = 关闭
        self.declare_parameter('status_period_sec', 5.0)

        odom_topic = self.get_parameter('odom_topic').value
        corr_topic = self.get_parameter('pose_correction_topic').value

        self.odom_jump_m = float(self.get_parameter('odom_jump_m').value)
        self.odom_jump_yaw_deg = float(
            self.get_parameter('odom_jump_yaw_deg').value)
        self.corr_jump_m = float(self.get_parameter('corr_jump_m').value)
        self.warn_throttle_sec = float(
            self.get_parameter('warn_throttle_sec').value)
        self.status_period_sec = float(
            self.get_parameter('status_period_sec').value)

        log_dir = str(self.get_parameter('log_dir').value)
        log_file = str(self.get_parameter('log_file').value)

        if not log_dir:
            log_dir = self._default_log_dir()

        self.csv_path = os.path.join(log_dir, log_file)
        self.log = self._open_log(self.csv_path)

        # ---------- 状态 ----------
        self.prev_stamp = None
        self.prev_x = None
        self.prev_y = None
        self.prev_yaw = None
        self.corr_x = 0.0
        self.corr_y = 0.0
        self.corr_yaw = 0.0
        self.corr_stamp = None
        self.prev_corr_x = None
        self.prev_corr_y = None
        self.warn_last = 0.0
        self.status_last = 0.0
        self.rows = 0
        self.jumps = 0

        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.on_odom, qos_profile_sensor_data)
        self.corr_sub = self.create_subscription(
            PoseStamped, corr_topic, self.on_correction,
            QoSProfile(depth=10,
                       reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.VOLATILE))

        self.get_logger().info(
            '位姿诊断记录: 已启动, odom=%s, correction=%s -> %s'
            % (odom_topic, corr_topic, self.csv_path))

    # ------------------------------------------------------------------
    def _default_log_dir(self):
        """默认写到本包 share 下的 log/ (与 C++ 调试日志同目录)。"""
        try:
            from ament_index_python.packages import (
                get_package_share_directory)
            return os.path.join(
                get_package_share_directory('test_draw_pgm_realtime'), 'log')
        except Exception:  # noqa: B902
            return '/tmp'

    def _open_log(self, path):
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            append = bool(self.get_parameter('append').value)
            handle = open(path, 'a' if append else 'w', encoding='utf-8')
        except OSError as exc:  # noqa: B902
            self.get_logger().error(
                '无法打开诊断日志 %s (%s), 仅保留终端输出' % (path, exc))
            return None

        if not append or os.path.getsize(path) == 0:
            handle.write(
                'stamp,dt,odom_x,odom_y,odom_z,'
                'roll_deg,pitch_deg,yaw_deg,vx,vy,wz,'
                'dxy,dyaw_deg,'
                'corr_x,corr_y,corr_yaw_deg,dcorr,corr_age_s,'
                'map_x,map_y,map_yaw_deg,flag\n')
            handle.flush()
        return handle

    # ------------------------------------------------------------------
    def on_correction(self, msg):
        q = msg.pose.orientation
        self.corr_yaw = quat_to_euler(q.x, q.y, q.z, q.w)[2]
        self.corr_x = float(msg.pose.position.x)
        self.corr_y = float(msg.pose.position.y)
        self.corr_stamp = time.time()

    def on_odom(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        roll, pitch, yaw = quat_to_euler(q.x, q.y, q.z, q.w)

        vx = float(msg.twist.twist.linear.x)
        vy = float(msg.twist.twist.linear.y)
        wz = float(msg.twist.twist.angular.z)

        # ---- 帧间变化量 ----
        if self.prev_x is None:
            dt = 0.0
            dxy = 0.0
            dyaw_deg = 0.0
        else:
            dt = (stamp - self.prev_stamp
                  if self.prev_stamp is not None else 0.0)
            dxy = math.hypot(p.x - self.prev_x, p.y - self.prev_y)
            dyaw_deg = wrap_deg(
                math.degrees(yaw - self.prev_yaw))

        flags = []
        if self.odom_jump_m > 0.0 and dxy > self.odom_jump_m:
            flags.append('odom_jump')
        if (self.odom_jump_yaw_deg > 0.0 and
                abs(dyaw_deg) > self.odom_jump_yaw_deg):
            flags.append('yaw_jump')

        # ---- 修正量变化 ----
        if self.prev_corr_x is None:
            dcorr = 0.0
        else:
            dcorr = math.hypot(self.corr_x - self.prev_corr_x,
                               self.corr_y - self.prev_corr_y)
        if self.corr_jump_m > 0.0 and dcorr > self.corr_jump_m:
            flags.append('corr_jump')

        corr_age = (time.time() - self.corr_stamp
                    if self.corr_stamp is not None else -1.0)

        # ---- 叠加修正后的地图位姿 (与 C++ transformPointByCorrection 同式) ----
        c = math.cos(self.corr_yaw)
        s = math.sin(self.corr_yaw)
        map_x = c * p.x - s * p.y + self.corr_x
        map_y = s * p.x + c * p.y + self.corr_y
        map_yaw_deg = wrap_deg(math.degrees(yaw + self.corr_yaw))

        flag = '+'.join(flags) if flags else '-'

        if self.log is not None:
            self.log.write(
                '%.9f,%.6f,%.6f,%.6f,%.6f,'
                '%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,'
                '%.4f,%.3f,'
                '%.4f,%.4f,%.3f,%.4f,%.3f,'
                '%.4f,%.4f,%.3f,%s\n' % (
                    stamp, dt, p.x, p.y, p.z,
                    math.degrees(roll), math.degrees(pitch),
                    math.degrees(yaw), vx, vy, wz,
                    dxy, dyaw_deg,
                    self.corr_x, self.corr_y,
                    math.degrees(self.corr_yaw), dcorr, corr_age,
                    map_x, map_y, map_yaw_deg, flag))
            self.log.flush()

        self.rows += 1
        if flags:
            self.jumps += 1
            now = time.time()
            if now - self.warn_last >= self.warn_throttle_sec:
                self.warn_last = now
                self.get_logger().warn(
                    '位姿异常[%s]: odom=(%.2f, %.2f) dxy=%.2f m dyaw=%.1f deg, '
                    'roll=%.1f pitch=%.1f deg, corr=(%.2f, %.2f) dcorr=%.2f m, '
                    'map=(%.2f, %.2f)' % (
                        flag, p.x, p.y, dxy, dyaw_deg,
                        math.degrees(roll), math.degrees(pitch),
                        self.corr_x, self.corr_y, dcorr, map_x, map_y))

        if self.status_period_sec > 0.0:
            now = time.time()
            if now - self.status_last >= self.status_period_sec:
                self.status_last = now
                self.get_logger().info(
                    '位姿状态: odom=(%.2f, %.2f, %.2f) rpy=(%.1f, %.1f, %.1f) deg, '
                    'corr=(%.2f, %.2f, %.1f deg) age=%.1f s, '
                    'map=(%.2f, %.2f), 累计 %d 帧 / %d 异常' % (
                        p.x, p.y, p.z,
                        math.degrees(roll), math.degrees(pitch),
                        math.degrees(yaw),
                        self.corr_x, self.corr_y,
                        math.degrees(self.corr_yaw), corr_age,
                        map_x, map_y, self.rows, self.jumps))

        self.prev_stamp = stamp
        self.prev_x = p.x
        self.prev_y = p.y
        self.prev_yaw = yaw
        self.prev_corr_x = self.corr_x
        self.prev_corr_y = self.corr_y

    def destroy_node(self):
        if self.log is not None:
            self.log.flush()
            self.log.close()
            self.log = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DiagLogger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl+C 或 launch 结束时的外部关闭: 安静退出, 不打印无意义堆栈
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
