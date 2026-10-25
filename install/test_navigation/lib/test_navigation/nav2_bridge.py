#!/usr/bin/env python3
# nav2_bridge.py
#
# 实时 PGM 链路 -> Nav2 的桥接节点:
#   1. 订阅 /realtime_pgm/image_history (mono8: 0=占据, 其余=未观测),
#      并从 test_navigation 参数服务同步 xmin/ymin/resolution,
#      转成 nav_msgs/OccupancyGrid 发布到 /map (transient_local, 供 Nav2
#      全局代价地图 static_layer 订阅; 网页"刷新累计图"后 /map 自动重建)
#   2. 订阅 /Odometry (FAST-LIO 输出的雷达位姿), 发布 TF: odom -> base_link,
#      并叠加 lidar_to_base 外参使 base_link 对齐车体中心 (与 pgm_viewer 一致)
#   3. 订阅 /pose_correction 发布动态 TF: map -> odom; 未收到校正前使用恒等变换。
#      校正消息由 C++ 节点的 2D 栅格配准发布，
#      桥接始终沿用最近一次有效值。**修正量按完整 SE(3) 处理**:
#      只取 yaw 会丢掉修正量里的姿态与高度分量。
#
# 运行: ros2 run test_navigation nav2_bridge.py
# (realtime_pgm.launch.py 会自动启动本节点)

import math
import threading
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.parameter import ParameterType
from rcl_interfaces.srv import GetParameters

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import Image
from tf2_ros import TransformBroadcaster


def quat_rotate(quat, vec):
    """用四元数 (x, y, z, w) 旋转三维向量 vec=(x, y, z), 返回三元组。

    实现即 q * v * q^-1 (单位四元数), 不引入 tf_transformations 依赖。
    """
    qx, qy, qz, qw = quat
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n < 1e-12:
        return (vec[0], vec[1], vec[2])
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n

    vx, vy, vz = vec
    # t = 2 * (q_vec × v)
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    # v' = v + qw * t + q_vec × t
    return (
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    )


class Nav2Bridge(Node):

    def __init__(self):
        super().__init__('nav2_bridge')

        self.declare_parameter('odom_topic', '/Odometry')
        self.declare_parameter('pose_correction_topic', '/pose_correction')
        self.declare_parameter('history_image_topic',
                               '/realtime_pgm/image_history')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('target_node', '/test_navigation')
        # 雷达->车体中心外参 (与 pgm_viewer.py 默认值一致, 单位: m)
        self.declare_parameter('lidar_to_base_x', 0.239)
        self.declare_parameter('lidar_to_base_y', -0.158)

        odom_topic = self.get_parameter('odom_topic').value
        correction_topic = self.get_parameter('pose_correction_topic').value
        history_topic = self.get_parameter('history_image_topic').value
        map_topic = self.get_parameter('map_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.target_node = self.get_parameter('target_node').value
        self.lidar_to_base_x = float(
            self.get_parameter('lidar_to_base_x').value)
        self.lidar_to_base_y = float(
            self.get_parameter('lidar_to_base_y').value)

        self.map_meta = {'res': None, 'xmin': None, 'ymin': None}
        self.latest_image = None
        self.image_lock = threading.Lock()
        # 位姿修正量 (map->odom, SE(3)): 保存**完整 6 自由度**。
        #   修正量按 SE(3) 处理, 若这里只取 yaw,
        #   姿态与高度修正会在 TF 这一环被悄悄丢掉。
        self.correction_lock = threading.Lock()
        self.correction_trans = [0.0, 0.0, 0.0]      # x, y, z
        self.correction_quat = [0.0, 0.0, 0.0, 1.0]  # x, y, z, w
        self.correction_received = False

        # /map 发布节流状态: 避免把每帧全量占据栅格(约 1.3 MB, 5 Hz)反复推给
        # Nav2 代价地图, 导致 static_layer 每次都要逐格比对 + 重建。
        # 地图几何(尺寸/分辨率/原点)变化时立即发布; 否则限制为 map_min_period
        # 秒一次, 且仅在占据格数变化超过 map_min_change_ratio 时才重发。
        self.map_min_period = float(
            self.declare_parameter('map_min_period', 1.0).value)
        self.map_min_change_ratio = float(
            self.declare_parameter('map_min_change_ratio', 0.005).value)
        self.map_min_subscribers = int(
            self.declare_parameter('map_min_subscribers', 0).value)
        self.last_map_time = 0.0
        self.last_map_occ = -1
        self.last_map_key = None

        # Nav2 static_layer 默认以 transient_local + reliable 订阅 /map,
        # 发布端必须提供 transient_local 才能建立连接
        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, map_topic, map_qos)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.image_sub = self.create_subscription(
            Image, history_topic, self.on_image, qos_profile_sensor_data)
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.on_odom, qos_profile_sensor_data)
        self.correction_sub = self.create_subscription(
            PoseStamped, correction_topic, self.on_pose_correction, 10)

        # 周期同步 C++ 节点的 xmin/ymin/resolution (网页滑条可改)
        self.param_cli = self.create_client(
            GetParameters, self.target_node + '/get_parameters')
        self.meta_future = None
        self.last_meta_fetch = 0.0
        # rclpy 里只有 create_timer (create_wall_timer 是 rclcpp 的 API)
        self.meta_timer = self.create_timer(1.0, self.meta_timer_cb)

        self.get_logger().info(
            'Nav2 桥接节点启动: history=%s -> %s (frame: %s->%s->%s), '
            'odom=%s, correction=%s, lidar_to_base=(%.2f, %.2f) m'
            % (history_topic, map_topic, self.map_frame, self.odom_frame,
               self.base_frame, odom_topic, correction_topic,
               self.lidar_to_base_x, self.lidar_to_base_y))
        self.get_logger().info(
            '/map 发布节流: 几何变化立即发, 否则 >= %.1f s/次 且占据格数变化 '
            '> %.1f%%' % (self.map_min_period,
                          self.map_min_change_ratio * 100.0))

    # ---------- 校正消息: map -> odom 修正量 (PoseStamped, SE(3)) ----------
    def on_pose_correction(self, msg):
        p = msg.pose.position
        q = msg.pose.orientation
        with self.correction_lock:
            self.correction_trans = [float(p.x), float(p.y), float(p.z)]
            self.correction_quat = [
                float(q.x), float(q.y), float(q.z), float(q.w)]
            self.correction_received = True

    def publish_map_odom(self, stamp):
        with self.correction_lock:
            trans = list(self.correction_trans)
            quat = list(self.correction_quat)

        msg = TransformStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.map_frame
        msg.child_frame_id = self.odom_frame
        msg.transform.translation.x = trans[0]
        msg.transform.translation.y = trans[1]
        msg.transform.translation.z = trans[2]
        msg.transform.rotation.x = quat[0]
        msg.transform.rotation.y = quat[1]
        msg.transform.rotation.z = quat[2]
        msg.transform.rotation.w = quat[3]
        self.tf_broadcaster.sendTransform(msg)

    # ---------- 里程计 -> TF odom -> base_link ----------
    def on_odom(self, msg):
        pos = msg.pose.pose.position
        q = msg.pose.pose.orientation

        # /Odometry 是雷达原点位姿; lidar_to_base_x/y 是雷达到车体中心的向量
        # (表达在雷达系)。这里用**完整四元数**旋转该向量, 并保留完整姿态与 z:
        #   map->base_link = (map->odom 修正量) ⊗ (odom->base_link)
        # 只有链路两端都按 6DoF 处理, 上式的旋转才能与里程计的姿态漂移相互抵消;
        # 若这里沿用"平面 yaw + z=0"的旧写法, 抵消不成立, Nav2 看到的 2D 位姿
        # 会残留一段与倾斜相关的偏移。
        off_x, off_y, off_z = quat_rotate(
            (q.x, q.y, q.z, q.w),
            (self.lidar_to_base_x, self.lidar_to_base_y, 0.0))

        t = TransformStamped()
        if msg.header.stamp.sec or msg.header.stamp.nanosec:
            t.header.stamp = msg.header.stamp
        else:
            t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = float(pos.x + off_x)
        t.transform.translation.y = float(pos.y + off_y)
        t.transform.translation.z = float(pos.z + off_z)
        t.transform.rotation.x = float(q.x)
        t.transform.rotation.y = float(q.y)
        t.transform.rotation.z = float(q.z)
        t.transform.rotation.w = float(q.w)
        self.tf_broadcaster.sendTransform(t)
        self.publish_map_odom(t.header.stamp)

    # ---------- 历史占据图 -> /map (OccupancyGrid) ----------
    def on_image(self, msg):
        if msg.encoding != 'mono8':
            self.get_logger().warning(
                '不支持的图像编码: %s' % msg.encoding,
                throttle_duration_sec=5.0)
            return
        img = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.width * msg.height != img.size:
            self.get_logger().warning('图像尺寸与数据长度不符，丢弃本帧')
            return
        img = img.reshape(msg.height, msg.width).copy()
        with self.image_lock:
            self.latest_image = img
        if self.map_meta['res'] is not None:
            self.publish_map(img)

    def publish_map(self, img):
        meta = self.map_meta
        if meta['res'] is None or meta['xmin'] is None or \
                meta['ymin'] is None:
            return
        if self.map_pub.get_subscription_count() == 0:
            return

        height, width = img.shape

        # ---------- 发布节流: 只在"地图内容真的变了"时重发 ----------
        # 图像语义: 0=占据(C++ 已按 dilate 膨胀), 其余(205)=未观测
        occ_mask = (img == 0)
        occ_count = int(occ_mask.sum())
        key = (width, height, float(meta['res']),
               float(meta['xmin']), float(meta['ymin']))
        now = time.monotonic()
        geom_changed = (key != self.last_map_key)
        if not geom_changed:
            # 几何未变: 限频 + 占据格数无明显变化则跳过, 避免 5 Hz 全量重发
            if now - self.last_map_time < self.map_min_period:
                return
            if self.last_map_occ > 0 and abs(
                    occ_count - self.last_map_occ) < \
                    self.map_min_change_ratio * self.last_map_occ:
                return

        self.last_map_key = key
        self.last_map_time = now
        self.last_map_occ = occ_count

        grid = OccupancyGrid()
        grid.header.frame_id = self.map_frame
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.info.resolution = key[2]
        grid.info.width = int(width)
        grid.info.height = int(height)
        grid.info.origin.position.x = key[3]
        grid.info.origin.position.y = key[4]
        grid.info.origin.orientation.w = 1.0

        # 图像第 0 行在顶部 (对应 ymax), OccupancyGrid 行 0 对应 origin(ymin)
        # 与 pgm_viewer 原有 A* 约定一致, 未观测区域视为自由(0)
        occ = np.where(occ_mask, 100, 0).astype(np.int8)
        grid.data = np.flipud(occ).reshape(-1).tolist()
        self.map_pub.publish(grid)

    # ---------- 从 C++ 节点同步 xmin/ymin/resolution ----------
    def meta_timer_cb(self):
        if self.meta_future is not None:
            if not self.meta_future.done():
                return
            future, self.meta_future = self.meta_future, None
            try:
                result = future.result()
            except Exception as e:  # noqa: B902
                self.get_logger().warning(
                    '读取地图参数失败: %s' % str(e),
                    throttle_duration_sec=10.0)
                return

            vals = result.values

            def dbl(i):
                return vals[i].double_value \
                    if vals[i].type == ParameterType.PARAMETER_DOUBLE \
                    else None

            meta = {'res': dbl(0), 'xmin': dbl(1), 'ymin': dbl(2)}
            if meta['res'] is None or meta['xmin'] is None or \
                    meta['ymin'] is None:
                return
            if meta != self.map_meta:
                self.map_meta = meta
                self.get_logger().info(
                    '地图元数据: resolution=%.3f origin=(%.2f, %.2f)'
                    % (meta['res'], meta['xmin'], meta['ymin']))
                with self.image_lock:
                    img = self.latest_image
                if img is not None:
                    self.publish_map(img)
            return

        # 已同步过则每 5 s 刷新一次 (防止外部 ros2 param set 改动)
        if self.map_meta['res'] is not None and \
                time.monotonic() - self.last_meta_fetch < 5.0:
            return
        if not self.param_cli.service_is_ready():
            return

        req = GetParameters.Request()
        req.names = ['resolution', 'xmin', 'ymin']
        self.meta_future = self.param_cli.call_async(req)
        self.last_meta_fetch = time.monotonic()


def main():
    rclpy.init()
    node = Nav2Bridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
