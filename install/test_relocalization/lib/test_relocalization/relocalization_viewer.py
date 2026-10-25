#!/usr/bin/env python3
# 精简版网页查看器 (继承自 test_draw_pgm_realtime/scripts/pgm_viewer.py 的显示面,
# 裁剪掉 Nav2 目标点/代价地图/占点/追击/运动控制等导航交互):
#   - 订阅当前图/历史图 (mono8 Image)、预览点云、/Odometry、/pose_correction
#     (点云与位姿箭头统一按 3D 重定位的 map->odom 修正量显示:
#      2D 节点的内部修正已在 launch 中关闭 apply_pose_correction=false)
#   - HTTP 端点: / 网页, /state, /pose, /image.png, /image_history.png,
#     /cloud, /set (参数热改), /refresh_history (刷新累计图), /lock (锁定基准),
#     /export_map (导出先验地图)
#   - 锁定基准时自动把历史点云队列导出为 map.pgm/map.yaml/map_clouds.bin
#     (固定 export_dir, 每次覆盖), 供 3D 重定位节点作先验地图;
#     解锁时自动清空历史点云 (放弃旧地图), 保证下次锁定导出纯新地图。
# 运行: ros2 run test_relocalization relocalization_viewer.py
# 网页: http://<ip>:1727/

import json
import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

import numpy as np
import cv2
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.parameter import Parameter
from rcl_interfaces.srv import GetParameters, SetParameters
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64


class RelocalizationViewer(Node):

    def __init__(self):
        super().__init__('relocalization_viewer')
        self.declare_parameter('port', 1727)
        self.declare_parameter('target_node', '/test_relocalization')
        # 3D 重定位节点 (锁定/解锁同步 enabled, 频率热调, fitness 显示)
        self.declare_parameter('target_node_3d', '/relocalization_3d')
        self.declare_parameter(
            'fitness_topic', '/relocalization_3d/fitness')
        # 先验地图导出目录 (固定, 每次导出覆盖旧文件; 3D 重定位节点从此读取)
        self.declare_parameter('export_dir', '/tmp/pgm_export')
        self.port = int(self.get_parameter('port').value)
        self.target_node = str(self.get_parameter('target_node').value)
        self.target_node_3d = str(
            self.get_parameter('target_node_3d').value)
        self.fitness_topic = str(
            self.get_parameter('fitness_topic').value)
        self.export_dir = str(self.get_parameter('export_dir').value)

        # 最新数据缓存 (各自锁保护)
        self._lock = threading.Lock()
        self._image = None            # 当前帧图 (mono8 numpy)
        self._image_history = None    # 历史累计图
        self._cloud = None            # 预览点云 [(x, y, z), ...]
        self._odom = None             # (x, y, yaw, stamp)
        self._correction = None       # (x, y, z, yaw, stamp) map->odom
        self._fitness3d = None        # 3D 配准 fitness (每次尝试都更新)
        self._locked = False          # 锁定基准 (重定位配准) 状态

        self.create_subscription(
            Image, '/realtime_pgm/image', self._on_image, 1)
        self.create_subscription(
            Image, '/realtime_pgm/image_history', self._on_image_history, 1)
        # 点云话题用 SensorDataQoS (best effort) —— 节点侧以该 QoS 发布,
        #   用默认 reliable 会因 QoS 不兼容收不到消息
        self.create_subscription(
            PointCloud2, '/realtime_pgm/cloud', self._on_cloud,
            qos_profile_sensor_data)
        self.create_subscription(
            Odometry, '/Odometry', self._on_odom, 10)
        self.create_subscription(
            PoseStamped, '/pose_correction', self._on_correction, 10)
        self.create_subscription(
            Float64, self.fitness_topic, self._on_fitness3d, 10)

        # 参数服务客户端 (与目标节点交互的唯一通道, 与 ros2 param set 等效)。
        #   持久持有, 异步调用 + 事件等待 —— 绝不从 HTTP 线程调用
        #   spin_until_future_complete (会与主执行器争抢 executor)。
        self._get_cli = self.create_client(
            GetParameters, self.target_node + '/get_parameters')
        self._set_cli = self.create_client(
            SetParameters, self.target_node + '/set_parameters')
        # 3D 重定位节点的参数服务 (enabled 同步 + rate_hz 热调)
        self._get_cli_3d = self.create_client(
            GetParameters, self.target_node_3d + '/get_parameters')
        self._set_cli_3d = self.create_client(
            SetParameters, self.target_node_3d + '/set_parameters')
        self._state_req_pending = False

        # 周期从目标节点同步锁定状态 (参数是唯一的命令通道)
        self.create_timer(1.0, self._sync_state)

        self._start_http()
        self.get_logger().info(
            '重定位查看器启动: http://0.0.0.0:%d (目标节点 %s)'
            % (self.port, self.target_node))

    # ---------------- ROS 回调 ----------------

    def _on_image(self, msg):
        img = self._mono8(msg)
        if img is not None:
            with self._lock:
                self._image = img

    def _on_image_history(self, msg):
        img = self._mono8(msg)
        if img is not None:
            with self._lock:
                self._image_history = img

    def _mono8(self, msg):
        if msg.encoding != 'mono8':
            return None
        img = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.width * msg.height != img.size:
            return None
        return img.reshape(msg.height, msg.width).copy()

    def _on_cloud(self, msg):
        pts = []
        for p in point_cloud2.read_points(
                msg, field_names=('x', 'y', 'z'), skip_nans=True):
            pts.append((float(p[0]), float(p[1]), float(p[2])))
            if len(pts) >= 20000:
                break
        with self._lock:
            self._cloud = pts

    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self._lock:
            self._odom = (
                msg.pose.pose.position.x, msg.pose.pose.position.y,
                yaw, msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

    def _on_correction(self, msg):
        q = msg.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self._lock:
            self._correction = (
                msg.pose.position.x, msg.pose.position.y,
                msg.pose.position.z, yaw,
                msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

    def _on_fitness3d(self, msg):
        # 3D 配准 fitness (<0 表示本次未收敛)
        with self._lock:
            self._fitness3d = float(msg.data)

    def _sync_state(self):
        # 从目标节点读 dynamic_obstacle_enabled (锁定 = 重定位基准冻结)
        if self._state_req_pending or not self._get_cli.service_is_ready():
            return
        req = GetParameters.Request()
        req.names = ['dynamic_obstacle_enabled']
        self._state_req_pending = True
        fut = self._get_cli.call_async(req)
        fut.add_done_callback(self._on_state)

    def _on_state(self, fut):
        self._state_req_pending = False
        try:
            resp = fut.result()
            if resp.values:
                with self._lock:
                    self._locked = bool(resp.values[0].bool_value)
        except Exception:
            pass

    # ---------------- 参数写入 (与 ros2 param set 等效) ----------------

    def set_param(self, name, value):
        """HTTP 线程调用: 异步发请求, 由主执行器跑回调, 用事件等待结果。"""
        return self._set_param_on(self._set_cli, name, value)

    def set_param_3d(self, name, value):
        """同上, 但作用于 3D 重定位节点 (enabled / rate_hz 等)。"""
        return self._set_param_on(self._set_cli_3d, name, value)

    def _set_param_on(self, cli, name, value):
        if not cli.service_is_ready():
            return False, '参数服务不可用 (节点未启动?)'
        req = SetParameters.Request()
        if isinstance(value, bool):
            p = Parameter(name, Parameter.Type.BOOL, value)
        elif isinstance(value, (int, float)):
            p = Parameter(name, Parameter.Type.DOUBLE, float(value))
        else:
            p = Parameter(name, Parameter.Type.STRING, str(value))
        req.parameters = [p.to_parameter_msg()]

        done = threading.Event()
        box = {}

        def _cb(fut):
            try:
                box['resp'] = fut.result()
            except Exception as e:      # noqa: BLE001
                box['error'] = str(e)
            done.set()

        fut = cli.call_async(req)
        fut.add_done_callback(_cb)
        if not done.wait(timeout=3.0):
            return False, '参数设置超时'
        if 'error' in box:
            return False, box['error']
        resp = box.get('resp')
        if resp is None or not resp.results:
            return False, '无响应'
        if not resp.results[0].successful:
            return False, resp.results[0].reason
        return True, 'ok'

    def get_params(self, names):
        """HTTP 线程调用: 异步读参数, 与 set_param 同模式 (事件等待)。"""
        return self._get_params_on(self._get_cli, names)

    def get_params_3d(self, names):
        """同上, 但读取 3D 重定位节点的参数。"""
        return self._get_params_on(self._get_cli_3d, names)

    def _get_params_on(self, cli, names):
        if not cli.service_is_ready():
            return None
        req = GetParameters.Request()
        req.names = list(names)

        done = threading.Event()
        box = {}

        def _cb(fut):
            try:
                box['resp'] = fut.result()
            except Exception as e:      # noqa: BLE001
                box['error'] = str(e)
            done.set()

        fut = cli.call_async(req)
        fut.add_done_callback(_cb)
        if not done.wait(timeout=2.0):
            return None
        resp = box.get('resp')
        if resp is None:
            return None
        out = {}
        for name, v in zip(names, resp.values):
            out[name] = v.double_value
        return out

    # ---------------- 先验地图导出 ----------------

    def export_prior_map(self):
        """让 C++ 节点把历史点云队列导出到 export_dir
        (map.pgm / map.yaml / map_clouds.bin / map.pcd), 每次覆盖旧文件。
        PCD 由节点最后写出, 本函数只等 bin 出现即返回 (PCD 稍后落盘)。
        返回 (ok, msg)。由 HTTP 线程调用 (事件等待模式, 同 set_param)。"""
        export_dir = self.export_dir
        try:
            os.makedirs(export_dir, exist_ok=True)
            # 先删旧文件: 既保证"每次覆盖上次导出", 又能以文件重新出现
            # 作为节点写完的判据 (避免读到上次的残留)
            #   map.pcd 由节点最后写出 (体素降采样较慢), 只清理不等它。
            for name in ('map.pgm', 'map.yaml', 'map_clouds.bin', 'map.pcd'):
                path = os.path.join(export_dir, name)
                if os.path.exists(path):
                    os.remove(path)
        except OSError as e:
            return False, '创建/清理导出目录失败: %s' % str(e)

        ok, reason = self.set_param('export_map_dir', export_dir)
        if not ok:
            return False, '设置导出目录失败: %s' % reason
        ok, reason = self.set_param('export_map_request', True)
        if not ok:
            return False, '导出请求失败: %s' % reason

        # 节点在渲染定时器周期内写出文件; map_clouds.bin 最后写,
        # 等它出现即表示三件套写完 (重定位只需要 bin, 但 pgm/yaml 一并保留)
        bin_path = os.path.join(export_dir, 'map_clouds.bin')
        deadline = time.time() + 5.0
        while time.time() < deadline:
            if os.path.exists(bin_path):
                self.get_logger().info(
                    '先验地图已导出到 %s (覆盖旧文件)' % export_dir)
                return True, export_dir
            time.sleep(0.05)
        return False, '导出超时: 未生成 map_clouds.bin'

    # ---------------- HTTP 服务器 ----------------

    def _start_http(self):
        node = self

        class Handler(BaseHTTPRequestHandler):

            def log_message(self, fmt, *args):
                pass

            def _send(self, code, ctype, body):
                self.send_response(code)
                self.send_header('Content-Type', ctype)
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(body)

            def _json(self, obj):
                self._send(200, 'application/json',
                           json.dumps(obj).encode('utf-8'))

            def do_GET(self):
                parsed = urlparse(self.path)
                path = parsed.path
                qs = parse_qs(parsed.query)

                if path == '/':
                    self._send(200, 'text/html; charset=utf-8',
                               PAGE_HTML.encode('utf-8'))
                elif path == '/state':
                    with node._lock:
                        locked = node._locked
                        has_cloud = node._cloud is not None
                        has_img = node._image_history is not None
                    self._json({
                        'locked': locked,
                        'has_cloud': has_cloud,
                        'has_image': has_img,
                    })
                elif path == '/pose':
                    with node._lock:
                        odom = node._odom
                        corr = node._correction
                        fit3d = node._fitness3d
                    self._json({
                        'odom': None if odom is None else {
                            'x': odom[0], 'y': odom[1], 'yaw': odom[2]},
                        'correction': None if corr is None else {
                            'x': corr[0], 'y': corr[1], 'z': corr[2],
                            'yaw': corr[3]},
                        'fitness3d': fit3d,
                    })
                elif path in ('/image.png', '/image_history.png'):
                    with node._lock:
                        img = (node._image if path == '/image.png'
                               else node._image_history)
                    if img is None:
                        self._send(404, 'text/plain', b'no image yet')
                        return
                    # mono8: 0=占据(黑), 205=未知(灰), 其余=空闲
                    # 节点发布的 mono8 图已是标准图像行序 (第 0 行在顶部 = ymax),
                    # 与网页端"画布顶部 = +y"的换算一致, 故直接编码, 不可翻转:
                    # 一旦翻转, 底图就会相对点云上下镜像 (表现为"黑图和点云对不上")。
                    vis = np.full(img.shape, 255, dtype=np.uint8)
                    vis[img == 0] = 0        # 占据 -> 黑
                    vis[img == 205] = 205     # 未知 -> 灰
                    ok, buf = cv2.imencode('.png', vis)
                    if ok:
                        self._send(200, 'image/png', buf.tobytes())
                    else:
                        self._send(500, 'text/plain', b'encode failed')
                elif path == '/cloud':
                    with node._lock:
                        pts = node._cloud
                    if pts is None:
                        self._json({'points': []})
                        return
                    self._json({
                        'points': [[p[0], p[1], p[2]] for p in pts]})
                elif path == '/meta':
                    # 地图几何 (供网页坐标换算): 直接读节点参数, 避免网页端
                    #   写死 ±20 m 导致改了地图范围后底图与点云错位。
                    vals = node.get_params(
                        ['resolution', 'xmin', 'ymin', 'xmax', 'ymax'])
                    if vals is None:
                        self._json({'ok': False})
                        return
                    with node._lock:
                        img = node._image_history
                    vals['ok'] = True
                    vals['w'] = 0 if img is None else int(img.shape[1])
                    vals['h'] = 0 if img is None else int(img.shape[0])
                    self._json(vals)
                elif path == '/params':
                    vals = node.get_params(['zmin', 'zmax'])
                    if vals is None:
                        self._json({'ok': False})
                    else:
                        self._json({'ok': True, **vals})
                elif path == '/set':
                    name = qs.get('name', [''])[0]
                    value = qs.get('value', [''])[0]
                    if not name:
                        self._json({'ok': False, 'reason': 'missing name'})
                        return
                    if value.lower() in ('true', 'false'):
                        v = value.lower() == 'true'
                    else:
                        try:
                            v = float(value)
                        except ValueError:
                            v = value
                    ok, reason = node.set_param(name, v)
                    self._json({'ok': ok, 'reason': reason})
                elif path == '/refresh_history':
                    ok, reason = node.set_param('clear_history', True)
                    self._json({'ok': ok, 'reason': reason})
                elif path == '/params3d':
                    vals = node.get_params_3d(['rate_hz', 'enabled'])
                    if vals is None:
                        self._json({'ok': False})
                    else:
                        self._json({'ok': True, **vals})
                elif path == '/set3d':
                    # 3D 重定位节点参数热调 (rate_hz 等)
                    name = qs.get('name', [''])[0]
                    value = qs.get('value', [''])[0]
                    if not name:
                        self._json({'ok': False, 'reason': 'missing name'})
                        return
                    if value.lower() in ('true', 'false'):
                        v = value.lower() == 'true'
                    else:
                        try:
                            v = float(value)
                        except ValueError:
                            v = value
                    ok, reason = node.set_param_3d(name, v)
                    self._json({'ok': ok, 'reason': reason})
                elif path == '/lock':
                    enable = qs.get('enable', ['true'])[0].lower() == 'true'
                    ok, reason = node.set_param(
                        'dynamic_obstacle_enabled', enable)
                    with node._lock:
                        node._locked = enable if ok else node._locked
                    resp = {'ok': ok, 'reason': reason}
                    # 锁定成功 -> 自动导出先验地图 (覆盖式, 静默不下载),
                    # 导出完成后 3D 重定位才有可用先验, 即"此时才开始工作"
                    if ok and enable:
                        exp_ok, exp_msg = node.export_prior_map()
                        resp['exported'] = exp_ok
                        resp['export_msg'] = exp_msg
                    # 解锁成功 -> 立即清空历史点云: 放弃旧地图从头累积,
                    # 保证下次锁定导出的是纯新地图 (换场地重建场景)
                    elif ok and not enable:
                        clr_ok, clr_reason = node.set_param(
                            'clear_history', True)
                        resp['cleared'] = clr_ok
                        resp['clear_msg'] = clr_reason
                    # 同步 3D 重定位节点门控: 锁定=启用, 解锁=停用。
                    #   3D 节点未启动时不算失败 (仅提示), 不影响 2D 流程。
                    if ok:
                        e_ok, e_msg = node.set_param_3d('enabled', enable)
                        resp['enabled3d'] = e_ok
                        resp['enabled3d_msg'] = e_msg
                    self._json(resp)
                elif path == '/export_map':
                    ok, msg = node.export_prior_map()
                    self._json({'ok': ok, 'reason': msg})
                else:
                    self._send(404, 'text/plain', b'not found')

        server = ThreadingHTTPServer(('0.0.0.0', node.port), Handler)
        threading.Thread(target=server.serve_forever, daemon=True).start()


# ---------------- 网页 (历史图 + 点云 + 位姿叠加) ----------------

PAGE_HTML = r'''<!DOCTYPE html>
<html><head><meta charset="utf-8">
<title>重定位查看器</title>
<style>
body { margin:0; background:#1e1e1e; color:#ddd;
       font-family: monospace; }
#bar { padding:8px 12px; background:#111; }
button { margin:2px 6px; padding:4px 10px; }
/* 双层画布叠加: 底层地图 (脏矩形增量更新) + 上层点云/位姿 (独立重绘)。
   容器背景色即地图未到达时的占位色, 避免出现黑屏/白屏空窗。 */
#wrap { position:relative; width:800px; height:800px; margin:8px;
        background:#205; }
#map, #ovl { position:absolute; left:0; top:0; }
/* 右侧 3D 点云画布 (纯 Canvas 软件投影, 与 2D 视图并排) */
#views { display:flex; align-items:flex-start; }
#wrap3 { position:relative; width:800px; height:800px; margin:8px;
         background:#101018; }
#cv3 { position:absolute; left:0; top:0; cursor:grab; }
#cv3:active { cursor:grabbing; }
#hint3 { position:absolute; left:8px; top:6px; color:#688;
         font-size:12px; pointer-events:none; }
.locked { color:#7f7; }
</style></head>
<body>
<div id="bar">
  <b>重定位查看器</b>
  <button onclick="act('/refresh_history')">刷新累计图</button>
  <button id="lockBtn" onclick="toggleLock()">锁定基准(重定位)</button>
  <button onclick="exportMap()">导出地图</button>
  <span id="expmsg" style="margin-left:4px;color:#8cf"></span>
  <span id="legend" style="margin-left:6px">图例: 历史图 灰(黑=占据/灰=未知)
    · 点云 <b id="legendCloud" style="color:#00d23c">●</b>
    · 位姿 <b style="color:#0af">&#9650;</b></span>
  <span id="status">...</span>
</div>
<div id="bar">
  zmin
  <input type="range" id="zminR" min="-3" max="0" step="0.01" value="-2.25"
         style="width:140px;vertical-align:middle">
  <input type="number" id="zminN" min="-3" max="0" step="0.01" value="-2.25"
         style="width:70px">
  zmax
  <input type="range" id="zmaxR" min="0" max="1.5" step="0.01" value="0.22"
         style="width:140px;vertical-align:middle">
  <input type="number" id="zmaxN" min="0" max="1.5" step="0.01" value="0.22"
         style="width:70px">
  <button onclick="resetZDefaults()" title="恢复配置文件默认值 zmin=-2.25 / zmax=0.22">
    恢复初始值</button>
  <span style="margin-left:14px">3D配准频率</span>
  <input type="number" id="rate3d" min="0.1" max="10" step="0.1" value="2.0"
         style="width:60px">
  <span>Hz</span>
  <button onclick="applyRate3d()" title="热更新 3D 重定位节点的配准频率">
    应用</button>
  <span id="fit3d" style="margin-left:10px;color:#8cf"></span>
</div>
<div id="views">
<div id="wrap">
  <canvas id="map" width="800" height="800"></canvas>
  <canvas id="ovl" width="800" height="800"></canvas>
</div>
<div id="wrap3">
  <canvas id="cv3" width="800" height="800"></canvas>
  <div id="hint3">3D 实时点云: 拖拽旋转 · 滚轮缩放 · 双击复位</div>
</div>
</div>
<script>
let locked = false, pose = null;
// 地图几何: 从 /meta 读节点真实配置 (resolution/xmin/ymin/xmax/ymax),
//   不再写死 ±20 m —— 否则一旦改动地图范围, 底图与点云就会错位。
//   换算: 画布顶部 = +ymax, 左侧 = xmin (与节点图像行序一致)。
const META = { xmin: -20, ymin: -20, xmax: 20, ymax: 20, w: 800, h: 800 };
function w2p(x, y) {
  const px = (x - META.xmin) / (META.xmax - META.xmin) * 800;
  const py = (META.ymax - y) / (META.ymax - META.ymin) * 800;
  return [px, py];
}
async function syncMeta() {
  try {
    const m = await (await fetch('/meta')).json();
    if (m.ok && m.xmax > m.xmin && m.ymax > m.ymin) {
      Object.assign(META, m);
    }
  } catch (e) {}
}

async function poll() {
  try {
    const st = await (await fetch('/state')).json();
    locked = st.locked;
    document.getElementById('lockBtn').textContent =
      locked ? '解锁基准' : '锁定基准(重定位)';
    document.getElementById('lockBtn').className = locked ? 'locked' : '';
    // 图例随锁定状态变色: 锁定=鲜亮绿, 解锁=暗绿 (与点云实际配色一致)
    document.getElementById('legendCloud').style.color =
      locked ? '#00ff00' : '#00d23c';
    document.getElementById('status').textContent =
      (locked ? ' [基准已锁定: 位姿修正运行中]' : '') +
      (st.has_image ? '' : ' 等待地图...');
    pose = await (await fetch('/pose')).json();
    // 3D 配准置信度: fitness <0 表示本次未收敛, null 表示节点未运行
    const f = pose.fitness3d;
    document.getElementById('fit3d').textContent =
      (f === null || f === undefined) ? '3D: 未运行'
        : (f < 0 ? '3D fitness: 未收敛' : '3D fitness: ' + f.toFixed(3));
  } catch (e) {}
}
setInterval(poll, 1000);

// ==================== 增量渲染 ====================
// 旧实现每秒先 ctx.fillRect 清空整块画布, 再等新 PNG 异步解码完成才重绘,
// 空窗期即为"短暂黑屏"。现改为:
//   底层 map: 永不整块清屏; 新图先画到离屏画布, 与上一帧像素逐点比对求出
//             "变化像素的包围盒"(脏矩形), 只把该矩形拷上屏; 整幅无变化则
//             一次绘制都不做 -> 无闪烁、无空窗, 且开销随变化面积而非整幅。
//   上层 ovl: 点云与位姿画在独立透明画布上, 只清它自己, 不影响地图层。
const W = 800, H = 800;
const mapCtx = document.getElementById('map').getContext('2d');
const ovlCtx = document.getElementById('ovl').getContext('2d');
// 离屏画布: 解码新图 + 取像素做逐点比对
const off = document.createElement('canvas');
off.width = W; off.height = H;
const offCtx = off.getContext('2d', { willReadFrequently: true });
let prevPixels = null;    // 上一帧像素 (比对基线)
let mapReady = false;     // 是否已有底图 (首次需全量上屏)
let fetching = false;     // 防止拉图请求堆积

// 求两帧像素的差异包围盒; 完全一致返回 null (无需任何绘制)
function diffBox(a, b) {
  let minX = W, minY = H, maxX = -1, maxY = -1;
  for (let y = 0; y < H; y++) {
    const row = y * W * 4;
    for (let x = 0; x < W; x++) {
      const i = row + x * 4;
      if (a[i] !== b[i] || a[i + 1] !== b[i + 1] || a[i + 2] !== b[i + 2]) {
        if (x < minX) minX = x;
        if (x > maxX) maxX = x;
        if (y < minY) minY = y;
        if (y > maxY) maxY = y;
      }
    }
  }
  if (maxX < 0) return null;
  // 外扩 1 px, 覆盖抗锯齿/边界像素
  const x = Math.max(0, minX - 1);
  const y = Math.max(0, minY - 1);
  const w = Math.min(W - x, maxX - minX + 3);
  const h = Math.min(H - y, maxY - minY + 3);
  return [x, y, w, h];
}

async function refreshMap() {
  if (fetching) return;
  fetching = true;
  try {
    // 用 blob + createImageBitmap: 解码在后台完成, 不像 <img> 那样
    // 先占位后补画, 天然没有空窗
    const resp = await fetch('/image_history.png?t=' + Date.now());
    if (!resp.ok) return;
    const bmp = await createImageBitmap(await resp.blob());
    offCtx.clearRect(0, 0, W, H);
    offCtx.drawImage(bmp, 0, 0, W, H);
    if (bmp.close) bmp.close();

    const cur = offCtx.getImageData(0, 0, W, H);
    if (!mapReady || !prevPixels) {
      mapCtx.drawImage(off, 0, 0);          // 首次: 全量上屏一次
      mapReady = true;
    } else {
      const box = diffBox(prevPixels.data, cur.data);
      if (box) {
        // 增量: 只重绘变化区域 (一次 GPU blit), 不清屏
        mapCtx.drawImage(off,
          box[0], box[1], box[2], box[3],   // 源矩形
          box[0], box[1], box[2], box[3]);  // 目标矩形 (同一位置)
      }
      // box === null -> 整幅未变, 不绘制 -> 无任何闪烁
    }
    prevPixels = cur;
  } catch (e) {
    // 网络/解码失败保持上一帧画面, 绝不清屏
  } finally {
    fetching = false;
  }
}

// 覆盖层: 点云 + 位姿 (500ms 一次); 只清透明上层, 地图层不受影响
//
// 点云配色 (与灰度底图区分, 统一绿色系, 不再使用红色):
//   解锁状态: 半透明暗绿细点 —— 与历史图叠加时保持轻量, 不遮挡底图细节
//   锁定基准后: 鲜亮绿 #0f0, 略大的点 —— 一眼可与冻结的灰色历史图区分开,
//              也能看出当前处于"以冻结基准做位姿配准"的状态
const CLOUD_UNLOCKED = 'rgba(0, 210, 60, 0.45)';
const CLOUD_LOCKED = 'rgba(0, 255, 0, 0.9)';
const CLOUD_SIZE_UNLOCKED = 1.5;
const CLOUD_SIZE_LOCKED = 2;

async function drawOverlay() {
  ovlCtx.clearRect(0, 0, W, H);
  try {
    const d = await (await fetch('/cloud')).json();
    lastCloud = d.points;   // 缓存供 3D 视图用 (每帧替换, 不累计)
    ovlCtx.fillStyle = locked ? CLOUD_LOCKED : CLOUD_UNLOCKED;
    const size = locked ? CLOUD_SIZE_LOCKED : CLOUD_SIZE_UNLOCKED;
    for (const p of d.points) {
      const q = applyCorrToPoint(p);
      const [px, py] = w2p(q[0], q[1]);
      ovlCtx.fillRect(px, py, size, size);
    }
  } catch (e) {}
  drawPose(ovlCtx);
  draw3d();
}
// 位姿必须落在 map 系才能与底图对齐: /Odometry 是 odom 系原始位姿,
// 需按节点内部的同一公式复合 /pose_correction (p_map = R * p_odom + t)。
// 未锁定基准时修正量为恒等, 结果与直接用里程计一致。
function mapPose() {
  if (!pose || !pose.odom) return null;
  let x = pose.odom.x, y = pose.odom.y, yaw = pose.odom.yaw;
  // 解锁态不施加修正 (与点云显示 applyCorrToPoint 保持一致)
  const c = locked ? pose.correction : null;
  if (c) {
    const cy = Math.cos(c.yaw), sy = Math.sin(c.yaw);
    const nx = cy * x - sy * y + c.x;
    const ny = sy * x + cy * y + c.y;
    x = nx; y = ny; yaw = yaw + c.yaw;
  }
  return { x: x, y: y, yaw: yaw };
}
// 点云显示修正: 点云话题 (/realtime_pgm/cloud) 现在是 odom 系原始点
//   (2D 节点侧的内部修正已在 launch 中关闭 apply_pose_correction=false),
//   这里用 3D 重定位节点的 map->odom 修正量搬到 map 系 —— 与位姿箭头
//   (mapPose) 用同一个修正量, 保证"点云贴图"和"箭头位置"永远一致。
//   未收到修正量 (3D 节点未运行) 时原样显示, 与未锁定时的行为相同。
function applyCorrToPoint(p) {
  if (!locked) return p;   // 解锁态不做任何修正 (与 mapPose 的语义一致)
  const c = (pose && pose.correction) || null;
  if (!c) return p;
  const cy = Math.cos(c.yaw), sy = Math.sin(c.yaw);
  return [cy * p[0] - sy * p[1] + c.x,
          sy * p[0] + cy * p[1] + c.y,
          p[2] + (c.z || 0)];
}
function drawPose(ctx) {
  const p = mapPose();
  if (!p) return;
  const [px, py] = w2p(p.x, p.y);
  ctx.save();
  ctx.translate(px, py);
  ctx.rotate(-p.yaw);
  ctx.fillStyle = '#0af';
  ctx.beginPath();
  ctx.moveTo(10, 0); ctx.lineTo(-6, 6); ctx.lineTo(-6, -6);
  ctx.closePath(); ctx.fill();
  ctx.restore();
}
// ==================== 3D 实时点云视图 ====================
// 纯 Canvas 软件投影 (零依赖, 离线可用): 鼠标拖拽绕 z 轴 360° 旋转 +
// 俯仰, 滚轮缩放, 双击复位。数据复用 /cloud (map 系, 每帧替换不累计),
// 观察中心实时跟随机器人位姿。
const cv3 = document.getElementById('cv3');
const ctx3 = cv3.getContext('2d');
let lastCloud = [];   // drawOverlay 每次拉取后更新
const CAM3_DEFAULT = { yaw: 0.6, pitch: 0.9, dist: 30 };
const cam3 = { yaw: CAM3_DEFAULT.yaw, pitch: CAM3_DEFAULT.pitch,
               dist: CAM3_DEFAULT.dist, f: 600 };
// 深度雾: 按深度分 4 档预设样式, 避免逐点拼字符串
const CLOUD3_STYLES = [
  'rgba(0,255,120,0.95)', 'rgba(0,230,110,0.70)',
  'rgba(0,200,100,0.45)', 'rgba(0,170,90,0.25)'];

// 世界点 (map 系) -> 屏幕坐标; 返回 [sx, sy, depth], 相机背后返回 null
function project3(x, y, z, cx, cy) {
  const vx = x - cx, vy = y - cy, vz = z;
  const cz = Math.cos(cam3.yaw), sz = Math.sin(cam3.yaw);
  const rx = cz * vx - sz * vy;
  const ry = sz * vx + cz * vy;
  const cp = Math.cos(cam3.pitch), sp = Math.sin(cam3.pitch);
  const ry2 = cp * ry - sp * vz;      // 深度方向
  const rz2 = sp * ry + cp * vz;      // 屏幕竖直方向 (高度)
  const depth = ry2 + cam3.dist;
  if (depth < 0.5) return null;
  const s = cam3.f / depth;
  return [400 + rx * s, 400 - rz2 * s, depth];
}

function draw3d() {
  ctx3.fillStyle = '#101018';
  ctx3.fillRect(0, 0, 800, 800);
  const p = mapPose();
  const cx = p ? p.x : 0, cy = p ? p.y : 0;
  for (const pt of lastCloud) {
    const q = applyCorrToPoint(pt);   // 与 2D 视图/箭头同一修正量
    const pr = project3(q[0], q[1], q[2], cx, cy);
    if (!pr) continue;
    if (pr[0] < 0 || pr[0] >= 800 || pr[1] < 0 || pr[1] >= 800) continue;
    ctx3.fillStyle = CLOUD3_STYLES[Math.min(3, (pr[2] / 15) | 0)];
    ctx3.fillRect(pr[0], pr[1], 2, 2);
  }
  // 机器人位姿: 蓝色三角 (与 2D 视图同一符号)
  if (p) {
    const c = Math.cos(p.yaw), s = Math.sin(p.yaw);
    const tri = [[0.6, 0], [-0.3, 0.3], [-0.3, -0.3]];
    ctx3.fillStyle = '#0af';
    ctx3.beginPath();
    let started = false, ok = true;
    for (const [dx, dy] of tri) {
      const pr = project3(p.x + c * dx - s * dy,
                          p.y + s * dx + c * dy, 0, cx, cy);
      if (!pr) { ok = false; break; }
      if (!started) { ctx3.moveTo(pr[0], pr[1]); started = true; }
      else ctx3.lineTo(pr[0], pr[1]);
    }
    if (ok && started) { ctx3.closePath(); ctx3.fill(); }
  }
}

// ---- 交互: 拖拽旋转 / 滚轮缩放 / 双击复位 ----
let drag3 = false, drag3x = 0, drag3y = 0;
cv3.addEventListener('mousedown', e => {
  drag3 = true; drag3x = e.clientX; drag3y = e.clientY;
});
window.addEventListener('mouseup', () => { drag3 = false; });
cv3.addEventListener('mousemove', e => {
  if (!drag3) return;
  cam3.yaw -= (e.clientX - drag3x) * 0.006;   // 水平拖拽: 绕 z 360°
  cam3.pitch = Math.max(-1.5, Math.min(1.5,
    cam3.pitch + (e.clientY - drag3y) * 0.006));
  drag3x = e.clientX; drag3y = e.clientY;
  draw3d();
});
cv3.addEventListener('wheel', e => {
  e.preventDefault();
  cam3.dist = Math.max(2, Math.min(120,
    cam3.dist * (e.deltaY > 0 ? 1.1 : 0.9)));
  draw3d();
}, { passive: false });
cv3.addEventListener('dblclick', () => {
  cam3.yaw = CAM3_DEFAULT.yaw;
  cam3.pitch = CAM3_DEFAULT.pitch;
  cam3.dist = CAM3_DEFAULT.dist;
  draw3d();
});

// 地图层 1 s 拉一次 (保持原节奏), 覆盖层 500 ms 重绘一次
setInterval(refreshMap, 1000);
setInterval(drawOverlay, 500);
refreshMap();
drawOverlay();
draw3d();
// 地图几何 5 s 同步一次 (网页滑条改地图范围后换算随之更新)
setInterval(syncMeta, 5000);
syncMeta();

async function act(url) {
  const r = await (await fetch(url)).json();
  if (!r.ok) alert('失败: ' + r.reason);
  poll();
}
// 锁定基准: 锁定成功后服务端自动覆盖式导出先验地图 (静默, 不下载),
// 导出结果在 expmsg 短暂提示; 解锁不触发导出。
async function toggleLock() {
  const enable = !locked;
  const r = await (await fetch('/lock?enable=' + enable)).json();
  if (!r.ok) { alert('失败: ' + r.reason); poll(); return; }
  // 3D 节点未启动时不算失败, 但要在提示里说明
  if (enable) {
    showExpMsg((r.exported
      ? '已锁定, 先验地图已导出'
      : '已锁定, 但先验地图导出失败: ' + r.export_msg) +
      (r.enabled3d ? ', 3D 重定位已启动' : ' [3D节点未启动]'),
      r.exported);
  } else {
    showExpMsg((r.cleared
      ? '已解锁, 历史已清空, 重新累积中'
      : '已解锁, 但历史清空失败: ' + r.clear_msg) +
      (r.enabled3d === false ? ' [3D节点未启动]' : ''), r.cleared);
  }
  poll();
}
// 独立导出按钮: 与锁定时自动导出同一目录, 覆盖旧文件
async function exportMap() {
  showExpMsg('正在导出...', true);
  const r = await (await fetch('/export_map')).json();
  showExpMsg(r.ok ? '已导出到 ' + r.reason : '导出失败: ' + r.reason, r.ok);
}
let expMsgTimer = null;
function showExpMsg(text, ok) {
  const el = document.getElementById('expmsg');
  el.textContent = text;
  el.style.color = ok ? '#8cf' : '#f66';
  clearTimeout(expMsgTimer);
  expMsgTimer = setTimeout(() => { el.textContent = ''; }, 5000);
}

// ---- zmin/zmax 热改: 滑条粗调 + 数字输入精调, 双向联动, 防抖下发 ----
const pushTimers = {};
function pushParam(name, v) {
  clearTimeout(pushTimers[name]);
  pushTimers[name] = setTimeout(async () => {
    const r = await (await fetch(
      '/set?name=' + name + '&value=' + v)).json();
    if (!r.ok) alert(name + ' 设置失败: ' + r.reason);
  }, 150);
}
function setPair(name, v, push) {
  const r = document.getElementById(name + 'R');
  const n = document.getElementById(name + 'N');
  r.value = v; n.value = v;
  if (push) pushParam(name, v);
}
function bindPair(name) {
  const r = document.getElementById(name + 'R');
  const n = document.getElementById(name + 'N');
  r.oninput = () => setPair(name, parseFloat(r.value), true);
  n.onchange = () => {
    // 数字输入精调: 超出滑条范围时夹取
    let v = parseFloat(n.value);
    if (isNaN(v)) return;
    v = Math.min(parseFloat(r.max), Math.max(parseFloat(r.min), v));
    setPair(name, v, true);
  };
}
bindPair('zmin');
bindPair('zmax');

// ---- 3D 配准频率热调 (relocalization_3d 节点 rate_hz) ----
async function applyRate3d() {
  const v = parseFloat(document.getElementById('rate3d').value);
  if (isNaN(v) || v < 0.1 || v > 10) {
    showExpMsg('频率需在 0.1 ~ 10 Hz 之间', false);
    return;
  }
  const r = await (await fetch('/set3d?name=rate_hz&value=' + v)).json();
  showExpMsg(r.ok ? '3D 配准频率已设为 ' + v + ' Hz'
                  : '频率设置失败: ' + r.reason, r.ok);
}
// 加载时回读 3D 节点当前频率 (节点未启动则保持默认显示)
(async () => {
  try {
    const p = await (await fetch('/params3d')).json();
    if (p.ok && p.rate_hz) {
      document.getElementById('rate3d').value = p.rate_hz;
    }
  } catch (e) {}
})();

// ---- 一键恢复配置文件默认值 (config/relocalization.yaml) ----
// 点击后同时把 zmin/zmax 拉回默认值 (滑条 + 数字框同步) 并立即下发节点。
// 与节点冷启动值一致, 不受中途手动拖动或命令行改参的影响。
const Z_DEFAULTS = { zmin: -2.25, zmax: 0.22 };
function resetZDefaults() {
  setPair('zmin', Z_DEFAULTS.zmin, true);
  setPair('zmax', Z_DEFAULTS.zmax, true);
}

// 加载时经 GetParameters 回读节点当前值, 滑条初始位置与之同步
(async () => {
  try {
    const p = await (await fetch('/params')).json();
    if (p.ok) {
      setPair('zmin', p.zmin, false);
      setPair('zmax', p.zmax, false);
    }
  } catch (e) {}
})();
</script>
</body></html>
'''


def main(args=None):
    rclpy.init(args=args)
    node = RelocalizationViewer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
