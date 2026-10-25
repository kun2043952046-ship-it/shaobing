#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Livox_Calibration.py
#
# 云台雷达外参标定 (雷达 -> 车体中心), 即求 lidar_to_base_x / lidar_to_base_y
#
# 原理:
#   云台绕"车体中心竖轴"旋转时, 车体本身不动, 因此雷达原点会在世界系里
#   绕车体中心画一个圆, 圆心就是车体中心 (也是云台旋转轴).
#   设 L = 车体中心在雷达坐标系下的坐标 (即待求的 lidar_to_base),
#   第 i 帧雷达位姿为 (p_i, psi_i), 车体中心世界坐标 C 未知但固定, 恒有:
#       p_i + R(psi_i) * L = C
#   展开后该式对 (Lx, Ly, Cx, Cy) 是线性的:
#        x 分量:  -cos(psi)*Lx + sin(psi)*Ly + Cx = p_x
#        y 分量:  -sin(psi)*Lx - cos(psi)*Ly + Cy = p_y
#   直接最小二乘即可解出, 再迭代剔除残差大的帧以抵抗里程计跳变.
#
# 用法:
#   source /opt/ros/humble/setup.bash
#   python3 Livox_Calibration.py            # 默认订阅 /Odometry
#   python3 Livox_Calibration.py --plot     # 结算后画图查看拟合圆
#   python3 Livox_Calibration.py --ros-args -p odom_topic:=/xxx -p qos:=auto
#   qos 可选: auto(默认, 按发布端自动匹配) / sensor(强制 BEST_EFFORT) /
#             reliable(强制 RELIABLE)
#   启动后手动转动云台 >=1 圈 (建议 2~3 圈, 车体保持不动), 按 P 结算标定;
#   C 清空重新采集; Q 或 Ctrl-C 退出.
#
# 自动启动 FAST-LIO:
#   脚本启动后会检测 /Odometry 是否有发布者, 若发现 FAST-LIO(或雷达驱动)
#   没在跑, 会自动帮忙拉起 Livox 驱动 + FAST-LIO, 无需再手动开一个终端.
#   相关参数(均可用 --ros-args -p 覆盖):
#     auto_start_lio     true    是否自动启动(关掉: -p auto_start_lio:=false)
#     auto_start_driver  true    是否连雷达驱动一起启动(检测到 /livox/lidar
#                                已有发布者时自动跳过, 不会重复启动驱动)
#     lio_config         avia.yaml          FAST-LIO 配置文件
#     lio_extra_args     ''                 fast_lio launch 的额外参数,
#                                          如无桌面环境可传 'rviz:=false'
#     lio_setup_files    ''                启动前要 source 的 setup.bash,
#                                          分号分隔; 留空则沿用当前终端环境,
#                                          找不到 fast_lio 包时再自动尝试
#                                          常见工作区(见 DEFAULT_LIO_SETUP_FILES)
#     lio_start_delay    3.0               观察期(秒), 先等 DDS 发现完再判断
#     lio_log_file       /tmp/livox_calibration_lio.log  子进程日志
#   脚本被 Q / Ctrl-C 退出时, 会一并关闭它自己拉起的这些节点.
#
# 输出:
#   lidar_to_base_x / lidar_to_base_y (单位 m), 可直接填到
#   realtime_pgm.launch.py 的 lidar_to_base_x / lidar_to_base_y 参数

import math
import os
import signal
import subprocess
import sys
import threading
import time
from collections import deque

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy)
from nav_msgs.msg import Odometry

# 主线程(结算输出/绘图)与后台状态线程共用 stdout, 用锁避免输出互相覆盖
IO_LOCK = threading.Lock()
# 结算过程中(含绘图)暂停状态行刷新, 避免刷屏盖掉结果
PAUSE = threading.Event()
# 退出信号, 用于结束后台状态线程
STOP = threading.Event()

# 自动启动 FAST-LIO 时, 如果当前终端环境里找不到 fast_lio 包, 会依次尝试
# source 下列 setup.bash (哪个能找到包就用哪个), 免得非要先手动 source 一遍.
# 可用参数 lio_setup_files 覆盖(分号分隔), 或把不存在的路径删掉.
DEFAULT_LIO_SETUP_FILES = [
    '/home/lele/source_code_from_git/ws_livox/install/setup.bash',
    '/home/lele/ros2_humble_1/install/setup.bash',
]


def read_key():
    """读取一次按键(无需回车), 返回小写字符; 读取失败返回空字符串. 跨平台实现."""
    # 1) POSIX 终端: termios 进入 cbreak 模式逐字符读取
    try:
        import termios
        import tty
        fd = sys.stdin.fileno()
        if os.isatty(fd):
            old = termios.tcgetattr(fd)
            try:
                tty.setcbreak(fd)
                ch = os.read(fd, 1)
            finally:
                # 无论正常返回还是 Ctrl-C 中断, 都要恢复终端属性
                termios.tcsetattr(fd, termios.TCSADRAIN, old)
            return ch.decode('utf-8', 'ignore').lower()
    except Exception:
        pass
    # 2) Windows: msvcrt 直接读一个按键
    try:
        import msvcrt
        return msvcrt.getch().decode('utf-8', 'ignore').lower()
    except Exception:
        pass
    # 3) 兜底: 按行输入 (需要回车确认)
    try:
        return sys.stdin.readline().strip().lower()[:1]
    except Exception:
        return ''


class LidarCalibrator(Node):
    """订阅里程计缓存雷达位姿, 按需做圆拟合求解 雷达->车体中心 外参."""

    def __init__(self):
        super().__init__('livox_calibration')
        # 里程计话题: FAST-LIO 输出的雷达/IMU 位姿
        self.declare_parameter('odom_topic', '/Odometry')
        # 最多缓存的帧数, 超出后自动丢弃最旧的帧 (按 10Hz~100Hz 足够转很多圈)
        self.declare_parameter('max_samples', 50000)

        # 话题名缓存为成员, 供诊断日志使用
        self.topic_name = str(self.get_parameter('odom_topic').value)
        self.max_samples = int(self.get_parameter('max_samples').value)

        # 环形缓冲: 保存 (x, y, yaw) 三元组, 避免长时间旋转时内存无限增长
        self.samples = deque(maxlen=self.max_samples)
        self.lock = threading.Lock()
        # 累计收到的里程计回调次数, 注意: 每调用一次 on_odom 就 +1,
        # 无论该帧最终是否被写入 samples, 便于判断"收不到消息"还是"收到但没入库"
        self.msg_count = 0

        # QoS 模式 (ros2 run/launch 传参: --ros-args -p qos:=auto|sensor|reliable):
        #   auto    : 先用最通用的 BEST_EFFORT 订阅 (与 RELIABLE/BEST_EFFORT
        #             发布端都兼容), 诊断阶段检测到发布端后按需重建为匹配 QoS
        #   sensor  : 强制 BEST_EFFORT (兼容一切发布端, 高负载下可能丢帧)
        #   reliable: 强制 RELIABLE (仅当发布端也是 RELIABLE 时才能收到数据)
        self.declare_parameter('qos', 'auto')
        self.qos_mode = str(self.get_parameter('qos').value).lower()
        self.odom_sub = None
        # 当前订阅使用的 reliability 字符串, 供诊断时与发布端比对
        self.sub_reliability = None

        # ---------- FAST-LIO 自动启动 ----------
        # 收不到 /Odometry 且该话题没有任何发布者时, 认为是"忘了先开建图节点",
        # 自动帮忙拉起 Livox 驱动 + FAST-LIO, 免得再手动开一个终端.
        self.declare_parameter('auto_start_lio', True)
        # FAST-LIO 必须有雷达数据才会输出 /Odometry, 因此默认连驱动一起拉起;
        # 若检测到雷达话题已有发布者(驱动已在别处运行)则自动跳过, 不重复启动
        self.declare_parameter('auto_start_driver', True)
        self.declare_parameter('driver_launch_pkg', 'livox_ros_driver2')
        self.declare_parameter('driver_launch_file', 'msg_MID360_launch.py')
        self.declare_parameter('driver_topic', '/livox/lidar')
        self.declare_parameter('lio_launch_pkg', 'fast_lio')
        self.declare_parameter('lio_launch_file', 'mapping.launch.py')
        self.declare_parameter('lio_config', 'avia.yaml')
        # 追加到 fast_lio launch 命令末尾的额外参数, 例如 'rviz:=false'
        self.declare_parameter('lio_extra_args', '')
        # source 的 setup.bash, 分号分隔; 留空 = 沿用当前终端环境,
        # 若当前环境找不到 fast_lio 包再自动尝试 DEFAULT_LIO_SETUP_FILES
        self.declare_parameter('lio_setup_files', '')
        # 观察期(秒): 启动瞬间 DDS 可能还没发现完对端, 先等一会儿再下结论
        self.declare_parameter('lio_start_delay', 3.0)
        # 自动拉起的子进程日志(每次自动启动会清空重写)
        self.declare_parameter('lio_log_file', '/tmp/livox_calibration_lio.log')

        self.auto_start_lio = bool(self.get_parameter('auto_start_lio').value)
        self.auto_start_driver = bool(
            self.get_parameter('auto_start_driver').value)
        self.driver_pkg = str(self.get_parameter('driver_launch_pkg').value)
        self.driver_file = str(self.get_parameter('driver_launch_file').value)
        self.driver_topic = str(self.get_parameter('driver_topic').value)
        self.lio_pkg = str(self.get_parameter('lio_launch_pkg').value)
        self.lio_file = str(self.get_parameter('lio_launch_file').value)
        self.lio_config = str(self.get_parameter('lio_config').value)
        self.lio_extra_args = str(
            self.get_parameter('lio_extra_args').value).strip()
        self.lio_setup_files = str(
            self.get_parameter('lio_setup_files').value)
        self.lio_start_delay = float(
            self.get_parameter('lio_start_delay').value)
        self.lio_log_file = str(self.get_parameter('lio_log_file').value)

        # 自动启动状态机: idle(未触发) / starting(已拉起, 等数据) /
        #                  running(数据已到) / failed(启动失败) / off(已关闭)
        self.launch_state = 'idle'
        self.node_start_time = time.time()
        self.procs = []      # 本脚本拉起的 [(名字, Popen), ...]
        if not self.auto_start_lio:
            self.launch_state = 'off'

        self._make_subscription(self._qos_for_mode(None))
        # 1Hz 心跳诊断: 收不到数据时自动定位原因, 必要时自动切换 QoS,
        # 避免出现"帧数一直 0 却不知道卡在哪一步"的情况
        self.create_timer(1.0, self.on_diag)
        self.get_logger().info(
            '雷达标定节点已启动, 订阅里程计: %s (qos=%s, 当前=%s, 自动启动FAST-LIO=%s)'
            % (self.topic_name, self.qos_mode, self.sub_reliability,
               '开' if self.auto_start_lio else '关'))

    # ---------- QoS 与订阅管理 ----------
    @staticmethod
    def _reliable_qos():
        """RELIABLE 订阅配置: 深度 50, 转云台/结算期间也不会漏帧."""
        return QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            durability=DurabilityPolicy.VOLATILE)

    @staticmethod
    def _rel_name(qos_cfg):
        """把 QoS 的 reliability 转成可读字符串, 便于日志与比对."""
        val = getattr(qos_cfg, 'reliability', None)
        return getattr(val, 'name', None) or str(val)

    def _qos_for_mode(self, pub_reliability):
        """按 qos 参数与发布端 reliability 决定订阅 QoS.

        BEST_EFFORT 订阅端与 RELIABLE/BEST_EFFORT 发布端都兼容, 因此
        auto 模式查不到发布端信息时先退到 BEST_EFFORT, 保证一定收得到.
        """
        if self.qos_mode.startswith('sens'):
            return qos_profile_sensor_data
        if self.qos_mode.startswith('rel'):
            return self._reliable_qos()
        # auto: 发布端为 RELIABLE 时升级为 RELIABLE(零丢帧), 否则 BEST_EFFORT
        if pub_reliability == ReliabilityPolicy.RELIABLE:
            return self._reliable_qos()
        return qos_profile_sensor_data

    def _make_subscription(self, qos_cfg, reason=''):
        """(重新)创建里程计订阅, 记录当前使用的 reliability 供诊断比对."""
        if self.odom_sub is not None:
            try:
                self.destroy_subscription(self.odom_sub)
            except Exception:
                pass
        self.odom_sub = self.create_subscription(
            Odometry, self.topic_name, self.on_odom, qos_cfg)
        self.sub_reliability = self._rel_name(qos_cfg)
        self.get_logger().info(
            '订阅 %s (QoS=%s)%s'
            % (self.topic_name, self.sub_reliability,
               (' ' + reason) if reason else ''))

    def _detect_publishers(self, topic=None):
        """查询话题发布端, 返回 (数量, 发布端 reliability 或 None).

        topic 缺省为里程计话题; 兼容不同 rclpy 版本: Humble 用
        get_publishers_info_by_topic, 老版本退化为只判断话题是否存在.
        """
        topic = topic or self.topic_name
        try:
            infos = self.get_publishers_info_by_topic(topic)
        except AttributeError:
            try:
                names = dict(self.get_topic_names_and_types())
                return (1, None) if topic in names else (0, None)
            except Exception:
                return 0, None
        except Exception:
            return 0, None
        if not infos:
            return 0, None
        rel = None
        try:
            rel = infos[0].qos_profile.reliability
        except Exception:
            rel = None
        return len(infos), rel

    # ---------- 心跳诊断 ----------
    def on_diag(self):
        """每秒检查一次: 收不到数据时自动定位原因, 必要时自动启动 FAST-LIO."""
        try:
            if self.msg_count > 0:
                self._mark_launch_ok()
                return
            n_pub, pub_rel = self._detect_publishers()
            if n_pub == 0:
                # 没有任何发布者 => FAST-LIO 没在跑, 按需自动拉起
                self._maybe_auto_start()
                if self.launch_state in ('starting', 'failed'):
                    return      # 已处理过, 不再重复刷告警
                self.get_logger().warning(
                    '尚未收到 %s 数据, 且未发现该话题的发布者: '
                    '请确认 FAST-LIO 已启动 (ros2 topic hz %s)'
                    % (self.topic_name, self.topic_name))
                return
            # 有发布者但收不到数据: 检查 QoS 是否匹配, 不匹配则自动重建订阅
            want = self._qos_for_mode(pub_rel)
            if self.sub_reliability != self._rel_name(want):
                self._make_subscription(
                    want, '(自动匹配发布端 QoS, 发布端=%s)' % pub_rel)
                return
            if self.launch_state == 'starting':
                return      # 刚被自动拉起, 等它真正开始输出即可
            self.get_logger().warning(
                '发现 %d 个发布者且 QoS 已匹配但仍无数据: '
                '可能 FAST-LIO 尚未真正开始输出, 或消息类型不一致' % n_pub)
        except Exception as exc:
            # 诊断只是辅助功能, 绝不能让异常打断 spin 线程(否则连数据都收不到)
            self.get_logger().warning('诊断出错(忽略): %r' % (exc,))

    # ---------- FAST-LIO 自动启动 ----------
    def _mark_launch_ok(self):
        """收到数据后更新自动启动状态, 只提示一次."""
        if self.launch_state == 'starting':
            self.launch_state = 'running'
            with IO_LOCK:
                print('\n[自动启动] 已收到 %s 数据, 链路就绪, 可以转动云台标定了'
                      % self.topic_name)

    @staticmethod
    def _run_shell(cmd, timeout=20.0):
        """在子 shell 里执行命令, 返回 (returncode, 合并后的输出文本)."""
        try:
            proc = subprocess.run(
                ['bash', '-c', cmd], stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, timeout=timeout)
            return proc.returncode, proc.stdout.decode('utf-8', 'ignore')
        except Exception as exc:
            return -1, repr(exc)

    def _setup_prefix(self):
        """返回启动子进程前要附加的 'source xxx && ' 前缀, 找不到包时返回 None.

        优先沿用当前终端环境; 若当前环境里没有 fast_lio 包(忘了 source
        工作区), 再依次尝试 lio_setup_files 与 DEFAULT_LIO_SETUP_FILES,
        哪个能真正找出包就用哪个, 避免 source 错工作区导致环境被覆盖.
        """
        if self._run_shell('ros2 pkg prefix %s' % self.lio_pkg)[0] == 0:
            return ''
        candidates = [f.strip() for f in self.lio_setup_files.split(';')
                      if f.strip()]
        candidates += [f for f in DEFAULT_LIO_SETUP_FILES
                       if f not in candidates]
        for path in candidates:
            if not os.path.exists(path):
                continue
            rc, _ = self._run_shell(
                'source %s && ros2 pkg prefix %s' % (path, self.lio_pkg),
                timeout=30.0)
            if rc == 0:
                self.get_logger().info('自动 source: %s' % path)
                return 'source %s && ' % path
        return None

    def _spawn(self, name, cmd, prefix):
        """后台启动一条 launch 命令, 并独立成进程组(退出时可整组结束)."""
        proc = subprocess.Popen(
            ['bash', '-c', (prefix or '') + cmd],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            start_new_session=True)
        self.procs.append((name, proc))
        threading.Thread(target=self._drain_output, args=(name, proc),
                         daemon=True).start()
        return proc

    def _drain_output(self, name, proc):
        """读取子进程输出: 全部写日志文件, 限量转发到终端避免刷屏."""
        last = 0.0
        fp = None
        try:
            fp = open(self.lio_log_file, 'ab', buffering=0)
        except Exception:
            fp = None
        try:
            for raw in iter(proc.stdout.readline, b''):
                if fp is not None:
                    try:
                        fp.write(raw)
                    except Exception:
                        pass
                now = time.time()
                if now - last < 0.2:        # 限速: 每路最多 5 行/秒
                    continue
                last = now
                line = raw.decode('utf-8', 'ignore').rstrip()
                if not line:
                    continue
                with IO_LOCK:
                    sys.stdout.write('\n[%s] %s\n' % (name, line))
                    sys.stdout.flush()
        except Exception:
            pass
        finally:
            if fp is not None:
                try:
                    fp.close()
                except Exception:
                    pass
            rc = proc.poll()
            if rc not in (None, 0) and not STOP.is_set():
                self.get_logger().warning(
                    '%s 已退出 (code=%s), 完整日志: %s'
                    % (name, rc, self.lio_log_file))

    def _maybe_auto_start(self):
        """收不到数据且无发布者时, 按配置触发一次自动启动(只触发一次)."""
        if not self.auto_start_lio or self.launch_state != 'idle':
            return
        if time.time() - self.node_start_time < self.lio_start_delay:
            return      # 观察期: 等 DDS 把已有节点发现完, 避免误判
        self.launch_state = 'starting'
        threading.Thread(target=self._auto_start_lio, daemon=True).start()

    def _auto_start_lio(self):
        """后台线程: 检查依赖 -> (按需)启动 Livox 驱动 -> 启动 FAST-LIO."""
        prefix = self._setup_prefix()
        if prefix is None:
            self.launch_state = 'failed'
            with IO_LOCK:
                print('\n[自动启动] 当前环境找不到 %s 包, 无法自动启动.'
                      % self.lio_pkg)
                print('  请先 source 含 FAST-LIO 的工作区, 例如:')
                print('    source /home/lele/source_code_from_git/'
                      'ws_livox/install/setup.bash')
                print('  或用参数指定: --ros-args -p '
                      "lio_setup_files:='/path/to/install/setup.bash'")
                print('  亦可关闭自动启动: --ros-args -p '
                      'auto_start_lio:=false')
            return
        try:
            # 每次自动启动清空日志, 方便查看本次的输出
            try:
                open(self.lio_log_file, 'wb').close()
            except Exception:
                pass
            with IO_LOCK:
                print('\n[自动启动] 未检测到 %s 的发布者, 正在拉起 FAST-LIO ...'
                      % self.topic_name)
            # 1) 雷达驱动: 只有确认 /livox/lidar 没人发时才启动, 避免重复驱动
            if self.auto_start_driver:
                n_lidar, _ = self._detect_publishers(self.driver_topic)
                if n_lidar == 0:
                    with IO_LOCK:
                        print('  启动 Livox 驱动: ros2 launch %s %s'
                              % (self.driver_pkg, self.driver_file))
                    self._spawn(
                        'Livox驱动',
                        'ros2 launch %s %s'
                        % (self.driver_pkg, self.driver_file), prefix)
                else:
                    with IO_LOCK:
                        print('  检测到 %s 已有发布者, 跳过驱动启动'
                              % self.driver_topic)
            # 2) FAST-LIO 建图(输出 /Odometry)
            lio_cmd = ('ros2 launch %s %s config_file:=%s'
                       % (self.lio_pkg, self.lio_file, self.lio_config))
            if self.lio_extra_args:
                lio_cmd += ' ' + self.lio_extra_args
            with IO_LOCK:
                print('  启动 FAST-LIO : %s' % lio_cmd)
            self._spawn('FAST-LIO', lio_cmd, prefix)
            with IO_LOCK:
                print('  已拉起, 等待 %s 数据... 完整日志: %s'
                      % (self.topic_name, self.lio_log_file))
        except Exception as exc:
            self.launch_state = 'failed'
            self.get_logger().error('自动启动 FAST-LIO 失败: %r' % (exc,))

    @staticmethod
    def _signal_proc(proc, sig):
        """给子进程及其整个进程组发信号(启动时已 start_new_session 独立成组)."""
        try:
            os.killpg(os.getpgid(proc.pid), sig)
        except Exception:
            try:
                proc.send_signal(sig)
            except Exception:
                pass

    def shutdown_launched(self):
        """退出时关闭本脚本拉起的子进程: 先 SIGINT 优雅退出, 超时再 SIGKILL."""
        if not self.procs:
            return
        with IO_LOCK:
            print('\n[自动启动] 正在关闭自动拉起的节点: %s'
                  % ', '.join(name for name, _ in self.procs))
        for _, proc in self.procs:
            if proc.poll() is None:
                self._signal_proc(proc, signal.SIGINT)
        deadline = time.time() + 8.0
        for _, proc in self.procs:
            if proc.poll() is not None:
                continue
            try:
                proc.wait(timeout=max(0.0, deadline - time.time()))
            except Exception:
                pass
        for _, proc in self.procs:
            if proc.poll() is None:
                self._signal_proc(proc, signal.SIGKILL)

    # ---------- 里程计回调 ----------
    def on_odom(self, msg):
        # 先计数再解析, 保证"收到消息"这一事实能被诊断逻辑看到
        self.msg_count += 1
        try:
            pos = msg.pose.pose.position
            q = msg.pose.pose.orientation
            # 由四元数取偏航角 (与 pgm_viewer 中的算法一致)
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            with self.lock:
                self.samples.append(
                    (float(pos.x), float(pos.y), float(yaw)))
        except Exception as exc:
            # 回调异常会让 spin 线程退出并中断整个数据流, 必须就地兜住
            self.get_logger().warning('里程计消息处理失败(忽略): %r' % (exc,))

    # ---------- 采样与清空 ----------
    def snapshot(self):
        """取当前所有采样, 返回 (N, 3) 的 numpy 数组 [(x, y, yaw), ...]."""
        with self.lock:
            if not self.samples:
                return np.zeros((0, 3), dtype=float)
            return np.asarray(self.samples, dtype=float)

    def clear(self):
        """清空已采集数据 (重新开始一轮旋转)."""
        with self.lock:
            self.samples.clear()

    # ---------- 最小二乘方程构造 ----------
    @staticmethod
    def _lsq(x, y, yaw):
        """构造 p_i + R(psi_i)L = C 的线性方程组 A * [Lx, Ly, Cx, Cy]^T = b."""
        c = np.cos(yaw)
        s = np.sin(yaw)
        n = x.size
        A = np.zeros((2 * n, 4), dtype=float)
        b = np.zeros(2 * n, dtype=float)
        # x 分量: -cos(psi)*Lx + sin(psi)*Ly + Cx = p_x
        A[0::2, 0] = -c
        A[0::2, 1] = s
        A[0::2, 2] = 1.0
        b[0::2] = x
        # y 分量: -sin(psi)*Lx - cos(psi)*Ly + Cy = p_y
        A[1::2, 0] = -s
        A[1::2, 1] = -c
        A[1::2, 3] = 1.0
        b[1::2] = y
        return A, b

    # ---------- 求解 ----------
    def solve(self):
        """做最小二乘 + 迭代残差剔除, 返回结果字典; 样本不足时返回 None."""
        arr = self.snapshot()
        n = arr.shape[0]
        if n < 50:
            return None

        x, y, yaw = arr[:, 0], arr[:, 1], arr[:, 2]

        # 迭代最小二乘: 每轮用 3*RMS 阈值剔除离群帧 (抵抗里程计跳变/误匹配)
        mask = np.ones(n, dtype=bool)
        sol = np.zeros(4, dtype=float)
        rms = float('inf')
        for _ in range(8):
            A, b = self._lsq(x[mask], y[mask], yaw[mask])
            sol = np.linalg.lstsq(A, b, rcond=None)[0]
            lx, ly, cx, cy = sol[0], sol[1], sol[2], sol[3]
            # 用本轮结果计算"全部帧"的残差, 便于重选内点
            c = np.cos(yaw)
            s = np.sin(yaw)
            rx = x - (-c * lx + s * ly + cx)
            ry = y - (-s * lx - c * ly + cy)
            r = np.hypot(rx, ry)
            rms = float(np.sqrt(np.mean(r[mask] ** 2))) if mask.any() else float('inf')
            thr = max(3.0 * rms, 0.002)   # 至少 2mm, 避免过拟合到噪声
            new_mask = r <= thr
            # 内点太少或不再变化则停止迭代
            if new_mask.sum() < 20 or new_mask.sum() == mask.sum():
                break
            mask = new_mask

        lx, ly, cx, cy = (float(v) for v in (sol[0], sol[1], sol[2], sol[3]))

        # 云台累计旋转角度 (对 yaw 解缠绕后取跨度), 用于判断是否转够了圈数
        yaw_unwrapped = np.unwrap(yaw)
        span_deg = float(np.degrees(
            yaw_unwrapped.max() - yaw_unwrapped.min()))

        # |L| = 雷达到旋转轴(车体中心)的距离, 应等于实测轨迹的拟合圆半径
        radius_fit = math.hypot(lx, ly)
        radius_traj = float(np.mean(
            np.hypot(x[mask] - cx, y[mask] - cy)))

        return {
            'n_total': int(n),
            'n_inlier': int(mask.sum()),
            'span_deg': span_deg,
            'lidar_to_base_x': lx,
            'lidar_to_base_y': ly,
            'center_x': cx,
            'center_y': cy,
            'radius_fit': radius_fit,
            'radius_traj': radius_traj,
            'rms': rms,
            'x': x, 'y': y, 'mask': mask, 'cx': cx, 'cy': cy,
        }


def show_plot(res):
    """结算后画图: 雷达轨迹 + 拟合圆 + 车体中心, 便于肉眼确认 (需 matplotlib)."""
    try:
        import matplotlib
        import matplotlib.pyplot as plt
    except Exception as exc:
        print('[提示] 未安装 matplotlib, 跳过绘图: %s' % exc)
        return
    try:
        th = np.linspace(0.0, 2.0 * math.pi, 200)
        fig, ax = plt.subplots(figsize=(6, 6))
        # 雷达轨迹: 内点绿色, 被剔除的离群点红色
        ax.plot(res['x'][res['mask']], res['y'][res['mask']],
                '.', ms=3, color='#2a7', label='雷达轨迹(内点)')
        if (~res['mask']).any():
            ax.plot(res['x'][~res['mask']], res['y'][~res['mask']],
                    '.', ms=3, color='#c33', label='剔除点')
        # 拟合圆(半径取实测轨迹半径) + 拟合出的车体中心
        ax.plot(res['cx'] + res['radius_traj'] * np.cos(th),
                res['cy'] + res['radius_traj'] * np.sin(th),
                '-', color='#888', lw=1, label='拟合圆')
        ax.plot([res['cx']], [res['cy']], 'r*', ms=16, label='拟合车体中心')
        ax.set_aspect('equal', 'box')
        ax.grid(True, alpha=0.3)
        ax.set_xlabel('x / m')
        ax.set_ylabel('y / m')
        ax.set_title('lidar_to_base = (%.3f, %.3f) m' %
                     (res['lidar_to_base_x'], res['lidar_to_base_y']))
        ax.legend(loc='best', fontsize=8)
        plt.tight_layout()
        plt.show()
    except Exception as exc:
        print('[提示] 绘图失败(可能无显示环境): %s' % exc)


def print_report(res):
    """格式化打印标定结果与质量评估."""
    lines = []
    lines.append('')
    lines.append('=' * 64)
    lines.append('云台雷达标定结果')
    lines.append('  有效帧数    : %d / %d' % (res['n_inlier'], res['n_total']))
    lines.append('  云台旋转跨度: %.1f deg' % res['span_deg'])
    lines.append('  拟合圆半径  : %.4f m  (|L| = 雷达到旋转轴距离)' %
                 res['radius_fit'])
    lines.append('  实测轨迹半径: %.4f m  (两者应接近)' % res['radius_traj'])
    lines.append('  残差 RMS    : %.4f m' % res['rms'])
    lines.append('  拟合车体中心: (%.4f, %.4f) m' %
                 (res['center_x'], res['center_y']))
    lines.append('-' * 64)
    lines.append('  lidar_to_base_x = %.4f' % res['lidar_to_base_x'])
    lines.append('  lidar_to_base_y = %.4f' % res['lidar_to_base_y'])
    lines.append('-' * 64)
    lines.append('  用法:')
    lines.append('    ros2 launch test_draw_pgm_realtime realtime_pgm.launch.py \\')
    lines.append('        lidar_to_base_x:=%.4f lidar_to_base_y:=%.4f'
                 % (res['lidar_to_base_x'], res['lidar_to_base_y']))
    lines.append('    或 ros2 param set /test_draw_pgm_realtime '
                 'lidar_to_base_x %.4f' % res['lidar_to_base_x'])
    lines.append('       ros2 param set /test_draw_pgm_realtime '
                 'lidar_to_base_y %.4f' % res['lidar_to_base_y'])

    # 质量提示: 跨度不够 / 残差偏大 / 车体可能移动了
    hints = []
    if res['span_deg'] < 180.0:
        hints.append('  * 旋转跨度不足 180 deg, 请再多转几圈后重新按 P 结算')
    if res['rms'] > 0.03:
        hints.append('  * 残差偏大, 标定过程中车体可能发生移动或里程计有跳变')
    if res['radius_fit'] > 0.01 and \
            abs(res['radius_fit'] - res['radius_traj']) > 0.05:
        hints.append('  * 拟合半径与实测轨迹半径差异较大, 建议重新标定')
    if hints:
        lines.append('-' * 64)
        lines.extend(hints)
    lines.append('=' * 64)
    print('\n'.join(lines))


def main():
    want_plot = '--plot' in sys.argv

    rclpy.init()
    node = LidarCalibrator()

    # 后台线程 1: 跑 ROS 回调 (必须在独立线程, 主线程要阻塞等按键)
    # 外面包一层守护循环: 某个回调抛异常导致 spin 退出时自动重启,
    # 避免整个节点悄悄失去数据流; 保存句柄以便退出时 join
    def spin_supervisor():
        while not STOP.is_set() and rclpy.ok():
            try:
                rclpy.spin(node)
            except Exception as exc:
                if STOP.is_set():
                    break   # 退出阶段 rclpy 被关闭属于正常现象
                node.get_logger().error('ROS 回调线程异常, 已自动重启: %r' % (exc,))
            time.sleep(0.2)

    spin_thread = threading.Thread(target=spin_supervisor, daemon=True)
    spin_thread.start()

    # 后台线程 2: 每 2s 刷新一行采集状态 (用 \r 原地覆盖)
    def status_loop():
        while not STOP.is_set():
            time.sleep(2.0)
            if STOP.is_set() or PAUSE.is_set():
                continue
            arr = node.snapshot()
            if arr.shape[0] >= 2:
                span = float(np.degrees(
                    np.unwrap(arr[:, 2]).max() - np.unwrap(arr[:, 2]).min()))
            else:
                span = 0.0
            with IO_LOCK:
                # 同时显示"收到消息数"和"入库帧数": 两者不一致时能立刻看出
                # 是消息没进来(收到=0)还是回调里出错(收到>0 但入库=0)
                sys.stdout.write(
                    '\r[采集] 收到=%6d  入库=%6d  云台旋转=%8.1f deg  |  '
                    'P 结算  C 清空  Q 退出   '
                    % (node.msg_count, arr.shape[0], span))
                sys.stdout.flush()

    threading.Thread(target=status_loop, daemon=True).start()

    with IO_LOCK:
        print('=' * 64)
        print('云台雷达标定:')
        print('  1) 保持车体不动, 手动转动云台 >=1 圈 (建议 2~3 圈)')
        print('  2) 按 P 结算并输出 lidar_to_base_x / lidar_to_base_y')
        print('  3) C 清空重新采集, Q 或 Ctrl-C 退出')
        print('-' * 64)
        if node.auto_start_lio:
            print('  未检测到 %s 发布者时会自动启动 FAST-LIO (约 %ds 后判断),'
                  % (node.topic_name, int(node.lio_start_delay)))
            print('  等状态行"收到"开始增长后再转动云台即可. 关闭: '
                  '-p auto_start_lio:=false')
        else:
            print('  自动启动 FAST-LIO 已关闭, 请自行确保 FAST-LIO 正在运行')
        print('=' * 64)

    try:
        while True:
            key = read_key()
            if key == 'p':
                # 结算期间暂停状态行刷新, 避免盖掉结果
                PAUSE.set()
                try:
                    res = node.solve()
                    with IO_LOCK:
                        if res is None:
                            # 区分"根本没收到里程计"和"收到了但帧数不够",
                            # 前者是话题/QoS 问题, 后者只要继续转动云台即可
                            if node.msg_count == 0:
                                print('\n[提示] 完全没有收到 %s 消息, 请检查:'
                                      % node.topic_name)
                                print('  1) FAST-LIO 是否已启动 '
                                      '(ros2 topic list | grep -i odom)')
                                print('  2) 话题名是否正确 '
                                      '(当前: %s)' % node.topic_name)
                                print('  3) QoS 是否匹配, 试: '
                                      'python3 Livox_Calibration.py '
                                      '--ros-args -p qos:=sensor')
                            else:
                                print('\n[提示] 已收到 %d 条里程计消息但仅入库 %d '
                                      '帧, 至少需要 50 帧, 请继续转动云台后按 P'
                                      % (node.msg_count, node.snapshot().shape[0]))
                        else:
                            print_report(res)
                    if res is not None and want_plot:
                        show_plot(res)
                finally:
                    PAUSE.clear()
            elif key == 'c':
                node.clear()
                with IO_LOCK:
                    print('\n[提示] 已清空采集数据, 可重新转动云台标定')
            elif key in ('q', '\x03'):
                break
            elif key == '':
                # 读取失败(如非交互环境), 避免死循环空转
                time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        # 先发停止信号, 并主动关闭 rclpy, 使 spin 线程退出;
        # 再等待线程结束, 避免解释器析构时 C++ 侧抛
        # "terminate called without an active exception"
        STOP.set()
        if rclpy.ok():
            rclpy.shutdown()
        # 关闭本脚本自动拉起的 FAST-LIO / 雷达驱动, 避免留下后台进程
        node.shutdown_launched()
        # 给 spin 线程一点时间退出 (它是 daemon 线程, 不 join 也可)
        spin_thread.join(timeout=1.0)
        node.destroy_node()
        with IO_LOCK:
            print('\n已退出雷达标定.')


if __name__ == '__main__':
    main()
