#!/usr/bin/env python3
# pgm_viewer.py
#
# 实时 PGM 网页查看器（无 GUI 依赖, 适合 SSH/无桌面环境）:
#   - 订阅 C++ 节点发布的 /realtime_pgm/image (mono8)
#   - 订阅 C++ 节点发布的 /realtime_pgm/cloud (降采样点云),
#     网页在栅格图右侧渲染实时可交互点云侧视图:
#       滚轮缩放 / 拖动平移 / 双击或按钮复位 / 可切换水平轴 (x / y / 距离)
#       用绿色虚线 + 色带可视化 zmin/zmax 范围, 拖动虚线可直接改 zmin/zmax 滑条
#   - 订阅 /Odometry, 网页用透明 canvas 图层在地图之上叠加显示角色位姿
#     (位姿按 map->odom 的 SE(3) 修正量变换到 map 系后再显示, 与底图严格对齐;
#      修正量由 C++ 节点的 2D 栅格配准给出, 见 /pose_correction)
#   - 网页主地图使用持久化历史地图: 最近 history_duration 秒点云累积成的
#     最终地图 (话题 /realtime_pgm/image_history), 车辆位姿和路径均叠加在此地图上
#     累积时按"帧"投票: 同一帧内同一栅格只计一次命中, 仅保留命中次数达到
#     history_min_hits 的栅格, 用于滤除单点噪声/动态目标残影
#   - 路径规划 (Nav2): 在最终地图上单击目标点, 通过
#     nav2_msgs/action/NavigateToPose 递交给 Nav2(bt_navigator);
#     Nav2 负责全局规划(/plan) + 局部跟踪避障(默认行为树 1Hz 重规划);
#     配套 scripts/nav2_bridge.py 将实时占据栅格转成 /map 并发布
#     map->odom / odom->base_link TF, 代价地图由 Nav2 维护
#   - 网页叠加显示: 全局/局部代价地图 (/gcm.png, /lcm.png, RViz 配色:
#     红=致命 100, 紫=内切 99, 蓝绿->黄红=膨胀代价 1~98),
#     全局路径(绿) / 局部路径(黄) / 目标点(红) / 车身位姿(橙框)
#   - 点击"开始运动"后叠加显示路径跟随细节 (两个勾选框可分别关闭):
#       前视点 + 向量: 目标前视点(青色星形) 与 车体中心->前视点 的方向箭头,
#                      箭头旁标注 vx/vy (车体系速度) 与总速度
#       车体系: 以车体中心为原点绘制车体坐标系 —— x 轴 = 车辆所指前方向,
#               y 轴由 x 轴逆时针旋转 90 度得到 (车辆左手边), 与运动帧
#               (vx=前方, vy=左手边) 定义一致
#     数据来源: C++ 节点 /nav_motion_cmd 帧 + /plan + /Odometry, 由本节点
#     以与 C++ computeFollowCommand 完全相同的几何复算, 因此仅反映显示
#   - 右键地图或"取消导航"按钮可取消当前导航目标
#   - 浏览器访问 http://<本机IP>:1726 查看实时画面
#   - 网页内滑条实时修改 C++ 节点参数:
#       resolution(cm): 滑条值 v -> resolution = v/100    (1..100 -> 0.01..1.00 米/像素)
#       zmin(cm)/zmax(cm): 滑条值 v -> z = v/100 - 10.0  (0..2000 -> -10.0..+10.0 米)
#       dilate(px): 滑条值直接就是膨胀像素数              (0..20)
#       历史保留(s): 滑条值直接就是 history_duration      (1..600)
#       最低命中帧数: 滑条值直接就是 history_min_hits     (1..20)
#       A* 软性避让范围(m): 滑条值 v -> astar_cost_reach = v/100  (0..2 -> 0..2.00 米)
#       A* 靠障碍惩罚: 滑条值 v -> astar_obstacle_penalty = v/100 (0..2 -> 0..2.00)
#   - 每个参数旁都有数值输入框, 可直接键入精确值(m, px, s), 越界自动夹紧,
#     失焦后按设定精度归一化显示; 与滑条双向同步, 回车即生效
#   - 网页内滑条手动调整雷达->车体中心外参 (仅影响本查看器的位姿显示与规划起点):
#       lidar_to_base_x/y(m): 滑条值 v -> 外参 = v/1000   (-2000..2000 -> -2.0..+2.0 米)
#   - 除外参外, 修改通过参数服务写回, 与 ros2 param set 完全等效
#
# 运行:
#   ros2 run test_navigation pgm_viewer.py
#   然后浏览器打开 http://<本机IP>:1726

import io
import json
import math
import os
import shutil
import threading
import time
import zipfile
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (qos_profile_sensor_data, QoSProfile,
                       QoSReliabilityPolicy, QoSDurabilityPolicy)
from rclpy.action import ActionClient
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters, SetParameters
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import UInt8MultiArray, String

try:
    from nav2_msgs.action import NavigateToPose
except ImportError:  # 允许在未安装 nav2 的机器上仅使用网页预览功能
    NavigateToPose = None

HTTP_PORT = 1726

TBAR_RES_MAX = 100  # resolution = v / 100
TBAR_Z_MAX = 2000   # z = v / 100 - 10.0 (±10 米)
TBAR_DIL_MAX = 20   # dilate = v
TBAR_EXT_MAX = 2000  # lidar_to_base = v / 1000 (滑条单位 mm)
TBAR_CR_MAX = 200    # astar_cost_reach = v / 100 (0..2.00 米)
TBAR_OP_MAX = 200    # astar_obstacle_penalty = v / 100 (0..2.00, 0 = 纯最短路)


def quat_rotate(quat, vec):
    """用四元数 (x, y, z, w) 旋转三维向量 vec=(x, y, z), 返回三元组。

    即 q * v * q^-1 (单位四元数), 不引入 tf_transformations 依赖。
    """
    qx, qy, qz, qw = quat
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n < 1e-12:
        return (vec[0], vec[1], vec[2])
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n

    vx, vy, vz = vec
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    )


def quat_mul(a, b):
    """四元数乘法 a * b (均为 (x, y, z, w))。"""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_to_yaw(q):
    """从四元数 (x, y, z, w) 提取偏航角 (rad)。"""
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

PAGE = '''<html><meta charset="utf-8"><title>Realtime PGM + PointCloud</title>
<style>.numin{background:#1c1c1c;color:#8cf;border:1px solid #555;border-radius:4px;
padding:1px 4px;width:92px;font:12px monospace}</style>
<body style="font-family:sans-serif;background:#222;color:#ddd;margin:12px">
<h3 style="margin:4px 0">Realtime PGM <span style="font-size:13px;color:#888">+ 实时点云侧视图</span></h3>
<div id="status" style="color:#8c8">connecting...</div>
<div id="pose" style="color:#fa6">pose: waiting odometry ...</div>
<div id="plan" style="color:#8cf;font-size:13px">Nav2 导航: 单击地图发送目标点, 右键地图或按钮取消 (需 with_nav2:=true 启动 Nav2 与 nav2_bridge)</div>
<div style="display:flex;flex-wrap:wrap;gap:18px;align-items:flex-start;margin:10px 0">
  <div>
    <div id="wrap" style="position:relative;display:inline-block">
      <img id="img" src="/image_history.png" style="max-width:100%;image-rendering:pixelated;display:block;cursor:crosshair">
      <canvas id="ov" style="position:absolute;left:0;top:0;width:100%;height:100%;pointer-events:none"></canvas>
    </div>
    <div id="historydesc" style="font-size:12px;color:#888;margin-top:4px">最终地图（历史点云累积的俯视占据投影）</div>
    <div style="margin-top:6px">
      <button id="histrefresh" style="background:#333;color:#ddd;border:1px solid #555;border-radius:6px;padding:4px 10px;cursor:pointer">刷新累计图</button>
      <button id="histlock" style="background:#333;color:#dfd;border:1px solid #2f8f45;border-radius:6px;padding:4px 10px;cursor:pointer">锁定历史累计图</button>
      <button id="mapexport" style="background:#333;color:#cfc;border:1px solid #3f7f6f;border-radius:6px;padding:4px 10px;cursor:pointer">导出地图</button>
      <button id="mapimport" style="background:#333;color:#cfc;border:1px solid #3f7f6f;border-radius:6px;padding:4px 10px;cursor:pointer">导入地图</button>
      <input id="mapfile" type="file" accept=".zip" style="display:none">
      <button id="holdbtn" style="background:#333;color:#ffd27f;border:1px solid #8f7a2f;border-radius:6px;padding:4px 10px;cursor:pointer">选择占点位置</button>
      <button id="chasebtn" style="background:#333;color:#7fe0ff;border:1px solid #2f6f8f;border-radius:6px;padding:4px 10px;cursor:pointer">选择追击目标</button>
      <button id="navcancel" style="background:#333;color:#ddd;border:1px solid #555;border-radius:6px;padding:4px 10px;cursor:pointer">取消导航</button>
      <button id="startmove" style="background:#1d5c2a;color:#dfd;border:1px solid #2f8f45;border-radius:6px;padding:4px 10px;cursor:pointer">开始运动</button>
      <button id="stopmove" style="background:#5c1d1d;color:#fdd;border:1px solid #8f2f2f;border-radius:6px;padding:4px 10px;cursor:pointer">停止运动</button>
      <label style="font-size:12px;color:#aaa;margin-left:8px"><input type="checkbox" id="cb_gcm" checked> 全局代价地图</label>
      <label style="font-size:12px;color:#aaa;margin-left:6px"><input type="checkbox" id="cb_lcm" checked> 局部代价地图</label>
      <label style="font-size:12px;color:#5f5;margin-left:6px"><input type="checkbox" id="cb_dyn" checked> 动态障碍</label>
      <label style="font-size:12px;color:#7fd;margin-left:6px"><input type="checkbox" id="cb_lk" checked> 前视点及向量</label>
      <label style="font-size:12px;color:#fc8;margin-left:6px"><input type="checkbox" id="cb_cs" checked> 车体系坐标系</label>
      <label style="font-size:12px;color:#7f7;margin-left:6px"><input type="checkbox" id="cb_sf" checked> 车体自过滤框</label>
      <span style="font-size:12px;color:#777;margin-left:8px">红=致命 紫=内切 蓝绿→黄红=膨胀代价 绿块=动态障碍 | 绿线=全局路径 黄线=局部路径 红点=目标 绿框=车体自过滤</span>
    </div>
    <div id="dynstat" style="font-size:12px;color:#8c8;margin-top:4px">动态障碍: 未锁定（点击“锁定历史累计图”后，新出现的障碍将以绿色标记，且不计入历史地图）</div>
    <div id="holdstat" style="font-size:12px;color:#ffd27f;margin-top:4px">占点: 未启用（点击“选择占点位置”，再点历史累计图上的目标位置即可开始占点）</div>
    <div id="chasestat" style="font-size:12px;color:#7fe0ff;margin-top:4px">追击: 未启用（先“锁定历史累计图”, 再点“选择追击目标”并点选地图上的目标物体）</div>
    <div id="motionstat" style="font-size:12px;color:#888;margin-top:4px">运动: 未开始（路径规划完成后点“开始运动”）</div>
    <div id="remainstat" style="font-size:12px;color:#8ff;margin-top:2px">剩余距离: --（未在跟随）</div>
    <div id="histstat" style="font-size:12px;color:#c96;margin-top:12px">最终地图: 等待 /realtime_pgm/image_history 数据 ...</div>
    <div id="wraph" style="position:relative;display:inline-block;margin-top:4px;display:none">
      <img id="imgh" style="max-width:100%;image-rendering:pixelated;display:block">
      <div style="position:absolute;left:4px;top:4px;font-size:11px;color:#9f9;background:rgba(0,0,0,0.45);padding:1px 6px;border-radius:4px">实时单帧地图（仅预览，不参与规划）</div>
    </div>
  </div>
  <div style="display:flex;flex-direction:column;gap:6px">
    <div style="font-size:13px;color:#aaa">点云侧视图: 滚轮缩放 / 拖动平移 / 拖动绿虚线改 zmin·zmax / 双击复位 <b id="pcinfo" style="color:#8c8"></b></div>
    <div id="pcbox" style="position:relative;width:600px;max-width:96vw;border:1px solid #444;border-radius:8px;background:#111;overflow:hidden">
      <canvas id="pc" style="display:block;width:100%;height:420px;cursor:grab"></canvas>
    </div>
    <div>
      <button id="pcreset" style="background:#333;color:#ddd;border:1px solid #555;border-radius:6px;padding:4px 10px;cursor:pointer">复位视图</button>
      <button id="pcaxis" style="background:#333;color:#ddd;border:1px solid #555;border-radius:6px;padding:4px 10px;cursor:pointer">水平轴: x</button>
      <span style="font-size:12px;color:#777;margin-left:8px">灰点 = zmin~zmax 范围外, 绿点 = 范围内</span>
    </div>
  </div>
</div>
<div style="background:#2a2a2a;padding:12px;border-radius:8px;max-width:640px">
<div>resolution: <input id="n_res" class="numin" type="number" min="0.01" max="1" step="0.01"> m/px</div>
<input id="res" type="range" min="1" max="100" value="5" style="width:100%"><br>
<div>显示/建图 zmin: <input id="n_zmin" class="numin" type="number" min="-10" max="10" step="0.01"> m &nbsp;&nbsp;
     zmax: <input id="n_zmax" class="numin" type="number" min="-10" max="10" step="0.01"> m</div>
<input id="zmin" type="range" min="0" max="2000" value="775" style="width:49%">
<input id="zmax" type="range" min="0" max="2000" value="1022" style="width:49%"><br>
<div>dilate: <input id="n_dil" class="numin" type="number" min="0" max="20" step="1"> px</div>
<input id="dil" type="range" min="0" max="20" value="2" style="width:100%">
<div>A* 软性避让范围: <input id="n_cr" class="numin" type="number" min="0" max="2" step="0.01"> m
     <span style="font-size:12px;color:#777">(障碍邻近代价影响半径, 与 Nav2 inflation_radius 同口径)</span></div>
<input id="cr" type="range" min="0" max="200" value="60" style="width:100%">
<div>A* 靠障碍惩罚: <input id="n_op" class="numin" type="number" min="0" max="2" step="0.01">
     <span style="font-size:12px;color:#777">(越大越远离障碍; 0 = 纯最短路, 绕行最近)</span></div>
<input id="op" type="range" min="0" max="200" value="80" style="width:100%">
<div>历史保留: <input id="n_hist" class="numin" type="number" min="1" max="600" step="1"> s &nbsp;&nbsp;
     最低命中帧数: <input id="n_hits" class="numin" type="number" min="1" max="20" step="1"></div>
<input id="hist" type="range" min="1" max="600" value="300" style="width:49%">
<input id="hits" type="range" min="1" max="20" value="2" style="width:49%">
<div style="margin-top:8px;border-top:1px solid #444;padding-top:8px">
  <span style="color:#ffd27f">占点范围</span>:
  <input id="n_hrad" class="numin" type="number" min="0.1" max="20" step="0.1"> m
  <span style="font-size:12px;color:#777">(半径内的障碍才算威胁; 占点时地图上会画圈)</span>
</div>
<input id="hrad" type="range" min="1" max="200" value="15" style="width:100%">
<div style="margin-top:8px;border-top:1px solid #444;padding-top:8px">
  <span style="color:#ffd27f">阻挡站位间距系数</span>:
  <input id="n_hstd" class="numin" type="number" min="1" max="10" step="0.05"> ×
  <span style="font-size:12px;color:#777">(站位点距障碍 = 系数 × 硬性间距 0.50 m; 越接近 1 越贴近敌人)</span>
</div>
<input id="hstd" type="range" min="100" max="400" value="110" style="width:100%">
<div style="margin-top:8px;border-top:1px solid #444;padding-top:8px">
  <span style="color:#7fe0ff">追击环带半径</span>:
  <input id="n_crad" class="numin" type="number" min="0.1" max="20" step="0.1" value="1.0"> m
  <span style="font-size:12px;color:#777">(保持车体到目标约该距离; 太近退、太远进)</span>
</div>
<input id="crad" type="range" min="1" max="200" value="10" style="width:100%">
<div style="margin-top:8px;border-top:1px solid #444;padding-top:8px">
  雷达→车体中心外参: x <input id="n_lx" class="numin" type="number" min="-2" max="2" step="0.001"> m &nbsp;&nbsp;
  y <input id="n_ly" class="numin" type="number" min="-2" max="2" step="0.001"> m
  <span style="font-size:12px;color:#777">(影响位姿与规划起点; 上方数值框可直接输入精确值)</span>
</div>
<input id="lx" type="range" min="-2000" max="2000" value="239" style="width:49%">
<input id="ly" type="range" min="-2000" max="2000" value="-158" style="width:49%">
<div style="font-size:12px;color:#777;margin-top:2px">x &gt; 0: 车体中心在雷达前方 &nbsp;&nbsp; y &gt; 0: 车体中心在雷达左方</div>
</div>
<script>
function fmt(){return{
  res:document.getElementById('res').value,
  zmin:document.getElementById('zmin').value,
  zmax:document.getElementById('zmax').value,
  dil:document.getElementById('dil').value,
  hist:document.getElementById('hist').value,
  hits:document.getElementById('hits').value,
  cr:document.getElementById('cr').value,
  op:document.getElementById('op').value,
  lx:document.getElementById('lx').value,
  ly:document.getElementById('ly').value}}
/* 各参数的数值框配置: 真实值 = 滑条值/sc + off, 边界 lo/hi 为滑条范围 */
var NUMCFG={
  res :{sc:100,  off:0,  lo:1,     hi:100,  dec:2},
  zmin:{sc:100,  off:-10, lo:0,    hi:2000, dec:2},
  zmax:{sc:100,  off:-10, lo:0,    hi:2000, dec:2},
  dil :{sc:1,    off:0,  lo:0,     hi:20,   dec:0},
  hist:{sc:1,    off:0,  lo:1,     hi:600,  dec:0},
  hits:{sc:1,    off:0,  lo:1,     hi:20,   dec:0},
  cr  :{sc:100,  off:0,  lo:0,     hi:200,  dec:2},
  op  :{sc:100,  off:0,  lo:0,     hi:200,  dec:2},
  lx  :{sc:1000, off:0,  lo:-2000, hi:2000, dec:3},
  ly  :{sc:1000, off:0,  lo:-2000, hi:2000, dec:3}};
function numIn(id){return document.getElementById('n_'+id)}
/* 滑条 -> 数值框 */
function syncNum(id){
  var c=NUMCFG[id];
  numIn(id).value=(parseFloat(document.getElementById(id).value)/c.sc+c.off).toFixed(c.dec)}
function syncNums(skip){for(var id in NUMCFG)if(id!==skip)syncNum(id)}
/* 数值框 -> 滑条: 越界夹紧; 正在输入的空串/负号不打断 */
function numChanged(id){
  var c=NUMCFG[id],el=numIn(id);
  if(el.value===''||el.value==='-')return;
  var v=parseFloat(el.value);
  if(!isFinite(v))return;
  v=Math.min(c.hi/c.sc+c.off,Math.max(c.lo/c.sc+c.off,v));
  document.getElementById(id).value=Math.round((v-c.off)*c.sc);
  onChange(id)}
['res','zmin','zmax','dil','hist','hits','cr','op','lx','ly'].forEach(function(id){
  var el=numIn(id);
  el.addEventListener('input',function(){numChanged(id)});
  el.addEventListener('blur',function(){syncNum(id)})});
function refresh(skip){
  var f=fmt();
  syncNums(skip);
  document.getElementById('historydesc').textContent='最终地图（最近 '+f.hist+' s 点云累积的俯视占据投影）'}
var sendTimer=null;
function onChange(src){
  refresh(src);
  if(sendTimer)clearTimeout(sendTimer);
  sendTimer=setTimeout(function(){
    var f=fmt();
      fetch('/set?res='+f.res+'&zmin='+f.zmin+'&zmax='+f.zmax+
          '&dil='+f.dil+
          '&hist='+f.hist+'&hits='+f.hits+'&cr='+f.cr+'&op='+f.op+
          '&lx='+f.lx+'&ly='+f.ly)
      .then(r=>r.json()).then(j=>{
        document.getElementById('status').textContent =
          j.ok?('OK: res='+(j.res/100).toFixed(2)+' zmin='+(j.zmin/100-10).toFixed(2)+
                ' zmax='+(j.zmax/100-10).toFixed(2)+' dil='+j.dil+
                ' cr='+(j.cr/100).toFixed(2)+' op='+(j.op/100).toFixed(2)+
                ' lidar_to_base=('+(j.lx/1000).toFixed(3)+', '+
                (j.ly/1000).toFixed(3)+') m'):('rejected: '+j.msg);
        if(j.ok)applyState(j)})},300)}
['res','zmin','zmax','dil','hist','hits','cr','op','lx','ly'].forEach(function(id){
  document.getElementById(id).addEventListener('input',function(){onChange()})});
function applyState(s){
  document.getElementById('res').value=s.res;
  document.getElementById('zmin').value=s.zmin;
  document.getElementById('zmax').value=s.zmax;
  document.getElementById('dil').value=s.dil;
  document.getElementById('hist').value=s.hist;
  document.getElementById('hits').value=s.hits;
  if(s.cr!==undefined)document.getElementById('cr').value=s.cr;
  if(s.op!==undefined)document.getElementById('op').value=s.op;
  if(s.lx!==undefined)document.getElementById('lx').value=s.lx;
  if(s.ly!==undefined)document.getElementById('ly').value=s.ly}
fetch('/state').then(r=>r.json()).then(applyState).then(refresh);

/* ---------- robot pose + planned path overlay (on top of map) ---------- */
var img=document.getElementById('img');
var ov=document.getElementById('ov');
var ctx=ov.getContext('2d');
var lastPose=null;
var navData=null;    /* /nav 轮询结果: {status, goal, plan, local_plan, meta} */
var lkData=null;     /* /lookahead 轮询结果: 开始运动后的前视点/速度矢量 */
var gcmImg=null,lcmImg=null;  /* 代价地图 RGBA PNG (后端已对齐底图, 直接铺满) */
var dynImg=null;              /* 动态障碍绿色 RGBA PNG (锁定历史累计图后才有) */
var histLocked=false;         /* 是否已锁定历史累计图 (动态障碍模式) */
function setPlanMsg(text,color){
  var el=document.getElementById('plan');
  el.textContent=text;el.style.color=color}
function w2c(p,meta){
  return{c:(p.x-meta.xmin)/meta.res-0.5,
         r:meta.height-1-((p.y-meta.ymin)/meta.res-0.5)}}
function drawPath(pts,meta,color,widthPx){
  ctx.save();ctx.strokeStyle=color;
  ctx.lineWidth=widthPx;ctx.lineJoin='round';ctx.lineCap='round';
  ctx.beginPath();
  for(var i=0;i<pts.length;i++){
    var q=w2c(pts[i],meta);
    if(i===0)ctx.moveTo(q.c,q.r);else ctx.lineTo(q.c,q.r)}
  ctx.stroke();ctx.restore()}
function drawMark(p,meta,color,fill){
  var q=w2c(p,meta);
  ctx.beginPath();ctx.arc(q.c,q.r,Math.max(4,ov.width/90),0,Math.PI*2);
  if(fill){ctx.fillStyle=color;ctx.fill()}
  ctx.strokeStyle=color;ctx.lineWidth=2;ctx.stroke()}
/* 带箭头头的有向线段: 从 canvas 点 a 指向 b */
function drawVec(a,b,color,widthPx){
  var dx=b.c-a.c,dy=b.r-a.r,len=Math.hypot(dx,dy);
  if(len<1)return;
  var ux=dx/len,uy=dy/len,head=Math.min(len*0.45,Math.max(9,ov.width/110));
  ctx.save();ctx.strokeStyle=color;ctx.fillStyle=color;
  ctx.lineWidth=widthPx;ctx.lineCap='round';ctx.beginPath();
  ctx.moveTo(a.c,a.r);ctx.lineTo(b.c,b.r);ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(b.c,b.r);
  ctx.lineTo(b.c-ux*head-uy*head*0.45,b.r-uy*head+ux*head*0.45);
  ctx.lineTo(b.c-ux*head+uy*head*0.45,b.r-uy*head-ux*head*0.45);
  ctx.closePath();ctx.fill();ctx.restore()}
function drawText(text,c,color){
  ctx.save();ctx.font='12px monospace';ctx.lineWidth=3;
  ctx.strokeStyle='rgba(0,0,0,0.75)';ctx.strokeText(text,c.c+6,c.r-6);
  ctx.fillStyle=color;ctx.fillText(text,c.c+6,c.r-6);ctx.restore()}
/* 目标前视点 + 车体中心 -> 前视点 的方向箭头 (青色) */
function drawLookahead(lk,meta){
  var base=w2c(lk.base,meta),tgt=w2c(lk.target,meta);
  var arm=Math.max(5,ov.width/160);
  ctx.save();ctx.strokeStyle='#22e6e6';ctx.lineWidth=2;
  ctx.beginPath();
  ctx.moveTo(tgt.c-arm,tgt.r);ctx.lineTo(tgt.c+arm,tgt.r);
  ctx.moveTo(tgt.c,tgt.r-arm);ctx.lineTo(tgt.c,tgt.r+arm);
  ctx.stroke();ctx.restore();
  drawVec(base,tgt,'rgba(34,230,230,0.95)',Math.max(2,ov.width/220));
  drawMark(lk.target,meta,'#22e6e6',false);
  drawText('目标前视点  d='+lk.lookahead_m.toFixed(2)+' m',tgt,'#8ff');
  drawText('vx='+lk.vx.toFixed(2)+' m/s  vy='+lk.vy.toFixed(2)+
           ' m/s  |v|='+lk.speed.toFixed(2),base,'#8ff');
  /* 剩余距离: 沿路径弧长 remaining_m, 到终点的直线距离 dist_end_m
     (后者与 C++ 到达判定/减速段使用的 dist_end 同一量) */
  if(lk.remaining_m!==undefined||lk.dist_end_m!==undefined)
    drawText('剩余 '+(lk.remaining_m!==undefined?lk.remaining_m.toFixed(2):'--')+
             ' m  (直线 '+
             (lk.dist_end_m!==undefined?lk.dist_end_m.toFixed(2):'--')+' m)',
             {c:base.c,r:base.r+16},'#8ff')}
/* 车体中心为原点的车体系: x=车辆所指前方向, y=x 逆时针旋转 90 度(车辆左手边)
   —— 与运动帧 vx(前方)/vy(左手边) 的定义完全一致 */
function drawBodyFrame(lk,meta,color){
  var o=w2c(lk.base,meta);
  var lenM=Math.max(0.35,(lastPose&&lastPose.length?lastPose.length:0.60)*0.8);
  var lenPx=lenM/meta.res;
  var ca=Math.cos(lk.yaw),sa=Math.sin(lk.yaw);
  /* 世界单位向量 -> canvas 单位向量 (图像 y 轴向下, 故纵坐标取反) */
  var ux={c:ca,r:-sa};                     /* x 轴: 车辆前方 */
  var uy={c:-sa,r:-ca};                    /* y 轴: x 轴逆时针 90 度 */
  var px=Math.max(1.5,ov.width/260);
  drawVec(o,{c:o.c+ux.c*lenPx,r:o.r+ux.r*lenPx},color.x,px);
  drawVec(o,{c:o.c+uy.c*lenPx,r:o.r+uy.r*lenPx},color.y,px);
  ctx.save();
  ctx.fillStyle=color.x;
  ctx.beginPath();ctx.arc(o.c,o.r,Math.max(3,ov.width/200),0,Math.PI*2);
  ctx.fill();
  ctx.font='bold 13px monospace';
  ctx.fillStyle=color.x;
  ctx.fillText('x',o.c+ux.c*lenPx*0.72+6,o.r+ux.r*lenPx*0.72-4);
  ctx.fillStyle=color.y;
  ctx.fillText('y',o.c+uy.c*lenPx*0.72+6,o.r+uy.r*lenPx*0.72-4);
  ctx.restore()}
function drawOverlay(){
  if(!ov.width||!ov.height)return;
  ctx.clearRect(0,0,ov.width,ov.height);
  /* 代价地图 PNG 由后端重投影到底图网格, 尺寸与底图一致, 直接铺满 */
  var cbg=document.getElementById('cb_gcm');
  var cbl=document.getElementById('cb_lcm');
  if(cbg&&cbg.checked&&gcmImg&&
     gcmImg.naturalWidth===ov.width&&gcmImg.naturalHeight===ov.height)
    ctx.drawImage(gcmImg,0,0);
  if(cbl&&cbl.checked&&lcmImg&&
     lcmImg.naturalWidth===ov.width&&lcmImg.naturalHeight===ov.height)
    ctx.drawImage(lcmImg,0,0);
  /* 动态障碍 (绿色): 画在代价地图之上, 保证临时障碍清晰可见 */
  var cbd=document.getElementById('cb_dyn');
  if(cbd&&cbd.checked&&histLocked&&dynImg&&
     dynImg.naturalWidth===ov.width&&dynImg.naturalHeight===ov.height)
    ctx.drawImage(dynImg,0,0);
  /* 车体自过滤框: 与导航状态无关, 只要有位姿就画 (独立于下面的路径绘制) */
  drawSelfFilterBox();
  var meta=navData?navData.meta:null;
  if(meta&&meta.res){
    if(navData&&navData.plan&&navData.plan.length>1)
      drawPath(navData.plan,meta,'rgba(57,211,83,0.9)',Math.max(2,ov.width/240));
    if(navData&&navData.local_plan&&navData.local_plan.length>1)
      drawPath(navData.local_plan,meta,'rgba(255,204,0,0.95)',Math.max(2,ov.width/240));
    /* 自建 A* 兜底路径 (黄色虚线): /plan 不可用时 C++ 节点规划, 供确认兜底路线 */
    if(navData&&navData.astar_plan&&navData.astar_plan.length>1){
      ctx.save();ctx.setLineDash([8,5]);
      drawPath(navData.astar_plan,meta,'rgba(255,235,59,1.0)',Math.max(2,ov.width/220));
      ctx.restore()}
    if(navData&&navData.goal)
      drawMark(navData.goal,meta,'#ff5544',true)
    drawHoldOverlay(meta)
    drawChaseOverlay(meta)
    drawFollowOverlay(meta)}

  if(!lastPose||!lastPose.ok||
     lastPose.res===null||lastPose.xmin===null||lastPose.ymin===null)return;
  var col=Math.floor((lastPose.x-lastPose.xmin)/lastPose.res);
  var row=ov.height-1-Math.floor((lastPose.y-lastPose.ymin)/lastPose.res);
  if(col<-20||col>ov.width+20||row<-20||row>ov.height+20)return;
  var mapRes=lastPose.res;
  var bodyLength=(lastPose.length||0.60)/mapRes;
  var bodyWidth=(lastPose.width||0.45)/mapRes;
  var halfL=bodyLength/2,halfW=bodyWidth/2;
  var arrowLength=Math.max(bodyLength*0.45,6);
  ctx.save();
  ctx.translate(col,row);
  /* 图像 y 轴向下，因此地图 yaw 在 canvas 中取反。 */
  ctx.rotate(-lastPose.yaw);
  ctx.fillStyle='rgba(255,72,32,0.30)';
  ctx.strokeStyle='#ff4820';
  ctx.lineWidth=Math.max(2,bodyWidth/12);
  ctx.beginPath();
  ctx.rect(-halfL,-halfW,bodyLength,bodyWidth);
  ctx.fill();ctx.stroke();
  /* 车头方向箭头：车体 x 正方向为前方。 */
  ctx.fillStyle='rgba(255,72,32,0.95)';
  ctx.beginPath();
  ctx.moveTo(halfL+arrowLength*0.35,0);
  ctx.lineTo(halfL-arrowLength*0.35,-Math.max(bodyWidth*0.35,4));
  ctx.lineTo(halfL-arrowLength*0.35,Math.max(bodyWidth*0.35,4));
  ctx.closePath();ctx.fill();ctx.stroke();
  ctx.restore()}
/* 车体自过滤框: 以车体中心为中心、边长 self_filter_size 的轴对齐正方形.
   与 C++ clearVehicleFootprint 完全一致: 该框内的占据格会被彻底清除
   (历史图/实时图/动态图/静态基准都扣, 且膨胀后会再扣一次).
   因此正常情况下框内应恒为空白背景, 若仍出现黑色说明自过滤未生效. */
function drawSelfFilterBox(){
  var cb=document.getElementById('cb_sf');
  if(!cb||!cb.checked)return;
  if(!lastPose||!lastPose.ok||
     lastPose.res===null||lastPose.xmin===null||lastPose.ymin===null)return;
  /* 车体中心 (cx, cy): /pose 的 x/y 就是按 lidar_to_base 反推的车体中心 */
  var cx=lastPose.x,cy=lastPose.y;
  if(cx===null||cx===undefined||cy===null||cy===undefined)return;
  var size=Number(lastPose.self_filter_size);
  if(!isFinite(size)||size<=0)return;
  /* 底图元数据: w2c 依赖 xmin/ymin/res/height, /pose 与 /nav 都带 */
  var m=(navData&&navData.meta&&navData.meta.res)?navData.meta:lastPose;
  if(m.res===null||m.res===undefined||
     m.xmin===null||m.xmin===undefined||
     m.ymin===null||m.ymin===undefined||
     m.height===null||m.height===undefined)return;
  var half=size/2;
  /* w2c 接收 {x,y} 对象, 返回 {c,r} 像素坐标 */
  var p0=w2c({x:cx-half,y:cy-half},m);
  var p1=w2c({x:cx+half,y:cy+half},m);
  if(!isFinite(p0.c)||!isFinite(p1.c))return;
  var x=Math.min(p0.c,p1.c),y=Math.min(p0.r,p1.r);
  var w=Math.max(1,Math.abs(p1.c-p0.c)),h=Math.max(1,Math.abs(p1.r-p0.r));
  ctx.save();
  /* 半透明填充: 让框内区域一眼可见 (框内黑色即"未被清除的残留") */
  ctx.fillStyle='rgba(0,255,80,0.28)';
  ctx.fillRect(x,y,w,h);
  /* 实线粗描边 (原虚线在小尺寸下几乎看不见) */
  ctx.strokeStyle='#00ff40';
  ctx.lineWidth=Math.max(4,ov.width/140);
  ctx.strokeRect(x,y,w,h);
  /* 角标: 尺寸文字, 便于确认 self_filter_size 实际生效值 */
  ctx.font='bold '+Math.max(11,ov.width/85)+'px monospace';
  ctx.fillStyle='#00ff40';
  ctx.strokeStyle='rgba(0,0,0,0.9)';
  ctx.lineWidth=3;
  var label=size.toFixed(2)+'m';
  ctx.strokeText(label,x+w+6,y+Math.max(11,ov.width/85));
  ctx.fillText(label,x+w+6,y+Math.max(11,ov.width/85));
  ctx.restore()}
/* 占点叠加层: 占点中心 + 占点范围圈 + 拦截目标连线 + 阻挡站位点.
   数据来源: /hold_state (含 viz: [中心, 阻挡站位点]) 与 /pose (车体中心).
   圈半径按地图分辨率换算成像素, 与 C++ 端 hold_point_radius 完全一致. */
function drawHoldOverlay(meta){
  if(!holdEnabled||!holdState)return;
  var cx=holdState.x,cy=holdState.y,rad=holdState.radius;
  if(cx===undefined||cy===undefined||!meta||!meta.res)return;
  var c=w2c({x:cx,y:cy},meta);
  /* 占点范围圈: 半径内的障碍才算威胁, 与 C++ 判定半径一致 */
  if(rad!==undefined&&rad>0){
    var rpx=rad/meta.res;
    ctx.save();
    ctx.beginPath();ctx.arc(c.c,c.r,rpx,0,Math.PI*2);
    ctx.fillStyle='rgba(255,210,127,0.10)';ctx.fill();
    ctx.strokeStyle='rgba(255,210,127,0.85)';
    ctx.lineWidth=Math.max(2,ov.width/300);
    ctx.setLineDash([9,6]);ctx.stroke();
    ctx.restore()}
  /* 占点中心十字标记 */
  ctx.save();
  ctx.strokeStyle='#ffd27f';ctx.lineWidth=Math.max(2,ov.width/260);
  var m=Math.max(5,ov.width/110);
  ctx.beginPath();
  ctx.moveTo(c.c-m,c.r);ctx.lineTo(c.c+m,c.r);
  ctx.moveTo(c.c,c.r-m);ctx.lineTo(c.c,c.r+m);
  ctx.stroke();ctx.restore();
  /* 威胁障碍: 红点 + "障碍 -> 中心"预测行进路线 (虚线) */
  if(holdState.threat&&holdState.threat_x!==undefined){
    var t={x:holdState.threat_x,y:holdState.threat_y};
    var q=w2c(t,meta);
    ctx.save();
    ctx.beginPath();ctx.arc(q.c,q.r,Math.max(4,ov.width/100),0,Math.PI*2);
    ctx.fillStyle='rgba(255,85,68,0.85)';ctx.fill();
    ctx.strokeStyle='#ff5544';ctx.lineWidth=2;ctx.stroke();
    /* 障碍 -> 占点中心 的连线: 即需要被阻挡的行进路线 */
    ctx.beginPath();ctx.setLineDash([6,5]);
    ctx.moveTo(q.c,q.r);ctx.lineTo(c.c,c.r);
    ctx.strokeStyle='rgba(255,85,68,0.9)';
    ctx.lineWidth=Math.max(2,ov.width/300);ctx.stroke();
    ctx.restore()}
  /* 阻挡站位点 (车体应到达的位置): 第 2 个路径点 */
  var viz=holdState.viz;
  if(viz&&viz.length>=2){
    var b=w2c(viz[1],meta);
    ctx.save();
    ctx.beginPath();ctx.arc(b.c,b.r,Math.max(5,ov.width/90),0,Math.PI*2);
    ctx.strokeStyle='#7fd0ff';ctx.lineWidth=Math.max(2,ov.width/280);
    ctx.stroke();
    ctx.beginPath();ctx.setLineDash([4,4]);
    ctx.moveTo(b.c,b.r);ctx.lineTo(c.c,c.r);
    ctx.strokeStyle='rgba(127,208,255,0.8)';ctx.stroke();
    ctx.restore()}
}
/* 追击叠加层: 当前目标 + 环带圈 + 重定位搜索圈 + 车体->目标连线 + 期望站位点 +
   目标历史轨迹. 数据来源: /chase_state (含 target_x/y, radius, track_radius,
   viz, track). */
function drawChaseOverlay(meta){
  if(!chaseEnabled||!chaseState||!meta||!meta.res)return;
  var s=chaseState;
  /* 目标历史轨迹: 青色折线 */
  var track=s.track;
  if(track&&track.length>1){
    ctx.save();
    drawPath(track,meta,'rgba(127,224,255,0.85)',Math.max(2,ov.width/260));
    ctx.restore()}
  var tx=s.target_x,ty=s.target_y;
  if(s.valid&&tx!==undefined&&ty!==undefined){
    var c=w2c({x:tx,y:ty},meta);
    /* 环带圈: 半径按地图分辨率换算成像素, 与 C++ 端 chase_radius 一致 */
    if(s.radius!==undefined&&s.radius>0){
      var rpx=s.radius/meta.res;
      ctx.save();
      ctx.beginPath();ctx.arc(c.c,c.r,rpx,0,Math.PI*2);
      ctx.fillStyle='rgba(127,224,255,0.10)';ctx.fill();
      ctx.strokeStyle='rgba(127,224,255,0.85)';
      ctx.lineWidth=Math.max(2,ov.width/300);
      ctx.setLineDash([9,6]);ctx.stroke();
      ctx.restore()}
    /* 重定位搜索圈: 半径 chase_track_radius, 圆心=当前目标位置.
       与 C++ 端 findNearestObstacleCluster 的搜索窗口一致, 始终显示。 */
    if(s.track_radius!==undefined&&s.track_radius>0){
      var srpx=s.track_radius/meta.res;
      ctx.save();
      ctx.beginPath();ctx.arc(c.c,c.r,srpx,0,Math.PI*2);
      ctx.strokeStyle='rgba(127,224,255,0.55)';
      ctx.lineWidth=Math.max(1.5,ov.width/420);
      ctx.setLineDash([3,7]);ctx.stroke();
      ctx.restore()}
    /* 目标标记 */
    ctx.save();
    ctx.beginPath();ctx.arc(c.c,c.r,Math.max(5,ov.width/95),0,Math.PI*2);
    ctx.fillStyle='rgba(80,220,255,0.85)';ctx.fill();
    ctx.strokeStyle='#50dcff';ctx.lineWidth=2;ctx.stroke();
    ctx.font='bold '+Math.max(11,ov.width/80)+'px monospace';
    ctx.fillStyle='#50dcff';
    ctx.fillText('追击目标',
      c.c+Math.max(6,ov.width/120),c.r-Math.max(6,ov.width/120));
    ctx.restore()}
  /* 期望站位点 (车体应到达的位置): 第 2 个路径点 */
  var viz=s.viz;
  if(viz&&viz.length>=2){
    var b=w2c(viz[1],meta);
    ctx.save();
    ctx.beginPath();ctx.arc(b.c,b.r,Math.max(5,ov.width/90),0,Math.PI*2);
    ctx.strokeStyle='#9be7ff';ctx.lineWidth=Math.max(2,ov.width/280);
    ctx.stroke();
    ctx.beginPath();ctx.setLineDash([4,4]);
    ctx.moveTo(b.c,b.r);ctx.lineTo(w2c(viz[0],meta).c,w2c(viz[0],meta).r);
    ctx.strokeStyle='rgba(155,231,255,0.8)';ctx.stroke();
    ctx.restore()}
}
/* 开始运动后: 前视点 + 车体中心->前视点向量 + 车体系坐标系 */
function drawFollowOverlay(meta){
  if(!lkData||!lkData.ok||!lkData.active)return;
  var cbl=document.getElementById('cb_lk');
  var cbc=document.getElementById('cb_cs');
  if(cbl&&cbl.checked)drawLookahead(lkData,meta);
  if(cbc&&cbc.checked)
    drawBodyFrame(lkData,meta,{x:'#ff9d3d',y:'#ff9d3d'})}
/* ---------- 占点 (hold point) 行为 ---------- */
var holdPickMode=false;   /* "选择占点位置"按钮按下后进入选点态, 下次点击地图即设占点中心 */
var holdEnabled=false;    /* C++ 节点回读的占点模式开关 */
var holdState=null;       /* /hold_state 轮询结果: {enabled,x,y,radius,threat,viz,...} */
function refreshHoldStat(){
  var el=document.getElementById('holdstat');
  if(!el)return;
  if(!holdEnabled){
    el.textContent='占点: 未启用（点击“选择占点位置”，再点历史累计图上的目标位置即可开始占点）';
    el.style.color='#ffd27f';return}
  var s=holdState||{};
  var txt='占点: 已启用  中心=('+(s.x!==undefined?s.x.toFixed(2):'--')+', '+
    (s.y!==undefined?s.y.toFixed(2):'--')+') m  范围='+
    (s.radius!==undefined?s.radius.toFixed(2):'--')+' m';
  if(s.threat){
    txt+='  | 拦截目标: 障碍('+s.threat_x.toFixed(2)+', '+s.threat_y.toFixed(2)+
      ') 距中心 '+(s.threat_dist!==undefined?
        s.threat_dist.toFixed(2):'--')+' m';
    el.style.color='#ff8f4d'}
  else{txt+='  | 驻守中（范围内暂无威胁）';el.style.color='#5f5'}
  el.textContent=txt}
function setHoldBtn(){
  var b=document.getElementById('holdbtn');
  if(!b)return;
  if(holdPickMode){b.textContent='请点击地图选点…';b.style.background='#5c4a1d'}
  else if(holdEnabled){b.textContent='解除占点';b.style.background='#5c1d1d'}
  else{b.textContent='选择占点位置';b.style.background='#333'}}
document.getElementById('holdbtn').onclick=function(){
  if(holdPickMode){
    /* 再点一次 => 退出选点态; 若已在占点则解除占点 */
    holdPickMode=false;
    if(holdEnabled)sendHold(false,null,null);
    else setHoldBtn();
    return}
  if(holdEnabled){
    /* 已占点: 再按进入"改占点中心"的选点态 */
    holdPickMode=true;setHoldBtn();
    setPlanMsg('请点击历史累计图上新的占点位置 (再次点击此按钮取消)','#ffd27f');
    return}
  holdPickMode=true;setHoldBtn();
  setPlanMsg('请点击历史累计图上的目标位置作为占点中心','#ffd27f')};
function holdRadiusVal(){
  var el=document.getElementById('hrad');
  return el?(parseFloat(el.value)/10):1.5}
function holdStandoffVal(){
  var el=document.getElementById('hstd');
  return el?(parseFloat(el.value)/100):1.10}
function sendHold(on,col,row){
  var url='/hold?on='+(on?'1':'0')+'&radius='+holdRadiusVal().toFixed(3)+
    '&standoff='+holdStandoffVal().toFixed(2);
  if(col!==null&&col!==undefined)url+='&c='+col+'&r='+row;
  fetch(url).then(function(r){return r.json()}).then(function(j){
    if(j.ok){
      holdEnabled=on;
      if(on)setPlanMsg('占点已启用'+(j.x!==undefined?
        (': 中心=('+j.x.toFixed(2)+', '+j.y.toFixed(2)+') m'):''),'#5f5');
      else setPlanMsg('占点已解除, 车辆交还普通导航','#fa6');
      holdPickMode=false;setHoldBtn();
      pollHold()}
    else setPlanMsg('占点操作失败: '+(j.msg||'未知错误'),'#f66')})
   .catch(function(){setPlanMsg('占点请求失败','#f66')})}
function pollHold(){
  fetch('/hold_state').then(function(r){return r.json()}).then(function(j){
    holdState=j.ok?j:null;
    if(j.ok)holdEnabled=!!j.enabled;
    setHoldBtn();refreshHoldStat();drawOverlay()}).catch(function(){})}
setInterval(pollHold,300);pollHold();
/* 占点范围滑条: 拖动即同步数值框, 并实时下发到 C++ 节点 */
function holdRadiusChanged(){
  var el=document.getElementById('hrad');
  var n=document.getElementById('n_hrad');
  if(n)n.value=(parseFloat(el.value)/10).toFixed(1);
  var stat=document.getElementById('holdstat');
  if(stat)stat.textContent='占点: 正在设置占点范围 '+(parseFloat(el.value)/10).toFixed(1)+' m ...';
  if(holdEnabled)sendHold(true,null,null)}
document.getElementById('hrad').addEventListener('input',holdRadiusChanged);
document.getElementById('n_hrad').addEventListener('change',function(){
  var v=parseFloat(this.value);
  if(!isFinite(v))return;
  v=Math.min(20,Math.max(0.1,v));
  document.getElementById('hrad').value=Math.round(v*10);
  holdRadiusChanged()});
/* 阻挡站位间距系数滑条: 拖动即同步数值框, 并实时下发到 C++ 节点 */
function holdStandoffChanged(){
  var el=document.getElementById('hstd');
  var n=document.getElementById('n_hstd');
  if(n)n.value=(parseFloat(el.value)/100).toFixed(2);
  var stat=document.getElementById('holdstat');
  if(stat)stat.textContent='占点: 正在设置站位间距系数 '+
    (parseFloat(el.value)/100).toFixed(2)+' × ...';
  if(holdEnabled)sendHold(true,null,null)}
document.getElementById('hstd').addEventListener('input',holdStandoffChanged);
document.getElementById('n_hstd').addEventListener('change',function(){
  var v=parseFloat(this.value);
  if(!isFinite(v))return;
  v=Math.min(10,Math.max(1,v));
  document.getElementById('hstd').value=Math.round(v*100);
  holdStandoffChanged()});
holdStandoffChanged();
/* ---------- 追击 (chase) 行为 ---------- */
var chasePickMode=false;  /* "选择追击目标"按钮按下后进入选点态, 下次点击地图即设目标点 */
var chaseEnabled=false;   /* C++ 节点回读的追击模式开关 */
var chaseState=null;      /* /chase_state 轮询结果: {enabled,valid,target_x,target_y,radius,track_radius,track,viz,...} */
function refreshChaseStat(){
  var el=document.getElementById('chasestat');
  if(!el)return;
  if(!chaseEnabled){
    el.textContent='追击: 未启用（先“锁定历史累计图”, 再点“选择追击目标”并点选地图上的目标物体）';
    el.style.color='#7fe0ff';return}
  var s=chaseState||{};
  if(!s.valid){
    el.textContent='追击: 已启用, 当前无目标（搜索半径内未找到障碍簇, 已停车等待其重新出现）';
    el.style.color='#ffb347';return}
  var txt='追击: 目标=('+s.target_x.toFixed(2)+', '+s.target_y.toFixed(2)+
    ') m  环带半径='+(s.radius!==undefined?s.radius.toFixed(2):'--')+' m';
  if(s.track!==undefined)txt+='  | 轨迹点='+s.track;
  el.style.color='#5f5';
  el.textContent=txt}
function setChaseBtn(){
  var b=document.getElementById('chasebtn');
  if(!b)return;
  if(chasePickMode){b.textContent='请点击地图选目标…';b.style.background='#1d4a5c'}
  else if(chaseEnabled){b.textContent='解除追击';b.style.background='#5c1d1d'}
  else{b.textContent='选择追击目标';b.style.background='#333'}}
document.getElementById('chasebtn').onclick=function(){
  if(chasePickMode){
    /* 再点一次 => 退出选点态; 若已在追击则解除 */
    chasePickMode=false;
    if(chaseEnabled)sendChase(false,null,null);
    else setChaseBtn();
    return}
  if(!histLocked){
    setPlanMsg('请先“锁定历史累计图”, 再选择追击目标','#ffb347');return}
  if(chaseEnabled){
    /* 已追击: 再按进入"改目标"的选点态 */
    chasePickMode=true;setChaseBtn();
    setPlanMsg('请点击历史累计图上新的追击目标 (再次点击此按钮取消)','#7fe0ff');
    return}
  chasePickMode=true;setChaseBtn();
  setPlanMsg('请点击历史累计图上的目标物体 (将锁定最近障碍簇作为追击目标)','#7fe0ff')};
function chaseRadiusVal(){
  var el=document.getElementById('crad');
  return el?(parseFloat(el.value)/10):1.0}
function sendChase(on,col,row){
  var url='/chase?on='+(on?'1':'0')+'&radius='+chaseRadiusVal().toFixed(3);
  if(col!==null&&col!==undefined)url+='&c='+col+'&r='+row;
  fetch(url).then(function(r){return r.json()}).then(function(j){
    if(j.ok){
      chaseEnabled=on;
      if(on)setPlanMsg('追击已启用'+(j.x!==undefined?
        (': 目标点=('+j.x.toFixed(2)+', '+j.y.toFixed(2)+') m, 将就近锁定目标'):''),'#5f5');
      else setPlanMsg('追击已解除, 车辆交还普通导航','#fa6');
      chasePickMode=false;setChaseBtn();
      pollChase()}
    else setPlanMsg('追击操作失败: '+(j.msg||'未知错误'),'#f66')})
   .catch(function(){setPlanMsg('追击请求失败','#f66')})}
function pollChase(){
  fetch('/chase_state').then(function(r){return r.json()}).then(function(j){
    chaseState=j.ok?j:null;
    if(j.ok)chaseEnabled=!!j.enabled;
    setChaseBtn();refreshChaseStat();drawOverlay()}).catch(function(){})}
setInterval(pollChase,300);pollChase();
/* 环带半径滑条: 拖动即同步数值框, 并实时下发到 C++ 节点 */
function chaseRadiusChanged(){
  var el=document.getElementById('crad');
  var n=document.getElementById('n_crad');
  if(n)n.value=(parseFloat(el.value)/10).toFixed(1);
  var stat=document.getElementById('chasestat');
  if(stat)stat.textContent='追击: 正在设置环带半径 '+(parseFloat(el.value)/10).toFixed(1)+' m ...';
  if(chaseEnabled)sendChase(true,null,null)}
var chaseRadiusEl=document.getElementById('crad');
if(chaseRadiusEl)chaseRadiusEl.addEventListener('input',chaseRadiusChanged);
var chaseRadiusNumEl=document.getElementById('n_crad');
if(chaseRadiusNumEl)chaseRadiusNumEl.addEventListener('change',function(){
  var v=parseFloat(this.value);
  if(!isFinite(v))return;
  v=Math.min(20,Math.max(0.1,v));
  document.getElementById('crad').value=Math.round(v*10);
  chaseRadiusChanged()});
/* ---------- 最终地图点击 -> 发送 Nav2 导航目标 / 占点选点 / 追击选点 ---------- */
document.getElementById('wrap').addEventListener('click',function(e){
  if(!img.naturalWidth||!img.naturalHeight)return;
  var rect=img.getBoundingClientRect();
  if(e.clientX<rect.left||e.clientX>rect.right||
     e.clientY<rect.top||e.clientY>rect.bottom)return;
  var col=Math.floor((e.clientX-rect.left)*img.naturalWidth/rect.width);
  var row=Math.floor((e.clientY-rect.top)*img.naturalHeight/rect.height);
  /* 占点选点态: 本次点击用于设定占点中心 (不发给 Nav2) */
  if(holdPickMode){sendHold(true,col,row);return}
  /* 追击选点态: 本次点击用于设定追击目标 (不发给 Nav2) */
  if(chasePickMode){sendChase(true,col,row);return}
  setPlanMsg('正在发送导航目标到 Nav2 ...','#8cf');
  fetch('/goal?c='+col+'&r='+row).then(function(resp){return resp.json()})
   .then(function(j){
    if(j.ok)setPlanMsg('目标已发送: ('+j.x.toFixed(2)+', '+j.y.toFixed(2)+
      ') m, Nav2 将规划路径并实时避障; 再次点击可更换目标','#5f5');
    else setPlanMsg('发送失败: '+(j.msg||'未知错误'),'#f66');
    pollNav()}).catch(function(){setPlanMsg('目标发送请求失败','#f66')})});
document.getElementById('wrap').addEventListener('contextmenu',function(e){
  var rect=img.getBoundingClientRect();
  if(e.clientX>=rect.left&&e.clientX<=rect.right&&
     e.clientY>=rect.top&&e.clientY<=rect.bottom){
    e.preventDefault();cancelNav()}});
function cancelNav(){
  fetch('/cancel').then(function(r){return r.json()}).then(function(j){
    if(j.ok)setPlanMsg('已请求取消当前导航','#fa6');
    else setPlanMsg(j.msg||'当前没有导航任务','#888');
    pollNav()}).catch(function(){})}
document.getElementById('navcancel').onclick=cancelNav;
/* ---------- 开始/停止运动: 由 C++ 节点按 Nav2 全局路径下发运动帧 ---------- */
function motionCmd(action){
  var el=document.getElementById('motionstat');
  el.textContent='运动: 正在'+(action==='start'?'开始':'停止')+' ...';
  el.style.color='#8cf';
  fetch('/'+action).then(function(r){return r.json()}).then(function(j){
    if(j.ok){
      el.textContent='运动: '+(j.msg||(action==='start'?'已开始':'已停止'));
      el.style.color=(action==='start')?'#5f5':'#fa6';
      setPlanMsg(el.textContent,(action==='start')?'#5f5':'#fa6')}
    else{
      el.textContent='运动: '+(j.msg||'操作失败');
      el.style.color='#f66';
      setPlanMsg('运动指令失败: '+(j.msg||'未知错误'),'#f66')}
    pollMotion()}).catch(function(){
      el.textContent='运动: 请求失败';
      el.style.color='#f66'})}
document.getElementById('startmove').onclick=function(){motionCmd('start')};
document.getElementById('stopmove').onclick=function(){motionCmd('stop')};
function pollMotion(){
  fetch('/motion').then(function(r){return r.json()}).then(function(j){
    if(!j.ok)return;
    var el=document.getElementById('motionstat');
    if(el.style.color==='#8cf')return;   /* 正在执行指令, 不覆盖提示 */
    if(j.active){
      el.textContent='运动: 进行中  control='+j.control+
        '  vx='+j.vx.toFixed(2)+' m/s  vy='+j.vy.toFixed(2)+' m/s';
      el.style.color='#5f5'}
    else if(j.control||j.vx||j.vy){
      el.textContent='运动: 已停止 (control='+j.control+')';
      el.style.color='#fa6'}
    else{
      el.textContent='运动: 未开始（路径规划完成后点“开始运动”）';
      el.style.color='#888'}}).catch(function(){})}
setInterval(pollMotion,500);pollMotion();
document.getElementById('histrefresh').onclick=function(){
  setPlanMsg('正在重建累计图: 清空历史点云并重新累积 ...','#8cf');
  fetch('/refresh_history').then(function(resp){return resp.json()})
   .then(function(j){
     if(j.ok){setPlanMsg('已清空历史点云, 累计图与 Nav2 全局地图正在重建','#5f5')}
     else{setPlanMsg('刷新失败: 目标节点参数服务不可用','#f66')}})
   .catch(function(){setPlanMsg('刷新请求失败','#f66')})};
/* ---------- 历史累计图导出 / 导入 ---------- */
document.getElementById('mapexport').onclick=function(){
  setPlanMsg('正在导出历史累计图 (PGM+YAML+点云) ...','#8cf');
  /* 通过隐藏链接触发浏览器下载: 由服务端 /export_map 返回 zip */
  fetch('/export_map').then(function(resp){
    if(!resp.ok){throw new Error('export failed')}
    return resp.blob()}).then(function(blob){
    var url=URL.createObjectURL(blob);
    var a=document.createElement('a');
    a.href=url;a.download='map_export.zip';
    document.body.appendChild(a);a.click();document.body.removeChild(a);
    URL.revokeObjectURL(url);
    setPlanMsg('历史累计图已导出 (map_export.zip)','#5f5')
  }).catch(function(){
    setPlanMsg('导出失败: 请确认目标节点参数服务可用','#f66')})};
document.getElementById('mapimport').onclick=function(){
  document.getElementById('mapfile').click()};
document.getElementById('mapfile').onchange=function(e){
  var file=e.target.files&&e.target.files[0];
  if(!file)return;
  setPlanMsg('正在导入地图 ('+file.name+') ...','#8cf');
  fetch('/import_map',{method:'POST',body:file})
   .then(function(resp){return resp.json()})
   .then(function(j){
     if(j.ok){setPlanMsg('地图导入完成, 正在刷新显示','#5f5')}
     else{setPlanMsg('导入失败: '+(j.msg||'未知错误'),'#f66')}})
   .catch(function(){setPlanMsg('导入请求失败','#f66')});
  e.target.value=''};
/* ---------- 锁定/解除历史累计图 (动态障碍模式) ---------- */
function updateDynStat(){
  var el=document.getElementById('dynstat');
  var btn=document.getElementById('histlock');
  if(!el)return;
  if(histLocked){
    el.textContent='动态障碍: 已锁定 — 历史累计图已冻结; 新出现的障碍以绿色显示, 不计入历史地图 (仍参与避障)';
    el.style.color='#5f5';
    if(btn){btn.textContent='解除锁定';btn.style.background='#5c1d1d';
            btn.style.color='#fdd';btn.style.borderColor='#8f2f2f'}}
  else{
    el.textContent='动态障碍: 未锁定（点击“锁定历史累计图”后，新出现的障碍将以绿色标记，且不计入历史地图）';
    el.style.color='#8c8';
    if(btn){btn.textContent='锁定历史累计图';btn.style.background='#333';
            btn.style.color='#dfd';btn.style.borderColor='#2f8f45'}}}
document.getElementById('histlock').onclick=function(){
  var want=!histLocked;
  var el=document.getElementById('dynstat');
  if(el){el.textContent='动态障碍: 正在'+(want?'锁定':'解除')
    +(want?' (导出先验地图, 约需 1~3 s)':'')+' ...';el.style.color='#8cf'}
  fetch('/lock?on='+(want?'1':'0')).then(function(r){return r.json()})
   .then(function(j){
     if(j.ok){
       histLocked=want;
       if(!want){dynImg=null}
       updateDynStat();drawOverlay();
       /* 锁定: 提示先验地图导出与 3D 重定位节点联动结果;
          解锁: 提示历史恢复累积与 3D 节点已停用 */
       var extra='';
       if(want){
         extra=(j.exported?'; 先验地图已导出':' ; 先验地图导出失败('+(j.export_msg||'')+')')
           +(j.enabled3d?', 3D 重定位已启用':' [3D 节点未启动, 未启用]');}
       else{
         extra=(j.enabled3d===false?' [3D 节点未启动]':', 3D 重定位已停用');}
       setPlanMsg((want
         ?'已锁定历史累计图: 历史图冻结为静态基准, 此后新出现的障碍判为动态障碍(绿色)'
         :'已解除锁定: 恢复历史累积, 绿色动态障碍层已清空')+extra,
         want?'#5f5':'#fa6')}
     else{
       if(el){el.textContent='动态障碍: '+(j.msg||'操作失败');el.style.color='#f66'}
       setPlanMsg('锁定操作失败: '+(j.msg||'未知错误'),'#f66')}})
   .catch(function(){
     if(el){el.textContent='动态障碍: 请求失败';el.style.color='#f66'}})};
updateDynStat();
document.getElementById('cb_gcm').addEventListener('change',drawOverlay);
document.getElementById('cb_lcm').addEventListener('change',drawOverlay);
document.getElementById('cb_dyn').addEventListener('change',drawOverlay);
document.getElementById('cb_sf').addEventListener('change',drawOverlay);
document.getElementById('cb_lk').addEventListener('change',drawOverlay);
document.getElementById('cb_cs').addEventListener('change',drawOverlay);
/* 开始运动后轮询前视点/速度矢量/车体系叠加数据 (仅运动跟随中有效) */
function pollLookahead(){
  fetch('/lookahead').then(function(r){return r.json()}).then(function(j){
    lkData=j;updateRemain(j);drawOverlay()}).catch(function(){})}
/* 剩余距离状态行: 沿路径剩余弧长 + 到终点直线距离(与减速/到点判定同量) */
function updateRemain(j){
  var el=document.getElementById('remainstat');
  if(!el)return;
  if(j&&j.ok&&j.active&&(j.remaining_m!==undefined||j.dist_end_m!==undefined)){
    el.textContent='剩余距离: 沿路径 '+
      (j.remaining_m!==undefined?j.remaining_m.toFixed(2):'--')+' m'+
      '  到终点直线 '+
      (j.dist_end_m!==undefined?j.dist_end_m.toFixed(2):'--')+' m';
    el.style.color='#5f5'}
  else{
    el.textContent='剩余距离: --（未在跟随 / 已到达终点）';
    el.style.color='#888'}}
setInterval(pollLookahead,200);pollLookahead();
function pollNav(){
  fetch('/nav').then(function(resp){return resp.json()}).then(function(j){
    navData=j;drawOverlay();
    var el=document.getElementById('plan');
    if(j.status&&j.status!=='idle'){
      el.textContent='Nav2 ['+j.status+']'+
        (j.goal?(' → ('+j.goal.x.toFixed(2)+', '+j.goal.y.toFixed(2)+') m'):'')+
        (j.plan?('  全局路径 '+j.plan.length+' 点'):'')+
        (j.local_plan?('  局部路径 '+j.local_plan.length+' 点'):'');
      el.style.color='#5f5'}
    else{
      el.textContent='Nav2 空闲: 单击地图发送目标点, 右键地图或按钮取消导航';
      el.style.color='#888'}}).catch(function(){})}
/* 路径刷新轮询: 保守提速到 250 ms (Nav2 路径/状态更跟手) */
setInterval(pollNav,250);pollNav();
/* 代价地图 PNG: 后端重投影到底图网格, 无数据时 404 忽略即可 */
function refreshCostmap(kind){
  var im=new Image();
  im.onload=function(){
    if(kind==='g')gcmImg=im;else lcmImg=im;
    drawOverlay()};
  im.src=(kind==='g'?'/gcm.png':'/lcm.png')+'?t='+Date.now()}
function refreshCostmaps(){refreshCostmap('g');refreshCostmap('l')}
/* 动态障碍清除后红色(cost 100)区域的"看得见"延迟下限就是这个轮询周期,
   故压到 200 ms, 与 local costmap 10 Hz / global costmap 5 Hz 的
   publish_frequency 对齐, 视觉上红色随障碍消失而即时消退 */
setInterval(refreshCostmaps,200);refreshCostmaps();
/* 动态障碍绿色层: 仅在锁定时拉取, 未锁定/无数据时 404 忽略。
   后端是按请求现场渲染最新消息(不缓存), 开销很小, 故与红色层同步压到 200 ms */
function refreshDynamic(){
  if(!histLocked)return;
  var im=new Image();
  im.onload=function(){dynImg=im;drawOverlay()};
  im.onerror=function(){dynImg=null;drawOverlay()};
  im.src='/dyn.png?t='+Date.now()}
setInterval(refreshDynamic,200);
function pollPose(){
  fetch('/pose').then(r=>r.json()).then(function(p){
    lastPose=p;drawOverlay();
    var el=document.getElementById('pose');
    if(p.ok){
      el.textContent='pose: x='+p.x.toFixed(2)+' m  y='+p.y.toFixed(2)+
        ' m  yaw='+(p.yaw*180/Math.PI).toFixed(1)+' deg  ('+p.age+'s ago)'+
        (p.corrected?'  [SE3 已修正]':'  [未修正]');
      el.style.color=(p.age>2)?'#f66':'#fa6';
    }else{
      el.textContent='pose: waiting odometry ...';
      el.style.color='#fa6'}}).catch(function(){})}
var imgh=document.getElementById('imgh');
var histstat=document.getElementById('histstat');
var mapFail=0;
img.onload=function(){
  mapFail=0;
  document.getElementById('status').textContent='live: final map';
  histstat.textContent='实时单帧地图预览（不参与规划）';
  histstat.style.color='#9f9';
  if(img.naturalWidth&&img.naturalHeight&&
     (ov.width!=img.naturalWidth||ov.height!=img.naturalHeight)){
    ov.width=img.naturalWidth;ov.height=img.naturalHeight}
  drawOverlay()};
img.onerror=function(){
  mapFail++;
  document.getElementById('status').textContent='waiting for final map ...';
  if(mapFail>=10){
    histstat.textContent='最终地图: 未收到 /realtime_pgm/image_history ! '
      +'请重新编译并重启 C++ 节点: colcon build --packages-select '
      +'test_navigation 后重新 ros2 launch, 再刷新本页';
    histstat.style.color='#f66'}};
imgh.onload=function(){document.getElementById('wraph').style.display='inline-block'};
/* 地图刷新轮询: 保守提速到 250 ms (与 C++ render_period_ms=150 配合,
   视觉上更跟手; 后端是全量重投, 该周期已比渲染周期略慢, 避免空转拉取) */
setInterval(function(){
  img.src='/image_history.png?t='+Date.now();
  imgh.src='/image.png?t='+Date.now()},250);
setInterval(pollPose,150);
pollPose();

/* ---------- 右侧: 实时可交互点云侧视图 ---------- */
var pc=document.getElementById('pc'),pctx=pc.getContext('2d');
var cloud=null,pcN=0,pcView=null,pcAxis='x',pcDrag=null,pcLast=null;
function hOf(x,y){
  if(pcAxis=='x')return x;
  if(pcAxis=='y')return y;
  return Math.sqrt(x*x+y*y)}
function Xof(h){return(h-pcView.x0)*pcView.sx}
function Yof(z){return pc.height-(z-pcView.z0)*pcView.sz}
function pcXY(e){
  var r=pc.getBoundingClientRect();
  return{px:(e.clientX-r.left)*pc.width/r.width,
         py:(e.clientY-r.top)*pc.height/r.height}}
function pcResize(){
  var w=Math.round(pc.clientWidth||600);
  if(pc.width!=w){pc.width=w;pc.height=420}}
function pcFit(){
  if(!pcN)return;
  pcResize();
  if(!pc.width)return;
  var lo=1e9,hi=-1e9,zl=1e9,zh=-1e9;
  for(var i=0;i<pcN;i++){
    var x=cloud[3*i],y=cloud[3*i+1],z=cloud[3*i+2],h=hOf(x,y);
    if(h<lo)lo=h;if(h>hi)hi=h;if(z<zl)zl=z;if(z>zh)zh=z}
  var f=fmt(),zmin=f.zmin/100-10,zmax=f.zmax/100-10;
  if(zmin<zl)zl=zmin;if(zmax>zh)zh=zmax;
  if(hi-lo<0.3){lo-=0.15;hi+=0.15}
  if(zh-zl<0.3){zl-=0.15;zh+=0.15}
  var mx=(hi-lo)*0.05,mz=(zh-zl)*0.10;
  pcView={x0:lo-mx,z0:zl-mz,
    sx:pc.width/(hi-lo+2*mx),sz:pc.height/(zh-zl+2*mz)}}
function drawZLine(y,label){
  pctx.save();pctx.strokeStyle='rgba(110,255,150,0.9)';pctx.lineWidth=1;
  pctx.setLineDash([6,4]);pctx.beginPath();
  pctx.moveTo(0,y);pctx.lineTo(pc.width,y);pctx.stroke();
  pctx.setLineDash([]);pctx.fillStyle='#9f9';pctx.font='11px sans-serif';
  pctx.fillText(label,5,y-4);pctx.restore()}
function pcDraw(){
  pcResize();
  if(!pc.width||!pc.height)return;
  pctx.fillStyle='#111';pctx.fillRect(0,0,pc.width,pc.height);
  if(!pcN)return;
  if(!pcView)pcFit();
  if(!pcView)return;
  var f=fmt(),zmin=f.zmin/100-10,zmax=f.zmax/100-10;
  var W=pc.width,H=pc.height;
  /* 1m 网格 */
  var stepX=1;while(pcView.sx*stepX<40)stepX++;
  var stepZ=1;while(pcView.sz*stepZ<40)stepZ++;
  pctx.strokeStyle='rgba(255,255,255,0.06)';pctx.lineWidth=1;
  pctx.beginPath();
  for(var gx=Math.ceil(pcView.x0/stepX)*stepX;
      gx<=pcView.x0+W/pcView.sx;gx+=stepX){
    var pxg=Xof(gx);pctx.moveTo(pxg,0);pctx.lineTo(pxg,H)}
  for(var gz=Math.ceil(pcView.z0/stepZ)*stepZ;
      gz<=pcView.z0+H/pcView.sz;gz+=stepZ){
    var pyg=Yof(gz);pctx.moveTo(0,pyg);pctx.lineTo(W,pyg)}
  pctx.stroke();
  /* zmin~zmax 范围色带 */
  var yz1=Yof(zmax),yz0=Yof(zmin);
  pctx.fillStyle='rgba(90,255,140,0.10)';
  pctx.fillRect(0,yz1,W,yz0-yz1);
  /* 点云: 绿=范围内, 灰=范围外 */
  for(var i=0;i<pcN;i++){
    var x=cloud[3*i],y=cloud[3*i+1],z=cloud[3*i+2];
    var px=Xof(hOf(x,y)),py=Yof(z);
    if(px<-2||px>W+2||py<-2||py>H+2)continue;
    pctx.fillStyle=(z>=zmin&&z<=zmax)?'#5fd35f':'rgba(150,150,150,0.30)';
    pctx.fillRect(px,py,1.6,1.6)}
  drawZLine(yz1,'zmax = '+zmax.toFixed(2)+' m');
  drawZLine(yz0,'zmin = '+zmin.toFixed(2)+' m')}
function pollCloud(){
  fetch('/cloud').then(r=>r.json()).then(function(c){
    if(!c.ok||!c.n){
      document.getElementById('pcinfo').textContent='(等待点云...)';return}
    cloud=new Float32Array(c.pts);pcN=c.n;
    document.getElementById('pcinfo').textContent=c.n+' pts';
    if(!pcView)pcFit();
    pcDraw()}).catch(function(){})}
setInterval(pollCloud,250);pollCloud();
pc.addEventListener('mousedown',function(e){
  if(!pcView||!pcN)return;e.preventDefault();
  var p=pcXY(e),f=fmt();
  var dz1=Math.abs(p.py-Yof(f.zmax/100-10));
  var dz0=Math.abs(p.py-Yof(f.zmin/100-10));
  pcDrag=(dz1<=7&&dz1<=dz0)?'zmax':(dz0<=7?'zmin':'pan');
  pcLast={px:p.px,py:p.py}});
window.addEventListener('mouseup',function(){pcDrag=null});
window.addEventListener('mousemove',function(e){
  if(!pcDrag||!pcView)return;
  var p=pcXY(e);
  if(pcDrag=='pan'){
    pcView.x0-=(p.px-pcLast.px)/pcView.sx;
    pcView.z0-=(p.py-pcLast.py)/pcView.sz;
    pcLast={px:p.px,py:p.py};pcDraw();return}
  var z=Math.min(10,Math.max(-10,(pc.height-p.py)/pcView.sz+pcView.z0));
  var t=Math.min(2000,Math.max(0,Math.round((z+10)*100)));
  var f=fmt();
  if(pcDrag=='zmax'){
    if(t<f.zmin)t=f.zmin;
    document.getElementById('zmax').value=t}
  else{
    if(t>f.zmax)t=f.zmax;
    document.getElementById('zmin').value=t}
  onChange();pcDraw()});
pc.addEventListener('mousemove',function(e){
  if(pcDrag||!pcView||!pcN)return;
  var p=pcXY(e),f=fmt();
  var near=Math.abs(p.py-Yof(f.zmax/100-10))<=7||
    Math.abs(p.py-Yof(f.zmin/100-10))<=7;
  pc.style.cursor=near?'ns-resize':'grab'});
pc.addEventListener('mouseleave',function(){
  if(!pcDrag)pc.style.cursor='grab'});
pc.addEventListener('wheel',function(e){
  if(!pcView||!pcN)return;e.preventDefault();
  var p=pcXY(e),k=e.deltaY<0?1.15:1/1.15;
  var h=p.px/pcView.sx+pcView.x0;
  var z=(pc.height-p.py)/pcView.sz+pcView.z0;
  pcView.sx*=k;pcView.sz*=k;
  pcView.x0=h-p.px/pcView.sx;
  pcView.z0=z-(pc.height-p.py)/pcView.sz;
  pcDraw()},{passive:false});
pc.addEventListener('dblclick',function(){pcView=null;pcFit();pcDraw()});
document.getElementById('pcreset').onclick=function(){pcView=null;pcFit();pcDraw()};
document.getElementById('pcaxis').onclick=function(){
  pcAxis=pcAxis=='x'?'y':(pcAxis=='y'?'r':'x');
  this.textContent='水平轴: '+(pcAxis=='r'?'距离 r':pcAxis);
  pcView=null;pcFit();pcDraw()};
window.addEventListener('resize',function(){pcView=null;pcFit();pcDraw()});
</script></body></html>'''


def to_tbar_resolution(res):
    return int(min(TBAR_RES_MAX, max(1, round(res * 100.0))))


def to_tbar_z(z):
    return int(min(TBAR_Z_MAX, max(0, round((z + 10.0) * 100.0))))


def to_tbar_dilate(d):
    return int(min(TBAR_DIL_MAX, max(0, int(d))))


def from_tbar_resolution(v):
    return v / 100.0


def from_tbar_z(v):
    return v / 100.0 - 10.0


def to_tbar_cost_reach(v):
    return int(min(TBAR_CR_MAX, max(0, round(float(v) * 100.0))))


def to_tbar_penalty(v):
    return int(min(TBAR_OP_MAX, max(0, round(float(v) * 100.0))))


def from_tbar_cost_reach(v):
    return v / 100.0


def from_tbar_penalty(v):
    return v / 100.0


def to_tbar_ext(v):
    return int(min(TBAR_EXT_MAX, max(-TBAR_EXT_MAX, round(v * 1000.0))))


def from_tbar_ext(v):
    return v / 1000.0


def costmap_to_rgba(msg, base_meta, out_w, out_h):
    """nav_msgs/OccupancyGrid 代价地图 -> 与底图对齐的 RGBA 图像.
    RViz costmap 配色: cost 1~98 蓝绿->黄红渐变(JET), 99 内切半径紫,
    100 致命红; cost 0(自由) 与 -1(未知/窗口外) 不上色(透明)."""
    info = msg.info
    res_c = float(info.resolution)
    ox = float(info.origin.position.x)
    oy = float(info.origin.position.y)
    w_c, h_c = int(info.width), int(info.height)
    if res_c <= 0.0 or w_c == 0 or h_c == 0:
        return None
    try:
        data = np.asarray(msg.data, dtype=np.int8).reshape(h_c, w_c)
    except (ValueError, TypeError):
        return None

    res_b = float(base_meta['res'])
    # 底图每个像素中心的世界坐标 -> 代价地图栅格索引
    # (OccupancyGrid data 为行主序, 行 0 对应 origin 所在的底边, 因此
    #  世界 y 直接 floor 即为行索引, 无需再翻转; ys 已处理底图第 0 行在顶部)
    xs = float(base_meta['xmin']) + (np.arange(out_w) + 0.5) * res_b
    ys = float(base_meta['ymin']) + \
        (out_h - 1 - np.arange(out_h) + 0.5) * res_b
    cc = np.floor((xs - ox) / res_c).astype(np.int64)
    rr = np.floor((ys - oy) / res_c).astype(np.int64)
    valid = (cc >= 0) & (cc < w_c) & (rr >= 0) & (rr < h_c)
    cc = np.clip(cc, 0, w_c - 1)
    rr = np.clip(rr, 0, h_c - 1)
    cost = data[np.ix_(rr, cc)]  # (out_h, out_w)
    cost[~valid] = -1

    rgba = np.zeros((out_h, out_w, 4), dtype=np.uint8)
    graded = (cost >= 1) & (cost <= 98)
    if graded.any():
        vals = (cost[graded].astype(np.float32) - 1.0) / 97.0 * 255.0
        colored = cv2.applyColorMap(
            vals.astype(np.uint8).reshape(-1, 1), cv2.COLORMAP_JET)
        rgba[graded, 0] = colored[:, 0, 2]
        rgba[graded, 1] = colored[:, 0, 1]
        rgba[graded, 2] = colored[:, 0, 0]
        alpha = 70 + (cost[graded].astype(np.int32) - 1) * 90 // 97
        rgba[graded, 3] = alpha.astype(np.uint8)
    rgba[cost == 99] = (255, 0, 255, 150)   # 内切半径: 紫
    rgba[cost == 100] = (0, 0, 255, 190)    # 致命障碍: 红
    return rgba


def dynamic_obstacle_to_rgba(img, base_meta, out_w, out_h):
    """动态障碍栅格图 (mono8: 0=动态障碍) -> 与底图对齐的绿色 RGBA 图像.
    与 costmap_to_rgba 同样的重投影思路, 但两者几何通常完全一致 (同一份
    配置生成的图), 此时直接锁定时序。非动态障碍格保持透明, 因此绿色只
    覆盖临时障碍, 不会遮住底图。"""
    if img is None:
        return None
    h_s, w_s = img.shape[:2]
    if out_h == h_s and out_w == w_s:
        mask = (img == 0)
    else:
        # 尺寸不一致 (滑条改过 resolution/范围): C++ 侧栅格与底图同源同参,
        # 因此 src 几何 == base 几何; 仍用 base_meta 做一次最近邻重采样兜底
        res_b = float(base_meta['res'])
        if res_b <= 0.0:
            return None
        xs = float(base_meta['xmin']) + (np.arange(out_w) + 0.5) * res_b
        ys = float(base_meta['ymin']) + \
            (out_h - 1 - np.arange(out_h) + 0.5) * res_b
        # 该图由 occupancyToPixels 生成, 同样是"第 0 行在顶部"的图像序,
        # 行索引可直接用 ys 反算 (与 w2c 的翻转约定一致)
        cc = np.clip(((xs - float(base_meta['xmin'])) / res_b)
                     .astype(np.int64), 0, w_s - 1)
        rr = np.clip((((h_s - 1) * res_b + float(base_meta['ymin']) - ys)
                      / res_b).astype(np.int64), 0, h_s - 1)
        mask = (img[np.ix_(rr, cc)] == 0)

    rgba = np.zeros((out_h, out_w, 4), dtype=np.uint8)
    rgba[mask] = (0, 230, 60, 200)   # 动态/可变障碍: 绿
    return rgba


class PgmViewer(Node):

    def __init__(self):
        super().__init__('pgm_viewer')

        self.declare_parameter('image_topic', '/realtime_pgm/image')
        self.declare_parameter('target_node', '/test_navigation')
        self.declare_parameter('http_port', HTTP_PORT)
        self.declare_parameter('odom_topic', '/Odometry')
        # 位姿修正量 (map->odom, SE(3)): 由 C++ 节点的 2D 栅格配准发布。网页显示的
        # 位姿必须落在 **map 系** (底图/路径/代价地图都在 map 系), 故这里订阅它并
        # 叠加到里程计位姿上。
        self.declare_parameter('pose_correction_topic', '/pose_correction')
        # 动态障碍栅格图 (mono8: 0=动态障碍, 其余=非动态障碍)
        # 由 C++ 节点在"锁定历史累计图"后发布, 网页画成绿色可变障碍层
        self.declare_parameter('dynamic_image_topic',
                               '/realtime_pgm/image_dynamic')
        self.declare_parameter('cloud_topic', '/realtime_pgm/cloud')
        self.declare_parameter(
            'history_image_topic', '/realtime_pgm/image_history')
        # ---------- Nav2 导航相关 ----------
        # 导航动作名: 网页点击 -> NavigateToPose -> Nav2 全局规划 + 局部避障
        self.declare_parameter('nav_action_name', 'navigate_to_pose')
        # 全局路径 (planner_server) 与局部路径 (controller_server)
        self.declare_parameter('global_plan_topic', '/plan')
        self.declare_parameter('local_plan_topic', '/local_plan')
        # 自建 A* 兜底路径 (C++ 节点在 /plan 不可用时规划, 网页用黄色显示)
        self.declare_parameter('astar_plan_topic', '/astar_plan')
        # Web 目标点转发话题: 点击目标时除发给 Nav2 NavigateToPose 外, 额外把
        #   目标点发布到该话题 (geometry_msgs/PoseStamped, map 系), 供 C++ 节点
        #   的自建 A* 兜底规划器作为终点 (动作接口不暴露话题, 节点无法直接订阅)。
        self.declare_parameter('goal_pose_topic', '/nav_goal_pose')
        # 占点 (hold point) 相关话题:
        #   status: C++ 节点发布 JSON 状态 (占点中心/范围/威胁/预测路线长度)
        #   viz:    C++ 节点发布 nav_msgs/Path, 第 1 点=占点中心, 第 2 点=阻挡站位点
        self.declare_parameter('hold_point_status_topic', '/hold_point_status')
        self.declare_parameter('hold_point_viz_topic', '/hold_point_viz')
        # 追击 (chase) 相关话题:
        #   status: C++ 节点发布 JSON 状态 (目标位置/环带半径/搜索半径)
        #   viz:    C++ 节点发布 nav_msgs/Path, 第 1 点=目标, 第 2 点=期望站位点
        #   track:  C++ 节点发布 nav_msgs/Path, 目标历史轨迹 (供网页画轨迹线)
        self.declare_parameter('chase_status_topic', '/chase_status')
        self.declare_parameter('chase_viz_topic', '/chase_viz')
        self.declare_parameter('chase_track_topic', '/chase_track')
        # 历史累计图导出/导入工作目录 (C++ 节点读写的中间文件目录)
        self.declare_parameter('export_map_dir', '/tmp/pgm_export')
        self.declare_parameter('import_map_dir', '/tmp/pgm_import')
        # 全局/局部代价地图 (nav_msgs/OccupancyGrid, Nav2 costmap 发布)
        self.declare_parameter('global_costmap_topic',
                               '/global_costmap/costmap')
        self.declare_parameter('local_costmap_topic',
                               '/local_costmap/costmap')
        # 运动帧下发话题 (由 test_navigation 节点发布, 8 字节 uint8)
        self.declare_parameter('motion_cmd_topic', '/nav_motion_cmd')
        # 速度量化系数: 必须与 C++ 节点 motion_vx_scale / motion_vy_scale
        # 以及接收方解析 (raw / 200.0f) 保持一致, 否则网页回显速度不准 (仅影响显示)
        self.declare_parameter('motion_vx_scale', 200.0)
        self.declare_parameter('motion_vy_scale', 200.0)
        # 车体几何与雷达安装外参 (单位: m)
        # 车体长度: 机器人前后方向尺寸, 用于网页绘制方块 (默认 0.60 m)
        self.declare_parameter('robot_length', 0.60)
        # 车体宽度: 机器人左右方向尺寸, 用于网页绘制方块 (默认 0.45 m)
        self.declare_parameter('robot_width', 0.45)
        # 雷达->车体中心外参 x: 车体中心在雷达坐标系下的纵向坐标, >0 表示车体中心在雷达前方 
        self.declare_parameter('lidar_to_base_x', 0.239)
        # 雷达->车体中心外参 y: 车体中心在雷达坐标系下的横向坐标, >0 表示车体中心在雷达左方 
        self.declare_parameter('lidar_to_base_y', -0.158)
        # 前视距离 (m): 与 C++ 节点 motion_lookahead 保持一致, 用于复算前视点
        self.declare_parameter('motion_lookahead', 0.45)
        # 前视距离速度增益: 与 C++ 节点 motion_lookahead_speed_gain 一致。
        # L = motion_lookahead + gain * V, 0 表示关闭自适应 (固定前视距离)
        self.declare_parameter('motion_lookahead_speed_gain', 0.52)
        # 前视距离上限 (m): 与 C++ 节点 motion_lookahead_max 一致, <= 0 表示不限
        self.declare_parameter('motion_lookahead_max', 1.50)
        # 目标速度 (m/s): 与 C++ 节点 motion_target_speed 保持一致, 用于复算
        # 自适应前视距离时估算巡航速度
        self.declare_parameter('motion_target_speed', 1.00)
        # 到点阈值 (m): 与 C++ 节点 motion_goal_tolerance 保持一致, 用于到达判定
        self.declare_parameter('motion_goal_tolerance', 0.25)
        # 车体自过滤正方形边长 (m): 与 C++ 节点 self_filter_size 保持一致,
        # 用于在历史累计图上画出绿色自过滤框 (<=0 表示不画)
        self.declare_parameter('self_filter_size', 1.0)
        # 3D 重定位节点 (test_relocalization 的 relocalization_3d) 节点名:
        #   锁定历史累计图时同步它的 enabled (锁定=启用, 解锁=停用),
        #   并在锁定时自动把先验地图 (map_clouds.bin) 导出到 export_map_dir。
        self.declare_parameter('target_node_3d', '/relocalization_3d')
        # 注意 (勿再加参数): 车体中心位姿本文件统一由 _base_pose() 计算, 它只依赖
        # 上面的 lidar_to_base_x/y (雷达系表达)。切勿再引入一份 "base_to_lidar"
        # 外参用于本文件的几何复算 —— 那会与 lidar_to_base 各算一套车体中心,
        # 导致橙色坐标系原点与红色车身框中心错开 (历史上曾因此偏差约 0.57 m)。
        # C++ 节点另有一份 motion_base_to_lidar_{x,y} (车体系表达), 语义是
        # "车体中心 -> 雷达原点", 反推车体中心时做减法, 与本文件的
        # lidar_to_base (雷达系表达, 做加法) 结果一致 —— 曾因在 C++ 侧误做
        # 加法导致镜像偏差约 0.57 m (详见 C++ computeFollowCommand 步骤 0)。
        # 两种外参只需存在一份, 本文件一律用 lidar_to_base, 勿再重复声明。

        image_topic = self.get_parameter('image_topic').value
        self.target_node = self.get_parameter('target_node').value
        self.target_node_3d = str(
            self.get_parameter('target_node_3d').value)
        http_port = int(self.get_parameter('http_port').value)
        odom_topic = self.get_parameter('odom_topic').value
        self.correction_topic = \
            self.get_parameter('pose_correction_topic').value
        cloud_topic = self.get_parameter('cloud_topic').value
        history_topic = self.get_parameter('history_image_topic').value
        nav_action_name = str(self.get_parameter('nav_action_name').value)
        global_plan_topic = self.get_parameter('global_plan_topic').value
        local_plan_topic = self.get_parameter('local_plan_topic').value
        astar_plan_topic = self.get_parameter('astar_plan_topic').value
        goal_pose_topic = self.get_parameter('goal_pose_topic').value
        gcm_topic = self.get_parameter('global_costmap_topic').value
        lcm_topic = self.get_parameter('local_costmap_topic').value
        dynamic_image_topic = \
            self.get_parameter('dynamic_image_topic').value
        self.robot_length = float(self.get_parameter('robot_length').value)
        self.robot_width = float(self.get_parameter('robot_width').value)
        self.self_filter_size = float(
            self.get_parameter('self_filter_size').value)
        self.lidar_to_base_x = float(
            self.get_parameter('lidar_to_base_x').value)
        self.lidar_to_base_y = float(
            self.get_parameter('lidar_to_base_y').value)
        # 运动帧速度量化系数 (与 C++ 节点保持一致)
        self.motion_vx_scale = float(
            self.get_parameter('motion_vx_scale').value)
        self.motion_vy_scale = float(
            self.get_parameter('motion_vy_scale').value)
        # 路径跟随几何参数 (与 C++ 节点保持一致, 用于复算前视点叠加显示)
        self.motion_lookahead = float(
            self.get_parameter('motion_lookahead').value)
        self.motion_lookahead_speed_gain = float(
            self.get_parameter('motion_lookahead_speed_gain').value)
        self.motion_lookahead_max = float(
            self.get_parameter('motion_lookahead_max').value)
        self.motion_target_speed = float(
            self.get_parameter('motion_target_speed').value)
        self.motion_goal_tolerance = float(
            self.get_parameter('motion_goal_tolerance').value)

        # 滑条初值 = 节点默认参数（启动后会再从节点读一次覆盖）
        self.tbar = {
            'res': 5, 'zmin': 950, 'zmax': 1100, 'dil': 2,
            'hist': 300, 'hits': 2, 'cr': 60, 'op': 80,
            'lx': to_tbar_ext(self.lidar_to_base_x),
            'ly': to_tbar_ext(self.lidar_to_base_y)}
        self.map_meta = {'xmin': None, 'ymin': None, 'res': None}
        self.latest_image = None
        self.image_lock = threading.Lock()
        self.set_lock = threading.Lock()
        self.latest_odom = None
        self.odom_lock = threading.Lock()
        # map->odom 修正量 (SE(3)): 完整保存 6 自由度 —— 统一按 SE(3) 处理,
        # 只取 yaw 会让网页显示的位姿与地图错位。
        self.correction_trans = [0.0, 0.0, 0.0]      # x, y, z
        self.correction_quat = [0.0, 0.0, 0.0, 1.0]  # x, y, z, w
        self.correction_received = False
        self.correction_lock = threading.Lock()
        self.latest_cloud = None
        self.cloud_lock = threading.Lock()
        self.latest_history_image = None
        self.history_lock = threading.Lock()

        # ---------- Nav2 导航状态 ----------
        self.nav_action = nav_action_name
        self.nav_lock = threading.Lock()
        self.nav_status = 'idle'
        self.nav_goal = None     # 最近一次点击的目标点 {x, y} (map 系)
        self.global_path = None  # /plan 最新的全局路径 [{x, y}, ...]
        self.local_path = None   # /local_plan 最新的局部路径 [{x, y}, ...]
        self.astar_path = None   # /astar_plan 自建 A* 兜底路径 [{x, y}, ...]
        self.goal_handle = None  # NavigateToPose goal handle (用于取消)

        # ---------- 代价地图渲染缓存 (RGBA, 已重投影对齐底图网格) ----------
        self.costmap_lock = threading.Lock()
        self.gcm_rgba = None
        self.lcm_rgba = None

        # ---------- 动态障碍 (可变障碍) 缓存 ----------
        #   latest_dynamic_image: C++ 节点发布的 mono8 栅格图 (0=动态障碍)
        #   dyn_rgba: 已重投影对齐底图的绿色 RGBA, 供网页绘制
        self.dynamic_lock = threading.Lock()
        self.latest_dynamic_image = None
        self.dyn_rgba = None

        # ---------- 占点 (hold point) 状态 ----------
        #   由 C++ 节点 /hold_point_status 发布 (JSON), 含占点中心/范围/威胁信息
        #   hold_viz: /hold_point_viz 的路径点 [{x,y}, ...], 用于绘制占点圈/连线
        self.hold_lock = threading.Lock()
        self.hold_status = None
        self.hold_viz = None

        # ---------- 追击 (chase) 状态 ----------
        #   由 C++ 节点 /chase_status 发布 (JSON), 含目标位置等信息;
        #   chase_viz: /chase_viz 的路径点 [目标, 站位点]; chase_track: 目标轨迹点
        self.chase_lock = threading.Lock()
        self.chase_status = None
        self.chase_viz = None
        self.chase_track = None

        # ---------- 运动控制状态 (网页"开始/停止运动"按钮) ----------
        # 运动帧由 C++ 节点发布; 这里仅记录并展示最近的指令与节点反馈
        self.motion_lock = threading.Lock()
        # 是否正在执行指令 (请求期间网页不覆盖提示文案)
        self.motion_busy = False
        # C++ 节点上报的当前运动帧: control / vx / vy
        self.motion_fb = {'active': False, 'control': 0, 'vx': 0.0, 'vy': 0.0}
        # 运动帧最近一次到达时间: 用于判断跟随是否仍在进行中
        self.motion_last_t = 0.0
        # 前视点可视化快照 (仅在运动跟随中有效), 只读展示用
        self.lookahead = None


        viewer = self
        class Handler(BaseHTTPRequestHandler):
            def reply(self, code, body, ctype='text/html; charset=utf-8'):
                if isinstance(body, str):
                    body = body.encode('utf-8')
                self.send_response(code)
                self.send_header('Content-Type', ctype)
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Cache-Control', 'no-cache')
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                parsed = urlparse(self.path)
                q = parse_qs(parsed.query)
                if parsed.path in ('/', '/index.html'):
                    self.reply(200, PAGE)
                elif parsed.path == '/state':
                    with viewer.set_lock:
                        state = dict(viewer.tbar)
                    self.reply(200, json.dumps(state),
                               ctype='application/json')
                elif parsed.path == '/pose':
                    with viewer.odom_lock:
                        odom = dict(viewer.latest_odom) \
                            if viewer.latest_odom is not None else None
                    with viewer.set_lock:
                        meta = dict(viewer.map_meta)
                    with viewer.history_lock:
                        final_shape = None if viewer.latest_history_image is None \
                            else viewer.latest_history_image.shape
                    if odom is None:
                        self.reply(
                            200, json.dumps({'ok': False}),
                            ctype='application/json')
                        return
                    cos_yaw = math.cos(odom['yaw'])
                    sin_yaw = math.sin(odom['yaw'])
                    base_x = odom['x'] + (
                        cos_yaw * viewer.lidar_to_base_x -
                        sin_yaw * viewer.lidar_to_base_y)
                    base_y = odom['y'] + (
                        sin_yaw * viewer.lidar_to_base_x +
                        cos_yaw * viewer.lidar_to_base_y)
                    self.reply(200, json.dumps({
                        'ok': True,
                        'x': base_x,
                        'y': base_y,
                        'yaw': odom['yaw'],
                        'length': viewer.robot_length,
                        'width': viewer.robot_width,
                        # 车体自过滤正方形边长 (m): 供网页画绿色自过滤框
                        'self_filter_size': viewer.self_filter_size,
                        'age': round(time.monotonic() - odom['t'], 2),
                        # 是否已叠加 map->odom 修正量 (网页据此提示 SE3 生效)
                        'corrected': bool(odom.get('corrected', False)),
                        'xmin': meta['xmin'],
                        'ymin': meta['ymin'],
                        'res': meta['res'],
                        # height 供网页 w2c 换算 (车体自过滤框需要)
                        'height': None if final_shape is None
                                  else final_shape[0],
                        'img_width': None if final_shape is None else final_shape[1],
                        'img_height': None if final_shape is None else final_shape[0]}),
                        ctype='application/json')
                elif parsed.path == '/set':
                    try:
                        with viewer.set_lock:
                            node_param_changed = False
                            for key, tag in (('res', 'res'), ('zmin', 'zmin'),
                                            ('zmax', 'zmax'),
                                            ('dil', 'dil'),
                                            ('hist', 'hist'), ('hits', 'hits'),
                                            ('cr', 'cr'), ('op', 'op')):
                                if tag in q:
                                    viewer.tbar[key] = int(q[tag][0])
                                    node_param_changed = True
                            if 'lx' in q:
                                viewer.tbar['lx'] = int(q['lx'][0])
                            if 'ly' in q:
                                viewer.tbar['ly'] = int(q['ly'][0])
                            if 'lx' in q or 'ly' in q:
                                # 外参只属于本查看器(影响位姿显示与规划起点), 不下发到 C++ 节点
                                viewer.set_extrinsics(
                                    from_tbar_ext(viewer.tbar['lx']),
                                    from_tbar_ext(viewer.tbar['ly']))
                            ok = viewer.send_params() \
                                if node_param_changed else True
                            state = dict(viewer.tbar)
                        self.reply(200, json.dumps(
                            {'ok': bool(ok), 'msg': '' if ok else 'param service unavailable',
                             'res': state['res'], 'zmin': state['zmin'],
                             'zmax': state['zmax'],
                             'dil': state['dil'],
                             'hist': state['hist'], 'hits': state['hits'],
                             'cr': state['cr'], 'op': state['op'],
                             'lx': state['lx'], 'ly': state['ly']}),
                            ctype='application/json')
                    except ValueError as e:
                        self.reply(400, json.dumps({'ok': False, 'msg': str(e)}),
                                  ctype='application/json')
                elif parsed.path == '/cloud':
                    with viewer.cloud_lock:
                        arr = None if viewer.latest_cloud is None \
                            else viewer.latest_cloud.copy()
                    if arr is None or arr.shape[0] == 0:
                        self.reply(
                            200, json.dumps({'ok': False, 'n': 0}),
                            ctype='application/json')
                        return
                    payload = np.round(arr, 3).ravel().tolist()
                    self.reply(200, json.dumps({
                        'ok': True,
                        'n': int(arr.shape[0]),
                        'pts': payload}),
                        ctype='application/json')
                elif parsed.path == '/goal':
                    try:
                        col = int(q['c'][0])
                        row = int(q['r'][0])
                    except (KeyError, ValueError, IndexError):
                        self.reply(
                            400,
                            json.dumps({'ok': False, 'stage': 'error',
                                        'msg': '参数 c/r 无效'}),
                            ctype='application/json')
                        return
                    self.reply(
                        200, json.dumps(viewer.handle_goal(col, row)),
                        ctype='application/json')
                elif parsed.path == '/cancel':
                    self.reply(
                        200, json.dumps(viewer.handle_cancel()),
                        ctype='application/json')
                elif parsed.path == '/refresh_history':
                    ok = viewer.request_history_reset()
                    self.reply(200, json.dumps({'ok': bool(ok)}),
                               ctype='application/json')
                elif parsed.path == '/export_map':
                    ok, data = viewer.export_history_map()
                    if not ok:
                        self.reply(
                            500,
                            json.dumps({'ok': False, 'msg': data}),
                            ctype='application/json')
                        return
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/zip')
                    self.send_header(
                        'Content-Disposition',
                        'attachment; filename="map_export.zip"')
                    self.send_header('Content-Length', str(len(data)))
                    self.send_header('Cache-Control', 'no-cache')
                    self.end_headers()
                    self.wfile.write(data)
                elif parsed.path in ('/start', '/stop'):
                    r = viewer.request_motion(parsed.path == '/start')
                    self.reply(200, json.dumps(r),
                               ctype='application/json')
                elif parsed.path == '/motion':
                    self.reply(200, json.dumps(viewer.motion_state()),
                               ctype='application/json')
                elif parsed.path == '/lookahead':
                    self.reply(200, json.dumps(viewer.lookahead_state()),
                               ctype='application/json')
                elif parsed.path == '/nav':
                    with viewer.nav_lock:
                        status = viewer.nav_status
                        goal = None if viewer.nav_goal is None \
                            else dict(viewer.nav_goal)
                        plan = None if viewer.global_path is None \
                            else list(viewer.global_path)
                        local_plan = None if viewer.local_path is None \
                            else list(viewer.local_path)
                        astar_plan = None if viewer.astar_path is None \
                            else list(viewer.astar_path)
                    with viewer.set_lock:
                        meta = dict(viewer.map_meta)
                    with viewer.history_lock:
                        shape = None if viewer.latest_history_image is None \
                            else viewer.latest_history_image.shape
                    meta = dict(meta)
                    meta['height'] = None if shape is None else int(shape[0])
                    meta['width'] = None if shape is None else int(shape[1])
                    self.reply(
                        200,
                        json.dumps({'ok': True, 'status': status,
                                    'goal': goal, 'plan': plan,
                                    'local_plan': local_plan,
                                    'astar_plan': astar_plan,
                                    'meta': meta}),
                        ctype='application/json')
                elif parsed.path in ('/gcm.png', '/lcm.png'):
                    with viewer.costmap_lock:
                        rgba = viewer.gcm_rgba \
                            if parsed.path == '/gcm.png' else viewer.lcm_rgba
                    if rgba is None:
                        self.reply(404, b'no costmap yet',
                                   ctype='text/plain')
                        return
                    ok, encoded = cv2.imencode('.png', rgba)
                    if not ok:
                        self.reply(500, b'encode failed',
                                   ctype='text/plain')
                        return
                    self.reply(200, encoded.tobytes(), ctype='image/png')
                elif parsed.path == '/dyn.png':
                    viewer.render_dynamic_obstacle()
                    with viewer.dynamic_lock:
                        rgba = viewer.dyn_rgba
                    if rgba is None:
                        self.reply(404, b'no dynamic obstacle yet',
                                   ctype='text/plain')
                        return
                    ok, encoded = cv2.imencode('.png', rgba)
                    if not ok:
                        self.reply(500, b'encode failed',
                                   ctype='text/plain')
                        return
                    self.reply(200, encoded.tobytes(), ctype='image/png')
                elif parsed.path == '/lock':
                    val = q.get('on', ['1'])[0]
                    enabled = str(val).lower() not in ('0', 'false', 'off')
                    self.reply(200, json.dumps(
                        viewer.request_dynamic_lock(enabled)),
                        ctype='application/json')
                elif parsed.path == '/hold':
                    # 占点 (hold point) 行为:
                    #   on=1/0         开启/关闭占点模式
                    #   c=&r=          点击像素 -> 占点中心 (map 系)
                    #   radius         占点范围 (m, 可选, 网页滑条动态调整)
                    #   standoff       站位间距系数 (可选, 越接近 1 越贴近敌人)
                    try:
                        col = int(q['c'][0]) if 'c' in q else None
                        row = int(q['r'][0]) if 'r' in q else None
                    except (ValueError, IndexError):
                        self.reply(400, json.dumps(
                            {'ok': False, 'msg': '参数 c/r 无效'}),
                            ctype='application/json')
                        return
                    val = q.get('on', ['1'])[0]
                    enabled = str(val).lower() not in ('0', 'false', 'off')
                    radius = None
                    if 'radius' in q:
                        try:
                            radius = float(q['radius'][0])
                        except (ValueError, IndexError):
                            radius = None
                    standoff = None
                    if 'standoff' in q:
                        try:
                            standoff = float(q['standoff'][0])
                        except (ValueError, IndexError):
                            standoff = None
                    self.reply(200, json.dumps(
                        viewer.request_hold_point(
                            enabled, col, row, radius, standoff)),
                        ctype='application/json')
                elif parsed.path == '/hold_state':
                    self.reply(200, json.dumps(viewer.hold_point_state()),
                               ctype='application/json')
                elif parsed.path == '/chase':
                    # 追击 (chase) 行为:
                    #   on=1/0         开启/关闭追击模式
                    #   c=&r=          点击像素 -> 追击目标点 (map 系)
                    #   radius         环带半径 (m, 可选, 网页滑条动态调整)
                    try:
                        col = int(q['c'][0]) if 'c' in q else None
                        row = int(q['r'][0]) if 'r' in q else None
                    except (ValueError, IndexError):
                        self.reply(400, json.dumps(
                            {'ok': False, 'msg': '参数 c/r 无效'}),
                            ctype='application/json')
                        return
                    val = q.get('on', ['1'])[0]
                    enabled = str(val).lower() not in ('0', 'false', 'off')
                    radius = None
                    if 'radius' in q:
                        try:
                            radius = float(q['radius'][0])
                        except (ValueError, IndexError):
                            radius = None
                    self.reply(200, json.dumps(
                        viewer.request_chase(enabled, col, row, radius)),
                        ctype='application/json')
                elif parsed.path == '/chase_state':
                    self.reply(200, json.dumps(viewer.chase_state()),
                               ctype='application/json')
                elif parsed.path == '/image_history.png':
                    with viewer.history_lock:
                        image = None if viewer.latest_history_image is None \
                            else viewer.latest_history_image.copy()
                    if image is None:
                        self.reply(404, b'no image yet', ctype='text/plain')
                        return
                    ok, encoded = cv2.imencode('.png', image)
                    if not ok:
                        self.reply(500, b'encode failed', ctype='text/plain')
                        return
                    self.reply(200, encoded.tobytes(), ctype='image/png')
                elif parsed.path == '/image.png':
                    with viewer.image_lock:
                        image = None if viewer.latest_image is None \
                            else viewer.latest_image.copy()
                    if image is None:
                        self.reply(404, b'no image yet', ctype='text/plain')
                        return
                    ok, encoded = cv2.imencode('.png', image)
                    if not ok:
                        self.reply(500, b'encode failed', ctype='text/plain')
                        return
                    self.reply(200, encoded.tobytes(), ctype='image/png')
                else:
                    self.reply(404, b'not found', ctype='text/plain')

            def do_POST(self):
                parsed = urlparse(self.path)
                if parsed.path != '/import_map':
                    self.reply(404, b'not found', ctype='text/plain')
                    return
                try:
                    length = int(self.headers.get('Content-Length', '0'))
                except ValueError:
                    length = 0
                if length <= 0:
                    self.reply(
                        400, json.dumps({'ok': False, 'msg': '空请求体'}),
                        ctype='application/json')
                    return
                if length > 200 * 1024 * 1024:
                    self.reply(
                        413, json.dumps({'ok': False, 'msg': '文件过大'}),
                        ctype='application/json')
                    return
                body = self.rfile.read(length)
                ok, msg = viewer.import_history_map(body)
                self.reply(200, json.dumps({'ok': bool(ok), 'msg': msg}),
                           ctype='application/json')

            def log_message(self, fmt, *args):
                pass

        self.http_server = ThreadingHTTPServer(('0.0.0.0', http_port), Handler)
        threading.Thread(target=self.http_server.serve_forever, daemon=True).start()
        self.get_cli = self.create_client(
            GetParameters, self.target_node + '/get_parameters')
        self.set_cli = self.create_client(
            SetParameters, self.target_node + '/set_parameters')
        # 3D 重定位节点参数服务 (锁定/解锁同步 enabled)
        self.set_cli_3d = self.create_client(
            SetParameters, self.target_node_3d + '/set_parameters')
        # 周期同步锁定状态 (外部用 ros2 param set 改锁定时, 网页显示也要跟上:
        #   位姿修正量仅在"已锁定"时叠加, 与 C++ 节点的修正施加口径一致)
        self.hist_locked = False
        self._lock_sync_pending = False
        self.create_timer(1.0, self._sync_lock_state)

        # Nav2 NavigateToPose 动作客户端: 网页点击发送/取消导航目标
        if NavigateToPose is not None:
            self.nav_client = ActionClient(
                self, NavigateToPose, nav_action_name)
        else:
            self.nav_client = None

        self.sub = self.create_subscription(
            Image, image_topic, self.on_image, qos_profile_sensor_data)
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.on_odom, qos_profile_sensor_data)
        self.correction_sub = self.create_subscription(
            PoseStamped, self.correction_topic, self.on_pose_correction, 10)
        self.cloud_sub = self.create_subscription(
            PointCloud2, cloud_topic, self.on_cloud,
            qos_profile_sensor_data)
        self.history_sub = self.create_subscription(
            Image, history_topic, self.on_history_image,
            qos_profile_sensor_data)

        # Nav2 全局/局部路径与代价地图订阅
        # (volatile + best_effort 订阅对 volatile/reliable、transient_local
        #  发布端都兼容, 避免与 Nav2 costmap 发布 QoS 不匹配收不到数据)
        costmap_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE)
        self.global_plan_sub = self.create_subscription(
            Path, global_plan_topic, self.on_global_plan, 10)
        self.local_plan_sub = self.create_subscription(
            Path, local_plan_topic, self.on_local_plan, 10)
        # C++ 节点自建 A* 兜底路径 (黄色显示) 与 Web 目标点转发发布
        self.astar_plan_sub = self.create_subscription(
            Path, astar_plan_topic, self.on_astar_plan, 10)
        self.goal_pose_pub = self.create_publisher(
            PoseStamped, goal_pose_topic, 10)
        self.gcm_sub = self.create_subscription(
            OccupancyGrid, gcm_topic, self.on_global_costmap, costmap_qos)
        self.lcm_sub = self.create_subscription(
            OccupancyGrid, lcm_topic, self.on_local_costmap, costmap_qos)
        # 动态障碍栅格图 (锁定历史累计图后才有数据)
        self.dynamic_image_sub = self.create_subscription(
            Image, dynamic_image_topic, self.on_dynamic_image,
            qos_profile_sensor_data)

        # 运动帧下发话题 (C++ 节点发布, 这里仅回显状态给网页)
        self.motion_cmd_topic = str(
            self.get_parameter('motion_cmd_topic').value)
        self.motion_sub = self.create_subscription(
            UInt8MultiArray, self.motion_cmd_topic, self.on_motion_cmd, 10)

        # 占点状态 (JSON) 与占点可视化路径 (中心 + 阻挡站位点)
        self.hold_status_topic = str(
            self.get_parameter('hold_point_status_topic').value)
        self.hold_viz_topic = str(
            self.get_parameter('hold_point_viz_topic').value)
        self.hold_status_sub = self.create_subscription(
            String, self.hold_status_topic, self.on_hold_status, 10)
        self.hold_viz_sub = self.create_subscription(
            Path, self.hold_viz_topic, self.on_hold_viz, 10)

        # 追击状态 (JSON) / 目标可视化 / 目标历史轨迹
        self.chase_status_topic = str(
            self.get_parameter('chase_status_topic').value)
        self.chase_viz_topic = str(
            self.get_parameter('chase_viz_topic').value)
        self.chase_track_topic = str(
            self.get_parameter('chase_track_topic').value)
        self.chase_status_sub = self.create_subscription(
            String, self.chase_status_topic, self.on_chase_status, 10)
        self.chase_viz_sub = self.create_subscription(
            Path, self.chase_viz_topic, self.on_chase_viz, 10)
        self.chase_track_sub = self.create_subscription(
            Path, self.chase_track_topic, self.on_chase_track, 10)

        self.get_logger().info(
            '网页查看器启动: http://<本机IP>:%d  '
            '(image=%s, odom=%s, cloud=%s, history=%s, 目标节点=%s, '
            'nav2=%s, plan=%s/%s, costmap=%s/%s, motion=%s, '
            'body=%.2fx%.2f m, lidar_to_base=(%.2f, %.2f) m)'
            % (http_port, image_topic, odom_topic, cloud_topic,
               history_topic, self.target_node, nav_action_name,
               global_plan_topic, local_plan_topic, gcm_topic, lcm_topic,
               self.motion_cmd_topic,
               self.robot_length, self.robot_width,
               self.lidar_to_base_x, self.lidar_to_base_y))

    # ---------- 图像订阅 ----------
    def on_image(self, msg):
        if msg.encoding != 'mono8':
            self.get_logger().warning(
                '不支持的图像编码: %s' % msg.encoding, throttle_duration_sec=5.0)
            return
        img = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.width * msg.height != img.size:
            self.get_logger().warning('图像尺寸与数据长度不符，丢弃本帧')
            return
        with self.image_lock:
            self.latest_image = img.reshape(msg.height, msg.width).copy()

    def on_history_image(self, msg):
        if msg.encoding != 'mono8':
            self.get_logger().warning(
                '不支持的图像编码: %s' % msg.encoding, throttle_duration_sec=5.0)
            return
        img = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.width * msg.height != img.size:
            self.get_logger().warning('图像尺寸与数据长度不符，丢弃本帧')
            return
        with self.history_lock:
            self.latest_history_image = \
                img.reshape(msg.height, msg.width).copy()

    # ---------- 动态障碍栅格图 (锁定历史累计图后由 C++ 节点发布) ----------
    def on_dynamic_image(self, msg):
        if msg.encoding != 'mono8':
            return
        img = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.width * msg.height != img.size:
            return
        with self.dynamic_lock:
            self.latest_dynamic_image = \
                img.reshape(msg.height, msg.width).copy()

    # ---------- 位姿修正量订阅 (map->odom, SE(3)) ----------
    def on_pose_correction(self, msg):
        p = msg.pose.position
        q = msg.pose.orientation
        with self.correction_lock:
            self.correction_trans = [float(p.x), float(p.y), float(p.z)]
            self.correction_quat = [
                float(q.x), float(q.y), float(q.z), float(q.w)]
            self.correction_received = True

    # ---------- 里程计订阅（角色在 map 系的实时位姿） ----------
    def on_odom(self, msg):
        pos = msg.pose.pose.position
        q = msg.pose.pose.orientation
        odom_quat = (float(q.x), float(q.y), float(q.z), float(q.w))
        yaw = quat_to_yaw(odom_quat)
        x = float(pos.x)
        y = float(pos.y)

        # 叠加 map->odom 修正量: latest_odom 对外一律是 **map 系** 位姿。
        #   底图/路径/代价地图/目标点全在 map 系; 若这里仍给原始 odom 位姿,
        #   一旦修正量非零, 网页上的车体标记、自过滤框、前视点都会与地图错位。
        #   仅在"已锁定历史累计图"时叠加 (与节点只锁定时施加修正的口径一致):
        #   解锁后外部 3D 节点停发 /pose_correction, 若仍沿用最后一次的修正量,
        #   箭头会停在旧修正量处, 与已回到原始坐标的点云层错开。
        with self.correction_lock:
            corr_ok = self.correction_received and self.hist_locked
            corr_t = list(self.correction_trans)
            corr_q = tuple(self.correction_quat)

        if corr_ok:
            rx, ry, _ = quat_rotate(corr_q, (x, y, float(pos.z)))
            x = rx + corr_t[0]
            y = ry + corr_t[1]
            yaw = quat_to_yaw(quat_mul(corr_q, odom_quat))

        with self.odom_lock:
            self.latest_odom = {
                'x': float(x),
                'y': float(y),
                'yaw': float(yaw),
                't': time.monotonic(),
                'corrected': bool(corr_ok),
            }

    # ---------- 运动帧订阅 (对应接收方 CAN_ID=0xFF 报文 Data[0..7]) ----------
    def on_motion_cmd(self, msg):
        """解析 C++ 节点下发的运动帧, 回显 control / vx / vy 给网页。
        速度按接收方同款 12 位有符号补码还原: raw / scale (scale 默认 200)。"""
        data = list(msg.data)
        if len(data) < 8:
            return

        vx_scale = self.motion_vx_scale if self.motion_vx_scale > 0 else 200.0
        vy_scale = self.motion_vy_scale if self.motion_vy_scale > 0 else 200.0

        def to_signed12(raw):
            raw &= 0x0FFF
            return raw - 0x1000 if raw & 0x0800 else raw

        def vel_from_raw(raw, scale):
            return round(to_signed12(raw) / scale, 3)

        # [5] vx 高 8 位 | [6] 高 4 位 vx 低 4 位 | [6] 低 4 位 vy 高 4 位 | [7] vy 低 8 位
        vx_raw = (int(data[5]) << 4) | ((int(data[6]) >> 4) & 0x0F)
        vy_raw = ((int(data[6]) & 0x0F) << 8) | int(data[7])

        with self.motion_lock:
            self.motion_fb = {
                'active': bool(data[0] & 0x01),
                'control': int(data[0]) & 0x01,
                'vx': vel_from_raw(vx_raw, vx_scale),
                'vy': vel_from_raw(vy_raw, vy_scale)}
            self.motion_last_t = time.monotonic()
            # 前视点/速度矢量叠加层素材 (与 C++ computeFollowCommand 同款几何)
            self.lookahead = self.compute_follow_geometry()

    # ---------- 前视点可视化: 复算 C++ computeFollowCommand 的几何量 ----------
    def compute_follow_geometry(self):
        """按与 C++ 节点 computeFollowCommand 完全一致的步骤复算:
        车体中心 -> 前方半平面内最近点 -> 沿弧长推进 lookahead 得到前视点。
        返回 map 系的 dict (供网页叠加绘制) 或 None (数据不足/已到达)。
        仅用于显示: 真正下发的 vx/vy 以 C++ 节点运动帧为准。"""
        with self.nav_lock:
            plan = None if self.global_path is None else list(self.global_path)
        with self.odom_lock:
            odom = dict(self.latest_odom) \
                if self.latest_odom is not None else None
        if not plan or len(plan) < 2 or odom is None:
            return None

        yaw = odom['yaw']
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        # 车体中心位姿: 必须复用 _base_pose() —— 它与 /pose 分支、红色车身框
        # 用的是同一份外参 lidar_to_base_x/y, 因此橙色坐标系原点与红框中心
        # 严格重合。切勿在这里另起一份 "base_to_lidar" 外参自行推算, 否则
        # 两套车体中心会错开 (见上方参数声明处的说明)。
        base_pose = self._base_pose()
        if base_pose is None:
            return None
        base_x, base_y = base_pose[0], base_pose[1]
        # base_pose[2] 即 odom['yaw'], 与上面 yaw 取自同一份里程计, 无需重取

        # 到达判定: 与 C++ 一致, 到达后不再显示前视点
        if math.hypot(plan[-1]['x'] - base_x, plan[-1]['y'] - base_y) <= \
                self.motion_goal_tolerance:
            return None

        # 最近点: 优先取车头前方半平面 (forward >= -5 cm) 内的最近点
        front_tol = 0.05
        best_dist = None
        nearest = 0
        for i, p in enumerate(plan):
            dx = p['x'] - base_x
            dy = p['y'] - base_y
            forward = dx * cos_yaw + dy * sin_yaw
            if forward < -front_tol:
                continue
            d = math.hypot(dx, dy)
            if best_dist is None or d < best_dist:
                best_dist = d
                nearest = i
        if best_dist is None:   # 前方无候选: 退回全局最近点
            for i, p in enumerate(plan):
                d = math.hypot(p['x'] - base_x, p['y'] - base_y)
                if best_dist is None or d < best_dist:
                    best_dist = d
                    nearest = i

        # 弧长推进得到前视点 (与 C++ 步骤 4c 相同)
        cumulative = [0.0] * len(plan)
        for i in range(1, len(plan)):
            cumulative[i] = cumulative[i - 1] + math.hypot(
                plan[i]['x'] - plan[i - 1]['x'],
                plan[i]['y'] - plan[i - 1]['y'])

        # 自适应前视距离 (与 C++ 步骤 4b 相同): L = L_min + gain * V, 再限上限。
        # V 取"实际下发速度"的近似 —— 用巡航速度按剩余距离线性减速得到 speed,
        # 与 C++ computeFollowCommand 步骤 4 的减速逻辑一致。
        dist_end_m = math.hypot(plan[-1]['x'] - base_x, plan[-1]['y'] - base_y)
        speed_cmd = min(max(0.0, self.motion_target_speed), 5.0)
        # 减速起始距离与 C++ effectiveSlowRadius 完全一致:
        #   slow_radius = max(goal_tol + 0.05, kSlowRadiusTime * speed),
        #   kSlowRadiusTime = 1.0 (见 C++ 同名常量)。速度越大越早开始减速。
        slow_radius = max(self.motion_goal_tolerance + 0.05, 1.0 * speed_cmd)
        speed = speed_cmd
        if slow_radius > self.motion_goal_tolerance and \
                dist_end_m < slow_radius:
            ratio = (dist_end_m - self.motion_goal_tolerance) / \
                (slow_radius - self.motion_goal_tolerance)
            speed = max(0.08, speed_cmd * ratio)

        lookahead = max(0.05, self.motion_lookahead)
        if self.motion_lookahead_speed_gain > 0.0:
            lookahead += self.motion_lookahead_speed_gain * \
                min(max(speed, 0.0), 5.0)
        if self.motion_lookahead_max > 0.0:
            lookahead = min(lookahead, self.motion_lookahead_max)

        lookahead_s = cumulative[nearest] + lookahead
        lookahead_index = len(plan) - 1
        for i in range(nearest, len(plan)):
            if cumulative[i] >= lookahead_s:
                lookahead_index = i
                break

        target_x = plan[lookahead_index]['x']
        target_y = plan[lookahead_index]['y']
        # 剩余距离 (供网页显示):
        #   dist_end_m  = 车体中心 -> 路径末端的直线距离, 与 C++
        #                 computeFollowCommand 步骤 1 的 dist_end 完全一致
        #                 (到达判定与减速段用的就是它)
        #   remaining_m = 沿路径的剩余弧长 (最近点 -> 路径末端), 反映"还要走多远"
        remaining_m = cumulative[-1] - cumulative[nearest]
        return {
            'base': {'x': round(base_x, 3), 'y': round(base_y, 3)},
            'target': {'x': round(target_x, 3), 'y': round(target_y, 3)},
            'yaw': round(yaw, 4),
            'lookahead_m': round(lookahead, 3),
            'nearest_index': int(nearest),
            'lookahead_index': int(lookahead_index),
            'plan_points': len(plan),
            'dist_end_m': round(dist_end_m, 3),
            'remaining_m': round(max(0.0, remaining_m), 3)}

    # ---------- 点云预览订阅（供网页右侧侧视图渲染） ----------
    def on_cloud(self, msg):
        pts = None
        try:
            structured = pc2.read_points(
                msg, field_names=('x', 'y', 'z'), skip_nans=True)
            pts = np.stack(
                [structured['x'], structured['y'], structured['z']], axis=1)
        except Exception:
            try:
                pts = self._parse_cloud_manual(msg)
            except Exception:
                pts = None
        if pts is None or pts.size == 0:
            self.get_logger().warning(
                '点云解析失败, 丢弃本帧', throttle_duration_sec=5.0)
            return
        with self.cloud_lock:
            self.latest_cloud = pts.astype(np.float32)

    @staticmethod
    def _parse_cloud_manual(msg):
        """不依赖 sensor_msgs_py 的兜底解析 (x/y/z 必须为 float32)."""
        offsets = {}
        for field in msg.fields:
            if field.name in ('x', 'y', 'z') and field.datatype == 7:
                offsets[field.name] = field.offset
        if len(offsets) != 3:
            raise ValueError('缺少 float32 x/y/z 字段')
        dtype = np.dtype({
            'names': ['x', 'y', 'z'],
            'formats': ['<f4', '<f4', '<f4'],
            'offsets': [offsets['x'], offsets['y'], offsets['z']],
            'itemsize': max(int(msg.point_step), 12)})
        arr = np.frombuffer(bytes(msg.data), dtype=dtype)
        mask = np.isfinite(arr['x']) & np.isfinite(arr['y']) \
            & np.isfinite(arr['z'])
        arr = arr[mask]
        return np.stack([arr['x'], arr['y'], arr['z']], axis=1)

    # ==================== 路径规划 (网页点击目标点 -> 车身到目标点 A*) ====================
    @staticmethod
    def _pixel_to_world(col, row, height, meta):
        """图像像素 -> map 系世界坐标 (PGM 第 0 行在顶部, y 轴已翻转)"""
        x = meta['xmin'] + (col + 0.5) * meta['res']
        y = meta['ymin'] + (height - 1 - row + 0.5) * meta['res']
        return x, y

    # ==================== Nav2 导航 (网页点击目标点 -> NavigateToPose) ====================
    @staticmethod
    def _pixel_to_world(col, row, height, meta):
        """图像像素 -> map 系世界坐标 (PGM 第 0 行在顶部, y 轴已翻转)"""
        x = meta['xmin'] + (col + 0.5) * meta['res']
        y = meta['ymin'] + (height - 1 - row + 0.5) * meta['res']
        return x, y

    def _base_pose(self):
        """当前车体中心位姿: /Odometry(雷达原点) + lidar_to_base 外参
        返回 (x, y, yaw, t) 或 None"""
        with self.odom_lock:
            odom = dict(self.latest_odom) \
                if self.latest_odom is not None else None
        if odom is None:
            return None
        cos_yaw = math.cos(odom['yaw'])
        sin_yaw = math.sin(odom['yaw'])
        base_x = odom['x'] + (
            cos_yaw * self.lidar_to_base_x -
            sin_yaw * self.lidar_to_base_y)
        base_y = odom['y'] + (
            sin_yaw * self.lidar_to_base_x +
            cos_yaw * self.lidar_to_base_y)
        return base_x, base_y, odom['yaw'], odom['t']

    def handle_goal(self, col, row):
        """网页单击入口: 点击像素 -> map 系目标点, 递交给 Nav2 NavigateToPose
        (Nav2 负责全局规划与局部避障, 默认行为树会周期重规划)"""
        if self.nav_client is None or NavigateToPose is None:
            return {'ok': False, 'stage': 'error',
                    'msg': 'nav2_msgs 不可用, 请先安装 ros-humble-nav2-msgs'}
        action_type = NavigateToPose
        with self.history_lock:
            image = None if self.latest_history_image is None \
                else self.latest_history_image.copy()
        with self.set_lock:
            meta = dict(self.map_meta)
        base = self._base_pose()

        if image is None:
            return {'ok': False, 'stage': 'error', 'msg': '暂无地图图像'}
        if base is None:
            return {'ok': False, 'stage': 'error',
                    'msg': '暂无车身位姿, 请等待 /Odometry 数据'}
        if meta['res'] is None or meta['xmin'] is None or \
                meta['ymin'] is None:
            return {'ok': False, 'stage': 'error',
                    'msg': '地图参数未同步, 请稍后重试'}

        height, width = image.shape
        if col < 0 or col >= width or row < 0 or row >= height:
            return {'ok': False, 'stage': 'error', 'msg': '点击位置超出地图'}

        x, y = self._pixel_to_world(col, row, height, meta)
        # 目标朝向: 从当前车身位置指向目标点 (Nav2 规划器主要使用位置)
        goal_yaw = math.atan2(y - base[1], x - base[0])

        # 目标点额外转发给 C++ 节点 (供其自建 A* 兜底规划器作为终点):
        #   Nav2 的 NavigateToPose 动作不暴露话题, C++ 节点无法直接订阅, 故这里
        #   把同一目标点以 PoseStamped 形式发布到 goal_pose_topic。
        gp = PoseStamped()
        gp.header.frame_id = 'map'
        gp.header.stamp = self.get_clock().now().to_msg()
        gp.pose.position.x = float(x)
        gp.pose.position.y = float(y)
        gp.pose.orientation.z = math.sin(0.5 * goal_yaw)
        gp.pose.orientation.w = math.cos(0.5 * goal_yaw)
        if self.goal_pose_pub is not None:
            self.goal_pose_pub.publish(gp)

        goal = action_type.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(0.5 * goal_yaw)
        goal.pose.pose.orientation.w = math.cos(0.5 * goal_yaw)

        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            return {'ok': False, 'stage': 'error',
                    'msg': 'Nav2 动作服务 %s 不可用 '
                           '(检查 Nav2 是否启动并激活)' % self.nav_action}

        # 必须挂上回调: send_goal_async 返回的 future 若被丢弃, on_goal_response
        # 永远不会被调用 -> 目标被拒/中止的原因永远不写进 nav_status, 网页会
        # 一直停在"已发送目标, 等待 Nav2 接受", 看起来就像"点了没反应、没有路径"。
        self.nav_client.send_goal_async(goal).add_done_callback(
            self.on_goal_response)
        with self.nav_lock:
            self.nav_status = '已发送目标, 等待 Nav2 接受'
            self.nav_goal = {'x': round(x, 3), 'y': round(y, 3)}
            self.goal_handle = None
            # 立刻清掉上一次的路径残留: 否则网页在"等待 Nav2 接受"期间仍画着
            # 旧绿线, 视觉上会被误判成"路径没更新 / 卡住了"
            self.global_path = None
            self.local_path = None
        self.get_logger().info(
            '导航目标已发送: (%.2f, %.2f) m' % (x, y))
        return {'ok': True, 'stage': 'goal_sent',
                'x': round(x, 3), 'y': round(y, 3)}

    def on_goal_response(self, future):
        """send_goal_async 完成: 记录 goal handle 并挂接结果回调"""
        try:
            handle = future.result()
        except Exception as e:  # noqa: B902
            with self.nav_lock:
                self.nav_status = '目标发送失败: %s' % e
            return
        if handle is None or not handle.accepted:
            # 目标被拒最常见的原因: bt_navigator 行为树加载失败 (例如
            # default_nav_to_pose_bt_xml 指向的文件不存在), 此时动作服务器
            # 会直接 reject 而不进入规划, 现象就是"点击后没有路径"。
            with self.nav_lock:
                self.nav_status = (
                    'Nav2 拒绝了目标 (常见原因: bt_navigator 行为树加载失败, '
                    '请检查 default_nav_to_pose_bt_xml 路径是否存在)')
            return
        with self.nav_lock:
            self.goal_handle = handle
            self.nav_status = '导航中'
        handle.get_result_async().add_done_callback(self.on_nav_result)

    def on_nav_result(self, future):
        try:
            result = future.result()
        except Exception as e:  # noqa: B902
            with self.nav_lock:
                self.nav_status = '导航异常: %s' % e
            return
        # GoalStatus: 4 成功 / 5 取消 / 6 中止
        status_text = {4: '已到达目标', 5: '导航已取消',
                       6: '导航失败(中止)'}.get(int(result.status))
        if status_text is None:
            status_text = '导航结束(状态 %s)' % result.status
        if int(result.status) == 6:
            # 中止多半是 planner_server 报 "Failed to create a plan":
            # 起点/终点落在致命障碍格, 或目标超出代价地图范围。
            # 附上已收到的全局路径点数, 便于区分"规划失败"与"路径没画出来"。
            with self.nav_lock:
                n_plan = 0 if self.global_path is None else len(self.global_path)
            if n_plan == 0:
                status_text += (
                    ' —— 未收到任何全局路径; 多为起点/终点被障碍封死或目标'
                    '越界, 请查看 planner_server 终端日志')
            else:
                status_text += ' (已收到 %d 个路径点)' % n_plan
        with self.nav_lock:
            self.nav_status = status_text
        self.get_logger().info('Nav2: ' + status_text)

    def handle_cancel(self):
        """取消当前导航目标 (网页右键 / 取消导航按钮)"""
        with self.nav_lock:
            handle = self.goal_handle
        if handle is None:
            return {'ok': False, 'stage': 'error',
                    'msg': '当前没有进行中的导航任务'}
        try:
            handle.cancel_goal_async()
        except Exception as e:  # noqa: B902
            return {'ok': False, 'stage': 'error', 'msg': '取消失败: %s' % e}
        with self.nav_lock:
            self.nav_status = '正在取消 ...'
        return {'ok': True, 'stage': 'canceling'}

    # ---------- Nav2 路径订阅 ----------
    @staticmethod
    def _path_to_points(msg):
        return [{'x': round(p.pose.position.x, 3),
                 'y': round(p.pose.position.y, 3)} for p in msg.poses]

    def on_global_plan(self, msg):
        with self.nav_lock:
            self.global_path = self._path_to_points(msg)

    def on_local_plan(self, msg):
        with self.nav_lock:
            self.local_path = self._path_to_points(msg)

    def on_astar_plan(self, msg):
        # 自建 A* 兜底路径 (/plan 不可用时 C++ 节点规划), 网页用黄色显示。
        # 空路径 (点数 < 2) 视为"无兜底路径": 置 None, 网页不再绘制。
        pts = self._path_to_points(msg)
        with self.nav_lock:
            self.astar_path = pts if len(pts) >= 2 else None

    # ---------- Nav2 代价地图订阅 -> 重投影到底图网格后缓存 ----------
    def on_global_costmap(self, msg):
        self.render_costmap(msg, is_global=True)

    def on_local_costmap(self, msg):
        self.render_costmap(msg, is_global=False)

    def render_costmap(self, msg, is_global):
        with self.history_lock:
            shape = None if self.latest_history_image is None \
                else self.latest_history_image.shape
        with self.set_lock:
            meta = dict(self.map_meta)
        if shape is None or meta['res'] is None or \
                meta['xmin'] is None or meta['ymin'] is None:
            return
        rgba = costmap_to_rgba(msg, meta, shape[1], shape[0])
        if rgba is None:
            return
        with self.costmap_lock:
            if is_global:
                self.gcm_rgba = rgba
            else:
                self.lcm_rgba = rgba

    # ---------- 动态障碍栅格图 -> 绿色 RGBA (供网页 /dyn.png) ----------
    def render_dynamic_obstacle(self):
        with self.dynamic_lock:
            img = None if self.latest_dynamic_image is None \
                else self.latest_dynamic_image
        with self.set_lock:
            meta = dict(self.map_meta)
        if img is None or meta['res'] is None or \
                meta['xmin'] is None or meta['ymin'] is None:
            return
        with self.history_lock:
            shape = None if self.latest_history_image is None \
                else self.latest_history_image.shape
        if shape is None:
            return
        rgba = dynamic_obstacle_to_rgba(img, meta, shape[1], shape[0])
        if rgba is None:
            return
        with self.dynamic_lock:
            self.dyn_rgba = rgba

    # ---------- 锁定状态同步 (executor 线程定时, 不得阻塞) ----------
    def _sync_lock_state(self):
        """周期读取 C++ 节点 dynamic_obstacle_enabled, 同步 self.hist_locked。
        (锁定状态决定网页是否把 /pose_correction 叠加到显示位姿上 —— 必须与
        节点"仅锁定时施加修正"的口径一致, 否则解锁后箭头会停在旧修正量处。)"""
        if self._lock_sync_pending or not self.get_cli.service_is_ready():
            return
        req = GetParameters.Request()
        req.names = ['dynamic_obstacle_enabled']
        self._lock_sync_pending = True
        self.get_cli.call_async(req).add_done_callback(self._on_lock_state)

    def _on_lock_state(self, future):
        self._lock_sync_pending = False
        try:
            resp = future.result()
            if resp.values:
                self.hist_locked = bool(resp.values[0].bool_value)
        except Exception:  # noqa: B902
            pass

    # ---------- 3D 重定位节点参数 (锁定/解锁同步 enabled) ----------
    def _get_bool_param(self, name, timeout_sec=1.0):
        """读主节点布尔参数 (HTTP 线程调用, 轮询等待); 读不到返回 None。"""
        if not self.get_cli.service_is_ready():
            return None
        req = GetParameters.Request()
        req.names = [name]
        future = self.get_cli.call_async(req)
        begin = time.time()
        while not future.done() and time.time() - begin < timeout_sec:
            time.sleep(0.02)
        if not future.done() or future.result() is None:
            return None
        vals = future.result().values
        return bool(vals[0].bool_value) if vals else None

    def _set_bool_param_3d(self, name, value, timeout_sec=1.0):
        """热改 3D 重定位节点参数; 节点未启动时返回 (False, 提示) 不算失败。
        由 HTTP 线程调用 (轮询等待, 不在 executor 线程阻塞)。"""
        if not self.set_cli_3d.service_is_ready():
            return False, '3D 重定位节点未启动'
        req = SetParameters.Request()
        req.parameters = [Parameter(
            name, Parameter.Type.BOOL, bool(value)).to_parameter_msg()]
        future = self.set_cli_3d.call_async(req)
        begin = time.time()
        while not future.done() and time.time() - begin < timeout_sec:
            time.sleep(0.02)
        if not future.done() or future.result() is None:
            return False, '参数设置超时'
        ok = all(r.successful for r in future.result().results)
        return ok, ('ok' if ok else '节点拒绝参数')

    # ---------- 锁定/解除历史累计图 (动态障碍模式开关) ----------
    def request_dynamic_lock(self, enabled):
        """锁定/解除历史累计图, 三步联动:
          1) 写 C++ 节点 dynamic_obstacle_enabled: 锁定后历史图冻结, 新出现的
             障碍被判为动态障碍(绿色)且不写入历史地图;
          2) 锁定时自动把历史点云队列导出为先验地图 (map_clouds.bin, 覆盖式,
             不下载): 3D 重定位节点按 mtime 轮询自动重载 —— 重新锁定 = 重新
             导出 = 重定位重新开始;
          3) 同步 3D 重定位节点 enabled (锁定=启用, 解锁=停用)。
        """
        if not self.set_cli.wait_for_service(timeout_sec=0.5):
            return {'ok': False, 'msg': '目标节点参数服务不可用'}
        req = SetParameters.Request()
        req.parameters = [Parameter(
            'dynamic_obstacle_enabled', Parameter.Type.BOOL,
            bool(enabled)).to_parameter_msg()]
        self.set_cli.call_async(req)

        result = {'ok': True, 'locked': bool(enabled)}

        if enabled:
            exp_ok, exp_msg = self.export_prior_map()
            result['exported'] = exp_ok
            result['export_msg'] = exp_msg

        # 3D 节点门控同步 —— 但若主节点当前使用内部 2D 配准
        #   (apply_pose_correction=true), 不能同时启用 3D 节点:
        #   两个发布者都发 /pose_correction 会互相打架 (网页/桥接收到交错修正量)。
        #   3D 节点未启动不算失败 (仅提示), 不影响 2D 流程。
        internal_2d = self._get_bool_param('apply_pose_correction')
        if enabled and internal_2d:
            result['enabled3d'] = False
            result['enabled3d_msg'] = '主节点使用内部 2D 配准, 已跳过 3D 启用'
        else:
            e_ok, e_msg = self._set_bool_param_3d('enabled', bool(enabled))
            result['enabled3d'] = e_ok
            result['enabled3d_msg'] = e_msg
        return result

    # ---------- 占点 (hold point) 行为 ----------
    def request_hold_point(self, enabled, col=None, row=None, radius=None,
                           standoff=None):
        """占点模式开关 / 设置占点中心 / 调整占点范围 / 调整站位间距系数。

        enabled=False 时关闭占点模式 (车辆交还普通导航逻辑)。
        col/row 提供时把点击像素换算成 map 系世界坐标写入 hold_point_x/y。
        radius 提供时同步写入占点范围 (m), 支持网页滑条动态调整。
        standoff 提供时同步写入站位间距系数 (越接近 1 越贴近敌人), 网页滑条调整。
        """
        if not self.set_cli.service_is_ready():
            return {'ok': False, 'msg': 'C++ 节点参数服务不可用, 请确认节点已启动'}

        params = [Parameter('hold_point_enabled', Parameter.Type.BOOL,
                            bool(enabled)).to_parameter_msg()]

        if radius is not None:
            params.append(Parameter('hold_point_radius', Parameter.Type.DOUBLE,
                                    float(radius)).to_parameter_msg())

        if standoff is not None:
            params.append(Parameter('hold_point_standoff_scale',
                                    Parameter.Type.DOUBLE,
                                    float(standoff)).to_parameter_msg())

        world = None
        if col is not None and row is not None:
            with self.history_lock:
                image = None if self.latest_history_image is None \
                    else self.latest_history_image.copy()
            with self.set_lock:
                meta = dict(self.map_meta)
            if image is None:
                return {'ok': False, 'msg': '暂无地图图像, 请稍后重试'}
            if meta.get('res') is None or meta.get('xmin') is None or \
                    meta.get('ymin') is None:
                return {'ok': False, 'msg': '地图参数未同步, 请稍后重试'}
            height, width = image.shape
            if col < 0 or col >= width or row < 0 or row >= height:
                return {'ok': False, 'msg': '点击位置超出地图'}
            x, y = self._pixel_to_world(col, row, height, meta)
            world = {'x': x, 'y': y}
            params.append(Parameter('hold_point_x', Parameter.Type.DOUBLE,
                                    float(x)).to_parameter_msg())
            params.append(Parameter('hold_point_y', Parameter.Type.DOUBLE,
                                    float(y)).to_parameter_msg())

        req = SetParameters.Request()
        req.parameters = params
        future = self.set_cli.call_async(req)
        begin = time.time()
        while not future.done() and time.time() - begin < 1.0:
            time.sleep(0.02)

        result = {'ok': True, 'enabled': bool(enabled)}
        if world is not None:
            result.update({'x': round(world['x'], 3),
                           'y': round(world['y'], 3)})
        if radius is not None:
            result['radius'] = round(float(radius), 3)
        if standoff is not None:
            result['standoff'] = round(float(standoff), 3)
        if enabled and world is None:
            self.get_logger().info('占点模式已开启 (沿用上次占点中心)')
        elif not enabled:
            self.get_logger().info('占点模式已关闭')
        elif world is not None:
            self.get_logger().info(
                '占点中心已设置: (%.2f, %.2f) m' % (world['x'], world['y']))
        return result

    def hold_point_state(self):
        """返回 C++ 节点发布的占点状态 (JSON), 供网页显示威胁/阻挡信息。"""
        with self.hold_lock:
            status = None if self.hold_status is None \
                else dict(self.hold_status)
            viz = None if self.hold_viz is None else list(self.hold_viz)
        if status is None:
            return {'ok': False}
        status['ok'] = True
        status['viz'] = viz
        return status

    def on_hold_status(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        with self.hold_lock:
            self.hold_status = data

    def on_hold_viz(self, msg):
        """占点可视化路径: 第 1 点=占点中心, 第 2 点=阻挡站位点 (可能不存在)。"""
        pts = self._path_to_points(msg)
        with self.hold_lock:
            self.hold_viz = pts

    # ---------- 追击 (chase) 行为 ----------
    def request_chase(self, enabled, col=None, row=None, radius=None):
        """追击模式开关 / 设置追击目标点 / 调整环带半径。

        enabled=False 时关闭追击模式 (车辆交还普通导航逻辑)。
        col/row 提供时把点击像素换算成 map 系世界坐标写入 chase_pick_x/y,
        C++ 节点据此就近锁定最近障碍簇作为追击目标。
        radius 提供时同步写入环带半径 (m), 支持网页滑条动态调整。
        """
        if not self.set_cli.service_is_ready():
            return {'ok': False, 'msg': 'C++ 节点参数服务不可用, 请确认节点已启动'}

        params = [Parameter('chase_enabled', Parameter.Type.BOOL,
                            bool(enabled)).to_parameter_msg()]

        if radius is not None:
            params.append(Parameter('chase_radius', Parameter.Type.DOUBLE,
                                    float(radius)).to_parameter_msg())

        world = None
        if col is not None and row is not None:
            with self.history_lock:
                image = None if self.latest_history_image is None \
                    else self.latest_history_image.copy()
            with self.set_lock:
                meta = dict(self.map_meta)
            if image is None:
                return {'ok': False, 'msg': '暂无地图图像, 请稍后重试'}
            if meta.get('res') is None or meta.get('xmin') is None or \
                    meta.get('ymin') is None:
                return {'ok': False, 'msg': '地图参数未同步, 请稍后重试'}
            height, width = image.shape
            if col < 0 or col >= width or row < 0 or row >= height:
                return {'ok': False, 'msg': '点击位置超出地图'}
            x, y = self._pixel_to_world(col, row, height, meta)
            world = {'x': x, 'y': y}
            params.append(Parameter('chase_pick_x', Parameter.Type.DOUBLE,
                                    float(x)).to_parameter_msg())
            params.append(Parameter('chase_pick_y', Parameter.Type.DOUBLE,
                                    float(y)).to_parameter_msg())

        req = SetParameters.Request()
        req.parameters = params
        future = self.set_cli.call_async(req)
        begin = time.time()
        while not future.done() and time.time() - begin < 1.0:
            time.sleep(0.02)

        result = {'ok': True, 'enabled': bool(enabled)}
        if world is not None:
            result.update({'x': round(world['x'], 3),
                           'y': round(world['y'], 3)})
        if radius is not None:
            result['radius'] = round(float(radius), 3)
        if not enabled:
            self.get_logger().info('追击模式已关闭')
        elif world is not None:
            self.get_logger().info(
                '追击目标点已设置: (%.2f, %.2f) m' % (world['x'], world['y']))
        else:
            self.get_logger().info('追击模式已开启 (沿用上次目标点)')
        return result

    def chase_state(self):
        """返回 C++ 节点发布的追击状态 (JSON), 供网页显示目标/丢失/搜索信息。"""
        with self.chase_lock:
            status = None if self.chase_status is None \
                else dict(self.chase_status)
            viz = None if self.chase_viz is None else list(self.chase_viz)
            track = None if self.chase_track is None \
                else list(self.chase_track)
        if status is None:
            return {'ok': False}
        status['ok'] = True
        status['viz'] = viz
        status['track'] = track
        return status

    def on_chase_status(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        with self.chase_lock:
            self.chase_status = data

    def on_chase_viz(self, msg):
        """追击可视化路径: 第 1 点=当前目标, 第 2 点=期望站位点。"""
        pts = self._path_to_points(msg)
        with self.chase_lock:
            self.chase_viz = pts

    def on_chase_track(self, msg):
        """目标历史轨迹: 目标位置点序列, 供网页画目标运动轨迹线。"""
        pts = self._path_to_points(msg)
        with self.chase_lock:
            self.chase_track = pts

    # ---------- 雷达->车体中心外参（网页滑条 / ros2 param set 均可修改） ----------
    def set_extrinsics(self, x, y):
        """更新外参: 同步内部缓存与本节点 ROS 参数, 并立刻反映到网页滑条"""
        self.lidar_to_base_x = float(x)
        self.lidar_to_base_y = float(y)
        try:
            self.set_parameters([
                Parameter('lidar_to_base_x', Parameter.Type.DOUBLE,
                          self.lidar_to_base_x),
                Parameter('lidar_to_base_y', Parameter.Type.DOUBLE,
                          self.lidar_to_base_y)])
        except Exception as e:  # noqa: B902
            self.get_logger().warning('写入外参参数失败: %s' % str(e))
        self.get_logger().info(
            '雷达->车体中心外参已更新: lidar_to_base=(%.3f, %.3f) m'
            % (self.lidar_to_base_x, self.lidar_to_base_y),
            throttle_duration_sec=0.5)

    def sync_extrinsics_from_params(self):
        """反向同步: 支持 ros2 param set /pgm_viewer lidar_to_base_x/y"""
        try:
            x = float(self.get_parameter('lidar_to_base_x').value)
            y = float(self.get_parameter('lidar_to_base_y').value)
        except Exception:  # noqa: B902
            return
        if x == self.lidar_to_base_x and y == self.lidar_to_base_y:
            return
        self.lidar_to_base_x = x
        self.lidar_to_base_y = y
        with self.set_lock:
            self.tbar['lx'] = to_tbar_ext(x)
            self.tbar['ly'] = to_tbar_ext(y)

    # ---------- 参数读取（定期同步滑条 + 地图元数据 xmin/ymin/resolution） ----------
    def fetch_current_params(self, timeout_sec=3.0, quiet=False):
        self.sync_extrinsics_from_params()
        if not self.get_cli.wait_for_service(timeout_sec=timeout_sec):
            if not quiet:
                self.get_logger().warning(
                    '找不到 %s 的参数服务, 滑条使用默认值 '
                    '(请确认 test_navigation 节点已启动)' % self.target_node)
            return

        req = GetParameters.Request()
        req.names = ['resolution', 'zmin', 'zmax', 'dilate', 'xmin', 'ymin',
                     'history_duration', 'history_min_hits',
                     'astar_cost_reach', 'astar_obstacle_penalty']
        future = self.get_cli.call_async(req)

        start = time.time()
        while not future.done() and time.time() - start < timeout_sec:
            time.sleep(0.05)

        if not future.done() or future.result() is None:
            if not quiet:
                self.get_logger().warning('读取参数超时, 滑条使用默认值')
            return

        vals = future.result().values

        def dbl(i, default):
            return vals[i].double_value \
              if vals[i].type == ParameterType.PARAMETER_DOUBLE else default

        def integ(i, default):
            return vals[i].integer_value \
              if vals[i].type == ParameterType.PARAMETER_INTEGER else default

        with self.set_lock:
            self.tbar['res'] = to_tbar_resolution(dbl(0, 0.05))
            self.tbar['zmin'] = to_tbar_z(dbl(1, -0.5))
            self.tbar['zmax'] = to_tbar_z(dbl(2, 1.0))
            self.tbar['dil'] = to_tbar_dilate(integ(3, 2))
            self.tbar['hist'] = int(min(600, max(1, round(dbl(6, 300)))))
            self.tbar['hits'] = int(min(20, max(1, integ(7, 2))))
            self.tbar['cr'] = to_tbar_cost_reach(dbl(8, 0.60))
            self.tbar['op'] = to_tbar_penalty(dbl(9, 0.80))
            self.map_meta['res'] = dbl(0, None)
            self.map_meta['xmin'] = dbl(4, None)
            self.map_meta['ymin'] = dbl(5, None)

        if not quiet:
            self.get_logger().info(
                '已同步节点当前参数: resolution=%.2f zmin=%.2f zmax=%.2f '
                'dilate=%d '
                'history_duration=%ds history_min_hits=%d '
                'astar_cost_reach=%.2f astar_obstacle_penalty=%.2f'
                % (from_tbar_resolution(self.tbar['res']),
                   from_tbar_z(self.tbar['zmin']),
                   from_tbar_z(self.tbar['zmax']),
                   self.tbar['dil'],
                   self.tbar['hist'],
                   self.tbar['hits'],
                   from_tbar_cost_reach(self.tbar['cr']),
                   from_tbar_penalty(self.tbar['op'])))

    # ---------- 后台周期同步（防止外部 ros2 param set 改动 xmin/ymin/resolution） ----------
    def sync_loop(self):
        while rclpy.ok():
            time.sleep(5.0)
            if not rclpy.ok():
                break
            try:
                self.fetch_current_params(timeout_sec=1.0, quiet=True)
            except Exception:
                pass

    # ---------- 参数写入（网页滑条 -> C++ 节点） ----------
    def send_params(self):
        if not self.set_cli.service_is_ready():
            self.get_logger().warning(
                '目标节点参数服务不可用, 跳过本次修改',
                throttle_duration_sec=5.0)
            return False

        req = SetParameters.Request()
        req.parameters = [
            Parameter('resolution', Parameter.Type.DOUBLE,
                      from_tbar_resolution(self.tbar['res'])).to_parameter_msg(),
            Parameter('zmin', Parameter.Type.DOUBLE,
                      from_tbar_z(self.tbar['zmin'])).to_parameter_msg(),
            Parameter('zmax', Parameter.Type.DOUBLE,
                      from_tbar_z(self.tbar['zmax'])).to_parameter_msg(),
            Parameter('dilate', Parameter.Type.INTEGER,
                      self.tbar['dil']).to_parameter_msg(),
            Parameter('history_duration', Parameter.Type.DOUBLE,
                      float(self.tbar['hist'])).to_parameter_msg(),
            Parameter('history_min_hits', Parameter.Type.INTEGER,
                      self.tbar['hits']).to_parameter_msg(),
            Parameter('astar_cost_reach', Parameter.Type.DOUBLE,
                      from_tbar_cost_reach(self.tbar['cr'])).to_parameter_msg(),
            Parameter('astar_obstacle_penalty', Parameter.Type.DOUBLE,
                      from_tbar_penalty(self.tbar['op'])).to_parameter_msg(),
        ]
        future = self.set_cli.call_async(req)
        future.add_done_callback(self.on_set_done)
        return True

    def on_set_done(self, future):
        try:
            result = future.result()
        except Exception as e:  # noqa: B902
            self.get_logger().warning('参数设置请求失败: %s' % str(e))
            return
        rejected = [r.reason for r in result.results if not r.successful]
        if rejected:
            self.get_logger().warning(
                '节点拒绝了修改(检查 zmin<=zmax 等): ' + '; '.join(rejected),
                throttle_duration_sec=2.0)
        else:
            self.get_logger().info('参数已更新')

    # ---------- 网页"刷新累计图" -> 清空节点历史点云缓冲并重建 ----------
    def request_history_reset(self):
        if not self.set_cli.service_is_ready():
            self.get_logger().warning(
                '目标节点参数服务不可用, 无法刷新累计图',
                throttle_duration_sec=5.0)
            return False

        def call(value, timeout_sec=1.0):
            req = SetParameters.Request()
            req.parameters = [
                Parameter('clear_history', Parameter.Type.BOOL,
                          value).to_parameter_msg()]
            future = self.set_cli.call_async(req)
            start = time.time()
            while not future.done() and time.time() - start < timeout_sec:
                time.sleep(0.02)
            return future.result() if future.done() else None

        result = call(True)
        # 复位为 False, 保证下次点击仍构成有效取值变化
        call(False)
        if result is None:
            self.get_logger().warning(
                '刷新累计图请求超时', throttle_duration_sec=5.0)
            return False
        ok = all(r.successful for r in result.results)
        if ok:
            self.get_logger().info('已触发历史累计图重建(清空历史点云)')
        return ok

    # ---------- 历史累计图导出 / 导入 ----------
    def _set_bool_trigger(self, name, value=True, timeout_sec=1.0):
        """置位一次性布尔触发参数并复位, 返回 C++ 节点是否接受。"""
        if not self.set_cli.service_is_ready():
            return False

        def call(v):
            req = SetParameters.Request()
            req.parameters = [
                Parameter(name, Parameter.Type.BOOL,
                          bool(v)).to_parameter_msg()]
            future = self.set_cli.call_async(req)
            begin = time.time()
            while not future.done() and time.time() - begin < timeout_sec:
                time.sleep(0.02)
            return future.result() if future.done() else None

        result = call(value)
        call(not value)   # 复位, 保证下次点击仍构成有效取值变化
        return result is not None and all(r.successful for r in result.results)

    def _set_string_param(self, name, value, timeout_sec=1.0):
        if not self.set_cli.service_is_ready():
            return False
        req = SetParameters.Request()
        req.parameters = [
            Parameter(name, Parameter.Type.STRING,
                      str(value)).to_parameter_msg()]
        future = self.set_cli.call_async(req)
        begin = time.time()
        while not future.done() and time.time() - begin < timeout_sec:
            time.sleep(0.02)
        if not future.done() or future.result() is None:
            return False
        return all(r.successful for r in future.result().results)

    def export_prior_map(self):
        """把历史点云队列导出为 3D 重定位节点的先验地图 (覆盖式, 不下载):
        写出 export_map_dir/map_clouds.bin (RPGMCLD1 格式), 节点按 mtime
        轮询自动重载。返回 (ok, msg)。由 HTTP 线程调用 (轮询等待)。"""
        export_dir = str(self.get_parameter('export_map_dir').value)
        try:
            os.makedirs(export_dir, exist_ok=True)
            # 先删旧文件: 既保证"每次覆盖上次导出", 又以文件重新出现作为
            #   节点写完的判据 (避免读到上次的残留)
            for name in ('map.pgm', 'map.yaml', 'map_clouds.bin'):
                path = os.path.join(export_dir, name)
                if os.path.exists(path):
                    os.remove(path)
        except OSError as e:  # noqa: B902
            return False, '创建导出目录失败: %s' % str(e)

        if not self._set_string_param('export_map_dir', export_dir):
            return False, 'C++ 节点参数服务不可用, 无法导出'
        if not self._set_bool_trigger('export_map_request', True):
            return False, '导出请求失败 (节点未响应)'

        # map_clouds.bin 在渲染定时器内最后写出: 等它出现即表示三件套写完
        bin_path = os.path.join(export_dir, 'map_clouds.bin')
        deadline = time.time() + 5.0
        while time.time() < deadline:
            if os.path.exists(bin_path):
                self.get_logger().info('先验地图已导出: %s' % bin_path)
                return True, export_dir
            time.sleep(0.05)
        return False, '导出超时: 未生成 map_clouds.bin'

    def export_history_map(self):
        """网页"导出地图": 让 C++ 节点把历史累计图 (PGM+YAML) 与原始点云队列
        写到 export_map_dir, 然后打包成 zip 返回字节流供浏览器下载。
        返回 (ok, data_or_msg)。"""
        export_dir = str(self.get_parameter('export_map_dir').value)
        try:
            os.makedirs(export_dir, exist_ok=True)
            # 清空旧文件, 避免导出残留
            for name in ('map.pgm', 'map.yaml', 'map_clouds.bin',
                         'map_export.zip'):
                path = os.path.join(export_dir, name)
                if os.path.exists(path):
                    os.remove(path)
        except OSError as e:  # noqa: B902
            return False, '创建导出目录失败: %s' % str(e)

        if not self._set_string_param('export_map_dir', export_dir):
            return False, 'C++ 节点参数服务不可用, 无法导出'
        if not self._set_bool_trigger('export_map_request', True):
            return False, '导出请求失败 (节点未响应)'

        # 等待 C++ 节点写出文件 (渲染定时器周期内完成)
        pgm = os.path.join(export_dir, 'map.pgm')
        deadline = time.time() + 3.0
        while time.time() < deadline and not os.path.exists(pgm):
            time.sleep(0.05)

        if not os.path.exists(pgm):
            return False, '导出超时: 未生成地图文件'

        buffer = io.BytesIO()
        try:
            with zipfile.ZipFile(buffer, 'w', zipfile.ZIP_DEFLATED) as zf:
                for name in ('map.pgm', 'map.yaml', 'map_clouds.bin'):
                    path = os.path.join(export_dir, name)
                    if os.path.exists(path):
                        zf.write(path, name)
        except OSError as e:  # noqa: B902
            return False, '打包导出文件失败: %s' % str(e)

        self.get_logger().info('历史累计图已导出并打包 (zip)')
        return True, buffer.getvalue()

    def import_history_map(self, zip_bytes):
        """网页"导入地图": 解包上传的 zip (含 map.pgm/map.yaml/map_clouds.bin)
        到 import_map_dir, 再让 C++ 节点读取恢复。返回 (ok, msg)。"""
        import_dir = str(self.get_parameter('import_map_dir').value)
        try:
            if os.path.isdir(import_dir):
                shutil.rmtree(import_dir)
            os.makedirs(import_dir, exist_ok=True)
        except OSError as e:  # noqa: B902
            return False, '创建导入目录失败: %s' % str(e)

        # 安全解包: 只提取白名单文件, 防止路径穿越
        allowed = ('map.pgm', 'map.yaml', 'map_clouds.bin')
        try:
            with zipfile.ZipFile(io.BytesIO(zip_bytes)) as zf:
                for member in zf.namelist():
                    base = os.path.basename(member)
                    if base not in allowed or not member:
                        continue
                    target = os.path.join(import_dir, base)
                    with zf.open(member) as src, open(target, 'wb') as dst:
                        shutil.copyfileobj(src, dst)
        except (zipfile.BadZipFile, OSError) as e:  # noqa: B902
            return False, '解包上传文件失败: %s' % str(e)

        if not os.path.exists(os.path.join(import_dir, 'map.pgm')) and \
                not os.path.exists(
                    os.path.join(import_dir, 'map_clouds.bin')):
            return False, '压缩包中缺少 map.pgm 或 map_clouds.bin'

        if not self._set_string_param('import_map_dir', import_dir):
            return False, 'C++ 节点参数服务不可用, 无法导入'
        if not self._set_bool_trigger('import_map_request', True):
            return False, '导入请求失败 (节点未响应)'

        self.get_logger().info('历史累计图已导入 (%s)' % import_dir)
        return True, '导入完成'

    # ---------- 网页"开始运动 / 停止运动" -> C++ 节点下发 8 字节运动帧 ----------
    def request_motion(self, start):
        """置位 C++ 节点的一次性触发参数 (start_motion / stop_motion),
        C++ 节点按 Nav2 全局路径跟随并发布运动帧。
        与 clear_history 相同: 置位后立刻复位, 保证下次点击仍构成有效取值变化。"""
        name = 'start_motion' if start else 'stop_motion'
        if not self.set_cli.service_is_ready():
            return {'ok': False,
                    'msg': 'C++ 节点参数服务不可用, 请确认节点已启动'}

        def call(value, timeout_sec=1.0):
            req = SetParameters.Request()
            req.parameters = [
                Parameter(name, Parameter.Type.BOOL,
                          value).to_parameter_msg()]
            future = self.set_cli.call_async(req)
            begin = time.time()
            while not future.done() and time.time() - begin < timeout_sec:
                time.sleep(0.02)
            return future.result() if future.done() else None

        with self.motion_lock:
            self.motion_busy = True
        try:
            result = call(True)
            call(False)   # 复位为 False, 使下次点击仍为变化
        finally:
            with self.motion_lock:
                self.motion_busy = False

        if result is None:
            return {'ok': False, 'msg': '运动指令请求超时'}

        rejected = [r.reason for r in result.results if not r.successful]
        if rejected:
            return {'ok': False, 'msg': '; '.join(rejected)}

        msg = '已请求开始运动 (按 Nav2 全局路径跟随)' if start \
            else '已请求停止运动 (立即下发 control=0)'
        self.get_logger().info(msg)
        return {'ok': True, 'msg': msg}

    def motion_state(self):
        """返回最近一次运动帧状态, 供网页显示"""
        with self.motion_lock:
            fb = dict(self.motion_fb)
            fb['busy'] = self.motion_busy
        fb['ok'] = True
        return fb

    def lookahead_state(self):
        """返回路径跟随可视化快照 (前视点/车体中心/速度矢量/车头航向)。
        仅在运动帧仍在持续到达 (跟随进行中) 且未到达终点时有效,
        这样网页上的前视点/向量/车体系会随运动实时移动, 停车后自动消失。"""
        with self.motion_lock:
            geometry = None if self.lookahead is None else dict(self.lookahead)
            fb = dict(self.motion_fb)
            last_t = self.motion_last_t
        fresh = last_t > 0.0 and time.monotonic() - last_t < 1.0
        if geometry is None or not fresh or not fb.get('active'):
            return {'ok': False, 'active': False}
        return {
            'ok': True,
            'active': True,
            'base': geometry['base'],
            'target': geometry['target'],
            'yaw': geometry['yaw'],
            'vx': fb['vx'],
            'vy': fb['vy'],
            'speed': round(math.hypot(fb['vx'], fb['vy']), 3),
            'lookahead_m': geometry['lookahead_m'],
            'nearest_index': geometry['nearest_index'],
            'lookahead_index': geometry['lookahead_index'],
            'plan_points': geometry['plan_points'],
            # 剩余距离: 直线(dist_end_m, 与 C++ 到达判定/减速同一量) 与
            # 沿路径弧长(remaining_m)
            'dist_end_m': geometry['dist_end_m'],
            'remaining_m': geometry['remaining_m']}



def main():
    rclpy.init()
    node = PgmViewer()

    def spin_until_shutdown():
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
            pass

    spin_thread = threading.Thread(target=spin_until_shutdown, daemon=True)
    spin_thread.start()
    node.fetch_current_params()
    threading.Thread(target=node.sync_loop, daemon=True).start()
    try:
        while rclpy.ok():
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.http_server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == '__main__':
    main()
