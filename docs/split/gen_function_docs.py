#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从 test_navigation.cpp 抽取每个函数/结构体/配置块, 生成拆分用的分析文档。

用法:
    python3 docs/split/gen_function_docs.py

输入:  test/test_navigation/src/test_navigation.cpp (7937 行, 单类 RealtimePgmNode)
输出:  docs/split/01_config.md / 02_tools.md / 03_tasks.md / 04_node.md / INDEX.md

设计要点:
  * 元数据只记录"起始行 + 职责 + 目标文件", 结束行由花括号配对自动求出,
    避免手工维护行号导致漂移。
  * 花括号扫描会跳过 // 注释、/* */ 注释、字符串与字符字面量,
    因此 RCLCPP_INFO 里的中文/符号不会干扰配对。
  * 需要整块搬运的区域(参数声明区/成员变量区/装配区)用 kind='block' 显式给 end。
"""

import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.normpath(
    os.path.join(HERE, '..', '..', 'test', 'test_navigation', 'src', 'test_navigation.cpp'))
OUT_DIR = HERE


# --------------------------------------------------------------------------
# 花括号配对: 求函数/结构体的结束行 (1-based, 闭区间)
# --------------------------------------------------------------------------
def find_end_line(lines, start_line):
    """从 start_line(1-based) 开始, 返回匹配到第一层 {} 闭合的行号(1-based)。"""
    depth = 0
    started = False
    in_block_comment = False
    for idx in range(start_line - 1, len(lines)):
        line = lines[idx]
        i = 0
        in_str = False
        in_chr = False
        n = len(line)
        while i < n:
            ch = line[i]
            if in_block_comment:
                if ch == '*' and i + 1 < n and line[i + 1] == '/':
                    in_block_comment = False
                    i += 2
                    continue
                i += 1
                continue
            if in_str:
                if ch == '\\':
                    i += 2
                    continue
                if ch == '"':
                    in_str = False
                i += 1
                continue
            if in_chr:
                if ch == '\\':
                    i += 2
                    continue
                if ch == "'":
                    in_chr = False
                i += 1
                continue
            if ch == '/' and i + 1 < n and line[i + 1] == '/':
                break                      # 行注释, 本行剩余忽略
            if ch == '/' and i + 1 < n and line[i + 1] == '*':
                in_block_comment = True
                i += 2
                continue
            if ch == '"':
                in_str = True
                i += 1
                continue
            if ch == "'":
                in_chr = True
                i += 1
                continue
            if ch == '{':
                depth += 1
                started = True
            elif ch == '}':
                depth -= 1
                if started and depth == 0:
                    return idx + 1
            i += 1
    raise RuntimeError('未能找到起始行 %d 的闭合花括号' % start_line)


# --------------------------------------------------------------------------
# 元数据: (名称, 起始行, 目标位置, 职责, 依赖/备注)
#   kind: func = 函数(自动配对); block = 显式区间; type = 结构体(自动配对)
# --------------------------------------------------------------------------
T = 'func'

TOOLS = [
    # --- 几何 / 变换 ---
    ('hasField', 1217, 'tools/ros_utils.hpp / ros_utils.cpp',
     '检查 PointCloud2 是否含指定字段名 (static, 无成员访问)',
     '入参 sensor_msgs::msg::PointCloud2, 纯查询'),
    ('quaternionToMatrix', 1240, 'tools/tf_math.hpp (inline)',
     '四元数 -> 行主序 3x3 旋转矩阵, 含归一化与退化时取单位阵',
     'static; 入参 x,y,z,w; 出参 double r[9]'),
    ('matrixToQuaternion', 1267, 'tools/tf_math.hpp (inline)',
     '3x3 旋转矩阵 -> 四元数 (trace 分支法)',
     'static; 出参 x,y,z,w'),
    ('yawFromMatrix', 1301, 'tools/tf_math.hpp (inline)',
     '从行主序 3x3 取纯 z 旋转偏航角 atan2(r[3], r[0])',
     'static; 仅两元素'),
    ('wrapAngle', 1307, 'tools/tf_math.hpp (inline)',
     '角度归一化到 (-pi, pi]',
     'static; 纯标量'),
    ('multiplyMatrix', 1321, 'tools/tf_math.hpp (inline)',
     '3x3 行主序矩阵相乘',
     'static; 纯标量'),
    ('transformPointByCorrection', 1398, 'tools/tf_math.hpp (inline)',
     '单点刚体变换 p_map = R * p + t',
     'const; 入参 Point + PoseCorrection'),
    ('score_at', 1469, 'tools/registration.hpp',
     '位姿命中率评分 (estimatePoseCorrection 内部的 lambda): 采样点变换后查栅格, 出界计入分母',
     'block(1469-1505); 捕获 samples/config/mask/width/height'),
    ('transformToBaseLink', 2454, 'tools/tf_math.cpp',
     'map -> base_link: 先用 odom + 雷达外参反推车体中心, 再 R(-yaw)*(p-base)',
     '读 odom_x_/odom_y_/odom_yaw_/odom_received_/motion_base_to_lidar_x,y_; 改造成入参即纯函数'),
    # --- 运动帧编解码 ---
    ('limitMotionSpeed', 3590, 'tools/motion_codec.hpp',
     '速度保方向等比限幅: 峰值超 kMotionMaxSpeed 时两轴同比缩放',
     'static'),
    ('effectiveSlowRadius', 3606, 'tools/motion_codec.hpp',
     '减速起始距离: 手动值优先, 否则 max(阈值+0.05, kSlowRadiusTime*目标速度)',
     'static'),
    ('encodeMotionRaw', 3619, 'tools/motion_codec.hpp',
     'm/s -> 12 位有符号原始量: 就近量化 + ±2040 饱和 + &0x0FFF',
     'static'),
    ('resolveAstarSpeed', 5808, 'tools/motion_codec.hpp',
     'A* 兜底速度解析: <=0 时回落到 motion_target_speed_, 并夹到 kMotionMaxSpeed',
     'const'),
    # --- 栅格几何 ---
    ('computeMapSize', 6153, 'tools/grid_geom.cpp',
     '由 Config 的地图范围/分辨率算 width/height, 并校验像素总数上限',
     '准纯 (仅 RCLCPP_ERROR_THROTTLE 日志, 建议改为返回错误码)'),
    ('computeVehicleFootprint', 6210, 'tools/grid_geom.cpp',
     '由里程计位姿 + self_filter_size 算出世界系轴对齐车体方框 FootprintBounds',
     '读 odom_* + config_.self_filter_size + 外参, 入参化后可纯'),
    ('clipCloudToMapBounds', 6262, 'tools/grid_geom.cpp',
     '按半开区间 [xmin,xmax) x [ymin,ymax) 就地裁剪点云, 返回丢弃点数',
     '读 config_ 四个边界'),
    ('filterVehicleFootprintPoints', 6301, 'tools/grid_geom.cpp',
     '点级自过滤: 丢弃落在车体方框(闭区间)内的本体点',
     '调用 computeVehicleFootprint'),
    ('clearVehicleFootprint', 6337, 'tools/grid.cpp',
     '栅格级清除: 车体方框内占据格一律置空',
     '准纯, 仅 RCLCPP_DEBUG'),
    ('accumulateCloud', 6395, 'tools/grid.cpp',
     '单帧点云投影入占据栅格 (z 过滤), 支持"同帧只计一次"的命中计数模式',
     '零成员零 ROS, 最干净的抽取对象'),
    ('applyDilate', 6462, 'tools/grid.cpp',
     '占据栅格圆形(方形核)膨胀指定格数',
     '零成员零 ROS'),
    ('occupancyToPixels', 6522, 'tools/grid.cpp',
     '占据栅格 -> mono8 像素 (0=占据, 205=未知), 行序按 PGM 约定翻转',
     '零成员零 ROS'),
    # --- 序列化 ---
    ('saveCloudPcd', 867, 'tools/io_pcd.cpp',
     '把一帧点云写成 ascii PCD 快照 (ASCII, 带时间戳文件名)',
     '读 debug_* 三个参数, 入参化后纯 IO'),
    ('writeHistoryCloudsBin', 6839, 'tools/io_bin.cpp',
     '历史点云队列 -> 自有二进制 (magic + 每帧时间戳 + float32 xyz)',
     '准纯, 仅日志; 无 rclcpp 依赖的改造点见 README'),
    ('readHistoryCloudsBin', 6895, 'tools/io_bin.cpp',
     '自有二进制 -> 历史点云队列 (反序列化, 校验 magic/长度)',
     '准纯, 仅日志'),
    ('writeMapYaml', 6985, 'tools/io_yaml.cpp',
     '写 map_server 风格 YAML (image/resolution/origin/negate/occupied_thresh)',
     '准纯, 仅日志'),
    # --- 栅格查询 / 聚类 (占点与追击共用) ---
    ('extractHoldObstacles', 4726, 'tools/grid_cluster.cpp',
     '按固定步长扫描占据栅格, 在线聚类(加权桶)求障碍簇质心, 只保留范围内且间距达标的簇',
     '含局部 struct Bucket; 读 astar_inflate_radius_'),
    ('computeHoldBlockPose', 4825, 'tools/grid_geom.cpp',
     '沿"障碍 -> 中心"连线取距障碍 standoff 的阻挡站位点',
     'const; standoff 入参化即纯'),
    ('isHoldPointClear', 4863, 'tools/grid_clearance.cpp',
     '点级硬性间距自检: 圆盘邻域内不得有未豁免的占据格',
     'const; 参数全传入, 已是纯栅格查询'),
    ('isHoldPointClearPath', 4934, 'tools/grid_clearance.cpp',
     '线段级硬性间距自检: 等间隔采样(上限 4000 点)后逐点判定',
     'const; 复用 isHoldPointClear 判据'),
    ('findNearestObstacleCluster', 5318, 'tools/grid_cluster.cpp',
     '在 (ref_x,ref_y) 为圆心、radius 为半径内取最近障碍簇质心',
     '调用 extractHoldObstacles'),
    ('chaseLineClear', 5352, 'tools/grid_clearance.cpp',
     '车体 -> 目标连线是否通畅(豁免目标本体), 决定直连还是 A* 绕行',
     '读 astar_inflate_radius_'),
]

TASKS = [
    # --- 点云流水线 ---
    ('cloudCallback', 1715, 'tasks/cloud_pipeline.cpp',
     '点云主流水线: 字段校验 -> 解析+isfinite -> 跳变拒绝窗丢弃 -> 位姿修正 -> 地图裁剪 '
     '-> 车体点级自过滤 -> 发自过滤点云 -> 动态障碍分离/绿层/清除环 -> 历史降采样 -> 调试日志 -> 入队',
     '订阅回调; 涉及 cloud/dynamic/config/motion 四把锁; 调用 8 个 tools 函数'),
    ('pruneHistory', 1952, 'tasks/cloud_pipeline.cpp',
     '按 history_duration 淘汰历史队列队首; 动态障碍模式下直接跳过',
     '读写 history_'),
    ('logCloudFrame', 917, 'tasks/debug_log.cpp',
     '逐帧统计质心/包围盒, 判定点数突增与质心跳变, 写 CSV, 异常时落 PCD 快照',
     '读多个 debug_* 阈值成员'),
    ('splitDynamicObstacles', 1985, 'tasks/dynamic_split.cpp',
     '用冻结的静态基准掩码把"新出现的临时障碍点"从整帧中原地摘出',
     '读写 dynamic_mutex_ 下的 static_obstacle_mask_'),
    ('subtractBaseline', 2087, 'tasks/dynamic_split.cpp',
     '同判据但不改输入, 输出"整帧 - 基准占据"的动态点 (绿色层数据源)',
     '与 splitDynamicObstacles 共用判据'),
    ('buildStaticBaseline', 2161, 'tasks/dynamic_split.cpp',
     '锁定瞬间: 历史累积图按 history_min_hits 投票 + 并入成品图 + 扣车体 + baseline_dilate 膨胀 '
     '-> 基准栅格; 再膨胀 1 格得到配准评分容差掩码',
     '写 static_obstacle_mask_ / static_obstacle_tolerance_'),
    # --- 位姿修正 ---
    ('getPoseCorrection', 1335, 'tasks/pose_correction.cpp',
     '加锁拷贝当前 map->odom 修正量',
     '读 pose_correction_mutex_'),
    ('publishPoseCorrection', 1342, 'tasks/pose_correction.cpp',
     '修正量转 PoseStamped 并发布到 /pose_correction',
     '发布'),
    ('setPoseCorrection2D', 1362, 'tasks/pose_correction.cpp',
     '(x,y,yaw) -> SE(3) 写入成员并广播',
     '写 pose_correction_, 经 publishPoseCorrection'),
    ('resetPoseCorrection', 1380, 'tasks/pose_correction.cpp',
     '置恒等修正; 仅 apply_pose_correction_ 为真时广播',
     '写 pose_correction_'),
    ('estimatePoseCorrection', 1411, 'tasks/pose_correction.cpp (+ tools/registration.hpp 评分内核)',
     '2D 栅格配准: 粗搜(±range/步长) + 亚格细化 + 单帧速率限幅 + 滞回/EMA/死区; '
     '失败计数达阈值触发全局重搜; 成功则 setPoseCorrection2D',
     '读 config_ + static_obstacle_mask_/tolerance_; 写原子失败计数; 内核 score_at 在 tools'),
    ('correctCloudForPose', 1690, 'tasks/pose_correction.cpp',
     '按 apply_pose_correction_ 二选一(自估 / 外部修正)后把整帧点云变换到 map 系',
     '读 pose_correction_ / dynamic_obstacle_enabled_'),
    # --- 点云发布 ---
    ('publishCloudPreview', 2323, 'tasks/cloud_publish.cpp',
     '按 cloud_preview_max_points_ 等步长抽稀后发布预览 PointCloud2; 无订阅者直接返回',
     '发布'),
    ('pointCloudStamp', 2384, 'tasks/cloud_publish.cpp',
     '生成 Nav2 安全时间戳: 最新里程计戳 - 0.1 s (避免 TF 外推到未来导致整帧被丢)',
     '读 odom_stamp_'),
    ('publishPointCloud', 2412, 'tasks/cloud_publish.cpp',
     '把点列打包成 PointCloud2 (frame_id 入参) 并发布',
     '发布; 时间戳来自 pointCloudStamp'),
    ('publishSelfFilteredCloud', 2510, 'tasks/cloud_publish.cpp',
     '限频 + 抽稀 + 转 base_link 后发布 (Nav2 obstacle_layer 的 clearing 源)',
     '写 self_filtered_last_publish_'),
    ('publishDynamicObstacleCloud', 7346, 'tasks/cloud_publish.cpp',
     '组避障点云(整帧 + 基准栅格取点) 发 Nav2',
     '调用 transformToBaseLink / publishPointCloud'),
    ('publishClearRingCloud', 7432, 'tasks/cloud_publish.cpp',
     '生成 base_link 系环点并发布, 让代价图沿射线清除旧红色区域',
     '读 clear_ring_points_/clear_ring_radius_'),
    # --- 建图渲染 ---
    ('renderPgm', 6554, 'tasks/map_render.cpp',
     '单帧建图流程: 算尺寸 -> 累积点云 -> 扣车体 -> 膨胀 -> 转像素 -> 发 Image -> 写 PGM 文件',
     '发布; 文件 IO'),
    ('renderHistoryPgm', 6659, 'tasks/map_render.cpp',
     '历史多帧投票累积渲染 -> 占据比例暴涨检测(WARN) -> 发 Image -> 写 PGM',
     '读 debug_prev_occ_* 统计'),
    ('renderDynamicObstaclePgm', 7279, 'tasks/map_render.cpp',
     '动态障碍栅格 -> 绿色层 Image 发布',
     '写 dynamic_image_published_'),
    # --- 地图导入导出 ---
    ('exportHistoryMap', 7046, 'tasks/map_io.cpp',
     '导出编排: 渲染历史图 + 写 PGM/YAML/BIN',
     '读 history_ / config_ / export_map_dir_'),
    ('importHistoryMap', 7158, 'tasks/map_io.cpp',
     '导入编排: 读 PGM/BIN -> 重建 history_ 与 imported_map_, 清 Nav2 代价图',
     '调用 buildStaticBaseline / clearCostmaps'),
    ('mergeImportedMap', 7026, 'tasks/map_io.cpp',
     '导入图占据格叠加到当前像素(仅覆盖未知格)',
     '读写 imported_map_*'),
    # --- A* ---
    ('buildMergedOccupancy', 4154, 'tasks/astar_planner.cpp',
     '合并占据栅格 = 历史累计 ∪ 动态障碍(同一 accumulateCloud/applyDilate 口径), 再扣车体方框; '
     'A* / 占点 / 追击共用',
     '读 history_ / latest_dynamic_obstacles_'),
    ('planAstarPath', 4239, 'tasks/astar_planner.cpp',
     'A* 入口: 把雷达位姿反推为车体中心, 以 Web 目标 goal_x_/goal_y_ 为终点调用 planAstarToGoal',
     '读 astar_enabled_ / goal_received_'),
    ('planAstarToGoal', 4273, 'tasks/astar_planner.cpp (+ tools/astar.cpp 内核)',
     'A* 核心: 建图 -> 膨胀 -> 障碍代价场 -> 八连通搜索 -> 回溯 -> 弧长抽稀',
     '内核可拆: inflateGrid / findNearestFreeCell / buildObstacleCostField / astarSearch / backtrackCells / cellsToWorld / resampleByArcLength'),
    ('getAstarPath', 4612, 'tasks/astar_planner.cpp',
     '带限频的规划入口: 按时间或位移判断是否重规划, 否则复用缓存',
     '读写 astar_cache_valid_ / astar_cached_path_ / astar_last_plan_*'),
    ('publishAstarPlan', 4672, 'tasks/astar_planner.cpp',
     'A* 路径 -> nav_msgs/Path (map 系) 供网页显示',
     '发布 /astar_plan'),
    # --- 占点 ---
    ('computeHoldPointCommand', 5017, 'tasks/hold_point.cpp',
     '占点主流程: 读参数 -> 建图 -> 聚簇 -> 仲裁最近威胁 -> 算站位点 -> 硬性间距自检 -> 跟随到站 '
     '-> 机动安全兜底 -> 记录状态 -> 发可视化',
     '读写 hold_point_*; 复用 motion_* 参数; 调 tools 的 extractHoldObstacles / computeHoldBlockPose / isHoldPointClearPath'),
    ('publishHoldPointViz', 5221, 'tasks/hold_point.cpp',
     '发布"占点中心 + 本帧期望点"两点 Path',
     '发布'),
    ('publishHoldPointStatus', 5254, 'tasks/hold_point.cpp',
     '发布占点状态 JSON (enabled/x/y/radius/threat*)',
     'std_msgs/String 发布'),
    # --- 追击 ---
    ('computeChaseCommand', 5371, 'tasks/chase.cpp',
     '追击主流程: 锁定/跟踪目标 -> 节流记录轨迹 -> 算环带站位点 -> 连线检查 -> A* 或直连跟随 '
     '-> 安全兜底 -> 记录 + 发布',
     '读写 chase_*; 调 extractHoldObstacles / findNearestObstacleCluster / chaseLineClear / planAstarToGoal'),
    ('recordTargetCloud', 5638, 'tasks/chase.cpp',
     '把目标邻域(半径约 radius*1.5)内的 3D 点云连同时间戳追加到 chase_track_, 按时长淘汰',
     '读 latest_cloud_, 写 chase_track_'),
    ('publishChaseViz', 5674, 'tasks/chase.cpp',
     '发布"目标点 + 站位点"两点 Path',
     '发布'),
    ('publishChaseStatus', 5713, 'tasks/chase.cpp',
     '发布追击状态 JSON (valid/target/goal/radius/track 点数)',
     'std_msgs/String 发布'),
    ('publishChaseTrack', 5761, 'tasks/chase.cpp',
     '发布目标历史轨迹 Path (网页画轨迹线)',
     '发布'),
    # --- 运动控制 ---
    ('computeFollowCommand', 5823, 'tasks/motion_control.cpp (+ tools/pure_pursuit.cpp 几何内核)',
     '跟随核心: 外参反推车体中心 -> 到达判定 -> 前视半平面最近点 -> 横向偏差保护 -> 梯形减速 '
     '-> 速度自适应 lookahead -> 弧长推进取前视点 -> 投影成车体系 (vx,vy) -> 写前视可视化',
     '内核可拆: baseFromLidar / nearestOnPlan / trapezoidSpeed / lookaheadByArc / projectToBody'),
    ('computeAstarFollowCommand', 5792, 'tasks/motion_control.cpp',
     'A* 兜底路径跟随薄壳: 解析巡航速度后转调 computeFollowCommand',
     '读 astar_speed_ / motion_target_speed_'),
    ('startMotion', 6038, 'tasks/motion_control.cpp',
     '开始运动: 校验里程计与 /plan (无 plan 但有目标点时允许, 交给 A* 兜底), 置运动态',
     '读写 motion_active_ / motion_waiting_ / astar_cache_valid_'),
    ('stopMotion', 6122, 'tasks/motion_control.cpp',
     '停车: 清运动/等待/重规划标志, 作废前视可视化, 下发 0 速帧',
     '调用 publishMotionFrame'),
    ('publishMotionFrame', 3432, 'tasks/motion_frame.cpp (打包段可入 tools/motion_codec)',
     '组装 8 字节运动帧(control 位 + vx/vy 各 12 位, vy 下发取反) -> 发 /motion -> 可选 CAN 直发',
     '读写 motion_vx_scale_/motion_vy_scale_/motion_control_'),
    ('clearCostmaps', 3317, 'tasks/nav2_service.cpp',
     '异步调用 Nav2 ClearEntireCostmap 清空局部/全局代价地图, 不阻塞渲染线程',
     '服务客户端'),
]

NODE = [
    ('constructor_assembly', 605, 'src/realtime_pgm_node.cpp',
     '构造函数装配区: 创建 6 个订阅 / 12 个发布器 / 2 个定时器 / 2 个服务客户端 / 参数回调句柄',
     'block(605-815); 参数声明区(118-604)见 01_config.md'),
    ('initDebugLog', 824, 'src/realtime_pgm_node.cpp',
     '启动期一次性初始化: 建 debug_log_dir_ 目录, 打开 cloud_frames.csv 写表头',
     '读 debug_* 参数'),
    ('printConfig', 7471, 'src/realtime_pgm_node.cpp',
     '把全部生效配置打印到日志 (含话题名与 CAN 状态)',
     '依赖几乎所有参数成员; 拆分后建议改为 config 结构体的 to_string'),
    ('createMotionTimer', 3631, 'src/realtime_pgm_node.cpp',
     '按 motion_rate_hz_ 取消并重建 wall timer (频率热改)',
     '写 motion_timer_'),
    ('odomCallback', 1045, 'src/callbacks.cpp',
     '订阅 /Odometry: 解 yaw -> 位姿跳变检测并开启"拒绝窗口" -> 施加 map->odom 修正 -> 写车体位姿',
     '写 odom_* / pose_jump_*, 加 motion_mutex_'),
    ('planCallback', 1150, 'src/callbacks.cpp',
     '缓存 Nav2 全局路径点 + 本机接收时刻 (新鲜度监督)',
     '写 motion_plan_*'),
    ('localPlanCallback', 1179, 'src/callbacks.cpp',
     '缓存局部路径(少于 2 点丢弃), 接收时刻用本机时钟',
     '写 local_plan_*'),
    ('goalPoseCallback', 1205, 'src/callbacks.cpp',
     '存网页 A* 终点 (x,y), 清零到达校验重试计数',
     '写 goal_x_/goal_y_/motion_goal_check_*'),
    ('renderTimerCallback', 3363, 'src/callbacks.cpp',
     '渲染定时器: 消费 导出/导入/清史 请求 -> 快照点云与配置 -> 分派 renderPgm / renderHistoryPgm / renderDynamicObstaclePgm',
     '定时器回调'),
    ('motionTimerCallback', 3651, 'src/callbacks.cpp (流程段建议拆到 tasks/motion_pipeline)',
     '运动主状态机: 按钮请求 -> 追击接管 -> 占点接管 -> 取路径(局部优先) -> A* 兜底 -> 正常跟随 '
     '-> 到点校验重试 -> 发帧',
     '可拆为 chaseStep(3669-3722) / holdPointStep(3731-3788) / astarFallbackStep(3899-3954) / goalCheckStep(3978-4119)'),
    ('parameterCallback', 2580, 'src/parameter_callback.cpp',
     '参数热更新: 拷影子变量 -> 逐参数解析 -> 集中校验 -> 分锁提交 -> 按需重建定时器/静态基准 -> 汇总日志',
     '参数分组见 01_config.md'),
    ('openCanSocket', 3487, 'src/can_socket.cpp',
     'socket / ioctl(SIOCGIFINDEX) / bind 建 SocketCAN 链路, 失败打错并返回 false (下次重试)',
     '读 can_interface_/can_id_/can_send_period_ms_'),
    ('closeCanSocket', 3478, 'src/can_socket.cpp',
     '关闭 CAN fd 并置 -1',
     '写 can_socket_fd_'),
    ('canFrameFromMotion', 3540, 'src/can_socket.cpp',
     '按 can_send_period_ms_ 节流组 can_frame 直写总线; 失败关 socket 待重连, 成功累加计数',
     '读写 can_last_send_ / can_sent_frames_'),
    ('main', 7927, 'src/main.cpp',
     'rclcpp::init -> 构造 RealtimePgmNode -> spin -> shutdown',
     '唯一自由函数, 原样搬运'),
]

CONFIG_TYPES = [
    ('consts', 46, 'config/constants.hpp',
     '运动帧协议常量: kMotionSpeedScale(量化 200) / kMotionMaxSpeed(±2 m/s) / '
     'kSlowRadiusTime(减速时间常数) / kMotionRawLimit(12 位饱和 2040)',
     'block(46-55)'),
    ('Point', 57, 'config/types.hpp',
     '点云基本单元 float x/y/z (被 tools 与 tasks 共用, 建议放最底层头文件)',
     'type; 应保持无 ROS 依赖'),
    ('LookaheadView', 70, 'config/types.hpp',
     '路径跟随可视化快照: valid/base_x/base_y/yaw/target_x/target_y/vx/vy/lookahead/dist_end',
     'type'),
    ('Config', 84, 'config/config.hpp',
     '建图配置: resolution/zmin/zmax/dilate/baseline_dilate/self_filter_size/地图范围/输出路径/'
     '历史时长/每帧点数上限/投票阈值',
     'type; 纯 POD, 建议按主题拆成 MappingConfig 等'),
    ('TimedCloud', 109, 'config/types.hpp',
     '带时间戳的点云缓存(滑动窗元素): rclcpp::Time stamp + shared_ptr<const vector<Point>>',
     'type; 若要 tools 无 ROS 依赖, stamp 可改 int64_t 纳秒'),
    ('PoseCorrection', 1233, 'config/types.hpp',
     'map->odom 刚体变换: double r[9] 行主序 + double t[3] + bool valid',
     'type'),
    ('HoldObstacleCluster', 4717, 'config/types.hpp',
     '障碍簇世界系质心 x/y (占点与追击共用)',
     'type'),
    ('FootprintBounds', 6196, 'config/types.hpp',
     '车体自过滤 AABB: valid / xmin / xmax / ymin / ymax',
     'type'),
    ('ChaseTrackPoint', 7848, 'config/types.hpp',
     '追击目标轨迹条目: stamp / x / y / 该时刻簇点云',
     'type; 嵌在成员区声明'),
]

# 成员变量区分组: (小节标题, 起始行, 结束行)
MEMBER_GROUPS = [
    ('点云基础 / 渲染周期', 7570, 7576),
    ('调试日志 / 异常快照 / 跳变检测', 7578, 7611),
    ('参数与配置快照', 7613, 7615),
    ('订阅 / 定时器 / 单帧图发布', 7617, 7630),
    ('历史图缓冲', 7632, 7638),
    ('地图导出 / 导入', 7640, 7655),
    ('动态障碍 / 位姿修正', 7657, 7704),
    ('自过滤点云发布', 7706, 7715),
    ('清除射线', 7717, 7725),
    ('清空 Nav2 代价地图', 7727, 7735),
    ('运动帧框架 / 输入话题', 7737, 7749),
    ('路径跟随参数', 7751, 7775),
    ('A* 兜底参数', 7777, 7788),
    ('占点', 7790, 7816),
    ('追击', 7818, 7846),
    ('轨迹 + A* 缓存 + 目标点缓存', 7847, 7867),
    ('运动状态 / 里程计 / 路径缓冲', 7869, 7903),
    ('运动请求 + 可视化快照', 7905, 7911),
    ('SocketCAN 直发', 7913, 7921),
    ('参数回调句柄', 7923, 7924),
]

# 成员变量组 -> 新结构去向（config 只放 yaml，故参数进 yaml、状态进 io/tasks 类）
MEMBER_TARGET = {
    '点云基础 / 渲染周期': 'tasks/cloud_pipeline.cpp（cloud_mutex_ / latest_cloud_）；话题名进 params.yaml',
    '调试日志 / 异常快照 / 跳变检测': 'tasks/debug_log.cpp → DebugLogger（阈值键来自 params.yaml）',
    '参数与配置快照': '值 → config/params.yaml；结构 → tools/types.hpp；锁与快照 → src/ 节点',
    '订阅 / 定时器 / 单帧图发布': '订阅与定时器 → src/ 节点；Image 发布器 → io/ros_image_pub',
    '历史图缓冲': 'tasks/cloud_pipeline.cpp（history_ 队列）',
    '地图导出 / 导入': 'tasks/map_io.cpp → MapIo（atomic 请求位）；落盘 → io/file_*',
    '动态障碍 / 位姿修正': 'tasks/dynamic_split.cpp + tasks/pose_correction.cpp',
    '自过滤点云发布': 'io/ros_cloud_pub.cpp → RosCloudPublisher',
    '清除射线': 'io/ros_cloud_pub.cpp → RosCloudPublisher',
    '清空 Nav2 代价地图': 'io/ros_nav2_client.cpp；开关键 `clear_costmap_on_history_reset` 进 params.yaml',
    '运动帧框架 / 输入话题': '话题名与 scale → config/params.yaml；打包与下发 → io/ros_motion_pub.cpp',
    '路径跟随参数': 'config/params.yaml（motion_* 14 项）',
    'A* 兜底参数': 'config/params.yaml（astar_* 8 项）+ tasks/astar_planner.cpp（缓存）',
    '占点': 'config/params.yaml（hold_point_*）+ tasks/hold_point.cpp（状态）',
    '追击': 'config/params.yaml（chase_*）+ tasks/chase.cpp（状态与轨迹）',
    '轨迹 + A* 缓存 + 目标点缓存': 'tasks/chase.cpp（chase_track_）+ tasks/astar_planner.cpp（astar_cache_*）',
    '运动状态 / 里程计 / 路径缓冲': 'tasks/motion_control.cpp（状态）+ src/callbacks.cpp（写入）',
    '运动请求 + 可视化快照': 'tasks/motion_control.cpp（lookahead_view_ 的写入上移到此）',
    'SocketCAN 直发': 'io/can_bus.cpp → CanBus；`can_*` 键进 params.yaml',
    '参数回调句柄': 'src/parameter_callback.cpp',
}

# 参数名 -> 主题 (用于自动生成参数分组表)
PARAM_GROUPS = [
    ('基础建图 (写入 Config)', [
        'resolution', 'zmin', 'zmax', 'dilate', 'baseline_dilate', 'self_filter_size',
        'xmin', 'ymin', 'xmax', 'ymax', 'output', 'history_output', 'history_duration',
        'history_max_points_per_frame', 'history_min_hits', 'cloud_topic']),
    ('调试日志 / 异常快照', [
        'debug_log_enabled', 'debug_log_dir', 'debug_pcd_enabled', 'debug_pcd_max_files',
        'debug_cloud_count_jump_ratio', 'debug_cloud_count_jump_delta',
        'debug_centroid_jump_m', 'debug_odom_jump_m', 'debug_odom_jump_yaw_deg',
        'pose_jump_reject_sec', 'debug_occ_ratio_jump']),
    ('渲染 / 话题名', [
        'render_period_ms', 'clear_history', 'cloud_preview_topic', 'cloud_preview_max_points',
        'history_image_topic', 'image_topic']),
    ('地图导出 / 导入', [
        'export_map_dir', 'export_map_name', 'export_map_request',
        'import_map_dir', 'import_map_request']),
    ('动态障碍模式', [
        'dynamic_obstacle_enabled', 'dynamic_obstacle_topic', 'dynamic_obstacle_image_topic']),
    ('位姿校正 (map->odom)', [
        'pose_correction_topic', 'pose_correction_global_period_sec', 'pose_correction_global_range_m',
        'pose_correction_local_range_m', 'pose_correction_score_min', 'pose_correction_max_delta_m',
        'pose_correction_ema_alpha', 'pose_correction_score_gain',
        'pose_correction_global_trigger_fails', 'apply_pose_correction']),
    ('自过滤点云 (Nav2 clearing)', [
        'self_filtered_topic', 'self_filtered_publish_rate', 'self_filtered_max_points']),
    ('清除射线环', ['clear_ring_topic', 'clear_ring_points', 'clear_ring_radius']),
    ('清空 Nav2 代价图', [
        'clear_costmap_on_history_reset', 'clear_local_costmap_service', 'clear_global_costmap_service']),
    ('运动帧 / 输入话题', [
        'motion_topic', 'odom_topic', 'global_plan_topic', 'local_plan_topic',
        'local_plan_timeout', 'motion_control', 'motion_vx_scale', 'motion_vy_scale', 'motion_rate_hz']),
    ('路径跟随', [
        'motion_target_speed', 'motion_lookahead', 'motion_lookahead_speed_gain',
        'motion_lookahead_max', 'motion_yaw_kp', 'motion_goal_tolerance',
        'motion_goal_check_tolerance', 'motion_goal_check_max_retry', 'motion_max_lateral',
        'motion_plan_timeout', 'motion_slow_radius', 'motion_min_speed',
        'motion_base_to_lidar_x', 'motion_base_to_lidar_y']),
    ('A* 兜底', [
        'astar_enabled', 'astar_speed', 'astar_inflate_radius', 'astar_obstacle_penalty',
        'astar_cost_reach', 'astar_path_spacing', 'astar_replan_interval',
        'astar_replan_move_dist', 'goal_pose_topic', 'astar_plan_topic']),
    ('网页一次性触发', ['start_motion', 'stop_motion']),
    ('占点', [
        'hold_point_enabled', 'hold_point_x', 'hold_point_y', 'hold_point_radius',
        'hold_point_speed', 'hold_point_slow_radius', 'hold_point_standoff_scale',
        'hold_point_recenter_deadband', 'hold_point_status_topic', 'hold_point_viz_topic']),
    ('追击', [
        'chase_enabled', 'chase_pick_x', 'chase_pick_y', 'chase_radius', 'chase_deadband',
        'chase_speed', 'chase_lock_radius', 'chase_track_radius', 'chase_track_history_sec',
        'chase_track_record_interval', 'chase_status_topic', 'chase_viz_topic', 'chase_track_topic']),
    ('SocketCAN 直发', ['can_enabled', 'can_interface', 'can_id', 'can_send_period_ms']),
]


# ---------------------------------------------------------------------------
# 目标路径覆写
#   约定: config/ 只放 yaml(调参用), 因此所有"数据类型/常量/文件 IO/ROS 收发"条目
#         一律改判到 tools/(纯计算) 或 io/(硬件与外部交互)。
# ---------------------------------------------------------------------------
OVERRIDE = {
    # --- 类型与常量: config/ -> tools/ (TimedCloud 带 rclcpp::Time, 归 io) ---
    'consts': 'tools/constants.hpp',
    'Point': 'tools/types.hpp',
    'LookaheadView': 'tools/types.hpp',
    'Config': 'tools/types.hpp',
    'PoseCorrection': 'tools/types.hpp',
    'HoldObstacleCluster': 'tools/types.hpp',
    'FootprintBounds': 'tools/types.hpp',
    'ChaseTrackPoint': 'tools/types.hpp',
    'TimedCloud': 'io/timed_cloud.hpp',
    # --- 文件序列化: tools/ -> io/ ---
    'saveCloudPcd': 'io/file_pcd.cpp',
    'writeHistoryCloudsBin': 'io/file_bin.cpp',
    'readHistoryCloudsBin': 'io/file_bin.cpp',
    'writeMapYaml': 'io/file_yaml.cpp',
    # --- ROS 发布: tasks/ -> io/ ---
    'publishCloudPreview': 'io/ros_cloud_pub.cpp',
    'publishPointCloud': 'io/ros_cloud_pub.cpp',
    'publishSelfFilteredCloud': 'io/ros_cloud_pub.cpp',
    'publishDynamicObstacleCloud': 'io/ros_cloud_pub.cpp',
    'publishClearRingCloud': 'io/ros_cloud_pub.cpp',
    'publishAstarPlan': 'io/ros_path_pub.cpp',
    'publishHoldPointViz': 'io/ros_path_pub.cpp',
    'publishHoldPointStatus': 'io/ros_path_pub.cpp',
    'publishChaseViz': 'io/ros_path_pub.cpp',
    'publishChaseStatus': 'io/ros_path_pub.cpp',
    'publishChaseTrack': 'io/ros_path_pub.cpp',
    'publishMotionFrame': 'io/ros_motion_pub.cpp',
    'publishPoseCorrection': 'io/ros_pose_pub.cpp',
    'pointCloudStamp': 'io/ros_stamp.cpp',
    'clearCostmaps': 'io/ros_nav2_client.cpp',
    # --- CAN / 调试文件: src/ 或 tasks/ -> io/ ---
    'openCanSocket': 'io/can_bus.cpp',
    'closeCanSocket': 'io/can_bus.cpp',
    'canFrameFromMotion': 'io/can_bus.cpp',
    'initDebugLog': 'io/debug_log_file.cpp',
}

PARAM_BLOCK = ('参数声明区 (constructor 118-604)', 118, 'config/params.yaml',
               '构造函数里的全部 declare_parameter: 参数名/类型/默认值的唯一来源',
               'block(118-604)')


def _apply(items):
    return [(n, s, OVERRIDE.get(n, t), p, nt) for (n, s, t, p, nt) in items]


CONFIG_TYPES = _apply(CONFIG_TYPES)
TOOLS = _apply(TOOLS)
TASKS = _apply(TASKS)
NODE = _apply(NODE)
ALL = CONFIG_TYPES + TOOLS + TASKS + NODE


def layer_of(target):
    """目标路径的首段即所属层: config / tools / io / tasks / src。"""
    return target.split('/')[0]


def by_layer(items, layer):
    return [i for i in items if layer_of(i[2]) == layer]


def pick(items, names):
    sel = set(names)
    return [i for i in items if i[0] in sel]


# 层 -> 文档文件名
LAYER_DOC = {
    'config': '01_config.md',
    'tools': '02_tools.md',
    'io': '03_io.md',
    'tasks': '04_tasks.md',
    'src': '05_node.md',
}


def read_slice(lines, start, end):
    return lines[start - 1:end]


# 按目标文件分组: (文件, 建议类/命名空间, [符号名])
FILE_GROUPS = [
    # ---------------- config/: 只有 yaml ----------------
    ('config/params.yaml', '参数默认值（由 declare_parameter 提炼）',
     ['参数声明区 (constructor 118-604)']),
    ('config/nav2_params.yaml', 'Nav2 参数（人工维护，无源码条目）', []),

    # ---------------- tools/: 纯计算 ----------------
    ('tools/types.hpp', 'namespace nav::tools', [
        'Point', 'LookaheadView', 'Config', 'PoseCorrection',
        'HoldObstacleCluster', 'FootprintBounds', 'ChaseTrackPoint']),
    ('tools/constants.hpp', 'namespace nav::tools', ['consts']),
    ('tools/ros_utils.hpp', 'namespace nav::tools', ['hasField']),
    ('tools/tf_math.hpp', 'namespace nav::tools (inline)', [
        'quaternionToMatrix', 'matrixToQuaternion', 'yawFromMatrix', 'wrapAngle',
        'multiplyMatrix', 'transformPointByCorrection']),
    ('tools/tf_math.cpp', 'namespace nav::tools', ['transformToBaseLink']),
    ('tools/registration.cpp', 'namespace nav::tools', ['score_at']),
    ('tools/motion_codec.hpp', 'namespace nav::tools', [
        'limitMotionSpeed', 'effectiveSlowRadius', 'encodeMotionRaw', 'resolveAstarSpeed']),
    ('tools/grid.cpp', 'namespace nav::tools', [
        'clearVehicleFootprint', 'accumulateCloud', 'applyDilate', 'occupancyToPixels']),
    ('tools/grid_geom.cpp', 'namespace nav::tools', [
        'computeMapSize', 'computeVehicleFootprint', 'clipCloudToMapBounds',
        'filterVehicleFootprintPoints', 'computeHoldBlockPose']),
    ('tools/grid_cluster.cpp', 'namespace nav::tools', [
        'extractHoldObstacles', 'findNearestObstacleCluster']),
    ('tools/grid_clearance.cpp', 'namespace nav::tools', [
        'isHoldPointClear', 'isHoldPointClearPath', 'chaseLineClear']),

    # ---------------- io/: 硬件与外部交互 ----------------
    ('io/timed_cloud.hpp', 'namespace nav::io（带 rclcpp::Time，故不放 tools）', ['TimedCloud']),
    ('io/can_bus.cpp', 'class CanBus', [
        'openCanSocket', 'closeCanSocket', 'canFrameFromMotion']),
    ('io/ros_cloud_pub.cpp', 'class RosCloudPublisher', [
        'publishCloudPreview', 'publishPointCloud', 'publishSelfFilteredCloud',
        'publishDynamicObstacleCloud', 'publishClearRingCloud']),
    ('io/ros_path_pub.cpp', 'class RosPathPublisher', [
        'publishAstarPlan', 'publishHoldPointViz', 'publishHoldPointStatus',
        'publishChaseViz', 'publishChaseStatus', 'publishChaseTrack']),
    ('io/ros_motion_pub.cpp', 'class RosMotionPublisher', ['publishMotionFrame']),
    ('io/ros_pose_pub.cpp', 'class RosPosePublisher', ['publishPoseCorrection']),
    ('io/ros_stamp.cpp', 'namespace nav::io', ['pointCloudStamp']),
    ('io/ros_nav2_client.cpp', 'class Nav2ServiceClient', ['clearCostmaps']),
    ('io/file_pcd.cpp', 'namespace nav::io', ['saveCloudPcd']),
    ('io/file_bin.cpp', 'namespace nav::io', ['writeHistoryCloudsBin', 'readHistoryCloudsBin']),
    ('io/file_yaml.cpp', 'namespace nav::io', ['writeMapYaml']),
    ('io/debug_log_file.cpp', 'class DebugLogFile', ['initDebugLog']),

    # ---------------- tasks/: 业务流程 ----------------
    ('tasks/cloud_pipeline.cpp', 'class CloudPipeline', ['cloudCallback', 'pruneHistory']),
    ('tasks/dynamic_split.cpp', 'class DynamicSplitter', [
        'splitDynamicObstacles', 'subtractBaseline', 'buildStaticBaseline']),
    ('tasks/pose_correction.cpp', 'class PoseCorrector', [
        'getPoseCorrection', 'setPoseCorrection2D', 'resetPoseCorrection',
        'estimatePoseCorrection', 'correctCloudForPose']),
    ('tasks/map_render.cpp', 'class MapRenderer', [
        'renderPgm', 'renderHistoryPgm', 'renderDynamicObstaclePgm']),
    ('tasks/map_io.cpp', 'class MapIo', [
        'exportHistoryMap', 'importHistoryMap', 'mergeImportedMap']),
    ('tasks/astar_planner.cpp', 'class AstarPlanner', [
        'buildMergedOccupancy', 'planAstarPath', 'planAstarToGoal', 'getAstarPath']),
    ('tasks/hold_point.cpp', 'class HoldPointTask', ['computeHoldPointCommand']),
    ('tasks/chase.cpp', 'class ChaseTask', ['computeChaseCommand', 'recordTargetCloud']),
    ('tasks/motion_control.cpp', 'class MotionController', [
        'computeFollowCommand', 'computeAstarFollowCommand', 'startMotion', 'stopMotion']),
    ('tasks/debug_log.cpp', 'class DebugLogger', ['logCloudFrame']),

    # ---------------- src/: 装配与回调 ----------------
    ('src/main.cpp', 'int main()', ['main']),
    ('src/realtime_pgm_node.cpp', 'class RealtimePgmNode', [
        'constructor_assembly', 'printConfig', 'createMotionTimer']),
    ('src/callbacks.cpp', 'class RealtimePgmNode (回调段)', [
        'odomCallback', 'planCallback', 'localPlanCallback', 'goalPoseCallback',
        'renderTimerCallback', 'motionTimerCallback']),
    ('src/parameter_callback.cpp', 'class RealtimePgmNode (参数段)', ['parameterCallback']),
]


def item_end(lines, name, note, start):
    """求某条目的结束行: block(x-y) 显式; score_at 是 lambda; 其余靠括号配对。"""
    if note and note.startswith('block('):
        return int(note.split('(')[1].split(')')[0].split('-')[1])
    if name == 'score_at':
        return 1505
    return find_end_line(lines, start)


def emit_item(out, lines, name, start, target, purpose, note, kind='func', end=None):
    real_end = end if end is not None else find_end_line(lines, start)
    body = read_slice(lines, start, real_end)
    out.append('### `%s`' % name)
    out.append('')
    out.append('- **源码行**: `%d-%d` (%d 行)' % (start, real_end, real_end - start + 1))
    out.append('- **目标位置**: `%s`' % target)
    out.append('- **职责**: %s' % purpose)
    if note:
        out.append('- **依赖 / 备注**: %s' % note)
    out.append('')
    out.append('```cpp')
    out.extend(body)
    out.append('```')
    out.append('')


def section(out, title, desc=''):
    out.append('## %s' % title)
    out.append('')
    if desc:
        out.append(desc)
        out.append('')


def build_doc(lines, title, intro, sections, params_table=False):
    """sections: [(小节标题, 小节说明, [(item kwargs dict)])]"""
    out = ['# %s' % title, '', intro, '']
    if params_table:
        out.extend(build_param_table(lines))
    for sec_title, sec_desc, items in sections:
        section(out, sec_title, sec_desc)
        for it in items:
            emit_item(out, lines, **it)
    return out


def scan_params(lines):
    """扫描 118-604 行的 declare_parameter, 返回 参数名 -> (类型, 默认值, 行号)。

    参数声明经常跨行书写（名字与默认值在下一行），因此先把该区间拼成一个字符串
    整体匹配，行号用换行计数还原。
    """
    text = '\n'.join(lines[117:604])
    pat = re.compile(
        r'declare_parameter<((?:[^<>]|<[^<>]*>)+)>\s*\(\s*"([A-Za-z0-9_]+)"\s*(?:,\s*([^;)]*))?\)')
    seen = {}
    for m in pat.finditer(text):
        name = m.group(2)
        typ = ' '.join(m.group(1).split())
        default = ' '.join((m.group(3) or '').split())
        line_no = text.count('\n', 0, m.start(2)) + 118
        seen[name] = (typ, default if default else '—', line_no)
    return seen


def build_yaml_skeleton(lines):
    """按 PARAM_GROUPS 生成 params.yaml 骨架（值 = 源码默认值）。"""
    seen = scan_params(lines)
    out = ['```yaml', '# Navigation 可调参数（由 test_navigation.cpp 的 declare_parameter 提炼）',
           '# 用法: 由 src/parameter_callback.cpp 通过 declare_parameter 默认值覆盖读取']
    mapped = set()
    for group, names in PARAM_GROUPS:
        keys = [n for n in names if n in seen]
        if not keys:
            continue
        out.append('')
        out.append('# ==================== %s ====================' % group)
        for n in keys:
            typ, dft, _ = seen[n]
            out.append('%-38s # %s, 默认 %s' % ('%s: %s' % (n, dft), typ, dft))
            mapped.add(n)
    rest = sorted(set(seen) - mapped)
    if rest:
        out.append('')
        out.append('# ==================== 其他 ====================')
        for n in rest:
            typ, dft, _ = seen[n]
            out.append('%-38s # %s' % ('%s: %s' % (n, dft), typ))
    out.append('```')
    out.append('')
    return out


NAV2_YAML_CANDIDATES = [
    os.path.normpath(os.path.join(HERE, '..', '..', 'config', 'nav2_params.yaml')),
    os.path.normpath(os.path.join(HERE, '..', '..', 'test', 'test_navigation', 'config',
                                  'nav2_params.yaml')),
]


def find_nav2_yaml():
    for p in NAV2_YAML_CANDIDATES:
        if os.path.exists(p):
            return p
    return None


def _norm(v):
    """去掉默认值两侧的引号, 便于与 yaml 里的裸标量比较。"""
    v = v.strip()
    if len(v) >= 2 and v[0] == v[-1] and v[0] in ('"', "'"):
        v = v[1:-1]
    return v


def _grab(yaml_lines, pattern, limit=8):
    """按正则抓 yaml 里的标量, 返回 [(行号, 值)]。"""
    hits = []
    for i, l in enumerate(yaml_lines):
        m = re.match(pattern, l)
        if m:
            hits.append((i + 1, m.group(1).strip()))
    return hits[:limit]


def _fmt_hits(hits):
    if not hits:
        return '未设置（用 Nav2 默认值）'
    by_val = {}
    for ln, v in hits:
        by_val.setdefault(v, []).append(ln)
    return ' / '.join('`%s`(行%s)' % (v, ','.join(str(x) for x in lns))
                      for v, lns in by_val.items())


def build_nav2_section(cpp_lines):
    path = find_nav2_yaml()
    out = ['## nav2_params.yaml 对照（Nav2 侧参数，需与本节点对齐）', '']
    if not path:
        out.append('未找到 `nav2_params.yaml`（候选路径见脚本里的 `NAV2_YAML_CANDIDATES`）。')
        out.append('')
        return out
    with open(path, 'r', encoding='utf-8') as f:
        yl = f.read().split('\n')
    rel = os.path.relpath(path, os.path.normpath(os.path.join(HERE, '..', '..')))
    out.append('本节点**不读取**这个文件：它由 Nav2 的 9 个节点段各自 `declare_parameter` 消费。'
               '但它有 7 组值与源码重复设置，**改一处必须同步另一处**，否则 Nav2 与本节点的安全距离/话题会对不上。')
    out.append('')
    out.append('- 数据来源（自动读取）：`%s`' % rel)
    out.append('- 注入链：launch 的 `nav2_params_file` → `nav2_bringup/navigation_launch.py`；'
               '行为树由 launch 的 `bt_xml_file` 覆写 `default_nav_to_pose_bt_xml`')
    out.append('')

    node = scan_params(cpp_lines)

    def nv(key):
        typ, dft, ln = node.get(key, ('-', '未声明', '-'))
        return typ, _norm(dft), ln

    rr = _grab(yl, r'^\s*robot_radius:\s*(\S+)')
    ir = _grab(yl, r'^\s*inflation_radius:\s*(\S+)')
    cf = _grab(yl, r'^\s*cost_scaling_factor:\s*(\S+)')
    od = _grab(yl, r'^\s*odom_topic:\s*(\S+)')
    tp = _grab(yl, r'^\s*topic:\s*(\S+)')
    bt = _grab(yl, r'^\s*default_nav_to_pose_bt_xml:\s*(\S+)')

    inflate = nv('astar_inflate_radius')
    reach = nv('astar_cost_reach')
    odom = nv('odom_topic')

    out.append('| 项目 | `nav2_params.yaml` 中的值 | 本节点对应参数（源码默认值） | 约束 |')
    out.append('|---|---|---|---|')
    out.append('| `robot_radius` | %s | `astar_inflate_radius` = `%s` | **必须相等**：同为“车体中心→障碍”的硬半径 |'
               % (_fmt_hits(rr), inflate[1]))
    out.append('| `inflation_radius` | %s | `astar_cost_reach` = `%s` | **必须相等**：软性避让半径 |'
               % (_fmt_hits(ir), reach[1]))
    out.append('| `cost_scaling_factor` | %s | 无对应 | Nav2 独有：代价衰减陡缓，只影响 Nav2 自身避障 |'
               % _fmt_hits(cf))
    out.append('| `odom_topic` | %s | `odom_topic` = `%s` | 话题名必须一致 |'
               % (_fmt_hits(od), odom[1]))
    out.append('| 观测源 `topic` | %s | `self_filtered_topic` = `%s`；`dynamic_obstacle_topic` = `%s`；'
               '`clear_ring_topic` = `%s` | 三个点云话题必须与 yaml 里的观测源一致，否则清除射线不生效 |'
               % (_fmt_hits(tp), nv('self_filtered_topic')[1],
                  nv('dynamic_obstacle_topic')[1], nv('clear_ring_topic')[1]))
    out.append('| 清除代价图服务 | %s | `clear_local_costmap_service` = `%s`；`clear_global_costmap_service` = `%s` | '
               'yaml 不配这两项，用 Nav2 默认服务名，源码里必须同名 |'
               % ('yaml 未出现（Nav2 默认）',
                  nv('clear_local_costmap_service')[1], nv('clear_global_costmap_service')[1]))
    out.append('| `default_nav_to_pose_bt_xml` | %s | 无对应 | yaml 里写相对路径会被 launch 用绝对路径覆写 |'
               % _fmt_hits(bt))
    out.append('')

    # 一致性检查
    warns = []
    yaml_vals = {v for _, v in rr}
    if yaml_vals and inflate[1] not in ('未声明', '-') and yaml_vals != {inflate[1]}:
        warns.append('`robot_radius`(%s) 与 `astar_inflate_radius`(%s) 不一致'
                     % ('/'.join(sorted(yaml_vals)), inflate[1]))
    yaml_vals = {v for _, v in ir}
    if yaml_vals and reach[1] not in ('未声明', '-') and yaml_vals != {reach[1]}:
        warns.append('`inflation_radius`(%s) 与 `astar_cost_reach`(%s) 不一致'
                     % ('/'.join(sorted(yaml_vals)), reach[1]))
    yaml_vals = {v for _, v in od}
    if yaml_vals and odom[1] not in ('未声明', '-') and yaml_vals != {odom[1]}:
        warns.append('`odom_topic`(%s) 与源码 `odom_topic`(%s) 不一致'
                     % ('/'.join(sorted(yaml_vals)), odom[1]))
    yaml_topics = {v for _, v in tp}
    for key in ('self_filtered_topic', 'dynamic_obstacle_topic', 'clear_ring_topic'):
        val = nv(key)[1]
        if yaml_topics and val not in ('未声明', '-') and val not in yaml_topics:
            warns.append('源码 `%s`(%s) 未在 yaml 观测源中出现' % (key, val))

    if warns:
        out.append('**当前检查结果：发现 %d 处不一致**' % len(warns))
        out.append('')
        for w in warns:
            out.append('- %s' % w)
    else:
        out.append('**当前检查结果：以上对应项全部一致。**')
    out.append('')
    out.append('> 其余参数（插件列表 `plugin_lib_names`、costmap 分辨率与更新频率、controller/planner 插件、'
               'behavior 超时、velocity_smoother 等）属于 Nav2 自身行为，改它们**不需要动 C++**，'
               '故不在此文档展开，直接看原 yaml。')
    out.append('')
    return out


def build_param_table(lines, heading='## 参数清单（自动扫描 118-604 行的 declare_parameter）'):
    """扫描参数声明区, 生成 参数名 -> 主题 的表格（含类型与默认值，可直接抄成 yaml）。"""
    out = [heading, '']
    seen = scan_params(lines)
    out.append('共 **%d** 个参数（这是 `config/params.yaml` 的全部键）。' % len(seen))
    out.append('')
    out.append('| 主题 | 参数 | 类型 | 默认值 | 声明行 |')
    out.append('|---|---|---|---|---|')
    mapped = set()
    for group, names in PARAM_GROUPS:
        for n in names:
            if n in seen:
                typ, dft, ln = seen[n]
                out.append('| %s | `%s` | `%s` | `%s` | %d |' % (group, n, typ, dft, ln))
                mapped.add(n)
    for n in sorted(set(seen) - mapped):
        typ, dft, ln = seen[n]
        out.append('| (未归类, 待补充) | `%s` | `%s` | `%s` | %d |' % (n, typ, dft, ln))
    out.append('')
    return out


def build_by_file(lines):
    """按"目标文件"汇总: 每个文件一节, 先列符号表, 再给合并后的完整源码。"""
    lookup = {}
    for item in ALL:
        n, s, t, p, nt = item
        if n in ('consts', '参数声明区 (constructor 118-604)', 'constructor_assembly'):
            kind = '代码块'
        elif n in [i[0] for i in CONFIG_TYPES]:
            kind = 'struct'
        else:
            kind = '函数'
        lookup[n] = dict(name=n, start=s, target=t, purpose=p, note=nt,
                         doc=LAYER_DOC.get(layer_of(t), 'INDEX.md'), kind=kind)
    n, s, t, p, nt = PARAM_BLOCK
    lookup[n] = dict(name='参数声明区', start=s, target=t, purpose=p, note=nt,
                     doc='01_config.md', kind='代码块')

    out = ['# 按目标文件汇总（BY_FILE）', '',
           '同一份拆分方案的"文件视角"：每个目标文件一节，先列该文件要装哪些符号，'
           '再给出**合并后的完整源码**（各段前有 `// ===== 源码行 a-b: 符号名 =====` 分隔注释，便于对照原文件）。', '',
           '层级顺序：`config/`（只有 yaml） → `tools/`（纯计算） → `io/`（硬件与 ROS/文件交互） → '
           '`tasks/`（业务流程） → `src/`（装配与回调），依赖只允许自上而下。', '',
           '> 注意：合并后的代码是**搬运视图**，不是可直接编译的完整文件——'
           '类方法的声明仍需在对应 `.hpp` 里按类组织，`private:` 成员、`#include`、'
           '`namespace` 包裹都要你在新建文件时补齐。成员变量的分配见文末附录。', '',
           '对照文档：`01_config.md` / `02_tools.md` / `03_io.md` / `04_tasks.md` / `05_node.md`，索引见 `INDEX.md`。', '']

    # 总览表
    out.append('## 文件总览')
    out.append('')
    out.append('| 目标文件 | 建议类 / 命名空间 | 符号数 | 源码行合计 |')
    out.append('|---|---|---|---|')
    for fname, hint, names in FILE_GROUPS:
        total = 0
        for n in names:
            it = lookup[n]
            e = item_end(lines, n, it['note'], it['start'])
            total += e - it['start'] + 1
        out.append('| `%s` | %s | %d | %d |' % (fname, hint, len(names), total))
    out.append('')

    # 逐文件
    for fname, hint, names in FILE_GROUPS:
        out.append('## `%s`' % fname)
        out.append('')
        out.append('建议落点：%s' % hint)
        out.append('')
        if not names:
            out.append('人工维护的参数文件，没有对应源码条目。')
            out.append('')
            continue
        out.append('| 符号 | 源码行 | 类型 | 职责 |')
        out.append('|---|---|---|---|')
        spans = []
        for n in names:
            it = lookup[n]
            e = item_end(lines, n, it['note'], it['start'])
            spans.append((it['start'], e, n))
            out.append('| `%s` | %d-%d | %s | %s |' %
                       (n, it['start'], e, it['kind'], it['purpose']))
        out.append('')
        out.append('```cpp')
        for start, end, n in spans:
            out.append('// ===================== 源码行 %d-%d · %s =====================' % (start, end, n))
            out.extend(read_slice(lines, start, end))
            out.append('')
        out.append('```')
        out.append('')

    # 附录
    out.append('## 附录：成员变量区（20 组）的分配参考')
    out.append('')
    out.append('`config/` 只放 yaml，所以成员变量分三种去向：① **可调参数** → `config/params.yaml` 的键；'
               '② **纯数据/结构** → `tools/types.hpp`；③ **运行期状态**（mutex / 队列 / 缓存 / 发布器）'
               '→ 对应 `io/` 或 `tasks/` 类的私有成员。逐组源码见 `05_node.md` 的附录。')
    out.append('')
    out.append('| 成员变量组 | 源码行 | 建议落点 |')
    out.append('|---|---|---|')
    for title, s, e in MEMBER_GROUPS:
        out.append('| %s | %d-%d | %s |' % (title, s, e, MEMBER_TARGET.get(title, '待定')))
    out.append('')
    return out


def main():
    if not os.path.exists(SRC):
        sys.exit('找不到源文件: %s' % SRC)
    with open(SRC, 'r', encoding='utf-8') as f:
        lines = f.read().split('\n')
    print('源文件: %s (%d 行)' % (SRC, len(lines) - 1))

    def mk(items):
        res = []
        for item in items:
            name, start, target, purpose, note = item[:5]
            kw = dict(name=name, start=start, target=target, purpose=purpose, note=note)
            if note.startswith('block('):
                kw['end'] = int(note.split('(')[1].split(')')[0].split('-')[1])
            res.append(kw)
        return res

    # ===================== 01_config.md：只放 yaml =====================
    cfg = [
        '# 01 参数配置（config/，只有 yaml）', '',
        '**约定**：`Navigation/config/` 只放 yaml，专用于调参；不放头文件、不放源码、不放数据结构。', '',
        '| 文件 | 内容 |', '|---|---|',
        '| `config/params.yaml` | 本节点全部可调参数（下表即其全部键） |',
        '| `config/nav2_params.yaml` | Nav2 栈参数（人工维护） |', '',
        'Nav2 行为树 XML 不是 yaml：`launch/behavior_trees/navigate_to_pose_w_replanning_5hz.xml`，'
        'launch 用 `RewrittenYaml` 注入绝对路径。', '',
        '**数据类型与常量不在 config**：`Point` / `Config` / `LookaheadView` / `PoseCorrection` / '
        '`HoldObstacleCluster` / `FootprintBounds` / `ChaseTrackPoint` → `tools/types.hpp`；'
        '运动帧协议常量 → `tools/constants.hpp`；带 `rclcpp::Time` 的 `TimedCloud` → `io/timed_cloud.hpp`。'
        '源码见 `02_tools.md` 第 1 节、`03_io.md` 第 7 节。', '']
    cfg.extend(build_param_table(lines))
    cfg.append('## params.yaml 骨架')
    cfg.append('')
    cfg.append('键名与默认值均由源码自动提取，可直接落成 `config/params.yaml`。')
    cfg.append('')
    cfg.extend(build_yaml_skeleton(lines))
    cfg.extend(build_nav2_section(lines))
    cfg.append('## 附录：成员变量（20 组）去向')
    cfg.append('')
    cfg.append('`config/` 只放 yaml，成员变量分三种去向：① 可调参数 → `params.yaml` 的键；'
               '② 纯数据结构 → `tools/types.hpp`；③ 运行期状态（mutex/队列/缓存/发布器）→ '
               '`io/` 或 `tasks/` 类的私有成员。逐组源码见 `05_node.md` 附录。')
    cfg.append('')
    cfg.append('| 成员变量组 | 源码行 | 去向 |')
    cfg.append('|---|---|---|')
    for title, s, e in MEMBER_GROUPS:
        cfg.append('| %s | %d-%d | %s |' % (title, s, e, MEMBER_TARGET.get(title, '待定')))
    cfg.append('')
    write('01_config.md', cfg)

    # ===================== 02_tools.md：纯计算 =====================
    tools_pool = by_layer(TOOLS + CONFIG_TYPES, 'tools')
    tool_secs = [
        ('1. 数据类型与协议常量', '结构体与 constexpr 从节点类里搬出，成为 tools 的公共类型；本层零 ROS、零 IO。',
         mk(pick(tools_pool, ('Point', 'LookaheadView', 'Config', 'PoseCorrection',
                              'HoldObstacleCluster', 'FootprintBounds', 'ChaseTrackPoint', 'consts')))),
        ('2. 几何 / 坐标变换', '全部只依赖入参；建议 `namespace nav::tools`。',
         mk(pick(tools_pool, ('hasField', 'quaternionToMatrix', 'matrixToQuaternion', 'yawFromMatrix',
                              'wrapAngle', 'multiplyMatrix', 'transformPointByCorrection',
                              'score_at', 'transformToBaseLink')))),
        ('3. 运动帧编解码（12 位有符号量协议）', '纯位运算与量化，可单独做单元测试。',
         mk(pick(tools_pool, ('limitMotionSpeed', 'effectiveSlowRadius', 'encodeMotionRaw', 'resolveAstarSpeed')))),
        ('4. 栅格几何与像素', '占据栅格/像素层面的纯运算，拆分收益最大的一块。',
         mk(pick(tools_pool, ('computeMapSize', 'computeVehicleFootprint', 'clipCloudToMapBounds',
                              'filterVehicleFootprintPoints', 'clearVehicleFootprint', 'accumulateCloud',
                              'applyDilate', 'occupancyToPixels')))),
        ('5. 栅格查询 / 聚类 / 间距判定', '占点与追击共用，属通用几何算法。',
         mk(pick(tools_pool, ('extractHoldObstacles', 'computeHoldBlockPose', 'isHoldPointClear',
                              'isHoldPointClearPath', 'findNearestObstacleCluster', 'chaseLineClear')))),
    ]
    tools_doc = build_doc(
        lines,
        '02 纯计算（tools/）',
        '**约定**：`tools/` 只做数学、几何、编解码 —— 不 include ROS、不碰 socket/文件、不加锁。\n'
        '迁入顶层 `Navigation/tools/`，统一 `namespace nav::tools`，编译成 `nav_tools` 静态库。\n\n'
        '> 文件读写（PCD/BIN/YAML）已划到 `io/file_*`；ROS 发布全部在 `io/ros_*`。',
        tool_secs)
    write('02_tools.md', tools_doc)

    # ===================== 03_io.md：硬件与外部交互 =====================
    io_pool = by_layer(CONFIG_TYPES + TOOLS + TASKS + NODE, 'io')
    io_secs = [
        ('1. ROS 点云发布', '抽稀/限频/坐标变换后的发布动作，对上只暴露 `publish(points, frame_id)`。',
         mk(pick(io_pool, ('publishCloudPreview', 'publishPointCloud', 'publishSelfFilteredCloud',
                           'publishDynamicObstacleCloud', 'publishClearRingCloud')))),
        ('2. ROS 路径 / 状态 / 轨迹发布', '纯消息打包，不含业务判断。',
         mk(pick(io_pool, ('publishAstarPlan', 'publishHoldPointViz', 'publishHoldPointStatus',
                           'publishChaseViz', 'publishChaseStatus', 'publishChaseTrack')))),
        ('3. ROS 运动帧与位姿修正下发', '运动帧打包（8 字节协议）+ 位姿修正量发布。',
         mk(pick(io_pool, ('publishMotionFrame', 'publishPoseCorrection')))),
        ('4. 时间戳与服务客户端', 'ROS 时间戳对齐 + Nav2 ClearEntireCostmap 调用。',
         mk(pick(io_pool, ('pointCloudStamp', 'clearCostmaps')))),
        ('5. SocketCAN 直发', '系统调用层：建链/关链/限频发送，业务只给 (vx, vy, control)。',
         mk(pick(io_pool, ('openCanSocket', 'closeCanSocket', 'canFrameFromMotion')))),
        ('6. 文件 IO', 'PCD 快照、历史点云二进制、map YAML、调试日志初始化。',
         mk(pick(io_pool, ('saveCloudPcd', 'writeHistoryCloudsBin', 'readHistoryCloudsBin',
                           'writeMapYaml', 'initDebugLog')))),
        ('7. 带 ROS 类型的容器', '`TimedCloud` 含 `rclcpp::Time`，放 io 才能让 tools 彻底无 ROS 依赖。',
         mk(pick(io_pool, ('TimedCloud',)))),
    ]
    io_doc = build_doc(
        lines,
        '03 硬件与外部交互（io/）',
        '**约定**：`io/` 是唯一接触外部世界的地方 —— ROS 话题/服务、SocketCAN、文件系统。'
        '它依赖 `tools/`，但 `tools/` 绝不反向依赖它。\n'
        '迁入顶层 `Navigation/io/`，编译成 `nav_io` 静态库（需要 rclcpp 与消息依赖）。',
        io_secs)
    write('03_io.md', io_doc)

    # ===================== 04_tasks.md：业务流程 =====================
    task_pool = by_layer(TASKS, 'tasks')
    task_secs = [
        ('1. 点云流水线 / 动态障碍分离', 'cloudCallback 是全文件最强耦合点（4 把锁），建议先抽 tools/io 再搬它。',
         mk(pick(task_pool, ('cloudCallback', 'pruneHistory', 'logCloudFrame',
                             'splitDynamicObstacles', 'subtractBaseline', 'buildStaticBaseline')))),
        ('2. 位姿修正（map->odom 2D 配准）', '流程与状态在 tasks；评分内核在 tools/registration，发布在 io/ros_pose_pub。',
         mk(pick(task_pool, ('getPoseCorrection', 'setPoseCorrection2D', 'resetPoseCorrection',
                             'estimatePoseCorrection', 'correctCloudForPose')))),
        ('3. 建图渲染', '渲染编排（栅格运算在 tools，写 PGM 在 io）。',
         mk(pick(task_pool, ('renderPgm', 'renderHistoryPgm', 'renderDynamicObstaclePgm')))),
        ('4. 地图导入导出', '流程编排 + 状态重建（字节读写已下沉 io/file_*）。',
         mk(pick(task_pool, ('exportHistoryMap', 'importHistoryMap', 'mergeImportedMap')))),
        ('5. A* 规划', '缓存/限频/目标解析留 tasks，搜索内核建议下沉 tools/astar。',
         mk(pick(task_pool, ('buildMergedOccupancy', 'planAstarPath', 'planAstarToGoal', 'getAstarPath')))),
        ('6. 占点行为', '业务状态机 + 复用 tools 的聚类/间距判定。',
         mk(pick(task_pool, ('computeHoldPointCommand',)))),
        ('7. 追击行为', '业务状态机 + 目标轨迹记录。',
         mk(pick(task_pool, ('computeChaseCommand', 'recordTargetCloud')))),
        ('8. 运动控制 / 路径跟随', '几何内核下沉 tools/pure_pursuit，类只保留状态机。',
         mk(pick(task_pool, ('computeFollowCommand', 'computeAstarFollowCommand',
                             'startMotion', 'stopMotion')))),
    ]
    tasks_doc = build_doc(
        lines,
        '04 业务流程（tasks/）',
        '**约定**：`tasks/` 是"做什么" —— 编排 io 的收发与 tools 的计算，持有状态与状态机；'
        '自己不做数学、不发消息、不写文件。\n'
        '迁入顶层 `Navigation/tasks/`，每小节一个类：`CloudPipeline` / `DynamicSplitter` / `PoseCorrector` / '
        '`MapRenderer` / `MapIo` / `AstarPlanner` / `HoldPointTask` / `ChaseTask` / `MotionController` / `DebugLogger`。',
        task_secs)
    write('04_tasks.md', tasks_doc)

    # ===================== 05_node.md：装配与回调 =====================
    node_pool = by_layer(NODE, 'src')
    node_secs = [
        ('1. 节点装配与主函数', '构造装配区（605-815）+ main + createMotionTimer + printConfig。',
         mk(pick(node_pool, ('constructor_assembly', 'main', 'printConfig', 'createMotionTimer')))),
        ('2. 订阅回调', '回调保持很薄：解析消息 → 写快照 → 交给 tasks。',
         mk(pick(node_pool, ('odomCallback', 'planCallback', 'localPlanCallback', 'goalPoseCallback')))),
        ('3. 定时器回调 / 运动状态机', 'motionTimerCallback 是全文件最长函数（1483 行），按 `README.md` 拆段表处理。',
         mk(pick(node_pool, ('renderTimerCallback', 'motionTimerCallback')))),
        ('4. 参数回调', '2580-3311 共 732 行；参数键的唯一来源是 `config/params.yaml`。',
         mk(pick(node_pool, ('parameterCallback',)))),
    ]
    member_items = []
    for title, s, e in MEMBER_GROUPS:
        member_items.append(dict(
            name='成员变量: %s' % title, start=s,
            target='（去向见 01_config.md 附录表）',
            purpose='（变量清单见源码；参数类键进 params.yaml）',
            note='block(%d-%d)' % (s, e), end=e))
    node_secs.append(('5. 附录：成员变量区源码（private, 7569-7924）',
                      '按组切块便于对照搬迁；每组去向见 `01_config.md` 附录表。', member_items))
    node_doc = build_doc(
        lines,
        '05 节点装配与回调（src/）',
        '**约定**：`src/` 只做 ROS 装配 —— 创建订阅/发布器/定时器/服务客户端，回调里转调 tasks。'
        '不含算法、不含文件读写、不含 CAN 系统调用（都在 io）。\n'
        '迁入顶层 `Navigation/src/`：`main.cpp` + `realtime_pgm_node.{hpp,cpp}` + `callbacks.cpp` + '
        '`parameter_callback.cpp`。',
        node_secs)
    write('05_node.md', node_doc)

    # ===================== INDEX.md =====================
    idx = ['# 函数索引（按层 -> 目标文件）', '',
           '源文件: `test/test_navigation/src/test_navigation.cpp` (7937 行)。'
           '「源码行」为起止行，「文档」为含完整源码的详表。', '',
           '层级：`config/`（只有 yaml） → `tools/`（纯计算） → `io/`（硬件与 ROS/文件） → '
           '`tasks/`（业务流程） → `src/`（装配与回调）。', '']
    idx.append('| # | 符号 / 块 | 源码行 | 目标位置 | 文档 |')
    idx.append('|---|---|---|---|---|')
    rows = []
    for item in ALL:
        n, s, t, p, nt = item
        e = item_end(lines, n, nt, s)
        rows.append((s, n, '%d-%d' % (s, e), t, LAYER_DOC.get(layer_of(t), '-')))
    n, s, t, p, nt = PARAM_BLOCK
    rows.append((s, '参数声明区', '%d-%d' % (s, 604), t, '01_config.md'))
    for title, s, e in MEMBER_GROUPS:
        rows.append((s, '成员变量: %s' % title, '%d-%d' % (s, e),
                     MEMBER_TARGET.get(title, '待定'), '05_node.md'))
    rows.sort(key=lambda r: r[0])
    for i, (_, n, span, t, doc) in enumerate(rows, 1):
        idx.append('| %d | `%s` | %s | `%s` | %s |' % (i, n, span, t, doc))
    idx.append('')
    idx.append('合计 **%d** 个条目。' % len(rows))
    write('INDEX.md', idx)
    write('BY_FILE.md', build_by_file(lines))
    print('已生成 %d 个条目' % len(rows))


def write(name, lines_out):
    path = os.path.join(OUT_DIR, name)
    text = '\n'.join(lines_out).rstrip('\n') + '\n'
    with open(path, 'w', encoding='utf-8') as f:
        f.write(text)
    print('写入 %s (%d 行)' % (path, text.count('\n')))


if __name__ == '__main__':
    main()
