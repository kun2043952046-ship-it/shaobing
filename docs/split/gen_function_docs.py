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


def read_slice(lines, start, end):
    return lines[start - 1:end]


# 按目标文件分组: (文件, 建议类/命名空间, [符号名])
FILE_GROUPS = [
    ('config/constants.hpp', 'namespace nav::config', ['consts']),
    ('config/types.hpp', 'namespace nav::config', [
        'Point', 'LookaheadView', 'TimedCloud', 'PoseCorrection',
        'HoldObstacleCluster', 'FootprintBounds', 'ChaseTrackPoint']),
    ('config/config.hpp', 'namespace nav::config', ['Config']),
    ('config/config_loader.cpp', 'nav::config::loadConfig(node)', ['参数声明区 (constructor 118-604)']),

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
    ('tools/io_pcd.cpp', 'namespace nav::tools', ['saveCloudPcd']),
    ('tools/io_bin.cpp', 'namespace nav::tools', ['writeHistoryCloudsBin', 'readHistoryCloudsBin']),
    ('tools/io_yaml.cpp', 'namespace nav::tools', ['writeMapYaml']),

    ('tasks/cloud_pipeline.cpp', 'class CloudPipeline', ['cloudCallback', 'pruneHistory']),
    ('tasks/debug_log.cpp', 'class DebugLogger', ['logCloudFrame']),
    ('tasks/dynamic_split.cpp', 'class DynamicSplitter', [
        'splitDynamicObstacles', 'subtractBaseline', 'buildStaticBaseline']),
    ('tasks/pose_correction.cpp', 'class PoseCorrector', [
        'getPoseCorrection', 'publishPoseCorrection', 'setPoseCorrection2D',
        'resetPoseCorrection', 'estimatePoseCorrection', 'correctCloudForPose']),
    ('tasks/cloud_publish.cpp', 'class CloudPublisher', [
        'publishCloudPreview', 'pointCloudStamp', 'publishPointCloud',
        'publishSelfFilteredCloud', 'publishDynamicObstacleCloud', 'publishClearRingCloud']),
    ('tasks/map_render.cpp', 'class MapRenderer', [
        'renderPgm', 'renderHistoryPgm', 'renderDynamicObstaclePgm']),
    ('tasks/map_io.cpp', 'class MapIo', [
        'exportHistoryMap', 'importHistoryMap', 'mergeImportedMap']),
    ('tasks/astar_planner.cpp', 'class AstarPlanner', [
        'buildMergedOccupancy', 'planAstarPath', 'planAstarToGoal',
        'getAstarPath', 'publishAstarPlan']),
    ('tasks/hold_point.cpp', 'class HoldPointTask', [
        'computeHoldPointCommand', 'publishHoldPointViz', 'publishHoldPointStatus']),
    ('tasks/chase.cpp', 'class ChaseTask', [
        'computeChaseCommand', 'recordTargetCloud', 'publishChaseViz',
        'publishChaseStatus', 'publishChaseTrack']),
    ('tasks/motion_control.cpp', 'class MotionController', [
        'computeFollowCommand', 'computeAstarFollowCommand', 'startMotion', 'stopMotion']),
    ('tasks/motion_frame.cpp', 'class MotionFramePublisher', ['publishMotionFrame']),
    ('tasks/nav2_service.cpp', 'class Nav2ServiceClient', ['clearCostmaps']),

    ('src/main.cpp', 'int main()', ['main']),
    ('src/realtime_pgm_node.cpp', 'class RealtimePgmNode', [
        'constructor_assembly', 'initDebugLog', 'printConfig', 'createMotionTimer']),
    ('src/callbacks.cpp', 'class RealtimePgmNode (回调段)', [
        'odomCallback', 'planCallback', 'localPlanCallback', 'goalPoseCallback',
        'renderTimerCallback', 'motionTimerCallback']),
    ('src/parameter_callback.cpp', 'class RealtimePgmNode (参数段)', ['parameterCallback']),
    ('src/can_socket.cpp', 'class CanSocket', [
        'openCanSocket', 'closeCanSocket', 'canFrameFromMotion']),
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


def build_param_table(lines):
    """扫描参数声明区, 生成 参数名 -> 主题 的表格。"""
    out = ['## 参数清单（自动扫描 118-604 行的 declare_parameter）', '']
    found = []
    pat = re.compile(r'declare_parameter<(.+?)>\s*\(\s*"([A-Za-z0-9_]+)"')
    for idx in range(117, 604):
        line = lines[idx]
        m = pat.search(line)
        if m:
            found.append((m.group(2), m.group(1).strip(), idx + 1))
    seen = {}
    for name, typ, ln in found:
        seen[name] = (typ, ln)
    out.append('共 **%d** 个参数。' % len(seen))
    out.append('')
    out.append('| 主题 | 参数 | 类型 | 声明行 |')
    out.append('|---|---|---|---|')
    mapped = set()
    for group, names in PARAM_GROUPS:
        for n in names:
            if n in seen:
                typ, ln = seen[n]
                out.append('| %s | `%s` | `%s` | %d |' % (group, n, typ, ln))
                mapped.add(n)
    rest = sorted(set(seen) - mapped)
    for n in rest:
        typ, ln = seen[n]
        out.append('| (未归类, 待补充) | `%s` | `%s` | %d |' % (n, typ, ln))
    out.append('')
    return out


def build_by_file(lines):
    """按"目标文件"汇总: 每个文件一节, 先列符号表, 再给合并后的完整源码。"""
    lookup = {}
    for item in CONFIG_TYPES:
        n, s, t, p, nt = item
        lookup[n] = dict(name=n, start=s, target=t, purpose=p, note=nt,
                         doc='01_config.md', kind='struct')
    lookup['参数声明区 (constructor 118-604)'] = dict(
        name='参数声明区', start=118, target='config/config_loader.cpp',
        purpose='构造函数的 104 个 declare_parameter', note='block(118-604)',
        doc='01_config.md', kind='代码块')
    for item in TOOLS:
        n, s, t, p, nt = item
        lookup[n] = dict(name=n, start=s, target=t, purpose=p, note=nt,
                         doc='02_tools.md', kind='函数')
    for item in TASKS:
        n, s, t, p, nt = item
        lookup[n] = dict(name=n, start=s, target=t, purpose=p, note=nt,
                         doc='03_tasks.md', kind='函数')
    for item in NODE:
        n, s, t, p, nt = item
        lookup[n] = dict(name=n, start=s, target=t, purpose=p, note=nt,
                         doc='04_node.md',
                         kind='代码块' if n == 'constructor_assembly' else '函数')

    out = ['# 按目标文件汇总（BY_FILE）', '',
           '同一份拆分方案的"文件视角"：每个目标文件一节，先列该文件要装哪些符号，'
           '再给出**合并后的完整源码**（各段前有 `// ===== 源码行 a-b: 符号名 =====` 分隔注释，便于对照原文件）。', '',
           '> 注意：合并后的代码是**搬运视图**，不是可直接编译的完整文件——'
           '类方法的声明仍需在对应 `.hpp` 里按类组织，`private:` 成员、`#include`、'
           '`namespace` 包裹都要你在新建文件时补齐。参数与成员变量的分配见文末附录。', '',
           '对照文档：`01_config.md` / `02_tools.md` / `03_tasks.md` / `04_node.md`，索引见 `INDEX.md`。', '']

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
    out.append('成员变量不是简单整块搬走：`config_*` / `motion_*` / `astar_*` 这类参数应进 '
               '`config/` 的结构体，运行期状态（mutex / 队列 / 缓存）应进对应 tasks 类的私有成员，'
               '发布器与订阅器留在 `src/` 节点类。逐组源码见 `01_config.md` 第 3 节。')
    out.append('')
    out.append('| 成员变量组 | 源码行 | 建议落点 |')
    out.append('|---|---|---|')
    member_target = {
        '点云基础 / 渲染周期': 'src/ 节点私有成员（cloud_mutex_ 保留在节点或 CloudPipeline）',
        '调试日志 / 异常快照 / 跳变检测': 'tasks/debug_log.cpp → DebugLogger',
        '参数与配置快照': 'config/config.hpp',
        '订阅 / 定时器 / 单帧图发布': 'src/ 节点私有成员',
        '历史图缓冲': 'tasks/cloud_pipeline.cpp 或 MapRenderer',
        '地图导出 / 导入': 'tasks/map_io.cpp → MapIo',
        '动态障碍 / 位姿修正': 'tasks/dynamic_split.cpp + tasks/pose_correction.cpp',
        '自过滤点云发布': 'tasks/cloud_publish.cpp → CloudPublisher',
        '清除射线': 'tasks/cloud_publish.cpp → CloudPublisher',
        '清空 Nav2 代价地图': 'tasks/nav2_service.cpp',
        '运动帧框架 / 输入话题': 'config/ + tasks/motion_frame.cpp',
        '路径跟随参数': 'config/config.hpp → MotionConfig',
        'A* 兜底参数': 'config/config.hpp → AstarConfig',
        '占点': 'config/ (参数) + tasks/hold_point.cpp (状态)',
        '追击': 'config/ (参数) + tasks/chase.cpp (状态)',
        '轨迹 + A* 缓存 + 目标点缓存': 'tasks/chase.cpp + tasks/astar_planner.cpp',
        '运动状态 / 里程计 / 路径缓冲': 'tasks/motion_control.cpp + src/ 回调',
        '运动请求 + 可视化快照': 'tasks/motion_control.cpp（lookahead_view_ 的写入上移到此）',
        'SocketCAN 直发': 'src/can_socket.cpp → CanSocket',
        '参数回调句柄': 'src/parameter_callback.cpp',
    }
    for title, s, e in MEMBER_GROUPS:
        out.append('| %s | %d-%d | %s |' % (title, s, e, member_target.get(title, '待定')))
    out.append('')
    return out


def main():
    if not os.path.exists(SRC):
        sys.exit('找不到源文件: %s' % SRC)
    with open(SRC, 'r', encoding='utf-8') as f:
        lines = f.read().split('\n')
    print('源文件: %s (%d 行)' % (SRC, len(lines) - 1))

    def mk(items, kind_default='func'):
        res = []
        for item in items:
            name, start, target, purpose, note = item[:5]
            kw = dict(name=name, start=start, target=target, purpose=purpose, note=note)
            if kind_default == 'block' and len(item) > 5:
                kw['end'] = item[5]
            res.append(kw)
        return res

    # ---- 01_config.md ----
    cfg_sections = [
        ('1. 常量与数据结构（config/）', '全部为类内 struct/constexpr, 建议搬到 config/ 下的纯头文件, 供 tools 与 tasks 共用。',
         [dict(name=n, start=s, target=t, purpose=p, note=nt,
               end=int(nt.split('(')[1].split(')')[0].split('-')[1]) if nt.startswith('block(') else None)
          for (n, s, t, p, nt) in CONFIG_TYPES]),
        ('2. 参数声明区（构造函数前半段, 建议替换为 config 加载器）',
         '这 487 行是 86 个 `declare_parameter`。拆分后建议改为：`config/params.yaml` + `config/config_loader.cpp` '
         '一次性读入 `AppConfig` 结构体，构造函数只保留装配。',
         [dict(name='参数声明区 (constructor 118-604)', start=118, target='config/config_loader.cpp',
               purpose='逐项 declare_parameter 并写入成员/Config', note='block(118-604)', end=604)]),
    ]
    # 成员变量分组
    member_items = []
    for title, s, e in MEMBER_GROUPS:
        member_items.append(dict(
            name='成员变量: %s' % title, start=s, target='config/config.hpp + 各 tasks 类的私有成员',
            purpose='（变量清单见源码, 拆分时按主题分配到对应类）', note='block(%d-%d)' % (s, e), end=e))
    cfg_sections.append(('3. 成员变量区（private, 7569-7924）',
                         '按主题切块，便于你逐块搬到 `config/` 结构体或各 tasks 类。',
                         member_items))

    cfg = build_doc(
        lines,
        '01 配置与数据类型（config/）',
        '来源: `test/test_navigation/src/test_navigation.cpp`。本文覆盖常量、数据结构、参数声明、成员变量四部分。\n'
        '迁入目录: 工作区顶层 `Navigation/config/`；若设为纯头文件目录，`tools/` 与 `tasks/` 都可直接 include。',
        cfg_sections, params_table=True)
    write('01_config.md', cfg)

    # ---- 02_tools.md ----
    tool_secs = [
        ('1. 几何 / 坐标变换', '全部无 ROS 通信、只依赖入参；抽到 tools/ 后建议加 `namespace nav::tools`。', mk([
            i for i in TOOLS if i[0] in
            ('hasField', 'quaternionToMatrix', 'matrixToQuaternion', 'yawFromMatrix', 'wrapAngle',
             'multiplyMatrix', 'transformPointByCorrection', 'score_at', 'transformToBaseLink')])),
        ('2. 运动帧编解码（12 位有符号量协议）', '纯位运算与量化，可直接脱离 ROS 单测。', mk([
            i for i in TOOLS if i[0] in
            ('limitMotionSpeed', 'effectiveSlowRadius', 'encodeMotionRaw', 'resolveAstarSpeed')])),
        ('3. 栅格几何与像素', '占据栅格/像素层面的纯运算, 是拆分收益最大的一块。', mk([
            i for i in TOOLS if i[0] in
            ('computeMapSize', 'computeVehicleFootprint', 'clipCloudToMapBounds',
             'filterVehicleFootprintPoints', 'clearVehicleFootprint', 'accumulateCloud',
             'applyDilate', 'occupancyToPixels')])),
        ('4. 序列化 (PCD / BIN / YAML)', '只做文件 IO, 不含发布; 建议签名改为 `bool write*(path, ...)`。', mk([
            i for i in TOOLS if i[0] in
            ('saveCloudPcd', 'writeHistoryCloudsBin', 'readHistoryCloudsBin', 'writeMapYaml')])),
        ('5. 栅格查询 / 聚类 / 间距判定', '占点与追击共用，说明它们是通用几何算法而非业务。', mk([
            i for i in TOOLS if i[0] in
            ('extractHoldObstacles', 'computeHoldBlockPose', 'isHoldPointClear',
             'isHoldPointClearPath', 'findNearestObstacleCluster', 'chaseLineClear')])),
    ]
    # score_at 是 lambda, 需要显式 end
    for it in tool_secs[0][2]:
        if it['name'] == 'score_at':
            it['end'] = 1505
    tools_doc = build_doc(
        lines,
        '02 纯计算与坐标转换（tools/）',
        '来源: `src/test_navigation.cpp`。这些函数只做数学/几何/编解码/文件 IO，**没有一个需要 ROS 节点上下文**。\n'
        '迁入目录: 工作区顶层 `Navigation/tools/`。建议按本文的小节拆成独立 .hpp/.cpp，'
        '并统一 `namespace nav::tools`，让 `tasks/` 只依赖头文件。\n\n'
        '> 大函数内部还可以继续抽出纯内核（A* 搜索、膨胀、代价场、跟随几何段），清单见 `README.md` 的"二次抽取"一节。',
        tool_secs)
    write('02_tools.md', tools_doc)

    # ---- 03_tasks.md ----
    task_secs = [
        ('1. 点云流水线 / 动态障碍分离', 'cloudCallback 是全文件最强耦合点（4 把锁），拆分时建议先抽 tools 再搬它。',
         mk([i for i in TASKS if i[0] in
             ('cloudCallback', 'pruneHistory', 'logCloudFrame', 'splitDynamicObstacles',
              'subtractBaseline', 'buildStaticBaseline')])),
        ('2. 位姿修正（map->odom 2D 配准）', '流程在 tasks，评分内核在 tools/registration。',
         mk([i for i in TASKS if i[0] in
             ('getPoseCorrection', 'publishPoseCorrection', 'setPoseCorrection2D',
              'resetPoseCorrection', 'estimatePoseCorrection', 'correctCloudForPose')])),
        ('3. 点云发布', '限频/抽稀/坐标变换 + ROS 发布。',
         mk([i for i in TASKS if i[0] in
             ('publishCloudPreview', 'pointCloudStamp', 'publishPointCloud',
              'publishSelfFilteredCloud', 'publishDynamicObstacleCloud', 'publishClearRingCloud')])),
        ('4. 建图渲染', '渲染流程（发布 Image + 写文件），其栅格运算部分已在 tools。',
         mk([i for i in TASKS if i[0] in
             ('renderPgm', 'renderHistoryPgm', 'renderDynamicObstaclePgm')])),
        ('5. 地图导入导出', '文件编排 + 状态重建。',
         mk([i for i in TASKS if i[0] in
             ('exportHistoryMap', 'importHistoryMap', 'mergeImportedMap')])),
        ('6. A* 规划', '外层流程（缓存/限频/发布）留 tasks，搜索内核建议下沉 tools/astar。',
         mk([i for i in TASKS if i[0] in
             ('buildMergedOccupancy', 'planAstarPath', 'planAstarToGoal',
              'getAstarPath', 'publishAstarPlan')])),
        ('7. 占点行为', '业务状态机 + 复用 tools 的聚类/间距判定。',
         mk([i for i in TASKS if i[0] in
             ('computeHoldPointCommand', 'publishHoldPointViz', 'publishHoldPointStatus')])),
        ('8. 追击行为', '业务状态机 + 目标轨迹记录 + 3 个发布。',
         mk([i for i in TASKS if i[0] in
             ('computeChaseCommand', 'recordTargetCloud', 'publishChaseViz',
              'publishChaseStatus', 'publishChaseTrack')])),
        ('9. 运动控制 / 路径跟随', '几何控制内核可下沉 tools/pure_pursuit，类只保留状态机与发布。',
         mk([i for i in TASKS if i[0] in
             ('computeFollowCommand', 'computeAstarFollowCommand', 'startMotion',
              'stopMotion', 'publishMotionFrame', 'clearCostmaps')])),
    ]
    tasks_doc = build_doc(
        lines,
        '03 业务流程（tasks/）',
        '来源: `src/test_navigation.cpp`。这些函数需要节点上下文（发布器/互斥量/参数），属于"任务/行为"层。\n'
        '迁入目录: 工作区顶层 `Navigation/tasks/`。建议每个小节一个类：`CloudPipeline` / `PoseCorrector` / '
        '`CloudPublisher` / `MapRenderer` / `MapIo` / `AstarPlanner` / `HoldPointTask` / `ChaseTask` / `MotionController`，'
        '统一由 `src/` 的节点类持有并注入 tools 依赖。',
        task_secs)
    write('03_tasks.md', tasks_doc)

    # ---- 04_node.md ----
    node_secs = [
        ('1. 节点装配与主函数（src/）', '构造函数装配区 + main + 启动期初始化。', mk([
            i for i in NODE if i[0] in
            ('constructor_assembly', 'main', 'initDebugLog', 'printConfig', 'createMotionTimer')])),
        ('2. 订阅回调（src/）', '回调本身保持很薄：解析消息 + 写成员 + 交给 tasks。', mk([
            i for i in NODE if i[0] in
            ('odomCallback', 'planCallback', 'localPlanCallback', 'goalPoseCallback')])),
        ('3. 定时器回调 / 运动状态机', 'motionTimerCallback 是全文件最长的函数（1483 行），'
         '排查/改动成本最高，建议按文末"二次拆解"清单拆段。', mk([
            i for i in NODE if i[0] in ('renderTimerCallback', 'motionTimerCallback')])),
        ('4. 参数回调', '2580-3311 共 732 行, 建议只保留"解析 -> 校验 -> 提交"三段, 参数表外置到 config。', mk([
            i for i in NODE if i[0] == 'parameterCallback'])),
        ('5. SocketCAN 直发', '系统调用层, 建议留在 src（或整体搬 tools/can_bus 并把 publish 部分留节点）。', mk([
            i for i in NODE if i[0] in ('openCanSocket', 'closeCanSocket', 'canFrameFromMotion')])),
    ]
    for sec_title, sec_desc, items in node_secs:
        for it in items:
            if it['name'] == 'constructor_assembly':
                it['end'] = 815
    node_doc = build_doc(
        lines,
        '04 节点装配与回调（src/）',
        '来源: `src/test_navigation.cpp`。这些函数与 ROS 强绑定（创建器/回调/系统调用），留给 `src/` 的节点类。\n'
        '迁入目录: 工作区顶层 `Navigation/src/`。建议拆成 `main.cpp` + `realtime_pgm_node.cpp` + '
        '`callbacks.cpp` + `parameter_callback.cpp` + `can_socket.cpp`，类声明放 `src/realtime_pgm_node.hpp`。',
        node_secs)
    write('04_node.md', node_doc)

    # ---- INDEX.md ----
    idx = ['# 函数索引（按目标目录）', '',
           '源文件: `test/test_navigation/src/test_navigation.cpp` (7937 行)。'
           '「行」列为源码起止行，点击对应文档可看完整源码。', '']
    idx.append('| # | 函数 / 块 | 源码行 | 目标位置 | 文档 |')
    idx.append('|---|---|---|---|---|')
    rows = []
    for item in CONFIG_TYPES:
        n, s, t, p, nt = item
        e = find_end_line(lines, s)
        if nt.startswith('block('):
            e = int(nt.split('(')[1].split(')')[0].split('-')[1])
        rows.append((s, n, '%d-%d' % (s, e), t, '01_config.md'))
    rows.append((118, '参数声明区', '118-604', 'config/config_loader.cpp', '01_config.md'))
    for title, s, e in MEMBER_GROUPS:
        rows.append((s, '成员变量: %s' % title, '%d-%d' % (s, e),
                     'config/config.hpp', '01_config.md'))
    for item in TOOLS:
        n, s, t, p, nt = item
        rows.append((s, n, '%d-%d' % (s, 1505 if n == 'score_at' else find_end_line(lines, s)),
                     t, '02_tools.md'))
    for item in TASKS:
        n, s, t, p, nt = item
        rows.append((s, n, '%d-%d' % (s, find_end_line(lines, s)), t, '03_tasks.md'))
    for item in NODE:
        n, s, t, p, nt = item
        e = 815 if n == 'constructor_assembly' else find_end_line(lines, s)
        rows.append((s, n, '%d-%d' % (s, e), t, '04_node.md'))
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
