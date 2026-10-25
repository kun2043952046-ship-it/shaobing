# 调试日志目录

本目录用于存放实时 PGM 节点的**调试日志与异常快照**，便于"雷达突然全是障碍"
一类问题事后溯源。运行时由节点自动创建/写入，无需手动准备。

## 文件说明

| 文件 | 内容 |
| --- | --- |
| `cloud_frames.csv` | **逐帧点云统计**（每次启动覆盖重写）。列：`stamp,frame,raw,valid,self_filtered,queued,cx,cy,cz,xmin,xmax,ymin,ymax,zmin,zmax,anomaly` |
| `cloud_anomaly_<ns>.pcd` | **异常帧点云快照**（ASCII PCD），异常触发时自动保存 |

### CSV 列含义

- `stamp`：点云 header 时间戳（秒）
- `frame`：本节点收到并处理的点云帧序号
- `raw`：PointCloud2 原始点数（width × height）
- `valid`：解析且 `isfinite` 通过的点数（`raw - valid` 即 NaN/Inf 点数）
- `self_filtered`：被车体自过滤剔除的点数
- `queued`：进入历史缓冲的点数（动态障碍模式下为 0，表示不累积）
- `cx,cy,cz`：自过滤后点云质心（m）
- `xmin..zmax`：自过滤后点云包围盒（m）
- `anomaly`：异常标签（`-` 表示正常；`点数突增` / `质心跳变`，可组合）

## 异常判定门限（可调）

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| `debug_cloud_count_jump_ratio` | 3.0 | 本帧点数 > 上帧 × 该倍数 |
| `debug_cloud_count_jump_delta` | 20000 | 且差值 > 该值 → 判"点数突增" |
| `debug_centroid_jump_m` | 1.5 | 质心与上帧距离 > 该值（m）→ 判"质心跳变" |
| `debug_odom_jump_m` | 0.5 | 里程计单帧位移 > 该值（m）→ WARN |
| `debug_odom_jump_yaw_deg` | 30.0 | 里程计单帧偏航 > 该值（°）→ WARN |
| `pose_jump_reject_sec` | 0.8 | 检出位姿跳变后，丢弃其后该时长内的点云帧（s，<=0 关闭） |
| `pose_correction_global_period_sec` | 1.0 | 锁定历史图后的全局粗配准周期（s） |
| `pose_correction_global_range_m` | 15.0 | 全局粗配准平移搜索范围（m） |
| `pose_correction_local_range_m` | 1.0 | 非全局帧局部精配准平移搜索范围（m） |
| `pose_correction_score_min` | 0.12 | 配准最低重合分数，低于该值沿用上次校正 |
| `pose_correction_max_delta_m` | 1.5 | 单帧相对上次校正的平移变化上限（m），超限判跳飞并沿用上次值；<=0 关闭 |
| `debug_occ_ratio_jump` | 0.15 | 历史图占据比例单帧涨幅 > 该值 → WARN |
| `debug_pcd_max_files` | 200 | PCD 快照保存上限（超过不再保存） |

## 排查建议

出现"突然全是障碍"时，按以下顺序看：

1. **`cloud_frames.csv` 的 `points` 是否骤增** → 若是，看 PCD 快照确认是否点云本身异常。
2. **`cx,cy` 是否阶跃** → 若是且同时有"里程计位姿跳变"WARN，基本可判定为
   LIO 位姿发散/重定位导致整帧 map 系点云错位。
3. **`zmin,zmax` 是否超出 `zmin/zmax` 配置窗口** → 若是，是 z 漂移。
4. **终端是否出现"历史图占据比例暴涨"WARN** → 定位到具体时刻，与上述两表交叉比对。

## 关闭方式

```bash
# 完全关闭调试日志
ros2 launch test_navigation realtime_pgm.launch.py debug_log_enabled:=false
# 只关 PCD 快照, 保留 CSV 统计
ros2 launch test_navigation realtime_pgm.launch.py debug_pcd_enabled:=false
# 指定日志目录 (绝对路径)
ros2 launch test_navigation realtime_pgm.launch.py \
  debug_log_dir:=/home/lele/aaaaa_mine_NJIT_Vision_Gen2/Navigation/test/test_navigation/log
```
