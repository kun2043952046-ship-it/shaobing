# 07 src 层实现设计（main 极简 · 装配集中 · 回调即转发）

> 与 `05_node.md` 的区别：`05_node.md` 是**现状源码**（原文件 7927 行里属于 src 的部分，逐段带源码）；
> 本文是**目标设计**——拆完之后 src 应该长什么样、main 应该多简洁、装配放在哪、回调放什么。

## 1. 硬指标（验收用）

| 指标 | 上限 | 说明 |
|---|---|---|
| `main.cpp` 行数 | **≤ 20 行**（含空行/大括号） | 只做 init / spin / shutdown + 异常兜底 |
| `realtime_pgm_node.cpp` | ≤ 250 行 | 只有 `loadConfig / createIo / createTasks / createXs / printConfig` |
| 单个回调函数 | **≤ 15 行** | 解包消息 → 调 task → `return` |
| `src/` 里出现的数学运算 | **0** | 不允许 `sin/cos/atan2/sqrt`、不允许栅格/路径计算 |
| `src/` 里的消息构造细节 | **0** | `PointCloud2` 填字段、`Path` 拼 pose 都在 `io/` 或 `tools/` |
| `src/` 里的文件/设备操作 | **0** | `ofstream`、`socket`、`ioctl` 都在 `io/` |
| `src/` 里的业务分支 | **≤ 3 层嵌套** | 追击/占点/跟随的 if-else 全在 `tasks/` |

一句话：**src 是"接线盒"，不是"电路"。**

## 2. 文件划分

| 文件 | 行数预算 | 职责 | 不允许出现 |
|---|---|---|---|
| `main.cpp` | ~16 | 入口 | 一切参数、一切成员 |
| `realtime_pgm_node.hpp` | ~120 | 类声明 + 成员分组 + 注释 | 实现 |
| `realtime_pgm_node.cpp` | ~250 | 构造 → `configure()` 五步装配 + `printConfig` | 算法、消息打包 |
| `callbacks.cpp` | ~150（6 个） | 订阅/定时器回调薄壳 | 超过 15 行的单个回调 |
| `parameter_callback.cpp` | ~200 | 解析 → 校验 → 提交三段 | 直接改成员（必须先落影子变量） |
| `config_loader.cpp` | ~120 | `params.yaml` → `tools::Config` + 话题名 | 业务默认值散落 |

## 3. main.cpp 目标代码（16 行）

```cpp
// src/main.cpp
#include <exception>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "realtime_pgm_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  int code = 0;
  try {
    rclcpp::spin(std::make_shared<nav::RealtimePgmNode>(rclcpp::NodeOptions{}));
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("main"), "节点异常退出: %s", e.what());
    code = 1;
  }

  rclcpp::shutdown();
  return code;
}
```

四点设计意图：

1. **不在 main 里读参数**：`NodeOptions` 透传给节点，参数在节点内加载；main 完全不认识 `params.yaml`。
2. **不在 main 里选模式**：追击/占点/跟随是运行时行为，由参数与目标切换，不需要 `main` 分支。
3. **try/catch 兜底**：原实现异常会直接 terminate，日志里看不出原因；这里统一转成 FATAL + 退出码。
4. **`rclcpp::spin` 在 close 之后**：`RealtimePgmNode` 的析构函数负责收尾（见 §8），main 不写任何 cleanup。

## 4. 节点类骨架（`realtime_pgm_node.hpp`）

```cpp
namespace nav {

class RealtimePgmNode : public rclcpp::Node
{
public:
  explicit RealtimePgmNode(const rclcpp::NodeOptions & options);
  ~RealtimePgmNode() override;          // 收尾: 停车 / 关 CAN / flush 日志

private:
  // ---------- 1. 配置 ----------
  void loadConfig();                    // params.yaml -> cfg_ + 话题名 + 模式开关
  void printConfig() const;             // 启动时打印全部生效配置

  // ---------- 2. 装配（按依赖顺序，五步）----------
  void createIo();                      // CanBus / Ros*Publisher / DebugLogFile / Nav2ServiceClient
  void createTasks();                   // 9 个 task 对象，注入 io 与 cfg_
  void createSubscriptions();           // 点云 / 里程计 / 全局路径 / 局部路径 / 目标点
  void createTimers();                  // render_timer_ / motion_timer_
  void createParameterCallback();       // 热更新入口

  // ---------- 3. 回调（薄壳，每个 ≤ 15 行）----------
  void onCloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void onOdom(nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void onGlobalPlan(nav_msgs::msg::Path::ConstSharedPtr msg);
  void onLocalPlan(nav_msgs::msg::Path::ConstSharedPtr msg);
  void onGoalPose(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
  void onRenderTimer();
  void onMotionTimer();

  // ---------- 4. 参数 ----------
  rcl_interfaces::msg::SetParametersResult onParamUpdate(
    const std::vector<rclcpp::Parameter> & params);

  // ---------- 5. 成员（按层分组，方便一眼看出依赖方向）----------
  tools::Config cfg_;                                   // 只读快照（换参数时整体替换）

  std::shared_ptr<io::CanBus>              can_;
  std::shared_ptr<io::RosCloudPublisher>   cloud_pub_;
  std::shared_ptr<io::RosPathPublisher>    path_pub_;
  std::shared_ptr<io::RosMotionPublisher>  motion_pub_;
  std::shared_ptr<io::RosPosePublisher>    pose_pub_;
  std::shared_ptr<io::RosImagePublisher>   image_pub_;
  std::shared_ptr<io::Nav2ServiceClient>   nav2_client_;
  std::shared_ptr<io::DebugLogFile>        debug_log_;

  std::shared_ptr<tasks::CloudPipeline>    cloud_;
  std::shared_ptr<tasks::DynamicSplitter>  splitter_;
  std::shared_ptr<tasks::PoseCorrector>    corrector_;
  std::shared_ptr<tasks::MapRenderer>      renderer_;
  std::shared_ptr<tasks::MapIo>            map_io_;
  std::shared_ptr<tasks::AstarPlanner>     astar_;
  std::shared_ptr<tasks::HoldPointTask>    hold_point_;
  std::shared_ptr<tasks::ChaseTask>        chase_;
  std::shared_ptr<tasks::MotionController> motion_;
  std::shared_ptr<tasks::DebugLogger>      logger_;

  rclcpp::Subscription<...>::SharedPtr cloud_sub_, odom_sub_, plan_sub_, local_plan_sub_, goal_sub_;
  rclcpp::TimerBase::SharedPtr render_timer_, motion_timer_;
  OnSetParametersCallbackHandle::SharedPtr param_handle_;
};

}  // namespace nav
```

构造函数只做**三步**，其余全部下沉：

```cpp
RealtimePgmNode::RealtimePgmNode(const rclcpp::NodeOptions & options)
: Node("test_navigation", options)
{
  loadConfig();          // 1. 参数 -> cfg_（含 122 个键的默认值与 yaml 覆盖）
  createIo();            // 2. 硬件/话题端点
  createTasks();         // 3. 业务对象
  createSubscriptions(); // 4. 输入
  createTimers();        // 5. 节拍
  createParameterCallback();

  RCLCPP_INFO(get_logger(), "test_navigation 启动完成");
}
```

## 5. 装配顺序为什么是这个顺序

```
loadConfig        ← 后面所有对象都要用 cfg_（分辨率、半径、话题名、开关）
createIo          ← 话题名来自 cfg_；io 只依赖 tools
createTasks       ← task 需要 io 的发布句柄 + cfg_；tasks 依赖 io
createSubscriptions ← 回调里要调 tasks，故 task 必须先存在
createTimers      ← 定时器最后，避免"定时器已跑、任务还没建好"的空窗
createParameterCallback ← 最后注册，防止构造期参数变化打断装配
```

**失败策略**：`loadConfig` 校验失败 → 抛 `std::runtime_error`（main 里转 FATAL 退出）；`createIo` 里 CAN 打不开只 WARN 不抛（可无 CAN 运行）；订阅/定时器创建失败直接抛。

## 6. 回调应该长什么样

### 例 1：里程计（现状 103 行 → 目标 ≤ 10 行）

现状 `odomCallback`（1045-1147）把"解四元数 → 跳变检测 → 拒绝窗口 → 施加 map→odom 修正 → 写 8 个成员"全塞在一起。

```cpp
void RealtimePgmNode::onOdom(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  motion_->updateOdom(*msg);          // 解算 + 跳变检测 + 拒绝窗口都在 MotionController
  if (motion_->odomRejected()) {      // 可选: 仅记录, 不影响控制流
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "位姿跳变, 本帧已丢弃");
  }
}
```

### 例 2：点云（现状 235 行 → 目标 ≤ 12 行）

```cpp
void RealtimePgmNode::onCloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  cloud_->onCloud(*msg);              // 校验/解析/修正/分离/入队 全在 CloudPipeline
}
```

### 例 3：定时器（现状 71 / 1483 行 → 目标各 ≤ 5 行）

```cpp
void RealtimePgmNode::onRenderTimer() { renderer_->tick(); }
void RealtimePgmNode::onMotionTimer() { motion_->tick(); }   // 追击/占点/跟随的状态机全部在 tasks
```

判据：**回调里出现 `if (chase_enabled_) ... else if (hold_point_enabled_) ...` 就是放错层了。**

## 7. 参数回调的三段式（`parameter_callback.cpp`）

```cpp
rcl_interfaces::msg::SetParametersResult
RealtimePgmNode::onParamUpdate(const std::vector<rclcpp::Parameter> & params)
{
  auto shadow = cfg_;                       // ① 影子副本: 要么全成, 要么全不动
  std::vector<std::string> changed;

  for (const auto & p : params) {           // ② 逐个解析 + 校验（不提交）
    std::string err = applyToShadow(p, shadow);
    if (!err.empty()) {
      return makeResult(false, err);
    }
    changed.push_back(p.get_name());
  }

  cfg_ = std::move(shadow);                 // ③ 一次性提交
  refreshDependent();                       // 重建定时器/基准图/日志目录（仅必要时）
  RCLCPP_INFO(get_logger(), "热更新 %zu 项: %s", changed.size(), join(changed).c_str());
  return makeResult(true, "");
}
```

要点：影子副本保证"改一半失败"不会留下半新半旧的状态；`refreshDependent()` 里只重建**真正受影响**的对象（例如 `history_duration` 变了才重建基准图）。

## 8. 析构与关闭

```cpp
RealtimePgmNode::~RealtimePgmNode()
{
  if (motion_) { motion_->stop(); }   // 下发 0 速帧, 避免停车瞬间残留速度
  if (can_)    { can_->close(); }     // SocketCAN fd
  if (logger_) { logger_->flush(); }  // CSV 落盘
}
```

顺序要求：**先停车，再关通信，最后 flush 日志**。

## 9. 执行器与线程（可选优化，不是必须）

现状单线程 `rclcpp::spin`，点云回调与运动定时器互相阻塞。若点云单帧耗时 > 30 ms，可改用：

```cpp
rclcpp::executors::MultiThreadedExecutor exe;
exe.add_node(node);
exe.spin();
```

配合回调组：点云/里程计 `MutuallyExclusive`（保序），渲染定时器 `Reentrant`（可被点云抢占）。**但必须同时把 `cloud_mutex_` 等 4 把锁按文档 §5 风险点 3 改成快照，否则加执行器只会把死锁概率放大。**

## 10. 迁移步骤（在 03/04 完成后做）

1. 先把 `tasks/` 的 9 个类建好并各自能独立编译（此时它们吃 `cfg_` + io 句柄）。
2. 新建 `main.cpp`（本文 §3）与 `realtime_pgm_node.{hpp,cpp}`（§4），先只装配 `CloudPipeline`，其余留空。
3. 逐个把回调搬成薄壳，每搬一个就 `ros2 topic echo` 验证行为不变。
4. 搬 `parameter_callback`，用 `ros2 param set` 逐组回归。
5. 删掉旧的 `test_navigation.cpp`，编译告警清零。

## 11. 验收清单

- [ ] `main.cpp` 行数 ≤ 20，且不含 `declare_parameter` / `create_publisher`
- [ ] `realtime_pgm_node.cpp` 里没有一处 `for` 循环遍历点云
- [ ] 6 个回调合计 ≤ 60 行
- [ ] `src/` 里 `grep -n "atan2\|sqrt\|sin(" ` 无结果
- [ ] `src/` 里 `grep -n "ofstream\|socket(" ` 无结果
- [ ] `grep -n "chase_enabled\|hold_point_enabled" src/` 仅出现在 `loadConfig`
- [ ] `ros2 param set /test_navigation resolution 0.05` 后重规划行为不崩
