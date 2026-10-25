/**
 * test_can.cpp
 *
 * CAN 发送 / 接收测试节点 + 网页调试面板。
 *
 * 功能：
 *   - 通过 SocketCAN（经典 CAN，1 Mbps）向底盘周期发送若干自定义 CAN 帧
 *   - 同时接收 can 总线上的全部帧（0x000~0x7FF 标准帧，扩展帧也可收），
 *     在网页上按 ID 累计显示每一帧的最新内容
 *   - 内置最小 HTTP 服务，浏览器打开 http://<本机IP>:8080 即可可视化编辑每一帧
 *     的每一个数据包（字节）以及每一个二进制位
 *   - 所有发送帧的 CAN_ID 固定为 0xFF（参数 can_id 可改）
 *
 * 网页能力（发送区）：
 *   - 可增删帧，每帧 DLC 可调（0~8）
 *   - 每个字节都有 8 个位开关「00000001」，可逐位点选；同时可输入十六进制
 *     「0xAA」或十进制数值，两者双向同步
 *   - 每帧可单独启用/停用；只有启用的帧会被周期发送
 *   - 支持「只发一次」「暂停/继续发送」，发送速率可在线调整
 *   - 页面实时回显最近一次实际发出的帧内容（HEX + BIN）
 *
 * 网页能力（接收区）：
 *   - 按 CAN_ID 分组，每组的每个字节同样以位图「00000001」显示每一位是 0 还是 1，
 *     并附带 DLC、累计收到帧数、刷新频率（Hz）
 *   - 下方是最近收到的帧历史列表（时间戳 / ID / DLC / HEX / BIN），最新的在最前
 *   - 支持输入自定义 ID 只显示关心的报文；支持「暂停刷新」冻结画面便于观察
 *   - 「清空接收」可把累计值与历史一并清零
 *   - 底部还有一块「0x100 四元数解算」面板：按 io/cboard.cpp 的约定把 ID 0x100 的帧
 *     解析成 x/y/z/w（大端 int16 ÷ 1e4），校验 x²+y²+z²+w² ≈ 1（容差 ±0.01），
 *     并换算显示内旋 Z-Y-X（yaw / pitch / roll）欧拉角，单位为度
 *
 * 使用前先拉起 CAN 接口（以 can0 为例，经典 CAN 1 Mbps）：
 *   sudo ip link set can0 down
 *   sudo ip link set can0 type can bitrate 1000000
 *   sudo ip link set can0 up
 *
 * 运行示例：
 *   ros2 run test_can test_can_node --ros-args \
 *     -p can_interface:=can0 -p can_id:=0xFF \
 *     -p http_port:=8080 -p send_rate_hz:=100.0 \
 *     -p rx_enabled:=true -p rx_history:=64 -p rx_own_msgs:=false
 *
 *   然后浏览器访问 http://<本机IP>:8080
 *
 * 参数：
 *   can_interface  收发共用的 CAN 接口名，默认 can0
 *   can_id         发送帧的 CAN_ID，默认 0xFF
 *   send_rate_hz   发送频率，默认 100.0
 *   http_port      网页面板端口，默认 8080
 *   rx_enabled     是否启用接收，默认 true
 *   rx_history     接收历史最多保留条数（1~4096），默认 64
 *   rx_own_msgs    是否把本节点自己发出的帧也计入接收，默认 false
 *
 * 调试：另一个终端执行  candump can0  可查看总线上的所有帧。
 */
#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>

namespace
{

constexpr size_t kMaxFrames = 16;   // 网页最多可配置的帧数
constexpr size_t kMaxDlc = 8;       // 经典 CAN 单帧最大字节数

// 四元数解算相关常量（参照 NJIT_Vision_Gen2/io/cboard.cpp）
constexpr uint32_t kQuatCanId = 0x100;  // 四元数报文的 CAN_ID
constexpr double kQuatTol = 1e-2;       // 模长平方 x²+y²+z²+w² 与 1 的允许偏差
constexpr double kPi = 3.14159265358979323846;

// 网页中可编辑的一帧
struct WebFrame
{
  bool enabled{true};
  uint8_t dlc{8};
  uint8_t data[kMaxDlc]{};
};

// 一条接收记录（用于历史列表）
struct RxRecord
{
  uint32_t id{0};      // 标准帧为 11 位 ID，扩展帧为 29 位 ID
  bool eff{false};     // 是否扩展帧
  uint8_t dlc{0};
  uint8_t data[kMaxDlc]{};
  double t{0.0};       // 相对节点启动的秒数
  uint64_t seq{0};     // 接收序号（从 1 开始递增）
};

// 按 ID 累计的「最新值」
struct RxEntry
{
  bool eff{false};
  uint8_t dlc{0};
  uint8_t data[kMaxDlc]{};
  uint64_t count{0};   // 该 ID 累计收到多少帧
  double t{0.0};       // 最近一次收到的时间（秒）
  double hz{0.0};      // 刷新频率（指数平滑后的瞬时值）
};

// 0x100 四元数报文的解算结果
struct QuatState
{
  bool has{false};          // 是否收到过 0x100
  bool valid{false};        // 模长校验是否通过
  uint8_t dlc{0};
  double x{0.0}, y{0.0}, z{0.0}, w{0.0};
  double norm2{0.0};        // x² + y² + z² + w²
  double yaw{0.0}, pitch{0.0}, roll{0.0};  // 内旋 Z-Y-X 欧拉角（度）
  double t{0.0};
  uint64_t seq{0};
  uint64_t count{0};         // 0x100 累计收到帧数
  uint64_t invalid_count{0}; // 其中未通过校验的帧数
};

// HTTP 响应用的小工具
struct HttpResponse
{
  int code{200};
  std::string ctype{"text/plain; charset=utf-8"};
  std::string body;
};

std::string toHexByte(uint8_t v)
{
  char buf[8];
  std::snprintf(buf, sizeof(buf), "%02X", static_cast<unsigned>(v));
  return std::string(buf);
}

// 数字转二进制字符串，宽度固定为 8 位（高位在前）
std::string toBinByte(uint8_t v)
{
  std::string s(8, '0');
  for (int i = 0; i < 8; ++i) {
    s[static_cast<size_t>(i)] = ((v >> (7 - i)) & 0x1) ? '1' : '0';
  }
  return s;
}

// CAN_ID 转十六进制字符串：标准帧 3 位，扩展帧 8 位
std::string idToHex(uint32_t id, bool eff)
{
  char buf[16];
  std::snprintf(buf, sizeof(buf), eff ? "%08X" : "%03X", static_cast<unsigned>(id));
  return std::string(buf);
}

// 保留两位小数，避免 JSON 里出现过长浮点
double round2(double v) { return std::lround(v * 100.0) / 100.0; }

// 保留四位小数（四元数分量本身就是 1e-4 量级）
double round4(double v) { return std::lround(v * 10000.0) / 10000.0; }

// 大端两字节转 int16（与 cboard.cpp 中 (int16_t)(d[0] << 8 | d[1]) 等价）
int16_t toInt16(const uint8_t * p)
{
  return static_cast<int16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
}

}  // namespace

class CanSenderNode : public rclcpp::Node
{
public:
  CanSenderNode() : Node("test_can")
  {
    // ---------------- 参数 ----------------
    can_interface_ = declare_parameter<std::string>("can_interface", "can0");
    can_id_ = static_cast<canid_t>(declare_parameter<int64_t>("can_id", 0xFF) & 0x7FF);
    send_rate_hz_ = declare_parameter<double>("send_rate_hz", 100.0);
    http_port_ = static_cast<int>(declare_parameter<int64_t>("http_port", 8080));

    // 接收相关参数
    rx_enabled_ = declare_parameter<bool>("rx_enabled", true);
    rx_own_msgs_ = declare_parameter<bool>("rx_own_msgs", false);
    const int64_t history = declare_parameter<int64_t>("rx_history", 64);
    rx_history_max_ = static_cast<size_t>(std::max<int64_t>(1, std::min<int64_t>(4096, history)));

    // 默认发 1 帧，内容全 0（网页上可自由增删改）
    WebFrame f0;
    f0.enabled = true;
    f0.dlc = 8;
    f0.data[0] = 0x01;  // 第 0 个数据包默认 [00000001]
    frames_.push_back(f0);

    startHttpServer();
    createSendTimer();
    if (rx_enabled_) {
      startRxThread();
    }

    RCLCPP_INFO(get_logger(),
                "test_can 节点启动，CAN 接口 %s，ID=0x%03X，发送 %.1f Hz",
                can_interface_.c_str(), can_id_, send_rate_hz_);
    RCLCPP_INFO(get_logger(), "接收: %s（历史保留 %zu 条，含自身回环: %s）",
                rx_enabled_ ? "已启用" : "已关闭", rx_history_max_,
                rx_own_msgs_ ? "是" : "否");
    RCLCPP_INFO(get_logger(), "网页面板: http://<本机IP>:%d", http_port_);
  }

  ~CanSenderNode() override
  {
    stopRxThread();
    stopHttpServer();
    closeSocket();
  }

private:
  // ==================== CAN socket ====================
  void closeSocket()
  {
    if (sock_fd_ >= 0) {
      ::close(sock_fd_);
      sock_fd_ = -1;
    }
  }

  bool openSocket()
  {
    sock_fd_ = ::socket(PF_CAN, SOCK_RAW | SOCK_CLOEXEC, CAN_RAW);
    if (sock_fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "创建 CAN socket 失败: %s", std::strerror(errno));
      return false;
    }

    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';
    if (::ioctl(sock_fd_, SIOCGIFINDEX, &ifr) < 0) {
      RCLCPP_ERROR(get_logger(), "找不到 CAN 接口 %s: %s（请确认已 ip link set up）",
                   can_interface_.c_str(), std::strerror(errno));
      closeSocket();
      return false;
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (::bind(sock_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
      RCLCPP_ERROR(get_logger(), "绑定 %s 失败: %s", can_interface_.c_str(), std::strerror(errno));
      closeSocket();
      return false;
    }

    RCLCPP_INFO(get_logger(), "SocketCAN 打开成功: %s，帧 ID=0x%03X，发送 %.1f Hz",
                can_interface_.c_str(), can_id_, send_rate_hz_);
    return true;
  }

  bool writeFrame(const struct can_frame & frame)
  {
    const ssize_t nbytes = ::write(sock_fd_, &frame, sizeof(frame));
    if (nbytes < 0) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                            "CAN 发送失败 (ID=0x%03X): %s（接口可能 DOWN，将尝试重连）",
                            frame.can_id, std::strerror(errno));
      closeSocket();
      return false;
    }
    return true;
  }

  // ==================== 定时发送 ====================
  void createSendTimer()
  {
    if (send_timer_) {
      send_timer_->cancel();
    }
    const double hz = std::max(0.1, send_rate_hz_);
    const auto period = std::chrono::milliseconds(
      std::max(1, static_cast<int>(std::lround(1000.0 / hz))));
    send_timer_ = create_wall_timer(period, std::bind(&CanSenderNode::onSendTimer, this));
  }

  void onSendTimer()
  {
    bool once = false;
    {
      std::lock_guard<std::mutex> lk(send_once_mutex_);
      if (send_once_pending_) {
        send_once_pending_ = false;
        once = true;
      }
    }
    if (once) {
      sendAll(/*force=*/true);
      return;
    }
    if (!paused_) {
      sendAll(/*force=*/false);
    }
  }

  // 发送当前所有启用的帧；force=true 时忽略暂停状态（用于「只发一次」）
  void sendAll(bool force)
  {
    if (!running_) {
      return;
    }
    if (sock_fd_ < 0) {
      const auto now = std::chrono::steady_clock::now();
      if (now - last_open_attempt_ < std::chrono::seconds(2)) {
        return;
      }
      last_open_attempt_ = now;
      if (!openSocket()) {
        return;
      }
    }

    std::vector<WebFrame> snapshot;
    {
      std::lock_guard<std::mutex> lk(frames_mutex_);
      snapshot = frames_;
    }

    for (const auto & wf : snapshot) {
      if (!wf.enabled) {
        continue;
      }
      struct can_frame frame;
      std::memset(&frame, 0, sizeof(frame));
      frame.can_id = can_id_;
      frame.can_dlc = static_cast<__u8>(std::min<size_t>(wf.dlc, kMaxDlc));
      for (size_t i = 0; i < frame.can_dlc; ++i) {
        frame.data[i] = wf.data[i];
      }
      if (!writeFrame(frame)) {
        return;
      }
      if (force) {
        RCLCPP_INFO(get_logger(), "单次发送 0x%03X: %s", can_id_,
                    frameToString(frame).c_str());
      }
    }

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "%s", lastSentText().c_str());
  }

  static std::string frameToString(const struct can_frame & frame)
  {
    std::string s;
    for (size_t i = 0; i < frame.can_dlc; ++i) {
      if (i) {
        s += ' ';
      }
      s += toHexByte(frame.data[i]);
    }
    return s;
  }

  std::string lastSentText()
  {
    std::lock_guard<std::mutex> lk(frames_mutex_);
    std::string s = "ID=0x" + toHexByte(static_cast<uint8_t>(can_id_ & 0xFF)) + " 发送中: ";
    bool first = true;
    for (size_t i = 0; i < frames_.size(); ++i) {
      const auto & f = frames_[i];
      if (!f.enabled) {
        continue;
      }
      if (!first) {
        s += " | ";
      }
      first = false;
      s += "帧" + std::to_string(i) + "[";
      for (size_t b = 0; b < f.dlc; ++b) {
        if (b) {
          s += ' ';
        }
        s += toHexByte(f.data[b]);
      }
      s += "]";
    }
    if (first) {
      s += "(没有启用任何帧)";
    }
    return s;
  }

  // ==================== CAN 接收 ====================
  // 接收使用独立的 socket，避免与发送侧的失败重连逻辑互相干扰。
  // 不设置 can_filter，因此默认接收总线上的全部帧（0x000~0x7FF 标准帧 + 扩展帧）。
  void closeRxSocket()
  {
    if (rx_fd_ >= 0) {
      ::close(rx_fd_);
      rx_fd_ = -1;
    }
  }

  bool openRxSocket()
  {
    closeRxSocket();

    rx_fd_ = ::socket(PF_CAN, SOCK_RAW | SOCK_CLOEXEC, CAN_RAW);
    if (rx_fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "创建 CAN 接收 socket 失败: %s", std::strerror(errno));
      return false;
    }

    // 是否把本节点自己发出的帧也算作接收（本机回环）
    int own = rx_own_msgs_ ? 1 : 0;
    ::setsockopt(rx_fd_, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &own, sizeof(own));

    // 200ms 接收超时，便于循环里检查退出标志
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 200 * 1000;
    ::setsockopt(rx_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';
    if (::ioctl(rx_fd_, SIOCGIFINDEX, &ifr) < 0) {
      RCLCPP_ERROR(get_logger(), "找不到 CAN 接口 %s: %s（请确认已 ip link set up）",
                   can_interface_.c_str(), std::strerror(errno));
      closeRxSocket();
      return false;
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (::bind(rx_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
      RCLCPP_ERROR(get_logger(), "接收 socket 绑定 %s 失败: %s", can_interface_.c_str(),
                   std::strerror(errno));
      closeRxSocket();
      return false;
    }

    RCLCPP_INFO(get_logger(), "SocketCAN 接收已就绪: %s（接收全部 ID）", can_interface_.c_str());
    return true;
  }

  void startRxThread()
  {
    if (rx_thread_.joinable()) {
      return;
    }
    rx_running_ = true;
    rx_thread_ = std::thread([this]() { rxLoop(); });
  }

  void stopRxThread()
  {
    rx_running_ = false;
    if (rx_thread_.joinable()) {
      rx_thread_.join();
    }
    closeRxSocket();
  }

  void rxLoop()
  {
    const auto start = std::chrono::steady_clock::now();
    while (rx_running_ && rclcpp::ok()) {
      if (rx_fd_ < 0) {
        if (!openRxSocket()) {
          // 失败后分片睡眠，保证能及时响应退出
          for (int i = 0; i < 10 && rx_running_; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
          }
          continue;
        }
      }

      struct can_frame frame;
      const ssize_t nbytes = ::read(rx_fd_, &frame, sizeof(frame));
      if (nbytes < 0) {
        if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
          continue;  // 超时或被打断，继续等下一条
        }
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "CAN 接收失败: %s（接口可能 DOWN，将尝试重连）", std::strerror(errno));
        closeRxSocket();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));  // 避免重连空转
        continue;
      }
      if (nbytes < static_cast<ssize_t>(sizeof(frame))) {
        continue;
      }
      if (frame.can_id & CAN_ERR_FLAG) {
        continue;  // 错误帧不参与显示
      }

      RxRecord rec;
      rec.eff = (frame.can_id & CAN_EFF_FLAG) != 0;
      rec.id = rec.eff ? (static_cast<uint32_t>(frame.can_id) & CAN_EFF_MASK)
                       : (static_cast<uint32_t>(frame.can_id) & CAN_SFF_MASK);
      rec.dlc = static_cast<uint8_t>(std::min<size_t>(frame.can_dlc, kMaxDlc));
      for (size_t i = 0; i < static_cast<size_t>(rec.dlc); ++i) {
        rec.data[i] = frame.data[i];
      }
      rec.t = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
      rec.seq = ++rx_seq_;
      pushRx(rec);
    }
  }

  // 更新「按 ID 累计的最新值」并追加历史记录
  void pushRx(const RxRecord & rec)
  {
    std::lock_guard<std::mutex> lk(rx_mutex_);
    rx_recv_count_++;

    auto & entry = rx_latest_[rec.id];
    if (entry.count > 0) {
      const double dt = rec.t - entry.t;
      if (dt > 1e-6) {
        const double inst = 1.0 / dt;  // 瞬时帧率
        entry.hz = (entry.hz > 0.0) ? (entry.hz * 0.7 + inst * 0.3) : inst;
      }
    }
    entry.eff = rec.eff;
    entry.dlc = rec.dlc;
    for (size_t i = 0; i < kMaxDlc; ++i) {
      entry.data[i] = rec.data[i];
    }
    entry.count++;
    entry.t = rec.t;

    rx_history_.push_front(rec);
    while (rx_history_.size() > rx_history_max_) {
      rx_history_.pop_back();
    }

    if (!rec.eff && rec.id == kQuatCanId) {
      decodeQuat(rec);
    }
  }

  // 解算 0x100 四元数报文：data[0..1]=x、data[2..3]=y、data[4..5]=z、data[6..7]=w，
  // 各为大端 int16 除以 1e4；若 x²+y²+z²+w² 与 1 的偏差超过 kQuatTol 则判定无效。
  void decodeQuat(const RxRecord & rec)
  {
    QuatState & q = quat_;
    q.has = true;
    q.valid = false;
    q.dlc = rec.dlc;
    q.t = rec.t;
    q.seq = rec.seq;
    q.count++;

    q.x = q.y = q.z = q.w = 0.0;
    q.norm2 = 0.0;
    q.yaw = q.pitch = q.roll = 0.0;

    if (rec.dlc < kMaxDlc) {
      q.invalid_count++;  // 数据长度不足 8 字节，无法解算
      return;
    }

    q.x = toInt16(&rec.data[0]) / 1e4;
    q.y = toInt16(&rec.data[2]) / 1e4;
    q.z = toInt16(&rec.data[4]) / 1e4;
    q.w = toInt16(&rec.data[6]) / 1e4;

    q.norm2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (std::abs(q.norm2 - 1.0) > kQuatTol) {
      q.invalid_count++;
      return;
    }
    q.valid = true;

    // 内旋 Z-Y-X（yaw 绕 Z、pitch 绕 Y、roll 绕 X）欧拉角，单位：度
    const double r2d = 180.0 / kPi;
    q.roll = std::atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y));
    q.pitch = std::asin(std::max(-1.0, std::min(1.0, 2.0 * (q.w * q.y - q.z * q.x))));
    q.yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    q.roll *= r2d;
    q.pitch *= r2d;
    q.yaw *= r2d;
  }

  // 接收数据序列化：latest 为按 ID 的最新值，history 为最近收到的帧（最新在前）
  std::string rxJson()
  {
    std::lock_guard<std::mutex> lk(rx_mutex_);

    auto emitHex = [](std::ostringstream & os, const uint8_t * data) {
      os << "\"hex\":[";
      for (size_t b = 0; b < kMaxDlc; ++b) {
        if (b) {
          os << ",";
        }
        os << "\"" << toHexByte(data[b]) << "\"";
      }
      os << "]";
    };
    auto emitBin = [](std::ostringstream & os, const uint8_t * data) {
      os << "\"bin\":[";
      for (size_t b = 0; b < kMaxDlc; ++b) {
        if (b) {
          os << ",";
        }
        os << "\"" << toBinByte(data[b]) << "\"";
      }
      os << "]";
    };

    std::ostringstream os;
    os << "{";
    os << "\"enabled\":" << (rx_enabled_ ? "true" : "false");
    os << ",\"can_open\":" << (rx_fd_ >= 0 ? "true" : "false");
    os << ",\"can_interface\":\"" << can_interface_ << "\"";
    os << ",\"own_msgs\":" << (rx_own_msgs_ ? "true" : "false");
    os << ",\"recv_count\":" << rx_recv_count_.load();
    os << ",\"id_count\":" << rx_latest_.size();
    os << ",\"history_max\":" << rx_history_max_;

    os << ",\"latest\":[";
    bool first = true;
    for (const auto & kv : rx_latest_) {
      if (!first) {
        os << ",";
      }
      first = false;
      const auto & e = kv.second;
      os << "{\"id\":" << kv.first;
      os << ",\"id_hex\":\"" << idToHex(kv.first, e.eff) << "\"";
      os << ",\"eff\":" << (e.eff ? "true" : "false");
      os << ",\"dlc\":" << static_cast<unsigned>(e.dlc);
      os << ",\"count\":" << e.count;
      os << ",\"hz\":" << round2(e.hz);
      os << ",\"t\":" << round2(e.t);
      os << ",\"bytes\":[";
      for (size_t b = 0; b < kMaxDlc; ++b) {
        if (b) {
          os << ",";
        }
        os << static_cast<unsigned>(e.data[b]);
      }
      os << "],";
      emitHex(os, e.data);
      os << ",";
      emitBin(os, e.data);
      os << "}";
    }
    os << "]";

    os << ",\"history\":[";
    first = true;
    for (const auto & r : rx_history_) {
      if (!first) {
        os << ",";
      }
      first = false;
      os << "{\"seq\":" << r.seq;
      os << ",\"id\":" << r.id;
      os << ",\"id_hex\":\"" << idToHex(r.id, r.eff) << "\"";
      os << ",\"eff\":" << (r.eff ? "true" : "false");
      os << ",\"dlc\":" << static_cast<unsigned>(r.dlc);
      os << ",\"t\":" << round2(r.t);
      os << ",\"bytes\":[";
      for (size_t b = 0; b < kMaxDlc; ++b) {
        if (b) {
          os << ",";
        }
        os << static_cast<unsigned>(r.data[b]);
      }
      os << "],";
      emitHex(os, r.data);
      os << ",";
      emitBin(os, r.data);
      os << "}";
    }
    os << "]";

    // 0x100 四元数解算结果
    os << ",\"quat\":{";
    os << "\"can_id\":" << kQuatCanId;
    os << ",\"can_id_hex\":\"" << idToHex(kQuatCanId, false) << "\"";
    os << ",\"tol\":" << kQuatTol;
    os << ",\"has\":" << (quat_.has ? "true" : "false");
    os << ",\"valid\":" << (quat_.valid ? "true" : "false");
    os << ",\"dlc\":" << static_cast<unsigned>(quat_.dlc);
    os << ",\"count\":" << quat_.count;
    os << ",\"invalid_count\":" << quat_.invalid_count;
    os << ",\"seq\":" << quat_.seq;
    os << ",\"t\":" << round2(quat_.t);
    os << ",\"x\":" << round4(quat_.x);
    os << ",\"y\":" << round4(quat_.y);
    os << ",\"z\":" << round4(quat_.z);
    os << ",\"w\":" << round4(quat_.w);
    os << ",\"norm2\":" << round4(quat_.norm2);
    os << ",\"yaw\":" << round2(quat_.yaw);
    os << ",\"pitch\":" << round2(quat_.pitch);
    os << ",\"roll\":" << round2(quat_.roll);
    os << "}";

    os << "}";
    return os.str();
  }

  void clearRxData()
  {
    std::lock_guard<std::mutex> lk(rx_mutex_);
    rx_latest_.clear();
    rx_history_.clear();
    rx_recv_count_ = 0;
    quat_ = QuatState{};
  }

  // ==================== 网页数据序列化 ====================
  std::string framesJson()
  {
    std::lock_guard<std::mutex> lk(frames_mutex_);
    std::ostringstream os;
    os << "{";
    os << "\"can_id\":" << static_cast<unsigned>(can_id_ & 0x7FF);
    os << ",\"can_id_hex\":\"0x" << toHexByte(static_cast<uint8_t>(can_id_ & 0xFF)) << "\"";
    os << ",\"can_interface\":\"" << can_interface_ << "\"";
    os << ",\"send_rate_hz\":" << send_rate_hz_;
    os << ",\"paused\":" << (paused_ ? "true" : "false");
    os << ",\"running\":" << (running_ ? "true" : "false");
    os << ",\"can_open\":" << (sock_fd_ >= 0 ? "true" : "false");
    os << ",\"sent_count\":" << sent_count_;
    os << ",\"frames\":[";
    for (size_t i = 0; i < frames_.size(); ++i) {
      const auto & f = frames_[i];
      if (i) {
        os << ",";
      }
      os << "{\"enabled\":" << (f.enabled ? "true" : "false");
      os << ",\"dlc\":" << static_cast<unsigned>(f.dlc);
      os << ",\"bytes\":[";
      for (size_t b = 0; b < kMaxDlc; ++b) {
        if (b) {
          os << ",";
        }
        os << static_cast<unsigned>(f.data[b]);
      }
      os << "],\"hex\":[";
      for (size_t b = 0; b < kMaxDlc; ++b) {
        if (b) {
          os << ",";
        }
        os << "\"" << toHexByte(f.data[b]) << "\"";
      }
      os << "],\"bin\":[";
      for (size_t b = 0; b < kMaxDlc; ++b) {
        if (b) {
          os << ",";
        }
        os << "\"" << toBinByte(f.data[b]) << "\"";
      }
      os << "]}";
    }
    os << "]}";
    return os.str();
  }

  // ==================== 「设置帧」的极简 JSON 解析 ====================
  // 只解析本页面自己发出来的固定结构，不做通用 JSON 库依赖：
  // {"frames":[{"enabled":true,"dlc":8,"bytes":[0,1,...]}, ...]}
  static std::string stripSpace(const std::string & s)
  {
    std::string out;
    out.reserve(s.size());
    for (char c : s) {
      if (c != ' ' && c != '\n' && c != '\r' && c != '\t') {
        out += c;
      }
    }
    return out;
  }

  // 从 pos 起找到匹配的配对括号，返回内部子串
  static bool sliceObject(const std::string & s, size_t open, size_t & close)
  {
    if (open >= s.size() || s[open] != '{') {
      return false;
    }
    int depth = 0;
    for (size_t i = open; i < s.size(); ++i) {
      if (s[i] == '{') {
        ++depth;
      } else if (s[i] == '}') {
        --depth;
        if (depth == 0) {
          close = i;
          return true;
        }
      }
    }
    return false;
  }

  static bool parseBoolField(const std::string & obj, const std::string & key, bool & out)
  {
    const std::string pat = "\"" + key + "\":";
    const auto p = obj.find(pat);
    if (p == std::string::npos) {
      return false;
    }
    out = obj.compare(p + pat.size(), 4, "true") == 0;
    return true;
  }

  static bool parseIntField(const std::string & obj, const std::string & key, int & out)
  {
    const std::string pat = "\"" + key + "\":";
    const auto p = obj.find(pat);
    if (p == std::string::npos) {
      return false;
    }
    try {
      size_t used = 0;
      out = std::stoi(obj.substr(p + pat.size()), &used, 10);
      return used > 0;
    } catch (const std::exception &) {
      return false;
    }
  }

  bool applyFramesJson(const std::string & raw, std::string & err)
  {
    const std::string s = stripSpace(raw);
    const auto arr_key = s.find("\"frames\":[");
    if (arr_key == std::string::npos) {
      err = "缺少 frames 字段";
      return false;
    }
    const size_t arr_begin = arr_key + std::strlen("\"frames\":[");
    const auto arr_end = s.rfind(']');
    if (arr_end == std::string::npos || arr_end < arr_begin) {
      err = "frames 数组格式错误";
      return false;
    }
    const std::string arr = s.substr(arr_begin, arr_end - arr_begin);

    std::vector<WebFrame> parsed;
    size_t pos = 0;
    while (pos < arr.size() && parsed.size() < kMaxFrames) {
      const auto obj_begin = arr.find('{', pos);
      if (obj_begin == std::string::npos) {
        break;
      }
      size_t obj_end = 0;
      if (!sliceObject(arr, obj_begin, obj_end)) {
        break;
      }
      const std::string obj = arr.substr(obj_begin, obj_end - obj_begin + 1);

      WebFrame wf;
      parseBoolField(obj, "enabled", wf.enabled);
      int dlc = 8;
      if (parseIntField(obj, "dlc", dlc)) {
        dlc = std::max(0, std::min(static_cast<int>(kMaxDlc), dlc));
        wf.dlc = static_cast<uint8_t>(dlc);
      }

      const auto bytes_key = obj.find("\"bytes\":[");
      if (bytes_key != std::string::npos) {
        const size_t b_begin = bytes_key + std::strlen("\"bytes\":[");
        const auto b_end = obj.find(']', b_begin);
        if (b_end != std::string::npos) {
          const std::string list = obj.substr(b_begin, b_end - b_begin);
          size_t p = 0;
          size_t idx = 0;
          while (p < list.size() && idx < kMaxDlc) {
            const auto comma = list.find(',', p);
            const std::string item =
              list.substr(p, comma == std::string::npos ? std::string::npos : comma - p);
            p = (comma == std::string::npos) ? list.size() : comma + 1;
            if (item.empty()) {
              continue;
            }
            try {
              const int v = std::stoi(item, nullptr, 10);
              wf.data[idx] = static_cast<uint8_t>(std::max(0, std::min(255, v)));
            } catch (const std::exception &) {
              wf.data[idx] = 0;
            }
            ++idx;
          }
        }
      }
      parsed.push_back(wf);
      pos = obj_end + 1;
    }

    if (parsed.empty()) {
      err = "至少需要保留一帧";
      return false;
    }
    std::lock_guard<std::mutex> lk(frames_mutex_);
    frames_ = parsed;
    return true;
  }

  // ==================== 内置 HTTP 服务 ====================
  static const char * pageHtml();

  void startHttpServer()
  {
    listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "创建 HTTP socket 失败: %s", std::strerror(errno));
      return;
    }
    int on = 1;
    ::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));

    struct sockaddr_in addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(static_cast<uint16_t>(http_port_));
    if (::bind(listen_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
      RCLCPP_ERROR(get_logger(), "HTTP 绑定端口 %d 失败: %s", http_port_, std::strerror(errno));
      ::close(listen_fd_);
      listen_fd_ = -1;
      return;
    }
    if (::listen(listen_fd_, 16) < 0) {
      RCLCPP_ERROR(get_logger(), "HTTP listen 失败: %s", std::strerror(errno));
      ::close(listen_fd_);
      listen_fd_ = -1;
      return;
    }

    http_thread_ = std::thread([this]() { httpLoop(); });
  }

  void stopHttpServer()
  {
    if (listen_fd_ >= 0) {
      ::shutdown(listen_fd_, SHUT_RDWR);
      ::close(listen_fd_);
      listen_fd_ = -1;
    }
    if (http_thread_.joinable()) {
      http_thread_.join();
    }
  }

  void httpLoop()
  {
    while (listen_fd_ >= 0) {
      const int client = ::accept(listen_fd_, nullptr, nullptr);
      if (client < 0) {
        if (errno == EINTR) {
          continue;
        }
        break;
      }
      handleClient(client);
      ::close(client);
    }
  }

  static bool readRequest(int fd, std::string & method, std::string & path, std::string & body)
  {
    std::string req;
    char buf[4096];
    size_t header_end = std::string::npos;
    size_t content_length = 0;

    while (true) {
      const ssize_t n = ::recv(fd, buf, sizeof(buf), 0);
      if (n <= 0) {
        break;
      }
      req.append(buf, static_cast<size_t>(n));
      header_end = req.find("\r\n\r\n");
      if (header_end != std::string::npos) {
        const std::string head = req.substr(0, header_end);
        const auto cl = head.find("Content-Length:");
        if (cl == std::string::npos) {
          const auto cl2 = head.find("content-length:");
          if (cl2 != std::string::npos) {
            content_length = static_cast<size_t>(std::stoul(head.substr(cl2 + 15)));
          }
        } else {
          content_length = static_cast<size_t>(std::stoul(head.substr(cl + 15)));
        }
        if (req.size() >= header_end + 4 + content_length) {
          break;
        }
      }
      if (req.size() > (1u << 20)) {
        break;
      }
    }

    if (header_end == std::string::npos) {
      return false;
    }

    const std::string head = req.substr(0, header_end);
    const auto sp1 = head.find(' ');
    const auto sp2 = head.find(' ', sp1 + 1);
    if (sp1 == std::string::npos || sp2 == std::string::npos) {
      return false;
    }
    method = head.substr(0, sp1);
    path = head.substr(sp1 + 1, sp2 - sp1 - 1);

    const size_t body_begin = header_end + 4;
    if (body_begin < req.size()) {
      body = req.substr(body_begin);
    }
    if (body.size() < content_length) {
      while (body.size() < content_length) {
        const ssize_t n = ::recv(fd, buf, sizeof(buf), 0);
        if (n <= 0) {
          break;
        }
        body.append(buf, static_cast<size_t>(n));
      }
    }
    return true;
  }

  static void sendAllFd(int fd, const std::string & data)
  {
    size_t sent = 0;
    while (sent < data.size()) {
      const ssize_t n = ::send(fd, data.data() + sent, data.size() - sent, MSG_NOSIGNAL);
      if (n <= 0) {
        break;
      }
      sent += static_cast<size_t>(n);
    }
  }

  static std::string urlDecode(const std::string & in)
  {
    std::string out;
    out.reserve(in.size());
    for (size_t i = 0; i < in.size(); ++i) {
      if (in[i] == '%' && i + 2 < in.size()) {
        try {
          out += static_cast<char>(std::stoi(in.substr(i + 1, 2), nullptr, 16));
          i += 2;
          continue;
        } catch (const std::exception &) {
        }
      }
      if (in[i] == '+') {
        out += ' ';
      } else {
        out += in[i];
      }
    }
    return out;
  }

  HttpResponse route(const std::string & method, const std::string & path,
                     const std::string & body)
  {
    HttpResponse r;
    const auto qpos = path.find('?');
    const std::string route_path = qpos == std::string::npos ? path : path.substr(0, qpos);
    const std::string query = qpos == std::string::npos ? std::string() : path.substr(qpos + 1);

    auto queryValue = [&query](const std::string & key) -> std::string {
      std::string pat = key + "=";
      const auto p = query.find(pat);
      if (p == std::string::npos) {
        return std::string();
      }
      const auto amp = query.find('&', p);
      return urlDecode(query.substr(p + pat.size(),
                                    amp == std::string::npos ? std::string::npos : amp - p - pat.size()));
    };

    if (route_path == "/" || route_path == "/index.html") {
      r.ctype = "text/html; charset=utf-8";
      r.body = pageHtml();
      return r;
    }

    if (route_path == "/api/frames") {
      r.ctype = "application/json; charset=utf-8";
      r.body = framesJson();
      return r;
    }

    if (route_path == "/api/rx") {
      r.ctype = "application/json; charset=utf-8";
      r.body = rxJson();
      return r;
    }

    if (route_path == "/api/set" && method == "POST") {
      std::string err;
      const bool ok = applyFramesJson(body, err);
      r.ctype = "application/json; charset=utf-8";
      r.body = ok ? std::string("{\"ok\":true,\"state\":") + framesJson() + "}"
                  : std::string("{\"ok\":false,\"msg\":\"") + err + "\"}";
      return r;
    }

    if (route_path == "/api/action") {
      const std::string act = queryValue("do");
      r.ctype = "application/json; charset=utf-8";
      if (act == "pause") {
        paused_ = true;
      } else if (act == "resume") {
        paused_ = false;
      } else if (act == "once") {
        std::lock_guard<std::mutex> lk(send_once_mutex_);
        send_once_pending_ = true;
      } else if (act == "rate") {
        double hz = 0.0;
        try {
          hz = std::stod(queryValue("hz"));
        } catch (const std::exception &) {
          hz = 0.0;
        }
        if (hz >= 0.1 && hz <= 1000.0) {
          send_rate_hz_ = hz;
          createSendTimer();
        }
      } else if (act == "clear_rx") {
        clearRxData();
      } else {
        r.body = "{\"ok\":false,\"msg\":\"unknown action\"}";
        return r;
      }
      r.body = std::string("{\"ok\":true,\"state\":") + framesJson() + "}";
      return r;
    }

    r.code = 404;
    r.body = "not found";
    return r;
  }

  void handleClient(int fd)
  {
    std::string method, path, body;
    if (!readRequest(fd, method, path, body)) {
      return;
    }
    const HttpResponse r = route(method, path, body);

    std::ostringstream os;
    os << "HTTP/1.1 " << r.code << " " << (r.code == 200 ? "OK" : "ERR") << "\r\n";
    os << "Content-Type: " << r.ctype << "\r\n";
    os << "Content-Length: " << r.body.size() << "\r\n";
    os << "Cache-Control: no-store\r\n";
    os << "Connection: close\r\n\r\n";
    os << r.body;
    sendAllFd(fd, os.str());
  }

  // ==================== 成员 ====================
  std::string can_interface_;
  canid_t can_id_{0xFF};
  double send_rate_hz_{100.0};
  int http_port_{8080};

  // 帧配置
  std::mutex frames_mutex_;
  std::vector<WebFrame> frames_;

  // 发送控制
  std::atomic<bool> running_{true};
  std::atomic<bool> paused_{false};
  std::atomic<uint64_t> sent_count_{0};
  std::mutex send_once_mutex_;
  bool send_once_pending_{false};

  // CAN 发送 socket
  int sock_fd_{-1};
  std::chrono::steady_clock::time_point last_open_attempt_{};

  // 接收配置
  bool rx_enabled_{true};
  bool rx_own_msgs_{false};
  size_t rx_history_max_{64};

  // CAN 接收 socket 与累计数据
  std::atomic<bool> rx_running_{false};
  std::thread rx_thread_;
  int rx_fd_{-1};
  std::mutex rx_mutex_;
  std::map<uint32_t, RxEntry> rx_latest_;   // 每个 ID 的最新值（按 ID 升序）
  std::deque<RxRecord> rx_history_;         // 最近收到的帧，最新的在最前
  std::atomic<uint64_t> rx_recv_count_{0};
  std::atomic<uint64_t> rx_seq_{0};
  QuatState quat_;                          // 0x100 四元数解算结果（受 rx_mutex_ 保护）

  // HTTP
  int listen_fd_{-1};
  std::thread http_thread_;

  // ROS
  rclcpp::TimerBase::SharedPtr send_timer_;
};

// ==================== 网页 UI ====================
const char * CanSenderNode::pageHtml()
{
  return R"RAW(<!DOCTYPE html>
<html lang="zh-CN"><head><meta charset="utf-8">
<title>CAN 帧调试面板 (ID 0xFF)</title>
<style>
  body{font-family:system-ui,Segoe UI,sans-serif;background:#1b1d21;color:#d7dae0;margin:0;padding:16px}
  h2{margin:0 0 4px;font-size:18px}
  .sub{color:#8b919b;font-size:12px;margin-bottom:12px}
  .bar{display:flex;flex-wrap:wrap;gap:8px;align-items:center;margin-bottom:12px;
       background:#24272c;padding:10px;border-radius:8px}
  button{background:#33383f;color:#dfe3e8;border:1px solid #4a5058;border-radius:6px;
         padding:5px 12px;cursor:pointer;font-size:13px}
  button:hover{background:#3d434b}
  button.green{background:#1d5c2a;border-color:#2f8f45;color:#dfffe6}
  button.red{background:#5c1d1d;border-color:#8f2f2f;color:#ffe0e0}
  button.blue{background:#1d3d5c;border-color:#2f6f8f;color:#dfeeff}
  .frame{background:#24272c;border:1px solid #34383e;border-radius:8px;
         padding:10px 12px;margin-bottom:10px}
  .fhead{display:flex;align-items:center;gap:10px;margin-bottom:8px;flex-wrap:wrap}
  .fname{font-weight:600;color:#8cf}
  .bytes{display:flex;flex-wrap:wrap;gap:8px}
  .byte{background:#1b1d21;border:1px solid #33383f;border-radius:6px;padding:6px 8px;
        display:flex;flex-direction:column;align-items:center;gap:4px}
  .byte .lbl{font-size:11px;color:#7f8792}
  .bits{display:flex;gap:2px}
  .bit{width:20px;height:24px;line-height:24px;text-align:center;font:12px monospace;
       background:#2b2f35;border:1px solid #3d434b;border-radius:4px;cursor:pointer;
       user-select:none;color:#8b919b}
  .bit.on{background:#1d5c2a;border-color:#2f8f45;color:#b8ffcb;font-weight:700}
  .hexin{width:52px;text-align:center;font:12px monospace;background:#111;color:#8cf;
         border:1px solid #444;border-radius:4px;padding:2px}
  .dec{font-size:11px;color:#c96}
  input[type=number]{background:#111;color:#8cf;border:1px solid #444;border-radius:4px;
                     padding:3px 6px;font:12px monospace}
  .hint{font-size:12px;color:#7f8792;margin-top:6px}
  #status{font-size:12px;color:#8c8}
  #preview{font:12px monospace;color:#9f9;background:#111;border:1px solid #333;
           border-radius:6px;padding:6px 10px;margin-bottom:12px;white-space:pre-wrap}
  .rxcard{background:#24272c;border:1px solid #34383e;border-radius:8px;
          padding:10px 12px;margin-bottom:10px}
  .rxlbl{font-size:12px;color:#8b919b}
  .bit.ro{cursor:default}
  .bit.ro.on{background:#1d5c2a;border-color:#2f8f45;color:#b8ffcb;font-weight:700}
  #rx_filter{background:#111;color:#8cf;border:1px solid #444;border-radius:4px;
             padding:3px 6px;font:12px monospace;width:180px}
  #rx_status{font-size:12px;color:#8c8}
  #rx_hist_wrap{max-height:320px;overflow:auto;border:1px solid #333;
                border-radius:6px;background:#111}
  table#rx_hist{border-collapse:collapse;width:100%;font:12px monospace}
  table#rx_hist th{position:sticky;top:0;background:#24272c;color:#8b919b;font-weight:600;
                   text-align:left;padding:4px 8px;border-bottom:1px solid #333}
  table#rx_hist td{padding:2px 8px;border-bottom:1px solid #222;color:#cbd;white-space:nowrap}
  table#rx_hist td.idc{color:#8cf}
  table#rx_hist td.binc{color:#9f9}
  .qgrid{display:flex;flex-wrap:wrap;gap:8px}
  .qitem{background:#1b1d21;border:1px solid #33383f;border-radius:6px;padding:6px 12px;
         display:flex;flex-direction:column;align-items:center;gap:2px;min-width:100px}
  .qitem .k{font-size:11px;color:#7f8792}
  .qitem .v{font:14px monospace;color:#8cf;white-space:nowrap}
  .qitem .v.bad{color:#e88}
</style></head>
<body>
<h2>CAN 帧调试面板</h2>
<div class="sub">CAN_ID = <b id="cid">0xFF</b> &nbsp;|&nbsp; 接口 <b id="iface">can0</b> &nbsp;|&nbsp;
  点击比特位即可切换 0/1，也可直接输入 HEX</div>

<div class="bar">
  <button class="green" id="btn_resume">继续发送</button>
  <button class="red" id="btn_pause">暂停发送</button>
  <button class="blue" id="btn_once">只发一次</button>
  <button id="btn_add">+ 新增一帧</button>
  <span style="color:#8b919b;font-size:13px">速率</span>
  <input type="number" id="rate" min="0.1" max="1000" step="0.1" value="100"> Hz
  <button id="btn_rate">应用速率</button>
  <span id="status">连接中...</span>
</div>

<div id="preview">等待首次发送...</div>
<div id="frames"></div>

<h2 style="margin-top:22px">CAN 接收</h2>
<div class="sub">接收 <b id="rx_iface">can0</b> 上 0x000~0x7FF 的全部标准帧（含扩展帧），
  按 ID 累计显示每一帧的最新内容；下方为最近收到的帧历史，最新的在最前</div>
<div class="bar">
  <span style="color:#8b919b;font-size:13px">ID 过滤</span>
  <input id="rx_filter" placeholder="留空=全部，如 1A2 或 0x1A2">
  <label style="font-size:13px;color:#8b919b"><input type="checkbox" id="rx_hold"> 暂停刷新</label>
  <button class="red" id="btn_rx_clear">清空接收</button>
  <span id="rx_status">接收中...</span>
</div>
<div id="rx_latest"></div>
<div class="sub" style="margin-top:12px">历史（最近 <span id="rx_hist_max">-</span> 条）</div>
<div id="rx_hist_wrap">
  <table id="rx_hist">
    <thead><tr><th>#</th><th>t(s)</th><th>ID</th><th>DLC</th><th>HEX</th>
      <th>BIN（每位 0/1）</th></tr></thead>
    <tbody id="rx_hist_body"></tbody>
  </table>
</div>

<div class="sub" style="margin-top:16px">0x100 四元数解算（x/y/z/w = 大端 int16 ÷ 1e4，
  校验 x²+y²+z²+w² ≈ 1，容差 ±0.01）</div>
<div class="rxcard">
  <div class="fhead">
    <span class="fname">ID 0x100</span>
    <span class="rxlbl" id="quat_status">等待数据...</span>
  </div>
  <div class="qgrid" id="quat_vals"></div>
  <div class="rxlbl" id="quat_euler" style="margin-top:8px"></div>
</div>

<script>
var MAXDLC=8;
var state=null;
var pending=null;

function bitsToByte(bits){var v=0;for(var i=0;i<8;i++){v=(v<<1)|(bits[i]?1:0)}return v}
function byteToBits(v){var a=[];for(var i=7;i>=0;i--)a.push(((v>>i)&1)===1);return a}

function render(){
  if(!state)return;
  document.getElementById('cid').textContent=state.can_id_hex;
  document.getElementById('iface').textContent=state.can_interface;
  document.getElementById('rate').value=state.send_rate_hz;
  var st=document.getElementById('status');
  st.textContent=(state.can_open?'CAN 已打开':'CAN 未就绪')+' · '+
    (state.paused?'已暂停':'发送中')+' · '+state.send_rate_hz+' Hz · 累计 '+
    state.sent_count+' 帧';
  st.style.color=state.can_open?'#8c8':'#e88';
  var host=document.getElementById('frames');
  host.innerHTML='';
  state.frames.forEach(function(f,fi){
    var box=document.createElement('div');box.className='frame';
    var head=document.createElement('div');head.className='fhead';
    var cb=document.createElement('input');cb.type='checkbox';cb.checked=f.enabled;
    cb.addEventListener('change',function(){f.enabled=cb.checked;push()});
    var name=document.createElement('span');name.className='fname';
    name.textContent='帧 '+fi;
    var dlcLab=document.createElement('span');dlcLab.style.fontSize='12px';
    dlcLab.style.color='#8b919b';dlcLab.textContent='DLC';
    var dlc=document.createElement('input');dlc.type='number';dlc.min=0;dlc.max=MAXDLC;
    dlc.value=f.dlc;dlc.style.width='52px';
    dlc.addEventListener('change',function(){
      var v=Math.max(0,Math.min(MAXDLC,parseInt(dlc.value)||0));f.dlc=v;push()});
    var en=document.createElement('span');en.style.fontSize='12px';en.style.color='#8b919b';
    en.textContent=f.enabled?'启用':'停用';
    var del=document.createElement('button');del.textContent='删除';del.className='red';
    del.style.marginLeft='auto';
    del.addEventListener('click',function(){
      state.frames.splice(fi,1);push()});
    head.appendChild(cb);head.appendChild(name);head.appendChild(dlcLab);
    head.appendChild(dlc);head.appendChild(en);head.appendChild(del);
    box.appendChild(head);

    var bytes=document.createElement('div');bytes.className='bytes';
    for(var bi=0;bi<MAXDLC;bi++){
      (function(bi){
        var cell=document.createElement('div');cell.className='byte';
        if(bi>=f.dlc)cell.style.opacity='0.35';
        var lbl=document.createElement('div');lbl.className='lbl';lbl.textContent='byte'+bi;
        var bits=document.createElement('div');bits.className='bits';
        var b=byteToBits(f.bytes[bi]);
        for(var k=0;k<8;k++){
          (function(k){
            var el=document.createElement('div');
            el.className='bit'+(b[k]?' on':'');
            el.textContent=b[k]?'1':'0';
            el.addEventListener('click',function(){
              b[k]=!b[k];f.bytes[bi]=bitsToByte(b);push()});
            bits.appendChild(el);
          })(k);
        }
        var hex=document.createElement('input');hex.className='hexin';
        hex.value=f.hex[bi];
        hex.addEventListener('change',function(){
          var s=hex.value.trim().toUpperCase().replace(/^0X/,'');
          var v=parseInt(s,16);
          if(isNaN(v))v=0;
          v=Math.max(0,Math.min(255,v));
          f.bytes[bi]=v;
          hex.value=('0'+(v.toString(16).toUpperCase())).slice(-2);
          push()});
        var dec=document.createElement('div');dec.className='dec';
        dec.textContent='0x'+f.hex[bi]+' / '+f.bytes[bi];
        cell.appendChild(lbl);cell.appendChild(bits);cell.appendChild(hex);cell.appendChild(dec);
        bytes.appendChild(cell);
      })(bi);
    }
    box.appendChild(bytes);
    host.appendChild(box);
  });
}

var timer=null;
function push(){
  render();
  if(timer)clearTimeout(timer);
  timer=setTimeout(function(){
    fetch('/api/set',{method:'POST',headers:{'Content-Type':'application/json'},
      body:JSON.stringify({frames:state.frames})})
      .then(function(r){return r.json()})
      .then(function(j){if(j.ok){state=j.state;render()}
        else{document.getElementById('status').textContent='拒绝: '+j.msg}});
  },260);
}

function action(name,extra){
  var url='/api/action?do='+name+(extra||'');
  fetch(url).then(function(r){return r.json()}).then(function(j){
    if(j.ok){state=j.state;render()}});
}

function preview(){
  if(!state)return;
  var parts=[];
  state.frames.forEach(function(f,i){
    if(!f.enabled)return;
    parts.push('帧'+i+' ['+f.hex.slice(0,f.dlc).join(' ')+']');
  });
  document.getElementById('preview').textContent=
    'ID '+state.can_id_hex+'  发送: '+(parts.length?parts.join('  |  '):'(没有启用任何帧)');
}

function poll(){
  fetch('/api/frames').then(function(r){return r.json()}).then(function(j){
    state=j;render();preview()}).catch(function(){});
}
setInterval(poll,1000);
poll();

document.getElementById('btn_pause').onclick=function(){action('pause')};
document.getElementById('btn_resume').onclick=function(){action('resume')};
document.getElementById('btn_once').onclick=function(){action('once')};
document.getElementById('btn_rate').onclick=function(){
  var hz=parseFloat(document.getElementById('rate').value);
  if(!isFinite(hz)||hz<0.1||hz>1000){alert('速率范围 0.1 ~ 1000 Hz');return}
  action('rate','&hz='+hz)};
document.getElementById('btn_add').onclick=function(){
  if(!state)return;
  if(state.frames.length>=16){alert('最多 16 帧');return}
  state.frames.push({enabled:true,dlc:8,bytes:[0,0,0,0,0,0,0,0]});
  push()};

// ==================== CAN 接收显示 ====================
var rx=null;

function rxFilterText(){
  return document.getElementById('rx_filter').value.trim().toUpperCase().replace(/^0X/,'');
}
function rxMatch(idHex){
  var f=rxFilterText();
  return !f||idHex.toUpperCase().indexOf(f)>=0;
}
function rxBits(v){var a=[];for(var i=7;i>=0;i--)a.push(((v>>i)&1)===1);return a}

function renderRx(){
  if(!rx)return;
  document.getElementById('rx_iface').textContent=rx.can_interface;
  document.getElementById('rx_hist_max').textContent=rx.history_max;
  var st=document.getElementById('rx_status');
  st.textContent=(rx.enabled?(rx.can_open?'接收中':'接收未就绪'):'接收已关闭')+
    ' · 累计 '+rx.recv_count+' 帧 / '+rx.id_count+' 个 ID'+(rx.own_msgs?' · 含自身回环':'');
  st.style.color=(rx.enabled&&rx.can_open)?'#8c8':'#e88';

  // ---- 上方：按 ID 的最新值 ----
  var host=document.getElementById('rx_latest');
  host.innerHTML='';
  var shown=0;
  (rx.latest||[]).forEach(function(e){
    if(!rxMatch(e.id_hex))return;
    shown++;
    var box=document.createElement('div');box.className='rxcard';
    var head=document.createElement('div');head.className='fhead';
    var name=document.createElement('span');name.className='fname';
    name.textContent='0x'+e.id_hex+(e.eff?' (扩展帧)':'');
    var meta=document.createElement('span');meta.className='rxlbl';
    meta.textContent='DLC '+e.dlc+' · 累计 '+e.count+' 帧 · '+e.hz.toFixed(1)+' Hz · 最近 '+
      e.t.toFixed(3)+'s';
    head.appendChild(name);head.appendChild(meta);
    box.appendChild(head);

    var bytes=document.createElement('div');bytes.className='bytes';
    for(var bi=0;bi<MAXDLC;bi++){
      var cell=document.createElement('div');cell.className='byte';
      if(bi>=e.dlc)cell.style.opacity='0.35';
      var lbl=document.createElement('div');lbl.className='lbl';lbl.textContent='byte'+bi;
      var bits=document.createElement('div');bits.className='bits';
      var b=rxBits(e.bytes[bi]);
      for(var k=0;k<8;k++){
        var el=document.createElement('div');
        el.className='bit ro'+(b[k]?' on':'');
        el.textContent=b[k]?'1':'0';
        bits.appendChild(el);
      }
      var dec=document.createElement('div');dec.className='dec';
      dec.textContent='0x'+e.hex[bi]+' / '+e.bytes[bi];
      cell.appendChild(lbl);cell.appendChild(bits);cell.appendChild(dec);
      bytes.appendChild(cell);
    }
    box.appendChild(bytes);
    host.appendChild(box);
  });
  if(shown===0){
    host.innerHTML='<div class="hint">'+
      (rxFilterText()?'没有匹配该 ID 的接收帧':'还没有收到任何帧')+'</div>';
  }

  // ---- 下方：历史滚动列表 ----
  var tb=document.getElementById('rx_hist_body');
  tb.innerHTML='';
  var rows=0;
  (rx.history||[]).forEach(function(r){
    if(!rxMatch(r.id_hex))return;
    rows++;
    var tr=document.createElement('tr');
    var td=document.createElement('td');td.textContent=r.seq;tr.appendChild(td);
    td=document.createElement('td');td.textContent=r.t.toFixed(3);tr.appendChild(td);
    td=document.createElement('td');td.className='idc';td.textContent='0x'+r.id_hex;tr.appendChild(td);
    td=document.createElement('td');td.textContent=r.dlc;tr.appendChild(td);
    td=document.createElement('td');td.textContent=r.hex.slice(0,r.dlc).join(' ');tr.appendChild(td);
    td=document.createElement('td');td.className='binc';
    td.textContent=r.bin.slice(0,r.dlc).join(' ');tr.appendChild(td);
    tb.appendChild(tr);
  });
  if(rows===0){
    var tr=document.createElement('tr');
    var td=document.createElement('td');
    td.colSpan=6;td.className='hint';
    td.textContent=rxFilterText()?'（没有匹配该 ID 的接收帧）':'（暂无接收记录）';
    tr.appendChild(td);tb.appendChild(tr);
  }
}

function rxPoll(){
  if(document.getElementById('rx_hold').checked)return;
  fetch('/api/rx').then(function(r){return r.json()}).then(function(j){
    rx=j;renderRx();renderQuat()}).catch(function(){});
}

// ---- 0x100 四元数解算 ----
function renderQuat(){
  if(!rx)return;
  var q=rx.quat;
  var st=document.getElementById('quat_status');
  var vals=document.getElementById('quat_vals');
  var eu=document.getElementById('quat_euler');
  if(!q||!q.has){
    st.textContent='尚未收到 0x100 帧';
    st.style.color='#e88';
    vals.innerHTML='';
    eu.textContent='';
    return;
  }
  st.textContent=(q.valid?'模长校验有效':'模长校验无效（已丢弃）')+
    ' · 累计 '+q.count+' 帧（无效 '+q.invalid_count+'）· 最近 '+q.t.toFixed(3)+
    's · 序号 '+q.seq+' · DLC '+q.dlc;
  st.style.color=q.valid?'#8c8':'#e88';

  var items=[['x',q.x,'byte0-1'],['y',q.y,'byte2-3'],['z',q.z,'byte4-5'],['w',q.w,'byte6-7']];
  vals.innerHTML='';
  items.forEach(function(it){
    var d=document.createElement('div');d.className='qitem';
    var k=document.createElement('div');k.className='k';k.textContent=it[0]+' ('+it[2]+')';
    var v=document.createElement('div');v.className='v'+(q.valid?'':' bad');
    v.textContent=it[1].toFixed(4);
    d.appendChild(k);d.appendChild(v);
    vals.appendChild(d);
  });
  var nd=document.createElement('div');nd.className='qitem';
  var k2=document.createElement('div');k2.className='k';k2.textContent='x²+y²+z²+w²';
  var v2=document.createElement('div');v2.className='v'+(q.valid?'':' bad');
  v2.textContent=q.norm2.toFixed(4)+' (偏差 '+(q.norm2-1).toFixed(4)+' / ±'+q.tol+')';
  nd.appendChild(k2);nd.appendChild(v2);
  vals.appendChild(nd);

  if(q.valid){
    eu.textContent='欧拉角 ZYX（度）　Yaw(Z) '+q.yaw.toFixed(2)+
      '　Pitch(Y) '+q.pitch.toFixed(2)+'　Roll(X) '+q.roll.toFixed(2);
  }else{
    eu.textContent='模长校验未通过，不计算欧拉角';
  }
}

document.getElementById('btn_rx_clear').onclick=function(){
  fetch('/api/action?do=clear_rx').then(function(){return rxPoll()});
};
document.getElementById('rx_filter').addEventListener('input',renderRx);
rxPoll();
setInterval(rxPoll,500);
</script>
</body></html>)RAW";
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanSenderNode>());
  rclcpp::shutdown();
  return 0;
}
