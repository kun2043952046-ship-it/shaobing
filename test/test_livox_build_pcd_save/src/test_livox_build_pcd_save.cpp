/**
 * test_livox_build_pcd_save.cpp
 *
 * 订阅 FAST-LIO 输出的世界坐标系配准点云 (/cloud_registered)，
 * 从收到第一帧开始采集 scan_duration 秒，拼接成全局地图，
 * 以二进制 PCD 保存到 output_dir（Navigation 目录），然后自动退出。
 *
 * 运行示例：
 *   ros2 run test_livox_build_pcd_save test_livox_build_pcd_save_node \
 *     --ros-args -p scan_duration:=10.0 \
 *     -p output_dir:=/home/lele/source_code_from_git/ws_livox/src/Navigation
 */
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace fs = std::filesystem;

class MapCollectorNode : public rclcpp::Node
{
public:
  MapCollectorNode() : Node("test_livox_build_pcd_save")
  {
    // ---------------- 参数（均可命令行覆盖） ----------------
    cloud_topic_   = declare_parameter<std::string>("cloud_topic", "/cloud_registered");
    scan_duration_ = declare_parameter<double>("scan_duration", 10.0);
    output_dir_    = declare_parameter<std::string>(
      "output_dir", "/home/lele/source_code_from_git/ws_livox/src/Navigation");

    RCLCPP_INFO(get_logger(), "配置: topic=%s, 采集时长=%.1fs, 输出目录=%s",
                cloud_topic_.c_str(), scan_duration_, output_dir_.c_str());

    // 订阅 FAST-LIO 的配准点云。
    // SensorDataQoS(默认 best_effort) 能兼容各种发布端 QoS，避免 QoS 不匹配收不到数据
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) { cloudCallback(msg); });

    // 5 秒还没收到数据时的提醒（方便排查驱动/FAST-LIO 是否启动）
    wait_timer_ = create_wall_timer(std::chrono::seconds(5), [this]() {
      if (!started_)
        RCLCPP_WARN(get_logger(),
          "尚未收到 %s 上的点云，请确认 livox 驱动与 FAST-LIO 已启动、话题名正确",
          cloud_topic_.c_str());
    });
  }

private:
  static bool hasField(const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name)
  {
    for (const auto & f : cloud.fields)
      if (f.name == name) return true;
    return false;
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (finished_) return;

    if (!started_) {
      started_ = true;
      start_time_ = std::chrono::steady_clock::now();
      wait_timer_->cancel();
      RCLCPP_INFO(get_logger(), "收到第一帧点云 (frame_id=%s)，开始采集 %.1f 秒...",
                  msg->header.frame_id.c_str(), scan_duration_);
    }

    // ---------- 累积本帧（FAST-LIO 输出为 float32 的 x/y/z[/intensity]） ----------
    if (!(hasField(*msg, "x") && hasField(*msg, "y") && hasField(*msg, "z"))) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "点云缺少 x/y/z 字段，跳过本帧");
      return;
    }

    const bool has_i = hasField(*msg, "intensity");
    sensor_msgs::PointCloud2ConstIterator<float> ix(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iy(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iz(*msg, "z");
    if (has_i) {
      sensor_msgs::PointCloud2ConstIterator<float> ii(*msg, "intensity");
      for (; ix != ix.end(); ++ix, ++iy, ++iz, ++ii) {
        if (std::isfinite(*ix) && std::isfinite(*iy) && std::isfinite(*iz))
          map_.push_back({*ix, *iy, *iz, *ii});
      }
    } else {
      for (; ix != ix.end(); ++ix, ++iy, ++iz) {
        if (std::isfinite(*ix) && std::isfinite(*iy) && std::isfinite(*iz))
          map_.push_back({*ix, *iy, *iz, 0.0f});
      }
    }
    ++frames_;

    // ---------- 时间到：保存并退出 ----------
    const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start_time_).count();
    if (elapsed >= scan_duration_) saveMap(elapsed);
  }

  void saveMap(double elapsed)
  {
    finished_ = true;
    const size_t n = map_.size();
    if (n == 0) {
      RCLCPP_ERROR(get_logger(), "采集结束但地图为空，未保存");
      scheduleExit();
      return;
    }

    // Navigation 目录不存在则自动创建
    std::error_code ec;
    fs::create_directories(output_dir_, ec);
    if (ec) {
      RCLCPP_ERROR(get_logger(), "创建目录失败: %s (%s)",
                   output_dir_.c_str(), ec.message().c_str());
      scheduleExit();
      return;
    }

    const std::string path = output_dir_ + "/map_" + timeStamp() + ".pcd";
    RCLCPP_INFO(get_logger(), "采集完成: %.1fs / %zu 帧 / %zu 点，正在保存: %s",
                elapsed, frames_, n, path.c_str());

    std::ofstream ofs(path, std::ios::binary | std::ios::trunc);
    if (!ofs) {
      RCLCPP_ERROR(get_logger(), "无法打开文件 %s", path.c_str());
      scheduleExit();
      return;
    }

    // 标准 PCD v0.7 二进制头，字段顺序/尺寸与 MapPoint 内存布局一致
    ofs << "# .PCD v0.7 - Point Cloud Data file format\n"
        << "VERSION 0.7\n"
        << "FIELDS x y z intensity\n"
        << "SIZE 4 4 4 4\n"
        << "TYPE F F F F\n"
        << "COUNT 1 1 1 1\n"
        << "WIDTH " << n << "\n"
        << "HEIGHT 1\n"
        << "VIEWPOINT 0 0 0 1 0 0 0\n"
        << "POINTS " << n << "\n"
        << "DATA binary\n";
    ofs.write(reinterpret_cast<const char *>(map_.data()),
              static_cast<std::streamsize>(n * sizeof(MapPoint)));
    ofs.close();

    if (ofs.good())
      RCLCPP_INFO(get_logger(), "PCD 保存成功 (%.1f MB)，程序即将退出",
                  n * sizeof(MapPoint) / 1024.0 / 1024.0);
    else
      RCLCPP_ERROR(get_logger(), "PCD 写入出错");
    scheduleExit();
  }

  // 用定时器在回调之外安全地结束 spin
  void scheduleExit()
  {
    exit_timer_ = create_wall_timer(std::chrono::milliseconds(200), []() {
      rclcpp::shutdown();
    });
  }

  static std::string timeStamp()
  {
    const std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_r(&t, &tm);  // POSIX 接口，Linux 下可用（Windows 编译需换 localtime_s）
    char buf[20];
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm);
    return std::string(buf);
  }

  struct MapPoint { float x, y, z, intensity; };  // 16 字节，无填充
  static_assert(sizeof(MapPoint) == 16, "MapPoint 布局必须与 PCD 头一致");

  // ---------- 成员 ----------
  std::string cloud_topic_;
  double scan_duration_{10.0};
  std::string output_dir_;

  std::vector<MapPoint> map_;
  size_t frames_{0};
  bool started_{false};
  bool finished_{false};
  std::chrono::steady_clock::time_point start_time_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr wait_timer_;
  rclcpp::TimerBase::SharedPtr exit_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapCollectorNode>());
  rclcpp::shutdown();
  return 0;
}
