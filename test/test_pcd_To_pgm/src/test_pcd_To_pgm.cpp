/**
 * @file  test_pcd_To_pgm.cpp
 * @brief 将 PCD 点云转换为 2D 占据栅格地图（PGM + YAML），可直接用于 nav2 map_server
 *
 * 转换流程：
 *   1. 读取 PCD 点云（兼容 XYZ / XYZI / XYZRGB 等所有含 x,y,z 字段的类型）；
 *   2. 按高度区间 [zmin, zmax] 过滤，剔除地面点与天花板/高处噪声点；
 *   3. 把剩余点垂直投影到 X-Y 平面，按分辨率栅格化，有障碍点的栅格记为占据；
 *   4. （可选）用圆形结构元素对障碍物做膨胀，补偿机器人本体半径；
 *   5. 按 nav2 约定输出 P5 二进制 PGM 与同名 YAML：
 *        像素 0   -> 占据（黑）
 *        像素 205 -> 未知（灰）
 *        像素 254 -> 空闲（白）
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <utility>
#include <vector>
#include <numeric>


#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// 像素值约定（与 nav2 map_server / map_saver 保持一致）
static constexpr uint8_t PIXEL_OCCUPIED = 0;   // 占据：黑
static constexpr uint8_t PIXEL_FREE = 254;     // 空闲：白
static constexpr uint8_t PIXEL_UNKNOWN = 205;  // 未知：灰

// 地图像素总数上限（防呆：分辨率设得过小会把内存撑爆）
static constexpr long long MAX_PIXELS = 200000000LL;

struct Params
{
  std::string input_pcd = "map.pcd";
  std::string output_pgm = "map.pgm";
  std::string output_yaml;  // 为空时自动取 output_pgm 主名 + ".yaml"
  double resolution = 0.05; // 地图分辨率（米/像素）
  double z_min = 0.10;      // 高度过滤下限：剔除地面
  double z_max = 2.00;      // 高度过滤上限：剔除天花板/高处噪声
  bool use_custom_range = false;
  double xmin = 0.0, ymin = 0.0, xmax = 0.0, ymax = 0.0;
  bool unknown_as_free = false; // 无点区域渲染为白(空闲)还是灰(未知)
  int dilate_cells = 0;         // 障碍物膨胀半径（像素）
};

static void printUsage(const char * prog)
{
  std::cout <<
    "用法: " << prog << " [选项]\n"
    "  -i, --input <file>      输入 PCD 路径（默认: map.pcd）\n"
    "  -o, --output <file>     输出 PGM 路径（默认: map.pgm，并自动生成同名 .yaml）\n"
    "  -r, --resolution <m>    地图分辨率，米/像素（默认: 0.05）\n"
    "  --zmin <m>              高度过滤下限，剔除地面点（默认: 0.10）\n"
    "  --zmax <m>              高度过滤上限，剔除高处点（默认: 2.00）\n"
    "  --range <xmin> <ymin> <xmax> <ymax>\n"
    "                          手动指定地图范围（米），默认按点云包围盒自动计算\n"
    "  --free                  无点区域输出为空闲(白 254)，默认为未知(灰 205)\n"
    "  --dilate <cells>        障碍物膨胀半径，单位像素（默认: 0）\n"
    "  -h, --help              显示本帮助\n";
}

static bool parseArgs(int argc, char ** argv, Params & p)
{
  bool ok = true;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];

    // 读取当前选项后紧跟的参数值；不存在则报错
    auto value = [&](const char * opt) -> std::string {
        if (i + 1 >= argc) {
          std::cerr << "错误: " << opt << " 缺少参数值\n";
          ok = false;
          return {};
        }
        return std::string(argv[++i]);
      };

    if (arg == "-h" || arg == "--help") {
      printUsage(argv[0]);
      std::exit(0);
    } else if (arg == "-i" || arg == "--input") {
      p.input_pcd = value("--input");
    } else if (arg == "-o" || arg == "--output") {
      p.output_pgm = value("--output");
    } else if (arg == "-r" || arg == "--resolution") {
      p.resolution = std::stod(value("--resolution"));
    } else if (arg == "--zmin") {
      p.z_min = std::stod(value("--zmin"));
    } else if (arg == "--zmax") {
      p.z_max = std::stod(value("--zmax"));
    } else if (arg == "--range") {
      p.xmin = std::stod(value("--range xmin"));
      p.ymin = std::stod(value("--range ymin"));
      p.xmax = std::stod(value("--range xmax"));
      p.ymax = std::stod(value("--range ymax"));
      p.use_custom_range = true;
    } else if (arg == "--free") {
      p.unknown_as_free = true;
    } else if (arg == "--dilate") {
      p.dilate_cells = std::stoi(value("--dilate"));
    } else {
      std::cerr << "未知选项: " << arg << "\n";
      ok = false;
    }

    if (!ok) {
      printUsage(argv[0]);
      return false;
    }
  }
  return true;
}

// 读取任意带 x/y/z 字段的 PCD，统一转成 PointXYZ（其余字段自动丢弃）
static bool loadCloud(const std::string & file, pcl::PointCloud<pcl::PointXYZ> & cloud)
{
  pcl::PCLPointCloud2 blob;
  if (pcl::io::loadPCDFile(file, blob) < 0) {
    std::cerr << "错误: 无法读取 PCD 文件: " << file << "\n";
    return false;
  }
  try {
    pcl::fromPCLPointCloud2(blob, cloud);
  } catch (const std::exception & e) {
    std::cerr << "错误: PCD 不含 x/y/z 字段或格式不兼容: " << e.what() << "\n";
    return false;
  }
  return true;
}

// 输出 z 高度统计和 ASCII 直方图
static void printZHistogram(
  const pcl::PointCloud<pcl::PointXYZ> & cloud,
  double bin_size = 0.10)
{
  if (bin_size <= 0.0) {
    std::cerr << "错误: z 直方图区间宽度必须大于 0\n";
    return;
  }

  std::vector<double> z_values;
  z_values.reserve(cloud.points.size());

  for (const auto & pt : cloud.points) {
    if (!std::isfinite(pt.z)) {
      continue;
    }

    z_values.push_back(static_cast<double>(pt.z));
  }

  if (z_values.empty()) {
    std::cerr << "错误: 点云中没有有效的 z 高度数据\n";
    return;
  }

  // 排序后可以计算分位数
  std::sort(z_values.begin(), z_values.end());

  const auto getPercentile =
    [&](double ratio) -> double
    {
      if (ratio <= 0.0) {
        return z_values.front();
      }
      if (ratio >= 1.0) {
        return z_values.back();
      }

      const std::size_t index = static_cast<std::size_t>(
        ratio * static_cast<double>(z_values.size() - 1));

      return z_values[index];
    };

  const double z_min = z_values.front();
  const double z_max = z_values.back();
  const double z_mean =
    std::accumulate(z_values.begin(), z_values.end(), 0.0) /
    static_cast<double>(z_values.size());

  const double z_p01 = getPercentile(0.01);
  const double z_p05 = getPercentile(0.05);
  const double z_p25 = getPercentile(0.25);
  const double z_p50 = getPercentile(0.50);
  const double z_p75 = getPercentile(0.75);
  const double z_p95 = getPercentile(0.95);
  const double z_p99 = getPercentile(0.99);

  std::cout << "\n========== 点云 z 高度统计 ==========\n";
  std::cout << std::fixed << std::setprecision(3);
  std::cout << "有效点数量: " << z_values.size() << "\n";
  std::cout << "z 最小值:   " << z_min << " m\n";
  std::cout << "z 最大值:   " << z_max << " m\n";
  std::cout << "z 平均值:   " << z_mean << " m\n";
  std::cout << "z 1% 分位:  " << z_p01 << " m\n";
  std::cout << "z 5% 分位:  " << z_p05 << " m\n";
  std::cout << "z 25% 分位: " << z_p25 << " m\n";
  std::cout << "z 50% 分位: " << z_p50 << " m\n";
  std::cout << "z 75% 分位: " << z_p75 << " m\n";
  std::cout << "z 95% 分位: " << z_p95 << " m\n";
  std::cout << "z 99% 分位: " << z_p99 << " m\n";

  // 直方图只显示 1%～99% 范围，避免极端异常值拉大显示范围
  const double display_min = std::floor(z_p01 / bin_size) * bin_size;
  const double display_max = std::ceil(z_p99 / bin_size) * bin_size;

  if (display_max <= display_min) {
    std::cout << "z 高度范围过小，无法生成直方图\n";
    return;
  }

  const std::size_t bin_count = static_cast<std::size_t>(
    std::ceil((display_max - display_min) / bin_size));

  std::vector<std::size_t> histogram(bin_count, 0);

  std::size_t below_count = 0;
  std::size_t above_count = 0;

  for (const double z : z_values) {
    if (z < display_min) {
      ++below_count;
      continue;
    }

    if (z >= display_max) {
      ++above_count;
      continue;
    }

    const std::size_t index = static_cast<std::size_t>(
      (z - display_min) / bin_size);

    if (index < histogram.size()) {
      ++histogram[index];
    }
  }

  const std::size_t max_count =
    *std::max_element(histogram.begin(), histogram.end());

  std::cout << "\n========== z 高度直方图 ==========\n";
  std::cout << "区间宽度: " << bin_size << " m\n";

  if (below_count > 0) {
    std::cout << "低于显示范围的点: "
              << below_count << " 个，占 "
              << 100.0 * static_cast<double>(below_count) /
                 static_cast<double>(z_values.size())
              << "%\n";
  }

  if (above_count > 0) {
    std::cout << "高于显示范围的点: "
              << above_count << " 个，占 "
              << 100.0 * static_cast<double>(above_count) /
                 static_cast<double>(z_values.size())
              << "%\n";
  }

  for (std::size_t i = 0; i < histogram.size(); ++i) {
    const double begin_z =
      display_min + static_cast<double>(i) * bin_size;
    const double end_z = begin_z + bin_size;

    const std::size_t count = histogram[i];

    // 将最长柱状条限制为 50 个字符
    const std::size_t bar_length =
      max_count == 0 ? 0 : count * 50 / max_count;

    std::cout << "["
              << std::setw(7) << std::setprecision(2) << begin_z
              << ", "
              << std::setw(7) << std::setprecision(2) << end_z
              << ") "
              << std::setw(10) << count
              << " 个点 "
              << std::setw(7) << std::setprecision(2)
              << 100.0 * static_cast<double>(count) /
                 static_cast<double>(z_values.size())
              << "% ";

    for (std::size_t j = 0; j < bar_length; ++j) {
      std::cout << '#';
    }

    std::cout << "\n";
  }

  std::cout << "===================================\n\n";
}


// 写 P5（二进制）PGM
static bool writePGM(
  const std::string & file, int width, int height,
  const std::vector<uint8_t> & pixels)
{
  std::ofstream f(file, std::ios::binary);
  if (!f) {
    std::cerr << "错误: 无法创建文件: " << file << "\n";
    return false;
  }
  f << "P5\n" << width << " " << height << "\n255\n";
  f.write(
    reinterpret_cast<const char *>(pixels.data()),
    static_cast<std::streamsize>(pixels.size()));
  return static_cast<bool>(f);
}

// 写 nav2 map_server 可直接加载的 YAML 描述文件
static bool writeYAML(const Params & p)
{
  std::ofstream f(p.output_yaml);
  if (!f) {
    std::cerr << "错误: 无法创建文件: " << p.output_yaml << "\n";
    return false;
  }
  // image 字段写成相对 YAML 的文件名，便于整个目录拷走
  std::string image_name = p.output_pgm;
  const auto slash = image_name.find_last_of("/\\");
  if (slash != std::string::npos) {
    image_name = image_name.substr(slash + 1);
  }
  f << std::fixed << std::setprecision(4);
  f << "image: " << image_name << "\n";
  f << "resolution: " << p.resolution << "\n";
  f << "origin: [" << p.xmin << ", " << p.ymin << ", 0.0]\n";
  f << "negate: 0\n";
  f << "occupied_thresh: 0.65\n";
  f << "free_thresh: 0.196\n";
  f << "mode: trinary\n";
  return static_cast<bool>(f);
}

int main(int argc, char ** argv)
{
  Params p;
  try {
    if (!parseArgs(argc, argv, p)) {
      return 1;
    }
  } catch (const std::exception & e) {
    std::cerr << "参数解析失败: " << e.what() << "\n";
    return 1;
  }

  if (p.output_yaml.empty()) {
    const auto dot = p.output_pgm.find_last_of('.');
    p.output_yaml = (dot == std::string::npos) ?
      p.output_pgm + ".yaml" :
      p.output_pgm.substr(0, dot) + ".yaml";
  }

  // ---------- 1. 读取点云 ----------
  pcl::PointCloud<pcl::PointXYZ> cloud;
  if (!loadCloud(p.input_pcd, cloud)) {
    return 1;
  }
  std::cout << "点云读取成功: " << cloud.size() << " 个点（" << p.input_pcd << "）\n";
  if (cloud.empty()) {
    std::cerr << "错误: 点云为空\n";
    return 1;
  }
  // 输出原始点云的 z 高度统计和直方图
  // 这里还没有使用 --zmin 和 --zmax，统计的是原始点云
  printZHistogram(cloud, 0.10);

  // ---------- 2. 高度过滤 + 计算地图范围 ----------
  if (!p.use_custom_range) {
    p.xmin = std::numeric_limits<double>::max();
    p.ymin = std::numeric_limits<double>::max();
    p.xmax = std::numeric_limits<double>::lowest();
    p.ymax = std::numeric_limits<double>::lowest();
  }

  std::size_t kept = 0;
  for (const auto & pt : cloud.points) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
      continue;
    }
    if (pt.z < p.z_min || pt.z > p.z_max) {
      continue;  // 剔除地面 / 高处点
    }
    ++kept;
    if (!p.use_custom_range) {
      p.xmin = std::min(p.xmin, static_cast<double>(pt.x));
      p.ymin = std::min(p.ymin, static_cast<double>(pt.y));
      p.xmax = std::max(p.xmax, static_cast<double>(pt.x));
      p.ymax = std::max(p.ymax, static_cast<double>(pt.y));

    }
  }
  if (kept == 0) {
    std::cerr << "错误: 高度过滤 [" << p.z_min << ", " << p.z_max <<
      "] 后没有剩余点，请调整 --zmin/--zmax\n";
    return 1;
  }
  std::cout << "高度过滤后保留: " << kept << " 个点\n";

  // ---------- 3. 计算栅格尺寸 ----------
  const int width = static_cast<int>(std::floor((p.xmax - p.xmin) / p.resolution)) + 1;
  const int height = static_cast<int>(std::floor((p.ymax - p.ymin) / p.resolution)) + 1;
  const long long n_pixels = static_cast<long long>(width) * height;
  if (width <= 0 || height <= 0 || n_pixels <= 0 || n_pixels > MAX_PIXELS) {
    std::cerr << "错误: 地图尺寸异常（" << width << " x " << height <<
      " 像素），请检查 --resolution / --range\n";
    return 1;
  }
  std::cout << "地图范围: x[" << p.xmin << ", " << p.xmax << "]  y[" <<
    p.ymin << ", " << p.ymax << "]\n" <<
    "地图尺寸: " << width << " x " << height << " 像素（分辨率 " <<
    p.resolution << " 米/像素）\n";

  // occupancy[iy * width + ix]：iy = 0 对应 y 最小的最底行
  std::vector<uint8_t> occupancy(static_cast<std::size_t>(width) * height, 0);

  // ---------- 4. 投影栅格化：有障碍点的格子置 1 ----------
  for (const auto & pt : cloud.points) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
      continue;
    }
    if (pt.z < p.z_min || pt.z > p.z_max) {
      continue;
    }
    const int ix = static_cast<int>(std::floor((pt.x - p.xmin) / p.resolution));
    const int iy = static_cast<int>(std::floor((pt.y - p.ymin) / p.resolution));
    if (ix < 0 || ix >= width || iy < 0 || iy >= height) {
      continue;  // 手动指定范围时，丢弃范围外的点
    }
    occupancy[static_cast<std::size_t>(iy) * width + ix] = 1;
  }

  // ---------- 5. 障碍物膨胀（圆形结构元素） ----------
  if (p.dilate_cells > 0) {
    const int r = p.dilate_cells;
    const int r2 = r * r;
    // 先收集所有占据像素，再逐个膨胀，避免全图暴力扫描
    std::vector<std::pair<int, int>> seeds;
    for (int iy = 0; iy < height; ++iy) {
      for (int ix = 0; ix < width; ++ix) {
        if (occupancy[static_cast<std::size_t>(iy) * width + ix]) {
          seeds.emplace_back(ix, iy);
        }
      }
    }
    for (const auto & seed : seeds) {
      const int sx = seed.first;
      const int sy = seed.second;
      const int y0 = std::max(0, sy - r);
      const int y1 = std::min(height - 1, sy + r);
      const int x0 = std::max(0, sx - r);
      const int x1 = std::min(width - 1, sx + r);
      for (int dy = y0; dy <= y1; ++dy) {
        for (int dx = x0; dx <= x1; ++dx) {
          if ((dx - sx) * (dx - sx) + (dy - sy) * (dy - sy) <= r2) {
            occupancy[static_cast<std::size_t>(dy) * width + dx] = 1;
          }
        }
      }
    }
    std::cout << "障碍物已按半径 " << r << " 像素膨胀\n";
  }

  // ---------- 6. 渲染像素并写出 PGM ----------
  // PGM 图像第 0 行在最上方（y 最大），因此要做一次上下翻转
  const uint8_t empty_pixel = p.unknown_as_free ? PIXEL_FREE : PIXEL_UNKNOWN;
  std::vector<uint8_t> pixels(static_cast<std::size_t>(width) * height);
  for (int iy = 0; iy < height; ++iy) {
    const int row = height - 1 - iy;
    for (int ix = 0; ix < width; ++ix) {
      pixels[static_cast<std::size_t>(row) * width + ix] =
        occupancy[static_cast<std::size_t>(iy) * width + ix] ? PIXEL_OCCUPIED
                                                             : empty_pixel;
    }
  }
  if (!writePGM(p.output_pgm, width, height, pixels)) {
    return 1;
  }
  std::cout << "PGM 已写出: " << p.output_pgm << "\n";

  // ---------- 7. 写出配套 YAML ----------
  if (!writeYAML(p)) {
    return 1;
  }
  std::cout << "YAML 已写出: " << p.output_yaml << "\n" <<
    "完成。map.yaml + map.pgm 可直接提供给 nav2 map_server 使用。\n";
  return 0;
}
