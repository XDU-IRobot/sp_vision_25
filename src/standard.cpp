#include <fmt/core.h>
#include <chrono>
#include <thread>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

// ROS2 headers (仅在 ROS2 可用时编译，用于初始化节点)
#ifdef AMENT_CMAKE_FOUND
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#endif

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/multithread/commandgener.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

using namespace std::chrono;

const std::string keys =
  "{help h usage ? |      | 输出命令行参数说明}"
  "{@config-path   | configs/standard3.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
    tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder;
  cv::CommandLineParser cli(argc, argv, keys);

  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }



  io::CBoard cboard(config_path);
  io::Camera camera(config_path);

  auto_aim::YOLO detector(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);
  auto_aim::multithread::CommandGener commandgener(shooter, aimer, cboard, plotter, false);
  // 🎯 所有模块初始化完成，启动相机触发
  tools::logger()->info("=== All modules initialized ===");
  tools::logger()->info("=== Entering main loop ===");

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  auto mode = io::Mode::idle;
  auto last_mode = io::Mode::idle;

  // 帧率统计
  int frame_count = 0;
  auto fps_start_time = std::chrono::steady_clock::now();
  double current_fps = 0.0;

  // 🆕 调试信息输出计数器（独立于FPS计数）
  int debug_frame_count = 0;

  // 性能分析计时器
  std::chrono::steady_clock::time_point t_start, t_end;
  std::map<std::string, double> timing_stats;  // 存储各步骤耗时统计

  // 🆕 同步匹配相关变量（需要在循环外声明，以便后续日志使用）
  uint64_t frame_id = 0;
  uint16_t current_imu_count = 0;
  int64_t trigger_imu_count = 0;

  // 🔧 同步方式选择：true = 基于时间戳 | false = 基于 count 硬同步
  bool use_timestamp_sync = false;  // 🆕 启用硬同步方案（使用环形数组）
  while (!exiter.exit()) {
    t_start = std::chrono::steady_clock::now();

    camera.read(img, t);
    t_end = std::chrono::steady_clock::now();
    double t_camera = std::chrono::duration<double, std::milli>(t_end - t_start).count();

    t_start = std::chrono::steady_clock::now();

    // 🔧 IMU 同步方式选择
    Eigen::Quaterniond q;
    std::chrono::steady_clock::time_point synced_imu_timestamp;

      // ==================== 基于 count 硬同步（使用环形数组） ====================
      // 核心思想：相机由MCU硬触发，IMU时间戳 = 相机时间戳
      //
      // MCU逻辑：当 imu_count % 10 == 0 时硬触发相机
      // 映射关系：camera frame_id N → trigger_imu_count = (N+1) × 10 + offset
      //
      static const int64_t frame_id_to_imu_offset = 0;  // 🔧 手动调试参数

      static bool first_frame = true;

      frame_id = camera.get_last_frame_id();  // 获取相机帧号
      // 计算当前帧对应的触发点IMU计数（加上手动偏移量）
      trigger_imu_count = (((frame_id + 1) * 10) + frame_id_to_imu_offset) % 10000;
      if (trigger_imu_count < 0) trigger_imu_count += 10000;
      // 🆕 使用环形数组O(1)查询IMU数据
      auto imu_result = cboard.get_imu_from_ring_buffer(trigger_imu_count);

      if (imu_result.valid) {
        // ✅ 环形数组查询成功
        q = imu_result.q;  // 四元数


        t = imu_result.timestamp;  // 🔑 相机时间戳继承自 MCU

      } else {
      }
    mode = cboard.mode;

    if (last_mode != mode) {
      tools::logger()->info("Switch to {}", io::MODES[mode]);
      last_mode = mode;
    }

    // recorder.record(img, q, t);
    solver.set_R_gimbal2world(q);
    auto armors = detector.detect(img);
    auto targets = tracker.track(armors, synced_imu_timestamp);
    auto command = aimer.aim(targets, synced_imu_timestamp, cboard.bullet_speed, false);
    cboard.send(command);
  }
  // 清理 ROS2
#ifdef AMENT_CMAKE_FOUND
  rclcpp::shutdown();
#endif

  return 0;
}