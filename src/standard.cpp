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
  
  cv::CommandLineParser cli(argc, argv, keys);

  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder;

  io::CBoard cboard(config_path);
  io::Camera camera(config_path);

  auto_aim::YOLO detector(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);
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

    if (use_timestamp_sync) {
    } else {
      // ==================== 方案B：基于 count 硬同步（使用环形数组） ====================
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
    }

    t_end = std::chrono::steady_clock::now();
    double t_imu = std::chrono::duration<double, std::milli>(t_end - t_start).count();

    mode = cboard.mode;

    if (last_mode != mode) {
      tools::logger()->info("Switch to {}", io::MODES[mode]);
      last_mode = mode;
    }

    // recorder.record(img, q, t);

    t_start = std::chrono::steady_clock::now();
    solver.set_R_gimbal2world(q);
    t_end = std::chrono::steady_clock::now();
    double t_solver_setup = std::chrono::duration<double, std::milli>(t_end - t_start).count();


    Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    t_start = std::chrono::steady_clock::now();
    auto armors = detector.detect(img);
    t_end = std::chrono::steady_clock::now();
    double t_detect = std::chrono::duration<double, std::milli>(t_end - t_start).count();

    t_start = std::chrono::steady_clock::now();
    // 🔑 使用硬同步的 IMU 时间戳（与 TF/Marker 严格一致）
    auto targets = tracker.track(armors, synced_imu_timestamp);
    t_end = std::chrono::steady_clock::now();
    double t_track = std::chrono::duration<double, std::milli>(t_end - t_start).count();

    t_start = std::chrono::steady_clock::now();
    // 🔑 使用硬同步的 IMU 时间戳（与 TF/Marker 严格一致）
    // to_now=false：不补偿处理延迟，使用触发时刻的状态（与 TF 时间一致）

    auto command = aimer.aim(targets, synced_imu_timestamp, cboard.bullet_speed, false);
    t_end = std::chrono::steady_clock::now();
    double t_aim = std::chrono::duration<double, std::milli>(t_end - t_start).count();

    t_start = std::chrono::steady_clock::now();
     command.pitch -= 0.10;
    command.yaw +=0;
    cboard.send(command);
    t_end = std::chrono::steady_clock::now();
    double t_send = std::chrono::duration<double, std::milli>(t_end - t_start).count();

    // 帧率计算
    frame_count++;
    debug_frame_count++;  // 🆕 独立计数器
    auto fps_current_time = std::chrono::steady_clock::now();
    auto fps_elapsed = std::chrono::duration<double>(fps_current_time - fps_start_time).count();

    if (fps_elapsed >= 1.0) {  // 每秒更新一次帧率
      current_fps = frame_count / fps_elapsed;
      frame_count = 0;
      fps_start_time = fps_current_time;
    }

    /// 重投影可视化 (类似auto_aim_debug_mpc)
    t_start = std::chrono::steady_clock::now();
    if (!targets.empty()) {
      auto target = targets.front();

      // 绘制所有装甲板（绿色）
      std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
      for (const Eigen::Vector4d & xyza : armor_xyza_list) {
        auto image_points =
          solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {0, 255, 0});
      }

      // 绘制瞄准点（红色）
      if (command.control && aimer.debug_aim_point.valid) {
        Eigen::Vector4d aim_xyza = aimer.debug_aim_point.xyza;
        auto image_points =
          solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {0, 0, 255});
      }
    }
    

  }

  // 清理 ROS2
#ifdef AMENT_CMAKE_FOUND
  rclcpp::shutdown();
#endif

  return 0;
}