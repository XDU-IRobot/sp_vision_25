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
  auto_aim::multithread::CommandGener commandgener(shooter, aimer, cboard, plotter, true);
  // 🎯 所有模块初始化完成，启动相机触发
  tools::logger()->info("=== All modules initialized ===");
  tools::logger()->info("=== Entering main loop ===");

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  auto mode = io::Mode::idle;
  auto last_mode = io::Mode::idle;

  // 性能分析计时器
  std::chrono::steady_clock::time_point t_start, t_end;

  // 🆕 同步匹配相关变量（需要在循环外声明，以便后续日志使用）
  uint64_t frame_id = 0;
  uint64_t frame_id_last =0;
  int64_t trigger_imu_count = 0;

  while (!exiter.exit()) {
    camera.read(img, t);

    // 🔧 IMU 同步方式选择
    Eigen::Quaterniond q;

      // ==================== 基于 count 硬同步（使用环形数组） ====================
      // 核心思想：相机由MCU硬触发,每来一帧图像，IMU计数器+10
      static const int64_t frame_id_to_imu_offset = 0;  // 🔧 手动调试参数

      static bool first_frame = true;

      frame_id = camera.get_last_frame_id();  // 获取相机帧号
     if(frame_id-frame_id_last!=0){
      trigger_imu_count = 0;
      if (trigger_imu_count < 0) trigger_imu_count += 10000;
      // 🆕 使用环形数组O(1)查询IMU数据
      auto imu_result = cboard.get_imu_from_ring_buffer(0);

      if (imu_result.valid) {
        // ✅ 环形数组查询成功
        q = imu_result.q;  // 四元数
        t = imu_result.timestamp;  
        
      } else {
      }
    mode = cboard.mode;
    frame_id_last=frame_id;
     }
    if (last_mode != mode) {
      tools::logger()->info("Switch to {}", io::MODES[mode]);
      last_mode = mode;
    }
    recorder.record(img, q, t);
    solver.set_R_gimbal2world(q);
    auto armors = detector.detect(img);
    auto targets = tracker.track(armors, t);
    auto command = aimer.aim(targets, t, cboard.bullet_speed);
    cboard.send(command);
  } 
  // 清理 ROS2
#ifdef AMENT_CMAKE_FOUND
  rclcpp::shutdown();
#endif

  return 0;
}