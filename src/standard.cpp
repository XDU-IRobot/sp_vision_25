#include <fmt/core.h>
#include <chrono>
#include <thread>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

// ROS2 headers (仅在 ROS2 可用时编译，用于可视化)
#ifdef AMENT_CMAKE_FOUND
#include <rclcpp/rclcpp.hpp>
#include "tools/ros2_visualizer.hpp"
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

#ifdef AMENT_CMAKE_FOUND
  // 初始化ROS2可视化
  rclcpp::init(argc, argv);
  auto visualizer = std::make_shared<tools::ROS2Visualizer>("standard_node", "standard_markers");
  tools::logger()->info("[ROS2] Visualizer initialized");
#endif

  io::CBoard cboard(config_path);
  io::Camera camera(config_path);

  auto_aim::YOLO detector(config_path, false);  // 启用调试，显示检测窗口
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);
  auto_aim::multithread::CommandGener commandgener(shooter, aimer, cboard, plotter, true);
  // 🎯 所有模块初始化完成，启动相机触发
  tools::logger()->info("=== All modules initialized ===");

#ifdef AMENT_CMAKE_FOUND
  // 发布静态TF: gimbal -> camera（使用标定参数）
  visualizer->publish_static_tf("gimbal", "camera",
    solver.R_camera2gimbal(),
    solver.t_camera2gimbal());
  tools::logger()->info("[ROS2] Published static TF: gimbal -> camera");
#endif

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

      // ==================== 基于 count 硬同步（使用环形数组） ====================
      // 核心思想：相机由MCU硬触发,每来一帧图像，IMU计数器+10
      static const int64_t frame_id_to_imu_offset = 0;  // 🔧 手动调试参数

      static bool first_frame = true;

      frame_id = camera.get_last_frame_id();  // 获取相机帧号
      if(frame_id-frame_id_last!=0){
      trigger_imu_count = 0;
      if (trigger_imu_count < 0) trigger_imu_count += 10000;
      //使用环形数组O(1)查询IMU数据
      auto imu_result = cboard.get_imu_from_ring_buffer(0);

      if (imu_result.valid) {
        // 环形数组查询成功
        q = imu_result.q;  // 四元数
        t = imu_result.timestamp;
        std::cout<<q<<std::endl;

#ifdef AMENT_CMAKE_FOUND
        // 发布动态TF: world -> gimbal（使用MCU四元数）
        auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
          t.time_since_epoch()).count();
        rclcpp::Time ros_time(ns);
        Eigen::Vector3d zero_trans(0, 0, 0);  // world和gimbal原点重合
        visualizer->publish_dynamic_tf("world", "gimbal", q, zero_trans, ros_time);
#endif
      } else {

      }
    mode = cboard.mode;
    frame_id_last=frame_id;
     }
    if (last_mode != mode) {
      tools::logger()->info("Switch to {}", io::MODES[mode]);
      last_mode = mode;
    }
    // recorder.record(img, q, t);
    solver.set_R_gimbal2world(q);
    Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    auto armors = detector.detect(img);
    auto targets = tracker.track(armors, t);

    // 调试：打印 armors 和 targets 数量
    fmt::print("[DEBUG] armors={}, targets={}\n", armors.size(), targets.size());

#ifdef AMENT_CMAKE_FOUND
    // 发布装甲板Marker（可视化检测结果）
    if (!armors.empty()) {
      auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        t.time_since_epoch()).count();
      rclcpp::Time ros_time(ns);

      visualization_msgs::msg::MarkerArray marker_array;
      int marker_id = 0;
      for (const auto & armor : armors) {
        auto marker = visualizer->create_sphere_marker(
          "world", "armors", marker_id++,
          armor.xyz_in_world.x(),
          armor.xyz_in_world.y(),
          armor.xyz_in_world.z(),
          1.0, 0.0, 0.0, 0.8,  // 红色，80%不透明
          0.1,  // 10cm直径
          ros_time
        );
        marker_array.markers.push_back(marker);
      }
      visualizer->publish_marker_array(marker_array);
    }
#endif

    auto command = aimer.aim(targets, t, cboard.bullet_speed, true);  // to_now=true，生成当前时刻命令
    command.shoot = shooter.shoot(command, aimer, targets, ypr);

    // 发送命令到 PlotJuggler（显示实际发送的值）
    nlohmann::json plot_data;
    plot_data["t"] = std::chrono::duration<double>(t - std::chrono::steady_clock::time_point()).count();
    plot_data["cmd_yaw"] = command.yaw * 180.0 / M_PI;
    plot_data["cmd_pitch"] = -command.pitch * 180.0 / M_PI;  // 取反，匹配实际发送值
    plot_data["control"] = command.control;
    plot_data["shoot"] = command.shoot;
    // 电控的欧拉角（从MCU获取的姿态）
    plot_data["mcu_yaw"] = ypr[0] * 180.0 / M_PI;
    plot_data["mcu_pitch"] = ypr[1] * 180.0 / M_PI;
    plot_data["mcu_roll"] = ypr[2] * 180.0 / M_PI;
    plotter.plot(plot_data);

    cboard.send(command);
    
    cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    // 相机输出为 RGB 格式，imshow 需要 BGR 格式
    cv::Mat img_bgr;
    cv::cvtColor(img, img_bgr, cv::COLOR_RGB2BGR);
    cv::imshow("reprojection", img_bgr);
    auto key = cv::waitKey(1);  
    if (key == 'q') break;
  }
  
  // 清理 ROS2
#ifdef AMENT_CMAKE_FOUND
  visualizer.reset();
  rclcpp::shutdown();
  tools::logger()->info("[ROS2] Shutdown complete");
#endif

  return 0;
}