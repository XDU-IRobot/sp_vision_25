#include <fmt/core.h>

#include <atomic>
#include <chrono>
#include <cstdlib>  // for setenv
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

// ROS2 headers (仅在 ROS2 可用时编译，用于可视化)
#ifdef AMENT_CMAKE_FOUND
#include <rclcpp/rclcpp.hpp>
#include "tools/ros2_visualizer.hpp"
#endif

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/thread_safe_queue.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | configs/standard3.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  // 🚀 关键性能优化：全局线程控制
  cv::setNumThreads(1);
  setenv("TBB_NUM_THREADS", "4", 1);
  setenv("OMP_NUM_THREADS", "4", 1);
  tools::logger()->info("[Performance] Thread limits: OpenCV=1, TBB=4, OMP=4");

  tools::Exiter exiter;
  tools::Plotter plotter;

  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

#ifdef AMENT_CMAKE_FOUND
  // 初始化ROS2可视化
  rclcpp::init(argc, argv);
  auto visualizer = std::make_shared<tools::ROS2Visualizer>("auto_aim_debug_mpc", "mpc_markers");
  tools::logger()->info("[ROS2] Visualizer initialized");
#endif

  io::CBoard cboard(config_path);
  io::Camera camera(config_path);

  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Planner planner(config_path);

#ifdef AMENT_CMAKE_FOUND
  // 发布静态TF: gimbal -> camera（使用标定参数）
  visualizer->publish_static_tf("gimbal", "camera",
    solver.R_camera2gimbal(),
    solver.t_camera2gimbal());
  tools::logger()->info("[ROS2] Published static TF: gimbal -> camera");
#endif

  tools::ThreadSafeQueue<std::optional<auto_aim::Target>, true> target_queue(1);
  target_queue.push(std::nullopt);

  std::atomic<bool> quit = false;
  auto plan_thread = std::thread([&]() {
    auto t0 = std::chrono::steady_clock::now();

    while (!quit) {
      auto target = target_queue.front();
      auto plan = planner.plan(target, cboard.bullet_speed);

      // 将 Plan 转换为 Command
      io::Command command;
      command.control = plan.control;
      command.shoot = plan.fire;
      command.yaw = plan.yaw;
      command.pitch = plan.pitch;

      cboard.send(command);

      nlohmann::json data;
      data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);

      data["target_yaw"] = plan.target_yaw * 180.0 / M_PI;
      data["target_pitch"] = plan.target_pitch * 180.0 / M_PI;

      data["plan_yaw"] = plan.yaw * 180.0 / M_PI;
      data["plan_yaw_vel"] = plan.yaw_vel * 180.0 / M_PI;
      data["plan_yaw_acc"] = plan.yaw_acc * 180.0 / M_PI;

      data["plan_pitch"] = plan.pitch * 180.0 / M_PI;
      data["plan_pitch_vel"] = plan.pitch_vel * 180.0 / M_PI;
      data["plan_pitch_acc"] = plan.pitch_acc * 180.0 / M_PI;

      data["cmd_yaw"] = command.yaw * 180.0 / M_PI;
      data["cmd_pitch"] = -command.pitch * 180.0 / M_PI;  // 取反，匹配实际发送值
      data["control"] = command.control;
      data["shoot"] = command.shoot;

      if (target.has_value()) {
        data["target_z"] = target->ekf_x()[4];   //z
        data["target_vz"] = target->ekf_x()[5];  //vz
        data["w"] = target->ekf_x()[7];
      } else {
        data["w"] = 0.0;
      }

      plotter.plot(data);

      std::this_thread::sleep_for(10ms);
    }
  });

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  auto mode = io::Mode::idle;
  auto last_mode = io::Mode::idle;

  // 同步匹配相关变量
  uint64_t frame_id = 0;
  uint64_t frame_id_last = 0;
  int64_t trigger_imu_count = 0;

  tools::logger()->info("=== Entering main loop ===");

  while (!exiter.exit()) {
    camera.read(img, t);

    // ==================== 基于 count 硬同步（使用环形数组） ====================
    static const int64_t frame_id_to_imu_offset = 0;  // 🔧 手动调试参数

    frame_id = camera.get_last_frame_id();
    if (frame_id - frame_id_last != 0) {
      trigger_imu_count = frame_id_to_imu_offset;
      if (trigger_imu_count < 0) trigger_imu_count += 10000;

      auto imu_result = cboard.get_imu_from_ring_buffer(trigger_imu_count);

      if (imu_result.valid) {
        q = imu_result.q;
        t = imu_result.timestamp;

        solver.set_R_gimbal2world(q);

#ifdef AMENT_CMAKE_FOUND
        // 发布动态TF: world -> gimbal（使用MCU四元数）
        auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
          t.time_since_epoch()).count();
        rclcpp::Time ros_time(ns);
        Eigen::Vector3d zero_trans(0, 0, 0);
        visualizer->publish_dynamic_tf("world", "gimbal", q, zero_trans, ros_time);
#endif
      } else {
        tools::logger()->warn("IMU data not found for trigger_imu_count={}", trigger_imu_count);
      }

      mode = cboard.mode;
      frame_id_last = frame_id;
    }

    if (last_mode != mode) {
      tools::logger()->info("Switch to {}", io::MODES[mode]);
      last_mode = mode;
    }

    Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);
    auto armors = yolo.detect(img);
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

    if (!targets.empty())
      target_queue.push(targets.front());
    else
      target_queue.push(std::nullopt);

    if (!targets.empty()) {
      auto target = targets.front();

      // 绘制跟踪到的所有装甲板位置（绿色）
      std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
      for (const Eigen::Vector4d & xyza : armor_xyza_list) {
        auto image_points =
          solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {0, 255, 0});
      }

      // 绘制MPC规划的瞄准位置（红色）
      Eigen::Vector4d aim_xyza = planner.debug_xyza;
      auto image_points =
        solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
      tools::draw_points(img, image_points, {0, 0, 255});
    }

    cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸

    // 相机输出为 RGB 格式，imshow 需要 BGR 格式
    cv::Mat img_bgr;
    cv::cvtColor(img, img_bgr, cv::COLOR_RGB2BGR);
    cv::imshow("MPC Debug", img_bgr);
    auto key = cv::waitKey(1);
    if (key == 'q') break;
  }

  quit = true;
  if (plan_thread.joinable()) plan_thread.join();

  // 停止控制并清零命令
  io::Command stop_command;
  stop_command.control = false;
  stop_command.shoot = false;
  stop_command.yaw = 0;
  stop_command.pitch = 0;
  cboard.send(stop_command);

  // 清理 ROS2
#ifdef AMENT_CMAKE_FOUND
  visualizer.reset();
  rclcpp::shutdown();
  tools::logger()->info("[ROS2] Shutdown complete");
#endif

  return 0;
}