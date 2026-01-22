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
#include "io/cboard_sp.hpp"
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
  
  // 初始化 ROS2 (仅在 ROS2 可用时)
#ifdef AMENT_CMAKE_FOUND
  rclcpp::init(argc, argv);
  auto ros_node = std::make_shared<rclcpp::Node>("armor_visualizer");

  // 静态 TF broadcaster (用于固定的坐标系关系)
  auto static_tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(ros_node);

  // 动态 TF broadcaster (用于实时更新的坐标系)
  auto dynamic_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(ros_node);

  // 在后台线程中运行 ROS2 spin
  std::thread ros_spin_thread([ros_node]() {
    rclcpp::spin(ros_node);
  });
  ros_spin_thread.detach();

  tools::logger()->info("ROS2 initialized for armor visualization with TF broadcasting");
#endif

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

#ifdef AMENT_CMAKE_FOUND
  // 将 ROS2 节点设置给 tracker
  tracker.set_ros2_node(ros_node);

  // 🔄 设置CBoard的TF发布器（IMU数据到达时立即发布TF）
  cboard.set_ros2_tf_publisher(ros_node, solver.R_gimbal2imubody());
  tools::logger()->info("[CBoard] TF publisher configured - TF will be sent on every IMU update");

  // 📌 发布静态 TF：map -> world (根坐标系)
  geometry_msgs::msg::TransformStamped map_to_world;
  map_to_world.header.frame_id = "map";
  map_to_world.child_frame_id = "world";
  map_to_world.transform.translation.x = 0.0;
  map_to_world.transform.translation.y = 0.0;
  map_to_world.transform.translation.z = 0.0;
  map_to_world.transform.rotation.x = 0.0;
  map_to_world.transform.rotation.y = 0.0;
  map_to_world.transform.rotation.z = 0.0;
  map_to_world.transform.rotation.w = 1.0;

  // 创建定时器定期发布 map -> world（确保 rviz2 能接收到）
  auto map_world_tf_timer = ros_node->create_wall_timer(
    std::chrono::seconds(1),
    [static_tf_broadcaster, ros_node]() {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = ros_node->now();
      tf.header.frame_id = "map";
      tf.child_frame_id = "world";
      tf.transform.translation.x = 0.0;
      tf.transform.translation.y = 0.0;
      tf.transform.translation.z = 0.0;
      tf.transform.rotation.x = 0.0;
      tf.transform.rotation.y = 0.0;
      tf.transform.rotation.z = 0.0;
      tf.transform.rotation.w = 1.0;
      static_tf_broadcaster->sendTransform(tf);
    });

  tools::logger()->info("Static TF published: map -> world");

  // 📌 发布静态 TF：camera -> gimbal (相机外参)
  // 🔧 使用配置文件中的标定参数
  geometry_msgs::msg::TransformStamped camera_to_gimbal;
  camera_to_gimbal.header.stamp = ros_node->now();
  camera_to_gimbal.header.frame_id = "gimbal";
  camera_to_gimbal.child_frame_id = "camera";

  // 🔑 相机外参（从configs/complete_template.yaml）
  // t_camera2gimbal: [0.13160669975045827, 0.10377721766577375, 0.024908271912914642]
  camera_to_gimbal.transform.translation.x = 0.13160669975045827;
  camera_to_gimbal.transform.translation.y = 0.10377721766577375;
  camera_to_gimbal.transform.translation.z = 0.024908271912914642;

  // R_camera2gimbal: 3x3旋转矩阵 → 四元数
  Eigen::Matrix3d R_cam2gim;
  R_cam2gim << -0.027182119030230909, -0.12616154330853446,  0.99163723074269183,
               -0.99949106557517331,   0.019998323121329122, -0.024853106601381177,
               -0.016695575474690555, -0.99180811252093692,  -0.12664093215554434;
  Eigen::Quaterniond q_cam2gim(R_cam2gim);
  camera_to_gimbal.transform.rotation.x = q_cam2gim.x();
  camera_to_gimbal.transform.rotation.y = q_cam2gim.y();
  camera_to_gimbal.transform.rotation.z = q_cam2gim.z();
  camera_to_gimbal.transform.rotation.w = q_cam2gim.w();
  static_tf_broadcaster->sendTransform(camera_to_gimbal);

  tools::logger()->info("Static TF published: gimbal -> camera");
#endif

  // 🎯 所有模块初始化完成，启动相机触发
  tools::logger()->info("=== All modules initialized ===");
  //cboard.start_camera_trigger();
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
      // current_imu_count = cboard.get_imu_count();  // 获取当前IMU计数

      // 计算当前帧对应的触发点IMU计数（加上手动偏移量）
      trigger_imu_count = (((frame_id + 1) * 10) + frame_id_to_imu_offset) % 10000;
      if (trigger_imu_count < 0) trigger_imu_count += 10000;

      // 🆕 使用环形数组O(1)查询IMU数据
      auto imu_result = cboard.imu_by_count(trigger_imu_count);
      q=imu_result;
      synced_imu_timestamp = t;
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
    auto targets = tracker.track(armors, t);
    t_end = std::chrono::steady_clock::now();
    double t_track = std::chrono::duration<double, std::milli>(t_end - t_start).count();

    t_start = std::chrono::steady_clock::now();
    // 🔑 使用硬同步的 IMU 时间戳（与 TF/Marker 严格一致）
    // to_now=false：不补偿处理延迟，使用触发时刻的状态（与 TF 时间一致）

    auto command = aimer.aim(targets, t, cboard.bullet_speed, false);
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

      // 🆕 发布 ROS2 Markers (使用 tracker 的集成功能)
#ifdef AMENT_CMAKE_FOUND
      // 🔑 使用强制同步后的 IMU 时间戳（而不是相机时间戳 t）
      // 这样 Marker 和 TF 都基于同一个 IMU 数据的时间戳
      auto ros_time_ptr = cboard.convert_to_ros_time(synced_imu_timestamp);
      auto ros_time = *std::static_pointer_cast<rclcpp::Time>(ros_time_ptr);

      if (command.control && aimer.debug_aim_point.valid) {
        tracker.publish_markers(targets, ros_time, aimer.debug_aim_point.xyza, true);
      } else {
        tracker.publish_markers(targets, ros_time);
      }
#endif
    }
    
    t_end = std::chrono::steady_clock::now();
    double t_visualize = std::chrono::duration<double, std::milli>(t_end - t_start).count();

    // 缩小图像并显示
    cv::Mat img_display;
    cv::resize(img, img_display, {}, 0.5, 0.5);

    // 计算总耗时
    double t_total = t_imu + t_solver_setup + t_detect + t_track + t_aim + t_send + t_visualize;

    // 显示帧率和性能数据
    int y_offset = 30;
    int line_height = 30;

    // FPS
    std::string fps_text = fmt::format("FPS: {:.1f}", current_fps);
    cv::putText(img_display, fps_text, cv::Point(10, y_offset),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
    y_offset += line_height;

    // 总耗时
    cv::putText(img_display, fmt::format("Total: {:.1f}ms", t_total),
                cv::Point(10, y_offset), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                cv::Scalar(255, 255, 255), 2);
    y_offset += line_height;


    cv::putText(img_display, fmt::format("IMU: {:.1f}ms", t_imu),
                cv::Point(10, y_offset), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(200, 200, 200), 1);
    y_offset += 25;

    cv::putText(img_display, fmt::format("Detect: {:.1f}ms", t_detect),
                cv::Point(10, y_offset), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(200, 200, 200), 1);
    y_offset += 25;

    cv::putText(img_display, fmt::format("Track: {:.1f}ms", t_track),
                cv::Point(10, y_offset), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(200, 200, 200), 1);
    y_offset += 25;

    cv::putText(img_display, fmt::format("Aim: {:.1f}ms", t_aim),
                cv::Point(10, y_offset), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(200, 200, 200), 1);
    y_offset += 25;

    cv::putText(img_display, fmt::format("Visual: {:.1f}ms", t_visualize),
                cv::Point(10, y_offset), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(200, 200, 200), 1);

    // 终端输出性能数据（每30帧输出一次）
    if (frame_count % 30 == 0) {
      // tools::logger()->info(
      //   "[Performance] FPS: {:.1f} | Total: {:.1f}ms | Camera: {:.1f}ms | IMU: {:.2f}ms | "
      //   "Detect: {:.1f}ms | Track: {:.2f}ms | Aim: {:.2f}ms | Visual: {:.2f}ms | Send: {:.2f}ms",
      //   current_fps, t_total, t_camera, t_imu, t_detect, t_track, t_aim, t_visualize, t_send);

      // 🆕 显示frame匹配信息
      // tools::logger()->info(
      //   "[Frame Match] current_imu_count={} trigger_imu={} q(w,x,y,z)=({:.4f},{:.4f},{:.4f},{:.4f})",
      //   current_imu_count, trigger_imu_count,
      //   q.w(), q.x(), q.y(), q.z());
    }

    cv::imshow("reprojection", img_display);
    int key = cv::waitKey(1);
  // 清理 ROS2
#ifdef AMENT_CMAKE_FOUND
  rclcpp::shutdown();
#endif

  return 0;
}