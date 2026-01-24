#include <fmt/core.h>

#include <chrono>
#include <fstream>
#include <thread>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

const std::string keys =
  "{help h usage ? |                              | 输出命令行参数说明 }"
  "{config-path c  | ../configs/vtune_test.yaml   | yaml配置文件的路径（相对于build目录）}"
  "{use-camera     | true                         | 使用真实相机而非视频文件（默认启用）}"
  "{start-index s  | 0                            | 视频起始帧下标    }"
  "{end-index e    | 0                            | 视频结束帧下标    }"
  "{@input-path    | ../assets/demo/demo          | avi和txt文件的路径（相对于build目录）}";

int main(int argc, char * argv[])
{
  // cv::setNumThreads(0); 
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto input_path = cli.get<std::string>(0);
  auto config_path = cli.get<std::string>("config-path");
  auto use_camera = cli.get<bool>("use-camera");
  auto start_index = cli.get<int>("start-index");
  auto end_index = cli.get<int>("end-index");

  tools::Plotter plotter;
  tools::Exiter exiter;

  // 根据选择使用相机或视频文件
  std::unique_ptr<io::Camera> camera;
  cv::VideoCapture video;
  std::ifstream text;
  
  if (use_camera) {
    tools::logger()->info("Using real camera input");
    camera = std::make_unique<io::Camera>(config_path);
  } else {
    tools::logger()->info("Using video file: {}", input_path);
    auto video_path = fmt::format("{}.avi", input_path);
    auto text_path = fmt::format("{}.txt", input_path);
    video.open(video_path);
    text.open(text_path);
    
    if (!video.isOpened()) {
      tools::logger()->error("Failed to open video: {}", video_path);
      return -1;
    }
    
    video.set(cv::CAP_PROP_POS_FRAMES, start_index);
    for (int i = 0; i < start_index; i++) {
      double t, w, x, y, z;
      text >> t >> w >> x >> y >> z;
    }
  }

  auto_aim::YOLO yolo(config_path);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);

  cv::Mat img, drawing;
  auto t0 = std::chrono::steady_clock::now();

  auto_aim::Target last_target;
  io::Command last_command;
  double last_t = -1;

  for (int frame_count = start_index; !exiter.exit(); frame_count++) {
    if (end_index > 0 && frame_count > end_index) break;

    std::chrono::steady_clock::time_point timestamp;
    
    if (use_camera) {
      // 使用相机输入
      camera->read(img, timestamp);
      if (img.empty()) {
        tools::logger()->warn("Failed to read from camera");
        continue;
      }
      
      // 相机模式下使用单位四元数（无云台运动）
      solver.set_R_gimbal2world({1, 0, 0, 0});
      
    } else {
      // 使用视频文件
      video.read(img);
      if (img.empty()) break;

      double t, w, x, y, z;
      text >> t >> w >> x >> y >> z;
      timestamp = t0 + std::chrono::microseconds(int(t * 1e6));
      
      // 设置云台姿态
      solver.set_R_gimbal2world({w, x, y, z});
    }

    auto yolo_start = std::chrono::steady_clock::now();
    auto armors = yolo.detect(img, frame_count);

    auto tracker_start = std::chrono::steady_clock::now();
    auto targets = tracker.track(armors, timestamp);

    auto aimer_start = std::chrono::steady_clock::now();
    auto command = aimer.aim(targets, timestamp, 27, false);

    if (
      !targets.empty() && aimer.debug_aim_point.valid &&
      std::abs(command.yaw - last_command.yaw) * 57.3 < 2)
      command.shoot = true;

    if (command.control) last_command = command;
    /// 调试输出

    auto finish = std::chrono::steady_clock::now();
    tools::logger()->info(
      "[{}] yolo: {:.1f}ms, tracker: {:.1f}ms, aimer: {:.1f}ms", frame_count,
      tools::delta_time(tracker_start, yolo_start) * 1e3,
      tools::delta_time(aimer_start, tracker_start) * 1e3,
      tools::delta_time(finish, aimer_start) * 1e3);

    tools::draw_text(
      img,
      fmt::format(
        "command is {},{:.2f},{:.2f},shoot:{}", command.control, command.yaw * 57.3,
        command.pitch * 57.3, command.shoot),
      {10, 60}, {154, 50, 205});

    if (!use_camera) {
      // 只在视频模式下显示云台信息 - 跳过，因为我们使用旧的w,x,y,z变量
      tools::draw_text(
        img,
        "Video Mode",
        {10, 90}, {255, 255, 255});
    } else {
      // 相机模式下显示帧率信息
      tools::draw_text(
        img,
        fmt::format("Camera Mode - Frame: {}", frame_count),
        {10, 90}, {255, 255, 255});
    }

    nlohmann::json data;

    if (!armors.empty()) {
      const auto & armor = armors.front();

      // 1. 绘制检测框（白色，粗线）
      tools::draw_points(img, armor.points, {255, 255, 255}, 2);
      tools::draw_text(img, "Detection", armor.points[0] - cv::Point2f(0, 10), {255, 255, 255});

      // 2. 绘制原始PnP重投影（橙色）
      auto reproject_raw = solver.reproject_armor(
        armor.xyz_in_world, armor.yaw_raw, armor.type, armor.name);
      // tools::draw_points(img, reproject_raw, {0, 165, 255});  // 橙色
      // tools::draw_text(img, "PnP Raw", reproject_raw[0] - cv::Point2f(0, 25), {0, 165, 255});

      // 3. 绘制优化后重投影
      auto reproject_opt = solver.reproject_armor(
        armor.xyz_in_world, armor.ypr_in_world[0], armor.type, armor.name);
      tools::draw_points(img, reproject_opt, {0, 255, 0});  
      tools::draw_text(img, "PnP Optimized", reproject_opt[0] - cv::Point2f(0, 40), {0, 255, 0});

      // 4. 绘制误差连线（检测点→重投影点）
      for (int i = 0; i < 4; i++) {
        // 检测→优化重投影
        cv::line(img, armor.points[i], reproject_opt[i], {0, 0, 255}, 1, cv::LINE_AA);
      }

      // 5. 在图像上显示误差数值
      double error_raw = 0.0, error_opt = 0.0;
      for (int i = 0; i < 4; i++) {
        error_raw += cv::norm(armor.points[i] - reproject_raw[i]);
        error_opt += cv::norm(armor.points[i] - reproject_opt[i]);
      }

      tools::draw_text(
        img,
        fmt::format("Reproj Error: Raw={:.1f}px, Opt={:.1f}px", error_raw, error_opt),
        {10, 120}, {255, 255, 0});
    }

    // 装甲板原始观测数据
    data["armor_num"] = armors.size();
    if (!armors.empty()) {
      const auto & armor = armors.front();
      data["armor_x"] = armor.xyz_in_world[0];
      data["armor_y"] = armor.xyz_in_world[1];
      data["armor_yaw"] = armor.ypr_in_world[0] * 57.3;
      data["armor_yaw_raw"] = armor.yaw_raw * 57.3;

      double yaw_diff_rad = armor.ypr_in_world[0] - armor.yaw_raw;
      double yaw_diff_normalized = tools::limit_rad(yaw_diff_rad);  // 归一化到[-π, π]
      double yaw_diff_abs = std::abs(yaw_diff_normalized);

      data["yaw_diff"] = yaw_diff_rad * 57.3;  // 原始差值（度）
      data["yaw_diff_normalized"] = yaw_diff_normalized * 57.3;  // 归一化差值（度）
      data["yaw_diff_abs"] = yaw_diff_abs * 57.3;  // 绝对差值（度）

      // 计算两个解的重投影误差进行对比
      auto reproject_points_optimized = solver.reproject_armor(
        armor.xyz_in_world, armor.ypr_in_world[0], armor.type, armor.name);
      auto reproject_points_raw = solver.reproject_armor(
        armor.xyz_in_world, armor.yaw_raw, armor.type, armor.name);

      double error_optimized = 0.0;
      double error_raw = 0.0;
      for (int i = 0; i < 4; i++) {
        error_optimized += cv::norm(armor.points[i] - reproject_points_optimized[i]);
        error_raw += cv::norm(armor.points[i] - reproject_points_raw[i]);
      }

      data["reproj_error_optimized"] = error_optimized;
      data["reproj_error_raw"] = error_raw;
      data["reproj_error_diff"] = error_optimized - error_raw;

      for (int i = 0; i < 4; i++) {
        double corner_error = cv::norm(armor.points[i] - reproject_points_optimized[i]);
        data["corner_" + std::to_string(i) + "_error"] = corner_error;
      }
      data["armor_type"] = armor.type;  // 0=big, 1=small
      data["armor_name"] = static_cast<int>(armor.name);
      data["armor_distance"] = armor.ypd_in_world[2];  // 距离
      data["armor_pitch"] = armor.ypr_in_world[1] * 57.3;
      data["armor_roll"] = armor.ypr_in_world[2] * 57.3;

      data["armor_center_x"] = armor.center_norm.x;
      data["armor_center_y"] = armor.center_norm.y;
    }

    if (!use_camera) {
      // 视频模式：使用简单的帧计数
      data["video_frame"] = frame_count;
    } else {
      // 相机模式：记录帧数和时间戳
      data["frame"] = frame_count;
      data["timestamp"] = tools::delta_time(timestamp, t0);
    }
    
    data["cmd_yaw"] = command.yaw * 57.3;
    data["shoot"] = command.shoot;

    if (!targets.empty()) {
      auto target = targets.front();

      if (last_t == -1) {
        last_target = target;
        last_t = use_camera ? tools::delta_time(timestamp, t0) : frame_count;
        continue;
      }

      std::vector<Eigen::Vector4d> armor_xyza_list;

      // 当前帧target更新后
      // armor_xyza_list = target.armor_xyza_list();
      // for (const Eigen::Vector4d & xyza : armor_xyza_list) {
      //   auto image_points =
      //     solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
      //   tools::draw_points(img, image_points, {0, 255, 0});
      // }

      // aimer瞄准位置
      // auto aim_point = aimer.debug_aim_point;
      // Eigen::Vector4d aim_xyza = aim_point.xyza;
      // auto image_points =
      //   solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
      // if (aim_point.valid) tools::draw_points(img, image_points, {0, 0, 255});


      // 观测器内部数据
      Eigen::VectorXd x = target.ekf_x();
      data["x"] = x[0];
      data["vx"] = x[1];
      data["y"] = x[2];
      data["vy"] = x[3];
      data["z"] = x[4];
      data["vz"] = x[5];
      data["a"] = x[6] * 57.3;
      data["w"] = x[7];
      data["r"] = x[8];
      data["l"] = x[9];
      data["h"] = x[10];
      data["last_id"] = target.last_id;

      // 卡方检验数据
      data["residual_yaw"] = target.ekf().data.at("residual_yaw");
      data["residual_pitch"] = target.ekf().data.at("residual_pitch");
      data["residual_distance"] = target.ekf().data.at("residual_distance");
      data["residual_angle"] = target.ekf().data.at("residual_angle");
      data["nis"] = target.ekf().data.at("nis");
      data["nees"] = target.ekf().data.at("nees");
      data["nis_fail"] = target.ekf().data.at("nis_fail");
      data["nees_fail"] = target.ekf().data.at("nees_fail");
      data["recent_nis_failures"] = target.ekf().data.at("recent_nis_failures");
    }

    plotter.plot(data);

    cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    cv::imshow("reprojection", img);
    auto key = cv::waitKey(1);  
    if (key == 'q') break;
  }

  return 0;

}