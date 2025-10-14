#include <chrono>
#include <opencv2/opencv.hpp>
#include <cstdlib>
#include <thread>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "io/command.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/multithread/mt_detector.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

const std::string keys = "{help h usage ? | | 输出命令行参数说明}"
                         "{@config-path   | | yaml配置文件路径 }";

int main(int argc, char *argv[]) {
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }

  auto config_path = cli.get<std::string>("@config-path");

  tools::Exiter exiter;
  tools::Plotter plotter;
  io::Camera camera(config_path);
  io::CBoard cboard(config_path);

  auto_aim::multithread::MultiThreadDetector detector(config_path);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  auto detect_thread = std::thread([&]() {
    cv::Mat img;
    std::chrono::steady_clock::time_point t;

    while (!exiter.exit()) {
      camera.read(img, t);
      detector.push(img, t);
    }
  });

  auto last_t = std::chrono::steady_clock::now();
  // CBoard 指令调试状态
  bool ctrl = false, fire = false;
  double cmd_yaw = 0.0, cmd_pitch = 0.0, cmd_dist = 0.0;
  // 1Hz 状态日志
  auto last_status_log = std::chrono::steady_clock::time_point::min();
  // GUI/Headless 检测
  const char* disp = std::getenv("DISPLAY");
  bool headless = (disp == nullptr || std::string(disp).empty());
  if (headless) {
    tools::logger()->warn("[UI] DISPLAY 未设置，进入无窗口(headless)模式：将每秒保存一帧到 /tmp/reprojection.jpg");
  } else {
    tools::logger()->info("[UI] DISPLAY={}", disp);
    cv::namedWindow("reprojection", cv::WINDOW_NORMAL);
    cv::resizeWindow("reprojection", 960, 600);
  }
  nlohmann::json data;

  while (!exiter.exit()) {
    auto [img, armors, t] = detector.debug_pop();

  Eigen::Quaterniond q = cboard.imu_at(t);

    solver.set_R_gimbal2world(q);

    Eigen::Vector3d gimbal_pos =
        tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    auto targets = tracker.track(armors, t);

    auto command = aimer.aim(targets, t, 22);

    shooter.shoot(command, aimer, targets, gimbal_pos);

    auto dt = tools::delta_time(t, last_t);
    last_t = t;

    data["dt"] = dt;
    data["fps"] = 1 / dt;
    plotter.plot(data);
    // 装甲板原始观测数据
    data["armor_num"] = armors.size();
    if (!armors.empty()) {
      auto min_x = 1e10;
      auto &armor = armors.front();
      for (auto &a : armors) {
        if (a.center.x < min_x) {
          min_x = a.center.x;
          armor = a;
        }
      } // always left
      solver.solve(armor);
      data["armor_x"] = armor.xyz_in_world[0];
      data["armor_y"] = armor.xyz_in_world[1];
      data["armor_yaw"] = armor.ypr_in_world[0] * 57.3;
      data["armor_yaw_raw"] = armor.yaw_raw * 57.3;
    }

    if (!targets.empty()) {
      auto target = targets.front();
      tools::draw_text(img, fmt::format("[{}]", tracker.state()), {10, 30},
                       {255, 255, 255});

      // 当前帧target更新后
      std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
      for (const Eigen::Vector4d &xyza : armor_xyza_list) {
        auto image_points = solver.reproject_armor(
            xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {0, 255, 0});
      }

      // aimer瞄准位置
      auto aim_point = aimer.debug_aim_point;
      Eigen::Vector4d aim_xyza = aim_point.xyza;
      auto image_points = solver.reproject_armor(
          aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
      if (aim_point.valid)
        tools::draw_points(img, image_points, {0, 0, 255}); // red
      else
        tools::draw_points(img, image_points, {255, 0, 0}); // blue

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
      data["distance"] = std::sqrt(x[0] * x[0] + x[2] * x[2] + x[4] * x[4]);

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
    // 叠加当前 CBoard 状态与指令调试信息
    {
      std::string line1 = fmt::format("[CBoard] mode={}, shoot_mode={}, bullet_speed={:.2f}m/s",
                                      io::MODES[cboard.mode], io::SHOOT_MODES[cboard.shoot_mode], cboard.bullet_speed);
      std::string line2 = fmt::format("[CMD] ctrl={}, fire={}, yaw={:.3f}rad, pitch={:.3f}rad, dist={:.2f}m",
                                      ctrl, fire, cmd_yaw, cmd_pitch, cmd_dist);
      tools::draw_text(img, line1, {10, 20}, {0, 255, 255});
      tools::draw_text(img, line2, {10, 45}, {0, 255, 255});
    }
    cv::resize(img, img, {}, 0.5, 0.5); // 显示时缩小图片尺寸
    if (headless) {
      static auto last_dump = std::chrono::steady_clock::time_point::min();
      if (tools::delta_time(t, last_dump) > 1.0) {
        cv::imwrite("/tmp/reprojection.jpg", img);
        tools::logger()->info("[UI] /tmp/reprojection.jpg 已更新");
        last_dump = t;
      }
    } else {
      try {
        cv::imshow("reprojection", img);
        auto key = cv::waitKey(1);
        if (key == 'q') break;
        // 键盘控制：c 切换控制，f 切换射击；j/l 调整 yaw，i/k 调整 pitch；u/o 调整距离，z 复位
        const double d_ang = 0.01; // rad
        const double d_dist = 0.05; // m
        if (key == 'c') ctrl = !ctrl;
        if (key == 'f') fire = !fire;
        if (key == 'j') cmd_yaw -= d_ang;
        if (key == 'l') cmd_yaw += d_ang;
        if (key == 'i') cmd_pitch -= d_ang;
        if (key == 'k') cmd_pitch += d_ang;
        if (key == 'u') cmd_dist += d_dist;
        if (key == 'o') cmd_dist -= d_dist;
        if (key == 'z') { cmd_yaw = 0; cmd_pitch = 0; cmd_dist = 0; }
      } catch (const cv::Exception& e) {
        tools::logger()->warn("[UI] imshow 失败: {}", e.what());
      }
    }

    // 向 CBoard 发送一帧指令（用于验证上行链路）
    io::Command cmd{};
    cmd.control = ctrl;
    cmd.shoot = fire;
    cmd.yaw = static_cast<float>(cmd_yaw);
    cmd.pitch = static_cast<float>(cmd_pitch);
    cmd.horizon_distance = static_cast<float>(cmd_dist);
    cboard.send(cmd);

    // 每秒打印一次状态，确认通信正常
    auto now = std::chrono::steady_clock::now();
    if (tools::delta_time(now, last_status_log) > 1.0) {
      tools::logger()->info("[CBoard][MIN] mode={}, shoot_mode={}, bullet_speed={:.2f}",
                            io::MODES[cboard.mode], io::SHOOT_MODES[cboard.shoot_mode], cboard.bullet_speed);
      last_status_log = now;
    }
  }

  detect_thread.join();

  return 0;
}