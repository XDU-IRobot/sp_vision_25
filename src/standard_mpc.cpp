#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>
#include <optional>
#include <yaml-cpp/yaml.h>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "io/command.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/multithread/commandgener.hpp"
#include "tasks/auto_aim/multithread/mt_detector.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_buff/buff_aimer.hpp"
#include "tasks/auto_buff/buff_detector.hpp"
#include "tasks/auto_buff/buff_solver.hpp"
#include "tasks/auto_buff/buff_target.hpp"
#include "tasks/auto_buff/buff_type.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{@config-path   | | yaml配置文件路径 }";

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }

  // 读取可选开关：是否启用打符（避免在不需要时加载不兼容的 IR 模型）
  bool enable_buff = false;
  bool force_auto_aim = false;
  try {
    auto yaml = YAML::LoadFile(config_path);
    if (yaml["enable_buff"]) enable_buff = yaml["enable_buff"].as<bool>();
    if (yaml["force_auto_aim"]) force_auto_aim = yaml["force_auto_aim"].as<bool>();
  } catch (...) { /* keep default */ }

  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder;
  io::Camera camera(config_path);
  io::CBoard cboard(config_path);

  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Planner planner(config_path);

  tools::ThreadSafeQueue<std::optional<auto_aim::Target>, true> target_queue(1);
  target_queue.push(std::nullopt);

  std::unique_ptr<auto_buff::Buff_Detector> buff_detector;
  std::unique_ptr<auto_buff::Solver> buff_solver;
  std::unique_ptr<auto_buff::Aimer> buff_aimer;
  std::unique_ptr<auto_buff::SmallTarget> buff_small_target;
  std::unique_ptr<auto_buff::BigTarget> buff_big_target;
  if (enable_buff) {
    buff_detector = std::make_unique<auto_buff::Buff_Detector>(config_path);
    buff_solver = std::make_unique<auto_buff::Solver>(config_path);
    buff_aimer = std::make_unique<auto_buff::Aimer>(config_path);
    buff_small_target = std::make_unique<auto_buff::SmallTarget>();
    buff_big_target = std::make_unique<auto_buff::BigTarget>();
  } else {
    tools::logger()->info("[standard_mpc] Buff modules disabled (enable_buff=false)");
  }

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  std::atomic<bool> quit = false;

  std::atomic<io::GimbalMode> mode{io::GimbalMode::IDLE};
  auto last_mode{io::GimbalMode::IDLE};
  std::atomic<double> bullet_speed_atomic{0.0};

  auto plan_thread = std::thread([&]() {
    auto t0 = std::chrono::steady_clock::now();
    uint16_t last_bullet_count = 0;

    while (!quit) {
      if (!target_queue.empty() && mode == io::GimbalMode::AUTO_AIM) {
        auto target = target_queue.front();
        float bs = static_cast<float>(bullet_speed_atomic.load());
        auto plan = planner.plan(target, bs);

  io::Command cmd{};
  cmd.control = plan.control;
  cmd.shoot = plan.fire;
  cmd.yaw = plan.yaw;
        cmd.pitch = plan.pitch;
        cmd.horizon_distance = 0.0;
        cboard.send(cmd);

        std::this_thread::sleep_for(10ms);
      } else {
        std::this_thread::sleep_for(200ms);
      }
    }
  });

  while (!exiter.exit()) {
    // 模式：可用 YAML 强制自瞄，或根据 CBoard 模式映射
    if (force_auto_aim) {
      mode = io::GimbalMode::AUTO_AIM;
    } else {
      io::GimbalMode mapped = io::GimbalMode::IDLE;
      switch (cboard.mode) {
        case io::Mode::idle:       mapped = io::GimbalMode::IDLE; break;
        case io::Mode::auto_aim:   mapped = io::GimbalMode::AUTO_AIM; break;
        case io::Mode::small_buff: mapped = io::GimbalMode::SMALL_BUFF; break;
        case io::Mode::big_buff:   mapped = io::GimbalMode::BIG_BUFF; break;
        case io::Mode::outpost:    mapped = io::GimbalMode::IDLE; break; // 可按需调整
      }
      mode = mapped;
    }

    if (last_mode != mode) {
      const char* MODE_STR[] = {"IDLE", "AUTO_AIM", "SMALL_BUFF", "BIG_BUFF"};
      tools::logger()->info("Switch to {}", MODE_STR[static_cast<int>(mode.load())]);
      last_mode = mode.load();
    }

    camera.read(img, t);
    q = cboard.imu_at(t);
    // 从 CBoard 接收弹速（其他角度信息由算法估计，设为 0）
    bullet_speed_atomic = cboard.bullet_speed;
    io::GimbalState gs{};
    gs.yaw = 0; gs.yaw_vel = 0; gs.pitch = 0; gs.pitch_vel = 0;
    gs.bullet_speed = static_cast<float>(bullet_speed_atomic.load());
    gs.bullet_count = 0;
    recorder.record(img, q, t);
    solver.set_R_gimbal2world(q);

    /// 自瞄
    if (mode.load() == io::GimbalMode::AUTO_AIM) {
      auto armors = yolo.detect(img);
      auto targets = tracker.track(armors, t);
      if (!targets.empty())
        target_queue.push(targets.front());
      else
        target_queue.push(std::nullopt);
    }

    /// 打符
    else if (mode.load() == io::GimbalMode::SMALL_BUFF || mode.load() == io::GimbalMode::BIG_BUFF) {
      if (!enable_buff) {
        // 未启用打符，保持不控，避免误触发
        io::Command idle{}; idle.control = false; idle.shoot = false; idle.yaw = 0; idle.pitch = 0; idle.horizon_distance = 0;
        cboard.send(idle);
      } else {
        buff_solver->set_R_gimbal2world(q);

        auto power_runes = buff_detector->detect(img);

        buff_solver->solve(power_runes);

        auto_aim::Plan buff_plan;
        if (mode.load() == io::GimbalMode::SMALL_BUFF) {
          buff_small_target->get_target(power_runes, t);
          auto target_copy = *buff_small_target;
          buff_plan = buff_aimer->mpc_aim(target_copy, t, gs, true);
        } else if (mode.load() == io::GimbalMode::BIG_BUFF) {
          buff_big_target->get_target(power_runes, t);
          auto target_copy = *buff_big_target;
          buff_plan = buff_aimer->mpc_aim(target_copy, t, gs, true);
        }

        io::Command cmd2{};
        cmd2.control = buff_plan.control;
        cmd2.shoot = buff_plan.fire;
        cmd2.yaw = buff_plan.yaw;
        cmd2.pitch = buff_plan.pitch;
        cmd2.horizon_distance = 0.0;
        cboard.send(cmd2);
      }

    } else {
      io::Command idle{}; idle.control = false; idle.shoot = false; idle.yaw = 0; idle.pitch = 0; idle.horizon_distance = 0;
      cboard.send(idle);
    }

    // Debug preview window for standard_mpc
    // Tip: press 'q' to quit
    cv::imshow("standard_mpc", img);
    int key = cv::waitKey(1);
    if (key == 'q') break;
  }

    
  if (plan_thread.joinable()) plan_thread.join();
  io::Command stop{}; stop.control = false; stop.shoot = false; stop.yaw = 0; stop.pitch = 0; stop.horizon_distance = 0;
  cboard.send(stop);

  return 0;
}
