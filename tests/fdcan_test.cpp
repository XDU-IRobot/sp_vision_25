#include "io/gimbal/gimbal_fdcan.hpp"

#include <chrono>
#include <cstdlib>
#include <exception>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>

#include "tools/exiter.hpp"
#include "tools/logger.hpp"

namespace
{
using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? | | Print this message}"
  "{control         | | Send control-enabled command frames}"
  "{shoot           | | Set shoot flag in command frames}"
  "{yaw             | 0.0 | Command yaw, radians unless scm_angles_in_deg converts it}"
  "{pitch           | 0.0 | Command pitch, radians unless scm_angles_in_deg converts it}"
  "{period-ms       | 10 | Command send period in milliseconds}"
  "{print-ms        | 500 | Status print period in milliseconds}"
  "{@config-path    | ../configs/camera.yaml | YAML config path}";

std::string mode_to_string(io::GimbalMode mode)
{
  switch (mode) {
    case io::GimbalMode::IDLE:
      return "IDLE";
    case io::GimbalMode::AUTO_AIM:
      return "AUTO_AIM";
    case io::GimbalMode::SMALL_BUFF:
      return "SMALL_BUFF";
    case io::GimbalMode::BIG_BUFF:
      return "BIG_BUFF";
    default:
      return "UNKNOWN";
  }
}

}  // namespace

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  const auto config_path = cli.get<std::string>("@config-path");
  const bool control = cli.get<bool>("control");
  const bool shoot = cli.get<bool>("shoot");
  const double yaw = cli.get<double>("yaw");
  const double pitch = cli.get<double>("pitch");
  const auto period_ms = std::max(1, cli.get<int>("period-ms"));
  const auto print_ms = std::max(1, cli.get<int>("print-ms"));

  if (!cli.check()) {
    cli.printErrors();
    return 1;
  }

  try {
    tools::Exiter exiter;

    tools::logger()->info(
      "[fdcan_test] config={}, control={}, shoot={}, yaw={:.6f}, pitch={:.6f}",
      config_path, control, shoot, yaw, pitch);
    tools::logger()->info("[fdcan_test] Constructing GimbalFdcan; this waits until the first valid RX frame.");

    io::GimbalFdcan gimbal(config_path);

    io::Command command{};
    command.control = control;
    command.shoot = shoot;
    command.yaw = yaw;
    command.pitch = pitch;

    auto tx_count = 0U;
    auto rx_sample_count = 0U;
    auto last_print = std::chrono::steady_clock::now();

    tools::logger()->info("[fdcan_test] Running. Press Ctrl+C to stop.");
    while (!exiter.exit()) {
      const auto now = std::chrono::steady_clock::now();

      gimbal.send_command_scm(command);
      ++tx_count;

      auto state = gimbal.state();
      auto mode = gimbal.mode();
      auto q = gimbal.q(now);
      ++rx_sample_count;

      if (now - last_print >= std::chrono::milliseconds(print_ms)) {
        tools::logger()->info(
          "[fdcan_test] mode={}, bullet_speed={:.2f}, q=[{:.6f}, {:.6f}, {:.6f}, {:.6f}], tx={}, rx_samples={}",
          mode_to_string(mode), state.bullet_speed, q.w(), q.x(), q.y(), q.z(), tx_count, rx_sample_count);
        last_print = now;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
    }

    command.control = false;
    command.shoot = false;
    gimbal.send_command_scm(command);
    tools::logger()->info("[fdcan_test] Stopped. tx={}, rx_samples={}", tx_count, rx_sample_count);
  } catch (const std::exception & e) {
    tools::logger()->error("[fdcan_test] {}", e.what());
    return 1;
  }

  return 0;
}
