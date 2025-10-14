#include <chrono>
#include <thread>

#include "io/dm_imu/dm_imu.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

using namespace std::chrono_literals;

int main(int argc, char *argv[]) {
  tools::Exiter exiter;
  // Prefer YAML config if provided: ./dm_test <config.yaml>
  if (argc > 1) {
    std::string config_path = argv[1];
    tools::logger()->info("[dm_test] using config: {}", config_path);
    io::DM_IMU imu(config_path);

    while (!exiter.exit()) {
      auto timestamp = std::chrono::steady_clock::now();

      std::this_thread::sleep_for(1ms);

      Eigen::Quaterniond q = imu.imu_at(timestamp);

      Eigen::Vector3d eulers = tools::eulers(q, 2, 1, 0) * 57.3;
      tools::logger()->info("z{:.2f} y{:.2f} x{:.2f} degree", eulers[0],
                            eulers[1], eulers[2]);
    }

    return 0;
  }

  // Fallback to defaults when no config path is provided
  io::DM_IMU imu;

  while (!exiter.exit()) {
    auto timestamp = std::chrono::steady_clock::now();

    std::this_thread::sleep_for(1ms);

    Eigen::Quaterniond q = imu.imu_at(timestamp);

    Eigen::Vector3d eulers = tools::eulers(q, 2, 1, 0) * 57.3;
    tools::logger()->info("z{:.2f} y{:.2f} x{:.2f} degree", eulers[0],
                          eulers[1], eulers[2]);
  }

  return 0;
}