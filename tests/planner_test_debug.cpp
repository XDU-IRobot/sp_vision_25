#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>
#include <iostream>

#include "tasks/auto_aim/planner/planner.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |     | 输出命令行参数说明    }"
  "{d              | 3.0 | Target距离(m)       }"
  "{w              | 5.0 | Target角速度(rad/s) }"
  "{@config-path   |     | yaml配置文件路径     }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  auto d = cli.get<double>("d");
  auto w = cli.get<double>("w");
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }

  tools::logger()->info("Starting planner test with d={:.1f}m, w={:.1f}rad/s", d, w);
  
  tools::Exiter exiter;
  tools::Plotter plotter;

  try {
    auto_aim::Planner planner(config_path);
    tools::logger()->info("Planner initialized successfully");
    
    auto_aim::Target target(d, w, 0.2, 0.1);
    tools::logger()->info("Target created: distance={:.1f}m, angular_velocity={:.1f}rad/s", d, w);

    auto t0 = std::chrono::steady_clock::now();
    int iteration = 0;

    tools::logger()->info("Starting simulation loop... Press 'q' to exit");

    while (!exiter.exit()) {
      target.predict(0.01);

      auto plan = planner.plan(target, 22);

      if (iteration % 100 == 0) { // 每100次迭代输出一次
        tools::logger()->info(
          "Iter {}: t={:.3f}s, target_yaw={:.3f}°, target_pitch={:.3f}°", 
          iteration,
          tools::delta_time(std::chrono::steady_clock::now(), t0),
          plan.target_yaw * 180.0 / M_PI,
          plan.target_pitch * 180.0 / M_PI
        );
        
        tools::logger()->info(
          "  Plan: yaw={:.3f}°, yaw_vel={:.3f}°/s, yaw_acc={:.3f}°/s²",
          plan.yaw * 180.0 / M_PI,
          plan.yaw_vel * 180.0 / M_PI, 
          plan.yaw_acc * 180.0 / M_PI
        );
        
        tools::logger()->info(
          "  Plan: pitch={:.3f}°, pitch_vel={:.3f}°/s, pitch_acc={:.3f}°/s²",
          plan.pitch * 180.0 / M_PI,
          plan.pitch_vel * 180.0 / M_PI,
          plan.pitch_acc * 180.0 / M_PI
        );
      }

      nlohmann::json data;
      data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);
      data["target_yaw"] = plan.target_yaw;
      data["target_pitch"] = plan.target_pitch;
      data["plan_yaw"] = plan.yaw;
      data["plan_yaw_vel"] = plan.yaw_vel;
      data["plan_yaw_acc"] = plan.yaw_acc;
      data["plan_pitch"] = plan.pitch;
      data["plan_pitch_vel"] = plan.pitch_vel;
      data["plan_pitch_acc"] = plan.pitch_acc;

      plotter.plot(data);

      std::this_thread::sleep_for(10ms);
      iteration++;
      
      if (iteration > 10000) { // 防止无限循环
        tools::logger()->info("Reached maximum iterations, exiting...");
        break;
      }
    }
    
    tools::logger()->info("Planner test completed successfully");

  } catch (const std::exception& e) {
    tools::logger()->error("Error during planner test: {}", e.what());
    return 1;
  }

  return 0;
}