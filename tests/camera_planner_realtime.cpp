#include <fmt/core.h>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/img_tools.hpp"

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明 }"
  "{@config-path   | configs/camera.yaml    | yaml配置文件的路径}"
  "{tradition t    |  false                 | 是否使用传统方法识别}";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  bool use_tradition = cli.get<bool>("tradition");

  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;
  io::Camera camera(config_path);
  auto_aim::Detector detector(config_path);
  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);
  auto_aim::Planner planner(config_path);

  std::chrono::steady_clock::time_point timestamp;
  int frame_count = 0;

  tools::logger()->info("Starting camera + planner test...");
  tools::logger()->info("Press 'q' to quit");

  while (!exiter.exit()) {
    cv::Mat img, display_img;
    std::list<auto_aim::Armor> armors;

    camera.read(img, timestamp);

    if (img.empty()) break;

    display_img = img.clone();
    frame_count++;

    auto detection_start = std::chrono::steady_clock::now();

    // 检测装甲板
    if (use_tradition)
      armors = detector.detect(img);
    else
      armors = yolo.detect(img);

    auto detection_end = std::chrono::steady_clock::now();
    auto detection_time = tools::delta_time(detection_end, detection_start);

    // 显示检测结果
    tools::draw_text(display_img, 
                     fmt::format("Frame: {}, FPS: {:.1f}, Armors: {}", 
                               frame_count, 1.0/detection_time, armors.size()),
                     {10, 30}, {0, 255, 0});

    // 如果检测到装甲板，进行PnP求解和轨迹规划
    if (!armors.empty()) {
      // 对所有装甲板进行PnP求解
      for (auto& armor : armors) {
        solver.solve(armor);
      }

      // 选择最近的有效装甲板
      auto valid_armor = std::find_if(armors.begin(), armors.end(),
        [](const auto& armor) {
          return armor.xyz_in_world.norm() > 0.1 && armor.xyz_in_world.norm() < 10.0;
        });

      if (valid_armor != armors.end()) {
        // 创建简化的目标用于轨迹规划测试
        // 这里我们创建一个虚拟的运动目标，基于检测到的装甲板位置
        double distance = valid_armor->xyz_in_world.norm();
        double angular_velocity = 2.0; // 假设2 rad/s的角速度
        
        auto_aim::Target virtual_target(distance, angular_velocity, 0.2, 0.1);
        
        // 进行轨迹规划
        auto plan_start = std::chrono::steady_clock::now();
        auto plan = planner.plan(virtual_target, 22);
        auto plan_end = std::chrono::steady_clock::now();
        auto plan_time = tools::delta_time(plan_end, plan_start);

        // 显示装甲板位置信息
        tools::draw_text(display_img,
                         fmt::format("Armor: ({:.2f}, {:.2f}, {:.2f})m", 
                                   valid_armor->xyz_in_world[0], 
                                   valid_armor->xyz_in_world[1], 
                                   valid_armor->xyz_in_world[2]),
                         {10, 60}, {255, 255, 0});

        // 显示轨迹规划结果
        tools::draw_text(display_img,
                         fmt::format("Plan: Yaw={:.1f}° Pitch={:.1f}°", 
                                   plan.yaw * 180.0 / M_PI, 
                                   plan.pitch * 180.0 / M_PI),
                         {10, 90}, {0, 255, 255});

        tools::draw_text(display_img,
                         fmt::format("Vel: {:.1f}°/s {:.1f}°/s", 
                                   plan.yaw_vel * 180.0 / M_PI,
                                   plan.pitch_vel * 180.0 / M_PI),
                         {10, 120}, {0, 255, 255});

        tools::draw_text(display_img,
                         fmt::format("Target: {:.1f}°, Plan Time: {:.1f}ms", 
                                   plan.target_yaw * 180.0 / M_PI, plan_time * 1000),
                         {10, 150}, {255, 0, 255});

        // 每30帧输出一次到控制台
        if (frame_count % 30 == 0) {
          tools::logger()->info("Frame {}: Detected armor at ({:.2f}, {:.2f}, {:.2f})m", 
                               frame_count,
                               valid_armor->xyz_in_world[0], 
                               valid_armor->xyz_in_world[1], 
                               valid_armor->xyz_in_world[2]);
          
          tools::logger()->info("  Plan: target={:.1f}° yaw={:.1f}° pitch={:.1f}°",
                               plan.target_yaw * 180.0 / M_PI,
                               plan.yaw * 180.0 / M_PI,
                               plan.pitch * 180.0 / M_PI);
          
          tools::logger()->info("  Velocity: yaw={:.1f}°/s pitch={:.1f}°/s",
                               plan.yaw_vel * 180.0 / M_PI,
                               plan.pitch_vel * 180.0 / M_PI);
        }

        // 绘制装甲板位置标记
        cv::circle(display_img, valid_armor->center_norm, 8, cv::Scalar(0, 0, 255), 3);
        
        // 显示装甲板距离标签
        std::string distance_label = fmt::format("{:.1f}m", distance);
        tools::draw_text(display_img, distance_label,
                         {(int)valid_armor->center_norm.x - 20, (int)valid_armor->center_norm.y - 20},
                         {0, 255, 0});
      }
    } else {
      // 没有检测到装甲板
      tools::draw_text(display_img, "No armors detected", {10, 60}, {0, 0, 255});
    }

    // 绘制所有检测到的装甲板
    int armor_idx = 0;
    for (const auto& armor : armors) {
      armor_idx++;
      
      // 绘制装甲板中心点
      cv::circle(display_img, armor.center_norm, 5, cv::Scalar(0, 255, 0), 2);
      
      // 显示装甲板编号和距离
      if (armor.xyz_in_world.norm() > 0.1) {
        std::string armor_info = fmt::format("A{}: {:.1f}m", armor_idx, armor.xyz_in_world.norm());
        tools::draw_text(display_img, armor_info,
                         {(int)armor.center_norm.x + 10, (int)armor.center_norm.y + 10},
                         {255, 255, 0});
      }
    }

    cv::imshow("Camera + Planner Real-time Test", display_img);

    // 检查退出条件
    char key = cv::waitKey(1);
    if (key == 'q' || key == 27) {
      tools::logger()->info("User requested exit");
      break;
    }
  }

  tools::logger()->info("Camera + planner test completed");
  tools::logger()->info("Total frames processed: {}", frame_count);

  cv::destroyAllWindows();
  return 0;
}