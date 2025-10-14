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
  auto_aim::Target previous_target;
  bool has_previous_target = false;

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

    // 如果检测到装甲板，进行轨迹规划
    if (!armors.empty()) {
      // 对所有装甲板进行PnP求解
      for (auto& armor : armors) {
        solver.solve(armor);
      }

      // 选择最近的装甲板作为目标
      auto closest_armor = std::min_element(armors.begin(), armors.end(),
        [](const auto& a, const auto& b) {
          return a.xyz_in_world.norm() < b.xyz_in_world.norm();
        });

      if (closest_armor->xyz_in_world.norm() > 0.1) {
        // 创建目标（简化版本，假设目标静止）
        auto_aim::Target current_target;
        
        // 使用 Armor 对象创建 Target
        Eigen::VectorXd P0_diag = Eigen::VectorXd::Ones(9) * 0.1;
        current_target = auto_aim::Target(*closest_armor, timestamp, 0.26, 4, P0_diag);

        // 轨迹规划
        auto plan_start = std::chrono::steady_clock::now();
        auto plan = planner.plan(current_target, 22);
        auto plan_end = std::chrono::steady_clock::now();
        auto plan_time = tools::delta_time(plan_end, plan_start);

        // 显示目标位置和规划结果
        tools::draw_text(display_img,
                         fmt::format("Target: ({:.2f}, {:.2f}, {:.2f})m", 
                                   closest_armor->xyz_in_world[0], 
                                   closest_armor->xyz_in_world[1], 
                                   closest_armor->xyz_in_world[2]),
                         {10, 60}, {255, 255, 0});

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
                         fmt::format("Plan Time: {:.2f}ms", plan_time * 1000),
                         {10, 150}, {255, 0, 255});

        // 每30帧输出一次到控制台
        if (frame_count % 30 == 0) {
          tools::logger()->info("Frame {}: Target at ({:.2f}, {:.2f}, {:.2f})m", 
                               frame_count,
                               closest_armor->xyz_in_world[0], 
                               closest_armor->xyz_in_world[1], 
                               closest_armor->xyz_in_world[2]);
          
          tools::logger()->info("  Plan: yaw={:.1f}° pitch={:.1f}°, vel=({:.1f}, {:.1f})°/s",
                               plan.yaw * 180.0 / M_PI,
                               plan.pitch * 180.0 / M_PI,
                               plan.yaw_vel * 180.0 / M_PI,
                               plan.pitch_vel * 180.0 / M_PI);
        }

        // 保存当前目标用于下一帧的速度估计
        previous_target = current_target;
        has_previous_target = true;

        // 绘制装甲板位置
        cv::circle(display_img, closest_armor->center_norm, 8, cv::Scalar(0, 0, 255), 3);
        
        // 如果有装甲板关键点，绘制出来
        if (closest_armor->points.size() == 4) {
          std::vector<cv::Point2f> points;
          for (const auto& pt : closest_armor->points) {
            points.push_back(pt);
          }
          cv::polylines(display_img, points, true, cv::Scalar(255, 0, 0), 2);
        }
      }
    } else {
      // 没有检测到装甲板
      tools::draw_text(display_img, "No targets detected", {10, 60}, {0, 0, 255});
      has_previous_target = false;
    }

    cv::imshow("Camera + Planner Test", display_img);

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