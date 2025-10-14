#include <fmt/core.h>

#include <chrono>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/auto_aim/solver.hpp"
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
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);
  auto use_tradition = cli.get<bool>("tradition");

  tools::Exiter exiter;

  io::Camera camera(config_path);
  auto_aim::Detector detector(config_path, true);
  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);

  std::chrono::steady_clock::time_point timestamp;

  tools::logger()->info("Starting camera PnP test...");
  tools::logger()->info("Press 'q' to quit, 'r' to reset");

  while (!exiter.exit()) {
    cv::Mat img, display_img;
    std::list<auto_aim::Armor> armors;

    camera.read(img, timestamp);

    if (img.empty()) break;

    display_img = img.clone();

    auto last = std::chrono::steady_clock::now();

    // 检测装甲板
    if (use_tradition)
      armors = detector.detect(img);
    else
      armors = yolo.detect(img);

    auto now = std::chrono::steady_clock::now();
    auto dt = tools::delta_time(now, last);

    // 显示检测到的装甲板数量
    tools::draw_text(display_img, 
                     fmt::format("FPS: {:.1f}, Armors: {}", 1.0/dt, armors.size()),
                     {10, 30}, {0, 255, 0});

    // 对每个检测到的装甲板进行PnP解算和重投影
    int armor_count = 0;
    for (const auto& armor : armors) {
      armor_count++;
      
      // 绘制检测框
      cv::circle(display_img, armor.center_norm, 5, cv::Scalar(0, 255, 0), 2);
      
      tools::draw_text(display_img,
                       fmt::format("A{}: ({:.1f}, {:.1f})", armor_count, armor.center_norm.x, armor.center_norm.y),
                       {10, 60 + armor_count * 30}, {255, 255, 0});

      // PnP解算结果显示
      if (armor.xyz_in_world.norm() > 0.1) { // 如果有有效的3D位置
        tools::draw_text(display_img,
                         fmt::format("  Pos: ({:.2f}, {:.2f}, {:.2f})", 
                                   armor.xyz_in_world[0], armor.xyz_in_world[1], armor.xyz_in_world[2]),
                         {10, 80 + armor_count * 30}, {255, 255, 255});

        // 重投影装甲板的四个角点
        try {
          auto projected_points = solver.reproject_armor(
            armor.xyz_in_world, armor.yaw_raw, armor.type, armor.name);
          
          if (projected_points.size() >= 4) {
            // 绘制重投影的装甲板框
            for (size_t i = 0; i < projected_points.size(); ++i) {
              cv::circle(display_img, projected_points[i], 3, cv::Scalar(0, 0, 255), 2);
              if (i > 0) {
                cv::line(display_img, projected_points[i-1], projected_points[i], cv::Scalar(0, 0, 255), 2);
              }
            }
            // 闭合框
            if (projected_points.size() >= 4) {
              cv::line(display_img, projected_points.back(), projected_points[0], cv::Scalar(0, 0, 255), 2);
            }
          }
        } catch (const std::exception& e) {
          tools::logger()->warn("PnP reprojection failed: {}", e.what());
        }
      }
    }

    tools::logger()->info("FPS: {:.2f}, Detected {} armors", 1.0/dt, armors.size());

    // 显示图像
    cv::resize(display_img, display_img, {}, 0.7, 0.7);  // 缩小显示
    cv::imshow("Camera PnP Test", display_img);
    
    auto key = cv::waitKey(1);
    if (key == 'q') break;
    if (key == 'r') {
      tools::logger()->info("Reset requested");
    }
  }

  cv::destroyAllWindows();
  tools::logger()->info("Camera PnP test finished");
  return 0;
}