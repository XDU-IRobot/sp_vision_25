/**
 * @file trigger_test.cpp
 * @brief 大恒相机硬触发功能测试程序
 *
 * 测试内容：
 * 1. 连续模式采集测试
 * 2. 硬触发模式测试（需要外部触发信号）
 * 3. 不同触发源和触发沿测试
 *
 * 使用方法：
 * 1. 修改 configs/daheng.yaml 中的触发参数
 * 2. 连接硬件触发信号（如需要）
 * 3. 运行: ./build/trigger_test
 */

#include <opencv2/opencv.hpp>
#include <chrono>
#include <iostream>

#include "io/camera.hpp"
#include "tools/logger.hpp"
#include "tools/yaml.hpp"

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
  std::string config_path = "configs/daheng.yaml";

  if (argc > 1) {
    config_path = argv[1];
  }

  tools::logger()->info("=== 大恒相机硬触发测试程序 ===");
  tools::logger()->info("配置文件: {}", config_path);

  // 读取配置，显示当前触发模式
  auto yaml = tools::load(config_path);
  bool trigger_enable = yaml["trigger_enable"] ? yaml["trigger_enable"].as<bool>() : false;
  int trigger_source = yaml["trigger_source"] ? yaml["trigger_source"].as<int>() : 0;
  int trigger_activation = yaml["trigger_activation"] ? yaml["trigger_activation"].as<int>() : 0;

  tools::logger()->info("触发模式配置:");
  tools::logger()->info("  trigger_enable: {}", trigger_enable ? "true (硬触发)" : "false (连续采集)");
  if (trigger_enable) {
    tools::logger()->info("  trigger_source: {} (Line{})", trigger_source, trigger_source);
    tools::logger()->info("  trigger_activation: {} ({})",
                         trigger_activation,
                         trigger_activation == 0 ? "上升沿" : "下降沿");
  }
  tools::logger()->info("");

  try {
    // 创建相机对象
    tools::logger()->info("正在初始化相机...");
    io::Camera camera(config_path);
    tools::logger()->info("相机初始化成功！");
    tools::logger()->info("");

    if (!trigger_enable) {
      // 连续模式测试
      tools::logger()->info("=== 连续采集模式测试 ===");
      tools::logger()->info("相机将自动连续采集图像");
      tools::logger()->info("按 'q' 键退出，'s' 键保存图像");
      tools::logger()->info("");

      int frame_count = 0;
      auto start_time = std::chrono::steady_clock::now();

      while (true) {
        cv::Mat img;
        std::chrono::steady_clock::time_point timestamp;
        camera.read(img, timestamp);

        frame_count++;

        // 计算帧率
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
          current_time - start_time).count();

        if (elapsed > 0 && frame_count % 30 == 0) {
          double fps = static_cast<double>(frame_count) / elapsed;
          tools::logger()->info("帧数: {}, 平均帧率: {:.2f} FPS", frame_count, fps);
        }

        // 显示图像
        cv::imshow("Camera Test - Continuous Mode", img);

        char key = cv::waitKey(1);
        if (key == 'q' || key == 'Q') {
          break;
        } else if (key == 's' || key == 'S') {
          std::string filename = "capture_" + std::to_string(frame_count) + ".jpg";
          cv::imwrite(filename, img);
          tools::logger()->info("图像已保存: {}", filename);
        }
      }

      tools::logger()->info("");
      tools::logger()->info("测试完成！共采集 {} 帧", frame_count);

    } else {
      // 硬触发模式测试
      tools::logger()->info("=== 硬触发模式测试 ===");
      tools::logger()->info("相机正在等待外部触发信号...");
      tools::logger()->info("请确保触发信号已正确连接到 Line{}", trigger_source);
      tools::logger()->info("触发方式: {}", trigger_activation == 0 ? "上升沿" : "下降沿");
      tools::logger()->info("按 'q' 键退出，'s' 键保存图像");
      tools::logger()->info("");

      int frame_count = 0;
      auto last_trigger_time = std::chrono::steady_clock::now();

      while (true) {
        cv::Mat img;
        std::chrono::steady_clock::time_point timestamp;
        camera.read(img, timestamp);

        frame_count++;

        // 计算触发间隔
        auto current_time = std::chrono::steady_clock::now();
        auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
          current_time - last_trigger_time).count();
        last_trigger_time = current_time;

        tools::logger()->info("收到触发 #{}, 间隔: {} ms", frame_count, interval);

        // 显示图像
        cv::Mat display_img = img.clone();
        std::string text = "Trigger #" + std::to_string(frame_count);
        cv::putText(display_img, text, cv::Point(50, 50),
                   cv::FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(0, 255, 0), 3);
        cv::imshow("Camera Test - Trigger Mode", display_img);

        char key = cv::waitKey(1);
        if (key == 'q' || key == 'Q') {
          break;
        } else if (key == 's' || key == 'S') {
          std::string filename = "trigger_" + std::to_string(frame_count) + ".jpg";
          cv::imwrite(filename, img);
          tools::logger()->info("图像已保存: {}", filename);
        }
      }

      tools::logger()->info("");
      tools::logger()->info("测试完成！共收到 {} 次触发", frame_count);
    }

    cv::destroyAllWindows();
    tools::logger()->info("相机已关闭");

  } catch (const std::exception& e) {
    tools::logger()->error("错误: {}", e.what());
    return 1;
  }

  return 0;
}
