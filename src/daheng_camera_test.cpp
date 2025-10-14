#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

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

  tools::Exiter exiter;
  
  // 初始化相机
  io::Camera camera(config_path);
  
  // 初始化检测器
  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  
  cv::Mat img;
  std::chrono::steady_clock::time_point t;
  
  std::cout << "程序启动成功！使用 Daheng 相机进行实时检测。按 ESC 或 q 退出。" << std::endl;
  
  while (!exiter.exit()) {
    // 读取相机图像
    camera.read(img, t);
    // resize half
    cv::resize(img, img, cv::Size(), 0.5, 0.5);
    
    // 检查图像是否有效
    if (img.empty()) {
      std::cout << "相机读取失败，请检查相机连接！" << std::endl;
      std::this_thread::sleep_for(100ms);
      continue;
    }
    
    // YOLO 检测
    auto armors = yolo.detect(img);
    
    // 跟踪
    auto targets = tracker.track(armors, t);
    
    // 绘制结果
    cv::Mat display_img = img.clone();
    
    // 绘制检测到的装甲板
    for (const auto& armor : armors) {
      cv::circle(display_img, armor.center_norm, 5, cv::Scalar(0, 255, 0), 2);
      cv::rectangle(display_img, armor.box, cv::Scalar(255, 0, 0), 2);
      
      // 显示装甲板信息
      std::string info = "ID:" + std::to_string(armor.class_id) + 
                        " Conf:" + std::to_string(armor.confidence).substr(0, 4);
      cv::putText(display_img, info, 
                  cv::Point(armor.box.x, armor.box.y - 5),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 1);
    }
    
    // 显示状态信息
    std::string status = "Detected: " + std::to_string(armors.size()) + " armors";
    cv::putText(display_img, status, cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255), 2);
    
    // 显示图像
    cv::imshow("Daheng Camera + YOLO Detection", display_img);
    
    char key = cv::waitKey(1);
    if (key == 27 || key == 'q' || key == 'Q') { // ESC 或 q 退出
      break;
    }
    
    std::this_thread::sleep_for(10ms);
  }
  
  cv::destroyAllWindows();
  std::cout << "程序正常退出" << std::endl;
  
  return 0;
}