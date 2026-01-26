#include <fmt/core.h>

#include <chrono>
#include <fstream>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/outpost_target.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include <rclcpp/rclcpp.hpp>
#include "tools/tf_publisher.hpp"
#include "tools/marker_publisher.hpp"

const std::string keys =
  "{help h usage ? |                     | 输出命令行参数说明 }"
  "{config-path c  | configs/camera.yaml | yaml配置文件的路径}"
  "{use-camera     | false               | 使用真实相机而非视频文件 }"
  "{start-index s  | 0                   | 视频起始帧下标    }"
  "{end-index e    | 0                   | 视频结束帧下标    }"
  "{@input-path    | assets/demo/outpost | avi和txt文件的路径}";

int main(int argc, char * argv[])
{
  // 初始化 ROS2
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("outpost_target_test");

  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    rclcpp::shutdown();
    return 0;
  }
  auto input_path = cli.get<std::string>(0);
  auto config_path = cli.get<std::string>("config-path");
  
  auto use_camera = cli.get<bool>("use-camera");
  auto start_index = cli.get<int>("start-index");
  auto end_index = cli.get<int>("end-index");

  tools::Plotter plotter;
  tools::Exiter exiter;

  // 根据选择使用相机或视频文件
  std::unique_ptr<io::Camera> camera;
  cv::VideoCapture video;
  std::ifstream text;
  
  if (use_camera) {
    tools::logger()->info("使用真实相机输入");
    // camera = std::make_unique<io::Camera>(config_path);
    io::Camera camera(config_path);
  } else {
    tools::logger()->info("使用视频文件: {}", input_path);
    auto video_path = fmt::format("{}.avi", input_path);
    auto text_path = fmt::format("{}.txt", input_path);
    video.open(video_path);
    text.open(text_path);
    
    if (!video.isOpened()) {
      tools::logger()->error("无法打开视频: {}", video_path);
      rclcpp::shutdown();
      return -1;
    }
    
    video.set(cv::CAP_PROP_POS_FRAMES, start_index);
    for (int i = 0; i < start_index; i++) {
      double t, w, x, y, z;
      text >> t >> w >> x >> y >> z;
    }
  }

  // 初始化模块
  // auto_aim::YOLO yolo(config_path);
  auto_aim::Detector detector(config_path);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);

  // 创建 TF Publisher 和 Marker Publisher
  auto tf_publisher = std::make_shared<tools::TFPublisher>(node);
  solver.set_TFPublisher(tf_publisher.get());
  
  auto marker_publisher = std::make_shared<tools::MarkerPublisher>(node);

  cv::Mat img;
  auto t0 = std::chrono::steady_clock::now();

  tools::logger()->info("========================================");
  tools::logger()->info("前哨站目标测试程序启动");
  tools::logger()->info("按 'q' 退出");
  tools::logger()->info("========================================");

  for (int frame_count = start_index; !exiter.exit() && rclcpp::ok(); frame_count++) {
    if (end_index > 0 && frame_count > end_index) break;

    std::chrono::steady_clock::time_point timestamp;
    
    if (use_camera) {
      // 使用相机输入
      camera->read(img, timestamp);
      if (img.empty()) {
        tools::logger()->warn("无法从相机读取图像");
        continue;
      }
      
      // 相机模式：使用单位四元数（无云台运动）
      solver.set_R_gimbal2world({1, 0, 0, 0});
      
    } else {
      // 使用视频文件
      video.read(img);
      if (img.empty()) break;

      double t, w, x, y, z;
      text >> t >> w >> x >> y >> z;
      timestamp = t0 + std::chrono::microseconds(int(t * 1e6));
      
      // 设置云台姿态
      solver.set_R_gimbal2world({w, x, y, z});
    }

    solver.publish_static_tfs();

    // 检测装甲板
    auto yolo_start = std::chrono::steady_clock::now();
    auto armors = detector.detect(img, frame_count);

    // 过滤：只保留前哨站装甲板
    std::list<auto_aim::Armor> outpost_armors;
    for (const auto & armor : armors) {
      if (armor.name == auto_aim::ArmorName::outpost) {
        outpost_armors.push_back(armor);
      }
    }

    auto tracker_start = std::chrono::steady_clock::now();
    auto targets = tracker.track(outpost_armors, timestamp);
    auto finish = std::chrono::steady_clock::now();

    // 性能统计
    tools::logger()->info(
      "[{}] yolo: {:.1f}ms, tracker: {:.1f}ms, outpost_num: {}", 
      frame_count,
      tools::delta_time(tracker_start, yolo_start) * 1e3,
      tools::delta_time(finish, tracker_start) * 1e3,
      outpost_armors.size());

    // 可视化检测结果
    int armor_idx = 0;
    for (const auto & armor : outpost_armors) {
      if (!armor.points.empty()) {
        tools::draw_points(img, armor.points, {0, 255, 255}, 2);  // 青色：前哨站
      } else {
        cv::rectangle(img, armor.box, {0, 255, 255}, 2);
      }

      auto label = fmt::format("Outpost #{} {:.0f}%", armor_idx, armor.confidence * 100);
      auto text_anchor = cv::Point(
        static_cast<int>(armor.center.x), 
        static_cast<int>(armor.center.y)
      );
      tools::draw_text(img, label, text_anchor, {0, 255, 255}, 0.6, 2);
      armor_idx++;
    }

    // 数据记录
    nlohmann::json data;

    // 装甲板原始观测数据
    data["armor_num"] = armors.size();
    data["outpost_num"] = static_cast<int>(outpost_armors.size());

    if (!use_camera) {
      data["video_frame"] = frame_count;
    } else {
      data["frame"] = frame_count;
      data["timestamp"] = tools::delta_time(timestamp, t0);
    }

    // 可视化跟踪结果
    if (!targets.empty()) {
      auto & target = targets.front();

      // RViz 可视化
      target.set_marker_publisher(marker_publisher.get());
      target.visualize(0);

      // 重投影所有预测的装甲板（3个）
      std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();

      for (size_t i = 0; i < armor_xyza_list.size(); i++) {
        const Eigen::Vector4d & xyza = armor_xyza_list[i];
        
        auto image_points = solver.reproject_armor(
          xyza.head(3), xyza[3], target.armor_type, target.name
        );

        // 不同装甲板用不同颜色区分
        cv::Scalar color;
        if (i == static_cast<size_t>(target.last_id)) {
          color = {0, 255, 0};  // 绿色：当前匹配
        } else {
          switch (i) {
            case 0: color = {0, 0, 255}; break;    // 红色
            case 1: color = {255, 0, 0}; break;    // 蓝色
            case 2: color = {0, 255, 255}; break;  // 黄色
            default: color = {200, 200, 200}; break;
          }
        }
        
        tools::draw_points(img, image_points, color, 2);
      }

      // 绘制状态信息
      Eigen::VectorXd x = target.ekf_x();
      
      int y_offset = 30;
      auto draw_info = [&](const std::string & text) {
        tools::draw_text(img, text, {10, y_offset}, {255, 255, 255}, 0.6, 2);
        y_offset += 25;
      };

      draw_info(fmt::format("帧数: {}", frame_count));
      draw_info(fmt::format("前哨站数量: {}", outpost_armors.size()));
      draw_info(fmt::format("当前ID: {}", target.last_id));
      draw_info(fmt::format("中心: ({:.2f}, {:.2f}, {:.2f})", x[0], x[2], x[4]));
      draw_info(fmt::format("角度: {:.1f}°", x[6] * 57.3));
      draw_info(fmt::format("角速度: {:.2f} rad/s", x[7]));
      draw_info(fmt::format("半径: {:.3f} m", x[8]));
      
      // 前哨站特有：高度差
      if (x.size() >= 13) {
        draw_info(fmt::format("h1: {:.3f} m", x[11]));
        draw_info(fmt::format("h2: {:.3f} m", x[12]));
      }

      // 观测器内部数据
      data["x"] = x[0];
      data["vx"] = x[1];
      data["y"] = x[2];
      data["vy"] = x[3];
      data["z"] = x[4];
      data["vz"] = x[5];
      data["a"] = x[6] * 57.3;
      data["w"] = x[7];
      data["r"] = x[8];
      data["last_id"] = target.last_id;

      if (x.size() >= 13) {
        data["h1"] = x[11];
        data["h2"] = x[12];
      }

      // 卡方检验数据
      data["nis"] = target.ekf().data.at("nis");
      data["nees"] = target.ekf().data.at("nees");
    } else {
      // 没有检测到前哨站
      tools::draw_text(img, "未检测到前哨站", {10, 30}, {0, 0, 255}, 0.8, 2);
    }

    plotter.plot(data);

    // 显示图像
    cv::resize(img, img, {}, 0.5, 0.5);
    cv::imshow("Outpost Target Test", img);
    auto key = cv::waitKey(30);
    if (key == 'q') break;

    // ROS2 回调
    rclcpp::spin_some(node);
  }

  rclcpp::shutdown();
  tools::logger()->info("前哨站测试程序退出");
  return 0;
}
