#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>
#include <rclcpp/rclcpp.hpp>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/outpost_target.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/tf_publisher.hpp"
#include "tools/marker_publisher.hpp"

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | configs/outpost.yaml   | yaml配置文件路径 }";

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  // 初始化 ROS2
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("outpost_target_test");
 
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    rclcpp::shutdown();
    return 0;
  }

  tools::Exiter exiter;
  tools::Plotter plotter;

  // ============================================
  // 初始化模块
  // ============================================
  io::Camera camera(config_path);
  io::Gimbal gimbal(config_path);

  auto_aim::Detector detector(config_path);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);

  // ✅ 创建 TF Publisher
  auto tf_publisher = std::make_shared<tools::TFPublisher>(node);
  solver.set_TFPublisher(tf_publisher.get());

  // ✅ 创建 Marker Publisher（只用于手动可视化）
  auto marker_publisher = std::make_shared<tools::MarkerPublisher>(node);

  // ============================================
  // 主循环变量
  // ============================================
  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  auto mode = io::GimbalMode::IDLE;
  auto last_mode = io::GimbalMode::IDLE;
  
  auto t0 = std::chrono::steady_clock::now();
  int frame_count = 0;

  tools::logger()->info("========================================");
  tools::logger()->info("前哨站目标测试程序启动");
  tools::logger()->info("按 'q' 退出");
  tools::logger()->info("========================================");

  while (!exiter.exit() && rclcpp::ok()) {
    // ============================================
    // 1. 读取数据
    // ============================================
    camera.read(img, t);
    q = gimbal.q(t - 1ms);
    mode = gimbal.mode();

    if (last_mode != mode) {
      tools::logger()->info("模式切换: {}", gimbal.str(mode));
      last_mode = mode;
    }

    // ============================================
    // 2. 检测装甲板
    // ============================================
    solver.set_R_gimbal2world(q);
    solver.publish_static_tfs();

    auto armors = detector.detect(img);

    // ✅ 过滤：只保留前哨站装甲板
    std::list<auto_aim::Armor> outpost_armors;
    for (const auto & armor : armors) {
      if (armor.name == auto_aim::ArmorName::outpost) {
        outpost_armors.push_back(armor);
      }
    }

    // ============================================
    // 3. 可视化检测结果
    // ============================================
    int armor_idx = 0;
    for (const auto & armor : outpost_armors) {
      // 绘制装甲板轮廓
      if (!armor.points.empty()) {
        tools::draw_points(img, armor.points, {0, 255, 255}, 2);  // 青色：前哨站
      } else {
        cv::rectangle(img, armor.box, {0, 255, 255}, 2);
      }

      // 绘制标签
      auto label = fmt::format(
        "Outpost #{} {:.0f}%", armor_idx, armor.confidence * 100);
      auto text_anchor = cv::Point(
        static_cast<int>(armor.center.x), 
        static_cast<int>(armor.center.y)
      );
      tools::draw_text(img, label, text_anchor, {0, 255, 255}, 0.6, 2);
      armor_idx++;
    }

    // ============================================
    // 4. 跟踪前哨站目标（使用 list）
    // ============================================
    auto targets = tracker.track(outpost_armors, t);

    // ============================================
    // 5. 可视化跟踪结果
    // ============================================
    if (!targets.empty()) {
      auto & target = targets.front();

      // ✅ 手动调用可视化（只针对这一个目标）
#ifdef HAS_VISUALIZATION
      target.set_marker_publisher(marker_publisher.get());
      target.visualize(0);  // base_id = 0
#endif

      // 重投影所有预测的装甲板（3个）
      std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
      
      for (size_t i = 0; i < armor_xyza_list.size(); i++) {
        const Eigen::Vector4d & xyza = armor_xyza_list[i];
        
        auto image_points = solver.reproject_armor(
          xyza.head(3), xyza[3], target.armor_type, target.name
        );

        // 不同装甲板用不同颜色
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

      // ============================================
      // 6. 绘制状态信息
      // ============================================
      Eigen::VectorXd x = target.ekf_x();
      
      // 左上角状态信息
      int y_offset = 30;
      auto draw_info = [&](const std::string & text) {
        tools::draw_text(img, text, {10, y_offset}, {255, 255, 255}, 0.6, 2);
        y_offset += 25;
      };

      draw_info(fmt::format("帧数: {}", frame_count++));
      draw_info(fmt::format("前哨站数量: {}", outpost_armors.size()));
      draw_info(fmt::format("当前ID: {}", target.last_id));
      draw_info(fmt::format("中心: ({:.2f}, {:.2f}, {:.2f})", x[0], x[2], x[4]));
      draw_info(fmt::format("角度: {:.1f}°", x[6] * 57.3));
      draw_info(fmt::format("角速度: {:.2f} rad/s", x[7]));
      draw_info(fmt::format("半径: {:.3f} m", x[8]));
      
      // ✅ 前哨站特有：高度差
      if (x.size() >= 13) {
        draw_info(fmt::format("h1: {:.3f} m", x[11]));
        draw_info(fmt::format("h2: {:.3f} m", x[12]));
      }

      // ============================================
      // 7. 数据记录（用于绘图）
      // ============================================
      nlohmann::json data;
      data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);
      data["outpost_num"] = static_cast<int>(outpost_armors.size());
      
      // EKF 状态
      data["x"] = x[0];
      data["y"] = x[2];
      data["z"] = x[4];
      data["vx"] = x[1];
      data["vy"] = x[3];
      data["vz"] = x[5];
      data["angle"] = x[6] * 57.3;
      data["vyaw"] = x[7];
      data["radius"] = x[8];
      
      if (x.size() >= 13) {
        data["h1"] = x[11];
        data["h2"] = x[12];
      }

      // 卡方检验
      data["nis"] = target.ekf().data.at("nis");
      data["nees"] = target.ekf().data.at("nees");
      data["last_id"] = target.last_id;

      plotter.plot(data);
    } else {
      // 没有检测到前哨站
      tools::draw_text(
        img, "未检测到前哨站", {10, 30}, {0, 0, 255}, 0.8, 2
      );
    }

    // ============================================
    // 8. 显示图像
    // ============================================
    cv::resize(img, img, {}, 0.5, 0.5);
    cv::imshow("Outpost Target Test", img);
    auto key = cv::waitKey(1);
    if (key == 'q') break;

    // ============================================
    // 9. ROS2 回调（发布 TF 和 Marker）
    // ============================================
    rclcpp::spin_some(node);
  }

  rclcpp::shutdown();
  tools::logger()->info("前哨站测试程序退出");
  return 0;
}