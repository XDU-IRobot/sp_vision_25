#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/dm_imu/dm_imu.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/armor_aimer.hpp"
#include "tasks/auto_aim/predictor.hpp"  
#include "tools/marker_publisher.hpp"
#include "tools/tf_publisher.hpp"

const std::string keys =
  "{help h usage ? |                  | 输出命令行参数说明}"
  "{@config-path   | configs/uav.yaml | yaml配置文件路径 }";

using namespace std::chrono_literals;
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("uav_onlyfronted_debug");
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    rclcpp::shutdown(); // 关闭ros2
    return 0;
  }

  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder;
  tools::MarkerPublisher marker_pub(node);
  
  // io::Camera camera(config_path);
  // io::CBoard cboard(config_path);
  io::Gimbal gimbal(config_path);
  io::Camera camera(config_path);
  
  auto_aim::Detector detector(config_path);
  auto_aim::Solver solver(config_path);
  auto_aim::Shooter shooter(config_path);
  auto_aim::Predictor predictor(config_path,solver);
  auto_aim::armor_aimer armor_aimer(config_path);
  predictor.set_marker_publisher(&marker_pub);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  // auto mode = io::Mode::idle; //if gimbal 修改为gimbalmode
  auto mode = io::GimbalMode::IDLE;
  // auto last_mode = io::Mode::idle;
  auto last_mode = io::GimbalMode::IDLE;
  auto t0 = std::chrono::steady_clock::now();

  // 自瞄流程
  while (!exiter.exit() && rclcpp::ok()) {
    camera.read(img, t);
    // q = cboard.imu_at(t - 1ms);
    q = gimbal.q(t - 1ms);
    // mode = cboard.mode;
    mode = gimbal.mode();
    // recorder.record(img, q, t);

    /*unknow code*/
    // if (last_mode != mode) {
    //   tools::logger()->info("Switch to {}", gimbal.str(mode));
    //   last_mode = mode;
    // }


    /// 自瞄
    solver.set_R_gimbal2world(q);
    Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);
    nlohmann::json data;
    auto armors = detector.detect(img);
    
    // 只处理第一个装甲板（如果存在）
    if (!armors.empty()) {
      auto & armor = armors.front();
      solver.solve(armor);
    }
    
    auto_aim::Predictor::PredictResult prediction;
    bool has_prediction = false;

    if (!armors.empty()) {
        prediction = predictor.track(armors.front(), t);  // ← 传入 Armor 和时间戳
        has_prediction = prediction.valid;
        tools::logger()->info("[Main] Frame: has_prediction={}, valid={}", has_prediction, prediction.valid);
        predictor.visualize(200); // 可视化预测结果
    }
    
    // 只有在有有效预测时才传递给 aimer
    std::list<auto_aim::Predictor::PredictResult> predictions;
    if (has_prediction && prediction.valid) {
      predictions = {prediction};
    }
    auto command = armor_aimer.aim(predictions, t, 10);
    
    // 重要调试：打印原始指令值（弧度）
    if (command.control) {
        tools::logger()->info(
            "[Main] Command: yaw={:.4f} rad ({:.2f}°), pitch={:.4f} rad ({:.2f}°)",
            command.yaw, command.yaw * 57.3,
            command.pitch, command.pitch * 57.3
        );
    }
    
    // command.shoot = shooter.shoot(command, armor_aimer, predictions, ypr);
    /* 暂不引入shooter */
    command.yaw = command.yaw*57.3;
    command.pitch = command.pitch*57.3;
    gimbal.send_command_scm(command);
    // cboard.send(command);
    
    /*todo: plotjuggler数据*/
    
    data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);
    data["armor_num"] = armors.size();

    // 装甲板观测数据
    if (!armors.empty()) {
      auto & armor = armors.front();
      
      int name_send = 0;
      if (armor.name == auto_aim::ArmorName::outpost) {
        name_send = 18;
      } else {
        name_send = static_cast<int>(armor.name);
      }
      
      data["armor_name"] = name_send;
      data["armor_x"] = armor.xyz_in_world[0];
      data["armor_y"] = armor.xyz_in_world[1];
      data["armor_z"] = armor.xyz_in_world[2];
      data["armor_yaw"] = armor.ypr_in_world[0] * 57.3;
      data["armor_confidence"] = armor.confidence;
    }

    // Predictor 状态
    if (has_prediction) {
      data["pred_valid"] = prediction.valid;
      data["pred_x"] = prediction.position[0];
      data["pred_y"] = prediction.position[1];
      data["pred_z"] = prediction.position[2];
      data["pred_vx"] = prediction.velocity[0];
      data["pred_vy"] = prediction.velocity[1];
      data["pred_vz"] = prediction.velocity[2];
      data["pred_yaw"] = prediction.yaw * 57.3;
      data["pred_yaw_rate"] = prediction.yaw_rate * 57.3;
      data["pred_horizon"] = prediction.horizon;
      
      // 预测误差（如果有真实观测）
      if (!armors.empty()) {
        auto & armor = armors.front();
        data["pred_error_x"] = prediction.position[0] - armor.xyz_in_world[0];
        data["pred_error_y"] = prediction.position[1] - armor.xyz_in_world[1];
        data["pred_error_norm"] = (prediction.position - armor.xyz_in_world).norm();
      }
    }

    // 云台状态
    data["gimbal_yaw"] = ypr[0] * 57.3;
    data["gimbal_pitch"] = ypr[1] * 57.3;
    data["gimbal_roll"] = ypr[2] * 57.3;
    
    // 指令数据
    if (command.control) {
      data["cmd_yaw"] = command.yaw;
      data["cmd_pitch"] = command.pitch;
      data["cmd_shoot"] = command.shoot;

      // 指令与云台的误差
      data["error_yaw"] = (command.yaw - ypr[0]) * 57.3;
      data["error_pitch"] = (command.pitch - ypr[1]) * 57.3;
    }

    // Aimer debug 数据
    if (armor_aimer.debug_aim_point.valid) {
      data["aim_x"] = armor_aimer.debug_aim_point.xyza[0];
      data["aim_y"] = armor_aimer.debug_aim_point.xyza[1];
      data["aim_z"] = armor_aimer.debug_aim_point.xyza[2];
      data["aim_yaw"] = armor_aimer.debug_aim_point.xyza[3] * 57.3;
    }

    plotter.plot(data);

    // ===== 重投影可视化 =====
    if (has_prediction && prediction.valid) {
      // 调试输出
      tools::logger()->debug(
        "[Reproject] pred_pos=({:.2f},{:.2f},{:.2f}), yaw={:.2f}°, type={}, name={}",
        prediction.position[0], prediction.position[1], prediction.position[2],
        prediction.yaw * 57.3,
        static_cast<int>(prediction.armor_type),
        static_cast<int>(prediction.armor_name)
      );

      // 绿色框：预测结果
      auto reprojected_points = solver.reproject_armor(
        prediction.position,
        prediction.yaw,
        prediction.armor_type,
        prediction.armor_name
      );
      
      // 检查投影点是否在合理范围
      bool points_valid = true;
      for (const auto & pt : reprojected_points) {
        if (pt.x < 0 || pt.x > img.cols || pt.y < 0 || pt.y > img.rows) {
          points_valid = false;
          tools::logger()->warn("[Reproject] Point out of image: ({:.1f}, {:.1f})", pt.x, pt.y);
        }
      }
      
      if (points_valid) {
        tools::draw_points(img, reprojected_points, {0, 255, 0}, 2);
      }

      // 红色框：原始观测
      if (!armors.empty()) {
        auto original_points = solver.reproject_armor(
          armors.front().xyz_in_world,
          armors.front().ypr_in_world[0],
          armors.front().type,
          armors.front().name
        );
        tools::draw_points(img, original_points, {0, 0, 255}, 1);
      }
    } else {
      tools::logger()->debug("[Reproject] Skipped: has_prediction={}, valid={}", 
                            has_prediction, prediction.valid);
    }

    // 蓝色框：瞄准点
    if (armor_aimer.debug_aim_point.valid && has_prediction) {
      auto aim_points = solver.reproject_armor(
        armor_aimer.debug_aim_point.xyza.head(3),
        armor_aimer.debug_aim_point.xyza[3],
        prediction.armor_type,
        prediction.armor_name
      );
      tools::draw_points(img, aim_points, {255, 0, 0}, 2);
    }

    // ===== 7. 显示 =====
    tools::draw_text(img, fmt::format("[{}]", predictor.state()), {10, 30}, {255, 255, 255});
    
    cv::resize(img, img, {}, 0.5, 0.5);
    cv::imshow("predictor_debug", img);
    auto key = cv::waitKey(1);
    if (key == 'q') break;
    rclcpp::spin_some(node);
}
rclcpp::shutdown(); // 关闭ros2
return 0;
}

