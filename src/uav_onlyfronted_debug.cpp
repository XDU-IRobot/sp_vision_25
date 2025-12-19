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

const std::string keys =
  "{help h usage ? |                  | 输出命令行参数说明}"
  "{@config-path   | configs/uav.yaml | yaml配置文件路径 }";

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
  tools::Plotter plotter;
  tools::Recorder recorder;
  
  // io::Camera camera(config_path);
  // io::CBoard cboard(config_path);
  io::Gimbal gimbal(config_path);
  io::Camera camera(config_path);
  
  auto_aim::Detector detector(config_path);
  auto_aim::Solver solver(config_path);
  auto_aim::YOLO yolo(config_path);
  auto_aim::Shooter shooter(config_path);
  auto_aim::Predictor predictor(config_path,solver);
  auto_aim::armor_aimer armor_aimer(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  // auto mode = io::Mode::idle; //修改为gimbalmode
  auto mode = io::GimbalMode::IDLE;
  // auto last_mode = io::Mode::idle;
  auto last_mode = io::GimbalMode::IDLE;
  auto t0 = std::chrono::steady_clock::now();

  // 自瞄流程
  while (!exiter.exit()) {
    camera.read(img, t);
    // q = cboard.imu_at(t - 1ms);
    q = gimbal.q(t - 1ms);
    // mode = cboard.mode;
    mode = gimbal.mode();
    // recorder.record(img, q, t);
    if (last_mode != mode) {
      tools::logger()->info("Switch to {}", gimbal.str(mode));
      last_mode = mode;
    }
    /// 自瞄
    solver.set_R_gimbal2world(q);

    Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    auto armors = detector.detect(img);
    for(auto & armor:armors){
      solver.solve(armor);
    }
    auto_aim::Predictor::PredictResult prediction;
    bool has_prediction = false;

    if (!armors.empty()) {
        prediction = predictor.track(armors.front(), t);  // ← 传入 Armor 和时间戳
        has_prediction = prediction.valid;
    }
    std::list<auto_aim::Predictor::PredictResult> predictions = {prediction};
    auto command = armor_aimer.aim(predictions, t, 10);
    // command.shoot = shooter.shoot(command, armor_aimer, predictions, ypr);
    /* 暂不引入shooter */
    //yaw,pitch范围为[-180,180],故需要增大180度
    auto wrap_rad_2pi = [](double rad) {
    double wrapped = std::fmod(rad, 2 * M_PI);
    if (wrapped < 0) wrapped += 2 * M_PI;
    return wrapped;
    };

    command.yaw = wrap_rad_2pi(command.yaw);
    command.pitch = wrap_rad_2pi(command.pitch);
    gimbal.send_command_scm(command);
    nlohmann::json data;
    data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);
    // 装甲板原始观测数据
    data["armor_num"] = armors.size();
    if (!armors.empty()) {
      auto min_x = 1e10;
      auto & armor = armors.front();
      for (auto & a : armors) {
        if (a.center.x < min_x) {
          min_x = a.center.x;
          armor = a;
        }
      }  //always left
      solver.solve(armor);
      //armor.name是字符串，转换为特定数字
      int name_send  =0;
      if (armor.name == auto_aim::ArmorName::outpost)
      {
        name_send = 18;
      }
      
      data["armor_name"] = name_send;
  
    // 使用solver重新投影predictor四个顶点并绘制
    if (prediction.valid)
    {
      // 重投影预测结果（绿色框）
  auto reprojected_points = solver.reproject_armor(
    prediction.position,      // xyz_in_world
    prediction.yaw,           // yaw
    prediction.armor_type,    // type
    prediction.armor_name     // name
  );
  tools::draw_points(img, reprojected_points, {0, 255, 0}, 2);
  
  // 也可以重投影原始观测（红色框对比）
  auto original_points = solver.reproject_armor(
    armors.front().xyz_in_world,
    armors.front().ypr_in_world[0],
    armors.front().type,
    armors.front().name
  );
  tools::draw_points(img, original_points, {0, 0, 255}, 1);
}
  // 重投影 debug_aim_point（蓝色框）
if (armor_aimer.debug_aim_point.valid) {
  auto aim_points = solver.reproject_armor(
    armor_aimer.debug_aim_point.xyza.head(3),  // xyz
    armor_aimer.debug_aim_point.xyza[3],       // yaw
    prediction.armor_type,
    prediction.armor_name
  );
  tools::draw_points(img, aim_points, {255, 0, 0}, 2);
}
    }
    

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
      data["cmd_yaw"] = command.yaw * 57.3;
      data["cmd_pitch"] = command.pitch * 57.3;
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

    // ===== 7. 显示 =====
    tools::draw_text(img, fmt::format("[{}]", predictor.state()), {10, 30}, {255, 255, 255});
    
    cv::resize(img, img, {}, 0.5, 0.5);
    cv::imshow("predictor_debug", img);
    auto key = cv::waitKey(1);
    if (key == 'q') break;
}
return 0;
}

