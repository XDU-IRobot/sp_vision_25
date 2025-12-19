#include "tasks/auto_aim/predictor.hpp"
#include <yaml-cpp/yaml.h>

#include <tuple>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
Predictor::Predictor(const std::string & config_path, Solver & solver)
: solver_(solver), state_("lost")
{
    YAML::Node config = YAML::LoadFile(config_path);
    enemy_color_ = (config["enemy_color"].as<std::string>() == "red") ? Color::red : Color::blue;
    // 读取预测参数
    default_horizon_ = config["predict_time"].as<double>();  // 例如 0.090
    pos_alpha_ = config["pos_alpha"].as<double>() ;  // EWMA 系数
    vel_alpha_ = config["vel_alpha"].as<double>() ;
};

Predictor::PredictResult Predictor::track(const Armor & armor, std::chrono::steady_clock::time_point t)
{
    auto logger = tools::logger();
    auto dt = tools::delta_time(t, last_timestamp_);
    // last_timestamp_ = t;
    auto_aim::Armor armor_copy = armor;
  // 时间间隔过长，说明可能发生了相机离线
    if (state_ != "lost" && dt > 0.1) {
    tools::logger()->warn("[Tracker] Large dt: {:.3f}s", dt);
    state_ = "lost";
  }
  // 过滤掉非我方装甲板
  // armors_.remove_if([&](const auto_aim::Armor & a) { return a.color != enemy_color_; });
    ingest(armor, t);
    last_timestamp_ = t;
    if (!ready()) {
    return {false, t, {}, {}, 0, 0, 0, ArmorType::small, ArmorName::not_armor, 0};
  }
    auto pos_pred = predict_pos(default_horizon_);
    return {
        .valid = true,
        .stamp = t,
        .position = pos_pred,
        .velocity = vel_ewma,
        .yaw = armor_copy.ypr_in_world[0],
        .yaw_rate = 0,
        .horizon = default_horizon_,
        .armor_type = armor_copy.type,
        .armor_name = armor_copy.name,
        .pitch = armor_copy.ypr_in_world[1]
    };
}
Eigen::Vector3d Predictor::diff_velocity(const Eigen::Vector3d & curr_pos, double dt)
{
    // 异常检测：时间间隔过小
    if (dt < 1e-6) {
        tools::logger()->warn("[Predictor] dt too small: {:.6f}s, returning zero velocity", dt);
        return Eigen::Vector3d::Zero();
    }
    
    auto vel = (curr_pos - last_pos) / dt;
    
    // 异常检测：速度过大（可能是噪声或跳变）
    constexpr double MAX_VELOCITY = 10.0;  // 最大速度阈值 (m/s)
    if (vel.norm() > MAX_VELOCITY) {
        tools::logger()->warn("[Predictor] Abnormal velocity detected: {:.2f} m/s, clamping", vel.norm());
        return vel_ewma;  // 返回上一次平滑值
    }
    
    return vel;
}
void Predictor::update_ewma(const Eigen::Vector3d & curr_pos, const Eigen::Vector3d & vel)
{
    pos_ewma = pos_alpha_ * curr_pos + (1.0 - pos_alpha_) * pos_ewma;
    vel_ewma = vel_alpha_ * vel + (1.0 - vel_alpha_) * vel_ewma;
}
Eigen::Vector3d Predictor::predict_pos(double horizon_sec) const
{
    if (!ready()) {
        return Eigen::Vector3d::Zero();
    }
    // 预测位置 = 当前位置信息 + 速度信息 * 前瞻时间
    return pos_ewma + vel_ewma * horizon_sec;
}
bool Predictor::ready() const
{
    return state_ == "tracking";
}
void Predictor::ingest(const Armor & armor, std::chrono::steady_clock::time_point t)
{
    auto logger = tools::logger();
    auto dt = tools::delta_time(t, last_timestamp_);
    if (dt > 0.1) {
    tools::logger()->warn("[Predictor] Large dt: {:.3f}s, reset", dt);
    state_ = "lost";
    return;
    }
    auto curr_pos = armor.xyz_in_world;
    // 首次初始化
    if (state_ == "lost") {
    pos_ewma = curr_pos;
    vel_ewma = Eigen::Vector3d::Zero();
    state_ = "tracking";
  } else {
    // 差分速度
    auto vel = diff_velocity(curr_pos, dt);
    
    // EWMA 更新
    update_ewma(curr_pos, vel);
  }
  
  last_pos = curr_pos;
  last_timestamp_ = t;
  sample_count_++;
}
} // namespace auto_aim