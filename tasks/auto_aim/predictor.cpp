#include "tasks/auto_aim/predictor.hpp"
#include <yaml-cpp/yaml.h>

#include <tuple>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/marker_publisher.hpp"
namespace auto_aim
{
Predictor::Predictor(const std::string & config_path, Solver & solver)
: solver_(solver), 
  state_("lost"),
  sample_count_(0),
  last_timestamp_(std::chrono::steady_clock::now())
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
    logger->debug("[Predictor::track] dt={:.6f}s", dt);
    // last_timestamp_ = t;
    auto_aim::Armor armor_copy = armor;
  // 时间间隔过长，说明可能发生了相机离线
    if (state_ != "lost" && dt > 0.1) {
    tools::logger()->warn("[Predictor] Large dt: {:.3f}s", dt);
    state_ = "lost";
    sample_count_ = 0;
  }
  // 过滤掉非我方装甲板
  // armors_.remove_if([&](const auto_aim::Armor & a) { return a.color != enemy_color_; });
    ingest(armor, t);
    // 删除重复更新：last_timestamp_ = t;
    if (!ready()) {
    tools::logger()->debug("[Predictor] Not ready: state={}, sample_count={}", state_, sample_count_);
    return {false, t, {}, {}, 0, 0, 0, ArmorType::small, ArmorName::not_armor, 0};
  }
    
    last_armor_type = armor_copy.type;
    last_armor_name = armor_copy.name;
    last_yaw = armor_copy.ypr_in_world[0];
    last_pitch = armor_copy.ypr_in_world[1];  

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
    return state_ == "tracking" && sample_count_ >= 2;
}
void Predictor::ingest(const Armor & armor, std::chrono::steady_clock::time_point t)
{
    auto logger = tools::logger();
    auto dt = tools::delta_time(t, last_timestamp_);
    logger->debug("[Predictor::ingest] dt={:.6f}s, state={}", dt, state_);
    
    // 检查时间间隔（但这里不需要重复检查，因为 track() 已经检查过了）
    // 只在状态为 tracking 时检查，避免首次调用时触发
    if (state_ != "lost" && dt > 0.1) {
        tools::logger()->warn("[Predictor] ingest: Large dt: {:.3f}s, reset", dt);
        state_ = "lost";
        sample_count_ = 0;
        return;
    }
    
    auto curr_pos = armor.xyz_in_world;
    
    // 首次初始化
    if (state_ == "lost") {
        pos_ewma = curr_pos;
        vel_ewma = Eigen::Vector3d::Zero();
        state_ = "tracking";
        sample_count_ = 1;
        tools::logger()->info("[Predictor] Initialized, sample_count=1");
    } else {
        // 差分速度
        auto vel = diff_velocity(curr_pos, dt);
        
        // EWMA 更新
        update_ewma(curr_pos, vel);
        sample_count_++;
        
        if (sample_count_ == 2) {
            tools::logger()->info("[Predictor] Ready! sample_count=2, vel_norm={:.3f}", vel.norm());
        }
    }
    
    last_pos = curr_pos;
    last_timestamp_ = t;
}
void Predictor::set_marker_publisher(tools::MarkerPublisher* marker_pub)
{
    marker_pub_ = marker_pub;
}
void Predictor::visualize(int base_id) const
{
    // 可视化当前估计（绿色）
    if (!marker_pub_ || !ready()) {
        tools::logger()->warn("[Predictor] Marker publisher not set or isnot ready");
        return;
    }
    Eigen::Quaternion q_current(
        Eigen::AngleAxisd(last_yaw,Eigen::Vector3d::UnitZ())*
        Eigen::AngleAxisd(last_pitch,Eigen::Vector3d::UnitY())
    );
    double width = (last_armor_type == ArmorType::small) ? 0.135 : 0.23;
    double height = 0.1;
    marker_pub_->publishArmorMarker(
        "world", base_id+1, pos_ewma, q_current, width, height, 0, 255, 0
    );
    // 可视化速度向量
    if (vel_ewma.norm() > 0.01) {  // 只在有明显速度时显示
        Eigen::Vector3d arrow_end = pos_ewma + vel_ewma * 0.5;  // 缩放速度向量便于观察
        marker_pub_->publishArrowMarker(
            "world", base_id + 1,
            pos_ewma, arrow_end,
            0, 255, 255  // 青色
        );
    }
    Eigen::Vector3d predicted_pos = predict_pos(default_horizon_);
    Eigen::Quaternion q_predicted = q_current; // 假设姿态不变
    marker_pub_->publishArmorMarker(
        "world", base_id + 2, predicted_pos, q_predicted, width, height, 255, 255, 0.5
    );

}

} // namespace auto_aim