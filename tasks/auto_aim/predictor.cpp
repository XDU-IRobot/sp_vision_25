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
    auto yaml = YAML::LoadFile(config_path);
    enemy_color_ = (yaml["enemy_color"].as<std::string>() == "red") ? Color::red : Color::blue;

};

Predictor::PredictResult Predictor::track(const Armor & armor, std::chrono::steady_clock::time_point t)
{
    auto logger = tools::logger();
    auto dt = tools::delta_time(t, last_timestamp_);
    last_timestamp_ = t;
    auto_aim::Armor armor_copy = armor;
  // 时间间隔过长，说明可能发生了相机离线
    if (state_ != "lost" && dt > 0.1) {
    tools::logger()->warn("[Tracker] Large dt: {:.3f}s", dt);
    state_ = "lost";
  }
  // 过滤掉非我方装甲板
  // armors_.remove_if([&](const auto_aim::Armor & a) { return a.color != enemy_color_; });
}
Eigen::Vector3d Predictor::diff_velocity(const Eigen::Vector3d & curr_pos, double dt)
{
    return (curr_pos - last_pos) / dt;
}
void Predictor::update_ewma(Eigen::Vector3d & curr_pos, Eigen::Vector3d & vel)
{
    
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
    last_timestamp_ = t;

}
} // namespace auto_aim