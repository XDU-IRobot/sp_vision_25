#include "armor_aimer.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <vector>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"

namespace auto_aim
{
armor_aimer::armor_aimer(const std::string & config_path)
{
    YAML::Node config = YAML::LoadFile(config_path);

    yaw_offset_ = config["yaw_offset"].as<double>();
    pitch_offset_ = config["pitch_offset"].as<double>();
    delay_time_ = config["delay_time"].as<double>();
    // 后续考虑加入左右yaw来适配英雄
}
io::Command armor_aimer::aim(
    std::list<Predictor::PredictResult> predictions, 
    std::chrono::steady_clock::time_point timestamp, 
    double bullet_speed,
    bool to_now)
{
    io::Command command{};
    // 目前默认单发
    if (predictions.empty())
    {
        command.shoot = false;
        command.yaw = 0.0;
        command.pitch = 0.0;
        return command;
    }

    // 选择第一个目标进行瞄准
    const auto & target = predictions.front();

    // 计算延时
    double delay_time = delay_time_;

    // 预测目标位置
    Eigen::Vector3d predicted_position = target.position;

    // 计算瞄准点
    AimPoint aim_point = choose_aim_point(target);
    debug_aim_point = aim_point;
    if (!aim_point.valid) {
        tools::logger()->debug("[armor_aimer] Invalid aim point");
        command.shoot = false;
        debug_aim_point.valid = false;
        command = {false, false, 0.0, 0.0};
        return command;
    }
    // 计算弹道
    Eigen::Vector3d xyz = aim_point.xyza.head(3);
    double d = std::sqrt(xyz.x() * xyz.x() + xyz.y() * xyz.y());
    
    tools::Trajectory trajectory(bullet_speed, d, xyz.z());

    if (trajectory.unsolvable) {
        tools::logger()->debug(
            "[armor_aimer] Unsolvable trajectory: speed={:.2f}, d={:.2f}, z={:.2f}",
            bullet_speed, d, xyz.z());
        debug_aim_point.valid = false;
        return {false, false, 0.0, 0.0};
    }
    /*todo: 优化弹道迭代*/

    // 计算最终角度
    /*todo: 角度计算 && 世界系的转换 */
    Eigen::Vector3d final_xyz = debug_aim_point.xyza.head(3);
    double yaw = std::atan2(final_xyz.y(), final_xyz.x()) + yaw_offset_;
    double pitch = -(trajectory.pitch + pitch_offset_);  //世界坐标系下pitch向上为负
    return {true, false, yaw, pitch};
    command.yaw = yaw;
    command.pitch = pitch;
    command.shoot = true;

    // 调试信息
    // debug_aim_point.valid = true;
    // debug_aim_point.xyza.head(3) = predicted_position;
    // debug_aim_point.xyza[3] = prediction.yaw + prediction.yaw_rate * fly_time

    /*todo : 优化aim函数*/

    return command;
}
AimPoint armor_aimer::choose_aim_point(const Predictor::PredictResult & prediction)
{   
    /*todo: 根据球坐标选择瞄准点逻辑  */
    if (!prediction.valid) {
        tools::logger()->warn("[armor_aimer] Invalid prediction result");
        return {false, {}};
    }
    
    Eigen::Vector4d xyza;
    xyza << prediction.position.x(),  // x
            prediction.position.y(),  // y
            prediction.position.z(),  // z
            prediction.yaw;           // a (yaw)
    
    return {true, xyza};
}
} // namespace auto_aim