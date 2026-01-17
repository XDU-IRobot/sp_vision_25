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

    // 配置文件中的 offset 是角度制，需要转换为弧度
    yaw_offset_ = config["yaw_offset"].as<double>() * M_PI / 180.0;
    pitch_offset_ = config["pitch_offset"].as<double>() * M_PI / 180.0;
    delay_time_ = config["delay_time"].as<double>();
    
    tools::logger()->info(
        "[armor_aimer] yaw_offset={:.3f} rad ({:.1f}°), pitch_offset={:.3f} rad ({:.1f}°)",
        yaw_offset_, yaw_offset_ * 180.0 / M_PI,
        pitch_offset_, pitch_offset_ * 180.0 / M_PI
    );
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
    
    // 角度连续化：消除 ±180° 跳变
    if (!first_command_) {
        // 计算与上一次 yaw 的差值（考虑周期性）
        double delta = yaw - last_yaw_;
        
        // 将差值归一化到 [-π, π]
        while (delta > M_PI) delta -= 2 * M_PI;
        while (delta < -M_PI) delta += 2 * M_PI;
        
        // 根据归一化后的差值更新 yaw（保持连续性）
        yaw = last_yaw_ + delta;
    }
    
    // 更新历史值
    last_yaw_ = yaw;
    first_command_ = false;
    
    // 调试日志
    tools::logger()->debug(
        "[armor_aimer] xyz=({:.3f},{:.3f},{:.3f}), d={:.3f}, "
        "traj.pitch={:.4f} rad ({:.2f}°), yaw={:.4f} rad ({:.2f}°), pitch={:.4f} rad ({:.2f}°)",
        final_xyz.x(), final_xyz.y(), final_xyz.z(), d,
        trajectory.pitch, trajectory.pitch * 57.3,
        yaw, yaw * 57.3,
        pitch, pitch * 57.3
    );
    
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