#ifndef AUTO_AIM__AIMER_AIMER_HPP
#define AUTO_AIM__AIMER_AIMER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <list>

#include "io/gimbal/gimbal.hpp" //使用c板时可尝试改为io/cboard.hpp
#include "io/command.hpp"
#include "predictor.hpp"
#include "aimer.hpp"  
namespace auto_aim
{
// struct AimPoint
// {
//     bool valid;
//     Eigen::Vector4d xyza;
// };
class armor_aimer
{
public:
    AimPoint debug_aim_point;
    explicit armor_aimer (const std::string & config_path);
    io::Command aim(
        std::list<Predictor::PredictResult> predictions, 
        std::chrono::steady_clock::time_point timestamp, 
        double bullet_speed,
        bool to_now = true);
    //暂不引入射击模式，默认单发，后续考虑增加


private:
    double yaw_offset_;
    double pitch_offset_;
    double comming_angle_;
    double leaving_angle_;
    // double lock_id_ = -1; //暂不引入锁定功能
    double delay_time_;
    
    double last_yaw_ = 0.0;  // 用于角度连续化
    bool first_command_ = true;  // 首次命令标记

    AimPoint choose_aim_point(const Predictor::PredictResult & prediction);
};

} // namespace auto_aim

#endif
