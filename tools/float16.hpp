#ifndef FLOAT16_HPP
#define FLOAT16_HPP
#include <cstdint>
namespace f16tools {

    // 定义 f16 类型为 uint16_t 表示半精度浮点数
    using f16 = uint16_t;
    
    // 将 f32 转换为 f16
    f16 f32_to_f16(float value);
    
    // 将 f16 转换为 f32
    float f16_to_f32(f16 value);
    double f16_to_f64(f16 value);
    f16 f64_to_f16(double value);


    }  // namespace f16tools

#endif // FLOAT16_HPP