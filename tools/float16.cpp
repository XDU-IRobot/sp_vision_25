#include "float16.hpp"
#include <cmath>
#include <cstring>

namespace f16tools {

    // 定义 f16 类型为 uint16_t 表示半精度浮点数
    using f16 = uint16_t;

    // 将 f32 转换为 f16
    f16 f32_to_f16(float value) {
        uint32_t f32_bits;
        std::memcpy(&f32_bits, &value, sizeof(float));

        uint32_t sign = (f32_bits >> 31) & 0x1;
        int32_t exponent = ((f32_bits >> 23) & 0xFF) - 127 + 15;
        uint32_t mantissa = (f32_bits & 0x7FFFFF);

        if (exponent <= 0) {
            // Underflow: 转换为零
            return static_cast<f16>(sign << 15);
        } else if (exponent >= 31) {
            // Overflow: 转换为无穷大
            return static_cast<f16>((sign << 15) | 0x7C00);
        }

        // 正常情况
        return static_cast<f16>((sign << 15) | (exponent << 10) | (mantissa >> 13));
    }

    // 将 f16 转换为 f32
    float f16_to_f32(f16 value) {
        uint32_t sign = (value >> 15) & 0x1;
        int32_t exponent = ((value >> 10) & 0x1F) - 15 + 127;
        uint32_t mantissa = (value & 0x3FF);

        uint32_t f32_bits = (sign << 31) | (exponent << 23) | (mantissa << 13);
        float result;
        std::memcpy(&result, &f32_bits, sizeof(float));
        return result;
    }

    double f16_to_f64(f16 value) {
        uint64_t sign = (value >> 15) & 0x1;
        int64_t exponent = ((value >> 10) & 0x1F) - 15 + 1023; // 转换为 double 的偏移量
        uint64_t mantissa = (value & 0x3FF);

        uint64_t f64_bits = (sign << 63) | (exponent << 52) | (mantissa << 42);
        double result;
        std::memcpy(&result, &f64_bits, sizeof(double));
        return result;
    }

    // 将 f64 转换为 f16
    f16 f64_to_f16(double value) {
        uint64_t f64_bits;
        std::memcpy(&f64_bits, &value, sizeof(double));

        uint64_t sign = (f64_bits >> 63) & 0x1;
        int64_t exponent = ((f64_bits >> 52) & 0x7FF) - 1023 + 15; // 转换为 f16 的偏移量
        uint64_t mantissa = (f64_bits & 0xFFFFFFFFFFFFF);

        if (exponent <= 0) {
            // Underflow: 转换为零
            return static_cast<f16>(sign << 15);
        } else if (exponent >= 31) {
            // Overflow: 转换为无穷大
            return static_cast<f16>((sign << 15) | 0x7C00);
        }

        // 正常情况
        return static_cast<f16>((sign << 15) | (exponent << 10) | (mantissa >> 42));
    }

}  // namespace f16tools