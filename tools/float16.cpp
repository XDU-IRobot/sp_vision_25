#include "float16.hpp"
#include <cmath>
#include <cstring>

namespace f16tools {

// IEEE 754 half-precision (f16) 格式:
// - 1 bit: sign (符号位)
// - 5 bits: exponent (指数位, bias=15)
// - 10 bits: mantissa/fraction (尾数位)
//
// IEEE 754 single-precision (f32) 格式:
// - 1 bit: sign
// - 8 bits: exponent (bias=127)
// - 23 bits: mantissa
//
// IEEE 754 double-precision (f64) 格式:
// - 1 bit: sign
// - 11 bits: exponent (bias=1023)
// - 52 bits: mantissa

// ============================================================================
// f32 -> f16 转换
// ============================================================================
f16 f32_to_f16(float value) {
    uint32_t f32_bits;
    std::memcpy(&f32_bits, &value, sizeof(float));

    // 提取 f32 的各个组成部分
    uint32_t sign = (f32_bits >> 31) & 0x1;
    uint32_t exp_bits = (f32_bits >> 23) & 0xFF;
    uint32_t mantissa = f32_bits & 0x7FFFFF;

    // 处理特殊情况：Infinity 或 NaN (exponent 全为 1)
    if (exp_bits == 0xFF) {
        if (mantissa == 0) {
            // ±Infinity
            return static_cast<f16>((sign << 15) | 0x7C00);
        } else {
            // NaN: 保留 NaN 标记，取 mantissa 的高位
            uint32_t nan_mantissa = mantissa >> 13;
            if (nan_mantissa == 0) nan_mantissa = 1; // 确保 NaN 的 mantissa 非零
            return static_cast<f16>((sign << 15) | 0x7C00 | nan_mantissa);
        }
    }

    // 计算无偏指数
    int32_t unbiased_exp = static_cast<int32_t>(exp_bits) - 127;
    int32_t f16_exp = unbiased_exp + 15;

    // 处理溢出：转换为 ±Infinity
    if (f16_exp >= 0x1F) {
        return static_cast<f16>((sign << 15) | 0x7C00);
    }

    // 处理下溢和 subnormal numbers
    if (f16_exp <= 0) {
        // 严重下溢，返回 ±0
        if (f16_exp < -10) {
            return static_cast<f16>(sign << 15);
        }

        // Subnormal number: 需要调整 mantissa
        // 添加隐藏的 1 位
        mantissa |= 0x800000;

        // 右移位数 = 14 - f16_exp (14 = 23 - 10 + 1)
        int shift = 14 - f16_exp;
        uint32_t round_bit = 1u << (shift - 1);
        uint32_t tie_mask = (3u << (shift - 1)) - 1;

        // 舍入：round to nearest, ties to even
        if ((mantissa & round_bit) && (mantissa & tie_mask)) {
            mantissa += round_bit;
        }

        uint32_t f16_mantissa = mantissa >> shift;
        return static_cast<f16>((sign << 15) | f16_mantissa);
    }

    // 正常数值转换
    // mantissa 从 23 位截断到 10 位，需要舍入
    uint32_t f16_mantissa = mantissa >> 13;

    // 舍入逻辑：round to nearest, ties to even
    // round_bit: 第 13 位（从右数，0-indexed）
    // 如果 round_bit=1 且（后面还有非零位 或 当前结果是奇数），则进位
    uint32_t round_bit = 0x1000; // bit 12
    uint32_t remaining_bits = mantissa & 0x1FFF; // 低 13 位

    if ((remaining_bits > round_bit) ||
        (remaining_bits == round_bit && (f16_mantissa & 0x1))) {
        f16_mantissa++;

        // 检查进位是否导致 mantissa 溢出
        if (f16_mantissa > 0x3FF) {
            f16_mantissa = 0;
            f16_exp++;

            // 检查指数溢出
            if (f16_exp >= 0x1F) {
                return static_cast<f16>((sign << 15) | 0x7C00); // Infinity
            }
        }
    }

    return static_cast<f16>((sign << 15) | (f16_exp << 10) | f16_mantissa);
}

// ============================================================================
// f16 -> f32 转换
// ============================================================================
float f16_to_f32(f16 value) {
    // 提取 f16 的各个组成部分
    uint32_t sign = (value >> 15) & 0x1;
    uint32_t exp_bits = (value >> 10) & 0x1F;
    uint32_t mantissa = value & 0x3FF;

    uint32_t f32_bits;

    // 处理特殊情况：±0
    if (exp_bits == 0 && mantissa == 0) {
        f32_bits = sign << 31;
    }
    // 处理特殊情况：Infinity 或 NaN
    else if (exp_bits == 0x1F) {
        if (mantissa == 0) {
            // ±Infinity
            f32_bits = (sign << 31) | 0x7F800000;
        } else {
            // NaN
            f32_bits = (sign << 31) | 0x7FC00000 | (mantissa << 13);
        }
    }
    // 处理 subnormal numbers
    else if (exp_bits == 0) {
        // 计算前导零个数
        int shift = 0;
        uint32_t temp = mantissa;
        while (temp && !(temp & 0x200)) { // 0x200 = bit 9 (最高位)
            temp <<= 1;
            shift++;
        }

        // 调整指数和尾数
        int32_t f32_exp = 127 - 15 - shift;
        uint32_t f32_mantissa = (mantissa << (14 + shift)) & 0x7FFFFF;
        f32_bits = (sign << 31) | (f32_exp << 23) | f32_mantissa;
    }
    // 正常数值
    else {
        int32_t unbiased_exp = static_cast<int32_t>(exp_bits) - 15;
        int32_t f32_exp = unbiased_exp + 127;
        uint32_t f32_mantissa = mantissa << 13;
        f32_bits = (sign << 31) | (f32_exp << 23) | f32_mantissa;
    }

    float result;
    std::memcpy(&result, &f32_bits, sizeof(float));
    return result;
}

// ============================================================================
// f64 -> f16 转换
// ============================================================================
f16 f64_to_f16(double value) {
    uint64_t f64_bits;
    std::memcpy(&f64_bits, &value, sizeof(double));

    // 提取 f64 的各个组成部分
    uint64_t sign = (f64_bits >> 63) & 0x1;
    uint64_t exp_bits = (f64_bits >> 52) & 0x7FF;
    uint64_t mantissa = f64_bits & 0xFFFFFFFFFFFFFull;

    // 处理特殊情况：Infinity 或 NaN
    if (exp_bits == 0x7FF) {
        if (mantissa == 0) {
            // ±Infinity
            return static_cast<f16>((sign << 15) | 0x7C00);
        } else {
            // NaN
            uint64_t nan_mantissa = mantissa >> 42;
            if (nan_mantissa == 0) nan_mantissa = 1;
            return static_cast<f16>((sign << 15) | 0x7C00 | static_cast<uint16_t>(nan_mantissa));
        }
    }

    // 计算无偏指数
    int64_t unbiased_exp = static_cast<int64_t>(exp_bits) - 1023;
    int64_t f16_exp = unbiased_exp + 15;

    // 处理溢出
    if (f16_exp >= 0x1F) {
        return static_cast<f16>((sign << 15) | 0x7C00);
    }

    // 处理下溢和 subnormal numbers
    if (f16_exp <= 0) {
        // 严重下溢
        if (f16_exp < -10) {
            return static_cast<f16>(sign << 15);
        }

        // Subnormal number
        mantissa |= 0x10000000000000ull; // 添加隐藏的 1 位

        int shift = 43 - static_cast<int>(f16_exp); // 43 = 52 - 10 + 1
        uint64_t round_bit = 1ull << (shift - 1);
        uint64_t tie_mask = (3ull << (shift - 1)) - 1;

        if ((mantissa & round_bit) && (mantissa & tie_mask)) {
            mantissa += round_bit;
        }

        uint64_t f16_mantissa = mantissa >> shift;
        return static_cast<f16>((sign << 15) | static_cast<uint16_t>(f16_mantissa));
    }

    // 正常数值转换
    // mantissa 从 52 位截断到 10 位
    uint64_t f16_mantissa = mantissa >> 42;

    // 舍入逻辑
    uint64_t round_bit = 0x20000000000ull; // bit 41
    uint64_t remaining_bits = mantissa & 0x3FFFFFFFFFFull; // 低 42 位

    if ((remaining_bits > round_bit) ||
        (remaining_bits == round_bit && (f16_mantissa & 0x1))) {
        f16_mantissa++;

        if (f16_mantissa > 0x3FF) {
            f16_mantissa = 0;
            f16_exp++;

            if (f16_exp >= 0x1F) {
                return static_cast<f16>((sign << 15) | 0x7C00);
            }
        }
    }

    return static_cast<f16>((sign << 15) | (f16_exp << 10) | f16_mantissa);
}

// ============================================================================
// f16 -> f64 转换
// ============================================================================
double f16_to_f64(f16 value) {
    // 提取 f16 的各个组成部分
    uint64_t sign = (value >> 15) & 0x1;
    uint64_t exp_bits = (value >> 10) & 0x1F;
    uint64_t mantissa = value & 0x3FF;

    uint64_t f64_bits;

    // 处理特殊情况：±0
    if (exp_bits == 0 && mantissa == 0) {
        f64_bits = sign << 63;
    }
    // 处理特殊情况：Infinity 或 NaN
    else if (exp_bits == 0x1F) {
        if (mantissa == 0) {
            // ±Infinity
            f64_bits = (sign << 63) | 0x7FF0000000000000ull;
        } else {
            // NaN
            f64_bits = (sign << 63) | 0x7FF8000000000000ull | (mantissa << 42);
        }
    }
    // 处理 subnormal numbers
    else if (exp_bits == 0) {
        // 计算前导零个数
        int shift = 0;
        uint64_t temp = mantissa;
        while (temp && !(temp & 0x200)) {
            temp <<= 1;
            shift++;
        }

        // 调整指数和尾数
        int64_t f64_exp = 1023 - 15 - shift;
        uint64_t f64_mantissa = (mantissa << (43 + shift)) & 0xFFFFFFFFFFFFFull;
        f64_bits = (sign << 63) | (f64_exp << 52) | f64_mantissa;
    }
    // 正常数值
    else {
        int64_t unbiased_exp = static_cast<int64_t>(exp_bits) - 15;
        int64_t f64_exp = unbiased_exp + 1023;
        uint64_t f64_mantissa = mantissa << 42;
        f64_bits = (sign << 63) | (f64_exp << 52) | f64_mantissa;
    }

    double result;
    std::memcpy(&result, &f64_bits, sizeof(double));
    return result;
}

}  // namespace f16tools
