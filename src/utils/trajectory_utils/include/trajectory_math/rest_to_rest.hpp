#ifndef TRAJECTORY_MATH_REST_TO_REST_HPP
#define TRAJECTORY_MATH_REST_TO_REST_HPP

// 7 阶 rest-to-rest 时间尺度平滑核(单一真相源)。
// 被 ctrl_node(LAND 降落高度曲线) 与 trajectory_utils(轨迹生成) 共用, 替代原先两份
// 逐字相同但各自维护的副本(ctrl_fsm.cc 注释曾写"复制自 trajectory_utils"), 避免分叉。
//
//   smoothP/V/A/J = 位置/速度/加速度/加加速度轮廓; clamp01 限幅到 [0,1];
//   smoothPIntegral 仅用于起始速度混合(保持 p=∫v 一致)。
//   边界 r=0/r=1 处 P=0/1, V/A/J=0, 内部连续。

#include <algorithm>
#include <cmath>

namespace trajectory_math
{
    inline double clamp01(double value)
    {
        return std::max(0.0, std::min(1.0, value));
    }

    // Position: 35r⁴ - 84r⁵ + 70r⁶ - 20r⁷
    inline double smoothP(double r)
    {
        r = clamp01(r);
        const double r2 = r * r;
        const double r3 = r2 * r;
        const double r4 = r3 * r;
        return 35.0 * r4 - 84.0 * r4 * r + 70.0 * r4 * r2 - 20.0 * r4 * r3;
    }

    // Velocity: 140 r³ (1-r)³
    inline double smoothV(double r)
    {
        r = clamp01(r);
        const double one_minus_r = 1.0 - r;
        return 140.0 * r * r * r * one_minus_r * one_minus_r * one_minus_r;
    }

    // Acceleration: 420 r² (1-r)² (1-2r)
    inline double smoothA(double r)
    {
        r = clamp01(r);
        const double one_minus_r = 1.0 - r;
        return 420.0 * r * r * one_minus_r * one_minus_r * (1.0 - 2.0 * r);
    }

    // Jerk: 840 r (1-r) (1 - 5r + 5r²)  [仅 trajectory_utils 使用]
    inline double smoothJ(double r)
    {
        r = clamp01(r);
        return 840.0 * r * (1.0 - r) * (1.0 - 5.0 * r + 5.0 * r * r);
    }

    // Integral of smoothP with G(0)=0, G(1)=0.5 [仅 trajectory_utils 起始速度混合用]
    inline double smoothPIntegral(double r)
    {
        r = clamp01(r);
        const double r5 = r * r * r * r * r;
        return r5 * (7.0 - 14.0 * r + 10.0 * r * r - 2.5 * r * r * r);
    }
} // namespace trajectory_math

#endif // TRAJECTORY_MATH_REST_TO_REST_HPP
