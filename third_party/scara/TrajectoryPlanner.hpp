// file: TrajectoryPlanner.hpp
#pragma once
#include <vector>
#include "timesync.h"

/**
 * @brief 轨迹点状态
 * @cite 包含位置、速度、加速度
 */
struct StatePoint {
    double pos;   // position
    double vel;   // velocity
    double acc;   // acceleration
};


class TrajectoryPlanner {
public:
    TrajectoryPlanner(double Vmax_, double Amax_, double Jmax_)  : Vmax(Vmax_), Amax(Amax_), Jmax(Jmax_),
                                                                   quinticCoff_(6)
    {}
    TrajectoryPlanner() : Vmax(), Amax(), Jmax(), quinticCoff_(6) { }
    ~TrajectoryPlanner() = default;

    // /**
    //  * @brief 五次多项式系数
    //  * @cite 包含位置、速度、加速度的五次多项式系数
    //  */
    // struct QuinticCoeff {
    //     double c0, c1, c2, c3, c4, c5;
    // };

    bool point_changed{};
    // 设置当前状态（起点）
    void setStartState(const double p_target, const double v_target, const double a_target);
    void setStartState(const StatePoint &start);

    // 请求一个新目标位置，期望终端速度为 0（可扩展为非零）
    void setTarget(const double p_target, const double v_target = 0.0, const double a_target = 0.0);
    void setTarget(const StatePoint &target);
    // 在时间 t（从规划起点为 t=0）处采样状态
    StatePoint sample(double t) const;
    // 在频率 freq 处采样状态
    StatePoint sample(int freq) const;
    // 返回当前轨迹的总时长
    double getTotalTime() const;

    int    getFrequency() const;

    void refineTrajectoryTime(const StatePoint &s0, const StatePoint &sf, double tol);
    
private:
    double Vmax;
    double Amax;
    double Jmax;

    StatePoint startp_{};
    StatePoint targetp_{};
    int8_t direction_{};
    double t_accel_{};         // 加速时间
    double t_cruise_{};        // 匀速时间
    double t_decel_{};         // 减速时间

    double T_{};                // 总时间
    int Freq_{};              // 总周期
    std::vector<double> quinticCoff_;      // 五次多项式系数

    // 生成 S-curve 时间段，计算运动总时间 T 及规划起点／终点边界
    void computeSCurveTime();

    // 根据 T 和起末状态，生成五次多项式系数
    void computeQuintic();

    // 计算多项式在 t 处的状态
    StatePoint evalQuintic(const double t) const;
};
