// file: TrajectoryPlanner.cpp
#include "TrajectoryPlanner.hpp"
#include <cassert>
#include <cmath>
#include <algorithm>
#include <iostream>

#define p(a,b) std::pow(a,b)


void TrajectoryPlanner::setStartState(const double p_target, const double v_target, const double a_target) {
    double v_set = std::clamp(v_target, -Vmax, Vmax);
    double a_set = std::clamp(a_target, -Amax, Amax);
    startp_ = {p_target, v_set, a_set};

    point_changed = true;
    T_ = 0.0;
    Freq_ = 0;
}



void TrajectoryPlanner::setStartState(const StatePoint &start) {
    startp_ = {start.pos, 
               std::clamp(start.vel, -Vmax, Vmax), 
               std::clamp(start.acc, -Amax, Amax)};
    point_changed = true;
    T_ = 0.0;
    Freq_ = 0;
}

void TrajectoryPlanner::setTarget(const double p_target, const double v_target, const double a_target) {
    double v_set = std::clamp(v_target, -Vmax, Vmax);
    double a_set = std::clamp(a_target, -Amax, Amax);
    targetp_ = {p_target, v_set, a_set};

    computeSCurveTime();
    // 生成五次多项式
    computeQuintic();

    // StatePoint check = evalQuintic(T_);
    // std::cerr << "Check end pos=" << check.pos 
    //         << " target pos=" << targetp_.pos 
    //         << " error=" << (check.pos - targetp_.pos) << "\n";
    point_changed = false;
}

void TrajectoryPlanner::setTarget(const StatePoint &target) {

    targetp_ = {target.pos, 
               std::clamp(target.vel, -Vmax, Vmax), 
               std::clamp(target.acc, -Amax, Amax)};

    computeSCurveTime();

    // 生成五次多项式
    computeQuintic();
    // StatePoint check = evalQuintic(T_);
    // std::cerr << "Check end pos=" << check.pos 
    //         << " target pos=" << targetp_.pos 
    //         << " error=" << (check.pos - targetp_.pos) << "\n";
    point_changed = false;
}

double TrajectoryPlanner::getTotalTime() const {
    return T_;
}

int    TrajectoryPlanner::getFrequency() const {
    return Freq_;
}

StatePoint TrajectoryPlanner::sample(double t) const {
    if (t <= 0.0) {
        return startp_;
    }
    if (t >= T_) {
        // return end state
        StatePoint end;
        end.pos = quinticCoff_[0]     + quinticCoff_[1] * T_ + quinticCoff_[2] * p(T_,2)
                + quinticCoff_[3]     * p(T_,3) + quinticCoff_[4] * p(T_,4)
                + quinticCoff_[5]     * p(T_,5);
        end.vel = quinticCoff_[1]     
                + 2 * quinticCoff_[2] * T_ 
                + 3 * quinticCoff_[3] * p(T_,2) 
                + 4 * quinticCoff_[4] * p(T_,3) 
                + 5 * quinticCoff_[5] * p(T_,4);
        end.acc = 2 * quinticCoff_[2] 
                + 6 * quinticCoff_[3] * T_ 
                + 12 * quinticCoff_[4] * p(T_,2)
                + 20 * quinticCoff_[5] * p(T_,3);
        return end;
    }
    return evalQuintic(t);
}

StatePoint TrajectoryPlanner::sample(int freq) const {
    if (freq <= 0.0) {
        return startp_;
    }
    if (freq >= Freq_) {
        // return end state
        StatePoint end;
        end.pos = quinticCoff_[0]     + quinticCoff_[1] * T_ + quinticCoff_[2] * p(T_,2)
                + quinticCoff_[3]     * p(T_,3) + quinticCoff_[4] * p(T_,4)
                + quinticCoff_[5]     * p(T_,5);
        end.vel = quinticCoff_[1]     
                + 2 * quinticCoff_[2] * T_ 
                + 3 * quinticCoff_[3] * p(T_,2) 
                + 4 * quinticCoff_[4] * p(T_,3) 
                + 5 * quinticCoff_[5] * p(T_,4);
        end.acc = 2 * quinticCoff_[2] 
                + 6 * quinticCoff_[3] * T_ 
                + 12 * quinticCoff_[4] * p(T_,2)
                + 20 * quinticCoff_[5] * p(T_,3);
                // std::cout << "target pos = " << targetp_.pos << std::endl;
                // std::cout << "sample end pos = " << end.pos << "   ";
                // for (auto & i : quinticCoff_)
                //     std::cout << i << "    ";
                // std::cout <<std::endl;
                return end;
    }
    return evalQuintic(static_cast<double>(freq) / FREQUENCY);
}

void TrajectoryPlanner::computeSCurveTime() {

    double delta_p = std::abs(targetp_.pos - startp_.pos);
    direction_ = ( (targetp_.pos - startp_.pos) >= 0 ? 1.0 : -1.0 );
    double v0 = startp_.vel; 
    double vf = targetp_.vel;
    double vmax = direction_ * Vmax;

//！可以先假设到达vmax
    double t1{};
    double s1{};
    if (v0 < vmax) // 开始加速
    {
        t1 = (vmax - v0) / Amax;
        s1 = v0 * t1 + 0.5 * Amax * t1 * t1;  //开始段位移
    }
    else // 开始减速
    {
        t1 = (v0 - vmax) / Amax;
        s1 = v0 * t1 - 0.5 * Amax * t1 * t1;  //开始段位移
    }

    double t3{};
    double s3{};
    if (vf < vmax) // 结束减速
    {
        t3 = (vmax - vf) / Amax;
        s3 = vmax * t3 - 0.5 * Amax * t3 * t3;
    }
    else // 结束加速
    {
        t3 = (vf - vmax) / Amax;
        s3 = vmax * t3 + 0.5 * Amax * t3 * t3;
    }

    if (std::abs(s1 + s3) < delta_p) // 若移动的距离不够则一定具有max匀速段
    {
        double s2 = delta_p * direction_ - (s1 + s3);
        double t2 = s2 / vmax;
        t_accel_ = t1;
        t_cruise_ = t2;
        t_decel_ = t3;
    } else {        // 无max匀速段，计算可达的峰值速度
        double v_peak = std::sqrt( (2 * Amax * delta_p + v0 * v0 + vf * vf) / 2.0 );
        if ( v_peak < v0) {
            v_peak = v0;
        }
        double t1p = std::max(0.0, (v_peak - v0) /  Amax);
        double t3p = std::max(0.0, std::abs(v_peak - vf) / Amax);
        t_accel_ = t1p * 2;
        t_cruise_ = 0.0;
        t_decel_ = t3p * 2;
    }
    std::cout << "t_accel=" << t_accel_ << " t_cruise=" << t_cruise_ << " t_decel=" << t_decel_ << "\n";
    if (std::isnan(t_accel_) || std::isnan(t_cruise_) || std::isnan(t_decel_))
    {
        std::cerr << "TrajectoryPlanner::computeSCurveTime() error: t_accel_ or t_cruise_ or t_decel_ is nan.\n";
        T_ = 10000.0; //延长T_以供反应时间
        Freq_ = std::ceil(T_ * FREQUENCY) + 1;
        return;
    }
    T_ = t_accel_ + t_cruise_ + t_decel_;

    Freq_ = std::ceil(T_ * FREQUENCY) + 1;
}

void TrajectoryPlanner::computeQuintic() {
    // 解五次多项式系数（按经典公式）
    // QuinticCoeff c;

    quinticCoff_[0] = startp_.pos;
    quinticCoff_[1] = startp_.vel;
    quinticCoff_[2] = startp_.acc / 2.0;

    double T2 = T_ * T_;
    double T3 = T2 * T_;
    double T4 = T3 * T_;
    double T5 = T4 * T_;

    // 辅助：
    double Dp = targetp_.pos - (startp_.pos + startp_.vel * T_ + 0.5 * startp_.acc * T2);
    double Dv = targetp_.vel - (startp_.vel + startp_.acc * T_);
    double Da = targetp_.acc -  startp_.acc;

    if (T_ != 0)
    {
        quinticCoff_[3] = (  10 * Dp -   4 * Dv * T_ + 0.5 * Da * T2 ) / T3;
        quinticCoff_[4] = ( -15 * Dp +   7 * Dv * T_ -       Da * T2 ) / T4;
        quinticCoff_[5] = (   6 * Dp -   3 * Dv * T_ + 0.5 * Da * T2 ) / T5;
    }
    else
    {
        quinticCoff_[3] = 0;
        quinticCoff_[4] = 0;
        quinticCoff_[5] = 0;
    }

    // coeff_ =  c;
}

StatePoint TrajectoryPlanner::evalQuintic(const double t) const {
    StatePoint s;
    s.pos = quinticCoff_[0] + quinticCoff_[1] * t 
          + quinticCoff_[2] * p(t, 2) + quinticCoff_[3] * p(t, 3) 
          + quinticCoff_[4] * p(t, 4) + quinticCoff_[5] * p(t, 5);
    s.vel = quinticCoff_[1] + 2 * quinticCoff_[2] * t 
          + 3 * quinticCoff_[3] * p(t, 2) + 4 * quinticCoff_[4] * p(t, 3) 
          + 5 * quinticCoff_[5] * p(t, 4);
    s.acc = 2 * quinticCoff_[2] + 6 * quinticCoff_[3] * t 
          + 12 * quinticCoff_[4] * p(t, 2) + 20 * quinticCoff_[5] * p(t, 3);
    return s;
}

