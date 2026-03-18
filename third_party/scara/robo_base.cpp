#include "robo_base.hpp"
#include <cmath>
#include <algorithm> 
#include <cstddef>
#include <iostream>
#include <Eigen/Dense>
#include <iomanip> 

//！！需要启用O3优化
// 归一化角度到 [-180, 180] 范围
static double normalizeAngle(double angle_deg) {
    while (angle_deg > 180.0) 
        angle_deg -= 360.0;
    while (angle_deg <= -180.0) 
        angle_deg += 360.0;
    return angle_deg;
}
// 计算单链接的变换矩阵
static Eigen::Matrix4d get_transform(const RoBoMotion::MDH_Link& link, double q_val) {
    double alpha = link.alpha;
    double a = link.a;
    double d, theta;

    if (link.is_prismatic) {
        d = q_val + link.d_static;
        theta = link.theta_static;
    }
    else {
        d = link.d_static;
        theta =  link.theta_static + q_val;
    }

    double ct = cos(theta);
    double st = sin(theta);
    double ca = cos(alpha);
    double sa = sin(alpha);

    Eigen::Matrix4d T;
    T <<     ct,    -st,   0,       a,
         st* ca, ct* ca, -sa, -d * sa,
         st* sa, ct* sa,  ca,  d * ca,
              0,      0,   0,       1;
    return T;
}

// 数值差分法计算雅可比矩阵
static Eigen::Matrix4d compute_jacobian(const std::vector<double>& unit) {
    Eigen::Matrix4d J;
    double delta = 1e-4; // 差分步长
    auto coordination = RoBoMotion::forword_kinematics(unit);
    // 计算当前位置
    Eigen::Vector4d p_curr = Eigen::Map<Eigen::Vector4d>(coordination.data());

    for (int i = 0; i < 4; i++) {
        std::vector<double> unit_new(unit);
        unit_new[i] += delta; 

        auto coordination_new = RoBoMotion::forword_kinematics(unit_new);
        Eigen::Vector4d p_new = Eigen::Map<Eigen::Vector4d>(coordination_new.data());

        Eigen::Vector4d diff = p_new - p_curr;
        // 角度跳变处理
        if (diff[3] > 180.0) diff[3] -= 360.0;
        if (diff[3] < -180.0) diff[3] += 360.0;

        J.col(i) = diff / delta;
    }
    return J;
}

/**
 * @brief DLS 求解器，适用于 SCARA 逆运动学
 * @cite 解决了奇异点精度差的问题
 * @param target_pose 目标位姿 (x, y, z, yaw)，单位 mm 和弧度
 * @param u_guess 初始关节猜测值 (u1, u2, u3, u4)，弧度
 * @return std::vector<double> 关节角度解 (u1, u2, u3, u4)，弧度
 */
std::vector<double> RoBoMotion::solve_dls_high_precision(const Coordination& target_pose, const std::vector<double>& u_guess) {
    std::vector<double> u_curr = u_guess;
    auto target_coor = target_pose;
    auto target_pose_eigen = Eigen::Map<Eigen::Vector4d>(target_coor.data());
    int max_iter = 200;         // 增加迭代次数上限
    double tol_pos = 1e-4;      // 位置公差 0.0001 mm
    double tol_ang = 1e-3;      // 角度公差 0.001 度

    for (int k = 0; k < max_iter; k++) {
        // 1. 计算误差
        Eigen::Vector4d p_curr = Eigen::Map<Eigen::Vector4d>(forword_kinematics(u_curr).data());        
        Eigen::Vector4d error = target_pose_eigen - p_curr;
        error[3] = normalizeAngle(error[3]);

        // 2. 检查收敛
        if (error.head(3).norm() < tol_pos && abs(error[3]) < tol_ang) {
            // 达到精度要求，返回
            return u_curr;
        }

        // 3. 计算雅可比
        Eigen::Matrix4d J = compute_jacobian(u_curr);

        // 4. 动态设置阻尼因子 (Lambda)
        // 之前设为0.5导致精度卡在0.17mm。
        // 策略：误差越大，阻尼越大(稳)；误差越小，阻尼越小(准)。
        double err_norm = error.norm();
        double lambda;

        if (err_norm > 10.0) {
            lambda = 0.1;   // 离得远，稳一点
        }
        else {
            lambda = 0.002; // 离得近/奇异点，阻尼极小，允许大幅修正
        }

        // 5. DLS 公式: dq = (J'J + lambda^2*I)^-1 * J'e
        Eigen::Matrix4d Jt = J.transpose();
        Eigen::Matrix4d I = Eigen::Matrix4d::Identity();

        Eigen::Matrix4d A = Jt * J + (lambda * lambda) * I;
        Eigen::Vector4d g = Jt * error;

        // 使用 LDLT 分解求解线性方程 (比直接求逆更稳健)
        Eigen::Vector4d dq = A.ldlt().solve(g);

        // 6. 步长限制 (Clamp)
        // 防止 Lambda 很小时关节瞬间跳变过大
        double max_step = 5.0; // 最大允许动5度/5mm
        if (dq.norm() > max_step) {
            dq = dq.normalized() * max_step;
        }

        // 7. 更新关节
        Eigen::Map<Eigen::Vector4d> u_curr_ev(u_curr.data());
        u_curr_ev += dq;
    }

    return u_curr;
}


Coordination RoBoMotion::forword_kinematics_pulseToUnit(const std::vector<int32_t> &pulse)
{
    Coordination coordination(toInt(AxisIndex::COUNT));
    Pulse  repulse = motorinfo.pulseToMotionMap(pulse);
    // std::cout << "forword_kinematics_repulse:" << std::endl;
    // for(auto element : coordination)
    //     std::cout << "  " << element;
    // std::cout << std::endl;

    std::vector<double> unit = motorinfo.repulseToUnit(repulse);
    std::vector<double> eunit = {unit[0], unit[1], unit[2], LZ * unit[2] / (2 * PI) + unit[3]};
    std::cout << "forword_kinematics_unit:" << std::endl;
    for(auto element : eunit)
        std::cout << "  " << element;
    std::cout << std::endl;
    // Eigen::Vector4d unit_ev = Eigen::Map<Eigen::Vector4d>(unit.data());
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    for (int i = 0; i < 4; i++) {
        T = T * get_transform(links[i], eunit[i]);
    }
    coordination[0] =  T(0, 3);
 
    coordination[1] =  T(1, 3);

    coordination[2] =  T(2, 3);

    coordination[3] =  atan2(T(1, 0), T(0, 0)); 

    std::cout << "forword_kinematics_Coordination:" << std::endl;
    for(auto element : coordination)
        std::cout << "  " << element;
    std::cout << std::endl;
    return coordination;
}

Coordination RoBoMotion::forword_kinematics(const std::vector<double> &unit) {
    std::vector<double> coordination(toInt(AxisIndex::COUNT));
    // std::vector<double> unit_t = unit;
    // Eigen::Vector4d unit_ev = Eigen::Map<Eigen::Vector4d>(unit_t.data());

    std::vector<double> eunit = {unit[0], unit[1], unit[2], LZ * unit[2] / (2 * PI) + unit[3]};
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    for (int i = 0; i < 4; i++) {
        T = T * get_transform(links[i], eunit[i]);
    }
    coordination[0] =  T(0, 3);
 
    coordination[1] =  T(1, 3);

    coordination[2] =  T(2, 3);

    coordination[3] =  atan2(T(1, 0), T(0, 0)); 

    return coordination;
}

Pulse RoBoMotion::inverse_kinematics_unitToPulse(const Coordination &coodination)
{
        // 1. 求关节2 (余弦定理)
    double r_sq = pow(coodination[0], 2) + pow(coodination[1], 2);
    double c2 = (r_sq - L1 * L1 - L2 * L2) / (2 * L1 * L2);

    // 边界保护 (防止NaN)
    if (c2 > 1.0) c2 = 1.0;
    if (c2 < -1.0) c2 = -1.0;

    double s2_pos = sqrt(1 - c2 * c2);
    double s2_neg = -sqrt(1 - c2 * c2);

    // 2. 求解两组可能的 theta1, theta2
    // 方案A
    double k1_a = L1 + L2 * c2;
    double k2_a = L2 * s2_pos;
    double theta1_a = (atan2(coodination[1], coodination[0]) - atan2(k2_a, k1_a));
    double theta2_a = atan2(s2_pos, c2);

    // 方案B
    double k1_b = L1 + L2 * c2;
    double k2_b = L2 * s2_neg;
    double theta1_b = (atan2(coodination[1], coodination[0]) - atan2(k2_b, k1_b));
    double theta2_b = atan2(s2_neg, c2);

    // 3. 策略：优先选择 theta1 为钝角/直角的解
    // 简单判断：通常绝对值较大的 theta1 对应张开的手臂
    double t1_final, t2_final;
    if (abs(theta1_a) > abs(theta1_b)) {
        t1_final = theta1_a;
        t2_final = theta2_a;
    }
    else {
        t1_final = theta1_b;
        t2_final = theta2_b;
    }

    // 4. 其余关节
    double theta3 = coodination[3] - t1_final - t2_final;
    double d4 = coodination[2]; // 假设Z轴直接对应

    /**
     * @brief 单位值
     * @param  unit[0] XR
     * @param  unit[1] YR
     * @param  unit[2] ZU
     * @param  unit[3] ZR
     */
    std::vector<double>  unit = {t1_final, t2_final, theta3, d4 - LZ * theta3 / (2 * PI)};
    std::cout << "inverse_kinematics_unit:" << std::endl;
    for(auto element : unit)
        std::cout << "  " << element;
    std::cout << std::endl;
    
    auto repulse = motorinfo.unitTorePulse(unit);

    auto pulse   = motorinfo.motionmapToPulse(repulse);

    //判断是否超限
    std::vector<int8_t> limit = motorinfo.compareLimit(pulse);
    bool allWthinLimit = std::all_of(limit.begin(), limit.end(), 
                                        [](int8_t x){return x == 0;});
    if (!allWthinLimit) pulse.clear();
    return pulse;
}

Pulse   RoBoMotion::inverse_kinematics_unitToPulse_csp(const Coordination &coodination)
{
    /**
     * @brief 单位值
     * @param  unit[0] XR
     * @param  unit[1] YR
     * @param  unit[2] ZU
     * @param  unit[3] ZR
     */
    std::vector<double>  unit(coodination.size());
    bool input_changed = std::any_of(trajectoryplanners.begin(), trajectoryplanners.end(),
                                    [](const TrajectoryPlanner &tp){ return tp.point_changed != false;});
    // std::cout << "input_changed:" << input_changed << std::endl;
    if (input_changed)
    {
        
        auto targetpulse_guess = inverse_kinematics_unitToPulse(coodination);

        std::vector<int8_t> limit = motorinfo.compareLimit(targetpulse_guess);
        bool allWthinLimit = std::all_of(limit.begin(), limit.end(), 
                                            [](int8_t x){return x == 0;});
        if (!allWthinLimit) 
        {
            targetpulse_guess.clear();
            return targetpulse_guess;
        }

        auto targetunit_guess = motorinfo.pulsesToUnit(targetpulse_guess);
        auto unit_solve = solve_dls_high_precision(coodination, targetunit_guess);
        for(size_t i = 0; i < unit_solve.size(); i++)
            trajectoryplanners[i].setTarget(unit_solve[i]);
        
        std::fill(sampleFreqs.begin(), sampleFreqs.end(), 0);
        for(size_t i = 0; i < unit.size(); i++)
        {
            statepoints[i] = trajectoryplanners[i].sample(sampleFreqs[i]);
            sampleFreqs[i]++;
            unit[i] = statepoints[i].pos;
        }
    }
    else
    {
        for(size_t i = 0; i < unit.size(); i++)
        {
            statepoints[i] = trajectoryplanners[i].sample(sampleFreqs[i]);
            sampleFreqs[i]++;
            unit[i] = statepoints[i].pos;
        }
    }

    auto repulse = motorinfo.unitTorePulse(unit);
    auto pulse   = motorinfo.motionmapToPulse(repulse);


    //判断是否超限
    std::vector<int8_t> limit = motorinfo.compareLimit(pulse);
    bool allWthinLimit = std::all_of(limit.begin(), limit.end(), 
                                        [](int8_t x){return x == 0;});
    if (!allWthinLimit) pulse.clear();
    return pulse;
}


std::vector<int8_t> RoBoMotion::limitCompare(const Pulse &pulses)  const
{
    auto out = motorinfo.compareLimit(pulses);
    return out;
}


