#include "robo_base.hpp"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>

Coordination RoBoMotion::forword_kinematics_pulseToUnit(const std::vector<int32_t> &pulse) const {
    Coordination coordination(toInt(AxisIndex::COUNT));
    Pulse repulse = motorinfo.pulseToMotionMap(pulse);
    // std::cout << "forword_kinematics_repulse:" << std::endl;
    // for(auto element : coordination)
    //     std::cout << "  " << element;
    // std::cout << std::endl;

    std::vector<double> unit = motorinfo.repulseToUnit(repulse);

    coordination[0] = L1 * cos(unit[0] - PI / 2) + L2 * cos(unit[0] - PI / 2 + unit[1]);

    coordination[1] = L1 * sin(unit[0] + PI / 2) + L2 * sin(unit[0] + PI / 2 + unit[1]);

    coordination[2] = LZ * unit[2] / (2 * PI) + unit[3];
    coordination[3] = unit[2] / PI * 180;

    std::cout << "forword_kinematics_Coordination:" << std::endl;
    for (auto element : coordination)
        std::cout << "  " << element;
    std::cout << std::endl;
    return coordination;
}

Pulse RoBoMotion::inverse_kinematics_unitToPulse(const Coordination &coodination) {
    /**
     * @brief 单位值
     * @param  unit[0] XR
     * @param  unit[1] YR
     * @param  unit[2] ZU
     * @param  unit[3] ZR
     */
    std::vector<double> unit(coodination.size());

    double y_cos_axis = (pow(coodination[0], 2) + pow(coodination[1], 2) - pow(L1, 2) - pow(L2, 2)) / (2 * L1 * L2);

    // clamp + reachability check
    if (y_cos_axis > 1.0 || y_cos_axis < -1.0) {
        Pulse empty;
        return empty; // 不可达：返回空
    }
    y_cos_axis = std::clamp(y_cos_axis, -1.0, 1.0);
    double y_sin_axis = sqrt(1 - pow(y_cos_axis, 2));
    unit[1] = atan2(y_sin_axis, y_cos_axis);

    double val_in1 = L1 + L2 * y_cos_axis;
    double val_in2 = L2 * y_sin_axis;
    unit[0] = atan2(val_in1 * coodination[0] - val_in2 * coodination[1],
                    val_in1 * coodination[1] + val_in2 * coodination[0]);

    unit[2] = coodination[3] / 180.0 * PI;
    unit[3] = coodination[2] - LZ * unit[2] / (2 * PI);

    auto repulse = motorinfo.unitTorePulse(unit);

    auto pulse = motorinfo.motionmapToPulse(repulse);

    // 判断是否超限
    std::vector<int8_t> limit = motorinfo.compareLimit(pulse);
    bool allWthinLimit = std::all_of(limit.begin(), limit.end(),
                                     [](int8_t x) { return x == 0; });
    if (!allWthinLimit)
        pulse.clear();
    return pulse;
}

Pulse RoBoMotion::inverse_kinematics_unitToPulse_csp(const Coordination &coodination) {
    /**
     * @brief 单位值
     * @param  unit[0] XR
     * @param  unit[1] YR
     * @param  unit[2] ZU
     * @param  unit[3] ZR
     */
    std::vector<double> unit(coodination.size());
    bool input_changed = std::any_of(trajectoryplanners.begin(), trajectoryplanners.end(),
                                     [](const TrajectoryPlanner &tp) { return tp.point_changed != false; });
    if (input_changed) {
        double y_cos_axis = (pow(coodination[0], 2) + pow(coodination[1], 2) - pow(L1, 2) - pow(L2, 2)) / (2 * L1 * L2);
        double y_sin_axis = sqrt(1 - pow(y_cos_axis, 2));
        unit[1] = atan2(y_sin_axis, y_cos_axis);

        double val_in1 = L1 + L2 * y_cos_axis;
        double val_in2 = L2 * y_sin_axis;
        unit[0] = atan2(val_in1 * coodination[0] - val_in2 * coodination[1],
                        val_in1 * coodination[1] + val_in2 * coodination[0]);

        unit[2] = coodination[3] / 180.0 * PI;
        unit[3] = coodination[2] - LZ * unit[2] / (2 * PI);

        for (size_t i = 0; i < unit.size(); i++)
            trajectoryplanners[i].setTarget(unit[i]);
        std::fill(sampleFreqs.begin(), sampleFreqs.end(), 0);
        for (size_t i = 0; i < unit.size(); i++) {
            statepoints[i] = trajectoryplanners[i].sample(sampleFreqs[i]);
            sampleFreqs[i]++;
            unit[i] = statepoints[i].pos;
        }
    } else {
        for (size_t i = 0; i < unit.size(); i++) {
            statepoints[i] = trajectoryplanners[i].sample(sampleFreqs[i]);
            sampleFreqs[i]++;
            unit[i] = statepoints[i].pos;
        }
    }

    auto repulse = motorinfo.unitTorePulse(unit);

    auto pulse = motorinfo.motionmapToPulse(repulse);

    // 判断是否超限
    std::vector<int8_t> limit = motorinfo.compareLimit(pulse);
    bool allWthinLimit = std::all_of(limit.begin(), limit.end(),
                                     [](int8_t x) { return x == 0; });
    if (!allWthinLimit)
        pulse.clear();
    return pulse;
}
// Pulse RoBoMotion::inverse_kinematics_unitToPulse_csp(const Coordination &coodination, const Pulse &cur_p) const
// {
//     /**
//      * @brief 单位值
//      * @param  unit[0] XR
//      * @param  unit[1] YR
//      * @param  unit[2] ZU
//      * @param  unit[3] ZR
//      */
//     std::vector<double>  unit(coodination.size());

//     double y_cos_axis = (pow(coodination[0], 2) + pow(coodination[1], 2) - pow(L1_LENGTH, 2) - pow(L2_LENGTH, 2)) / (2 * L1_LENGTH * L2_LENGTH);
//     double y_sin_axis = sqrt(1 - pow(y_cos_axis, 2));
//     unit[1] = atan2(y_sin_axis, y_cos_axis);

//     double val_in1 = L1_LENGTH + L2_LENGTH * y_cos_axis;
//     double val_in2 = L2_LENGTH * y_sin_axis;
//     unit[0] = atan2(val_in1 * coodination[1] - val_in2 * coodination[0],
//                     val_in1 * coodination[0] + val_in2 * coodination[1]);

//     unit[2] = coodination[3] / 180.0 * PI;
//     unit[3] = (coodination[2] - unit[3]) / LZ;

//     auto repulse = motorinfo.unitToPulse(unit);
//     // for (size_t i = 0; i < repulse.size(); i++)
//     // {
//     //     if (i == toInt(AxisIndex::XP) || i == toInt(AxisIndex::YP) || i == toInt(AxisIndex::ZP))
//     //         repulse[i] = std::clamp(repulse[i], -AXIS_P_CSP, AXIS_P_CSP);
//     //     else
//     //         repulse[i] = std::clamp(repulse[i], -AXIS_R_CSP, AXIS_R_CSP);
//     // }

//     auto pulse   = motorinfo.motionmapToPulse(repulse);
//     //判断是否超限
//     auto limit = motorinfo.compareLimit(pulse);
//     bool allWthinLimit = std::all_of(limit.begin(), limit.end(),
//                                         [](int8_t x){return x == 0;});
//     if (!allWthinLimit) pulse.clear();
//     else
//         for (size_t i = 0; i < repulse.size(); i++)
//         {
//             if (i == toInt(AxisIndex::ZP))
//                 pulse[i] = std::clamp(pulse[i], cur_p[i]-AXIS_P_CSP, cur_p[i] + AXIS_P_CSP);
//             else
//                 pulse[i] = std::clamp(pulse[i], cur_p[i]-AXIS_R_CSP, cur_p[i] + AXIS_R_CSP);
//         }
//     return pulse;
// }

std::vector<int8_t> RoBoMotion::limitCompare(const Pulse &pulses) const {
    auto out = motorinfo.compareLimit(pulses);
    return out;
}
