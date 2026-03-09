#pragma once

#include "TrajectoryPlanner.hpp"
#include <array>
#include <cstdint>
#include <type_traits>
#include <vector>

inline constexpr double PI = 3.1415926535; //   π

///! 坐标结构体
/**
 * @brief 坐标结构体
 * @param[in] x 坐标x  初始为90
 * @param[in] y 坐标y  初始为0
 * @param[in] z 坐标z
 * @param[in] r 坐标r
 */
typedef std::vector<double> Coordination;

/**
 * @brief 轴脉冲结构体
 * @param[in] xr 脉冲x
 * @param[in] yr 脉冲y
 * @param[in] zu 脉冲z
 * @param[in] zp 脉冲r
 */
typedef std::vector<int32_t> Pulse;

/**
 * @brief 轴速度单位值
 * @param[in] xr x
 * @param[in] yr y
 * @param[in] zu z
 * @param[in] zp r
 */
typedef std::vector<double> Velocity;

/**
 * @brief 轴速度脉冲值
 * @param[in] xr x
 * @param[in] yr y
 * @param[in] zu z
 * @param[in] zp r
 */
typedef std::vector<int32_t> VelPulse;

/// @brief 限制轴调用
enum class AxisIndex : int {
    RX = 0,
    RY,
    ZU,
    ZP,
    COUNT
};

constexpr int toInt(AxisIndex a) noexcept {
    return static_cast<std::underlying_type_t<AxisIndex>>(a);
}

/////////////////////////////////// 统一单位为mm 弧度///////////////////////////////////
#define SCREW_LEAD 16.0 // mm

// 轴分辨率定义，单位mm/脉冲
#define RX_POSITION_RES ((360 / ((67108864.0 / (8388608.0 / 5625.0)) * 80.0)) * PI / 180.0)                           // 0.0001度 单脉冲
#define RY_POSITION_RES ((360 / ((67108864.0 / (1048576.0 / 1125.0)) * 50.0)) * PI / 180.0)                           // 0.0001度 单脉冲
#define ZU_POSITION_RES ((360 / ((67108864.0 / (6291456.0 / 48125.0)) * (60.0 / 22.0) * (72.0 / 28.0))) * PI / 180.0) // 0.0001度 单脉冲
#define ZP_POSITION_RES (SCREW_LEAD / ((67108864.0 / (1572864.0 / 3125.0)) * (48.0 / 40.0)))                          // 0.0001mm 单脉冲

#define ACC_LIMIT_FACTOR 0.15

#define AXIS_R_CSP 2000
#define AXIS_P_CSP 3000

#define L1_LENGTH 349.5 // 杆1长度
#define L2_LENGTH 234.8 // 杆2长度
#define LZ_RES 0.0      // 单位设定，coordination输入量转motionmap
/////////////////////////////////// 编码特值///////////////////////////////////
enum HomePostion : int32_t {
    RX_HOME_POSITION = -440000,
    RY_HOME_POSITION = 920000,
    ZU_HOME_POSITION = 2500000, // 2488978
    ZP_HOME_POSITION = 0,
};

//! ZU ZP按理来说不存在极限位置，应该按实际位置进行限制
enum PostionMaxLimit : int32_t {
    RX_MAX_POSITION = -40000,  //-39643
    RY_MAX_POSITION = 1450000, // 1442733
    ZU_MAX_POSITION = 2147483647,
    ZP_MAX_POSITION = 2147483647,
};

enum PostionMinLimit : int32_t {
    RX_MIN_POSITION = -531460, //-531460
    RY_MIN_POSITION = 600000,  // 578574
    ZU_MIN_POSITION = -2147483647,
    ZP_MIN_POSITION = -2147483647,
};

enum ZeroPosition : int32_t {
    RX_ZERO_POSITION = -440000, //-443079
    RY_ZERO_POSITION = 920000,
    ZU_ZERO_POSITION = 2500000,
    ZP_ZERO_POSITION = 0,
};

// 末端正转位置上升
enum Direction : int8_t {
    RX_Direction = 1,  // X轴正转X角增大
    RY_Direction = 1,  // y轴正转y角增大
    ZU_Direction = -1, // u轴正转末端下降 末端逆时针
    ZP_Direction = 1,  // z轴正转末端上升
};

enum MotorVelocity : int32_t {
    RX_VELOCITY = 500000,
    RY_VELOCITY = 500000,
    ZU_VELOCITY = 45000000,
    ZP_VELOCITY = 2000000, // 为实现旋转应使ZP速度为2/45 ZU
};

struct MotorInfo {

    std::array<double, toInt(AxisIndex::COUNT)> positionRes{};
    std::array<int32_t, toInt(AxisIndex::COUNT)> homePos{};
    std::array<int32_t, toInt(AxisIndex::COUNT)> maxLimit{};
    std::array<int32_t, toInt(AxisIndex::COUNT)> minLimit{};
    std::array<int32_t, toInt(AxisIndex::COUNT)> zeroPos{};
    std::array<int8_t, toInt(AxisIndex::COUNT)> direction{};
    std::array<int32_t, toInt(AxisIndex::COUNT)> velocity{};

    // constexpr 构造 编译期初始化
    constexpr MotorInfo()
        : positionRes{RX_POSITION_RES, RY_POSITION_RES, ZU_POSITION_RES, ZP_POSITION_RES}, homePos{RX_HOME_POSITION, RY_HOME_POSITION, ZU_HOME_POSITION, ZP_HOME_POSITION}, maxLimit{RX_MAX_POSITION, RY_MAX_POSITION, ZU_MAX_POSITION, ZP_MAX_POSITION}, minLimit{RX_MIN_POSITION, RY_MIN_POSITION, ZU_MIN_POSITION, ZP_MIN_POSITION}, zeroPos{RX_ZERO_POSITION, RY_ZERO_POSITION, ZU_ZERO_POSITION, ZP_ZERO_POSITION}, direction{RX_Direction, RY_Direction, ZU_Direction, ZP_Direction}, velocity{RX_VELOCITY, RY_VELOCITY, ZU_VELOCITY, ZP_VELOCITY} {}

    ~MotorInfo() = default;

    // 访问函数 编译期求值
    constexpr double res(AxisIndex ax) const { return positionRes[toInt(ax)]; }
    constexpr int32_t home(AxisIndex ax) const { return homePos[toInt(ax)]; }
    constexpr int32_t min(AxisIndex ax) const { return minLimit[toInt(ax)]; }
    constexpr int32_t max(AxisIndex ax) const { return maxLimit[toInt(ax)]; }
    constexpr int32_t zero(AxisIndex ax) const { return zeroPos[toInt(ax)]; }
    constexpr int8_t dir(AxisIndex ax) const { return direction[toInt(ax)]; }
    constexpr int32_t vel(AxisIndex ax) const { return velocity[toInt(ax)]; }

    /**
     * @brief 将实际脉冲转为运动视图
     *
     * @param pulse
     * @return Pulse
     */
    Pulse pulseToMotionMap(const Pulse &pulse) const {
        Pulse repulse(pulse.size());
        for (size_t i = 0; i < pulse.size(); i++)
            repulse[i] = static_cast<int32_t>(direction[i]) * (pulse[i] - zeroPos[i]);
        return repulse;
    }

    /**
     * @brief 将运动视图转为实际脉冲
     *
     * @param repulse
     * @return Pulse&
     */
    Pulse motionmapToPulse(const Pulse &repulse) const {
        Pulse pulse(repulse.size());
        for (size_t i = 0; i < repulse.size(); i++)
            pulse[i] = repulse[i] * static_cast<int32_t>(direction[i]) + zeroPos[i];
        return pulse;
    }

    /**
     * @brief 将直接脉冲转化为motionmap的unit
     *
     * @param ax
     * @param pulses
     * @return unit
     */
    constexpr double pulseToUnit(AxisIndex ax, int32_t pulses) const {
        return (pulses - zero(ax)) * dir(ax) * res(ax);
    }

    /**
     * @brief 将直接脉冲转化为motionmap的unit
     *
     * @param pulses
     * @return unit
     */
    std::vector<double> pulsesToUnit(const Pulse &pulses) const {
        std::vector<double> out(pulses.size());
        for (size_t i = 0; (i < pulses.size()) && (i < positionRes.size()); i++)
            out[i] = static_cast<double>((pulses[i] - zeroPos[i]) * direction[i]) * positionRes[i];
        return out;
    }

    /**
     * @brief 单次转换相对脉冲
     *
     * @param ax
     * @param  unit
     * @return constexpr int32_t
     */
    constexpr int32_t unitTorePulse(AxisIndex ax, double unit) const {
        return static_cast<int32_t>(unit / res(ax));
    }

    Pulse unitTorePulse(const std::vector<double> &unit) const {
        Pulse repulse(unit.size());
        for (size_t i = 0; (i < unit.size()) && (i < positionRes.size()); i++)
            repulse[i] = static_cast<int32_t>(unit[i] / positionRes[i]);
        return repulse;
    }

    /**
     * @brief 相对脉冲转单位值
     *
     * @param ax
     * @param pulses
     * @return double
     */
    constexpr double repulseToUnit(AxisIndex ax, int32_t pulses) const {
        return static_cast<double>(pulses) * res(ax);
    }

    std::vector<double> repulseToUnit(const std::vector<int32_t> &pulses) const {
        std::vector<double> out(pulses.size());
        for (size_t i = 0; (i < pulses.size()) && (i < positionRes.size()); i++)
            out[i] = static_cast<double>(pulses[i]) * positionRes[i];
        return out;
    }

    /**
     * @brief 单位转绝对脉冲
     *
     * @param ax
     * @param  unit
     * @return constexpr int32_t
     */
    constexpr int32_t unitToPulse(AxisIndex ax, double unit) const {
        return static_cast<int32_t>(unit / res(ax) * dir(ax)) + zero(ax);
    }

    Pulse unitToPulse(const std::vector<double> &unit) const {
        Pulse pulse(unit.size());
        for (size_t i = 0; (i < unit.size()) && (i < positionRes.size()); i++)
            pulse[i] = static_cast<int32_t>(unit[i] / positionRes[i]) * direction[i] + zeroPos[i];
        return pulse;
    }

    /**
     * @brief 单次比较
     *
     * @param ax
     * @param position
     * @return true
     * @return false
     */
    constexpr bool compareLimit(AxisIndex ax, int32_t position) const {
        return position >= min(ax) && position <= max(ax);
    }

    /**
     * @brief 比较限位
     *
     * @param pulses[0] YP
     * @param pulses[1] XP
     * @param pulses[2] RZ  α
     * @param pulses[3] ZP
     * @param pulses[4] RX  β
     * @param pulses[5] RY  γ
     * @return std::vector<int8_t>
     */
    std::vector<int8_t> compareLimit(const std::vector<int32_t> &pulses) const {
        std::vector<int8_t> out(pulses.size());
        for (size_t i = 0; (i < pulses.size()) && (i < positionRes.size()); i++)
            static_cast<double>(pulses[i])<minLimit[i] ? out[i] = -1 : static_cast<double>(pulses[i])> maxLimit[i] ? out[i] = 1 : out[i] = 0;
        // std::cout << "compareLimit:" << std::endl;
        // for(auto element : out)
        //     std::cout << "  " << static_cast<int>(element);
        // std::cout << std::endl;
        return out;
    }

    /**
     * @brief 将脉冲值转为速度单位值
     *
     * @param vel_pulse
     * @return Velocity
     */
    Velocity velToUnitVel(const VelPulse &vel_pulse) const {
        Velocity unit(vel_pulse.size());
        for (size_t i = 0; (i < vel_pulse.size()) && (i < positionRes.size()); i++)
            unit[i] = static_cast<double>(vel_pulse[i]) * positionRes[i] * dir(static_cast<AxisIndex>(i));
        return unit;
    }

    /**
     * @brief 将速度单位值转为脉冲值
     *
     * @param vel_unit
     * @return VelPulse
     */
    VelPulse unitToVelPulse(const Velocity &vel_unit) const {
        VelPulse vel_pulse(vel_unit.size());
        for (size_t i = 0; (i < vel_unit.size()) && (i < positionRes.size()); i++)
            vel_pulse[i] = static_cast<int32_t>(vel_unit[i] / positionRes[i]) * dir(static_cast<AxisIndex>(i));
        return vel_pulse;
    }
};

static_assert(toInt(AxisIndex::COUNT) == 4, "axis count changed");

class RoBoMotion {
public:
    RoBoMotion()
        // :motorinfo(), L1(349.5), L2(234.8), L3(60.0)
        : motorinfo(), statepoints(toInt(AxisIndex::COUNT)), sampleFreqs(toInt(AxisIndex::COUNT)),
          L1(L1_LENGTH), L2(L2_LENGTH), LZ(LZ_RES) {
        trajectoryplanners.resize(toInt(AxisIndex::COUNT));
        for (int i = 0; i < toInt(AxisIndex::COUNT); ++i) {
            double Vmax = static_cast<double>(motorinfo.vel(static_cast<AxisIndex>(i))) * motorinfo.res(static_cast<AxisIndex>(i)); // 最大速度 单位为对应位置/s
            double Amax = Vmax * ACC_LIMIT_FACTOR;
            double Jmax = Amax * 0.25;
            trajectoryplanners[i] = TrajectoryPlanner(Vmax, Amax, Jmax);
        }
    }

    ~RoBoMotion() = default;

    struct RotationAngle {
        RotationAngle() = default;
        RotationAngle(double angle) {
            u_p = angle;
            z_p = -angle / 360.0 * SCREW_LEAD; // 末端顺时针会上升，z轴旋转拟合
        }
        double u_p{}; // U轴旋转使末端转动的角度
        double z_p{}; // Z轴旋转使末端移动的距离
    };

    struct ScrewMotion {
        ScrewMotion() = default;
        /**
         * @brief 构造一个新的 Screw Motion 对象
         *
         * @param lead 导程 包含方向
         * @param distance 移动距离 固定向下
         */
        ScrewMotion(double lead, double distance) {
            u_p = std::abs(distance) / std::abs(lead) * 360.0; // lead为导程包含方向 运算得到旋转角度
            z_p = -std::abs(distance) - std::abs(distance) / lead * SCREW_LEAD;
        }
        double u_p{}; // U轴旋转使末端转动的角度
        double z_p{}; // Z轴旋转使末端移动的距离
    };
    MotorInfo motorinfo;
    std::vector<StatePoint> statepoints;
    std::vector<int32_t> sampleFreqs;
    std::vector<TrajectoryPlanner> trajectoryplanners;

    void init_trajectoryplanners(StatePoint &sp, size_t i) {
        trajectoryplanners[i].setStartState(sp);
        sampleFreqs[i] = 0;
        // std::fill(sampleFreqs.begin(), sampleFreqs.end(), 0);
    }
    /**
     * @brief 正运动学 脉冲转运动视图
     *
     * @param  pulse[0] x
     * @param  pulse[1] y
     * @param  pulse[2] r
     * @param  pulse[3] z
     */
    Coordination forword_kinematics_pulseToUnit(const std::vector<int32_t> &pulse) const;

    /**
     * @brief 逆运动学 单位值转脉冲
     *
     * @param coodination[0] x
     * @param coodination[1] y
     * @param coodination[2] z
     * @param coodination[3] r
     */
    Pulse inverse_kinematics_unitToPulse(const Coordination &coodination);

    Pulse inverse_kinematics_unitToPulse_csp(const Coordination &coodination);

    Pulse inverse_kinematics_unitToPulse_csp(const Coordination &coodination, const Pulse &cur_p) const;

    std::vector<int8_t> limitCompare(const Pulse &pulses) const;

    // 获取L1,L2,LZ 的接口
    double getL1() const { return L1; }
    double getL2() const { return L2; }
    double getLZ() const { return LZ; }

private:
    double L1;
    double L2;
    double LZ;
};

inline RoBoMotion robomotion;