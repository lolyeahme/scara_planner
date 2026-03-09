#include "config.hpp"
#include "robo_base.hpp"
#include "scara_planner.hpp"

#include <fmt/format.h>

// ScaraPlanner类的构造函数：初始化 OPC UA 服务器、注册方法、创建发布者和定时器
ScaraPlanner::ScaraPlanner(std::string_view name, rm::OpcuaClient &exec)
    : Node(name), _srv(cfg::kCtrlPort, "SCARA LoadUnload Planner"), _exec(exec) {

    // 注册 OPC UA 方法
    installOpcuaMethods();

    // LPSS publisher：发布目标关节
    _tar_pub = this->createPublisher<rm::msg::JointState>(cfg::kTarJointsTopic);

    _pub_timer = this->createTimer(cfg::kPubPeriod, [this]() {
        if (!_running.load(std::memory_order_relaxed))
            return;

        switch (_task_mode) {
        case TaskMode::Idle:
            return;
        case TaskMode::BodyLoad:
            _q_cmd = sampleBLNextQ();
            break;
        case TaskMode::ThreadRingTighten:
            _q_cmd = sampleTRTNextQ();
            break;
        }

        publishQCmd(_q_cmd);
    });

    // OPC UA server：处理 OPC UA 请求
    // 当发布周期为10ms时，使用这种写法，线程不会卡住，
    _opcua_timer = this->createTimer(cfg::kOpcuaPumpPeriod, [this]() { 
        fmt::println("[planner] pump OPC UA server");
        _srv.spinOnce(); });
    // 但是当发布频率很高时，上述写法导致线程卡死，就要单独开一个线程
    // _opcua_thread = std::jthread([this](std::stop_token st) {
    //     while (!st.stop_requested()) {
    //         _srv.spinOnce();
    //         std::this_thread::sleep_for(cfg::kOpcuaPumpPeriod);
    //     }
    // });
} // 构造函数结束

void ScaraPlanner::installOpcuaMethods() {
    // Start：切 CSP + 开始发布
    rm::Method startBL_m = [this](const rm::NodeId &, const rm::Variables &) {
        // 1、如果当前有别的任务在执行，则拒绝启动body_load，并返回false
        if (_running.load(std::memory_order_relaxed)) {
            fmt::println("[planner] warning: another task is running, cannot start {}", cfg::kLuStartBrowse);
            return std::make_pair(false, rm::Variables{});
        }
        // 2、切 CSP 跟随模式，重置body_load轨迹状态，准备开始执行
        bool ok_follow = enableCspFollow();
        if (!ok_follow) {
            fmt::println("[planner] warning: enableCspFollow failed");
            return std::make_pair(false, rm::Variables{});
        }
        bool ok_traj = resetBLTraj();
        if (!ok_traj) {
            fmt::println("[planner] warning: resetBLTrajectory failed");
            return std::make_pair(false, rm::Variables{});
        }

        _task_mode = TaskMode::BodyLoad;

        bool ok = ok_follow && ok_traj;
        _running.store(ok, std::memory_order_relaxed);
        fmt::println("[planner] {}_start -> running={}", cfg::kLuStartBrowse, ok);
        return std::make_pair(ok, rm::Variables{});
    };
    startBL_m.browse_name = std::string(cfg::kLuStartBrowse);
    startBL_m.display_name = std::string(cfg::kLuStartDisp);
    startBL_m.description = std::string(cfg::kLuStartDesc);
    startBL_m.iargs = {};
    _srv.addMethodNode(startBL_m);

    rm::Method startTRT_m = [this](const rm::NodeId &, const rm::Variables &) {
        // 1、如果当前有别的任务在执行，则拒绝启动thread_ring_tighten，并返回false
        if (_running.load(std::memory_order_relaxed)) {
            fmt::println("[planner] warning: another task is running, cannot start {}", cfg::kTRTStartBrowse);
            return std::make_pair(false, rm::Variables{});
        }
        // 2、切 CSP 跟随模式，重置thread_ring_tighten轨迹状态，准备开始执行
        bool ok_follow = enableCspFollow();
        if (!ok_follow) {
            fmt::println("[planner] warning: enableCspFollow failed");
            return std::make_pair(false, rm::Variables{});
        }
        bool ok_traj = resetTRTTraj();
        if (!ok_traj) {
            fmt::println("[planner] warning: resetTRTTrajectory failed");
            return std::make_pair(false, rm::Variables{});
        }

        _task_mode = TaskMode::ThreadRingTighten;

        bool ok = ok_follow && ok_traj;
        _running.store(ok, std::memory_order_relaxed);
        fmt::println("[planner] {}_start -> running={}", cfg::kTRTStartBrowse, ok);
        return std::make_pair(ok, rm::Variables{});
    };
    startTRT_m.browse_name = std::string(cfg::kTRTStartBrowse);
    startTRT_m.display_name = std::string(cfg::kTRTStartDisp);
    startTRT_m.description = std::string(cfg::kTRTStartDesc);
    startTRT_m.iargs = {};
    _srv.addMethodNode(startTRT_m);
}

// 设置 CSP 跟随模式
bool ScaraPlanner::enableCspFollow() {
    auto [res, out] = _exec.callx("set_follow", cfg::kFollowCsp);
    fmt::println("[planner] call set_follow({}) res={}", cfg::kFollowCsp, res);
    return res;
}

// 笛卡尔转关节空间
// bool ScaraPlanner::cartToq(double x, double y, double z, double r_deg, std::array<double, 4> &q_out) {

//     Coordination coord(4);
//     coord = {x, y, z, r_deg};
//     Pulse p = robomotion.inverse_kinematics_unitToPulse(coord);
//     if (p.empty())
//         return false;
//     auto qv = robomotion.motorinfo.pulsesToUnit(p);
//     if (qv.size() < 4)
//         return false;
//     q_out = {qv[0], qv[1], qv[2], qv[3]};
//     return true;
// }

bool ScaraPlanner::cartToq(double x, double y, double z, double r_deg, std::array<double, 4> &q_out) {
    double L1 = robomotion.getL1();
    double L2 = robomotion.getL2();
    double LZ = robomotion.getLZ();

    // 1. 求 θ2 (RY轴)
    double cos_q2 = (x * x + y * y - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);

    // 可达性检查：cos_q2 必须在 [-1, 1] 范围内
    if (cos_q2 < -1.0 || cos_q2 > 1.0)
        return false;

    double sin_q2 = sqrt(1.0 - cos_q2 * cos_q2);
    q_out[1] = atan2(sin_q2, cos_q2);

    // 2. 求 θ1 (RX轴)
    double k1 = L1 + L2 * cos_q2;
    double k2 = L2 * sin_q2;
    q_out[0] = atan2(k1 * x - k2 * y,
                     k1 * y + k2 * x);

    // 3. 求 ZU轴 (弧度) 和 ZP轴 (mm)
    q_out[2] = r_deg / 180.0 * PI;
    q_out[3] = z - LZ * q_out[2] / (2.0 * PI);

    // 4. 用现有函数做限位检查
    std::vector<double> unit_vec(q_out.begin(), q_out.end());
    auto repulse = robomotion.motorinfo.unitTorePulse(unit_vec);
    auto pulse = robomotion.motorinfo.motionmapToPulse(repulse);
    auto limit = robomotion.motorinfo.compareLimit(pulse);
    bool withinLimit = std::all_of(limit.begin(), limit.end(),
                                   [](int8_t v) { return v == 0; });

    return withinLimit;
}

void ScaraPlanner::publishQCmd(const std::array<double, 4> &q) {
    rm::msg::JointState js;
    js.name = {"joint_1", "joint_2", "joint_3", "joint_4"};
    js.position = {q[0], q[1], q[2], q[3]};
    js.velocity = {0.0, 0.0, 0.0, 0.0};
    js.effort = {0.0, 0.0, 0.0, 0.0};
    _tar_pub->publish(js);
}