#include "config.hpp"
#include "robo_base.hpp"
#include "scara_planner.hpp"

#include <fmt/format.h>

/*  将各个笛卡尔关键点转换为关节点，利用关键点拼凑出完整轨迹_path，
    初始化第一段路径，类似于在地图上绘制总路线*/
bool ScaraPlanner::resetBLTraj() {

    std::array<double, 4> q_pick{}, q_place{}, q_pick_safe{}, q_place_safe{};

    if (!cartToq(cfg::kPick.x, cfg::kPick.y, cfg::kPick.z, cfg::kPick.r_deg, q_pick)) {
        fmt::println("[planner] IK fail: pick");
        return false;
    }
    if (!cartToq(cfg::kPick.x, cfg::kPick.y, cfg::kSafeZ, cfg::kPick.r_deg, q_pick_safe)) {
        fmt::println("[planner] IK fail: pick_safe");
        return false;
    }
    if (!cartToq(cfg::kPlace.x, cfg::kPlace.y, cfg::kPlace.z, cfg::kPlace.r_deg, q_place)) {
        fmt::println("[planner] IK fail: place");
        return false;
    }
    if (!cartToq(cfg::kPlace.x, cfg::kPlace.y, cfg::kSafeZ, cfg::kPlace.r_deg, q_place_safe)) {
        fmt::println("[planner] IK fail: place_safe");
        return false;
    }

    _lu_path = {
        q_place,
        q_place_safe,
        q_pick_safe,
        q_pick,
        q_pick_safe,
        q_place_safe,
        q_place};

    _seg_idx = 0;
    startBLSegment(_seg_idx);
    return true;
}

/*  负责初始化第seg_idx段的轨迹，用当前段的起点终点，初始化轨迹规划器*/
void ScaraPlanner::startBLSegment(size_t seg_idx) {
    for (int i = 0; i < 4; ++i) {
        auto &tp = robomotion.trajectoryplanners[i];
        // 设置当前段轨迹的起点状态和终点状态，期望速度和加速度都为0
        StatePoint s0{_lu_path[seg_idx][i], 0.0, 0.0};
        StatePoint sf{_lu_path[seg_idx + 1][i], 0.0, 0.0};

        tp.setStartState(s0);
        tp.setTarget(sf);
        // 把采样计数器清零，以保证新段轨迹从头开始采样，同时记录该段轨迹的总采样点数
        _tp_idx[i] = 0;
        // 获取当前轨迹总的采样点数，用于后续判断当前轨迹段是否已经执行完毕
        // linearmove不是这样做的，而是直接根据反馈来确定
        _tp_N[i] = tp.getFrequency();
    }
}

/*  在每个周期计算当前轨迹的下一帧关节值，也就是说不停计算当前段的轨迹点
    当前段计算完成后，段号加1，然后开启下一段：startSegment   */
std::array<double, 4> ScaraPlanner::sampleBLNextQ() {

    if (_seg_idx + 1 >= _lu_path.size()) {
        _running.store(false, std::memory_order_relaxed);
        _task_mode = TaskMode::Idle;
        return _lu_path.back(); // 保持在最后一个路径点
    } // 检查是否已经完成所有轨迹段的执行

    std::array<double, 4> q{};
    // 对当前轨迹段的每个关节，采样下一个点的关节角度，并更新采样索引
    for (int i = 0; i < 4; ++i) {
        auto &tp = robomotion.trajectoryplanners[i];
        q[i] = tp.sample(_tp_idx[i]).pos;
        _tp_idx[i]++;
    }
    // 判断结束的条件是每个关节的采样索引都已经达到或超过该段轨迹的总采样点数，
    bool done = true;
    for (int i = 0; i < 4; ++i) {
        done &= (_tp_idx[i] >= _tp_N[i]);
    }

    // 如果是，则认为当前段轨迹执行完成，进入下一段轨迹
    if (done) {
        _seg_idx++;
        if (_seg_idx + 1 < _lu_path.size()) {
            startBLSegment(_seg_idx);
        } else {
            // 所有段都执行完成，停止运行
            _running.store(false, std::memory_order_relaxed);
            _task_mode = TaskMode::Idle;
        }
    }
    return q;
}