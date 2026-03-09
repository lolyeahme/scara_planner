#include "config.hpp"
#include "robo_base.hpp"
#include "scara_planner.hpp"

#include <fmt/format.h>

void ScaraPlanner::startTRTSegment(size_t seg_idx) {
    for (int i = 0; i < 4; ++i) {
        auto &tp = robomotion.trajectoryplanners[i];
        StatePoint s0{_trt_path[seg_idx][i], 0.0, 0.0};
        StatePoint sf{_trt_path[seg_idx + 1][i], 0.0, 0.0};

        tp.setStartState(s0);
        tp.setTarget(sf);
        _tp_idx[i] = 0;
        _tp_N[i] = tp.getFrequency();
    }
}

bool ScaraPlanner::resetTRTTraj() {
    std::array<double, 4> q_place{}, q_tighten{};

    if (!cartToq(cfg::kPlace.x, cfg::kPlace.y, cfg::kPlace.z, cfg::kPlace.r_deg, q_place)) {
        fmt::println("[planner] IK fail: place");
        return false;
    }

    if (!cartToq(cfg::kTighten.x, cfg::kTighten.y, cfg::kTighten.z, cfg::kTighten.r_deg, q_tighten)) {
        fmt::println("[planner] IK fail: thread ring tighten");
        return false;
    }

    _trt_path = {
        q_place,
        q_tighten,
        q_place};

    _seg_idx = 0;
    startTRTSegment(_seg_idx);
    return true;
}

/*  在每个周期计算当前轨迹的下一帧关节值，也就是说不停计算当前段的轨迹点
    当前段计算完成后，段号加1，然后开启下一段：startSegment   */
std::array<double, 4> ScaraPlanner::sampleTRTNextQ() {

    if (_seg_idx + 1 >= _trt_path.size()) {
        _running.store(false, std::memory_order_relaxed);
        _task_mode = TaskMode::Idle;
        return _trt_path.back(); // 保持在最后一个路径点
    }

    std::array<double, 4> q{};
    for (int i = 0; i < 4; ++i) {
        auto &tp = robomotion.trajectoryplanners[i];
        q[i] = tp.sample(_tp_idx[i]).pos;
        _tp_idx[i]++;
    }
    bool done = true;
    for (int i = 0; i < 4; ++i) {
        done &= (_tp_idx[i] >= _tp_N[i]);
    }

    if (done) {
        _seg_idx++;
        if (_seg_idx + 1 < _trt_path.size()) {
            startTRTSegment(_seg_idx);
        } else {
            _running.store(false, std::memory_order_relaxed);
            _task_mode = TaskMode::Idle;
        }
    }
    return q;
}
