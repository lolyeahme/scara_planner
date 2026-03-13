/*按照绝对时间调用body_load和thread_ring_tighten两个opcua方法*/
#include <chrono>
#include <string>
#include <thread>

#include <fmt/format.h>
#include <rmvl/opcua/client.hpp>

using namespace std::chrono_literals;
using clock_type = std::chrono::steady_clock;

// 任务周期还需要再调整，不同的任务截至时间需要不同
constexpr auto cycle_period = 200s;         // 每200s为一个周期，周期内调用一次body_load和thread_ring_tighten
constexpr auto t_body_load = 0s;            // body_load在每个周期的第0s调用
constexpr auto t_thread_ring_tighten = 70s; // thread_ring_tighten在每个周期的第60s调用
constexpr auto task_ddl = 30s;              // 每个任务的截止时间为30s
constexpr auto poll_period = 100ms;         // 调度器轮询planner的周期

static bool readRunningVar(const rm::OpcuaClient &planner,
                           const auto &node,
                           bool &running_out) {
    rm::Variable running_var = planner.read(node);
    if (running_var.empty()) {
        fmt::println("[scheduler] warning: failed to read running variable");
        return false;
    }
    running_out = running_var.cast<bool>();
    return true;
}

static bool waitDone(rm::OpcuaClient &planner,
                     clock_type::time_point deadline,
                     clock_type::time_point t0,
                     const auto &node,
                     const char *task_name) {
    while (clock_type::now() < deadline) {
        bool running = false;
        bool ok = readRunningVar(planner, node, running);

        if (ok && !running) {
            auto finish_s = (clock_type::now() - t0) / 1s;
            fmt::println("[scheduler] {} finish time: {}s", task_name, finish_s);
            fmt::println("[scheduler] {} finished within {}s -> YES", task_name, task_ddl.count());
            return true;
        }

        std::this_thread::sleep_for(poll_period);
    }

    auto timeout_s = (clock_type::now() - t0) / 1s;
    fmt::println("[scheduler] {} finish time: >{}s", task_name, timeout_s);
    fmt::println("[scheduler] {} finished within {}s -> NO", task_name, task_ddl.count());
    return false;
}

int main() {
    // 连接planner的opcua服务器
    rm::OpcuaClient planner("opc.tcp://127.0.0.1:4841");

    if (!planner.ok()) {
        fmt::println("[scheduler] connect failed: opc.tcp://127.0.0.1:4841");
        return 1;
    }
    auto node = planner.find("running");
    if (node.empty()) {
        fmt::println("[scheduler] cannot find running variable node");
        return 1;
    }
    ///////////////////////////////////////定义节拍参数////////////////////////////////////////

    auto t0 = clock_type::now(); // 记录程序开始的时间点
    std::size_t cycle_count = 0; // 周期计数器

    fmt::println("[scheduler] started. Cycle period: {}s", cycle_period.count());

    ////////////////////////////////////////节拍循环////////////////////////////////////////
    while (true) {
        auto cycle_t0 = t0 + cycle_count * cycle_period; // 当前周期的起始时间点
        // 0s:调用body_load
        std::this_thread::sleep_until(cycle_t0 + t_body_load);
        {
            // 打印当前时刻
            fmt::println("[scheduler] cycle {}: body_load start time: {}s",
                         cycle_count,
                         (clock_type::now() - t0) / 1s);
            auto [ok, out] = planner.callx("body_load");
            fmt::println("[scheduler] cycle {}: call body_load -> {}", cycle_count, ok ? "OK" : "FAIL");
            if (ok) {
                waitDone(planner, cycle_t0 + t_body_load + task_ddl, t0, node, "body_load");
            }
        }
        // 40s:调用thread_ring_tighten
        std::this_thread::sleep_until(cycle_t0 + t_thread_ring_tighten);
        {
            // 打印当前时刻
            fmt::println("[scheduler] cycle {}: thread_ring_tighten start time: {}s",
                         cycle_count,
                         (clock_type::now() - t0) / 1s);
            auto [ok, out] = planner.callx("thread_ring_tighten");
            fmt::println("[scheduler] cycle {}: call thread_ring_tighten -> {}", cycle_count, ok ? "OK" : "FAIL");
            if (ok) {
                waitDone(planner, cycle_t0 + t_thread_ring_tighten + task_ddl, t0, node, "thread_ring_tighten");
            }
        }

        std::this_thread::sleep_until(cycle_t0 + cycle_period); // 等待直到下一个周期开始
        // 打印第一轮完成的时间点
        fmt::println("[scheduler] cycle {} completed. Current time: {}s",
                     cycle_count,
                     (clock_type::now() - t0) / 1s);
        cycle_count++;
    }

    return 0;
}