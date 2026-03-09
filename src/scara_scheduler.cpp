/*按照绝对时间调用body_load和thread_ring_tighten两个opcua方法*/
#include <chrono>
#include <string>
#include <thread>

#include <fmt/format.h>
#include <rmvl/opcua/client.hpp>

using namespace std::chrono_literals;
using clock_type = std::chrono::steady_clock;

int main() {
    // 连接planner的opcua服务器
    rm::OpcuaClient planner("opc.tcp://127.0.0.1:4841");

    if (!planner.ok()) {
        fmt::println("[scheduler] connect failed: opc.tcp://127.0.0.1:4841");
        return 1;
    }

    ///////////////////////////////////////定义节拍参数////////////////////////////////////////
    const auto cycle_period = 120s;         // 每120s为一个周期，周期内调用一次body_load和thread_ring_tighten
    const auto t_body_load = 0s;            // body_load在每个周期的第0s调用
    const auto t_thread_ring_tighten = 40s; // thread_ring_tighten在每个周期的第40s调用

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
                         std::chrono::duration_cast<std::chrono::seconds>(clock_type::now() - t0).count());
            auto [ok, out] = planner.callx("body_load", rm::Variables{});
            fmt::println("[scheduler] cycle {}: call body_load -> {}", cycle_count, ok ? "OK" : "FAIL");
        }
        // 40s:调用thread_ring_tighten
        std::this_thread::sleep_until(cycle_t0 + t_thread_ring_tighten);
        {
            // 打印当前时刻
            fmt::println("[scheduler] cycle {}: thread_ring_tighten start time: {}s",
                         cycle_count,
                         std::chrono::duration_cast<std::chrono::seconds>(clock_type::now() - t0).count());
            auto [ok, out] = planner.callx("thread_ring_tighten", rm::Variables{});
            fmt::println("[scheduler] cycle {}: call thread_ring_tighten -> {}", cycle_count, ok ? "OK" : "FAIL");
        }

        std::this_thread::sleep_until(cycle_t0 + cycle_period); // 等待直到下一个周期开始
        // 打印第一轮完成的时间点
        fmt::println("[scheduler] cycle {} completed. Current time: {}s",
                     cycle_count,
                     std::chrono::duration_cast<std::chrono::seconds>(clock_type::now() - t0).count());
        cycle_count++;
    }

    return 0;
}