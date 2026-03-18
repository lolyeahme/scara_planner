#include "config.hpp"
#include "scara_planner.hpp"

#include <chrono>
#include <fmt/format.h>
#include <thread>

using namespace std::chrono_literals;

int main() {
    rm::OpcuaClient exec(std::string(cfg::kExecEndpoint));
    auto [res_start, out_start] = exec.callx("start");
    fmt::println("[planner-main] Start result: {}", res_start);
    std::this_thread::sleep_for(2s);
    auto [res_home, out_home] = exec.callx("home");
    fmt::println("[planner-main] Home result: {}", res_home);
    std::this_thread::sleep_for(2s);

    ScaraPlanner node("scara_planner", exec);
    node.spin();

    auto [res_stop, out_stop] = exec.callx("stop");
    fmt::println("[planner-main] Stop result: {}", res_stop);
    return 0;
}