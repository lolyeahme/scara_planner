#include "config.hpp"
#include "scara_planner.hpp"

#include <chrono>
#include <thread>

using namespace std::chrono_literals;

int main() {
    rm::OpcuaClient exec(std::string(cfg::kExecEndpoint));
    exec.callx("start");
    std::this_thread::sleep_for(1s);
    exec.callx("home");

    ScaraPlanner node("scara_planner", exec);
    node.spin();

    exec.callx("stop");
    return 0;
}