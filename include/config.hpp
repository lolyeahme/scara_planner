#pragma once
#include <chrono>
#include <string_view>

using namespace std::chrono_literals;

namespace cfg {
constexpr std::string_view kExecEndpoint = "opc.tcp://127.0.0.1:4840";
constexpr int kCtrlPort = 4841;

// LPSS 目标关节话题
constexpr std::string_view kTarJointsTopic = "/scara/tar/joints";

// 发布周期10ms
constexpr auto kPubPeriod = 4ms;

// OPC UA pump 周期
constexpr auto kOpcuaPumpPeriod = 5ms;

constexpr auto kWorkDwell = 2s; // 工艺点停留时间

// set_follow 模式值
constexpr uint8_t kFollowDebug = 0; // 0: 调试/不跟随
constexpr uint8_t kFollowCsp = 1;   // 1: CSP
constexpr uint8_t kFollowCsv = 2;   // 2: CSV
constexpr uint8_t kFollowCst = 3;   // 3: CST

// OPC UA body load方法节点的 BrowseName、DisplayName 和 Description
constexpr std::string_view kLuStartBrowse = "body_load";
constexpr std::string_view kLuStartDisp = "BodyLoad";
constexpr std::string_view kLuStartDesc = "Start the body load/unload operation";

// OPC UA thread ring tighten方法节点的 BrowseName、DisplayName 和 Description
constexpr std::string_view kTRTStartBrowse = "thread_ring_tighten";
constexpr std::string_view kTRTStartDisp = "ThreadRingTighten";
constexpr std::string_view kTRTStartDesc = "Start the thread ring tighten operation";

// 运行状态变量的 BrowseName、DisplayName 和 Description
constexpr std::string_view kRunningVarBrowse = "running";
constexpr std::string_view kRunningVarDisp = "Running";
constexpr std::string_view kRunningVarDesc = "Whether planner is currently executing a task";

// 笛卡儿上下料工艺参数
struct endPose {
    double x{};
    double y{};
    double z{};
    double r_deg{};
};

// 上料台位置
// 下料台位置
// 工作台位置
// 安全位置高度
constexpr endPose kHome{0.0, 500.0, 0.0, 0.0};      // home位置，单位与IK一致：x/y/z=mm，r=deg
constexpr endPose kPickL{-103.0, 308.0, 0.0, 0.0};  // 上料台位置，单位与IK一致：x/y/z=mm，r=deg
constexpr endPose kPickU{-113.0, 400.0, 0.0, 0.0};  // 下料台位置，单位与IK一致：x/y/z=mm，r=deg
constexpr endPose kPlace{178.0, 438.0, -80.0, 0.0}; // 工作台位置，单位与IK一致：x/y/z=mm，r=deg
constexpr double kSafeZ = 5.0;                      // 安全位置的Z高度，单位与IK一致：mm

constexpr endPose kTighten{178.0, 438.0, -80.0, 0.0}; // thread ring tighten位置，单位与IK一致：x/y/z=mm，r=deg

} // namespace cfg
