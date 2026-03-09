#pragma once
#include <chrono>
#include <string_view>

using namespace std::chrono_literals;

namespace cfg {
constexpr std::string_view kExecEndpoint = "opc.tcp://127.0.0.1:4840";
inline constexpr int kCtrlPort = 4841;

// LPSS 目标关节话题
inline constexpr std::string_view kTarJointsTopic = "/scara/tar/joints";

// 发布周期10ms
inline constexpr auto kPubPeriod = 4ms;

// OPC UA pump 周期
inline constexpr auto kOpcuaPumpPeriod = 10ms;

// set_follow 模式值
inline constexpr uint8_t kFollowDebug = 0; // 0: 调试/不跟随
inline constexpr uint8_t kFollowCsp = 1;   // 1: CSP
inline constexpr uint8_t kFollowCsv = 2;   // 2: CSV
inline constexpr uint8_t kFollowCst = 3;   // 3: CST

// OPC UA body load方法节点的 BrowseName、DisplayName 和 Description
inline constexpr std::string_view kLuStartBrowse = "body_load";
inline constexpr std::string_view kLuStartDisp = "BodyLoad";
inline constexpr std::string_view kLuStartDesc = "Start the body load/unload operation";

// OPC UA thread ring tighten方法节点的 BrowseName、DisplayName 和 Description
inline constexpr std::string_view kTRTStartBrowse = "thread_ring_tighten";
inline constexpr std::string_view kTRTStartDisp = "ThreadRingTighten";
inline constexpr std::string_view kTRTStartDesc = "Start the thread ring tighten operation";

// 笛卡儿上下料工艺参数
struct endPose {
    double x{};
    double y{};
    double z{};
    double r_deg{};
};

inline constexpr endPose kPick{100.0, 560.0, 5.0, 30.0};   // pick位置，单位与IK一致：x/y/z=mm，r=deg
inline constexpr endPose kPlace{140.0, 560.0, 6.0, -20.0}; // place位置，单位与IK一致：x/y/z=mm，r=deg
inline constexpr double kSafeZ = 20.0;

inline constexpr endPose kTighten{120.0, 560.0, 5.0, -20.0}; // thread ring tighten位置，单位与IK一致：x/y/z=mm，r=deg

} // namespace cfg
