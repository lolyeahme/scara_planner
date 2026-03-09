#include <fmt/format.h>
#include <rmvl/opcua/client.hpp>
#include <string>

int main(int argc, char **argv) {
    // 用法：
    //   ./scara_lu_remote body_load
    //   ./scara_lu_remote thread_ring_tighten
    // 可选：第三个参数指定 endpoint，例如：
    //   ./scara_lu_remote body_load opc.tcp://127.0.0.1:4841

    std::string endpoint = "opc.tcp://127.0.0.1:4841";
    if (argc < 2) {
        fmt::println("Usage: {} body_load|thread_ring_tighten [endpoint]", argv[0]);
        return 1;
    }
    if (argc >= 3)
        endpoint = argv[2];

    std::string cmd = argv[1];
    rm::OpcuaClient cli(endpoint);

    if (!cli.ok()) {
        fmt::println("[remote] connect failed: {}", endpoint);
        return 2;
    }

    if (cmd == "body_load") {
        auto [ok, out] = cli.callx("body_load", rm::Variables{});
        fmt::println("[remote] call body_load -> {}", ok ? "OK" : "FAIL");
        return ok ? 0 : 3;
    }

    if (cmd == "thread_ring_tighten") {
        auto [ok, out] = cli.callx("thread_ring_tighten", rm::Variables{});
        fmt::println("[remote] call thread_ring_tighten -> {}", ok ? "OK" : "FAIL");
        return ok ? 0 : 3;
    }

    fmt::println("Unknown cmd: {}", cmd);
    fmt::println("Usage: {} body_load|thread_ring_tighten [endpoint]", argv[0]);
    return 1;
}