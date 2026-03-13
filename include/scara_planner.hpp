#pragma once
#include <array>
#include <atomic>
#include <stop_token>
#include <string_view>
#include <thread>

#include <rmvl/lpss/node.hpp>
#include <rmvl/opcua/client.hpp>
#include <rmvl/opcua/server.hpp>
#include <rmvlmsg/sensor/joint_state.hpp>

#include "robo_base.hpp"

enum class TaskMode {
    Idle = 0,
    BodyLoad,
    ThreadRingTighten
};

class ScaraPlanner : public rm::lpss::async::Node {
public:
    // 构造函数，初始化ScaraPlanner节点,加上explicit必须显示调用构造函数
    explicit ScaraPlanner(std::string_view name, rm::OpcuaClient &exec);

private:
    //////////////////////////////两个方法的公共函数(core)//////////////////////////////

    // 注册opcua method
    void installOpcuaMethods();
    // 注册opcua data
    void installOpcuaData();

    // 设置set follow模式
    bool enableCspFollow();
    // 发布setpoint数据
    void publishQCmd(const std::array<double, 4> &q);
    // 逆运动学
    bool cartToq(double x, double y, double z, double r_deg, std::array<double, 4> &q_out);

    //////////////////////////////body_load相关函数//////////////////////////////

    void startBLSegment(size_t seg_idx);   // 开始执行body_load当前段轨迹
    bool resetBLTraj();                    // 重置body_load轨迹到初始状态，准备开始新一轮执行
    std::array<double, 4> sampleBLNextQ(); // body_load采样当前段轨迹的下一个点，更新内部状态以准备下一次采样

    //////////////////////////////thread_ring_tighten相关函数//////////////////////////////

    void startTRTSegment(size_t seg_idx);   // 开始执行thread_ring_tighten当前段轨迹
    bool resetTRTTraj();                    // 重置thread_ring_tighten轨迹到初始状态，准备开始新一轮执行
    std::array<double, 4> sampleTRTNextQ(); // thread_ring_tighten采样当前段轨迹的下一个点，更新内部状态以准备下一次采样

private:
    rm::OpcuaServer _srv;   //!< OPC UA服务器对象
    rm::OpcuaClient &_exec; //!< 执行节点的OPC UA客户端引用包装

    // std::atomic_bool _running{false}; //!< 运行状态
    bool _running{false}; //!< 运行状态,把_running改成普通布尔变量

    rm::lpss::async::Publisher<rm::msg::JointState>::ptr _tar_pub{}; //!< 关节状态发布
    rm::lpss::async::Timer::ptr _pub_timer{};                        //!< 发布定时器
    rm::lpss::async::Timer::ptr _opcua_timer{};
    // std::jthread _opcua_thread; //!< OPC UA服务器线程,发布频率较高时需要单独线程处理OPC UA请求

    std::array<double, 4> _q_cmd{0.0, 0.0, 0.0, 0.0}; //!< 当前要发布的关节角度

    bool _dwelling{false};                              //!< 是否处于工艺点停留阶段
    int _dwell_ticks_left{0};                           //!< 工艺点停留剩余周期数
    std::array<double, 4> _dwell_q{0.0, 0.0, 0.0, 0.0}; //!< 工艺点停留阶段的目标关节角度

    std::array<int, 4> _tp_N{};   //!< 每段中每个轨迹的总采样点数
    std::array<int, 4> _tp_idx{}; //!< 每段中每个关节当前采样到第几个点

    // home->place_safe->place->pick_safe->pick->pick_safe->place_safe->place
    // 路径点需要动态调整，用vector更合适
    std::vector<std::array<double, 4>> _bl_path{}; //!< 一共有13个waypoint，每个waypoint对应4个关节角度的目标值
    // place->tighten->place
    std::vector<std::array<double, 4>> _trt_path{}; //!< 一共有5个waypoint，每个waypoint对应4个关节角度的目标值
    size_t _seg_idx{0};                             //!< 当前执行到第几段轨迹
    TaskMode _task_mode{TaskMode::Idle};            //!< 当前任务模式
};