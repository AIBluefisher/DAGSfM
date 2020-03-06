#ifndef _SRC_MAPREDUCE_RUNNING_INFO_H_
#define _SRC_MAPREDUCE_RUNNING_INFO_H_

#include <string>

#include <rpc/msgpack.hpp>

namespace GraphSfM {

namespace {
    inline std::string ConnectStatus(const rpc::client::connection_state cs)
    {
        switch (cs)
        {
        case rpc::client::connection_state::connected:
            return "CONNECTED";
            break;
        case rpc::client::connection_state::disconnected:
            return "DISCONNECTED";
            break;
        case rpc::client::connection_state::initial:
            return "INITIAL";
            break;
        case rpc::client::connection_state::reset:
            return "RESET";
            break;
        }
    }
}

struct RunningInfo
{
    RunningInfo() {};

    RunningInfo(const RunningInfo& other)
    {
        is_reachable = other.is_reachable;
        idle = other.idle;
        in_progress = other.in_progress;
        completed = other.completed;
        ip = other.ip;
        running_time = other.running_time;
    }

    // // target class should be either copyable or movable (or both).
    // RunningInfo(RunningInfo const&) = default;
    // RunningInfo(RunningInfo&&) = default; 
    bool is_reachable = false;

    bool idle = true;

    bool in_progress = false;

    bool completed = false;

    std::string ip = "";

    double running_time = 0.0;

    // MSGPACK_DEFINE(idle, in_progress, completed, ip, running_time);
};

struct SfMRunningInfo : RunningInfo
{
    SfMRunningInfo() {};

    SfMRunningInfo(const SfMRunningInfo& other)
    {
        idle = other.idle;
        in_progress = other.in_progress;
        completed = other.completed;
        ip = other.ip;
        running_time = other.running_time;
        total_image_num = other.total_image_num;
        registered_image_num = other.registered_image_num;
    }

    // // target class should be either copyable or movable (or both).
    // SfMRunningInfo(SfMRunningInfo const&) = default;
    // SfMRunningInfo(SfMRunningInfo&&) = default;

    size_t total_image_num = 0;

    size_t registered_image_num = 0;

    void Reset()
    {
        is_reachable = false;
        idle = true;
        in_progress = false;
        completed = false;
        running_time = 0.0;
        total_image_num = 0;
        registered_image_num = 0;
    }

    MSGPACK_DEFINE(idle, 
                   in_progress, 
                   completed, 
                   ip, 
                   running_time, 
                   total_image_num,
                   registered_image_num);
};

} // namespace GraphSfM

#endif