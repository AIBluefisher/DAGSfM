#include "map_reduce/master.h"

namespace GraphSfM {

SfMMaster::SfMMaster() : Master() {}

int SfMMaster::GetIdleWorker() const
{
    const size_t kClusterNum = map_reduce_config_.server_ips.size();
    int idle_server_id = -1;

    for (size_t k = 0; k < kClusterNum; k++) {
        const SfMRunningInfo info = GetRunningInfo(k, false);

        if (info.idle) {
        // if (worker_infos_.at(k).idle) {
            idle_server_id = k;
            break;
        }
    }

    return idle_server_id;
}

bool SfMMaster::AllServerCompleted() const
{
    const size_t kClusterNum = map_reduce_config_.server_ips.size();

    for (size_t k = 0; k < kClusterNum; k++) {
        if (!worker_infos_.at(k).completed) {
            return false;
        }
    }
    return true;
}

void SfMMaster::ShowProgress()
{
    std::system("clear");

    std::cout << std::setw(12) << std::setfill(' ') << "Cluster Id"
              << std::setw(14) << std::setfill(' ') << "IP"
              << std::setw(18) << std::setfill(' ') << "Status"
              << std::setw(14) << std::setfill(' ') << "Progress"
              << std::setw(10) << std::setfill(' ') << "Time\n";

    const size_t kClusterNum = map_reduce_config_.server_ips.size();
    if (worker_infos_.size() < kClusterNum) { return; }

    for (size_t i = 0; i < kClusterNum; i++) {
        const std::string ip = map_reduce_config_.server_ips[i];
        const uint16_t port = map_reduce_config_.server_ports[i];
        const size_t sec = (size_t)worker_infos_.at(i).running_time;
        const size_t h = sec / 3600;
        const size_t m = sec % 3600 / 60;

        std::cout << std::setw(7) << std::setfill(' ') << i 
                  << std::setw(20) << std::setfill(' ') << ip << ":" << port
                  << std::setw(12) << std::setfill(' ') << 
                     (worker_infos_.at(i).idle ? "IDLE " : "NONIDLE")
                  << std::setprecision(2) 
                  << std::setw(8) << std::setfill(' ')
                  << worker_infos_.at(i).registered_image_num << "/"
                  << worker_infos_.at(i).total_image_num << " %"
                  << std::setw(4) << std::setfill(' ') << " "
                  << std::setw(2) << std::setfill('0') << h << ":"
                  << std::setw(2) << std::setfill('0') << m << ":" 
                  << std::setw(2) << std::setfill('0') << sec % 3600 % 60 << "\n";
    }
}

void SfMMaster::ShowResult() const
{

}

void SfMMaster::CallRunSfM(const size_t id, const bool async)
{
    rpc::client c(map_reduce_config_.server_ips[id],
                  map_reduce_config_.server_ports[id]);
    if (async) {
        c.async_call("RunSfM");
    } else {
        c.call("RunSfM");
    }
}

void SfMMaster::CallGetRunningInfo(const size_t id, const bool async)
{
    const SfMRunningInfo info = GetRunningInfo(id, async);
    LOG(INFO) << info.is_reachable;
    if (!info.is_reachable) {
        LOG(WARNING) << map_reduce_config_.server_ips[id] 
                     << " is unreacheable, skipped this server.";
        return;
    }
    worker_infos_[id] = info;
    worker_infos_[id].running_time = timers_[id].ElapsedSeconds();
}

void SfMMaster::UpdateRunningInfo(const bool async)
{
    const size_t kClusterNum = map_reduce_config_.server_ips.size();

    for (size_t i = 0; i < kClusterNum; i++) {
        CallGetRunningInfo(i, async);
    }
}

const SfMRunningInfo SfMMaster::GetRunningInfo(const size_t id,
                                               const bool async) const
{
    rpc::client c(map_reduce_config_.server_ips[id],
                  map_reduce_config_.server_ports[id]);

    SfMRunningInfo running_info;
    rpc::client::connection_state cs = c.get_connection_state();

    if (cs == rpc::client::connection_state::disconnected) {
        return running_info;
    }

    if (async) {
        auto running_info_obj = c.async_call("GetRunningInfo");
        running_info = running_info_obj.get().as<SfMRunningInfo>();
        running_info.is_reachable = true;
    } else {
        auto running_info_obj = c.call("GetRunningInfo");
        running_info = running_info_obj.get().as<SfMRunningInfo>();
        running_info.is_reachable = true;
    }
    return running_info;
}

} // namespace GraphSfM