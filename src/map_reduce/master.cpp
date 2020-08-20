#include "map_reduce/master.h"

namespace GraphSfM {

SfMMaster::SfMMaster() : Master() {}

int SfMMaster::GetIdleWorker() const {
  const size_t kClusterNum = map_reduce_config_.server_ips.size();
  int idle_worker_id = -1;

  for (size_t k = 0; k < kClusterNum; k++) {
    const SfMRunningInfo info = GetRunningInfo(k, false);

    if (info.idle) {
      idle_worker_id = k;
      break;
    }
  }

  return idle_worker_id;
}

bool SfMMaster::AllServerCompleted() const {
  const size_t kClusterNum = map_reduce_config_.server_ips.size();

  for (size_t k = 0; k < kClusterNum; k++) {
    if (!worker_infos_.at(k).completed) {
      return false;
    }
  }
  return true;
}

void SfMMaster::ShowProgress(const std::string& task_status) {
  std::system("clear");

  std::cout << std::setw(12) << std::setfill(' ') << "Cluster Id"
            << std::setw(14) << std::setfill(' ') << "IP" << std::setw(22)
            << std::setfill(' ') << "Worker Status" << std::setw(14)
            << std::setfill(' ') << "Progress" << std::setw(14)
            << std::setfill(' ') << "Task Status" << std::setw(10)
            << std::setfill(' ') << "Time\n";

  const size_t kClusterNum = map_reduce_config_.server_ips.size();
  const size_t worker_infos_size = WorkerInfosSize();

  if (worker_infos_size < kClusterNum) {
    return;
  }

  for (size_t i = 0; i < kClusterNum; i++) {
    const std::string ip = map_reduce_config_.server_ips[i];
    const uint16_t port = map_reduce_config_.server_ports[i];
    const SfMRunningInfo worker_info = GetRunningInfo(i);

    const size_t sec = (size_t)worker_info.running_time;
    const size_t h = sec / 3600;
    const size_t m = sec % 3600 / 60;
    const int sub_num = task_status == "matching"
                            ? worker_info.matched_image_pairs
                            : worker_info.registered_image_num;
    const int total_num = task_status == "matching"
                              ? worker_info.total_matching_pairs
                              : worker_info.total_image_num;

    std::cout << std::setw(7) << std::setfill(' ') << i << std::setw(20)
              << std::setfill(' ') << ip << ":" << port << std::setw(13)
              << std::setfill(' ') << (worker_info.idle ? "IDLE " : "NONIDLE")
              << std::setprecision(2) << std::setw(11) << std::setfill(' ')
              << sub_num << "/" << total_num << " %" << std::setw(4)
              << std::setfill(' ') << " " << task_status << std::setw(6)
              << std::setfill(' ') << " " << std::setw(2) << std::setfill('0')
              << h << ":" << std::setw(2) << std::setfill('0') << m << ":"
              << std::setw(2) << std::setfill('0') << sec % 3600 % 60 << "\n";
  }
}

void SfMMaster::ShowResult() const {}

void SfMMaster::CallRunSfM(const size_t id, const bool async) {
  rpc::client c(map_reduce_config_.server_ips[id],
                map_reduce_config_.server_ports[id]);
  if (async) {
    c.async_call("RunSfM");
  } else {
    c.call("RunSfM");
  }
}

void SfMMaster::CallGetRunningInfo(const size_t id, const bool async) {
  const SfMRunningInfo info = GetRunningInfo(id, async);

  // LOG(INFO) << info.is_reachable;
  if (!info.is_reachable) {
    LOG(WARNING) << map_reduce_config_.server_ips[id]
                 << " is unreachable, skipped this server.";
    return;
  }

  worker_infos_[id] = info;
  worker_infos_[id].running_time = timers_[id].ElapsedSeconds();
}

void SfMMaster::UpdateRunningInfo(const bool async) {
  const size_t kClusterNum = map_reduce_config_.server_ips.size();

  for (size_t i = 0; i < kClusterNum; i++) {
    CallGetRunningInfo(i, async);
  }
}

const SfMRunningInfo SfMMaster::GetRunningInfo(const size_t id,
                                               const bool async) const {
  rpc::client c(map_reduce_config_.server_ips[id],
                map_reduce_config_.server_ports[id]);

  SfMRunningInfo running_info;
  rpc::client::connection_state cs = c.get_connection_state();

  if (cs == rpc::client::connection_state::disconnected) {
    LOG(WARNING) << "disconnected";
    return running_info;
  }

  auto running_info_obj = c.call("GetRunningInfo");
  running_info = running_info_obj.get().as<SfMRunningInfo>();
  running_info.is_reachable = true;

  return running_info;
}

const SfMRunningInfo SfMMaster::GetRunningInfo(const size_t worker_id) {
  std::lock_guard<std::mutex> guard(worker_infos_mutex_);
  if (worker_infos_.count(worker_id) > 0) {
    return worker_infos_.at(worker_id);
  } else {
    return SfMRunningInfo();
  }
}

SfMRunningInfo& SfMMaster::GetRunningInfoMutable(const size_t worker_id) {
  std::lock_guard<std::mutex> guard(worker_infos_mutex_);
  return worker_infos_[worker_id];
}

bool SfMMaster::ResetRunningInfo(const size_t worker_id) {
  std::lock_guard<std::mutex> guard(worker_infos_mutex_);
  worker_infos_[worker_id].Reset();

  return true;
}

size_t SfMMaster::WorkerInfosSize() {
  size_t size = 0;
  std::lock_guard<std::mutex> guard(worker_infos_mutex_);
  size = worker_infos_.size();

  return size;
}

void SfMMaster::StartWorkerTimer(const size_t worker_id) {
  std::lock_guard<std::mutex> guard(timers_mutex_);
  timers_[worker_id].Start();
}

void SfMMaster::PauseWorkerTimer(const size_t worker_id) {
  std::lock_guard<std::mutex> guard(timers_mutex_);
  timers_[worker_id].Pause();
}

double SfMMaster::WorkerElapsedTimeSeconds(const size_t worker_id) {
  std::lock_guard<std::mutex> guard(timers_mutex_);
  if (timers_.count(worker_id) > 0) {
    return timers_.at(worker_id).ElapsedSeconds();
  }
  // LOG(WARNING) << "not found worker " << worker_id;
  return 0.0;
}

}  // namespace GraphSfM