// BSD 3-Clause License

// Copyright (c) 2020, Chenyu
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// 1. Redistributions of source code must retain the above copyright notice,
// this
//    list of conditions and the following disclaimer.

// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.

// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "map_reduce/master.h"

namespace DAGSfM {

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

}  // namespace DAGSfM