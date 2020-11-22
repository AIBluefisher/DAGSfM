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

#ifndef _SRC_MAPREDUCE_MASTER_H_
#define _SRC_MAPREDUCE_MASTER_H_

#include <glog/logging.h>

#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

#include "map_reduce/map_reduce_config.h"
#include "map_reduce/running_info.h"
#include "util/misc.h"
#include "util/threading.h"
#include "util/timer.h"

namespace DAGSfM {

class Master {
 public:
  Master() {}

  virtual bool RunSequential() = 0;
  virtual bool RunDistributed() = 0;

  inline void SetMapReduceConfig(const MapReduceConfig& map_reduce_config) {
    map_reduce_config_ = map_reduce_config;
  }

  inline const MapReduceConfig& GetMapReduceConfig() {
    return map_reduce_config_;
  }

  inline virtual void ShowProgress(
      const std::string& task_status = "mapping") = 0;

  inline virtual void ShowResult() const = 0;

  virtual int GetIdleWorker() const = 0;

  virtual bool AllServerCompleted() const = 0;

  inline void StopWorker(const size_t id) {
    rpc::client c(map_reduce_config_.server_ips[id],
                  map_reduce_config_.server_ports[id]);
    c.async_call("Stop");
  }

  inline void CallSaveImages(const int id, const std::string& master_image_path,
                             const std::string& worker_image_path,
                             const std::vector<std::string>& image_names) {
    rpc::client c(map_reduce_config_.server_ips[id],
                  map_reduce_config_.server_ports[id]);
    for (auto image_name : image_names) {
      std::fstream is(colmap::JoinPaths(master_image_path, image_name).c_str(),
                      std::ifstream::in | std::ifstream::binary);
      // Compute buffer length.
      is.seekg(0, is.end);
      int length = is.tellg();
      is.seekg(0, is.beg);

      // Create buffers.
      std::vector<char> buffer(length);

      // Read image to buffer.
      is.read(buffer.data(), length);

      // Remote procedure call to save image.
      auto save_complete_obj =
          c.call("SaveImage", worker_image_path, image_name, buffer, length);
      bool save_complete = save_complete_obj.get().as<bool>();
      if (save_complete) {
        // do nothing.
      }

      is.close();
    }
  }

 protected:
  MapReduceConfig map_reduce_config_;
};

class SfMMaster : public Master {
 public:
  SfMMaster();

  virtual bool RunSequential() override { return true; }

  virtual bool RunDistributed() override {
    // Currently just an implementation for playing and testing.
    CHECK_GT(map_reduce_config_.server_ips.size(), 0);

    int idle_server_id = GetIdleWorker();
    while (idle_server_id != -1) {
      // ShowProgress();
      LOG(INFO) << "Idle server id: " << idle_server_id;
      rpc::client c(map_reduce_config_.server_ips[idle_server_id],
                    map_reduce_config_.server_ports[idle_server_id]);
      c.call("SetNonIdle");
      timers_[idle_server_id].Start();

      // Distributing your tasks here.
      // ......
      // For example: c.async_call("RunSfM");
      std::this_thread::sleep_for(std::chrono::seconds(1));

      CallGetRunningInfo(idle_server_id, false);
      // if (worker_infos_[idle_server_id].completed) {
      //     StopWorker(idle_server_id);
      // }

      idle_server_id = GetIdleWorker();
    }

    while (!AllServerCompleted()) {
      ShowProgress();
      UpdateRunningInfo(false);
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return true;
  }

  virtual int GetIdleWorker() const override;

  virtual bool AllServerCompleted() const override;

  virtual void ShowProgress(
      const std::string& task_status = "mapping") override;

  virtual void ShowResult() const override;

  void CallRunSfM(const size_t id, const bool async);

  void CallGetRunningInfo(const size_t id, const bool async);

  void UpdateRunningInfo(const bool async);

  const SfMRunningInfo GetRunningInfo(const size_t worker_id);
  SfMRunningInfo& GetRunningInfoMutable(const size_t worker_id);
  bool ResetRunningInfo(const size_t worker_id);
  size_t WorkerInfosSize();

  void StartWorkerTimer(const size_t worker_id);
  void PauseWorkerTimer(const size_t worker_id);
  double WorkerElapsedTimeSeconds(const size_t worker_id);

 protected:
  std::unordered_map<size_t, SfMRunningInfo> worker_infos_;
  std::mutex worker_infos_mutex_;

  std::unordered_map<size_t, colmap::Timer> timers_;
  std::mutex timers_mutex_;

 private:
  const SfMRunningInfo GetRunningInfo(const size_t id, const bool async) const;
};

}  // namespace DAGSfM

#endif