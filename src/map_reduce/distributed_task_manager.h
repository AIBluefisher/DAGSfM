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

#ifndef _SRC_MAPREDUCE_DISTRIBUTED_TASK_MANAGER_H_
#define _SRC_MAPREDUCE_DISTRIBUTED_TASK_MANAGER_H_

#include <rpc/client.h>

#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "base/database_cache.h"
#include "base/database_info.h"
#include "base/reconstruction.h"
#include "clustering/image_clustering.h"
#include "map_reduce/master.h"
#include "util/types.h"

namespace DAGSfM {

enum class DistributedDataType { SFM, MATCHING };

struct TaskDataContainer {
  std::vector<std::string> server_ips;
  std::vector<uint16_t> server_ports;
  size_t cluster_num = 0;

  std::string master_image_path;
  std::unordered_map<size_t, std::vector<std::string>> cluster_images;

  std::string task_type = "";

  virtual void DistributeTask(const size_t job_id, const int worker_id){};
  virtual void ReduceTask(const int worker_id){};
  virtual void MergeData(){};
};

struct MatchesDataContainer : public TaskDataContainer {
  std::vector<DatabaseInfo> database_infos;
  std::unordered_map<size_t, std::vector<ImageNamePair>> cluster_matching_pairs;

  std::unordered_map<std::string, image_t> image_name_to_id;

  virtual void DistributeTask(const size_t job_id,
                              const int worker_id) override {
    rpc::client c(server_ips[worker_id], server_ports[worker_id]);

    const std::vector<std::string>& image_names = cluster_images[job_id];
    const std::vector<ImageNamePair>& image_pairs =
        cluster_matching_pairs[job_id];

    c.async_call("RunMatching", image_names, image_pairs, job_id);
    c.call("SetNonIdle");
  }

  virtual void ReduceTask(const int worker_id) override {
    rpc::client c(server_ips[worker_id], server_ports[worker_id]);

    auto local_database_info_obj = c.call("GetLocalDatabaseInfo");

    DatabaseInfo local_database_info =
        local_database_info_obj.get().as<DatabaseInfo>();
    database_infos.emplace_back(local_database_info);

    c.async_call("ResetWorkerInfo");
  }

  virtual void MergeData() override {
    CHECK_GT(database_infos.size(), 0);
    database_infos[0].UpdateImageIndex(image_name_to_id);

    for (uint i = 1; i < database_infos.size(); i++) {
      database_infos[i].UpdateImageIndex(image_name_to_id);
      database_infos[0].Merge(database_infos[i]);
    }
  }
};

struct SfMDataContainer : public TaskDataContainer {
  std::vector<std::unique_ptr<Reconstruction>> reconstructions;
  std::unique_ptr<std::unordered_map<size_t, DatabaseCache>> cluster_database_caches;

  SfMDataContainer() : cluster_database_caches(new std::unordered_map<size_t, DatabaseCache>()){}
  virtual void DistributeTask(const size_t job_id,
                              const int worker_id) override {
    rpc::client c(server_ips[worker_id], server_ports[worker_id]);
    c.async_call("RunSfM", (*cluster_database_caches)[job_id]);
    c.call("SetNonIdle");
  }

  virtual void ReduceTask(const int worker_id) override {
    rpc::client c(server_ips[worker_id], server_ports[worker_id]);

    auto local_recons_obj = c.call("GetLocalRecons");
    reconstructions.emplace_back(new Reconstruction(local_recons_obj.get().as<Reconstruction>()));

    c.async_call("ResetWorkerInfo");
  }

  virtual void MergeData() override {
    // implement outside.
  }
};

template <typename DataType>
class DistributedTaskManager : public SfMMaster {
 public:
  DistributedTaskManager();

  inline const DataType& DataContainer() const;
  inline DataType& DataContainer();

  inline size_t ClusterNum();

  inline size_t WaittingJobNum();
  inline void Push(const size_t job_id);
  inline size_t Pop();

  inline int GetWorkingWorkerId(const size_t job_id);
  inline bool RemoveWorkingJob(const size_t job_id);
  inline void InsertWorkingJob(const size_t job_id, const int worker_id);
  std::vector<size_t> GetWorkingJobIds();

  inline size_t GetFinishedJobNum();
  inline void IncrementFinishedJobNum();

  virtual bool RunSequential();
  virtual bool RunDistributed();

  void SetDataContainer(const DataType& data_container);

 private:
  void ReduceTasks();
  void DistributeTasks();
  void MonitorTasks();

 private:
  DataType data_container_;
  std::mutex data_mutex_;

  // TODO: (chenyu) Consider to use a priority_queue.
  std::queue<size_t> job_queue_;
  std::mutex job_queue_mutex_;

  // { (job_id, worker_id) }.
  std::unordered_map<size_t, int> working_jobs_;
  std::mutex working_jobs_mutex_;

  size_t finished_job_num_;
  std::mutex finished_jobs_mutex_;
};

template <typename DataType>
inline const DataType& DistributedTaskManager<DataType>::DataContainer() const {
  return data_container_;
}

template <typename DataType>
inline DataType& DistributedTaskManager<DataType>::DataContainer() {
  return data_container_;
}

template <typename DataType>
inline size_t DistributedTaskManager<DataType>::ClusterNum() {
  // size_t cluster_num = 0;
  // data_mutex_.lock();
  // cluster_num = data_container_.cluster_num;
  // data_mutex_.unlock();
  // return cluster_num;
  return data_container_.cluster_num;
}

template <typename DataType>
inline size_t DistributedTaskManager<DataType>::WaittingJobNum() {
  std::lock_guard<std::mutex> guard(job_queue_mutex_);
  return job_queue_.size();
}

template <typename DataType>
inline void DistributedTaskManager<DataType>::Push(const size_t job_id) {
  std::lock_guard<std::mutex> guard(job_queue_mutex_);
  job_queue_.push(job_id);
}

template <typename DataType>
inline size_t DistributedTaskManager<DataType>::Pop() {
  size_t job_id;
  std::lock_guard<std::mutex> guard(job_queue_mutex_);
  job_id = job_queue_.front();
  job_queue_.pop();

  return job_id;
};

template <typename DataType>
inline int DistributedTaskManager<DataType>::GetWorkingWorkerId(
    const size_t job_id) {
  int worker_id = -1;

  std::lock_guard<std::mutex> guard(working_jobs_mutex_);
  if (working_jobs_.count(job_id) > 0) {
    worker_id = working_jobs_.at(job_id);
  }

  return worker_id;
}

template <typename DataType>
inline bool DistributedTaskManager<DataType>::RemoveWorkingJob(
    const size_t job_id) {
  std::lock_guard<std::mutex> guard(working_jobs_mutex_);
  if (working_jobs_.count(job_id) == 0) {
    return false;
  }

  working_jobs_.erase(job_id);

  return true;
}

template <typename DataType>
inline void DistributedTaskManager<DataType>::InsertWorkingJob(
    const size_t job_id, const int worker_id) {
  std::lock_guard<std::mutex> guard(working_jobs_mutex_);
  working_jobs_.emplace(job_id, worker_id);
}

template <typename DataType>
inline std::vector<size_t>
DistributedTaskManager<DataType>::GetWorkingJobIds() {
  std::vector<size_t> working_job_ids;

  working_jobs_mutex_.lock();
  for (const auto& working_job : working_jobs_) {
    working_job_ids.push_back(working_job.first);
  }
  working_jobs_mutex_.unlock();

  return working_job_ids;
};

template <typename DataType>
inline size_t DistributedTaskManager<DataType>::GetFinishedJobNum() {
  size_t finished_job_num = 0;

  finished_jobs_mutex_.lock();
  finished_job_num = finished_job_num_;
  finished_jobs_mutex_.unlock();

  return finished_job_num;
};

template <typename DataType>
inline void DistributedTaskManager<DataType>::IncrementFinishedJobNum() {
  finished_jobs_mutex_.lock();
  finished_job_num_++;
  finished_jobs_mutex_.unlock();
};

}  // namespace DAGSfM

#include "map_reduce/distributed_task_manager.inl"

#endif